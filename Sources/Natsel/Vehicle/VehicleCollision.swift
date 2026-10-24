internal import Dispatch

// Wheel collision: each step every wheel of an awake vehicle is cast along its suspension. The casts
// only read the world, so with workers they run in parallel across vehicles.

extension World {
    /// Cast all wheels and wake whatever the vehicles stand on. Runs after the narrow phase and
    /// before the solver, while waking solver sets is still allowed.
    func updateVehicleCollisions() {
        guard vehicleIdPool.idCount > 0 else { return }

        // Validate, wake, and steer serially; these change world state.
        var active: [Int] = []
        for i in vehicles.indices where vehicles[i].id != B3.nullIndex {
            vehicles[i].active = false

            let chassisId = vehicles[i].bodyId
            guard isValidBody(chassisId) else {
                // The chassis was destroyed; the vehicle goes with it.
                let generation = vehicles[i].generation
                vehicles[i] = VehicleRecord.empty
                vehicles[i].generation = generation
                vehicleIdPool.free(Int32(i))
                continue
            }

            let chassisIndex = bodyIndex(chassisId)
            if bodies[Int(chassisIndex)].setIndex != SetType.awakeSet.rawValue {
                guard shouldWakeSleepingVehicle(i) else { continue }
                wakeBody(chassisIndex)
                if bodies[Int(chassisIndex)].setIndex != SetType.awakeSet.rawValue { continue }
            }

            vehicles[i].active = true
            applyVehicleSteering(i)
            active.append(i)
        }

        // Cast the wheels.
        let workers = Int(maxInt(1, minInt(workerCount, 8)))
        vehicles.withUnsafeMutableBufferPointer { buffer in
            nonisolated(unsafe) let records = buffer.baseAddress!
            nonisolated(unsafe) let world = self
            @Sendable func castAll(_ i: Int) {
                let record = records + i
                let chassisIndex = world.bodyIndex(record.pointee.bodyId)
                let transform = world.bodyTransformQuick(chassisIndex)
                for w in 0..<record.pointee.wheels.count {
                    world.castVehicleWheel(record, w, transform, chassisIndex)
                }
            }
            if workers > 1 && active.count >= 8 {
                active.withUnsafeBufferPointer { list in
                    nonisolated(unsafe) let indices = list
                    DispatchQueue.concurrentPerform(iterations: indices.count) { k in castAll(indices[k]) }
                }
            } else {
                for i in active { castAll(i) }
            }
        }

        for i in active {
            let chassisIndex = bodyIndex(vehicles[i].bodyId)

            // Bodies under the wheels must be awake to receive the tire and suspension impulses.
            for w in vehicles[i].wheels.indices where vehicles[i].wheels[w].hasContact {
                let ground = vehicles[i].wheels[w].contactBody
                if bodies[Int(ground)].type == .dynamicBody
                    && bodies[Int(ground)].setIndex >= SetType.firstSleepingSet.rawValue
                {
                    wakeBody(ground)
                }
            }

            // Throttle keeps the vehicle awake even when it is stuck.
            if vehicles[i].forwardInput != 0 {
                bodies[Int(chassisIndex)].sleepTime = 0
            }
        }
    }

    /// A sleeping vehicle wakes when its ground moves or disappears.
    private func shouldWakeSleepingVehicle(_ index: Int) -> Bool {
        for wheel in vehicles[index].wheels where wheel.hasContact {
            guard isValidBody(wheel.contactBodyId) else { return true }
            let ground = bodies[Int(bodyIndex(wheel.contactBodyId))]
            if ground.setIndex == SetType.awakeSet.rawValue && ground.type != .staticBody {
                return true
            }
        }
        return false
    }

    /// Steering from driver input. Motorcycles limit steering by the lean they can sustain.
    private func applyVehicleSteering(_ index: Int) {
        let speed = vehicleForwardSpeed(VehicleID(index1: vehicles[index].id + 1, world0: worldId, generation: vehicles[index].generation))
        var record = VehicleRecord.empty
        swap(&record, &vehicles[index])
        defer { swap(&record, &vehicles[index]) }

        if record.drivetrain.kind == .tracked {
            for w in record.wheels.indices { record.wheels[w].steerAngle = 0 }
            return
        }

        var limit: Float = .greatestFiniteMagnitude
        if record.drivetrain.kind == .motorcycle && record.drivetrain.lean.settings.enableLeanSteeringLimit {
            let g = length(gravity)
            let wheelBase = record.drivetrain.lean.wheelBase
            if speed * speed > 1.0e-4 && wheelBase > 0 && g > 0 {
                let cs = computeCosSin(record.drivetrain.lean.settings.maxLeanAngle)
                let tanLean = cs.cosine > 0 ? cs.sine / cs.cosine : .greatestFiniteMagnitude
                limit = atan2Approx(wheelBase * g * tanLean, speed * speed)
            }
        }

        for w in record.wheels.indices {
            let maxSteer = minFloat(record.wheels[w].settings.maxSteerAngle, limit)
            // Positive right input turns right: a negative rotation about the steering (up) axis.
            record.wheels[w].steerAngle = -record.rightInput * maxSteer
        }
    }

    private func castVehicleWheel(
        _ recordPointer: UnsafeMutablePointer<VehicleRecord>, _ w: Int, _ chassis: WorldTransform, _ chassisIndex: Int32
    ) {
        // Read fields, not the whole record: a copy would retain the wheel array and make the write
        // back below copy it.
        let tester = recordPointer.pointee.tester
        let filter = recordPointer.pointee.filter
        let cosMaxSlope = recordPointer.pointee.cosMaxSlopeAngle
        var wheel = recordPointer.pointee.wheels[w]
        let s = wheel.settings

        let origin = transformWorldPoint(chassis, s.position)
        let direction = rotateVector(chassis.q, s.suspensionDirection)
        let steer = makeQuatFromAxisAngle(s.steeringAxis, wheel.steerAngle)
        let forward = rotateVector(chassis.q, rotateVector(steer, s.wheelForward))
        let accept: (Vec3) -> Bool = { normal in dot(normal, -direction) >= cosMaxSlope }

        var hit = QueryHit()
        var length: Float = 0
        switch tester {
        case .ray:
            let castLength = s.suspensionMaxLength + s.radius
            hit = closestRayHit(
                origin, castLength * direction, filter, ignoreBody: chassisIndex, accept: accept)
            length = hit.fraction * castLength - s.radius

        case .sphere:
            var point = Vec3()
            hit = withUnsafePointer(to: &point) { pointer in
                let proxy = ShapeProxy(points: pointer, count: 1, radius: s.radius)
                return closestShapeCastHit(
                    origin, proxy, s.suspensionMaxLength * direction, filter,
                    ignoreBody: chassisIndex, accept: accept)
            }
            length = hit.fraction * s.suspensionMaxLength

        case .cylinder(let fraction):
            hit = withUnsafeTemporaryAllocation(of: Vec3.self, capacity: 2 * cylinderSegments) { buffer in
                let radius = makeWheelCylinderPoints(s, chassis.q, steer, convexRadiusFraction: fraction, buffer)
                let proxy = ShapeProxy(points: UnsafePointer(buffer.baseAddress), count: Int32(buffer.count), radius: radius)
                return closestShapeCastHit(
                    origin, proxy, s.suspensionMaxLength * direction, filter,
                    ignoreBody: chassisIndex, accept: accept)
            }
            length = hit.fraction * s.suspensionMaxLength

            // A flat cylinder touches along a line; GJK reports one end of it. Move the contact to
            // the wheel's mid plane so the tire force acts in the middle of the tread.
            if hit.hit {
                let axle = rotateVector(chassis.q, rotateVector(steer, cross(s.wheelUp, s.wheelForward)))
                let center = mulAdd(origin, length, direction)
                hit.point = mulSub(hit.point, dot(subPos(hit.point, center), axle), axle)
            }
        }

        let lateral = hit.hit ? cross(hit.normal, forward) : Vec3()
        if hit.hit && lengthSquared(lateral) > 1.0e-6 && length <= s.suspensionMaxLength {
            let shape = shapes[Int(hit.shapeIndex)]
            let material = shape.material(hit.materialIndex)
            wheel.hasContact = true
            wheel.contactBody = shape.bodyId
            wheel.contactBodyId = makeBodyID(shape.bodyId)
            wheel.contactShapeId = ShapeID(index1: shape.id + 1, world0: worldId, generation: shape.generation)
            wheel.contactPosition = hit.point
            wheel.contactNormal = hit.normal
            wheel.lateral = normalize(lateral)
            wheel.longitudinal = cross(wheel.lateral, hit.normal)
            wheel.suspensionLength = length
            wheel.surfaceFriction = material.friction
            wheel.userMaterialId = material.userMaterialId
        } else {
            wheel.hasContact = false
            wheel.contactBody = B3.nullIndex
            wheel.suspensionLength = s.suspensionMaxLength
            wheel.longitudinal = forward
            wheel.lateral = Vec3()
            wheel.contactNormal = -direction
            wheel.contactPosition = mulAdd(origin, s.suspensionMaxLength + s.radius, direction)
        }

        recordPointer.pointee.wheels[w] = wheel
    }

    /// A rounded cylinder around the axle, relative to the suspension attachment. Fills `points`
    /// with `2 * cylinderSegments` vertices and returns the rounding radius.
    private func makeWheelCylinderPoints(
        _ s: WheelParameters, _ chassis: Quat, _ steer: Quat, convexRadiusFraction: Float,
        _ points: UnsafeMutableBufferPointer<Vec3>
    ) -> Float {
        let halfWidth = 0.5 * s.width
        let convexRadius = clampFloat(convexRadiusFraction, 0, 1) * minFloat(s.radius, halfWidth)
        let rim = s.radius - convexRadius
        let side = maxFloat(halfWidth - convexRadius, 0)

        let rotation = mulQuat(chassis, steer)
        let up = rotateVector(rotation, s.wheelUp)
        let forward = rotateVector(rotation, s.wheelForward)
        let axle = normalize(cross(up, forward))

        for k in 0..<cylinderSegments {
            let cs = computeCosSin(B3.twoPi * Float(k) / Float(cylinderSegments))
            let radial = cs.cosine * up + cs.sine * forward
            let center = rim * radial
            points[2 * k] = mulAdd(center, side, axle)
            points[2 * k + 1] = mulAdd(center, -side, axle)
        }
        return convexRadius
    }
}

/// Rim points per side of the cylinder wheel proxy.
private let cylinderSegments = 12
