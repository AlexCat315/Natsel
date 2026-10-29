// Body-level queries. These run against a caller supplied body transform so they can be
// used for speculative movement without stepping the simulation.

extension World {
    /// Cast a ray against the shapes of a single body.
    public func bodyCastRay(
        _ bodyId: BodyID,
        _ origin: Pos,
        _ translation: Vec3,
        _ filter: QueryFilter = .defaultFilter,
        _ maxFraction: Float = 1,
        _ bodyTransform: WorldTransform
    ) -> BodyCastResult {
        var result = BodyCastResult()

        let body = self.body(bodyId)
        guard body.id != B3.nullIndex else {
            return result
        }

        // The consistent framing is to center on the ray origin.
        var shapeInput = RayCastInput()
        shapeInput.origin = Vec3()
        shapeInput.translation = translation
        shapeInput.maxFraction = maxFraction

        let transform = toRelativeTransform(bodyTransform, origin)

        var shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            shapeId = shape.nextShapeId

            if shouldQueryCollide(shape.filter, filter) == false {
                continue
            }

            let shapeOutput = rayCastShape(shape, transform, shapeInput)

            if shapeOutput.hit == false {
                continue
            }

            if shapeOutput.fraction > shapeInput.maxFraction {
                continue
            }

            // Careful with id, shapeId is the next shape.
            let id = ShapeID(
                index1: shape.id + 1, world0: bodyId.world0, generation: shape.generation
            )
            let materialIndex = clampInt(shapeOutput.materialIndex, 0, shape.materialCount - 1)
            let userMaterialId = shape.material(materialIndex).userMaterialId

            result.shapeId = id
            result.point = offsetPos(origin, shapeOutput.point)
            result.normal = shapeOutput.normal
            result.fraction = shapeOutput.fraction
            result.triangleIndex = shapeOutput.triangleIndex
            result.userMaterialId = userMaterialId
            result.iterations = shapeOutput.iterations
            result.hit = true

            shapeInput.maxFraction = shapeOutput.fraction
        }

        return result
    }

    /// Cast a shape against the shapes of a single body.
    public func bodyCastShape(
        _ bodyId: BodyID,
        _ origin: Pos,
        _ proxy: ShapeProxy,
        _ translation: Vec3,
        _ filter: QueryFilter = .defaultFilter,
        _ maxFraction: Float = 1,
        _ canEncroach: Bool = false,
        _ bodyTransform: WorldTransform
    ) -> BodyCastResult {
        var result = BodyCastResult()

        let body = self.body(bodyId)
        guard body.id != B3.nullIndex else {
            return result
        }

        let transform = toRelativeTransform(bodyTransform, origin)

        var shapeInput = ShapeCastInput()
        shapeInput.proxy = proxy
        shapeInput.translation = translation
        shapeInput.maxFraction = maxFraction
        shapeInput.canEncroach = canEncroach

        var shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            shapeId = shape.nextShapeId

            if shouldQueryCollide(shape.filter, filter) == false {
                continue
            }

            let shapeOutput = shapeCastShape(shape, transform, shapeInput)

            if shapeOutput.hit == false {
                continue
            }

            if shapeOutput.fraction > shapeInput.maxFraction {
                continue
            }

            // Careful with id, shapeId is the next shape.
            let id = ShapeID(
                index1: shape.id + 1, world0: bodyId.world0, generation: shape.generation
            )
            let materialIndex = clampInt(shapeOutput.materialIndex, 0, shape.materialCount - 1)
            let userMaterialId = shape.material(materialIndex).userMaterialId

            result.shapeId = id
            result.point = offsetPos(origin, shapeOutput.point)
            result.normal = shapeOutput.normal
            result.fraction = shapeOutput.fraction
            result.triangleIndex = shapeOutput.triangleIndex
            result.userMaterialId = userMaterialId
            result.iterations = shapeOutput.iterations
            result.hit = true

            shapeInput.maxFraction = shapeOutput.fraction
        }

        return result
    }

    /// Test a shape against the shapes of a single body.
    public func bodyOverlapShape(
        _ bodyId: BodyID,
        _ origin: Pos,
        _ proxy: ShapeProxy,
        _ filter: QueryFilter = .defaultFilter,
        _ bodyTransform: WorldTransform
    ) -> Bool {
        let body = self.body(bodyId)
        guard body.id != B3.nullIndex else {
            return false
        }

        let transform = toRelativeTransform(bodyTransform, origin)

        var shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            shapeId = shape.nextShapeId

            if shouldQueryCollide(shape.filter, filter) == false {
                continue
            }

            if Natsel.overlapShape(shape, transform, proxy) {
                return true
            }
        }

        return false
    }

    /// Collide a capsule mover against the convex shapes of a single body.
    public func bodyCollideMover(
        _ bodyId: BodyID,
        _ origin: Pos,
        _ mover: Capsule,
        _ filter: QueryFilter = .defaultFilter,
        _ bodyTransform: WorldTransform,
        capacity: Int
    ) -> [BodyPlaneResult] {
        guard capacity > 0 else {
            return []
        }

        let body = self.body(bodyId)
        guard body.id != B3.nullIndex else {
            return []
        }

        let transform = toRelativeTransform(bodyTransform, origin)

        var results: [BodyPlaneResult] = []
        var shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            shapeId = shape.nextShapeId

            if shouldQueryCollide(shape.filter, filter) == false {
                continue
            }

            switch shape.type {
            case .sphereShape, .capsuleShape, .hullShape:
                break
            default:
                continue
            }

            let planes = Natsel.collideMover(shape, transform, mover, capacity: 1)
            if let plane = planes.first {
                let id = ShapeID(
                    index1: shape.id + 1, world0: bodyId.world0, generation: shape.generation
                )
                var entry = BodyPlaneResult()
                entry.shapeId = id
                entry.result = plane
                results.append(entry)

                if results.count == capacity {
                    return results
                }
            }
        }

        return results
    }

    /// Compute the time of impact of a capsule mover against a single body.
    public func bodyTimeOfImpactMover(
        _ bodyId: BodyID,
        _ origin: Pos,
        _ mover: Capsule,
        _ moverTranslation: Vec3,
        _ filter: QueryFilter = .defaultFilter,
        _ bodyTransform1: WorldTransform,
        _ bodyTransform2: WorldTransform
    ) -> BodyTOIResult {
        var result = BodyTOIResult()
        result.fraction = 1

        let body = self.body(bodyId)
        guard body.id != B3.nullIndex else {
            return result
        }

        let xf1 = toRelativeTransform(bodyTransform1, origin)
        let xf2 = toRelativeTransform(bodyTransform2, origin)

        let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]
        let localCenter = sim.localCenter

        var input = TOIInput()
        let capsulePoints = [mover.center1, mover.center2]
        input.sweepA.c1 = transformPoint(xf1, localCenter)
        input.sweepA.c2 = transformPoint(xf2, localCenter)
        input.sweepA.q1 = bodyTransform1.q
        input.sweepA.q2 = bodyTransform2.q
        input.sweepA.localCenter = localCenter

        input.sweepB.c1 = Vec3()
        input.sweepB.c2 = moverTranslation
        input.sweepB.q1 = Quat.identity
        input.sweepB.q2 = Quat.identity
        input.sweepB.localCenter = Vec3()

        input.maxFraction = 1

        var shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            shapeId = shape.nextShapeId

            if shouldQueryCollide(shape.filter, filter) == false {
                continue
            }

            switch shape.type {
            case .sphereShape, .capsuleShape, .hullShape:
                break
            default:
                continue
            }

            let buffer = makeShapeProxy(shape)
            let output: TOIOutput = capsulePoints.withUnsafeBufferPointer { points in
                buffer.withProxy { proxyA in
                    input.proxyA = proxyA
                    input.proxyB = ShapeProxy(points: points.baseAddress, count: 2, radius: mover.radius)
                    return timeOfImpact(input)
                }
            }

            // Mimic the continuous query: ignore shapes that initially overlap.
            if 0 < output.fraction && output.fraction < result.fraction {
                input.maxFraction = output.fraction

                result.point = offsetPos(origin, output.point)
                result.normal = output.normal
                result.fraction = output.fraction
                result.shapeId = ShapeID(
                    index1: shape.id + 1, world0: worldId, generation: shape.generation
                )
            }
        }

        return result
    }
}

extension World {
    /// Contact manifold digests for divergence debugging.
    public func contactDump() -> [String] {
        var out: [String] = []
        for i in contacts.indices where contacts[i].contactId == Int32(i) {
            let c = contacts[i]
            guard c.payload != nil, c.manifoldCount > 0 else { continue }
            for m in c.manifolds {
                var parts: [Float] = [m.normal.x, m.normal.y, m.normal.z, m.twistImpulse, m.frictionImpulse.x, m.frictionImpulse.y, m.frictionImpulse.z, m.rollingImpulse.x, m.rollingImpulse.y, m.rollingImpulse.z]
                for pi in 0..<Int(m.pointCount) {
                    let p = m.points[pi]
                    parts += [p.anchorA.x, p.anchorA.y, p.anchorA.z, p.anchorB.x, p.anchorB.y, p.anchorB.z,
                              p.separation, p.normalImpulse, p.totalNormalImpulse]
                }
                out.append("C \(c.shapeIdA) \(c.shapeIdB) " + parts.map { String($0.bitPattern, radix: 16) }.joined(separator: " "))
            }
        }
        return out
    }

    /// Per-body position, rotation, and awake velocity bits for divergence debugging.
    public func bodyStateDump() -> [String] {
        var out: [String] = []
        for i in 0..<bodies.count where bodies[i].id == Int32(i) {
            let body = bodies[i]
            let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]
            let positionBits = [sim.transform.p.x, sim.transform.p.y, sim.transform.p.z].map { String($0.bitPattern, radix: 16) }
            var parts = [sim.transform.q.v.x,
                         sim.transform.q.v.y, sim.transform.q.v.z, sim.transform.q.s]
            if body.setIndex == SetType.awakeSet.rawValue {
                let s = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)]
                parts += [s.linearVelocity.x, s.linearVelocity.y, s.linearVelocity.z,
                          s.angularVelocity.x, s.angularVelocity.y, s.angularVelocity.z]
            }
            out.append("\(i) " + positionBits.joined(separator: " ") + " " + parts.map { String($0.bitPattern, radix: 16) }.joined(separator: " "))
        }
        return out
    }

    /// FNV-1a hash over every live body's transform and, when awake, its velocities.
    public func worldStateHash() -> UInt64 {
        let hashInit: UInt64 = 14_695_981_039_346_656_037
        let prime: UInt64 = 1_099_511_628_211

        var hash = hashInit

        @inline(__always)
        func mixBits(_ value: Float, _ hash: inout UInt64) {
            hash = (hash ^ UInt64(value.bitPattern)) &* prime
        }

        @inline(__always)
        func mixPosition(_ p: Pos, _ hash: inout UInt64) {
            hash = (hash ^ UInt64(p.x.bitPattern)) &* prime
            hash = (hash ^ UInt64(p.y.bitPattern)) &* prime
            hash = (hash ^ UInt64(p.z.bitPattern)) &* prime
        }

        for i in 0..<bodies.count {
            let body = bodies[i]

            // Free or never-used slot
            if body.id != Int32(i) {
                continue
            }

            let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]

            mixPosition(sim.transform.p, &hash)
            mixBits(sim.transform.q.v.x, &hash)
            mixBits(sim.transform.q.v.y, &hash)
            mixBits(sim.transform.q.v.z, &hash)
            mixBits(sim.transform.q.s, &hash)

            // Only awake bodies have a state.
            if body.setIndex == SetType.awakeSet.rawValue {
                let state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)]
                mixBits(state.linearVelocity.x, &hash)
                mixBits(state.linearVelocity.y, &hash)
                mixBits(state.linearVelocity.z, &hash)
                mixBits(state.angularVelocity.x, &hash)
                mixBits(state.angularVelocity.y, &hash)
                mixBits(state.angularVelocity.z, &hash)
            }
        }

        return hash
    }
}

// MARK: - Explosion

extension World {
    /// Apply a radial impulse field to every dynamic shape within range. The impulse scales with
    /// each shape's projected area and its explosion scale, and falls off between `radius` and
    /// `radius + falloff`.
    public func explode(_ def: ExplosionDef) {
        let radius = def.radius
        let falloff = def.falloff
        let impulsePerArea = def.impulsePerArea
        let position = def.position

        b3Assert(isValidPosition(position))
        b3Assert(radius >= 0)
        b3Assert(falloff >= 0)

        // The broad-phase tree is float, so translate a local query box out to world with outward rounding
        let extent = radius + falloff
        let localBox = AABB(Vec3(-extent, -extent, -extent), Vec3(extent, extent, extent))
        let aabb = aabbOffset(localBox, position)

        let tree = broadPhase.trees[Int(BodyType.dynamicBody.rawValue)]
        _ = tree.query(aabb, maskBits: def.maskBits, requireAllBits: false) { _, userData in
            let shapeId = Int32(Int64(bitPattern: userData))
            self.applyExplosion(
                shapeId, position, radius, falloff, impulsePerArea
            )
            return true
        }
    }

    private func applyExplosion(
        _ shapeId: Int32, _ position: Pos, _ radius: Float, _ falloff: Float,
        _ impulsePerArea: Float
    ) {
        var shape = shapes[Int(shapeId)]
        if shape.explosionScale == 0 {
            return
        }

        let bodyId = shape.bodyId
        b3Assert(bodies[Int(bodyId)].type == .dynamicBody)

        let xf = bodyTransformQuick(bodyId)

        // Re-center the explosion into the shape local frame so distance and direction stay precise
        // far from the origin. Everything below runs in that near-origin frame.
        let localPosition = invTransformWorldPoint(xf, position)

        let buffer = makeShapeProxy(shape)
        var localPoint = localPosition
        var output = DistanceOutput()
        var cache = SimplexCache()

        output = withUnsafePointer(to: &localPoint) { point in
            buffer.withProxy { proxyA in
                var input = DistanceInput()
                input.proxyA = proxyA
                input.proxyB = ShapeProxy(points: point, count: 1, radius: 0)
                input.transform = Transform.identity
                input.useRadii = true
                return shapeDistance(input, &cache)
            }
        }

        if output.distance > radius + falloff {
            return
        }

        wakeBody(bodyId)

        var body = bodies[Int(bodyId)]
        if body.setIndex != SetType.awakeSet.rawValue {
            return
        }

        // Witness point is already in the body local query frame
        var closestPoint = output.pointA
        if output.distance == 0 {
            closestPoint = getShapeCentroid(shape)
        }

        var direction = closestPoint - localPosition
        if lengthSquared(direction) > 100 * Float.ulpOfOne * Float.ulpOfOne {
            direction = normalize(direction)
        } else {
            direction = Vec3(1, 0, 0)
        }

        let area = getShapeProjectedArea(shape, direction)
        var scale: Float = 1
        if output.distance > radius && falloff > 0 {
            scale = clampFloat((radius + falloff - output.distance) / falloff, 0, 1)
        }

        let magnitude = impulsePerArea * area * scale * shape.explosionScale
        let impulse = magnitude * rotateVector(xf.q, direction)

        let localIndex = body.localIndex
        let awakeSetIndex = Int(SetType.awakeSet.rawValue)
        var state = solverSets[awakeSetIndex].bodyStates[Int(localIndex)]
        let sim = solverSets[awakeSetIndex].bodySims[Int(localIndex)]
        state.linearVelocity = mulAdd(state.linearVelocity, sim.invMass, impulse)

        // Lever arm from the center of mass to the closest point, rotated to world
        let r = rotateVector(xf.q, closestPoint - sim.localCenter)
        state.angularVelocity = state.angularVelocity
            + mulMV(sim.invInertiaWorld, cross(r, impulse))

        solverSets[awakeSetIndex].bodyStates[Int(localIndex)] = state

        // Keep the local shape copy in sync if the wake path touched it.
        shape = shapes[Int(shapeId)]
        shapes[Int(shapeId)] = shape
        body = bodies[Int(bodyId)]
        bodies[Int(bodyId)] = body
    }
}

// MARK: - Body flags

extension World {
    /// Enable or disable contact recycling for a body. When either body of a pair disables it the
    /// pair is rebuilt each step instead of reusing cached separations.
    public func enableBodyContactRecycling(_ bodyId: BodyID, _ enable: Bool) {
        let index = bodyIndex(bodyId)
        var body = bodies[Int(index)]

        let newFlag = enable ? BodyFlags.enableContactRecycling.rawValue : 0
        if (body.flags & BodyFlags.enableContactRecycling.rawValue) == newFlag {
            return
        }

        body.flags &= ~BodyFlags.enableContactRecycling.rawValue
        body.flags |= newFlag
        bodies[Int(index)] = body

        syncBodyFlags(index)
    }

    /// Is contact recycling enabled on this body?
    public func isBodyContactRecyclingEnabled(_ bodyId: BodyID) -> Bool {
        let body = self.body(bodyId)
        return (body.flags & BodyFlags.enableContactRecycling.rawValue) != 0
    }
}

// MARK: - Bullet and motion locks

extension World {
    /// Enable or disable continuous collision for this body. The flag is written to the body as
    /// well as the sim so a later `syncBodyFlags` cannot revert it.
    public func setBodyBullet(_ bodyId: BodyID, _ bullet: Bool) {
        let index = bodyIndex(bodyId)
        var body = bodies[Int(index)]

        let newFlag = bullet ? BodyFlags.isBullet.rawValue : 0
        if (body.flags & BodyFlags.isBullet.rawValue) == newFlag {
            return
        }

        body.flags &= ~BodyFlags.isBullet.rawValue
        body.flags |= newFlag
        bodies[Int(index)] = body

        // Write the sim directly: the bullet flag is not a transient flag and must survive
        // the sync path unchanged.
        var sim = bodySim(body)
        sim.flags &= ~BodyFlags.isBullet.rawValue
        sim.flags |= newFlag
        solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)] = sim
    }

    /// Is continuous collision enabled for this body?
    public func isBodyBullet(_ bodyId: BodyID) -> Bool {
        (body(bodyId).flags & BodyFlags.isBullet.rawValue) != 0
    }

    /// Replace the motion locks on a body. Locks that are reverting from locked to free wake the
    /// body so the newly allowed motion starts immediately.
    public func setBodyMotionLocks(_ bodyId: BodyID, _ locks: MotionLocks) {
        let index = bodyIndex(bodyId)
        var body = bodies[Int(index)]

        let lockMask = BodyFlags.lockLinearX.rawValue | BodyFlags.lockLinearY.rawValue
            | BodyFlags.lockLinearZ.rawValue | BodyFlags.lockAngularX.rawValue
            | BodyFlags.lockAngularY.rawValue | BodyFlags.lockAngularZ.rawValue

        var newFlags = body.flags & ~lockMask
        newFlags |= locks.linearX ? BodyFlags.lockLinearX.rawValue : 0
        newFlags |= locks.linearY ? BodyFlags.lockLinearY.rawValue : 0
        newFlags |= locks.linearZ ? BodyFlags.lockLinearZ.rawValue : 0
        newFlags |= locks.angularX ? BodyFlags.lockAngularX.rawValue : 0
        newFlags |= locks.angularY ? BodyFlags.lockAngularY.rawValue : 0
        newFlags |= locks.angularZ ? BodyFlags.lockAngularZ.rawValue : 0

        if newFlags == body.flags {
            return
        }

        // Unlocking a direction is a state change the user can observe, so wake the body.
        let unlocked = (body.flags & ~newFlags) & lockMask
        body.flags = newFlags
        bodies[Int(index)] = body

        syncBodyFlags(index)

        if unlocked != 0 {
            wakeBody(index)
        }
    }

    /// The current motion locks for a body.
    public func bodyMotionLocks(_ bodyId: BodyID) -> MotionLocks {
        let flags = body(bodyId).flags
        var locks = MotionLocks()
        locks.linearX = (flags & BodyFlags.lockLinearX.rawValue) != 0
        locks.linearY = (flags & BodyFlags.lockLinearY.rawValue) != 0
        locks.linearZ = (flags & BodyFlags.lockLinearZ.rawValue) != 0
        locks.angularX = (flags & BodyFlags.lockAngularX.rawValue) != 0
        locks.angularY = (flags & BodyFlags.lockAngularY.rawValue) != 0
        locks.angularZ = (flags & BodyFlags.lockAngularZ.rawValue) != 0
        return locks
    }
}

// MARK: - Enable and disable

extension World {
    /// Disable a body: destroy its contacts, park its joints and sim in the disabled set, and drop
    /// its broad-phase proxies. A disabled body is skipped by the simulation.
    public func disableBody(_ bodyId: BodyID) {
        let index = bodyIndex(bodyId)
        var body = bodies[Int(index)]
        if body.setIndex == SetType.disabledSet.rawValue {
            return
        }

        // Destroy contacts and wake bodies touching this body. This avoids floating bodies.
        // This is necessary even for static bodies.
        destroyBodyContacts(index, true)

        body = bodies[Int(index)]
        let currentSetIndex = body.setIndex

        let disabledSetIndex = SetType.disabledSet.rawValue

        // Unlink joints and transfer them to the disabled set
        var jointKey = body.headJointKey
        while jointKey != B3.nullIndex {
            let jointId = jointKey >> 1
            let edgeIndex = Int(jointKey & 1)

            let joint = joints[Int(jointId)]
            jointKey = joint.edges[edgeIndex].nextKey

            // The joint may already be disabled by the other body.
            if joint.setIndex == disabledSetIndex {
                continue
            }

            b3Assert(
                joint.setIndex == currentSetIndex
                    || currentSetIndex == SetType.staticSet.rawValue
                    || joint.setIndex == SetType.staticSet.rawValue
            )

            // Remove the joint from its island
            unlinkJoint(jointId)

            let refreshed = joints[Int(jointId)]
            transferJoint(disabledSetIndex, refreshed.setIndex, jointId)
        }

        // Remove shapes from the broad-phase
        body = bodies[Int(index)]
        var shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            var shape = shapes[Int(shapeId)]
            shapeId = shape.nextShapeId
            destroyShapeProxy(&shape, broadPhase)
            shapes[Int(shape.id)] = shape
        }

        // Disabled bodies are not in an island. If the island becomes empty it is destroyed.
        removeBodyFromIsland(index)

        // The contact teardown above may have woken this body or its neighbors, moving the sim.
        // Re-read the set before transferring or the source index is stale.
        let finalSetIndex = bodies[Int(index)].setIndex
        transferBody(disabledSetIndex, finalSetIndex, index)
    }

    /// Re-enable a previously disabled body.
    public func enableBody(_ bodyId: BodyID) {
        let index = bodyIndex(bodyId)
        var body = bodies[Int(index)]
        if body.setIndex != SetType.disabledSet.rawValue {
            return
        }

        let disabledSetIndex = SetType.disabledSet.rawValue
        let setId =
            body.type == .staticBody ? SetType.staticSet.rawValue : SetType.awakeSet.rawValue

        transferBody(setId, disabledSetIndex, index)

        // The sim moved sets, so re-read the body before indexing the new set.
        body = bodies[Int(index)]
        let transform = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)].transform

        // Add shapes to the broad-phase
        var shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            var shape = shapes[Int(shapeId)]
            let nextShapeId = shape.nextShapeId
            createShapeProxy(&shape, body.type, transform, true)
            shapes[Int(shape.id)] = shape
            shapeId = nextShapeId
        }

        if setId != SetType.staticSet.rawValue {
            createIslandForBody(setId, index)
        }

        // Transfer joints. If the other body is still disabled, leave the joint alone.
        body = bodies[Int(index)]
        var jointKey = body.headJointKey
        while jointKey != B3.nullIndex {
            let jointId = jointKey >> 1
            let edgeIndex = Int(jointKey & 1)

            let joint = joints[Int(jointId)]
            b3Assert(joint.setIndex == disabledSetIndex)
            b3Assert(joint.islandId == B3.nullIndex)

            jointKey = joint.edges[edgeIndex].nextKey

            let bodyA = bodies[Int(joint.edges[0].bodyId)]
            let bodyB = bodies[Int(joint.edges[1].bodyId)]

            if bodyA.setIndex == disabledSetIndex || bodyB.setIndex == disabledSetIndex {
                // One body is still disabled.
                continue
            }

            let jointSetId: Int32
            if bodyA.type != .dynamicBody && bodyB.type != .dynamicBody {
                jointSetId = SetType.staticSet.rawValue
            } else if bodyA.setIndex == SetType.staticSet.rawValue {
                jointSetId = bodyB.setIndex
            } else {
                jointSetId = bodyA.setIndex
            }

            transferJoint(jointSetId, disabledSetIndex, jointId)

            // Now that the joint is in the correct set, link it into the island.
            if jointSetId != SetType.staticSet.rawValue {
                linkJoint(jointId)
            }
        }
    }

    /// True while the body is enabled (not parked in the disabled set).
    public func isBodyEnabled(_ bodyId: BodyID) -> Bool {
        body(bodyId).setIndex != SetType.disabledSet.rawValue
    }
}

// MARK: - Body type

extension World {
    /// Change a body's type, moving it between solver sets and rebuilding its contacts, joints,
    /// and broad-phase proxies. Not supported for bodies with compound or height-field shapes.
    /// The body type: static, kinematic, or dynamic.
    public func bodyType(_ bodyId: BodyID) -> BodyType {
        body(bodyId).type
    }

    public func setBodyType(_ bodyId: BodyID, _ type: BodyType) {
        let index = bodyIndex(bodyId)
        var body = bodies[Int(index)]

        let originalType = body.type
        if originalType == type {
            return
        }

        if type != .staticBody {
            var shapeId = body.headShapeId
            while shapeId != B3.nullIndex {
                let shape = shapes[Int(shapeId)]
                if shape.type == .compoundShape || shape.type == .heightShape {
                    // Setting the body type is not supported for bodies with compound shapes.
                    return
                }
                shapeId = shape.nextShapeId
            }
        }

        let disabledSetIndex = SetType.disabledSet.rawValue
        let staticSetIndex = SetType.staticSet.rawValue
        let awakeSetIndex = SetType.awakeSet.rawValue

        // Stage 1: disabled bodies do not change solver sets or islands when they change type.
        if body.setIndex == disabledSetIndex {
            body.type = type
            if type == .dynamicBody {
                body.flags |= BodyFlags.dynamicFlag.rawValue
            } else {
                body.flags &= ~BodyFlags.dynamicFlag.rawValue
            }
            bodies[Int(index)] = body
            syncBodyFlags(index)
            updateBodyMassData(index)
            return
        }

        // Stage 2: destroy all contacts but don't wake bodies (we don't need to).
        destroyBodyContacts(index, false)

        // Stage 3: wake this body (does nothing if static).
        wakeBody(index)

        // Stage 4: move joints to temporary storage in the static set.
        body = bodies[Int(index)]
        var jointKey = body.headJointKey
        while jointKey != B3.nullIndex {
            let jointId = jointKey >> 1
            let edgeIndex = Int(jointKey & 1)

            let joint = joints[Int(jointId)]
            jointKey = joint.edges[edgeIndex].nextKey

            // The joint may be disabled by the other body.
            if joint.setIndex == disabledSetIndex {
                continue
            }

            // Wake attached bodies: the wake above does not wake bodies attached to a static body.
            wakeBody(joint.edges[0].bodyId)
            wakeBody(joint.edges[1].bodyId)

            // Remove the joint from its island
            unlinkJoint(jointId)

            // Transfer every joint to the static set so it can acquire consistent colors.
            let refreshed = joints[Int(jointId)]
            transferJoint(staticSetIndex, refreshed.setIndex, jointId)
        }

        // Stage 5: change the body type and transfer the body.
        body = bodies[Int(index)]
        body.type = type
        if type == .dynamicBody {
            body.flags |= BodyFlags.dynamicFlag.rawValue
        } else {
            body.flags &= ~BodyFlags.dynamicFlag.rawValue
        }
        bodies[Int(index)] = body

        body = bodies[Int(index)]
        let sourceSetIndex = bodies[Int(index)].setIndex
        let targetSetIndex = type == .staticBody ? staticSetIndex : awakeSetIndex
        transferBody(targetSetIndex, sourceSetIndex, index)

        // Stage 6: update island participation.
        if originalType == .staticBody {
            createIslandForBody(awakeSetIndex, index)
        } else if type == .staticBody {
            removeBodyFromIsland(index)
        }

        // Stage 7: transfer joints to the target set.
        body = bodies[Int(index)]
        jointKey = body.headJointKey
        while jointKey != B3.nullIndex {
            let jointId = jointKey >> 1
            let edgeIndex = Int(jointKey & 1)

            let joint = joints[Int(jointId)]
            jointKey = joint.edges[edgeIndex].nextKey

            if joint.setIndex == disabledSetIndex {
                continue
            }

            let bodyA = bodies[Int(joint.edges[0].bodyId)]
            let bodyB = bodies[Int(joint.edges[1].bodyId)]

            if bodyA.type == .dynamicBody || bodyB.type == .dynamicBody {
                transferJoint(awakeSetIndex, staticSetIndex, jointId)
            }
        }

        // Recreate shape proxies in the broad-phase.
        body = bodies[Int(index)]
        let transform = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)].transform
        var shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            var shape = shapes[Int(shapeId)]
            let nextShapeId = shape.nextShapeId
            destroyShapeProxy(&shape, broadPhase)
            createShapeProxy(&shape, type, transform, true)
            shapes[Int(shape.id)] = shape
            shapeId = nextShapeId
        }

        // Relink all joints.
        body = bodies[Int(index)]
        jointKey = body.headJointKey
        while jointKey != B3.nullIndex {
            let jointId = jointKey >> 1
            let edgeIndex = Int(jointKey & 1)

            let joint = joints[Int(jointId)]
            jointKey = joint.edges[edgeIndex].nextKey

            let otherEdgeIndex = edgeIndex ^ 1
            let otherBody = bodies[Int(joint.edges[otherEdgeIndex].bodyId)]

            if otherBody.setIndex == disabledSetIndex {
                continue
            }

            if body.type != .dynamicBody && otherBody.type != .dynamicBody {
                continue
            }

            linkJoint(jointId)
        }

        syncBodyFlags(index)

        // A body type change flips the mass properties.
        updateBodyMassData(index)
    }
}

// MARK: - Awake state and fast flag

extension World {
    /// Awake or sleep a body. Sleeping splits the island first when it has pending removals.
    public func setBodyAwake(_ bodyId: BodyID, _ awake: Bool) {
        let index = bodyIndex(bodyId)
        let body = bodies[Int(index)]

        if awake && body.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeBody(index)
        } else if awake == false && body.setIndex == SetType.awakeSet.rawValue {
            let island = islands[Int(body.islandId)]
            if island.constraintRemoveCount > 0 {
                // Must split the island before sleeping. This is expensive.
                splitIsland(body.islandId)
            }
            trySleepIsland(bodies[Int(index)].islandId)
        }
    }

    /// True while the body is in the awake solver set.
    public func isBodyAwake(_ bodyId: BodyID) -> Bool {
        body(bodyId).setIndex == SetType.awakeSet.rawValue
    }

    /// True while the body sim carries the fast flag. The flag is set during body finalization and
    /// gates contact recycling, so it only lives on the sim.
    public func isBodyFast(_ bodyId: BodyID) -> Bool {
        let body = self.body(bodyId)
        guard body.setIndex != B3.nullIndex else { return false }
        let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]
        return (sim.flags & BodyFlags.isFast.rawValue) != 0
    }
}

// MARK: - Names

extension World {
    /// Set a body's name. The string is interned in the world cache, so repeated names share an id.
    public func setBodyName(_ bodyId: BodyID, _ name: String?) {
        let index = bodyIndex(bodyId)
        var body = bodies[Int(index)]
        body.nameId = names.add(name)
        bodies[Int(index)] = body
    }

    /// The body's name, or nil when it has none.
    public func bodyName(_ bodyId: BodyID) -> String? {
        names.find(body(bodyId).nameId)
    }

    /// Set a shape's name.
    public func setShapeName(_ shapeId: ShapeID, _ name: String?) {
        let index = shapeId.index1 - 1
        var shape = shapes[Int(index)]
        shape.nameId = names.add(name)
        shapes[Int(index)] = shape
    }

    /// The shape's name, or nil when it has none.
    public func shapeName(_ shapeId: ShapeID) -> String? {
        names.find(shapes[Int(shapeId.index1 - 1)].nameId)
    }
}

extension World {
    /// Ray cast a single shape from `origin`. The cast is re-centered on the origin so it stays
    /// precise far from the world origin; the returned point is a world position.
    public func shapeRayCast(_ shapeId: ShapeID, _ origin: Pos, _ translation: Vec3) -> WorldCastOutput {
        let shape = shapes[Int(shapeId.index1 - 1)]
        let transform = toRelativeTransform(bodyTransformQuick(shape.bodyId), origin)

        // The ray starts at the origin, so its origin in the re-centered frame is zero.
        var input = RayCastInput()
        input.origin = Vec3()
        input.translation = translation
        input.maxFraction = 1

        return WorldCastOutput(rayCastShape(shape, transform, input), origin: origin)
    }
}

// MARK: - Forces and impulses

extension World {
    /// Apply a force at a world point. The force is cleared after the next step.
    public func applyBodyForce(_ bodyId: BodyID, _ force: Vec3, _ point: Pos, wake: Bool = true) {
        b3Assert(isValidVec3(force))
        let index = bodyIndex(bodyId)

        if wake { wakeBody(index) }

        guard bodies[Int(index)].setIndex == SetType.awakeSet.rawValue else { return }

        var sim = bodySimFor(bodyId)
        sim.force = sim.force + force
        sim.torque = sim.torque + cross(subPos(point, sim.center), force)
        setBodySim(bodyId, sim)
    }

    /// Apply a force through the center of mass.
    public func applyBodyForceToCenter(_ bodyId: BodyID, _ force: Vec3, wake: Bool = true) {
        b3Assert(isValidVec3(force))
        let index = bodyIndex(bodyId)

        if wake { wakeBody(index) }

        guard bodies[Int(index)].setIndex == SetType.awakeSet.rawValue else { return }

        var sim = bodySimFor(bodyId)
        sim.force = sim.force + force
        setBodySim(bodyId, sim)
    }

    /// Apply a pure torque. The torque is cleared after the next step.
    public func applyBodyTorque(_ bodyId: BodyID, _ torque: Vec3, wake: Bool = true) {
        b3Assert(isValidVec3(torque))
        let index = bodyIndex(bodyId)

        if wake { wakeBody(index) }

        guard bodies[Int(index)].setIndex == SetType.awakeSet.rawValue else { return }

        var sim = bodySimFor(bodyId)
        sim.torque = sim.torque + torque
        setBodySim(bodyId, sim)
    }

    /// Apply an impulse at a world point, changing velocity immediately.
    public func applyBodyLinearImpulse(_ bodyId: BodyID, _ impulse: Vec3, _ point: Pos, wake: Bool = true) {
        b3Assert(isValidVec3(impulse))
        let index = bodyIndex(bodyId)

        if wake { wakeBody(index) }

        let record = bodies[Int(index)]
        guard record.setIndex == SetType.awakeSet.rawValue else { return }

        var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)]
        let sim = bodySimFor(bodyId)

        state.linearVelocity = mulAdd(state.linearVelocity, sim.invMass, impulse)

        // Clamp to the maximum linear speed.
        let maxSpeed = maxLinearSpeed
        if lengthSquared(state.linearVelocity) > maxSpeed * maxSpeed {
            state.linearVelocity = maxSpeed * normalize(state.linearVelocity)
        }

        let delta = mulMV(sim.invInertiaWorld, cross(subPos(point, sim.center), impulse))
        state.angularVelocity = state.angularVelocity + delta
        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)] = state
    }

    /// Apply an impulse through the center of mass.
    public func applyBodyLinearImpulseToCenter(_ bodyId: BodyID, _ impulse: Vec3, wake: Bool = true) {
        b3Assert(isValidVec3(impulse))
        let index = bodyIndex(bodyId)

        if wake { wakeBody(index) }

        let record = bodies[Int(index)]
        guard record.setIndex == SetType.awakeSet.rawValue else { return }

        var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)]
        let sim = bodySimFor(bodyId)

        state.linearVelocity = mulAdd(state.linearVelocity, sim.invMass, impulse)

        let maxSpeed = maxLinearSpeed
        if lengthSquared(state.linearVelocity) > maxSpeed * maxSpeed {
            state.linearVelocity = maxSpeed * normalize(state.linearVelocity)
        }

        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)] = state
    }

    /// Apply an angular impulse, changing angular velocity immediately.
    public func applyBodyAngularImpulse(_ bodyId: BodyID, _ impulse: Vec3, wake: Bool = true) {
        b3Assert(isValidVec3(impulse))
        let index = bodyIndex(bodyId)

        if wake { wakeBody(index) }

        let record = bodies[Int(index)]
        guard record.setIndex == SetType.awakeSet.rawValue else { return }

        var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)]
        let sim = bodySimFor(bodyId)
        state.angularVelocity = state.angularVelocity + mulMV(sim.invInertiaWorld, impulse)
        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)] = state
    }

    // MARK: Body queries

    /// Number of shapes attached to the body.
    public func bodyShapeCount(_ bodyId: BodyID) -> Int32 {
        body(bodyId).shapeCount
    }

    /// The body's joints, in list order.
    public func bodyJoints(_ bodyId: BodyID) -> [JointID] {
        var ids: [JointID] = []
        var key = body(bodyId).headJointKey
        while key != B3.nullIndex {
            let jointId = key >> 1
            let edgeIndex = Int(key & 1)
            let joint = joints[Int(jointId)]
            key = joint.edges[edgeIndex].nextKey
            ids.append(
                JointID(index1: jointId + 1, world0: worldId, generation: joint.generation))
        }
        return ids
    }
}

// MARK: - Shape accessors

extension World {
    /// The shape's geometry type.
    public func shapeType(_ shapeId: ShapeID) -> ShapeType {
        shapes[Int(shapeId.index1 - 1)].type
    }

    /// The body that owns this shape.
    public func shapeBody(_ shapeId: ShapeID) -> BodyID {
        let shape = shapes[Int(shapeId.index1 - 1)]
        let body = bodies[Int(shape.bodyId)]
        return BodyID(index1: body.id + 1, world0: worldId, generation: body.generation)
    }

    // MARK: Material

    /// The shape's density.
    public func shapeDensity(_ shapeId: ShapeID) -> Float {
        shapes[Int(shapeId.index1 - 1)].density
    }

    /// Set the shape density. Updating the body mass requires a body pass.
    public func setShapeDensity(_ shapeId: ShapeID, _ density: Float, updateBodyMass: Bool = true) {
        var shape = shapes[Int(shapeId.index1 - 1)]
        shape.density = density
        shapes[Int(shapeId.index1 - 1)] = shape

        // The setter recomputes the mass inline rather than deferring it, so the dirty flag
        // never survives into a step.
        if updateBodyMass {
            updateBodyMassData(shape.bodyId)
        }
    }

    /// The base surface material's friction.
    public func shapeFriction(_ shapeId: ShapeID) -> Float {
        shapes[Int(shapeId.index1 - 1)].material(0).friction
    }

    /// Set the base surface material's friction.
    public func setShapeFriction(_ shapeId: ShapeID, _ friction: Float) {
        var shape = shapes[Int(shapeId.index1 - 1)]
        guard shape.materials.isEmpty == false else { return }
        shape.materials[0].friction = friction
        shapes[Int(shapeId.index1 - 1)] = shape
    }

    /// The base surface material's restitution.
    public func shapeRestitution(_ shapeId: ShapeID) -> Float {
        shapes[Int(shapeId.index1 - 1)].material(0).restitution
    }

    /// Set the base surface material's restitution.
    public func setShapeRestitution(_ shapeId: ShapeID, _ restitution: Float) {
        var shape = shapes[Int(shapeId.index1 - 1)]
        guard shape.materials.isEmpty == false else { return }
        shape.materials[0].restitution = restitution
        shapes[Int(shapeId.index1 - 1)] = shape
    }

    /// The base surface material.
    public func shapeSurfaceMaterial(_ shapeId: ShapeID) -> SurfaceMaterial {
        shapes[Int(shapeId.index1 - 1)].material(0)
    }

    /// Replace the base surface material. Per-triangle mesh materials are untouched.
    public func setShapeSurfaceMaterial(_ shapeId: ShapeID, _ material: SurfaceMaterial) {
        var shape = shapes[Int(shapeId.index1 - 1)]
        guard shape.materials.isEmpty == false else { return }
        shape.materials[0] = material
        shapes[Int(shapeId.index1 - 1)] = shape
    }

    /// The shape's collision filter.
    public func shapeFilter(_ shapeId: ShapeID) -> Filter {
        shapes[Int(shapeId.index1 - 1)].filter
    }

    // MARK: Event flags

    public func shapeAreSensorEventsEnabled(_ shapeId: ShapeID) -> Bool {
        (shapes[Int(shapeId.index1 - 1)].flags & ShapeFlags.enableSensorEvents.rawValue) != 0
    }

    public func shapeAreContactEventsEnabled(_ shapeId: ShapeID) -> Bool {
        (shapes[Int(shapeId.index1 - 1)].flags & ShapeFlags.enableContactEvents.rawValue) != 0
    }

    public func shapeAreHitEventsEnabled(_ shapeId: ShapeID) -> Bool {
        (shapes[Int(shapeId.index1 - 1)].flags & ShapeFlags.enableHitEvents.rawValue) != 0
    }

    public func shapeArePreSolveEventsEnabled(_ shapeId: ShapeID) -> Bool {
        (shapes[Int(shapeId.index1 - 1)].flags & ShapeFlags.enablePreSolveEvents.rawValue) != 0
    }

    // MARK: Geometry access

    /// The shape's sphere, or nil when it is another type.
    public func shapeSphere(_ shapeId: ShapeID) -> Sphere? {
        if case .sphere(let sphere) = shapes[Int(shapeId.index1 - 1)].geometry { return sphere }
        return nil
    }

    /// The shape's capsule, or nil when it is another type.
    public func shapeCapsule(_ shapeId: ShapeID) -> Capsule? {
        if case .capsule(let capsule) = shapes[Int(shapeId.index1 - 1)].geometry { return capsule }
        return nil
    }

    /// The shape's hull, or nil when it is another type.
    public func shapeHull(_ shapeId: ShapeID) -> Hull? {
        if case .hull(let hull) = shapes[Int(shapeId.index1 - 1)].geometry { return hull }
        return nil
    }

    /// The shape's mesh, or nil when it is another type.
    public func shapeMesh(_ shapeId: ShapeID) -> Mesh? {
        if case .mesh(let mesh) = shapes[Int(shapeId.index1 - 1)].geometry { return mesh }
        return nil
    }

    /// The shape's height field, or nil when it is another type.
    public func shapeHeightField(_ shapeId: ShapeID) -> HeightField? {
        if case .heightField(let field) = shapes[Int(shapeId.index1 - 1)].geometry { return field }
        return nil
    }

    /// The shape's compound, or nil when it is another type.
    public func shapeCompound(_ shapeId: ShapeID) -> Compound? {
        if case .compound(let compound) = shapes[Int(shapeId.index1 - 1)].geometry {
            return compound
        }
        return nil
    }
}

// MARK: - Additional shape API

extension World {
    /// True when the shape is a sensor.
    public func shapeIsSensor(_ shapeId: ShapeID) -> Bool {
        shapes[Int(shapeId.index1 - 1)].sensorIndex != B3.nullIndex
    }

    /// The shape's user data pointer.
    public func shapeUserData(_ shapeId: ShapeID) -> UnsafeMutableRawPointer? {
        shapes[Int(shapeId.index1 - 1)].userData
    }

    /// Replace the shape's user data pointer.
    public func setShapeUserData(_ shapeId: ShapeID, _ userData: UnsafeMutableRawPointer?) {
        var shape = shapes[Int(shapeId.index1 - 1)]
        shape.userData = userData
        shapes[Int(shapeId.index1 - 1)] = shape
    }

    /// The shape's AABB in world space.
    public func shapeAABB(_ shapeId: ShapeID) -> AABB {
        let shape = shapes[Int(shapeId.index1 - 1)]
        let transform = bodyTransformQuick(shape.bodyId)
        return offsetAABB(computeShapeAABB(shape, Transform(Vec3(), transform.q)), transform.p)
    }

    /// The closest point on the shape to a target, in world space.
    public func shapeClosestPoint(_ shapeId: ShapeID, _ target: Pos) -> Pos {
        let shape = shapes[Int(shapeId.index1 - 1)]
        let transform = bodyTransformQuick(shape.bodyId)

        // The low level query runs in the body frame so it stays precise far from the origin.
        var local = invTransformWorldPoint(transform, target)
        var output = DistanceOutput()
        var cache = SimplexCache()

        output = withUnsafePointer(to: &local) { point in
            makeShapeProxy(shape).withProxy { proxyA in
                var input = DistanceInput()
                input.proxyA = proxyA
                input.proxyB = ShapeProxy(points: point, count: 1, radius: 0)
                input.transform = .identity
                input.useRadii = true
                return shapeDistance(input, &cache)
            }
        }

        return transformWorldPoint(transform, output.pointA)
    }

    /// The number of materials a mesh shape carries.
    public func shapeMeshMaterialCount(_ shapeId: ShapeID) -> Int32 {
        shapes[Int(shapeId.index1 - 1)].materialCount
    }

    /// A mesh shape's per-triangle surface material.
    public func shapeMeshSurfaceMaterial(_ shapeId: ShapeID, _ index: Int32) -> SurfaceMaterial? {
        let shape = shapes[Int(shapeId.index1 - 1)]
        guard index >= 0, index < shape.materialCount else { return nil }
        return shape.material(index)
    }

    /// Replace a mesh shape's per-triangle surface material.
    public func setShapeMeshMaterial(_ shapeId: ShapeID, _ index: Int32, _ material: SurfaceMaterial) {
        var shape = shapes[Int(shapeId.index1 - 1)]
        guard index >= 0, index < Int32(shape.materials.count) else { return }
        shape.materials[Int(index)] = material
        shapes[Int(shapeId.index1 - 1)] = shape
    }

    /// Enable or disable sensor events for a shape.
    public func enableShapeSensorEvents(_ shapeId: ShapeID, _ enable: Bool) {
        setShapeFlag(shapeId, ShapeFlags.enableSensorEvents, enable)
    }

    /// Enable or disable contact events for a shape.
    public func enableShapeContactEvents(_ shapeId: ShapeID, _ enable: Bool) {
        setShapeFlag(shapeId, ShapeFlags.enableContactEvents, enable)
    }

    /// Enable or disable hit events for a shape.
    public func enableShapeHitEvents(_ shapeId: ShapeID, _ enable: Bool) {
        setShapeFlag(shapeId, ShapeFlags.enableHitEvents, enable)
    }

    /// Enable or disable pre-solve events for a shape.
    public func enableShapePreSolveEvents(_ shapeId: ShapeID, _ enable: Bool) {
        setShapeFlag(shapeId, ShapeFlags.enablePreSolveEvents, enable)
    }

    private func setShapeFlag(_ shapeId: ShapeID, _ flag: ShapeFlags, _ enable: Bool) {
        var shape = shapes[Int(shapeId.index1 - 1)]
        if enable {
            shape.flags |= flag.rawValue
        } else {
            shape.flags &= ~flag.rawValue
        }
        shapes[Int(shapeId.index1 - 1)] = shape
    }

    /// Replace the shape's collision filter. This is almost as expensive as recreating the shape.
    public func setShapeFilter(_ shapeId: ShapeID, _ filter: Filter) {
        var shape = shapes[Int(shapeId.index1 - 1)]
        shape.filter = filter
        shapes[Int(shapeId.index1 - 1)] = shape
    }

    /// The world's axis-aligned bounds, or nil when it has no shapes.
    public func worldBounds() -> AABB? {
        var bounds = AABB()
        var haveBounds = false

        for tree in broadPhase.trees {
            guard tree.proxyCount > 0 else { continue }
            let root = tree.rootBounds
            bounds = haveBounds ? aabbUnion(bounds, root) : root
            haveBounds = true
        }
        return haveBounds ? bounds : nil
    }
}

// MARK: - Body extras

extension World {
    /// The touching contacts of a body. Only touching contacts are reported.
    public func bodyContactData(_ bodyId: BodyID, capacity: Int = 32) -> [ContactData] {
        var result: [ContactData] = []
        var key = body(bodyId).headContactKey

        while key != B3.nullIndex && result.count < capacity {
            let contactId = key >> 1
            let edgeIndex = Int(key & 1)
            let contact = contacts[Int(contactId)]
            key = contact.edges[edgeIndex].nextKey

            guard (contact.flags & ContactFlags.touching.rawValue) != 0 else { continue }

            let shapeA = shapes[Int(contact.shapeIdA)]
            let shapeB = shapes[Int(contact.shapeIdB)]

            var data = ContactData()
            data.contactId = ContactID(
                index1: contact.contactId + 1, world0: worldId, generation: contact.generation)
            data.shapeIdA = ShapeID(
                index1: shapeA.id + 1, world0: worldId, generation: shapeA.generation)
            data.shapeIdB = ShapeID(
                index1: shapeB.id + 1, world0: worldId, generation: shapeB.generation)
            data.manifolds = contact.manifoldBuffer
            result.append(data)

        }

        return result
    }

    /// The world AABB containing all of the body's shapes. Empty and centered on the body origin
    /// when the body has no shapes.
    public func bodyComputeAABB(_ bodyId: BodyID) -> AABB {
        let record = body(bodyId)
        if record.headShapeId == B3.nullIndex {
            let p = bodyTransformQuick(bodyIndex(bodyId)).p
            return offsetAABB(AABB(Vec3(), Vec3()), p)
        }

        var aabb = shapes[Int(record.headShapeId)].aabb
        var shapeId = shapes[Int(record.headShapeId)].nextShapeId
        while shapeId != B3.nullIndex {
            aabb = aabbUnion(aabb, shapes[Int(shapeId)].aabb)
            shapeId = shapes[Int(shapeId)].nextShapeId
        }
        return aabb
    }

    /// Recompute the body's mass properties from its shapes.
    public func applyBodyMassFromShapes(_ bodyId: BodyID) {
        updateBodyMassData(bodyIndex(bodyId))
    }

    /// The closest point on the body to a world target, and its distance.
    public func bodyClosestPoint(_ bodyId: BodyID, _ target: Vec3) -> (point: Vec3, distance: Float) {
        let transform = toRelativeTransform(bodyTransformQuick(bodyIndex(bodyId)), Pos())

        var closestDistance = Float.greatestFiniteMagnitude
        var closestPoint = transform.p

        var localTarget = target
        let record = body(bodyId)
        var shapeId = record.headShapeId

        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            var cache = SimplexCache()

            let output = withUnsafePointer(to: &localTarget) { point in
                makeShapeProxy(shape).withProxy { proxyB in
                    var input = DistanceInput()
                    input.proxyA = ShapeProxy(points: point, count: 1, radius: 0)
                    input.proxyB = proxyB
                    input.transform = transform
                    input.useRadii = false
                    return shapeDistance(input, &cache)
                }
            }

            if output.distance < closestDistance {
                closestDistance = output.distance
                closestPoint = output.pointB
            }
            shapeId = shape.nextShapeId
        }

        return (closestPoint, closestDistance)
    }

    /// Allow or forbid fast rotation for this body.
    public func allowBodyFastRotation(_ bodyId: BodyID, _ allow: Bool) {
        let index = bodyIndex(bodyId)
        var record = bodies[Int(index)]
        if allow {
            record.flags |= BodyFlags.allowFastRotation.rawValue
        } else {
            record.flags &= ~BodyFlags.allowFastRotation.rawValue
        }
        bodies[Int(index)] = record
        syncBodyFlags(index)
    }

    /// True when this body may rotate faster than the angular speed limit.
    public func isBodyFastRotationAllowed(_ bodyId: BodyID) -> Bool {
        (body(bodyId).flags & BodyFlags.allowFastRotation.rawValue) != 0
    }

    /// The body's user data pointer.
    public func bodyUserData(_ bodyId: BodyID) -> UnsafeMutableRawPointer? {
        body(bodyId).userData
    }

    /// Replace the body's user data pointer.
    public func setBodyUserData(_ bodyId: BodyID, _ userData: UnsafeMutableRawPointer?) {
        let index = bodyIndex(bodyId)
        var record = bodies[Int(index)]
        record.userData = userData
        bodies[Int(index)] = record
    }

    /// Drive a body toward a target transform by setting its velocity. Used by character movers.
    public func setBodyTargetTransform(
        _ bodyId: BodyID, _ target: WorldTransform, timeStep: Float, wake: Bool = true
    ) {
        let index = bodyIndex(bodyId)
        let record = bodies[Int(index)]

        if record.setIndex == SetType.disabledSet.rawValue { return }
        if record.type == .staticBody || timeStep <= 0 { return }
        if record.setIndex != SetType.awakeSet.rawValue && wake == false { return }

        if wake { wakeBody(index) }

        let sim = bodySimFor(bodyId)
        let current = body(bodyId)
        guard current.setIndex == SetType.awakeSet.rawValue else { return }

        // Linear velocity from the center difference, taken in world precision.
        let center1 = sim.center
        let center2 = transformWorldPoint(target, sim.localCenter)
        let invTimeStep = 1 / timeStep
        let linearVelocity = invTimeStep * subPos(center2, center1)

        // w = 2 * (q2 - q1) * conjugate(q1) / dt, along the shortest arc.
        var q1 = sim.transform.q
        var q2 = target.q
        if dotQuat(q1, q2) < 0 {
            q2 = Quat(-q2.v, -q2.s)
        }
        q1 = sim.transform.q
        let dq = Quat(q2.v - q1.v, q2.s - q1.s)
        let omega = mulQuat(dq, conjugate(q1))
        let angularVelocity = (2 * invTimeStep) * omega.v

        var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(current.localIndex)]
        state.linearVelocity = linearVelocity
        state.angularVelocity = angularVelocity
        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(current.localIndex)] = state
    }
}

// MARK: - Shape contacts and contact validity

extension World {
    /// The touching contacts that involve this shape. Sensors report nothing.
    public func shapeContactData(_ shapeId: ShapeID, capacity: Int = 32) -> [ContactData] {
        let shapeIndex = shapeId.index1 - 1
        let shape = shapes[Int(shapeIndex)]
        if shape.sensorIndex != B3.nullIndex { return [] }

        var result: [ContactData] = []
        var key = bodies[Int(shape.bodyId)].headContactKey

        while key != B3.nullIndex && result.count < capacity {
            let contactId = key >> 1
            let edgeIndex = Int(key & 1)
            let contact = contacts[Int(contactId)]
            key = contact.edges[edgeIndex].nextKey

            let involvesShape =
                contact.shapeIdA == shapeIndex || contact.shapeIdB == shapeIndex
            guard involvesShape,
                (contact.flags & ContactFlags.touching.rawValue) != 0
            else { continue }

            let shapeA = shapes[Int(contact.shapeIdA)]
            let shapeB = shapes[Int(contact.shapeIdB)]

            var data = ContactData()
            data.contactId = ContactID(
                index1: contact.contactId + 1, world0: worldId, generation: contact.generation)
            data.shapeIdA = ShapeID(
                index1: shapeA.id + 1, world0: worldId, generation: shapeA.generation)
            data.shapeIdB = ShapeID(
                index1: shapeB.id + 1, world0: worldId, generation: shapeB.generation)

            data.manifolds = contact.manifoldBuffer
            result.append(data)
        }

        return result
    }

    /// True while the contact id refers to a live contact.
    public func contactIsValid(_ contactId: ContactID) -> Bool {
        let index = contactId.index1 - 1
        guard index >= 0, Int(index) < contacts.count else { return false }
        let contact = contacts[Int(index)]
        return contact.contactId != B3.nullIndex
            && contact.generation == contactId.generation
    }
}

extension World {
    /// True when continuous collision handled this body during the most recent step. The flag is
    /// transient and is cleared at the start of the next step.
    public func isBodyTimeOfImpact(_ bodyId: BodyID) -> Bool {
        let record = body(bodyId)
        let sim = solverSets[Int(record.setIndex)].bodySims[Int(record.localIndex)]
        return (sim.flags & BodyFlags.hadTimeOfImpact.rawValue) != 0
    }

    /// The body's minimum extent, the smallest half dimension over its shapes.
    public func bodyMinExtentValue(_ bodyId: BodyID) -> Float {
        bodyMinExtent(bodyId)
    }
}
