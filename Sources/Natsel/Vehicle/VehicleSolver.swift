// Vehicle constraint solver. Each substep:
// - warm start: engine torque, drivetrain coupling through the clutch, anti-roll bars, and the
//   accumulated suspension/tire impulses of the previous substep;
// - solve (with and without bias): suspension spring and bump stop, brakes, longitudinal and
//   lateral tire friction inside a friction ellipse, pitch/roll limit, and motorcycle lean.
//
// Sign conventions: the contact normal points from the ground toward the chassis. A positive
// suspension impulse pushes the chassis away from the ground. Positive wheel angular velocity rolls
// the vehicle forward, so the tire surface moves at `w * r` relative to the chassis.

extension World {
    // MARK: Prepare

    func prepareVehicles(_ context: World.StepContext) {
        guard vehicleIdPool.idCount > 0 else { return }
        for i in vehicles.indices where vehicles[i].active {
            prepareVehicle(i, context)
        }
    }

    private func prepareVehicle(_ index: Int, _ context: World.StepContext) {
        let awake = Int(SetType.awakeSet.rawValue)
        var record = VehicleRecord.empty
        swap(&record, &vehicles[index])
        defer { swap(&record, &vehicles[index]) }

        let chassis = bodies[Int(bodyIndex(record.bodyId))]
        guard chassis.setIndex == SetType.awakeSet.rawValue else {
            record.active = false
            return
        }

        let simA = solverSets[awake].bodySims[Int(chassis.localIndex)]
        let stateA = solverSets[awake].bodyStates[Int(chassis.localIndex)]
        record.chassisIndex = chassis.localIndex
        record.invMass = simA.invMass
        record.invI = simA.invInertiaWorld

        let q = simA.transform.q
        let chassisUp = rotateVector(q, record.localUp)
        let chassisForward = rotateVector(q, record.localForward)
        if let up = record.worldUp {
            record.stepUp = up
        } else if lengthSquared(gravity) > 0 {
            record.stepUp = -normalize(gravity)
        } else {
            record.stepUp = chassisUp
        }

        // Throttle against the direction of travel brakes first (automatic gearbox only).
        let forwardSpeed = dot(stateA.linearVelocity, chassisForward)
        var throttle = record.forwardInput
        var brake = record.brakeInput
        if record.drivetrain.transmission.settings.mode == .automatic
            && throttle * forwardSpeed < 0 && absFloat(forwardSpeed) > 1.0 * lengthUnitsPerMeter
        {
            brake = maxFloat(brake, absFloat(throttle))
            throttle = 0
        }
        record.effectiveThrottle = throttle
        record.effectiveBrake = brake

        // Per wheel contact geometry, masses, slip, and friction.
        let h = context.h
        var canShiftUp = true
        for w in record.wheels.indices {
            var wheel = record.wheels[w]
            let s = wheel.settings
            wheel.suspensionImpulse = 0
            wheel.stopImpulse = 0
            wheel.longitudinalImpulse = 0
            wheel.lateralImpulse = 0
            wheel.brakeImpulse = 0
            wheel.maxBrakeImpulse = record.drivetrain.kind == .tracked
                ? 0 : (brake * s.maxBrakeTorque + record.handBrakeInput * s.maxHandBrakeTorque) * h

            if wheel.hasContact == false {
                wheel.groundIndex = B3.nullIndex
                wheel.longitudinalSlip = 0
                wheel.lateralSlip = 0
                record.wheels[w] = wheel
                if isDrivenWheel(record, Int32(w)) { canShiftUp = false }
                continue
            }

            let n = wheel.contactNormal
            let ground = bodies[Int(wheel.contactBody)]
            var vB = Vec3()
            var wB = Vec3()
            if ground.setIndex == SetType.awakeSet.rawValue {
                let simB = solverSets[awake].bodySims[Int(ground.localIndex)]
                let stateB = solverSets[awake].bodyStates[Int(ground.localIndex)]
                wheel.groundIndex = ground.localIndex
                wheel.groundInvMass = simB.invMass
                wheel.groundInvI = simB.invInertiaWorld
                wheel.rB = subPos(wheel.contactPosition, simB.center)
                vB = stateB.linearVelocity
                wB = stateB.angularVelocity
            } else {
                wheel.groundIndex = B3.nullIndex
                wheel.groundInvMass = 0
                wheel.groundInvI = Matrix3()
                wheel.rB = Vec3()
            }

            wheel.rA = subPos(wheel.contactPosition, simA.center)
            wheel.rAs = s.enableSuspensionForcePoint
                ? subPos(transformWorldPoint(simA.transform, s.suspensionForcePoint), simA.center)
                : wheel.rA

            let mA = record.invMass
            let mB = wheel.groundInvMass
            let iA = record.invI
            let iB = wheel.groundInvI

            func bodyK(_ rA: Vec3, _ rB: Vec3, _ axis: Vec3) -> Float {
                let rnA = cross(rA, axis)
                let rnB = cross(rB, axis)
                return mA + mB + dot(rnA, mulMV(iA, rnA)) + dot(rnB, mulMV(iB, rnB))
            }

            let kN = bodyK(wheel.rAs, wheel.rB, n)
            wheel.suspensionMass = kN > 0 ? 1 / kN : 0
            wheel.longitudinalBodyK = bodyK(wheel.rA, wheel.rB, wheel.longitudinal)
            let kL = bodyK(wheel.rA, wheel.rB, wheel.lateral)
            wheel.lateralMass = kL > 0 ? 1 / kL : 0

            // Spring: frequency relative to the carried mass, or absolute stiffness and damping.
            if s.suspensionStiffness > 0 && wheel.suspensionMass > 0 {
                let m = wheel.suspensionMass
                let omega = (s.suspensionStiffness / m).squareRoot()
                let zeta = s.suspensionDamping / (2 * (s.suspensionStiffness * m).squareRoot())
                wheel.suspensionSoftness = makeSoft(omega / B3.twoPi, zeta, h)
            } else {
                wheel.suspensionSoftness = makeSoft(s.suspensionFrequency, s.suspensionDampingRatio, h)
            }
            wheel.suspensionPosition = wheel.suspensionLength - (s.suspensionMaxLength + s.suspensionPreloadLength)
            wheel.minLengthPosition = wheel.suspensionLength - s.suspensionMinLength

            // Slip from the velocities at the start of the step.
            let vRel = (stateA.linearVelocity + cross(stateA.angularVelocity, wheel.rA)) - (vB + cross(wB, wheel.rB))
            let vLong = dot(vRel, wheel.longitudinal)
            let vLat = dot(vRel, wheel.lateral)
            let surfaceSpeed = wheelSurfaceSpeed(record, w)
            let slowSpeed: Float = 0.05 * lengthUnitsPerMeter
            wheel.longitudinalSlip = absFloat(surfaceSpeed - vLong) / maxFloat(absFloat(vLong), slowSpeed)
            wheel.lateralSlip = atan2Approx(absFloat(vLat), absFloat(vLong))

            var longitudinal: Float
            var lateral: Float
            if record.drivetrain.kind == .tracked {
                longitudinal = s.trackLongitudinalFriction
                lateral = s.trackLateralFriction
            } else if absFloat(vLong) < slowSpeed && absFloat(vLat) < slowSpeed && absFloat(surfaceSpeed) < slowSpeed {
                // Static grip: a parked tire does not slip.
                longitudinal = curvePeak(record.wheelSettings[w].longitudinalFriction)
                lateral = curvePeak(record.wheelSettings[w].lateralFriction)
            } else {
                longitudinal = record.wheelSettings[w].longitudinalFriction.evaluate(wheel.longitudinalSlip)
                lateral = record.wheelSettings[w].lateralFriction.evaluate(wheel.lateralSlip * B3.radToDeg)
            }

            let surface = wheel.surfaceFriction
            if let callback = record.tireFriction {
                let result = callback(Int32(w), longitudinal, lateral, surface, wheel.userMaterialId)
                wheel.longitudinalFriction = maxFloat(0, result.longitudinal)
                wheel.lateralFriction = maxFloat(0, result.lateral)
            } else {
                let scale = maxFloat(0, surface).squareRoot()
                wheel.longitudinalFriction = longitudinal * scale
                wheel.lateralFriction = lateral * scale
            }

            record.wheels[w] = wheel
        }

        // Gearbox once per step. Shifting up is held while a driven wheel is in the air.
        record.drivetrain.transmission.update(
            context.dt, engineRPM: record.drivetrain.engine.rpm, forwardInput: throttle, canShiftUp: canShiftUp)

        // An automatic gearbox decouples the engine when coasting down to idle or braking, so the
        // idling engine neither creeps the vehicle nor fights the brakes.
        record.drivetrain.clutchOpen = false
        if record.drivetrain.transmission.settings.mode == .automatic && throttle == 0 {
            let idle = record.drivetrain.engine.settings.minRPM
            record.drivetrain.clutchOpen = brake > 0 || drivenEngineRPM(record) < 1.5 * idle
        }

        for t in record.drivetrain.tracks.indices {
            record.drivetrain.tracks[t].brakeImpulse = 0
            record.drivetrain.tracks[t].maxBrakeImpulse = brake * record.drivetrain.tracks[t].settings.maxBrakeTorque * h
        }

        // Pitch/roll limit.
        record.pitchRollActive = false
        record.pitchRollImpulse = 0
        if record.maxPitchRollAngle < B3.pi {
            let cosAngle = dot(chassisUp, record.stepUp)
            let limit = computeCosSin(record.maxPitchRollAngle).cosine
            // Engage slightly before the limit so the constraint is speculative.
            if cosAngle < limit + 0.05 {
                var len: Float = 0
                let axis = getLengthAndNormalize(&len, cross(chassisUp, record.stepUp))
                if len > 1.0e-5 {
                    let k = dot(axis, mulMV(record.invI, axis))
                    record.pitchRollActive = k > 0
                    record.pitchRollAxis = axis
                    record.pitchRollMass = k > 0 ? 1 / k : 0
                    record.pitchRollChassisUp = chassisUp
                }
            }
        }

        if record.drivetrain.kind == .motorcycle {
            prepareMotorcycleLean(&record, simA, stateA, context)
        }
    }

    @inline(__always)
    private func isDrivenWheel(_ record: VehicleRecord, _ wheel: Int32) -> Bool {
        if record.drivetrain.kind == .tracked {
            return Int(wheel) < record.drivetrain.wheelTrack.count && record.drivetrain.wheelTrack[Int(wheel)] >= 0
        }
        return record.drivetrain.differentials.contains { $0.leftWheel == wheel || $0.rightWheel == wheel }
    }

    /// Engine speed the driven wheels would impose through the current gear.
    private func drivenEngineRPM(_ record: VehicleRecord) -> Float {
        let ratio = absFloat(record.drivetrain.transmission.currentRatio)
        var sum: Float = 0
        var count: Float = 0
        if record.drivetrain.kind == .tracked {
            for track in record.drivetrain.tracks {
                sum += absFloat(track.angularVelocity) * track.settings.differentialRatio
                count += 1
            }
        } else {
            for diff in record.drivetrain.differentials {
                for wheel in [diff.leftWheel, diff.rightWheel] where wheel >= 0 {
                    sum += absFloat(record.wheels[Int(wheel)].angularVelocity) * diff.differentialRatio
                    count += 1
                }
            }
        }
        return count > 0 ? ratio * sum / count * angularVelocityToRPM : 0
    }

    @inline(__always)
    private func wheelSurfaceSpeed(_ record: VehicleRecord, _ w: Int) -> Float {
        if record.drivetrain.kind == .tracked {
            let t = record.drivetrain.wheelTrack[w]
            guard t >= 0 else { return record.wheels[w].angularVelocity * record.wheels[w].settings.radius }
            let track = record.drivetrain.tracks[Int(t)]
            return track.angularVelocity * track.drivenRadius
        }
        return record.wheels[w].angularVelocity * record.wheels[w].settings.radius
    }

    private func curvePeak(_ curve: LinearCurve) -> Float {
        curve.points.reduce(0) { maxFloat($0, $1.y) }
    }

    private func prepareMotorcycleLean(
        _ record: inout VehicleRecord, _ simA: BodySim, _ stateA: BodyState, _ context: StepContext
    ) {
        var lean = record.drivetrain.lean
        let settings = lean.settings
        let q = simA.transform.q
        let forward = rotateVector(q, record.localForward)
        let chassisUp = rotateVector(q, record.localUp)

        // Current lean about the forward axis, relative to world up.
        var upLength: Float = 0
        let projectedUp = getLengthAndNormalize(&upLength, record.stepUp - dot(record.stepUp, forward) * forward)
        guard upLength > 1.0e-4 else {
            lean.active = false
            record.drivetrain.lean = lean
            return
        }
        let side = cross(forward, projectedUp)
        lean.currentLean = atan2Approx(dot(chassisUp, side), dot(chassisUp, projectedUp))

        // Target lean balances gravity against the centripetal acceleration of the steered turn.
        let g = length(gravity)
        var target: Float = 0
        let front = Int(lean.frontWheel)
        if g > 0 && front < record.wheels.count && lean.wheelBase > 0 {
            let steer = record.wheels[front].steerAngle
            let cs = computeCosSin(absFloat(steer))
            if cs.sine > 1.0e-4 {
                let speed = dot(stateA.linearVelocity, forward)
                let radius = lean.wheelBase * cs.cosine / cs.sine
                let acceleration = speed * speed / radius
                target = atan2Approx(steer > 0 ? -acceleration : acceleration, g)
            }
        }
        target = clampFloat(target, -settings.maxLeanAngle, settings.maxLeanAngle)
        let smoothing = clampFloat(settings.leanSmoothingFactor, 0, 0.999)
        lean.targetLean = smoothing * lean.targetLean + (1 - smoothing) * target

        // Integral term against steady state error.
        let dt = context.dt
        lean.integral += (lean.targetLean - lean.currentLean) * dt
        lean.integral *= maxFloat(0, 1 - settings.leanSpringIntegrationCoefficientDecay * dt)

        let k = dot(forward, mulMV(record.invI, forward))
        lean.active = k > 0 && settings.leanSpringConstant > 0
        lean.axis = forward
        lean.mass = k > 0 ? 1 / k : 0
        var effectiveTarget = lean.targetLean
        if settings.leanSpringConstant > 0 {
            effectiveTarget += settings.leanSpringIntegrationCoefficient / settings.leanSpringConstant * lean.integral
        }
        lean.position = lean.currentLean - effectiveTarget
        if lean.active {
            let omega = (settings.leanSpringConstant * k).squareRoot()
            let zeta = settings.leanSpringDamping * (k / settings.leanSpringConstant).squareRoot() * 0.5
            lean.softness = makeSoft(omega / B3.twoPi, zeta, context.h)
        }
        lean.impulse = 0
        record.drivetrain.lean = lean
    }

    // MARK: Substep stages

    func warmStartVehicles(_ context: World.StepContext) {
        guard vehicleIdPool.idCount > 0 else { return }
        withSolverStates { states in
            for i in vehicles.indices where vehicles[i].active {
                vehicles[i].warmStart(states, context)
            }
        }
    }

    func solveVehicles(_ context: World.StepContext, _ useBias: Bool) {
        guard vehicleIdPool.idCount > 0 else { return }
        withSolverStates { states in
            for i in vehicles.indices where vehicles[i].active {
                vehicles[i].solve(states, context, useBias)
            }
        }
    }

    /// Record totals and advance the visual wheel rotation.
    func finalizeVehicles(_ context: World.StepContext) {
        guard vehicleIdPool.idCount > 0 else { return }
        let subSteps = Float(context.subStepCount)
        for i in vehicles.indices where vehicles[i].active {
            var record = vehicles[i]
            for w in record.wheels.indices {
                var wheel = record.wheels[w]
                if record.drivetrain.kind == .tracked {
                    let t = record.drivetrain.wheelTrack[w]
                    if t >= 0 {
                        let track = record.drivetrain.tracks[Int(t)]
                        wheel.angularVelocity = track.angularVelocity * track.drivenRadius / wheel.settings.radius
                    }
                }
                wheel.rotationAngle = remainderAngle(wheel.rotationAngle + wheel.angularVelocity * context.dt)
                wheel.totalSuspensionImpulse = (wheel.suspensionImpulse + wheel.stopImpulse) * subSteps
                wheel.totalLongitudinalImpulse = wheel.longitudinalImpulse * subSteps
                wheel.totalLateralImpulse = wheel.lateralImpulse * subSteps
                record.wheels[w] = wheel
            }
            vehicles[i] = record
        }
    }
}

@inline(__always)
private func remainderAngle(_ angle: Float) -> Float {
    var a = angle
    if a > B3.pi || a < -B3.pi {
        a -= B3.twoPi * (a / B3.twoPi).rounded()
    }
    return a
}

// MARK: - Constraint math

extension VehicleRecord {
    @inline(__always)
    private func groundState(_ states: UnsafeMutableBufferPointer<BodyState>, _ index: Int32) -> BodyState {
        World.stateAt(states, index)
    }

    /// Apply an impulse along `axis` to the chassis at rA and the opposite to the ground at rB.
    @inline(__always)
    private func applyPair(
        _ stateA: inout BodyState, _ stateB: inout BodyState, _ wheel: VehicleWheel,
        _ rA: Vec3, _ axis: Vec3, _ impulse: Float
    ) {
        let p = impulse * axis
        stateA.linearVelocity = mulAdd(stateA.linearVelocity, invMass, p)
        stateA.angularVelocity = stateA.angularVelocity + mulMV(invI, cross(rA, p))
        stateB.linearVelocity = mulSub(stateB.linearVelocity, wheel.groundInvMass, p)
        stateB.angularVelocity = stateB.angularVelocity - mulMV(wheel.groundInvI, cross(wheel.rB, p))
    }

    @inline(__always)
    private func relativeVelocity(_ stateA: BodyState, _ stateB: BodyState, _ rA: Vec3, _ rB: Vec3) -> Vec3 {
        (stateA.linearVelocity + cross(stateA.angularVelocity, rA))
            - (stateB.linearVelocity + cross(stateB.angularVelocity, rB))
    }

    /// Change of the suspension length since the step started.
    @inline(__always)
    private func suspensionDelta(_ stateA: BodyState, _ stateB: BodyState, _ wheel: VehicleWheel) -> Float {
        let dA = stateA.deltaPosition + (rotateVector(stateA.deltaRotation, wheel.rAs) - wheel.rAs)
        let dB = stateB.deltaPosition + (rotateVector(stateB.deltaRotation, wheel.rB) - wheel.rB)
        return dot(wheel.contactNormal, dA - dB)
    }

    // MARK: Warm start

    mutating func warmStart(_ states: UnsafeMutableBufferPointer<BodyState>, _ context: World.StepContext) {
        let h = context.h
        driveSubstep(h)

        // Brakes restart every substep.
        for w in wheels.indices { wheels[w].brakeImpulse = 0 }
        for t in drivetrain.tracks.indices { drivetrain.tracks[t].brakeImpulse = 0 }

        let chassis = chassisIndex
        var stateA = states[Int(chassis)]

        // Anti-roll bars: an explicit spring on the difference of the suspension lengths.
        for bar in antiRollBars {
            let left = Int(bar.leftWheel)
            let right = Int(bar.rightWheel)
            guard wheels[left].hasContact && wheels[right].hasContact else { continue }

            var leftState = groundState(states, wheels[left].groundIndex)
            let leftLength = wheels[left].suspensionLength + suspensionDelta(stateA, leftState, wheels[left])
            var rightState = groundState(states, wheels[right].groundIndex)
            let rightLength = wheels[right].suspensionLength + suspensionDelta(stateA, rightState, wheels[right])

            // The longer side is higher; push it down and the other side up.
            let impulse = bar.stiffness * (rightLength - leftLength) * h
            applyPair(&stateA, &leftState, wheels[left], wheels[left].rAs, wheels[left].contactNormal, impulse)
            World.setStateAt(states, wheels[left].groundIndex, leftState)
            rightState = groundState(states, wheels[right].groundIndex)
            applyPair(&stateA, &rightState, wheels[right], wheels[right].rAs, wheels[right].contactNormal, -impulse)
            World.setStateAt(states, wheels[right].groundIndex, rightState)
        }

        // Accumulated impulses of the previous substep.
        for w in wheels.indices where wheels[w].hasContact {
            let wheel = wheels[w]
            var stateB = groundState(states, wheel.groundIndex)
            applyPair(&stateA, &stateB, wheel, wheel.rAs, wheel.contactNormal, wheel.suspensionImpulse + wheel.stopImpulse)
            applyPair(&stateA, &stateB, wheel, wheel.rA, wheel.longitudinal, wheel.longitudinalImpulse)
            applyPair(&stateA, &stateB, wheel, wheel.rA, wheel.lateral, wheel.lateralImpulse)
            World.setStateAt(states, wheel.groundIndex, stateB)
            let radius = rollingProperties(w).radius
            applyRollingAngularImpulse(w, -radius * wheel.longitudinalImpulse)
        }

        if pitchRollActive {
            stateA.angularVelocity = mulAdd(stateA.angularVelocity, pitchRollImpulse, mulMV(invI, pitchRollAxis))
        }
        if drivetrain.kind == .motorcycle && drivetrain.lean.active {
            let lean = drivetrain.lean
            stateA.angularVelocity = mulAdd(stateA.angularVelocity, lean.impulse, mulMV(invI, lean.axis))
        }

        states[Int(chassis)] = stateA
    }

    /// Brake accumulator and capacity of the rolling part at wheel `w`.
    @inline(__always)
    private func brake(_ w: Int) -> (impulse: Float, max: Float) {
        if drivetrain.kind == .tracked {
            let t = drivetrain.wheelTrack[w]
            if t >= 0 {
                let track = drivetrain.tracks[Int(t)]
                return (track.brakeImpulse, track.maxBrakeImpulse)
            }
        }
        return (wheels[w].brakeImpulse, wheels[w].maxBrakeImpulse)
    }

    /// Change the rotation of the rolling part at wheel `w` by an angular impulse, optionally
    /// recording the part of it delivered by the brake.
    @inline(__always)
    private mutating func applyRollingAngularImpulse(_ w: Int, _ angular: Float, brake: Float = 0) {
        if drivetrain.kind == .tracked {
            let t = Int(drivetrain.wheelTrack[w])
            if t >= 0 {
                drivetrain.tracks[t].angularVelocity += angular * drivetrain.tracks[t].inverseInertia
                drivetrain.tracks[t].brakeImpulse += brake
                return
            }
        }
        wheels[w].angularVelocity += angular * wheels[w].inverseInertia
        wheels[w].brakeImpulse += brake
    }

    /// Solve the tire and the brake of one wheel together. Unknowns are the tire impulse on the
    /// chassis (dLambda) and the brake's angular impulse on the wheel (dBeta):
    ///   v' = v + kB dLambda,  w' = w - R invI dLambda + invI dBeta,
    /// with v' = R w' (no slip) and w' = 0 (brake holds) while neither impulse is clamped. Solving
    /// the pair exactly avoids the slow convergence of a light wheel between a heavy chassis and its
    /// brake.
    @inline(__always)
    private func solveBrakedTire(
        v: Float, omega: Float, radius: Float, invI: Float, kB: Float,
        lambda: Float, maxLambda: Float, beta: Float, maxBeta: Float
    ) -> (dLambda: Float, dBeta: Float) {
        guard kB > 0, invI > 0 else { return (0, 0) }

        // Both free: the wheel locks and the contact sticks.
        var dLambda = -v / kB
        var newLambda = clampFloat(lambda + dLambda, -maxLambda, maxLambda)
        dLambda = newLambda - lambda
        var dBeta = -omega / invI + radius * dLambda
        let newBeta = clampFloat(beta + dBeta, -maxBeta, maxBeta)
        if newBeta == beta + dBeta { return (dLambda, dBeta) }

        // The brake slips: it delivers its limit and the tire sees a rolling wheel.
        dBeta = newBeta - beta
        let k = kB + radius * radius * invI
        dLambda = -(v - radius * (omega + invI * dBeta)) / k
        newLambda = clampFloat(lambda + dLambda, -maxLambda, maxLambda)
        dLambda = newLambda - lambda
        return (dLambda, dBeta)
    }

    // MARK: Drivetrain

    /// Engine torque, damping, and the clutch for one substep.
    private mutating func driveSubstep(_ h: Float) {
        let throttle = absFloat(effectiveThrottle)
        var engine = drivetrain.engine
        let invIe = engine.inverseInertia
        engine.angularVelocity += engine.torque(throttle) * h * invIe
        engine.angularVelocity *= 1 / (1 + h * engine.settings.angularDamping)

        for w in wheels.indices {
            wheels[w].angularVelocity *= 1 / (1 + h * wheels[w].settings.angularDamping)
        }
        for t in drivetrain.tracks.indices {
            drivetrain.tracks[t].angularVelocity *= 1 / (1 + h * drivetrain.tracks[t].settings.angularDamping)
        }

        let ratio = drivetrain.transmission.currentRatio
        let clutch = drivetrain.clutchOpen
            ? 0 : drivetrain.transmission.settings.clutchStrength * drivetrain.transmission.clutchFriction
        if ratio != 0 && clutch > 0 {
            if drivetrain.kind == .tracked {
                coupleTracks(&engine, ratio, clutch, h)
            } else {
                coupleWheels(&engine, ratio, clutch, h)
            }
        }

        engine.clampSpeed()
        drivetrain.engine = engine
    }

    /// Implicit viscous clutch between the engine and the driven wheels. Each wheel's coupling
    /// coefficient is its share of the clutch shaft speed, so power is conserved:
    /// w_clutch = sum(c_i * w_i).
    private mutating func coupleWheels(_ engine: inout VehicleEngine, _ ratio: Float, _ clutch: Float, _ h: Float) {
        let differentials = drivetrain.differentials
        guard differentials.isEmpty == false else { return }

        // Torque between differentials, biased toward the slower axle.
        let limited = drivetrain.differentialLimitedSlipRatio
        let biased = differentials.count > 1 && limited < .greatestFiniteMagnitude && limited > 1
        func axleSpeed(_ diff: VehicleDifferentialSettings) -> Float {
            var sum: Float = 0
            var count: Float = 0
            if diff.leftWheel >= 0 { sum += absFloat(wheels[Int(diff.leftWheel)].angularVelocity); count += 1 }
            if diff.rightWheel >= 0 { sum += absFloat(wheels[Int(diff.rightWheel)].angularVelocity); count += 1 }
            return maxFloat(1.0e-3, count > 0 ? sum / count : 0)
        }
        var slowest: Float = .greatestFiniteMagnitude
        if biased {
            for diff in differentials { slowest = minFloat(slowest, axleSpeed(diff)) }
        }
        func axleRatio(_ diff: VehicleDifferentialSettings) -> Float {
            let base = maxFloat(0, diff.engineTorqueRatio)
            guard biased else { return base }
            let t = clampFloat((axleSpeed(diff) / slowest - 1) / (limited - 1), 0, 1)
            return base * (1 - t)
        }
        var total: Float = 0
        for diff in differentials { total += axleRatio(diff) }
        guard total > 0 else { return }

        // Each wheel's share of the clutch shaft speed: w_clutch = sum(c_i * w_i).
        for w in wheels.indices { wheels[w].clutchCoefficient = 0 }
        for diff in differentials {
            let axle = ratio * diff.differentialRatio * axleRatio(diff) / total
            let left = diff.leftWheel >= 0 ? wheels[Int(diff.leftWheel)].angularVelocity : 0
            let right = diff.rightWheel >= 0 ? wheels[Int(diff.rightWheel)].angularVelocity : 0
            var split = limitedSlipFractions(diff.leftRightSplit, left, right, diff.limitedSlipRatio)
            if diff.leftWheel < 0 { split = (0, 1) }
            if diff.rightWheel < 0 { split = (1, 0) }
            if diff.leftWheel >= 0 { wheels[Int(diff.leftWheel)].clutchCoefficient += axle * split.left }
            if diff.rightWheel >= 0 { wheels[Int(diff.rightWheel)].clutchCoefficient += axle * split.right }
        }

        let invIe = engine.inverseInertia
        var clutchSpeed: Float = 0
        var invK = invIe
        for w in wheels.indices {
            let c = wheels[w].clutchCoefficient
            clutchSpeed += c * wheels[w].angularVelocity
            invK += c * c * wheels[w].inverseInertia
        }
        let damping = clutch * h
        let impulse = damping * (engine.angularVelocity - clutchSpeed) / (1 + damping * invK)
        engine.angularVelocity -= impulse * invIe
        for w in wheels.indices where wheels[w].clutchCoefficient != 0 {
            wheels[w].angularVelocity += impulse * wheels[w].clutchCoefficient * wheels[w].inverseInertia
        }
    }

    /// Implicit viscous clutch between the engine and both tracks.
    private mutating func coupleTracks(_ engine: inout VehicleEngine, _ ratio: Float, _ clutch: Float, _ h: Float) {
        guard drivetrain.tracks.count >= 2 else { return }
        let invIe = engine.inverseInertia
        let c0 = 0.5 * ratio * drivetrain.tracks[0].settings.differentialRatio * leftTrackRatio
        let c1 = 0.5 * ratio * drivetrain.tracks[1].settings.differentialRatio * rightTrackRatio
        let clutchSpeed = c0 * drivetrain.tracks[0].angularVelocity + c1 * drivetrain.tracks[1].angularVelocity
        let invK = invIe + c0 * c0 * drivetrain.tracks[0].inverseInertia + c1 * c1 * drivetrain.tracks[1].inverseInertia
        let damping = clutch * h
        let impulse = damping * (engine.angularVelocity - clutchSpeed) / (1 + damping * invK)
        engine.angularVelocity -= impulse * invIe
        drivetrain.tracks[0].angularVelocity += impulse * c0 * drivetrain.tracks[0].inverseInertia
        drivetrain.tracks[1].angularVelocity += impulse * c1 * drivetrain.tracks[1].inverseInertia
    }

    // MARK: Solve

    mutating func solve(_ states: UnsafeMutableBufferPointer<BodyState>, _ context: World.StepContext, _ useBias: Bool) {
        let chassis = chassisIndex
        var stateA = states[Int(chassis)]
        let inv_h = context.inv_h
        let contactSoftness = context.contactSoftness

        // Brakes on rolling parts in the air. Grounded parts brake together with their tire.
        for w in wheels.indices where wheels[w].maxBrakeImpulse > 0 && wheels[w].hasContact == false {
            let invI = wheels[w].inverseInertia
            guard invI > 0 else { continue }
            let maxImpulse = wheels[w].maxBrakeImpulse
            let old = wheels[w].brakeImpulse
            let new = clampFloat(old - wheels[w].angularVelocity / invI, -maxImpulse, maxImpulse)
            wheels[w].brakeImpulse = new
            wheels[w].angularVelocity += (new - old) * invI
        }
        for t in drivetrain.tracks.indices where drivetrain.tracks[t].maxBrakeImpulse > 0
            && drivetrain.trackWheels[t].allSatisfy({ !wheels[Int($0)].hasContact })
        {
            let invI = drivetrain.tracks[t].inverseInertia
            guard invI > 0 else { continue }
            let maxImpulse = drivetrain.tracks[t].maxBrakeImpulse
            let old = drivetrain.tracks[t].brakeImpulse
            let new = clampFloat(old - drivetrain.tracks[t].angularVelocity / invI, -maxImpulse, maxImpulse)
            drivetrain.tracks[t].brakeImpulse = new
            drivetrain.tracks[t].angularVelocity += (new - old) * invI
        }

        let tracked = drivetrain.kind == .tracked
        for w in wheels.indices where wheels[w].hasContact {
            // One copy in and out per wheel; the rolling part is the wheel or its track.
            var wheel = wheels[w]
            let track = tracked ? Int(drivetrain.wheelTrack[w]) : -1
            var stateB = groundState(states, wheel.groundIndex)
            let n = wheel.contactNormal

            // Suspension first: tire friction is bounded by the load it carries.
            let delta = suspensionDelta(stateA, stateB, wheel)
            do {
                // Spring. It can push the chassis up but never pull it down.
                let vn = dot(relativeVelocity(stateA, stateB, wheel.rAs, wheel.rB), n)
                let soft = wheel.suspensionSoftness
                let c = wheel.suspensionPosition + delta
                var impulse = -soft.massScale * wheel.suspensionMass * (vn + soft.biasRate * c)
                    - soft.impulseScale * wheel.suspensionImpulse
                let newImpulse = maxFloat(wheel.suspensionImpulse + impulse, 0)
                impulse = newImpulse - wheel.suspensionImpulse
                wheel.suspensionImpulse = newImpulse
                applyPair(&stateA, &stateB, wheel, wheel.rAs, n, impulse)
            }
            do {
                // Bump stop at the minimum length.
                let c = wheel.minLengthPosition + delta
                let vn = dot(relativeVelocity(stateA, stateB, wheel.rAs, wheel.rB), n)
                var bias: Float = 0
                var massScale: Float = 1
                var impulseScale: Float = 0
                if c > 0 {
                    bias = c * inv_h
                } else if useBias {
                    bias = maxFloat(contactSoftness.biasRate * c, -4 * lengthUnitsPerMeter)
                    massScale = contactSoftness.massScale
                    impulseScale = contactSoftness.impulseScale
                }
                var impulse = -massScale * wheel.suspensionMass * (vn + bias) - impulseScale * wheel.stopImpulse
                let newImpulse = maxFloat(wheel.stopImpulse + impulse, 0)
                impulse = newImpulse - wheel.stopImpulse
                wheel.stopImpulse = newImpulse
                applyPair(&stateA, &stateB, wheel, wheel.rAs, n, impulse)
            }

            // Tire friction inside the friction ellipse.
            let load = wheel.suspensionImpulse + wheel.stopImpulse
            let maxLongitudinal = wheel.longitudinalFriction * load
            let maxLateral = wheel.lateralFriction * load

            var radius = wheel.settings.radius
            var invInertia = wheel.inverseInertia
            var omega = wheel.angularVelocity
            var brakeAccumulated = wheel.brakeImpulse
            var brakeMax = wheel.maxBrakeImpulse
            if track >= 0 {
                radius = drivetrain.tracks[track].drivenRadius
                invInertia = drivetrain.tracks[track].inverseInertia
                omega = drivetrain.tracks[track].angularVelocity
                brakeAccumulated = drivetrain.tracks[track].brakeImpulse
                brakeMax = drivetrain.tracks[track].maxBrakeImpulse
            }

            // Longitudinal: the contact patch moves with the tire surface.
            do {
                let v = dot(relativeVelocity(stateA, stateB, wheel.rA, wheel.rB), wheel.longitudinal)
                var impulse: Float
                var brakeImpulse: Float = 0
                if brakeMax > 0 {
                    (impulse, brakeImpulse) = solveBrakedTire(
                        v: v, omega: omega, radius: radius, invI: invInertia, kB: wheel.longitudinalBodyK,
                        lambda: wheel.longitudinalImpulse, maxLambda: maxLongitudinal,
                        beta: brakeAccumulated, maxBeta: brakeMax)
                } else {
                    let k = wheel.longitudinalBodyK + radius * radius * invInertia
                    impulse = k > 0 ? -(v - omega * radius) / k : 0
                    let newImpulse = clampFloat(wheel.longitudinalImpulse + impulse, -maxLongitudinal, maxLongitudinal)
                    impulse = newImpulse - wheel.longitudinalImpulse
                }
                wheel.longitudinalImpulse += impulse
                applyPair(&stateA, &stateB, wheel, wheel.rA, wheel.longitudinal, impulse)
                omega += (brakeImpulse - radius * impulse) * invInertia
                brakeAccumulated += brakeImpulse
            }

            // Lateral.
            do {
                let cdot = dot(relativeVelocity(stateA, stateB, wheel.rA, wheel.rB), wheel.lateral)
                var impulse = -wheel.lateralMass * cdot
                let newImpulse = clampFloat(wheel.lateralImpulse + impulse, -maxLateral, maxLateral)
                impulse = newImpulse - wheel.lateralImpulse
                wheel.lateralImpulse = newImpulse
                applyPair(&stateA, &stateB, wheel, wheel.rA, wheel.lateral, impulse)
            }

            // Combined slip: the tire cannot deliver both peak forces at once.
            if maxLongitudinal > 0 && maxLateral > 0 {
                let x = wheel.longitudinalImpulse / maxLongitudinal
                let y = wheel.lateralImpulse / maxLateral
                let r2 = x * x + y * y
                if r2 > 1 {
                    let scale = 1 / r2.squareRoot()
                    let newLongitudinal = wheel.longitudinalImpulse * scale
                    let newLateral = wheel.lateralImpulse * scale
                    let dLong = newLongitudinal - wheel.longitudinalImpulse
                    let dLat = newLateral - wheel.lateralImpulse
                    wheel.longitudinalImpulse = newLongitudinal
                    wheel.lateralImpulse = newLateral
                    applyPair(&stateA, &stateB, wheel, wheel.rA, wheel.longitudinal, dLong)
                    applyPair(&stateA, &stateB, wheel, wheel.rA, wheel.lateral, dLat)
                    omega -= radius * dLong * invInertia
                }
            }

            if track >= 0 {
                drivetrain.tracks[track].angularVelocity = omega
                drivetrain.tracks[track].brakeImpulse = brakeAccumulated
            } else {
                wheel.angularVelocity = omega
                wheel.brakeImpulse = brakeAccumulated
            }
            World.setStateAt(states, wheel.groundIndex, stateB)
            wheels[w] = wheel
        }

        // Pitch/roll limit against the world up.
        if pitchRollActive {
            let up = rotateVector(stateA.deltaRotation, pitchRollChassisUp)
            let angle = atan2Approx(length(cross(up, stepUp)), dot(up, stepUp))
            let c = maxPitchRollAngle - angle
            let cdot = dot(stateA.angularVelocity, pitchRollAxis)
            var bias: Float = 0
            var massScale: Float = 1
            var impulseScale: Float = 0
            if c > 0 {
                bias = c * inv_h
            } else if useBias {
                bias = contactSoftness.biasRate * c
                massScale = contactSoftness.massScale
                impulseScale = contactSoftness.impulseScale
            }
            var impulse = -massScale * pitchRollMass * (cdot + bias) - impulseScale * pitchRollImpulse
            let newImpulse = maxFloat(pitchRollImpulse + impulse, 0)
            impulse = newImpulse - pitchRollImpulse
            pitchRollImpulse = newImpulse
            stateA.angularVelocity = mulAdd(stateA.angularVelocity, impulse, mulMV(invI, pitchRollAxis))
        }

        // Motorcycle lean spring about the forward axis.
        if drivetrain.kind == .motorcycle && drivetrain.lean.active {
            var lean = drivetrain.lean
            // Small rotation of the chassis about the lean axis since the step started.
            let rotated = 2 * dot(stateA.deltaRotation.v, lean.axis) * (stateA.deltaRotation.s < 0 ? -1 : 1)
            let c = lean.position + rotated
            let cdot = dot(stateA.angularVelocity, lean.axis)
            let soft = lean.softness
            let impulse = -soft.massScale * lean.mass * (cdot + soft.biasRate * c) - soft.impulseScale * lean.impulse
            lean.impulse += impulse
            stateA.angularVelocity = mulAdd(stateA.angularVelocity, impulse, mulMV(invI, lean.axis))
            drivetrain.lean = lean
        }

        states[Int(chassis)] = stateA
    }

    /// Radius, inverse inertia, and surface speed of the part rolling on the ground at wheel `w`.
    @inline(__always)
    private func rollingProperties(_ w: Int) -> (radius: Float, invInertia: Float, surfaceSpeed: Float) {
        if drivetrain.kind == .tracked {
            let t = drivetrain.wheelTrack[w]
            if t >= 0 {
                let track = drivetrain.tracks[Int(t)]
                return (track.drivenRadius, track.inverseInertia, track.angularVelocity * track.drivenRadius)
            }
        }
        let wheel = wheels[w]
        return (wheel.settings.radius, wheel.inverseInertia, wheel.angularVelocity * wheel.settings.radius)
    }
}
