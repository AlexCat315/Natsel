

// Joint solver pipeline and the distance joint (distance_joint.c).

extension World {
    /// Fetch the body sim for a body id from whatever set owns it.
    func jointBodySim(_ bodyId: Int32) -> BodySim {
        let body = bodies[Int(bodyId)]
        return solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]
    }

    func prepareJoint(_ sim: inout JointSim, _ context: StepContext) {
        // Clamp joint hertz based on the time step to reduce jitter.
        let hertz = minFloat(sim.constraintHertz, 0.25 * context.inv_h)
        sim.constraintSoftness = makeSoft(hertz, sim.constraintDampingRatio, context.h)

        switch sim.type {
        case .distanceJoint:
            prepareDistanceJoint(&sim, context)
        case .revoluteJoint:
            prepareRevoluteJoint(&sim, context)
        case .weldJoint:
            prepareWeldJoint(&sim, context)
        case .parallelJoint:
            prepareParallelJoint(&sim, context)
        case .motorJoint:
            prepareMotorJoint(&sim, context)
        case .prismaticJoint:
            preparePrismaticJoint(&sim, context)
        case .sphericalJoint:
            prepareSphericalJoint(&sim, context)
        case .wheelJoint:
            prepareWheelJoint(&sim, context)
        default:
            break
        }
    }

    func warmStartJoint(_ sim: inout JointSim, _ context: StepContext) {
        switch sim.type {
        case .distanceJoint:
            warmStartDistanceJoint(&sim, context)
        case .revoluteJoint:
            warmStartRevoluteJoint(&sim, context)
        case .weldJoint:
            warmStartWeldJoint(&sim, context)
        case .parallelJoint:
            warmStartParallelJoint(&sim, context)
        case .motorJoint:
            warmStartMotorJoint(&sim, context)
        case .prismaticJoint:
            warmStartPrismaticJoint(&sim, context)
        case .sphericalJoint:
            warmStartSphericalJoint(&sim, context)
        case .wheelJoint:
            warmStartWheelJoint(&sim, context)
        default:
            break
        }
    }

    func solveJoint(_ sim: inout JointSim, _ context: StepContext, _ useBias: Bool) {
        switch sim.type {
        case .distanceJoint:
            solveDistanceJoint(&sim, context, useBias)
        case .revoluteJoint:
            solveRevoluteJoint(&sim, context, useBias)
        case .weldJoint:
            solveWeldJoint(&sim, context, useBias)
        case .parallelJoint:
            solveParallelJoint(&sim, context)
        case .motorJoint:
            solveMotorJoint(&sim, context)
        case .prismaticJoint:
            solvePrismaticJoint(&sim, context, useBias)
        case .sphericalJoint:
            solveSphericalJoint(&sim, context, useBias)
        case .wheelJoint:
            solveWheelJoint(&sim, context, useBias)
        default:
            break
        }
    }

    // MARK: Pipeline

    func prepareAllJoints(_ context: StepContext) {
        for ci in 0..<Int(B3.graphColorCount) {
            prepareJoints(Int32(ci), context)
        }
    }

    func warmStartAllJoints(_ context: StepContext) {
        for ci in 0..<Int(B3.graphColorCount) {
            warmStartJoints(Int32(ci), context)
        }
    }

    func solveAllJoints(_ context: StepContext, _ useBias: Bool) {
        for ci in 0..<Int(B3.graphColorCount) {
            solveJoints(Int32(ci), context, useBias)
        }
    }

    // MARK: Per-color joint stages

    /// Prepare the joints of a single graph color.
    ///
    /// The array is mutated through `withUnsafeMutableBufferPointer` so the (large) `JointSim`
    /// value does not get copied out and back per color, and no exclusive-access nesting is needed.
    func prepareJoints(_ colorIndex: Int32, _ context: StepContext) {
        constraintGraph.colors[Int(colorIndex)].jointSims.withUnsafeMutableBufferPointer { list in
            for i in 0..<list.count {
                prepareJoint(&list[i], context)
            }
        }
    }

    /// Warm start the joints of a single graph color.
    func warmStartJoints(_ colorIndex: Int32, _ context: StepContext) {
        constraintGraph.colors[Int(colorIndex)].jointSims.withUnsafeMutableBufferPointer { list in
            for i in 0..<list.count {
                warmStartJoint(&list[i], context)
            }
        }
    }

    /// Solve the joints of a single graph color.
    func solveJoints(_ colorIndex: Int32, _ context: StepContext, _ useBias: Bool) {
        constraintGraph.colors[Int(colorIndex)].jointSims.withUnsafeMutableBufferPointer { list in
            for i in 0..<list.count {
                solveJoint(&list[i], context, useBias)
            }
        }
    }

    // MARK: Distance joint

    func prepareDistanceJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .distanceJoint)

        let bodyA = bodies[Int(base.bodyIdA)]
        let bodyB = bodies[Int(base.bodyIdB)]

        b3Assert(
            bodyA.setIndex == SetType.awakeSet.rawValue || bodyB.setIndex == SetType.awakeSet.rawValue
        )

        let simA = solverSets[Int(bodyA.setIndex)].bodySims[Int(bodyA.localIndex)]
        let simB = solverSets[Int(bodyB.setIndex)].bodySims[Int(bodyB.localIndex)]

        let mA = simA.invMass
        let iA = simA.invInertiaWorld
        let mB = simB.invMass
        let iB = simB.invInertiaWorld

        base.invMassA = mA
        base.invMassB = mB
        base.invIA = iA
        base.invIB = iB

        var joint = base.distanceJoint

        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex

        // initial anchors in world space
        joint.anchorA = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.anchorB = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)
        joint.deltaCenter = subPos(simB.center, simA.center)

        let rA = joint.anchorA
        let rB = joint.anchorB
        let separation = (rB - rA) + joint.deltaCenter
        let axis = normalize(separation)

        // compute effective mass
        let crA = cross(rA, axis)
        let crB = cross(rB, axis)
        let k = mA + mB + dot(crA, mulMV(iA, crA)) + dot(crB, mulMV(iB, crB))
        joint.axialMass = k > 0 ? 1 / k : 0

        joint.distanceSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

        if context.enableWarmStarting == false {
            joint.impulse = 0
            joint.lowerImpulse = 0
            joint.upperImpulse = 0
            joint.motorImpulse = 0
        }

        base.distanceJoint = joint
    }

    func warmStartDistanceJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .distanceJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        let joint = base.distanceJoint
        var stateA = solverState(joint.indexA)
        var stateB = solverState(joint.indexB)

        let rA = rotateVector(stateA.deltaRotation, joint.anchorA)
        let rB = rotateVector(stateB.deltaRotation, joint.anchorB)

        let ds = (stateB.deltaPosition - stateA.deltaPosition) + (rB - rA)
        let separation = joint.deltaCenter + ds
        let axis = normalize(separation)

        let axialImpulse = joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse
        let P = axialImpulse * axis

        if (stateA.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateA.linearVelocity = stateA.linearVelocity - mA * P
            stateA.angularVelocity = stateA.angularVelocity - mulMV(iA, cross(rA, P))
        }

        if (stateB.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateB.linearVelocity = stateB.linearVelocity + mB * P
            stateB.angularVelocity = stateB.angularVelocity + mulMV(iB, cross(rB, P))
        }

        storeSolverState(joint.indexA, stateA)
        storeSolverState(joint.indexB, stateB)
    }

    func solveDistanceJoint(_ base: inout JointSim, _ context: StepContext, _ useBias: Bool) {
        b3Assert(base.type == .distanceJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        var joint = base.distanceJoint
        var stateA = solverState(joint.indexA)
        var stateB = solverState(joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        // current anchors
        let rA = rotateVector(stateA.deltaRotation, joint.anchorA)
        let rB = rotateVector(stateB.deltaRotation, joint.anchorB)

        // current separation
        let ds = (stateB.deltaPosition - stateA.deltaPosition) + (rB - rA)
        let separation = joint.deltaCenter + ds

        let length = length(separation)
        let axis = normalize(separation)

        // joint is soft if
        // - spring is enabled
        // - and (joint limit is disabled or limits are not equal)
        if joint.enableSpring && (joint.minLength < joint.maxLength || joint.enableLimit == false) {
            // spring
            if joint.hertz > 0 {
                // Cdot = dot(u, v + cross(w, r))
                let vr = (vB - vA) + (cross(wB, rB) - cross(wA, rA))
                let Cdot = dot(axis, vr)
                let C = length - joint.length
                let bias = joint.distanceSoftness.biasRate * C

                let m = joint.distanceSoftness.massScale * joint.axialMass
                let oldImpulse = joint.impulse
                var impulse = -m * (Cdot + bias) - joint.distanceSoftness.impulseScale * oldImpulse
                let h = context.h
                joint.impulse = clampFloat(
                    joint.impulse + impulse, joint.lowerSpringForce * h, joint.upperSpringForce * h
                )
                impulse = joint.impulse - oldImpulse

                let P = impulse * axis
                vA = vA - mA * P
                wA = wA - mulMV(iA, cross(rA, P))
                vB = vB + mB * P
                wB = wB + mulMV(iB, cross(rB, P))
            }

            if joint.enableLimit {
                // lower limit
                do {
                    let vr = (vB - vA) + (cross(wB, rB) - cross(wA, rA))
                    let Cdot = dot(axis, vr)

                    let C = length - joint.minLength

                    var bias: Float = 0
                    var massCoeff: Float = 1
                    var impulseCoeff: Float = 0
                    if C > 0 {
                        // speculative
                        bias = C * context.inv_h
                    } else if useBias {
                        bias = base.constraintSoftness.biasRate * C
                        massCoeff = base.constraintSoftness.massScale
                        impulseCoeff = base.constraintSoftness.impulseScale
                    }

                    var impulse = -massCoeff * joint.axialMass * (Cdot + bias)
                        - impulseCoeff * joint.lowerImpulse
                    let newImpulse = maxFloat(0, joint.lowerImpulse + impulse)
                    impulse = newImpulse - joint.lowerImpulse
                    joint.lowerImpulse = newImpulse

                    let P = impulse * axis
                    vA = vA - mA * P
                    wA = wA - mulMV(iA, cross(rA, P))
                    vB = vB + mB * P
                    wB = wB + mulMV(iB, cross(rB, P))
                }

                // upper
                do {
                    let vr = (vA - vB) + (cross(wA, rA) - cross(wB, rB))
                    let Cdot = dot(axis, vr)

                    let C = joint.maxLength - length

                    var bias: Float = 0
                    var massScale: Float = 1
                    var impulseScale: Float = 0
                    if C > 0 {
                        // speculative
                        bias = C * context.inv_h
                    } else if useBias {
                        bias = base.constraintSoftness.biasRate * C
                        massScale = base.constraintSoftness.massScale
                        impulseScale = base.constraintSoftness.impulseScale
                    }

                    var impulse = -massScale * joint.axialMass * (Cdot + bias)
                        - impulseScale * joint.upperImpulse
                    let newImpulse = maxFloat(0, joint.upperImpulse + impulse)
                    impulse = newImpulse - joint.upperImpulse
                    joint.upperImpulse = newImpulse

                    let P = -impulse * axis
                    vA = vA - mA * P
                    wA = wA - mulMV(iA, cross(rA, P))
                    vB = vB + mB * P
                    wB = wB + mulMV(iB, cross(rB, P))
                }
            }

            if joint.enableMotor {
                let vr = (vB - vA) + (cross(wB, rB) - cross(wA, rA))
                let Cdot = dot(axis, vr)
                var impulse = joint.axialMass * (joint.motorSpeed - Cdot)
                let oldImpulse = joint.motorImpulse
                let maxImpulse = context.h * joint.maxMotorForce
                joint.motorImpulse = clampFloat(joint.motorImpulse + impulse, -maxImpulse, maxImpulse)
                impulse = joint.motorImpulse - oldImpulse

                let P = impulse * axis
                vA = vA - mA * P
                wA = wA - mulMV(iA, cross(rA, P))
                vB = vB + mB * P
                wB = wB + mulMV(iB, cross(rB, P))
            }
        } else {
            // rigid constraint
            let vr = (vB - vA) + (cross(wB, rB) - cross(wA, rA))
            let Cdot = dot(axis, vr)

            let C = length - joint.length

            var bias: Float = 0
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias {
                bias = base.constraintSoftness.biasRate * C
                massScale = base.constraintSoftness.massScale
                impulseScale = base.constraintSoftness.impulseScale
            }

            let impulse = -massScale * joint.axialMass * (Cdot + bias) - impulseScale * joint.impulse
            joint.impulse += impulse

            let P = impulse * axis
            vA = vA - mA * P
            wA = wA - mulMV(iA, cross(rA, P))
            vB = vB + mB * P
            wB = wB + mulMV(iB, cross(rB, P))
        }

        if (stateA.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateA.linearVelocity = vA
            stateA.angularVelocity = wA
        }

        if (stateB.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateB.linearVelocity = vB
            stateB.angularVelocity = wB
        }

        storeSolverState(joint.indexA, stateA)
        storeSolverState(joint.indexB, stateB)

        base.distanceJoint = joint
    }
}
