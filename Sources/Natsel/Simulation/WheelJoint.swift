

// Wheel joint (wheel_joint.c). Suspension along A's x-axis, spin about B's z-axis, optional
// steering about A's x-axis.

struct WheelJointSim {
    var linearImpulse = Vec2()
    var angularImpulse = Vec2()
    var spinImpulse: Float = 0
    var maxSpinTorque: Float = 0
    var spinSpeed: Float = 0
    var suspensionSpringImpulse: Float = 0
    var lowerSuspensionImpulse: Float = 0
    var upperSuspensionImpulse: Float = 0
    var lowerSuspensionLimit: Float = 0
    var upperSuspensionLimit: Float = 0
    var suspensionHertz: Float = 0
    var suspensionDampingRatio: Float = 0
    var steeringSpringImpulse: Float = 0
    var lowerSteeringImpulse: Float = 0
    var upperSteeringImpulse: Float = 0
    var lowerSteeringLimit: Float = 0
    var upperSteeringLimit: Float = 0
    var targetSteeringAngle: Float = 0
    var maxSteeringTorque: Float = 0
    var steeringHertz: Float = 0
    var steeringDampingRatio: Float = 0

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var frameA = Transform()
    var frameB = Transform()
    var deltaCenter = Vec3()
    var spinMass: Float = 0
    var suspensionMass: Float = 0
    var steeringMass: Float = 0
    var suspensionSoftness = Softness()
    var steeringSoftness = Softness()

    var enableSpinMotor = false
    var enableSuspensionSpring = false
    var enableSuspensionLimit = false
    var enableSteering = false
    var enableSteeringLimit = false
    var enableSteeringMotor = false

    init() {}
}

extension JointSim {
    var wheelJoint: WheelJointSim {
        get { wheel }
        set { wheel = newValue }
    }
}

extension World {
    @discardableResult
    public func createWheelJoint(_ def: WheelJointDef) -> JointID {
        let pair = createJoint(def.base, .wheelJoint)

        var sim = pair.sim
        var joint = WheelJointSim()
        joint.maxSpinTorque = def.maxSpinTorque
        joint.spinSpeed = def.spinSpeed
        joint.lowerSuspensionLimit = def.lowerSuspensionLimit
        joint.upperSuspensionLimit = def.upperSuspensionLimit
        joint.suspensionHertz = def.suspensionHertz
        joint.suspensionDampingRatio = def.suspensionDampingRatio
        joint.lowerSteeringLimit = def.lowerSteeringLimit
        joint.upperSteeringLimit = def.upperSteeringLimit
        joint.targetSteeringAngle = def.targetSteeringAngle
        joint.maxSteeringTorque = def.maxSteeringTorque
        joint.steeringHertz = def.steeringHertz
        joint.steeringDampingRatio = def.steeringDampingRatio
        joint.enableSpinMotor = def.enableSpinMotor
        joint.enableSuspensionSpring = def.enableSuspensionSpring
        joint.enableSuspensionLimit = def.enableSuspensionLimit
        joint.enableSteering = def.enableSteering
        joint.enableSteeringLimit = def.enableSteeringLimit
        sim.wheelJoint = joint
        writeJointSim(pair.jointId, sim)

        let j = joints[Int(pair.jointId)]
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: j.generation)
    }

    /// Steering axis shared by prepare, warm start, and solve.
    @inline(__always)
    private func wheelSteeringAxis(_ matrixA: Matrix3, _ matrixB: Matrix3) -> (Vec3, Float, Float) {
        // Steering param ib = cz_b, ia = cz_a, ja = -cy_a
        let cs = dot(matrixB.cz, matrixA.cz)
        let ss = -dot(matrixB.cz, matrixA.cy)
        var den = cs * cs + ss * ss
        den = den > 0 ? 1 / den : 0
        let axis = den * cross(matrixB.cz, (-cs) * matrixA.cy - ss * matrixA.cz)
        return (axis, cs, ss)
    }

    func prepareWheelJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .wheelJoint)

        let bodyA = bodies[Int(base.bodyIdA)]
        let bodyB = bodies[Int(base.bodyIdB)]

        b3Assert(
            bodyA.setIndex == SetType.awakeSet.rawValue || bodyB.setIndex == SetType.awakeSet.rawValue
        )

        let simA = solverSets[Int(bodyA.setIndex)].bodySims[Int(bodyA.localIndex)]
        let simB = solverSets[Int(bodyB.setIndex)].bodySims[Int(bodyB.localIndex)]

        base.invMassA = simA.invMass
        base.invMassB = simB.invMass
        base.invIA = simA.invInertiaWorld
        base.invIB = simB.invInertiaWorld

        let invInertiaSum = addMM(base.invIA, base.invIB)
        base.fixedRotation = det(invInertiaSum) < 1000 * Float.leastNormalMagnitude

        var joint = base.wheelJoint
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex

        // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.frameA.q = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.frameA.p = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.frameB.q = mulQuat(simB.transform.q, base.localFrameB.q)
        joint.frameB.p = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)

        // Compute the initial center delta. Incremental position updates are relative to this.
        joint.deltaCenter = subPos(simB.center, simA.center)

        let rA = joint.frameA.p
        let rB = joint.frameB.p

        let matrixA = makeMatrixFromQuat(joint.frameA.q)
        let matrixB = makeMatrixFromQuat(joint.frameB.q)

        do {
            let suspensionAxis = matrixA.cx
            let rAn = cross(rA, suspensionAxis)
            let rBn = cross(rB, suspensionAxis)

            let k = base.invMassA + base.invMassB + dot(rAn, mulMV(base.invIA, rAn))
                + dot(rBn, mulMV(base.invIB, rBn))
            joint.suspensionMass = k > 0 ? 1 / k : 0
        }

        joint.suspensionSoftness = makeSoft(
            joint.suspensionHertz, joint.suspensionDampingRatio, context.h
        )
        joint.steeringSoftness = makeSoft(joint.steeringHertz, joint.steeringDampingRatio, context.h)

        do {
            // Rotation axis is the z-axis of body A.
            let spinAxis = matrixB.cz
            let k = dot(spinAxis, mulMV(invInertiaSum, spinAxis))
            joint.spinMass = k > 0 ? 1 / k : 0
        }

        do {
            // Twist constraint around x-axis
            let (axis, _, _) = wheelSteeringAxis(matrixA, matrixB)
            let k = dot(axis, mulMV(invInertiaSum, axis))
            joint.steeringMass = k > 0 ? 1 / k : 0
        }

        if context.enableWarmStarting == false {
            joint.linearImpulse = Vec2()
            joint.angularImpulse = Vec2()
            joint.spinImpulse = 0
            joint.suspensionSpringImpulse = 0
            joint.lowerSuspensionImpulse = 0
            joint.upperSuspensionImpulse = 0
            joint.steeringSpringImpulse = 0
            joint.lowerSteeringImpulse = 0
            joint.upperSteeringImpulse = 0
        }

        base.wheelJoint = joint
    }

    func warmStartWheelJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .wheelJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        let joint = base.wheelJoint
        var stateA = solverState(joint.indexA)
        var stateB = solverState(joint.indexB)

        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

        let d = ((stateB.deltaPosition - stateA.deltaPosition) + joint.deltaCenter) + (rB - rA)

        let quatA = mulQuat(stateA.deltaRotation, joint.frameA.q)
        var quatB = mulQuat(stateB.deltaRotation, joint.frameB.q)
        if dotQuat(quatA, quatB) < 0 {
            quatB = negateQuat(quatB)
        }

        let matrixA = makeMatrixFromQuat(quatA)
        let matrixB = makeMatrixFromQuat(quatB)

        let sAx = cross(d + rA, matrixA.cx)
        let sBx = cross(rB, matrixA.cx)
        let sAy = cross(d + rA, matrixA.cy)
        let sBy = cross(rB, matrixA.cy)
        let sAz = cross(d + rA, matrixA.cz)
        let sBz = cross(rB, matrixA.cz)

        let suspensionImpulse =
            joint.suspensionSpringImpulse + joint.lowerSuspensionImpulse - joint.upperSuspensionImpulse

        let linearImpulseY = joint.linearImpulse.x
        let linearImpulseZ = joint.linearImpulse.y
        let angularImpulseX = joint.angularImpulse.x
        let angularImpulseY = joint.angularImpulse.y

        let linearImpulse = blend3(
            suspensionImpulse, matrixA.cx, linearImpulseY, matrixA.cy, linearImpulseZ, matrixA.cz
        )
        let angularImpulseA = blend3(suspensionImpulse, sAx, linearImpulseY, sAy, linearImpulseZ, sAz)
        let angularImpulseB = blend3(suspensionImpulse, sBx, linearImpulseY, sBy, linearImpulseZ, sBz)
        var angularImpulse = joint.spinImpulse * matrixA.cz

        let spinAxis = matrixB.cz

        if joint.enableSteering {
            // Twist constraint around x-axis
            let (steeringAxis, _, _) = wheelSteeringAxis(matrixA, matrixB)

            let perpAxis = cross(spinAxis, matrixA.cx)
            let steeringImpulse =
                joint.steeringSpringImpulse + joint.lowerSteeringImpulse - joint.upperSteeringImpulse
            angularImpulse = blend3(
                angularImpulseX, perpAxis, joint.spinImpulse, spinAxis, steeringImpulse, steeringAxis
            )
        } else {
            let relQ = invMulQuat(quatA, quatB)
            let perpAxisX = 0.5 * rotateVector(
                quatA, relQ.s * Vec3.axisX + cross(relQ.v, Vec3.axisX))
            let perpAxisY = 0.5 * rotateVector(
                quatA, relQ.s * Vec3.axisY + cross(relQ.v, Vec3.axisY))
            angularImpulse = angularImpulse
                + blend3(angularImpulseX, perpAxisX, angularImpulseY, perpAxisY, joint.spinImpulse, spinAxis)
        }

        if (stateA.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateA.linearVelocity = stateA.linearVelocity - mA * linearImpulse
            stateA.angularVelocity = stateA.angularVelocity
                - mulMV(iA, angularImpulseA + angularImpulse)
        }

        if (stateB.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateB.linearVelocity = stateB.linearVelocity + mB * linearImpulse
            stateB.angularVelocity = stateB.angularVelocity
                + mulMV(iB, angularImpulseB + angularImpulse)
        }

        storeSolverState(joint.indexA, stateA)
        storeSolverState(joint.indexB, stateB)
    }

    func solveWheelJoint(_ base: inout JointSim, _ context: StepContext, _ useBias: Bool) {
        b3Assert(base.type == .wheelJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        var joint = base.wheelJoint
        var stateA = solverState(joint.indexA)
        var stateB = solverState(joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let fixedRotation = base.fixedRotation

        // current anchors
        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

        let quatA = mulQuat(stateA.deltaRotation, joint.frameA.q)
        var quatB = mulQuat(stateB.deltaRotation, joint.frameB.q)

        if dotQuat(quatA, quatB) < 0 {
            quatB = negateQuat(quatB)
        }

        let relQ = invMulQuat(quatA, quatB)
        let matrixA = makeMatrixFromQuat(quatA)
        let matrixB = makeMatrixFromQuat(quatB)

        let d = ((stateB.deltaPosition - stateA.deltaPosition) + joint.deltaCenter) + (rB - rA)
        let sAx = cross(d + rA, matrixA.cx)
        let sBx = cross(rB, matrixA.cx)
        let sAy = cross(d + rA, matrixA.cy)
        let sBy = cross(rB, matrixA.cy)
        let sAz = cross(d + rA, matrixA.cz)
        let sBz = cross(rB, matrixA.cz)

        let translation = dot(matrixA.cx, d)

        // Steering param ib = cz_b, ia = cz_a, ja = -cy_a
        let (steeringAxis, cs, ss) = wheelSteeringAxis(matrixA, matrixB)

        // motor constraint
        if joint.enableSpinMotor && fixedRotation == false {
            let spinAxis = matrixB.cz
            let cdot = dot(wB - wA, spinAxis) - joint.spinSpeed
            var impulse = -joint.spinMass * cdot
            let oldImpulse = joint.spinImpulse
            let maxImpulse = context.h * joint.maxSpinTorque
            joint.spinImpulse = clampFloat(
                joint.spinImpulse + impulse, -maxImpulse, maxImpulse
            )
            impulse = joint.spinImpulse - oldImpulse

            wA = wA - mulMV(iA, impulse * spinAxis)
            wB = wB + mulMV(iB, impulse * spinAxis)
        }

        // suspension
        if joint.enableSuspensionSpring {
            // This is a real spring and should be applied even during relax
            let c = translation
            let bias = joint.suspensionSoftness.biasRate * c
            let massScale = joint.suspensionSoftness.massScale
            let impulseScale = joint.suspensionSoftness.impulseScale

            let cdot = dot(matrixA.cx, vB - vA) + dot(sBx, wB) - dot(sAx, wA)
            let impulse = -massScale * joint.suspensionMass * (cdot + bias)
                - impulseScale * joint.suspensionSpringImpulse
            joint.suspensionSpringImpulse += impulse

            let linearImpulse = impulse * matrixA.cx
            let angularImpulseA = impulse * sAx
            let angularImpulseB = impulse * sBx

            vA = vA - mA * linearImpulse
            wA = wA - mulMV(iA, angularImpulseA)
            vB = vB + mB * linearImpulse
            wB = wB + mulMV(iB, angularImpulseB)
        }

        // steering
        if joint.enableSteering && fixedRotation == false {
            let steeringAngle = atan2Approx(ss, cs)

            do {
                // This is a real spring and should be applied even during relax
                let c = steeringAngle - joint.targetSteeringAngle
                let bias = joint.steeringSoftness.biasRate * c
                let massScale = joint.steeringSoftness.massScale
                let impulseScale = joint.steeringSoftness.impulseScale

                let cdot = dot(steeringAxis, wB - wA)
                let oldImpulse = joint.steeringSpringImpulse
                var impulse = -massScale * joint.steeringMass * (cdot + bias)
                    - impulseScale * oldImpulse
                let maxImpulse = context.h * joint.maxSteeringTorque
                joint.steeringSpringImpulse = clampFloat(
                    oldImpulse + impulse, -maxImpulse, maxImpulse
                )
                impulse = joint.steeringSpringImpulse - oldImpulse

                wA = wA - mulMV(iA, impulse * steeringAxis)
                wB = wB + mulMV(iB, impulse * steeringAxis)
            }

            if joint.enableSteeringLimit {
                // Lower limit
                do {
                    let c = steeringAngle - joint.lowerSteeringLimit
                    var bias: Float = 0
                    var massScale: Float = 1
                    var impulseScale: Float = 0

                    if c > 0 {
                        // speculation
                        bias = c * context.inv_h
                    } else if useBias {
                        bias = base.constraintSoftness.biasRate * c
                        massScale = base.constraintSoftness.massScale
                        impulseScale = base.constraintSoftness.impulseScale
                    }

                    let cdot = dot(steeringAxis, wB - wA)
                    let oldImpulse = joint.lowerSteeringImpulse
                    var impulse = -massScale * joint.steeringMass * (cdot + bias)
                        - impulseScale * oldImpulse
                    joint.lowerSteeringImpulse = maxFloat(oldImpulse + impulse, 0)
                    impulse = joint.lowerSteeringImpulse - oldImpulse

                    wA = wA - mulMV(iA, impulse * steeringAxis)
                    wB = wB + mulMV(iB, impulse * steeringAxis)
                }

                // Upper limit
                // Note: signs are flipped to keep c positive when the constraint is satisfied.
                do {
                    // sign flipped
                    let c = joint.upperSteeringLimit - steeringAngle
                    var bias: Float = 0
                    var massScale: Float = 1
                    var impulseScale: Float = 0

                    if c > 0 {
                        // speculation
                        bias = c * context.inv_h
                    } else if useBias {
                        bias = base.constraintSoftness.biasRate * c
                        massScale = base.constraintSoftness.massScale
                        impulseScale = base.constraintSoftness.impulseScale
                    }

                    // sign flipped on cdot
                    let cdot = dot(steeringAxis, wA - wB)
                    let oldImpulse = joint.upperSteeringImpulse
                    var impulse = -massScale * joint.steeringMass * (cdot + bias)
                        - impulseScale * oldImpulse
                    joint.upperSteeringImpulse = maxFloat(oldImpulse + impulse, 0)
                    impulse = joint.upperSteeringImpulse - oldImpulse

                    // sign flipped on applied impulse
                    wA = wA + mulMV(iA, impulse * steeringAxis)
                    wB = wB - mulMV(iB, impulse * steeringAxis)
                }
            }
        }

        if joint.enableSuspensionLimit {
            // Lower limit
            do {
                let c = translation - joint.lowerSuspensionLimit
                var bias: Float = 0
                var massScale: Float = 1
                var impulseScale: Float = 0

                if c > 0 {
                    // speculation
                    bias = c * context.inv_h
                } else if useBias {
                    bias = base.constraintSoftness.biasRate * c
                    massScale = base.constraintSoftness.massScale
                    impulseScale = base.constraintSoftness.impulseScale
                }

                let cdot = dot(matrixA.cx, vB - vA) + dot(sBx, wB) - dot(sAx, wA)
                var impulse = -massScale * joint.suspensionMass * (cdot + bias)
                    - impulseScale * joint.lowerSuspensionImpulse
                let oldImpulse = joint.lowerSuspensionImpulse
                joint.lowerSuspensionImpulse = maxFloat(oldImpulse + impulse, 0)
                impulse = joint.lowerSuspensionImpulse - oldImpulse

                let linearImpulse = impulse * matrixA.cx
                let angularImpulseA = impulse * sAx
                let angularImpulseB = impulse * sBx

                vA = vA - mA * linearImpulse
                wA = wA - mulMV(iA, angularImpulseA)
                vB = vB + mB * linearImpulse
                wB = wB + mulMV(iB, angularImpulseB)
            }

            // Upper limit
            // Note: signs are flipped to keep c positive when the constraint is satisfied.
            do {
                // sign flipped
                let c = joint.upperSuspensionLimit - translation
                var bias: Float = 0
                var massScale: Float = 1
                var impulseScale: Float = 0

                if c > 0 {
                    // speculation
                    bias = c * context.inv_h
                } else if useBias {
                    bias = base.constraintSoftness.biasRate * c
                    massScale = base.constraintSoftness.massScale
                    impulseScale = base.constraintSoftness.impulseScale
                }

                // sign flipped on cdot
                let cdot = dot(matrixA.cx, vA - vB) + dot(sAx, wA) - dot(sBx, wB)
                var impulse = -massScale * joint.suspensionMass * (cdot + bias)
                    - impulseScale * joint.upperSuspensionImpulse
                let oldImpulse = joint.upperSuspensionImpulse
                joint.upperSuspensionImpulse = maxFloat(oldImpulse + impulse, 0)
                impulse = joint.upperSuspensionImpulse - oldImpulse

                let linearImpulse = impulse * matrixA.cx
                let angularImpulseA = impulse * sAx
                let angularImpulseB = impulse * sBx

                // sign flipped on applied impulse
                vA = vA + mA * linearImpulse
                wA = wA + mulMV(iA, angularImpulseA)
                vB = vB - mB * linearImpulse
                wB = wB - mulMV(iB, angularImpulseB)
            }
        }

        // Collinearity constraint
        if fixedRotation == false {
            if joint.enableSteering {
                var bias: Float = 0
                var massScale: Float = 1
                var impulseScale: Float = 0
                if useBias {
                    let c = dot(matrixA.cx, matrixB.cz)

                    bias = base.constraintSoftness.biasRate * c
                    massScale = base.constraintSoftness.massScale
                    impulseScale = base.constraintSoftness.impulseScale
                }

                let u = cross(matrixB.cz, matrixA.cx)
                let cdot = dot(wB - wA, u)

                let invInertiaSum = addMM(iA, iB)
                let k = dot(u, mulMV(invInertiaSum, u))
                let perpMass: Float = k > 0 ? 1 / k : 0

                let deltaImpulse = -massScale * perpMass * (cdot + bias)
                    - impulseScale * joint.angularImpulse.x
                joint.angularImpulse.x += deltaImpulse

                wA = wA - deltaImpulse * mulMV(iA, u)
                wB = wB + deltaImpulse * mulMV(iB, u)
            } else {
                var bias = Vec2()
                var massScale: Float = 1
                var impulseScale: Float = 0

                if useBias {
                    let c = Vec2(relQ.v.x, relQ.v.y)
                    bias = Vec2(
                        base.constraintSoftness.biasRate * c.x,
                        base.constraintSoftness.biasRate * c.y
                    )
                    massScale = base.constraintSoftness.massScale
                    impulseScale = base.constraintSoftness.impulseScale
                }

                // Collinearity constraint as 2-by-2
                let perpAxisX = 0.5 * rotateVector(
                    quatA, relQ.s * Vec3.axisX + cross(relQ.v, Vec3.axisX))
                let perpAxisY = 0.5 * rotateVector(
                    quatA, relQ.s * Vec3.axisY + cross(relQ.v, Vec3.axisY))

                let invInertiaSum = addMM(iA, iB)
                let kxx = dot(perpAxisX, mulMV(invInertiaSum, perpAxisX))
                let kyy = dot(perpAxisY, mulMV(invInertiaSum, perpAxisY))
                let kxy = dot(perpAxisX, mulMV(invInertiaSum, perpAxisY))

                let k = Matrix2(Vec2(kxx, kxy), Vec2(kxy, kyy))

                let wRel = wB - wA
                let cdot = Vec2(dot(wRel, perpAxisX), dot(wRel, perpAxisY))
                let oldImpulse = joint.angularImpulse
                let sol = solve2(k, Vec2(cdot.x + bias.x, cdot.y + bias.y))
                let deltaImpulse = Vec2(
                    -massScale * sol.x - impulseScale * oldImpulse.x,
                    -massScale * sol.y - impulseScale * oldImpulse.y
                )
                joint.angularImpulse = Vec2(
                    oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y
                )

                let angularImpulse = blend2(deltaImpulse.x, perpAxisX, deltaImpulse.y, perpAxisY)
                wA = wA - mulMV(iA, angularImpulse)
                wB = wB + mulMV(iB, angularImpulse)
            }
        }

        // Solve point-to-line constraint
        do {
            let perpY = matrixA.cy
            let perpZ = matrixA.cz

            var bias = Vec2()
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias {
                let c = Vec2(dot(perpY, d), dot(perpZ, d))
                bias = Vec2(
                    base.constraintSoftness.biasRate * c.x, base.constraintSoftness.biasRate * c.y
                )
                massScale = base.constraintSoftness.massScale
                impulseScale = base.constraintSoftness.impulseScale
            }

            let vRel = (vB + cross(wB, rB)) - vA - cross(wA, rA + d)
            let cdot = Vec2(dot(perpY, vRel), dot(perpZ, vRel))

            let kyy = mA + mB + dot(sAy, mulMV(iA, sAy)) + dot(sBy, mulMV(iB, sBy))
            let kyz = dot(sAy, mulMV(iA, sAz)) + dot(sBy, mulMV(iB, sBz))
            let kzz = mA + mB + dot(sAz, mulMV(iA, sAz)) + dot(sBz, mulMV(iB, sBz))

            let k = Matrix2(Vec2(kyy, kyz), Vec2(kyz, kzz))

            let oldImpulse = joint.linearImpulse
            let sol = solve2(k, Vec2(cdot.x + bias.x, cdot.y + bias.y))
            let deltaImpulse = Vec2(
                -massScale * sol.x - impulseScale * oldImpulse.x,
                -massScale * sol.y - impulseScale * oldImpulse.y
            )
            joint.linearImpulse = Vec2(
                oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y
            )

            let linearImpulse = blend2(deltaImpulse.x, perpY, deltaImpulse.y, perpZ)

            vA = vA - mA * linearImpulse
            wA = wA - mulMV(iA, blend2(deltaImpulse.x, sAy, deltaImpulse.y, sAz))
            vB = vB + mB * linearImpulse
            wB = wB + mulMV(iB, blend2(deltaImpulse.x, sBy, deltaImpulse.y, sBz))
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

        base.wheelJoint = joint
    }
}
