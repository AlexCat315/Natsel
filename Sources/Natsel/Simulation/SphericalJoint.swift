

// Spherical joint (spherical_joint.c).

struct SphericalJointSim: BitwiseCopyable {
    var linearImpulse = Vec3()
    var springImpulse = Vec3()
    var motorImpulse = Vec3()
    var lowerTwistImpulse: Float = 0
    var upperTwistImpulse: Float = 0
    var swingImpulse: Float = 0
    var hertz: Float = 0
    var dampingRatio: Float = 0
    var maxMotorTorque: Float = 0
    var motorVelocity = Vec3()
    var lowerTwistAngle: Float = 0
    var upperTwistAngle: Float = 0
    var coneAngle: Float = 0
    var targetRotation = Quat()

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var frameA = Transform()
    var frameB = Transform()
    var deltaCenter = Vec3()
    var swingAxis = Vec3()
    var twistJacobian = Vec3()

    var rotationMass = Matrix3()
    var swingMass: Float = 0
    var twistMass: Float = 0
    var springSoftness = Softness()

    var enableSpring = false
    var enableMotor = false
    var enableConeLimit = false
    var enableTwistLimit = false

    init() {}
}

extension JointSim {
    var sphericalJoint: SphericalJointSim {
        get { spherical }
        set { spherical = newValue }
    }
}

extension World {
    @discardableResult
    public func createSphericalJoint(_ def: SphericalJointDef) -> JointID {
        let pair = createJoint(def.base, .sphericalJoint)

        var sim = pair.sim
        var joint = SphericalJointSim()
        joint.hertz = def.hertz
        joint.dampingRatio = def.dampingRatio
        joint.maxMotorTorque = def.maxMotorTorque
        joint.motorVelocity = def.motorVelocity
        joint.lowerTwistAngle = def.lowerTwistAngle
        joint.upperTwistAngle = def.upperTwistAngle
        joint.coneAngle = def.coneAngle
        joint.targetRotation = def.targetRotation
        joint.enableSpring = def.enableSpring
        joint.enableMotor = def.enableMotor
        joint.enableConeLimit = def.enableConeLimit
        joint.enableTwistLimit = def.enableTwistLimit
        sim.sphericalJoint = joint
        writeJointSim(pair.jointId, sim)

        let j = joints[Int(pair.jointId)]
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: j.generation)
    }

    func prepareSphericalJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .sphericalJoint)

        let bodyA = jointPrepareBody(base.bodyIdA)
        let bodyB = jointPrepareBody(base.bodyIdB)

        b3Assert(
            bodyA.setIndex == SetType.awakeSet.rawValue || bodyB.setIndex == SetType.awakeSet.rawValue
        )

        let simA = jointPrepareSim(bodyA)
        let simB = jointPrepareSim(bodyB)

        base.invMassA = simA.invMass
        base.invMassB = simB.invMass
        base.invIA = simA.invInertiaWorld
        base.invIB = simB.invInertiaWorld

        let invInertiaSum = addMM(base.invIA, base.invIB)
        base.fixedRotation = det(invInertiaSum) < 1000 * Float.leastNormalMagnitude

        var joint = base.sphericalJoint
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex

        // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.frameA.q = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.frameA.p = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.frameB.q = mulQuat(simB.transform.q, base.localFrameB.q)
        joint.frameB.p = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)

        joint.deltaCenter = subPos(simB.center, simA.center)

        // Cone axis is the z-axis of body A.
        let coneAxis = rotateVector(joint.frameA.q, Vec3.axisZ)

        // Twist axis is the z-axis of body B.
        let twistAxis = rotateVector(joint.frameB.q, Vec3.axisZ)

        if joint.enableConeLimit {
            // Swing axis may be zero
            let swingAxis = normalize(cross(coneAxis, twistAxis))
            let k = dot(swingAxis, mulMV(invInertiaSum, swingAxis))
            joint.swingMass = k > 0 ? 1 / k : 0
            joint.swingAxis = swingAxis
        }

        if joint.enableTwistLimit {
            let relQ = invMulQuat(joint.frameA.q, joint.frameB.q)
            let tanThetaOver2 = (
                (relQ.v.x * relQ.v.x + relQ.v.y * relQ.v.y)
                    / (relQ.v.z * relQ.v.z + relQ.s * relQ.s)
            ).squareRoot()

            let swingAxis = normalize(cross(coneAxis, twistAxis))
            let perpAxis = cross(swingAxis, coneAxis)
            let twistJacobian = mulAdd(coneAxis, tanThetaOver2, perpAxis)
            let k = dot(twistJacobian, mulMV(invInertiaSum, twistJacobian))
            joint.twistMass = k > 0 ? 1 / k : 0
            joint.twistJacobian = twistJacobian
        }

        if base.fixedRotation == false {
            joint.rotationMass = invertMatrix(invInertiaSum)
        } else {
            joint.rotationMass = Matrix3()
        }

        joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

        if context.enableWarmStarting == false {
            joint.linearImpulse = Vec3()
            joint.motorImpulse = Vec3()
            joint.springImpulse = Vec3()
            joint.swingImpulse = 0
            joint.lowerTwistImpulse = 0
            joint.upperTwistImpulse = 0
        }

        base.sphericalJoint = joint
    }

    func warmStartSphericalJoint(_ base: inout JointSim, _ context: StepContext, _ states: UnsafeMutableBufferPointer<BodyState>) {
        b3Assert(base.type == .sphericalJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        let joint = base.sphericalJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

        var angularImpulse = joint.springImpulse + joint.motorImpulse
        angularImpulse = angularImpulse - joint.swingImpulse * joint.swingAxis
        angularImpulse = mulAdd(
            angularImpulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, joint.twistJacobian
        )

        vA = vA - mA * joint.linearImpulse
        wA = wA - mulMV(iA, cross(rA, joint.linearImpulse) + angularImpulse)

        vB = vB + mB * joint.linearImpulse
        wB = wB + mulMV(iB, cross(rB, joint.linearImpulse) + angularImpulse)

        if (stateA.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateA.linearVelocity = vA
            stateA.angularVelocity = wA
        }

        if (stateB.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateB.linearVelocity = vB
            stateB.angularVelocity = wB
        }

        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
    }

    func solveSphericalJoint(_ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        var joint = base.sphericalJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let fixedRotation = base.fixedRotation
        let quatA = mulQuat(stateA.deltaRotation, joint.frameA.q)
        let quatB = mulQuat(stateB.deltaRotation, joint.frameB.q)

        let relQ = invMulQuat(quatA, quatB)

        if fixedRotation == false
            && (joint.enableSpring || joint.enableMotor || joint.enableTwistLimit || joint.enableConeLimit)
        {
            let invInertiaSum = addMM(iA, iB)
            var cdot = wB - wA
            var angularImpulse = Vec3()

            // Solve spring
            if joint.enableSpring {
                // Rotation constraint error
                let deltaRotation = deltaQuatToRotation(relQ, joint.targetRotation)
                let c = -(rotateVector(quatA, deltaRotation))

                let bias = joint.springSoftness.biasRate * c
                let massScale = joint.springSoftness.massScale
                let impulseScale = joint.springSoftness.impulseScale

                let impulse = (-massScale) * mulMV(joint.rotationMass, cdot + bias)
                    - impulseScale * joint.springImpulse
                joint.springImpulse = joint.springImpulse + impulse

                cdot = cdot + mulMV(invInertiaSum, impulse)
                angularImpulse = angularImpulse + impulse
            }

            if joint.enableMotor {
                var lambda = -(mulMV(joint.rotationMass, cdot - joint.motorVelocity))
                var newImpulse = joint.motorImpulse + lambda
                let length = length(newImpulse)
                let maxImpulse = joint.maxMotorTorque * context.h
                if length > maxImpulse {
                    newImpulse = (maxImpulse / length) * newImpulse
                }

                lambda = newImpulse - joint.motorImpulse
                joint.motorImpulse = newImpulse

                cdot = cdot + mulMV(invInertiaSum, lambda)
                angularImpulse = angularImpulse + lambda
            }

            if joint.enableTwistLimit {
                let twistAngle = getTwistAngle(relQ)

                let twistJacobian = joint.twistJacobian
                let twistResponse = mulMV(invInertiaSum, twistJacobian)

                // Lower limit
                do {
                    let c = twistAngle - joint.lowerTwistAngle
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

                    let twistCdot = dot(cdot, twistJacobian)
                    let oldImpulse = joint.lowerTwistImpulse
                    let deltaImpulse = maxFloat(
                        -massScale * joint.twistMass * (twistCdot + bias) - impulseScale * oldImpulse,
                        -oldImpulse
                    )
                    joint.lowerTwistImpulse = oldImpulse + deltaImpulse

                    cdot = mulAdd(cdot, deltaImpulse, twistResponse)
                    angularImpulse = mulAdd(angularImpulse, deltaImpulse, twistJacobian)
                }

                // Upper limit
                do {
                    let c = joint.upperTwistAngle - twistAngle
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

                    // sign flipped on Cdot
                    let twistCdot = -dot(cdot, twistJacobian)
                    let oldImpulse = joint.upperTwistImpulse
                    let deltaImpulse = maxFloat(
                        -massScale * joint.twistMass * (twistCdot + bias) - impulseScale * oldImpulse,
                        -oldImpulse
                    )
                    joint.upperTwistImpulse = oldImpulse + deltaImpulse

                    // sign flipped on applied impulse
                    cdot = cdot - deltaImpulse * twistResponse
                    angularImpulse = angularImpulse - deltaImpulse * twistJacobian
                }
            }

            if joint.enableConeLimit {
                let swingAngle = getSwingAngle(relQ)

                let swingAxis = joint.swingAxis

                let c = joint.coneAngle - swingAngle
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

                // sign flipped on Cdot
                let swingCdot = -dot(cdot, swingAxis)
                let oldImpulse = joint.swingImpulse
                let deltaImpulse = maxFloat(
                    -massScale * joint.swingMass * (swingCdot + bias) - impulseScale * oldImpulse,
                    -oldImpulse
                )
                joint.swingImpulse = oldImpulse + deltaImpulse

                // sign flipped on applied impulse
                angularImpulse = angularImpulse - deltaImpulse * swingAxis
            }

            wA = wA - mulMV(iA, angularImpulse)
            wB = wB + mulMV(iB, angularImpulse)
        }

        // Solve point-to-point constraint
        do {
            let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
            let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

            let cdot = (vB + cross(wB, rB)) - vA - cross(wA, rA)

            var bias = Vec3()
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias {
                let dcA = stateA.deltaPosition
                let dcB = stateB.deltaPosition

                let separation = ((dcB - dcA) + (rB - rA)) + joint.deltaCenter

                bias = base.constraintSoftness.biasRate * separation
                massScale = base.constraintSoftness.massScale
                impulseScale = base.constraintSoftness.impulseScale
            }

            let b = solvePointConstraint(
                mA, mB, base.invIA, base.invIB, rA, rB, cdot + bias
            )

            let impulse = (-massScale) * b - impulseScale * joint.linearImpulse
            joint.linearImpulse = joint.linearImpulse + impulse

            vA = vA - mA * impulse
            wA = wA - mulMV(iA, cross(rA, impulse))
            vB = vB + mB * impulse
            wB = wB + mulMV(iB, cross(rB, impulse))
        }

        if (stateA.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateA.linearVelocity = vA
            stateA.angularVelocity = wA
        }

        if (stateB.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateB.linearVelocity = vB
            stateB.angularVelocity = wB
        }

        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)

        base.sphericalJoint = joint
    }
}
