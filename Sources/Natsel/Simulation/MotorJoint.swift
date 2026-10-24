

// Motor joint (motor_joint.c). Controls relative velocity and optionally a spring pose.

struct MotorJointSim: BitwiseCopyable {
    var linearVelocity = Vec3()
    var angularVelocity = Vec3()
    var maxVelocityForce: Float = 0
    var maxVelocityTorque: Float = 0
    var linearHertz: Float = 0
    var linearDampingRatio: Float = 0
    var maxSpringForce: Float = 0
    var angularHertz: Float = 0
    var angularDampingRatio: Float = 0
    var maxSpringTorque: Float = 0

    var linearVelocityImpulse = Vec3()
    var angularVelocityImpulse = Vec3()
    var linearSpringImpulse = Vec3()
    var angularSpringImpulse = Vec3()

    var linearSpring = Softness()
    var angularSpring = Softness()

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var frameA = Transform()
    var frameB = Transform()
    var deltaCenter = Vec3()
    var angularMass = Matrix3()

    init() {}
}

extension JointSim {
    var motorJoint: MotorJointSim {
        get { motor }
        set { motor = newValue }
    }
}

extension World {
    @discardableResult
    public func createMotorJoint(_ def: MotorJointDef) -> JointID {
        let pair = createJoint(def.base, .motorJoint)

        var sim = pair.sim
        var joint = MotorJointSim()
        joint.linearVelocity = def.linearVelocity
        joint.angularVelocity = def.angularVelocity
        joint.maxVelocityForce = def.maxVelocityForce
        joint.maxVelocityTorque = def.maxVelocityTorque
        joint.linearHertz = def.linearHertz
        joint.linearDampingRatio = def.linearDampingRatio
        joint.maxSpringForce = def.maxSpringForce
        joint.angularHertz = def.angularHertz
        joint.angularDampingRatio = def.angularDampingRatio
        joint.maxSpringTorque = def.maxSpringTorque
        sim.motorJoint = joint
        writeJointSim(pair.jointId, sim)

        let j = joints[Int(pair.jointId)]
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: j.generation)
    }

    func prepareMotorJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .motorJoint)

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

        var joint = base.motorJoint
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex

        // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.frameA.q = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.frameA.p = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.frameB.q = mulQuat(simB.transform.q, base.localFrameB.q)
        joint.frameB.p = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)

        // Compute the initial center delta. Incremental position updates are relative to this.
        joint.deltaCenter = subPos(simB.center, simA.center)

        joint.linearSpring = makeSoft(joint.linearHertz, joint.linearDampingRatio, context.h)
        joint.angularSpring = makeSoft(joint.angularHertz, joint.angularDampingRatio, context.h)

        joint.angularMass = invertMatrix(invInertiaSum)

        if context.enableWarmStarting == false {
            joint.linearVelocityImpulse = Vec3()
            joint.angularVelocityImpulse = Vec3()
            joint.linearSpringImpulse = Vec3()
            joint.angularSpringImpulse = Vec3()
        }

        base.motorJoint = joint
    }

    func warmStartMotorJoint(_ base: inout JointSim, _ context: StepContext, _ states: UnsafeMutableBufferPointer<BodyState>) {
        b3Assert(base.type == .motorJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        let joint = base.motorJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

        let linearImpulse = joint.linearVelocityImpulse + joint.linearSpringImpulse
        let angularImpulse = joint.angularVelocityImpulse + joint.angularSpringImpulse

        // The C warm start writes the velocities unconditionally, so kinematic bodies are modified.
        stateA.linearVelocity = stateA.linearVelocity - mA * linearImpulse
        stateA.angularVelocity = stateA.angularVelocity
            - mulMV(iA, cross(rA, linearImpulse) + angularImpulse)
        stateB.linearVelocity = stateB.linearVelocity + mB * linearImpulse
        stateB.angularVelocity = stateB.angularVelocity
            + mulMV(iB, cross(rB, linearImpulse) + angularImpulse)

        if joint.indexA >= 0 {
            states[Int(joint.indexA)] = stateA
        }
        if joint.indexB >= 0 {
            states[Int(joint.indexB)] = stateB
        }
    }

    func solveMotorJoint(_ base: inout JointSim, _ context: StepContext, _ states: UnsafeMutableBufferPointer<BodyState>) {
        b3Assert(base.type == .motorJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        var joint = base.motorJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let quatA = mulQuat(stateA.deltaRotation, joint.frameA.q)
        var quatB = mulQuat(stateB.deltaRotation, joint.frameB.q)

        if dotQuat(quatA, quatB) < 0 {
            // this keeps the rotation angle in the range [-pi, pi]
            quatB = negateQuat(quatB)
        }

        let relQ = invMulQuat(quatA, quatB)

        // angular spring
        if joint.maxSpringTorque > 0 && joint.angularHertz > 0 {
            let targetQuat = Quat.identity
            let deltaRotation = deltaQuatToRotation(relQ, targetQuat)
            let c = -(rotateVector(quatA, deltaRotation))

            let bias = joint.angularSpring.biasRate * c
            let massScale = joint.angularSpring.massScale
            let impulseScale = joint.angularSpring.impulseScale

            let cdot = wB - wA

            let maxImpulse = context.h * joint.maxSpringTorque
            let oldImpulse = joint.angularSpringImpulse
            var impulse = (-massScale) * mulMV(joint.angularMass, cdot + bias)
                - impulseScale * oldImpulse
            joint.angularSpringImpulse = oldImpulse + impulse
            if lengthSquared(joint.angularSpringImpulse) > maxImpulse * maxImpulse {
                joint.angularSpringImpulse = maxImpulse * normalize(joint.angularSpringImpulse)
            }
            impulse = joint.angularSpringImpulse - oldImpulse

            wA = wA - mulMV(iA, impulse)
            wB = wB + mulMV(iB, impulse)
        }

        // angular velocity
        if joint.maxVelocityTorque > 0 {
            let cdot = (wB - wA) - joint.angularVelocity
            var impulse = -(mulMV(joint.angularMass, cdot))

            let maxImpulse = context.h * joint.maxVelocityTorque
            let oldImpulse = joint.angularVelocityImpulse
            joint.angularVelocityImpulse = oldImpulse + impulse
            if lengthSquared(joint.angularVelocityImpulse) > maxImpulse * maxImpulse {
                joint.angularVelocityImpulse = maxImpulse * normalize(joint.angularVelocityImpulse)
            }
            impulse = joint.angularVelocityImpulse - oldImpulse

            wA = wA - mulMV(iA, impulse)
            wB = wB + mulMV(iB, impulse)
        }

        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

        // linear spring
        if joint.maxSpringForce > 0 && joint.linearHertz > 0 {
            let dcA = stateA.deltaPosition
            let dcB = stateB.deltaPosition
            let c = ((dcB - dcA) + (rB - rA)) + joint.deltaCenter

            let bias = joint.linearSpring.biasRate * c
            let massScale = joint.linearSpring.massScale
            let impulseScale = joint.linearSpring.impulseScale

            let cdot = (vB + cross(wB, rB)) - (vA + cross(wA, rA))

            let b = solvePointConstraint(
                mA, mB, base.invIA, base.invIB, rA, rB, cdot + bias
            )

            let oldImpulse = joint.linearSpringImpulse
            var impulse = (-massScale) * b - impulseScale * oldImpulse
            let maxImpulse = context.h * joint.maxSpringForce
            joint.linearSpringImpulse = joint.linearSpringImpulse + impulse

            if lengthSquared(joint.linearSpringImpulse) > maxImpulse * maxImpulse {
                joint.linearSpringImpulse = maxImpulse * normalize(joint.linearSpringImpulse)
            }

            impulse = joint.linearSpringImpulse - oldImpulse

            vA = vA - mA * impulse
            wA = wA - mulMV(iA, cross(rA, impulse))
            vB = vB + mB * impulse
            wB = wB + mulMV(iB, cross(rB, impulse))
        }

        // linear velocity
        if joint.maxVelocityForce > 0 {
            var cdot = (vB + cross(wB, rB)) - (vA + cross(wA, rA))
            cdot = cdot - joint.linearVelocity

            let b = solvePointConstraint(mA, mB, base.invIA, base.invIB, rA, rB, cdot)
            var impulse = -(b)

            let oldImpulse = joint.linearVelocityImpulse
            let maxImpulse = context.h * joint.maxVelocityForce
            joint.linearVelocityImpulse = joint.linearVelocityImpulse + impulse

            if lengthSquared(joint.linearVelocityImpulse) > maxImpulse * maxImpulse {
                joint.linearVelocityImpulse = maxImpulse * normalize(joint.linearVelocityImpulse)
            }

            impulse = joint.linearVelocityImpulse - oldImpulse

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

        base.motorJoint = joint
    }
}
