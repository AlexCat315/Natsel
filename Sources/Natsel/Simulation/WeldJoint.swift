

// Weld joint.

struct WeldJointSim: BitwiseCopyable {
    var linearHertz: Float = 0
    var linearDampingRatio: Float = 0
    var angularHertz: Float = 0
    var angularDampingRatio: Float = 0

    var linearSpring = Softness()
    var angularSpring = Softness()
    var linearImpulse = Vec3()
    var angularImpulse = Vec3()

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var frameA = Transform()
    var frameB = Transform()
    var deltaCenter = Vec3()

    var angularMass = Matrix3()

    init() {}
}

extension JointSim {
    var weldJoint: WeldJointSim {
        get { weld }
        set { weld = newValue }
    }
}

extension World {
    @discardableResult
    public func createWeldJoint(_ def: WeldJointDef) -> JointID {
        let pair = createJoint(def.base, .weldJoint)

        var sim = pair.sim
        var joint = WeldJointSim()
        joint.linearHertz = def.linearHertz
        joint.linearDampingRatio = def.linearDampingRatio
        joint.angularHertz = def.angularHertz
        joint.angularDampingRatio = def.angularDampingRatio
        sim.weldJoint = joint
        writeJointSim(pair.jointId, sim)

        let j = joints[Int(pair.jointId)]
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: j.generation)
    }

    func prepareWeldJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .weldJoint)

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

        var joint = base.weldJoint
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex

        // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.frameA.q = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.frameA.p = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.frameB.q = mulQuat(simB.transform.q, base.localFrameB.q)
        joint.frameB.p = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)

        joint.deltaCenter = subPos(simB.center, simA.center)
        joint.angularMass = invertMatrix(invInertiaSum)

        if joint.linearHertz == 0 {
            joint.linearSpring = base.constraintSoftness
        } else {
            joint.linearSpring = makeSoft(joint.linearHertz, joint.linearDampingRatio, context.h)
        }

        if joint.angularHertz == 0 {
            joint.angularSpring = base.constraintSoftness
        } else {
            joint.angularSpring = makeSoft(joint.angularHertz, joint.angularDampingRatio, context.h)
        }

        if context.enableWarmStarting == false {
            joint.linearImpulse = Vec3()
            joint.angularImpulse = Vec3()
        }

        base.weldJoint = joint
    }

    func warmStartWeldJoint(_ base: inout JointSim, _ context: StepContext, _ states: UnsafeMutableBufferPointer<BodyState>) {
        b3Assert(base.type == .weldJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        let joint = base.weldJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

        vA = vA - mA * joint.linearImpulse
        wA = wA - mulMV(iA, cross(rA, joint.linearImpulse) + joint.angularImpulse)

        vB = vB + mB * joint.linearImpulse
        wB = wB + mulMV(iB, cross(rB, joint.linearImpulse) + joint.angularImpulse)

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

    func solveWeldJoint(_ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        var joint = base.weldJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let fixedRotation = base.fixedRotation
        let quatA = mulQuat(stateA.deltaRotation, joint.frameA.q)
        var quatB = mulQuat(stateB.deltaRotation, joint.frameB.q)

        if dotQuat(quatA, quatB) < 0 {
            // this keeps the rotation angle in the range [-pi, pi]
            quatB = negateQuat(quatB)
        }

        let relQ = invMulQuat(quatA, quatB)

        // angular constraint
        if fixedRotation == false {
            var bias = Vec3()
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias || joint.angularHertz > 0 {
                let targetQuat = Quat.identity
                let deltaRotation = deltaQuatToRotation(relQ, targetQuat)
                let c = -(rotateVector(quatA, deltaRotation))

                bias = joint.angularSpring.biasRate * c
                massScale = joint.angularSpring.massScale
                impulseScale = joint.angularSpring.impulseScale
            }

            let cdot = wB - wA
            let impulse = (-massScale) * mulMV(joint.angularMass, cdot + bias)
                - impulseScale * joint.angularImpulse
            joint.angularImpulse = joint.angularImpulse + impulse

            wA = wA - mulMV(iA, impulse)
            wB = wB + mulMV(iB, impulse)
        }

        // linear constraint
        do {
            let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
            let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

            let cdot = (vB + cross(wB, rB)) - (vA + cross(wA, rA))

            var bias = Vec3()
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias || joint.linearHertz > 0 {
                let dcA = stateA.deltaPosition
                let dcB = stateB.deltaPosition

                let separation = ((dcB - dcA) + (rB - rA)) + joint.deltaCenter

                bias = joint.linearSpring.biasRate * separation
                massScale = joint.linearSpring.massScale
                impulseScale = joint.linearSpring.impulseScale
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

        base.weldJoint = joint
    }
}
