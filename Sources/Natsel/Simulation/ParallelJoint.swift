

// Parallel joint (parallel_joint.c). Keeps the bodies aligned along their z-axes.

struct ParallelJointSim {
    var hertz: Float = 0
    var dampingRatio: Float = 0
    var maxTorque: Float = 0

    var perpImpulse = Vec2()
    var perpAxisX = Vec3()
    var perpAxisY = Vec3()

    var quatA = Quat()
    var quatB = Quat()
    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var softness = Softness()

    init() {}
}

extension JointSim {
    var parallelJoint: ParallelJointSim {
        get { parallel }
        set { parallel = newValue }
    }
}

extension World {
    @discardableResult
    public func createParallelJoint(_ def: ParallelJointDef) -> JointID {
        let pair = createJoint(def.base, .parallelJoint)

        var sim = pair.sim
        var joint = ParallelJointSim()
        joint.hertz = def.hertz
        joint.dampingRatio = def.dampingRatio
        joint.maxTorque = def.maxTorque
        sim.parallelJoint = joint
        writeJointSim(pair.jointId, sim)

        let j = joints[Int(pair.jointId)]
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: j.generation)
    }

    func prepareParallelJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .parallelJoint)

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

        var joint = base.parallelJoint
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex

        // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.quatA = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.quatB = mulQuat(simB.transform.q, base.localFrameB.q)

        let relQ = invMulQuat(joint.quatA, joint.quatB)

        do {
            // These are needed for warm starting
            joint.perpAxisX = 0.5 * rotateVector(
                joint.quatA, relQ.s * Vec3.axisX + cross(relQ.v, Vec3.axisX))
            joint.perpAxisY = 0.5 * rotateVector(
                joint.quatA, relQ.s * Vec3.axisY + cross(relQ.v, Vec3.axisY))
        }

        joint.softness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

        if context.enableWarmStarting == false {
            joint.perpImpulse = Vec2()
        }

        base.parallelJoint = joint
    }

    func warmStartParallelJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .parallelJoint)

        let iA = base.invIA
        let iB = base.invIB

        let joint = base.parallelJoint
        var stateA = solverState(joint.indexA)
        var stateB = solverState(joint.indexB)

        var wA = stateA.angularVelocity
        var wB = stateB.angularVelocity

        let angularImpulse = blend2(
            joint.perpImpulse.x, joint.perpAxisX, joint.perpImpulse.y, joint.perpAxisY
        )

        wA = wA - mulMV(iA, angularImpulse)
        wB = wB + mulMV(iB, angularImpulse)

        if (stateA.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateA.angularVelocity = wA
        }

        if (stateB.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateB.angularVelocity = wB
        }

        storeSolverState(joint.indexA, stateA)
        storeSolverState(joint.indexB, stateB)
    }

    func solveParallelJoint(_ base: inout JointSim, _ context: StepContext) {
        let iA = base.invIA
        let iB = base.invIB

        var joint = base.parallelJoint
        var stateA = solverState(joint.indexA)
        var stateB = solverState(joint.indexB)

        var wA = stateA.angularVelocity
        var wB = stateB.angularVelocity

        let fixedRotation = base.fixedRotation
        let quatA = mulQuat(stateA.deltaRotation, joint.quatA)
        var quatB = mulQuat(stateB.deltaRotation, joint.quatB)

        if dotQuat(quatA, quatB) < 0 {
            // this keeps the rotation angle in the range [-pi, pi]
            quatB = negateQuat(quatB)
        }

        let relQ = invMulQuat(quatA, quatB)

        if fixedRotation == false && joint.maxTorque > 0 {
            let c = Vec2(relQ.v.x, relQ.v.y)
            let bias = Vec2(joint.softness.biasRate * c.x, joint.softness.biasRate * c.y)
            let massScale = joint.softness.massScale
            let impulseScale = joint.softness.impulseScale

            // Collinearity constraint as 2-by-2
            let perpAxisX = 0.5 * rotateVector(
                quatA, relQ.s * Vec3.axisX + cross(relQ.v, Vec3.axisX))
            let perpAxisY = 0.5 * rotateVector(
                quatA, relQ.s * Vec3.axisY + cross(relQ.v, Vec3.axisY))
            joint.perpAxisX = perpAxisX
            joint.perpAxisY = perpAxisY

            let invInertiaSum = addMM(iA, iB)
            let kxx = dot(perpAxisX, mulMV(invInertiaSum, perpAxisX))
            let kyy = dot(perpAxisY, mulMV(invInertiaSum, perpAxisY))
            let kxy = dot(perpAxisX, mulMV(invInertiaSum, perpAxisY))

            let k = Matrix2(Vec2(kxx, kxy), Vec2(kxy, kyy))

            let wRel = wB - wA
            let cdot = Vec2(dot(wRel, perpAxisX), dot(wRel, perpAxisY))

            let maxImpulse = context.h * joint.maxTorque
            let oldImpulse = joint.perpImpulse
            let cdotPlusBias = Vec2(cdot.x + bias.x, cdot.y + bias.y)
            let sol = solve2(k, cdotPlusBias)
            var deltaImpulse = Vec2(
                -massScale * sol.x - impulseScale * oldImpulse.x,
                -massScale * sol.y - impulseScale * oldImpulse.y
            )
            joint.perpImpulse = Vec2(oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y)
            let lengthSquared =
                joint.perpImpulse.x * joint.perpImpulse.x + joint.perpImpulse.y * joint.perpImpulse.y
            if lengthSquared > maxImpulse * maxImpulse {
                let s = maxImpulse / lengthSquared.squareRoot()
                joint.perpImpulse = Vec2(
                    s * joint.perpImpulse.x, s * joint.perpImpulse.y
                )
            }

            deltaImpulse = Vec2(
                joint.perpImpulse.x - oldImpulse.x, joint.perpImpulse.y - oldImpulse.y
            )

            let angularImpulse = blend2(deltaImpulse.x, perpAxisX, deltaImpulse.y, perpAxisY)
            wA = wA - mulMV(iA, angularImpulse)
            wB = wB + mulMV(iB, angularImpulse)
        }

        if (stateA.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateA.angularVelocity = wA
        }

        if (stateB.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            stateB.angularVelocity = wB
        }

        storeSolverState(joint.indexA, stateA)
        storeSolverState(joint.indexB, stateB)

        base.parallelJoint = joint
    }
}
