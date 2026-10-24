

// Prismatic joint (prismatic_joint.c). Body B slides along body A's x-axis.

struct PrismaticJointSim: BitwiseCopyable {
    var perpImpulse = Vec2()
    var angularImpulse = Vec3()
    var springImpulse: Float = 0
    var motorImpulse: Float = 0
    var lowerImpulse: Float = 0
    var upperImpulse: Float = 0
    var hertz: Float = 0
    var dampingRatio: Float = 0
    var maxMotorForce: Float = 0
    var motorSpeed: Float = 0
    var targetTranslation: Float = 0
    var lowerTranslation: Float = 0
    var upperTranslation: Float = 0

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var frameA = Transform()
    var frameB = Transform()
    var jointAxis = Vec3()
    var perpAxisY = Vec3()
    var perpAxisZ = Vec3()
    var deltaCenter = Vec3()
    var deltaAngle: Float = 0
    var rotationMass = Matrix3()
    var springSoftness = Softness()

    var enableSpring = false
    var enableLimit = false
    var enableMotor = false

    init() {}
}

extension JointSim {
    var prismaticJoint: PrismaticJointSim {
        get { prismatic }
        set { prismatic = newValue }
    }
}

extension World {
    @discardableResult
    public func createPrismaticJoint(_ def: PrismaticJointDef) -> JointID {
        let pair = createJoint(def.base, .prismaticJoint)

        var sim = pair.sim
        var joint = PrismaticJointSim()
        joint.hertz = def.hertz
        joint.dampingRatio = def.dampingRatio
        joint.maxMotorForce = def.maxMotorForce
        joint.motorSpeed = def.motorSpeed
        joint.targetTranslation = def.targetTranslation
        joint.lowerTranslation = def.lowerTranslation
        joint.upperTranslation = def.upperTranslation
        joint.enableSpring = def.enableSpring
        joint.enableLimit = def.enableLimit
        joint.enableMotor = def.enableMotor
        sim.prismaticJoint = joint
        writeJointSim(pair.jointId, sim)

        let j = joints[Int(pair.jointId)]
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: j.generation)
    }

    func preparePrismaticJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .prismaticJoint)

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

        var joint = base.prismaticJoint
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex

        // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.frameA.q = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.frameA.p = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.frameB.q = mulQuat(simB.transform.q, base.localFrameB.q)
        joint.frameB.p = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)

        joint.deltaCenter = subPos(simB.center, simA.center)
        joint.rotationMass = invertMatrix(invInertiaSum)

        // Initial joint axes in world space
        let matrixA = makeMatrixFromQuat(joint.frameA.q)
        joint.jointAxis = matrixA.cx
        joint.perpAxisY = matrixA.cy
        joint.perpAxisZ = matrixA.cz

        joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

        if context.enableWarmStarting == false {
            joint.perpImpulse = Vec2()
            joint.angularImpulse = Vec3()
            joint.motorImpulse = 0
            joint.springImpulse = 0
            joint.lowerImpulse = 0
            joint.upperImpulse = 0
        }

        base.prismaticJoint = joint
    }

    func warmStartPrismaticJoint(_ base: inout JointSim, _ context: StepContext, _ states: UnsafeMutableBufferPointer<BodyState>) {
        b3Assert(base.type == .prismaticJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        let joint = base.prismaticJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)
        let d = ((stateB.deltaPosition - stateA.deltaPosition) + joint.deltaCenter) + (rB - rA)
        let jointAxis = rotateVector(stateA.deltaRotation, joint.jointAxis)
        let sAx = cross(rA + d, jointAxis)
        let sBx = cross(rB, jointAxis)

        let perpY = rotateVector(stateA.deltaRotation, joint.perpAxisY)
        let perpZ = rotateVector(stateA.deltaRotation, joint.perpAxisZ)
        let sAy = cross(rA + d, perpY)
        let sBy = cross(rB, perpY)
        let sAz = cross(rA + d, perpZ)
        let sBz = cross(rB, perpZ)

        let axialImpulse =
            joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
        let perpImpulse = joint.perpImpulse

        let P = blend3(axialImpulse, jointAxis, perpImpulse.x, perpY, perpImpulse.y, perpZ)
        let LA = blend3(axialImpulse, sAx, perpImpulse.x, sAy, perpImpulse.y, sAz) + joint.angularImpulse
        let LB = blend3(axialImpulse, sBx, perpImpulse.x, sBy, perpImpulse.y, sBz) + joint.angularImpulse

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity
        vA = vA - mA * P
        wA = wA - mulMV(iA, LA)
        vB = vB + mB * P
        wB = wB + mulMV(iB, LB)

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

    func solvePrismaticJoint(_ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        var joint = base.prismaticJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let fixedRotation = base.fixedRotation
        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

        let dcA = stateA.deltaPosition
        let dcB = stateB.deltaPosition
        let d = ((dcB - dcA) + joint.deltaCenter) + (rB - rA)

        let jointAxis = rotateVector(stateA.deltaRotation, joint.jointAxis)
        let sAx = cross(rA + d, jointAxis)
        let sBx = cross(rB, jointAxis)
        let jointTranslation = dot(d, jointAxis)
        let targetTranslation = joint.targetTranslation

        // The axial effective mass must be fresh to avoid divergence when the joint is stressed
        let ka = mA + mB + dot(sAx, mulMV(iA, sAx)) + dot(sBx, mulMV(iB, sBx))
        let axialMass: Float = ka > 0 ? 1 / ka : 0

        // Solve spring
        if joint.enableSpring && fixedRotation == false {
            // Get the substep relative rotation
            let c = jointTranslation - targetTranslation

            let bias = joint.springSoftness.biasRate * c
            let massScale = joint.springSoftness.massScale
            let impulseScale = joint.springSoftness.impulseScale

            let vRel = (vB + cross(wB, rB)) - vA - cross(wA, rA + d)
            let cdot = dot(vRel, jointAxis)
            let deltaImpulse = -massScale * axialMass * (cdot + bias) - impulseScale * joint.springImpulse
            joint.springImpulse += deltaImpulse

            let P = deltaImpulse * jointAxis
            let LA = deltaImpulse * sAx
            let LB = deltaImpulse * sBx

            vA = vA - mA * P
            wA = wA - mulMV(iA, LA)
            vB = vB + mB * P
            wB = wB + mulMV(iB, LB)
        }

        if joint.enableMotor && fixedRotation == false {
            let vRel = (vB + cross(wB, rB)) - vA - cross(wA, rA + d)
            let cdot = dot(vRel, jointAxis) - joint.motorSpeed

            var deltaImpulse = -axialMass * cdot
            var newImpulse = joint.motorImpulse + deltaImpulse
            let maxImpulse = joint.maxMotorForce * context.h
            newImpulse = clampFloat(newImpulse, -maxImpulse, maxImpulse)
            deltaImpulse = newImpulse - joint.motorImpulse
            joint.motorImpulse = newImpulse

            let P = deltaImpulse * jointAxis
            let LA = deltaImpulse * sAx
            let LB = deltaImpulse * sBx

            vA = vA - mA * P
            wA = wA - mulMV(iA, LA)
            vB = vB + mB * P
            wB = wB + mulMV(iB, LB)
        }

        if joint.enableLimit && fixedRotation == false {
            let speculativeDistance = 0.25 * (joint.upperTranslation - joint.lowerTranslation)

            // Lower limit
            do {
                let C = jointTranslation - joint.lowerTranslation

                if C < speculativeDistance {
                    var bias: Float = 0
                    var massScale: Float = 1
                    var impulseScale: Float = 0
                    if C > 0 {
                        // speculation
                        bias = C * context.inv_h
                    } else if useBias {
                        bias = base.constraintSoftness.biasRate * C
                        massScale = base.constraintSoftness.massScale
                        impulseScale = base.constraintSoftness.impulseScale
                    }

                    let vRel = (vB + cross(wB, rB)) - vA - cross(wA, rA + d)
                    let cdot = dot(vRel, jointAxis)
                    let oldImpulse = joint.lowerImpulse
                    var deltaImpulse = -massScale * axialMass * (cdot + bias) - impulseScale * oldImpulse
                    joint.lowerImpulse = maxFloat(oldImpulse + deltaImpulse, 0)
                    deltaImpulse = joint.lowerImpulse - oldImpulse

                    let P = deltaImpulse * jointAxis
                    let LA = deltaImpulse * sAx
                    let LB = deltaImpulse * sBx

                    vA = vA - mA * P
                    wA = wA - mulMV(iA, LA)
                    vB = vB + mB * P
                    wB = wB + mulMV(iB, LB)
                } else {
                    joint.lowerImpulse = 0
                }
            }

            // Upper limit
            do {
                let C = joint.upperTranslation - jointTranslation

                if C < speculativeDistance {
                    var bias: Float = 0
                    var massScale: Float = 1
                    var impulseScale: Float = 0
                    if C > 0 {
                        // speculation
                        bias = C * context.inv_h
                    } else if useBias {
                        bias = base.constraintSoftness.biasRate * C
                        massScale = base.constraintSoftness.massScale
                        impulseScale = base.constraintSoftness.impulseScale
                    }

                    // sign flipped on Cdot
                    let vRel = (vB + cross(wB, rB)) - vA - cross(wA, rA + d)
                    let cdot = -dot(vRel, jointAxis)
                    let oldImpulse = joint.upperImpulse
                    let deltaImpulse = -massScale * axialMass * (cdot + bias) - impulseScale * oldImpulse
                    joint.upperImpulse = maxFloat(oldImpulse + deltaImpulse, 0)

                    // sign flipped on applied impulse
                    let negDeltaImpulse = oldImpulse - joint.upperImpulse
                    let P = negDeltaImpulse * jointAxis
                    let LA = negDeltaImpulse * sAx
                    let LB = negDeltaImpulse * sBx

                    vA = vA - mA * P
                    wA = wA - mulMV(iA, LA)
                    vB = vB + mB * P
                    wB = wB + mulMV(iB, LB)
                } else {
                    joint.upperImpulse = 0
                }
            }
        }

        // Rotation constraint
        if fixedRotation == false {
            var bias = Vec3()
            var massScale: Float = 1
            var impulseScale: Float = 0

            if useBias {
                let quatA = mulQuat(stateA.deltaRotation, joint.frameA.q)
                let quatB = mulQuat(stateB.deltaRotation, joint.frameB.q)

                let relQ = invMulQuat(quatA, quatB)
                let targetQuat = Quat.identity
                let deltaRotation = deltaQuatToRotation(relQ, targetQuat)
                let c = -(rotateVector(quatA, deltaRotation))

                bias = base.constraintSoftness.biasRate * c
                massScale = base.constraintSoftness.massScale
                impulseScale = base.constraintSoftness.impulseScale
            }

            let cdot = wB - wA
            let impulse = (-massScale) * mulMV(joint.rotationMass, cdot + bias)
                - impulseScale * joint.angularImpulse
            joint.angularImpulse = joint.angularImpulse + impulse

            wA = wA - mulMV(iA, impulse)
            wB = wB + mulMV(iB, impulse)
        }

        // Solve point-to-line constraint
        do {
            let perpY = rotateVector(stateA.deltaRotation, joint.perpAxisY)
            let perpZ = rotateVector(stateA.deltaRotation, joint.perpAxisZ)

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

            // K = [(1/mA + 1/mB) * eye(2) - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]
            let sAy = cross(rA + d, perpY)
            let sBy = cross(rB, perpY)
            let sAz = cross(rA + d, perpZ)
            let sBz = cross(rB, perpZ)

            let kyy = mA + mB + dot(sAy, mulMV(iA, sAy)) + dot(sBy, mulMV(iB, sBy))
            let kyz = dot(sAy, mulMV(iA, sAz)) + dot(sBy, mulMV(iB, sBz))
            let kzz = mA + mB + dot(sAz, mulMV(iA, sAz)) + dot(sBz, mulMV(iB, sBz))

            let K = Matrix2(Vec2(kyy, kyz), Vec2(kyz, kzz))

            let oldImpulse = joint.perpImpulse
            let sol = solve2(K, Vec2(cdot.x + bias.x, cdot.y + bias.y))
            let deltaImpulse = Vec2(
                (-massScale) * sol.x - impulseScale * oldImpulse.x,
                (-massScale) * sol.y - impulseScale * oldImpulse.y
            )
            joint.perpImpulse = Vec2(
                oldImpulse.x + deltaImpulse.x, oldImpulse.y + deltaImpulse.y
            )

            let P = blend2(deltaImpulse.x, perpY, deltaImpulse.y, perpZ)

            vA = vA - mA * P
            wA = wA - mulMV(
                iA, blend2(deltaImpulse.x, sAy, deltaImpulse.y, sAz)
            )
            vB = vB + mB * P
            wB = wB + mulMV(
                iB, blend2(deltaImpulse.x, sBy, deltaImpulse.y, sBz)
            )
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

        base.prismaticJoint = joint
    }
}
