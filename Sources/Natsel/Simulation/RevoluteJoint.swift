

// Revolute joint (revolute_joint.c).

struct RevoluteJointSim: BitwiseCopyable {
    var linearImpulse = Vec3()
    var perpImpulse = Vec2()
    var springImpulse: Float = 0
    var motorImpulse: Float = 0
    var lowerImpulse: Float = 0
    var upperImpulse: Float = 0
    var hertz: Float = 0
    var dampingRatio: Float = 0
    var maxMotorTorque: Float = 0
    var motorSpeed: Float = 0
    var targetAngle: Float = 0
    var lowerAngle: Float = 0
    var upperAngle: Float = 0

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var frameA = Transform()
    var frameB = Transform()
    var rotationAxisZ = Vec3()
    var perpAxisX = Vec3()
    var perpAxisY = Vec3()
    var deltaCenter = Vec3()
    var deltaAngle: Float = 0
    var axialMass: Float = 0
    var springSoftness = Softness()

    var enableSpring = false
    var enableMotor = false
    var enableLimit = false

    init() {}
}

/// Solve a 3-by-3 point-to-point constraint.
@inline(__always)
func solvePointConstraint(
    _ mA: Float, _ mB: Float, _ iA: Matrix3, _ iB: Matrix3, _ rA: Vec3, _ rB: Vec3, _ rhs: Vec3
) -> Vec3 {
    let axy = rA.z * iA.cy.y - rA.y * iA.cz.y
    let axz = rA.z * iA.cy.z - rA.y * iA.cz.z
    let ay = rA.x * iA.cz - rA.z * iA.cx
    let az = rA.y * iA.cx - rA.x * iA.cy

    let bxy = rB.z * iB.cy.y - rB.y * iB.cz.y
    let bxz = rB.z * iB.cy.z - rB.y * iB.cz.z
    let by = rB.x * iB.cz - rB.z * iB.cx
    let bz = rB.y * iB.cx - rB.x * iB.cy

    let m = mA + mB
    let kxx = (axy * rA.z - axz * rA.y) + (bxy * rB.z - bxz * rB.y) + m
    let kyy = (ay.z * rA.x - ay.x * rA.z) + (by.z * rB.x - by.x * rB.z) + m
    let kzz = (az.x * rA.y - az.y * rA.x) + (bz.x * rB.y - bz.y * rB.x) + m
    let kxy = (ay.y * rA.z - ay.z * rA.y) + (by.y * rB.z - by.z * rB.y)
    let kxz = (az.y * rA.z - az.z * rA.y) + (bz.y * rB.z - bz.z * rB.y)
    let kyz = (az.z * rA.x - az.x * rA.z) + (bz.z * rB.x - bz.x * rB.z)

    let cxx = kyy * kzz - kyz * kyz
    let cyy = kxx * kzz - kxz * kxz
    let czz = kxx * kyy - kxy * kxy
    let cxy = kxz * kyz - kxy * kzz
    let cxz = kxy * kyz - kyy * kxz
    let cyz = kxy * kxz - kxx * kyz

    let determinant = kxx * cxx + kxy * cxy + kxz * cxz
    if absFloat(determinant) > 1000 * Float.leastNormalMagnitude {
        let invDet = 1 / determinant
        return Vec3(
            invDet * (cxx * rhs.x + cxy * rhs.y + cxz * rhs.z),
            invDet * (cxy * rhs.x + cyy * rhs.y + cyz * rhs.z),
            invDet * (cxz * rhs.x + cyz * rhs.y + czz * rhs.z)
        )
    }

    return Vec3()
}

extension JointSim {
    var revoluteJoint: RevoluteJointSim {
        get { revolute }
        set { revolute = newValue }
    }
}

extension World {
    @discardableResult
    public func createRevoluteJoint(_ def: RevoluteJointDef) -> JointID {
        let pair = createJoint(def.base, .revoluteJoint)

        var sim = pair.sim
        var joint = RevoluteJointSim()
        joint.hertz = def.hertz
        joint.dampingRatio = def.dampingRatio
        joint.maxMotorTorque = def.maxMotorTorque
        joint.motorSpeed = def.motorSpeed
        joint.targetAngle = def.targetAngle
        joint.lowerAngle = def.lowerAngle
        joint.upperAngle = def.upperAngle
        joint.enableSpring = def.enableSpring
        joint.enableMotor = def.enableMotor
        joint.enableLimit = def.enableLimit
        sim.revoluteJoint = joint
        writeJointSim(pair.jointId, sim)

        let j = joints[Int(pair.jointId)]
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: j.generation)
    }

    func prepareRevoluteJoint(_ base: inout JointSim, _ context: StepContext) {
        b3Assert(base.type == .revoluteJoint)

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

        var joint = base.revoluteJoint
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex

        // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.frameA.q = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.frameA.p = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.frameB.q = mulQuat(simB.transform.q, base.localFrameB.q)
        joint.frameB.p = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)

        joint.deltaCenter = subPos(simB.center, simA.center)

        do {
            // Rotation axis is the z-axis of body A.
            let rotationAxisZ = rotateVector(joint.frameA.q, Vec3.axisZ)
            let k = dot(rotationAxisZ, mulMV(invInertiaSum, rotationAxisZ))
            joint.axialMass = k > 0 ? 1 / k : 0
            joint.rotationAxisZ = rotationAxisZ
        }

        let relQ = invMulQuat(joint.frameA.q, joint.frameB.q)

        do {
            // These are needed for warm starting
            joint.perpAxisX = 0.5 * rotateVector(
                joint.frameA.q, relQ.s * Vec3.axisX + cross(relQ.v, Vec3.axisX))
            joint.perpAxisY = 0.5 * rotateVector(
                joint.frameA.q, relQ.s * Vec3.axisY + cross(relQ.v, Vec3.axisY))
        }

        joint.springSoftness = makeSoft(joint.hertz, joint.dampingRatio, context.h)

        if context.enableWarmStarting == false {
            joint.linearImpulse = Vec3()
            joint.perpImpulse = Vec2()
            joint.motorImpulse = 0
            joint.springImpulse = 0
            joint.lowerImpulse = 0
            joint.upperImpulse = 0
        }

        base.revoluteJoint = joint
    }

    func warmStartRevoluteJoint(_ base: inout JointSim, _ context: StepContext, _ states: UnsafeMutableBufferPointer<BodyState>) {
        b3Assert(base.type == .revoluteJoint)

        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        let joint = base.revoluteJoint
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)

        let axialImpulse =
            joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
        var angularImpulse =
            joint.perpImpulse.x * joint.perpAxisX + joint.perpImpulse.y * joint.perpAxisY
        angularImpulse = mulAdd(angularImpulse, axialImpulse, joint.rotationAxisZ)

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

    func solveRevoluteJoint(_ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB

        var joint = base.revoluteJoint
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

        if fixedRotation == false && (joint.enableSpring || joint.enableMotor || joint.enableLimit) {
            let axis = joint.rotationAxisZ
            let gA = mulMV(iA, axis)
            let gB = mulMV(iB, axis)
            let k = dot(axis, gA + gB)
            var axialCdot = dot(wB - wA, axis)
            var axialImpulse: Float = 0

            // Solve spring
            if joint.enableSpring {
                // Get the substep relative rotation
                let targetAngle = joint.targetAngle
                let angle = getTwistAngle(relQ)
                let c = angle - targetAngle

                let bias = joint.springSoftness.biasRate * c
                let massScale = joint.springSoftness.massScale
                let impulseScale = joint.springSoftness.impulseScale

                let deltaImpulse = -massScale * joint.axialMass * (axialCdot + bias)
                    - impulseScale * joint.springImpulse
                joint.springImpulse += deltaImpulse

                axialCdot += k * deltaImpulse
                axialImpulse += deltaImpulse
            }

            if joint.enableMotor {
                let cdot = axialCdot - joint.motorSpeed
                let oldImpulse = joint.motorImpulse
                let maxImpulse = joint.maxMotorTorque * context.h
                let deltaImpulse = clampFloat(-joint.axialMass * cdot, -maxImpulse - oldImpulse, maxImpulse - oldImpulse)
                joint.motorImpulse = oldImpulse + deltaImpulse

                axialCdot += k * deltaImpulse
                axialImpulse += deltaImpulse
            }

            if joint.enableLimit {
                let angle = getTwistAngle(relQ)

                // todo does an updated twist axis help?

                // Lower limit
                do {
                    let c = angle - joint.lowerAngle
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

                    let oldImpulse = joint.lowerImpulse
                    let deltaImpulse = maxFloat(
                        -massScale * joint.axialMass * (axialCdot + bias) - impulseScale * oldImpulse,
                        -oldImpulse
                    )
                    joint.lowerImpulse = oldImpulse + deltaImpulse

                    axialCdot += k * deltaImpulse
                    axialImpulse += deltaImpulse
                }

                // Upper limit
                do {
                    let c = joint.upperAngle - angle
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
                    let cdot = -axialCdot
                    let oldImpulse = joint.upperImpulse
                    let deltaImpulse = maxFloat(
                        -massScale * joint.axialMass * (cdot + bias) - impulseScale * oldImpulse,
                        -oldImpulse
                    )
                    joint.upperImpulse = oldImpulse + deltaImpulse

                    // sign flipped on applied impulse
                    axialImpulse -= deltaImpulse
                }
            }

            wA = wA - axialImpulse * gA
            wB = wB + axialImpulse * gB
        }

        // Collinearity constraint
        if fixedRotation == false {
            var bias = Vec2()
            var massScale: Float = 1
            var impulseScale: Float = 0

            if useBias {
                let c = Vec2(relQ.v.x, relQ.v.y)
                bias = Vec2(
                    base.constraintSoftness.biasRate * c.x, base.constraintSoftness.biasRate * c.y
                )
                massScale = base.constraintSoftness.massScale
                impulseScale = base.constraintSoftness.impulseScale
            }

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
            let oldImpulse = joint.perpImpulse
            let sol = solve2(k, Vec2(cdot.x + bias.x, cdot.y + bias.y))
            let deltaImpulse = Vec2(
                (-massScale) * sol.x - impulseScale * oldImpulse.x,
                (-massScale) * sol.y - impulseScale * oldImpulse.y
            )
            joint.perpImpulse = Vec2(
                joint.perpImpulse.x + deltaImpulse.x, joint.perpImpulse.y + deltaImpulse.y
            )

            let angularImpulse = deltaImpulse.x * perpAxisX + deltaImpulse.y * perpAxisY
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

        base.revoluteJoint = joint
    }
}
