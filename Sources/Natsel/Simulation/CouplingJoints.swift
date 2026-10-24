// Coupling joints: gear, rack and pinion, and pulley. Each couples one degree of freedom of two
// bodies. Angles and translations are tracked across steps without wrapping, so a gear may turn any
// number of revolutions. Gear and rack joints can measure their coordinates through the revolute
// and prismatic joints that hold the bodies, which keeps the coupling exact on moving mounts.

/// Measures the rotation of a body about a hinge axis, unwrapped across steps.
struct AngleProbe: BitwiseCopyable {
    // Through a revolute joint
    var usesJoint = false
    var jointIndex: Int32 = B3.nullIndex
    var jointGeneration: UInt16 = 0
    var jointBodyA: Int32 = B3.nullIndex
    var jointBodyB: Int32 = B3.nullIndex
    var jointFrameA = Quat.identity
    var jointFrameB = Quat.identity
    var sign: Float = 1

    // Directly from the body
    var body: Int32 = B3.nullIndex
    var hingeFrame = Quat.identity
    var rotation0 = Quat.identity

    /// Last wrapped measurement and the unwrapped total.
    var twist: Float = 0
    var angle: Float = 0
}

/// Measures the translation of a body along an axis.
struct TranslationProbe: BitwiseCopyable {
    var usesJoint = false
    var jointIndex: Int32 = B3.nullIndex
    var jointGeneration: UInt16 = 0
    var jointBodyA: Int32 = B3.nullIndex
    var jointBodyB: Int32 = B3.nullIndex
    var jointFrameA = Transform.identity
    var jointAnchorB = Vec3()
    var sign: Float = 1

    var body: Int32 = B3.nullIndex
    var center0 = Pos()
    var axis0 = Vec3()

    var value: Float = 0
}

struct GearJointSim: BitwiseCopyable {
    var ratio: Float = 1
    var impulse: Float = 0
    var probeA = AngleProbe()
    var probeB = AngleProbe()
    var offset: Float = 0

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var axisA = Vec3()
    var axisB = Vec3()
    var mass: Float = 0
    var position: Float = 0

    init() {}
}

struct RackAndPinionJointSim: BitwiseCopyable {
    var ratio: Float = 1
    var impulse: Float = 0
    var pinion = AngleProbe()
    var rack = TranslationProbe()
    var offset: Float = 0

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var axisA = Vec3()
    var axisB = Vec3()
    var mass: Float = 0
    var position: Float = 0

    init() {}
}

struct PulleyJointSim: BitwiseCopyable {
    var groundAnchorA = Pos()
    var groundAnchorB = Pos()
    var ratio: Float = 1
    var minLength: Float = 0
    var maxLength: Float = 0
    var impulse: Float = 0
    var lowerImpulse: Float = 0
    var upperImpulse: Float = 0

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var centerA = Pos()
    var centerB = Pos()
    var rA = Vec3()
    var rB = Vec3()
    var mass: Float = 0

    init() {}
}

/// Small rotation of a delta quaternion about an axis.
@inline(__always)
private func deltaAngle(_ dq: Quat, _ axis: Vec3) -> Float {
    let s: Float = dq.s < 0 ? -2 : 2
    return s * dot(dq.v, axis)
}

extension World {
    // MARK: Probes

    private func makeAngleProbe(_ body: Int32, _ hingeFrame: Quat, reference: JointID) -> AngleProbe {
        var probe = AngleProbe()
        probe.body = body
        probe.hingeFrame = hingeFrame
        probe.rotation0 = bodyTransformQuick(body).q

        let index = reference.index1 - 1
        if index >= 0 && Int(index) < joints.count {
            let joint = joints[Int(index)]
            if joint.jointId != B3.nullIndex && joint.generation == reference.generation && joint.type == .revoluteJoint
                && (joint.edges[0].bodyId == body || joint.edges[1].bodyId == body)
            {
                let sim = readJointSim(index)
                probe.usesJoint = true
                probe.jointIndex = index
                probe.jointGeneration = joint.generation
                probe.jointBodyA = sim.bodyIdA
                probe.jointBodyB = sim.bodyIdB
                probe.jointFrameA = sim.localFrameA.q
                probe.jointFrameB = sim.localFrameB.q
                probe.sign = sim.bodyIdB == body ? 1 : -1
            }
        }
        probe.twist = measureAngle(probe)
        probe.angle = probe.twist
        return probe
    }

    private func measureAngle(_ probe: AngleProbe) -> Float {
        if probe.usesJoint {
            let qA = mulQuat(bodyTransformQuick(probe.jointBodyA).q, probe.jointFrameA)
            let qB = mulQuat(bodyTransformQuick(probe.jointBodyB).q, probe.jointFrameB)
            return probe.sign * getTwistAngle(invMulQuat(qA, qB))
        }
        // Rotation since creation, expressed in the hinge frame.
        let relative = invMulQuat(probe.rotation0, bodyTransformQuick(probe.body).q)
        let inHinge = invMulQuat(probe.hingeFrame, mulQuat(relative, probe.hingeFrame))
        return getTwistAngle(inHinge)
    }

    /// Accumulate the change since the last step without wrapping.
    private func updateAngle(_ probe: inout AngleProbe) {
        if probe.usesJoint {
            let index = Int(probe.jointIndex)
            let alive = index < joints.count && joints[index].jointId != B3.nullIndex
                && joints[index].generation == probe.jointGeneration
            if alive == false {
                // The reference joint is gone: continue from the body rotation.
                probe.usesJoint = false
                probe.rotation0 = bodyTransformQuick(probe.body).q
                probe.twist = 0
                return
            }
        }
        let twist = measureAngle(probe)
        probe.angle += unwindAngle(twist - probe.twist)
        probe.twist = twist
    }

    private func makeTranslationProbe(_ body: Int32, _ axisLocal: Vec3, reference: JointID) -> TranslationProbe {
        var probe = TranslationProbe()
        probe.body = body
        let xf = bodyTransformQuick(body)
        probe.center0 = transformWorldPoint(xf, bodySim(bodies[Int(body)]).localCenter)
        probe.axis0 = rotateVector(xf.q, axisLocal)

        let index = reference.index1 - 1
        if index >= 0 && Int(index) < joints.count {
            let joint = joints[Int(index)]
            if joint.jointId != B3.nullIndex && joint.generation == reference.generation && joint.type == .prismaticJoint
                && (joint.edges[0].bodyId == body || joint.edges[1].bodyId == body)
            {
                let sim = readJointSim(index)
                probe.usesJoint = true
                probe.jointIndex = index
                probe.jointGeneration = joint.generation
                probe.jointBodyA = sim.bodyIdA
                probe.jointBodyB = sim.bodyIdB
                probe.jointFrameA = sim.localFrameA
                probe.jointAnchorB = sim.localFrameB.p
                probe.sign = sim.bodyIdB == body ? 1 : -1
            }
        }
        probe.value = measureTranslation(probe)
        return probe
    }

    private func measureTranslation(_ probe: TranslationProbe) -> Float {
        if probe.usesJoint {
            let xfA = bodyTransformQuick(probe.jointBodyA)
            let xfB = bodyTransformQuick(probe.jointBodyB)
            let axis = rotateVector(xfA.q, rotateVector(probe.jointFrameA.q, .axisX))
            let pA = transformWorldPoint(xfA, probe.jointFrameA.p)
            let pB = transformWorldPoint(xfB, probe.jointAnchorB)
            return probe.sign * dot(subPos(pB, pA), axis)
        }
        let xf = bodyTransformQuick(probe.body)
        let center = transformWorldPoint(xf, bodySim(bodies[Int(probe.body)]).localCenter)
        return dot(subPos(center, probe.center0), probe.axis0)
    }

    private func updateTranslation(_ probe: inout TranslationProbe) {
        if probe.usesJoint {
            let index = Int(probe.jointIndex)
            let alive = index < joints.count && joints[index].jointId != B3.nullIndex
                && joints[index].generation == probe.jointGeneration
            if alive == false {
                let value = probe.value
                probe.usesJoint = false
                let xf = bodyTransformQuick(probe.body)
                probe.center0 = transformWorldPoint(xf, bodySim(bodies[Int(probe.body)]).localCenter)
                probe.axis0 = rotateVector(xf.q, probe.axis0)
                probe.value = value
                return
            }
            probe.value = measureTranslation(probe)
        } else {
            probe.value = measureTranslation(probe)
        }
    }

    // MARK: Creation

    private func validJointBodies(_ def: JointDef) -> Bool {
        isValidBody(def.bodyIdA) && isValidBody(def.bodyIdB) && def.bodyIdA != def.bodyIdB
    }

    @discardableResult
    public func createGearJoint(_ def: GearJointDef) -> JointID {
        guard validJointBodies(def.base), def.ratio.isFinite else { return JointID() }
        let pair = createJoint(def.base, .gearJoint)
        var sim = pair.sim
        var gear = GearJointSim()
        gear.ratio = def.ratio
        gear.probeA = makeAngleProbe(sim.bodyIdA, sim.localFrameA.q, reference: def.referenceJointA)
        gear.probeB = makeAngleProbe(sim.bodyIdB, sim.localFrameB.q, reference: def.referenceJointB)
        gear.offset = gear.probeA.angle + gear.ratio * gear.probeB.angle
        sim.gear = gear
        writeJointSim(pair.jointId, sim)
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: joints[Int(pair.jointId)].generation)
    }

    @discardableResult
    public func createRackAndPinionJoint(_ def: RackAndPinionJointDef) -> JointID {
        guard validJointBodies(def.base), def.ratio.isFinite else { return JointID() }
        let pair = createJoint(def.base, .rackAndPinionJoint)
        var sim = pair.sim
        var joint = RackAndPinionJointSim()
        joint.ratio = def.ratio
        joint.pinion = makeAngleProbe(sim.bodyIdA, sim.localFrameA.q, reference: def.referencePinionJoint)
        joint.rack = makeTranslationProbe(
            sim.bodyIdB, rotateVector(sim.localFrameB.q, .axisX), reference: def.referenceRackJoint)
        joint.offset = joint.rack.value - joint.ratio * joint.pinion.angle
        sim.rackAndPinion = joint
        writeJointSim(pair.jointId, sim)
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: joints[Int(pair.jointId)].generation)
    }

    @discardableResult
    public func createPulleyJoint(_ def: PulleyJointDef) -> JointID {
        guard validJointBodies(def.base), def.ratio > 0, isValidPosition(def.groundAnchorA),
            isValidPosition(def.groundAnchorB)
        else { return JointID() }
        let pair = createJoint(def.base, .pulleyJoint)
        var sim = pair.sim
        var joint = PulleyJointSim()
        joint.groundAnchorA = def.groundAnchorA
        joint.groundAnchorB = def.groundAnchorB
        joint.ratio = def.ratio
        let pA = transformWorldPoint(bodyTransformQuick(sim.bodyIdA), sim.localFrameA.p)
        let pB = transformWorldPoint(bodyTransformQuick(sim.bodyIdB), sim.localFrameB.p)
        let current = distance(pA, def.groundAnchorA) + def.ratio * distance(pB, def.groundAnchorB)
        joint.minLength = def.minLength > 0 ? def.minLength : current
        joint.maxLength = def.maxLength > 0 ? max(def.maxLength, joint.minLength) : max(current, joint.minLength)
        sim.pulley = joint
        writeJointSim(pair.jointId, sim)
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: joints[Int(pair.jointId)].generation)
    }

    // MARK: Shared solver helpers

    @inline(__always)
    private func prepareBodies(_ base: inout JointSim) -> (simA: BodySim, simB: BodySim, indexA: Int32, indexB: Int32) {
        let bodyA = jointPrepareBody(base.bodyIdA)
        let bodyB = jointPrepareBody(base.bodyIdB)
        let simA = jointPrepareSim(bodyA)
        let simB = jointPrepareSim(bodyB)
        base.invMassA = simA.invMass
        base.invMassB = simB.invMass
        base.invIA = simA.invInertiaWorld
        base.invIB = simB.invInertiaWorld
        let indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        let indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex
        return (simA, simB, indexA, indexB)
    }

    @inline(__always)
    private func softTerms(_ base: JointSim, _ c: Float, _ useBias: Bool) -> (bias: Float, massScale: Float, impulseScale: Float) {
        guard useBias else { return (0, 1, 0) }
        let soft = base.constraintSoftness
        return (soft.biasRate * c, soft.massScale, soft.impulseScale)
    }

    // MARK: Gear

    func prepareGearJoint(_ base: inout JointSim, _ context: StepContext) {
        let (simA, simB, indexA, indexB) = prepareBodies(&base)
        var joint = base.gear
        joint.indexA = indexA
        joint.indexB = indexB
        updateAngle(&joint.probeA)
        updateAngle(&joint.probeB)
        joint.axisA = rotateVector(simA.transform.q, rotateVector(base.localFrameA.q, .axisZ))
        joint.axisB = rotateVector(simB.transform.q, rotateVector(base.localFrameB.q, .axisZ))
        let k = dot(joint.axisA, mulMV(base.invIA, joint.axisA))
            + joint.ratio * joint.ratio * dot(joint.axisB, mulMV(base.invIB, joint.axisB))
        joint.mass = k > 0 ? 1 / k : 0
        joint.position = joint.probeA.angle + joint.ratio * joint.probeB.angle - joint.offset
        if context.enableWarmStarting == false { joint.impulse = 0 }
        base.gear = joint
    }

    func warmStartGearJoint(_ base: inout JointSim, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let joint = base.gear
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        stateA.angularVelocity = mulAdd(stateA.angularVelocity, joint.impulse, mulMV(base.invIA, joint.axisA))
        stateB.angularVelocity = mulAdd(stateB.angularVelocity, joint.ratio * joint.impulse, mulMV(base.invIB, joint.axisB))
        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
    }

    func solveGearJoint(
        _ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>
    ) {
        var joint = base.gear
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        let c = joint.position + deltaAngle(stateA.deltaRotation, joint.axisA)
            + joint.ratio * deltaAngle(stateB.deltaRotation, joint.axisB)
        let cdot = dot(stateA.angularVelocity, joint.axisA) + joint.ratio * dot(stateB.angularVelocity, joint.axisB)
        let (bias, massScale, impulseScale) = softTerms(base, c, useBias)
        let impulse = -massScale * joint.mass * (cdot + bias) - impulseScale * joint.impulse
        joint.impulse += impulse
        stateA.angularVelocity = mulAdd(stateA.angularVelocity, impulse, mulMV(base.invIA, joint.axisA))
        stateB.angularVelocity = mulAdd(stateB.angularVelocity, joint.ratio * impulse, mulMV(base.invIB, joint.axisB))
        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
        base.gear = joint
    }

    // MARK: Rack and pinion

    func prepareRackAndPinionJoint(_ base: inout JointSim, _ context: StepContext) {
        let (simA, simB, indexA, indexB) = prepareBodies(&base)
        var joint = base.rackAndPinion
        joint.indexA = indexA
        joint.indexB = indexB
        updateAngle(&joint.pinion)
        updateTranslation(&joint.rack)
        joint.axisA = rotateVector(simA.transform.q, rotateVector(base.localFrameA.q, .axisZ))
        joint.axisB = rotateVector(simB.transform.q, rotateVector(base.localFrameB.q, .axisX))
        let k = base.invMassB + joint.ratio * joint.ratio * dot(joint.axisA, mulMV(base.invIA, joint.axisA))
        joint.mass = k > 0 ? 1 / k : 0
        joint.position = joint.rack.value - joint.ratio * joint.pinion.angle - joint.offset
        if context.enableWarmStarting == false { joint.impulse = 0 }
        base.rackAndPinion = joint
    }

    func warmStartRackAndPinionJoint(_ base: inout JointSim, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let joint = base.rackAndPinion
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        stateA.angularVelocity = mulAdd(stateA.angularVelocity, -joint.ratio * joint.impulse, mulMV(base.invIA, joint.axisA))
        stateB.linearVelocity = mulAdd(stateB.linearVelocity, base.invMassB * joint.impulse, joint.axisB)
        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
    }

    func solveRackAndPinionJoint(
        _ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>
    ) {
        var joint = base.rackAndPinion
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        let c = joint.position + dot(stateB.deltaPosition, joint.axisB)
            - joint.ratio * deltaAngle(stateA.deltaRotation, joint.axisA)
        let cdot = dot(stateB.linearVelocity, joint.axisB) - joint.ratio * dot(stateA.angularVelocity, joint.axisA)
        let (bias, massScale, impulseScale) = softTerms(base, c, useBias)
        let impulse = -massScale * joint.mass * (cdot + bias) - impulseScale * joint.impulse
        joint.impulse += impulse
        stateA.angularVelocity = mulAdd(stateA.angularVelocity, -joint.ratio * impulse, mulMV(base.invIA, joint.axisA))
        stateB.linearVelocity = mulAdd(stateB.linearVelocity, base.invMassB * impulse, joint.axisB)
        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
        base.rackAndPinion = joint
    }

    // MARK: Pulley

    func preparePulleyJoint(_ base: inout JointSim, _ context: StepContext) {
        let (simA, simB, indexA, indexB) = prepareBodies(&base)
        var joint = base.pulley
        joint.indexA = indexA
        joint.indexB = indexB
        joint.centerA = simA.center
        joint.centerB = simB.center
        joint.rA = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.rB = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)

        let uA = pulleyDirection(offsetPos(joint.centerA, joint.rA), joint.groundAnchorA)
        let uB = pulleyDirection(offsetPos(joint.centerB, joint.rB), joint.groundAnchorB)
        let ruA = cross(joint.rA, uA)
        let ruB = cross(joint.rB, uB)
        let k = base.invMassA + dot(ruA, mulMV(base.invIA, ruA))
            + joint.ratio * joint.ratio * (base.invMassB + dot(ruB, mulMV(base.invIB, ruB)))
        joint.mass = k > 0 ? 1 / k : 0
        // Only the accumulators of the active mode may be warm started.
        if joint.maxLength - joint.minLength < B3.linearSlop {
            joint.lowerImpulse = 0
            joint.upperImpulse = 0
        } else {
            joint.impulse = 0
        }
        if context.enableWarmStarting == false {
            joint.impulse = 0
            joint.lowerImpulse = 0
            joint.upperImpulse = 0
        }
        base.pulley = joint
    }

    private func pulleyDirection(_ anchor: Pos, _ ground: Pos) -> Vec3 {
        var len: Float = 0
        let u = getLengthAndNormalize(&len, subPos(anchor, ground))
        return len > B3.linearSlop ? u : Vec3()
    }

    /// Current anchors, directions, and rope length.
    @inline(__always)
    private func pulleyGeometry(_ joint: PulleyJointSim, _ stateA: BodyState, _ stateB: BodyState)
        -> (rA: Vec3, rB: Vec3, uA: Vec3, uB: Vec3, length: Float)
    {
        let rA = rotateVector(stateA.deltaRotation, joint.rA)
        let rB = rotateVector(stateB.deltaRotation, joint.rB)
        let pA = offsetPos(offsetPos(joint.centerA, stateA.deltaPosition), rA)
        let pB = offsetPos(offsetPos(joint.centerB, stateB.deltaPosition), rB)
        var lenA: Float = 0
        var lenB: Float = 0
        var uA = getLengthAndNormalize(&lenA, subPos(pA, joint.groundAnchorA))
        var uB = getLengthAndNormalize(&lenB, subPos(pB, joint.groundAnchorB))
        if lenA <= B3.linearSlop { uA = Vec3() }
        if lenB <= B3.linearSlop { uB = Vec3() }
        return (rA, rB, uA, uB, lenA + joint.ratio * lenB)
    }

    private func applyPulleyImpulse(
        _ joint: PulleyJointSim, _ base: JointSim, _ g: (rA: Vec3, rB: Vec3, uA: Vec3, uB: Vec3, length: Float),
        _ impulse: Float, _ stateA: inout BodyState, _ stateB: inout BodyState
    ) {
        let pA = impulse * g.uA
        let pB = (joint.ratio * impulse) * g.uB
        stateA.linearVelocity = mulAdd(stateA.linearVelocity, base.invMassA, pA)
        stateA.angularVelocity = stateA.angularVelocity + mulMV(base.invIA, cross(g.rA, pA))
        stateB.linearVelocity = mulAdd(stateB.linearVelocity, base.invMassB, pB)
        stateB.angularVelocity = stateB.angularVelocity + mulMV(base.invIB, cross(g.rB, pB))
    }

    func warmStartPulleyJoint(_ base: inout JointSim, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let joint = base.pulley
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        let g = pulleyGeometry(joint, stateA, stateB)
        applyPulleyImpulse(joint, base, g, joint.impulse + joint.lowerImpulse - joint.upperImpulse, &stateA, &stateB)
        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
    }

    func solvePulleyJoint(
        _ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>
    ) {
        var joint = base.pulley
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        let g = pulleyGeometry(joint, stateA, stateB)

        func cdot() -> Float {
            dot(g.uA, stateA.linearVelocity + cross(stateA.angularVelocity, g.rA))
                + joint.ratio * dot(g.uB, stateB.linearVelocity + cross(stateB.angularVelocity, g.rB))
        }

        if joint.maxLength - joint.minLength < B3.linearSlop {
            // Taut rope: equality.
            let c = g.length - joint.minLength
            let (bias, massScale, impulseScale) = softTerms(base, c, useBias)
            let impulse = -massScale * joint.mass * (cdot() + bias) - impulseScale * joint.impulse
            joint.impulse += impulse
            applyPulleyImpulse(joint, base, g, impulse, &stateA, &stateB)
        } else {
            // Lower bound: lengthA + ratio * lengthB >= minLength.
            do {
                let c = g.length - joint.minLength
                var bias: Float = 0
                var massScale: Float = 1
                var impulseScale: Float = 0
                if c > 0 {
                    bias = c * context.inv_h
                } else if useBias {
                    (bias, massScale, impulseScale) = softTerms(base, c, true)
                }
                var impulse = -massScale * joint.mass * (cdot() + bias) - impulseScale * joint.lowerImpulse
                let newImpulse = maxFloat(0, joint.lowerImpulse + impulse)
                impulse = newImpulse - joint.lowerImpulse
                joint.lowerImpulse = newImpulse
                applyPulleyImpulse(joint, base, g, impulse, &stateA, &stateB)
            }
            // Upper bound: the rope cannot stretch.
            do {
                let c = joint.maxLength - g.length
                var bias: Float = 0
                var massScale: Float = 1
                var impulseScale: Float = 0
                if c > 0 {
                    bias = c * context.inv_h
                } else if useBias {
                    (bias, massScale, impulseScale) = softTerms(base, c, true)
                }
                var impulse = -massScale * joint.mass * (-cdot() + bias) - impulseScale * joint.upperImpulse
                let newImpulse = maxFloat(0, joint.upperImpulse + impulse)
                impulse = newImpulse - joint.upperImpulse
                joint.upperImpulse = newImpulse
                applyPulleyImpulse(joint, base, g, -impulse, &stateA, &stateB)
            }
        }

        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
        base.pulley = joint
    }

    // MARK: Reactions

    func couplingJointForce(_ sim: JointSim) -> Vec3 {
        switch sim.type {
        case .sixDOFJoint:
            return inv_h * sixDOFImpulses(sim).linear
        case .pathJoint:
            return inv_h * pathJointImpulses(sim).linear
        case .rackAndPinionJoint:
            let xfB = bodyTransformQuick(sim.bodyIdB)
            let axis = rotateVector(xfB.q, rotateVector(sim.localFrameB.q, .axisX))
            return (inv_h * sim.rackAndPinion.impulse) * axis
        case .pulleyJoint:
            let joint = sim.pulley
            let pA = transformWorldPoint(bodyTransformQuick(sim.bodyIdA), sim.localFrameA.p)
            let u = pulleyDirection(pA, joint.groundAnchorA)
            return (inv_h * (joint.impulse + joint.lowerImpulse - joint.upperImpulse)) * u
        default:
            return Vec3()
        }
    }

    func couplingJointTorque(_ sim: JointSim) -> Vec3 {
        switch sim.type {
        case .sixDOFJoint:
            return inv_h * sixDOFImpulses(sim).angular
        case .pathJoint:
            return inv_h * pathJointImpulses(sim).angular
        case .gearJoint:
            let xfA = bodyTransformQuick(sim.bodyIdA)
            let axis = rotateVector(xfA.q, rotateVector(sim.localFrameA.q, .axisZ))
            return (inv_h * sim.gear.impulse) * axis
        case .rackAndPinionJoint:
            let xfA = bodyTransformQuick(sim.bodyIdA)
            let axis = rotateVector(xfA.q, rotateVector(sim.localFrameA.q, .axisZ))
            return (-inv_h * sim.rackAndPinion.ratio * sim.rackAndPinion.impulse) * axis
        default:
            return Vec3()
        }
    }

    /// Linear and angular impulse magnitudes for joint events.
    func couplingJointImpulses(_ sim: JointSim) -> (Float, Float) {
        switch sim.type {
        case .pathJoint:
            let impulses = pathJointImpulses(sim)
            return (length(impulses.linear), length(impulses.angular))
        case .sixDOFJoint:
            let impulses = sixDOFImpulses(sim)
            return (length(impulses.linear), length(impulses.angular))
        case .gearJoint:
            return (0, absFloat(sim.gear.impulse))
        case .rackAndPinionJoint:
            let joint = sim.rackAndPinion
            return (absFloat(joint.impulse), absFloat(joint.ratio * joint.impulse))
        case .pulleyJoint:
            let joint = sim.pulley
            return (absFloat(joint.impulse + joint.lowerImpulse - joint.upperImpulse), 0)
        default:
            return (0, 0)
        }
    }

    // MARK: Debug draw

    func drawCouplingJoint(
        _ sim: JointSim, _ xfA: WorldTransform, _ xfB: WorldTransform, _ pA: Pos, _ pB: Pos, _ scale: Float,
        _ drawSegment: (Pos, Pos, HexColor) -> Void, _ drawPoint: ((Pos, Float, HexColor) -> Void)?
    ) {
        switch sim.type {
        case .gearJoint, .rackAndPinionJoint:
            let length = 0.25 * scale
            let axisA = rotateVector(xfA.q, rotateVector(sim.localFrameA.q, .axisZ))
            let axisB = sim.type == .gearJoint
                ? rotateVector(xfB.q, rotateVector(sim.localFrameB.q, .axisZ))
                : rotateVector(xfB.q, rotateVector(sim.localFrameB.q, .axisX))
            drawSegment(mulSub(pA, length, axisA), mulAdd(pA, length, axisA), DebugColors.orange)
            drawSegment(mulSub(pB, length, axisB), mulAdd(pB, length, axisB), DebugColors.cyan)
            drawSegment(pA, pB, DebugColors.gray)
        case .sixDOFJoint:
            let length = 0.15 * scale
            let frameA = mulWorldTransforms(xfA, sim.localFrameA)
            let frameB = mulWorldTransforms(xfB, sim.localFrameB)
            for (axis, color) in [(Vec3.axisX, DebugColors.red), (Vec3.axisY, DebugColors.green), (Vec3.axisZ, DebugColors.blue)] {
                drawSegment(frameA.p, offsetPos(frameA.p, length * rotateVector(frameA.q, axis)), color)
                drawSegment(frameB.p, offsetPos(frameB.p, 0.6 * length * rotateVector(frameB.q, axis)), color)
            }
            drawSegment(pA, pB, DebugColors.gray)
        case .pathJoint:
            drawPathJoint(sim, xfA, pB, scale, drawSegment, drawPoint)
        case .pulleyJoint:
            let joint = sim.pulley
            drawSegment(pA, joint.groundAnchorA, DebugColors.lightBlue)
            drawSegment(joint.groundAnchorA, joint.groundAnchorB, DebugColors.lightBlue)
            drawSegment(joint.groundAnchorB, pB, DebugColors.lightBlue)
            drawPoint?(joint.groundAnchorA, 6, DebugColors.white)
            drawPoint?(joint.groundAnchorB, 6, DebugColors.white)
        default:
            break
        }
    }

    // MARK: Accessors

    private func couplingSim(_ jointId: JointID, _ type: JointType) -> (index: Int32, sim: JointSim)? {
        let index = jointId.index1 - 1
        guard jointId.world0 == worldId, index >= 0, Int(index) < joints.count else { return nil }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.generation == jointId.generation, joint.type == type else {
            return nil
        }
        return (index, readJointSim(index))
    }

    public func gearJointRatio(_ jointId: JointID) -> Float? {
        couplingSim(jointId, .gearJoint)?.sim.gear.ratio
    }

    /// Change the gear ratio. The current angles become the new reference.
    @discardableResult
    public func setGearJointRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        guard ratio.isFinite, var (_, sim) = couplingSim(jointId, .gearJoint) else { return false }
        sim.gear.ratio = ratio
        sim.gear.offset = sim.gear.probeA.angle + ratio * sim.gear.probeB.angle
        sim.gear.impulse = 0
        writeJointSim(jointId.index1 - 1, sim)
        return true
    }

    public func rackAndPinionJointRatio(_ jointId: JointID) -> Float? {
        couplingSim(jointId, .rackAndPinionJoint)?.sim.rackAndPinion.ratio
    }

    @discardableResult
    public func setRackAndPinionJointRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        guard ratio.isFinite, var (_, sim) = couplingSim(jointId, .rackAndPinionJoint) else { return false }
        sim.rackAndPinion.ratio = ratio
        sim.rackAndPinion.offset = sim.rackAndPinion.rack.value - ratio * sim.rackAndPinion.pinion.angle
        sim.rackAndPinion.impulse = 0
        writeJointSim(jointId.index1 - 1, sim)
        return true
    }

    public func pulleyJointRatio(_ jointId: JointID) -> Float? {
        couplingSim(jointId, .pulleyJoint)?.sim.pulley.ratio
    }

    public func pulleyJointGroundAnchors(_ jointId: JointID) -> (a: Pos, b: Pos)? {
        guard let sim = couplingSim(jointId, .pulleyJoint)?.sim else { return nil }
        return (sim.pulley.groundAnchorA, sim.pulley.groundAnchorB)
    }

    /// Current rope segment lengths from the ground anchors.
    public func pulleyJointCurrentLengths(_ jointId: JointID) -> (a: Float, b: Float)? {
        guard let sim = couplingSim(jointId, .pulleyJoint)?.sim else { return nil }
        let pA = transformWorldPoint(bodyTransformQuick(sim.bodyIdA), sim.localFrameA.p)
        let pB = transformWorldPoint(bodyTransformQuick(sim.bodyIdB), sim.localFrameB.p)
        return (distance(pA, sim.pulley.groundAnchorA), distance(pB, sim.pulley.groundAnchorB))
    }

    public func pulleyJointLengthRange(_ jointId: JointID) -> (min: Float, max: Float)? {
        guard let sim = couplingSim(jointId, .pulleyJoint)?.sim else { return nil }
        return (sim.pulley.minLength, sim.pulley.maxLength)
    }

    /// Allow slack (min < max) or change the rope length.
    @discardableResult
    public func setPulleyJointLengthRange(_ jointId: JointID, _ minLength: Float, _ maxLength: Float) -> Bool {
        guard minLength > 0, maxLength >= minLength, var (_, sim) = couplingSim(jointId, .pulleyJoint) else { return false }
        sim.pulley.minLength = minLength
        sim.pulley.maxLength = maxLength
        writeJointSim(jointId.index1 - 1, sim)
        wakeJointBodies(jointId)
        return true
    }
}
