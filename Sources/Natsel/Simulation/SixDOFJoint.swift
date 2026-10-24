// Six degree of freedom joint. The joint space is body A's local frame: translations are measured
// along its x, y, z axes and rotations about them. Every axis can be free, limited, or locked, and
// can carry a velocity or position motor and friction. Rotation limits act on the relative rotation
// quaternion component of each axis (exact for single axis rotation); swing can instead be limited by
// a cone around the x axis.

public enum SixDOFAxis: Int, Sendable, CaseIterable {
    case translationX
    case translationY
    case translationZ
    case rotationX
    case rotationY
    case rotationZ
}

public enum SixDOFMotion: Sendable, BitwiseCopyable {
    case free
    /// Between `lowerLimit` and `upperLimit`.
    case limited
    case locked
}

public enum SixDOFMotorMode: Sendable, BitwiseCopyable {
    case off
    /// Drive the axis speed to `motorTarget`.
    case velocity
    /// Spring the axis toward `motorTarget` (translation) or `SixDOFJointDef.targetRotation` (rotation).
    case position
}

public struct SixDOFAxisSettings: Sendable, BitwiseCopyable {
    public var motion = SixDOFMotion.free
    /// Limits in meters or radians. Rotation limits must lie in (-pi, pi).
    public var lowerLimit: Float = 0
    public var upperLimit: Float = 0
    /// Soft limits. Zero hertz makes the limits rigid.
    public var limitHertz: Float = 0
    public var limitDampingRatio: Float = 0

    public var motorMode = SixDOFMotorMode.off
    /// Target speed (velocity mode) or target translation (position mode, translation axes).
    public var motorTarget: Float = 0
    /// Largest motor force (N) or torque (N m).
    public var maxMotorForce: Float = 0
    /// Spring of the position motor. As with other joint springs the frequency is relative to the
    /// axis' own effective mass; a rotation spring on an anchor far from the center of mass, held by
    /// translation constraints, moves the larger inertia about the anchor and is softer than this.
    public var motorHertz: Float = 2
    public var motorDampingRatio: Float = 1

    /// Force (N) or torque (N m) resisting motion along or about the axis.
    public var maxFriction: Float = 0

    public init() {}

    public init(_ motion: SixDOFMotion, lower: Float = 0, upper: Float = 0) {
        self.motion = motion
        lowerLimit = lower
        upperLimit = upper
    }
}

public enum SixDOFSwingLimit: Sendable, BitwiseCopyable {
    /// Rotation y and z use their own limits.
    case pyramid
    /// The x axes of the two frames stay within `angle` of each other. Rotation y and z limits are
    /// ignored; locking either still locks it.
    case cone(angle: Float)
}

public struct SixDOFJointDef {
    public var base = JointDef()
    /// Indexed by `SixDOFAxis.rawValue`. All axes start free.
    public var axes = [SixDOFAxisSettings](repeating: SixDOFAxisSettings(), count: 6)
    public var swingLimit = SixDOFSwingLimit.pyramid
    /// Target of frame B relative to frame A for rotation position motors.
    public var targetRotation = Quat.identity

    public init() {}

    public subscript(_ axis: SixDOFAxis) -> SixDOFAxisSettings {
        get { axes[axis.rawValue] }
        set { axes[axis.rawValue] = newValue }
    }

    /// Convenience: lock every axis, then free the listed ones.
    public mutating func lockAll(except free: [SixDOFAxis] = []) {
        for axis in SixDOFAxis.allCases {
            axes[axis.rawValue].motion = free.contains(axis) ? .free : .locked
        }
    }
}

/// Solver data kept in the joint record.
struct SixDOFJointSim: BitwiseCopyable {
    var slot: Int32 = B3.nullIndex
    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var frameA = Transform()
    var frameB = Transform()
    var deltaCenter = Vec3()

    init() {}
}

/// Per axis accumulated impulses and softness.
struct SixDOFAxisState: BitwiseCopyable {
    var settings = SixDOFAxisSettings()
    var lockImpulse: Float = 0
    var lowerImpulse: Float = 0
    var upperImpulse: Float = 0
    var motorImpulse: Float = 0
    var frictionImpulse: Float = 0
    var limitSoftness = Softness()
    var motorSoftness = Softness()

    /// Impulse along the positional Jacobian (lock and limits).
    var positionalImpulse: Float { lockImpulse + lowerImpulse - upperImpulse }
    /// Impulse along the axis (motor and friction).
    var axialImpulse: Float { motorImpulse + frictionImpulse }
}

struct SixDOFJointData: BitwiseCopyable {
    var axes = InlineArray<6, SixDOFAxisState>(repeating: SixDOFAxisState())
    var coneAngle: Float = 0
    var useCone = false
    var coneImpulse: Float = 0
    var targetRotation = Quat.identity
    /// Block solves when all three axes of a kind are locked.
    var linearBlockImpulse = Vec3()
    var angularBlockImpulse = Vec3()

    var linearLocked: Bool {
        axes[0].settings.motion == .locked && axes[1].settings.motion == .locked && axes[2].settings.motion == .locked
    }

    var angularLocked: Bool {
        axes[3].settings.motion == .locked && axes[4].settings.motion == .locked && axes[5].settings.motion == .locked
    }

    init() {}
}

/// Derivative of relQ.v[i] with respect to the relative angular velocity:
/// d(relQ.v_i)/dt = dot(J_i, wB - wA).
@inline(__always)
private func quaternionJacobian(_ frame: Quat, _ rel: Quat, _ axis: Vec3) -> Vec3 {
    0.5 * rotateVector(frame, rel.s * axis + cross(rel.v, axis))
}

@inline(__always)
private func component(_ v: Vec3, _ i: Int) -> Float { i == 0 ? v.x : (i == 1 ? v.y : v.z) }

@inline(__always)
private func unitAxis(_ i: Int) -> Vec3 { i == 0 ? .axisX : (i == 1 ? .axisY : .axisZ) }

extension World {
    // MARK: Storage

    private func allocateSixDOFData(_ data: SixDOFJointData) -> Int32 {
        let slot = sixDOFPool.alloc()
        if Int(slot) == sixDOFStore.count {
            sixDOFStore.append(data)
        } else {
            sixDOFStore[Int(slot)] = data
        }
        return slot
    }

    func releaseSixDOFData(_ slot: Int32) {
        guard slot >= 0 && Int(slot) < sixDOFStore.count else { return }
        sixDOFStore[Int(slot)] = SixDOFJointData()
        sixDOFPool.free(slot)
    }

    // MARK: Creation

    @discardableResult
    public func createSixDOFJoint(_ def: SixDOFJointDef) -> JointID {
        guard isValidBody(def.base.bodyIdA), isValidBody(def.base.bodyIdB), def.base.bodyIdA != def.base.bodyIdB,
            def.axes.count == 6
        else { return JointID() }

        var data = SixDOFJointData()
        for i in 0..<6 {
            var settings = def.axes[i]
            if settings.motion == .limited && settings.upperLimit < settings.lowerLimit {
                swap(&settings.lowerLimit, &settings.upperLimit)
            }
            data.axes[i].settings = settings
        }
        if case .cone(let angle) = def.swingLimit {
            data.useCone = true
            data.coneAngle = clampFloat(angle, 0, B3.pi)
        }
        data.targetRotation = normalizeQuat(def.targetRotation)

        let pair = createJoint(def.base, .sixDOFJoint)
        var sim = pair.sim
        var joint = SixDOFJointSim()
        joint.slot = allocateSixDOFData(data)
        sim.sixDOF = joint
        writeJointSim(pair.jointId, sim)
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: joints[Int(pair.jointId)].generation)
    }

    // MARK: Prepare

    func prepareSixDOFJoint(_ base: inout JointSim, _ context: StepContext) {
        let bodyA = bodies[Int(base.bodyIdA)]
        let bodyB = bodies[Int(base.bodyIdB)]
        let simA = solverSets[Int(bodyA.setIndex)].bodySims[Int(bodyA.localIndex)]
        let simB = solverSets[Int(bodyB.setIndex)].bodySims[Int(bodyB.localIndex)]
        base.invMassA = simA.invMass
        base.invMassB = simB.invMass
        base.invIA = simA.invInertiaWorld
        base.invIB = simB.invInertiaWorld

        var joint = base.sixDOF
        joint.indexA = bodyA.setIndex == SetType.awakeSet.rawValue ? bodyA.localIndex : B3.nullIndex
        joint.indexB = bodyB.setIndex == SetType.awakeSet.rawValue ? bodyB.localIndex : B3.nullIndex
        joint.frameA.q = mulQuat(simA.transform.q, base.localFrameA.q)
        joint.frameA.p = rotateVector(simA.transform.q, base.localFrameA.p - simA.localCenter)
        joint.frameB.q = mulQuat(simB.transform.q, base.localFrameB.q)
        joint.frameB.p = rotateVector(simB.transform.q, base.localFrameB.p - simB.localCenter)
        joint.deltaCenter = subPos(simB.center, simA.center)
        base.sixDOF = joint

        let slot = Int(joint.slot)
        for i in 0..<6 {
            var axis = sixDOFStore[slot].axes[i]
            axis.limitSoftness = makeSoft(axis.settings.limitHertz, axis.settings.limitDampingRatio, context.h)
            axis.motorSoftness = makeSoft(axis.settings.motorHertz, axis.settings.motorDampingRatio, context.h)
            if context.enableWarmStarting == false {
                axis.lockImpulse = 0
                axis.lowerImpulse = 0
                axis.upperImpulse = 0
                axis.motorImpulse = 0
                axis.frictionImpulse = 0
            }
            if axis.settings.motion != .limited {
                axis.lowerImpulse = 0
                axis.upperImpulse = 0
            }
            if axis.settings.motion != .locked { axis.lockImpulse = 0 }
            if axis.settings.motorMode == .off { axis.motorImpulse = 0 }
            if axis.settings.maxFriction <= 0 { axis.frictionImpulse = 0 }
            sixDOFStore[slot].axes[i] = axis
        }
        if context.enableWarmStarting == false {
            sixDOFStore[slot].coneImpulse = 0
            sixDOFStore[slot].linearBlockImpulse = Vec3()
            sixDOFStore[slot].angularBlockImpulse = Vec3()
        }
        if sixDOFStore[slot].linearLocked == false { sixDOFStore[slot].linearBlockImpulse = Vec3() }
        if sixDOFStore[slot].angularLocked == false { sixDOFStore[slot].angularBlockImpulse = Vec3() }
    }

    // MARK: Geometry

    private struct SixDOFFrame {
        var quatA: Quat
        var quatB: Quat
        var rel: Quat
        var rA: Vec3
        var rB: Vec3
        var d: Vec3
        var axes: Matrix3
    }

    @inline(__always)
    private func sixDOFFrame(_ joint: SixDOFJointSim, _ stateA: BodyState, _ stateB: BodyState) -> SixDOFFrame {
        let quatA = mulQuat(stateA.deltaRotation, joint.frameA.q)
        var quatB = mulQuat(stateB.deltaRotation, joint.frameB.q)
        if dotQuat(quatA, quatB) < 0 { quatB = negateQuat(quatB) }
        let rA = rotateVector(stateA.deltaRotation, joint.frameA.p)
        let rB = rotateVector(stateB.deltaRotation, joint.frameB.p)
        let d = ((stateB.deltaPosition - stateA.deltaPosition) + joint.deltaCenter) + (rB - rA)
        return SixDOFFrame(quatA: quatA, quatB: quatB, rel: invMulQuat(quatA, quatB), rA: rA, rB: rB, d: d,
                           axes: makeMatrixFromQuat(quatA))
    }

    // MARK: Warm start

    func warmStartSixDOFJoint(_ base: inout JointSim, _ states: UnsafeMutableBufferPointer<BodyState>) {
        let joint = base.sixDOF
        let data = sixDOFStore[Int(joint.slot)]
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        let f = sixDOFFrame(joint, stateA, stateB)

        var linear = Vec3()
        var angularA = Vec3()
        var angularB = Vec3()

        if data.linearLocked {
            linear = data.linearBlockImpulse
            angularA = cross(f.rA, linear)
            angularB = cross(f.rB, linear)
        }
        for i in 0..<3 {
            let state = data.axes[i]
            let total = (data.linearLocked ? 0 : state.positionalImpulse) + state.axialImpulse
            guard total != 0 else { continue }
            let axis = i == 0 ? f.axes.cx : (i == 1 ? f.axes.cy : f.axes.cz)
            linear = mulAdd(linear, total, axis)
            angularA = mulAdd(angularA, total, cross(f.d + f.rA, axis))
            angularB = mulAdd(angularB, total, cross(f.rB, axis))
        }

        var angular = Vec3()
        if data.angularLocked {
            let block = data.angularBlockImpulse
            angular = block.x * quaternionJacobian(f.quatA, f.rel, .axisX)
                + block.y * quaternionJacobian(f.quatA, f.rel, .axisY)
                + block.z * quaternionJacobian(f.quatA, f.rel, .axisZ)
        }
        for i in 0..<3 {
            let state = data.axes[3 + i]
            let axis = i == 0 ? f.axes.cx : (i == 1 ? f.axes.cy : f.axes.cz)
            if state.settings.motorMode == .position && state.motorImpulse != 0 {
                // The position motor acts along the Jacobian of the rotation error.
                var error = invMulQuat(data.targetRotation, f.rel)
                if error.s < 0 { error = negateQuat(error) }
                let jacobian = 2 * quaternionJacobian(mulQuat(f.quatA, data.targetRotation), error, unitAxis(i))
                angular = mulAdd(angular, state.motorImpulse, jacobian)
                angular = mulAdd(angular, state.frictionImpulse, axis)
            } else {
                angular = mulAdd(angular, state.axialImpulse, axis)
            }
            if data.angularLocked == false && state.positionalImpulse != 0 {
                angular = mulAdd(angular, state.positionalImpulse, quaternionJacobian(f.quatA, f.rel, unitAxis(i)))
            }
        }
        if data.useCone && data.coneImpulse != 0 {
            let xA = rotateVector(f.quatA, .axisX)
            let xB = rotateVector(f.quatB, .axisX)
            var len: Float = 0
            let coneAxis = getLengthAndNormalize(&len, cross(xA, xB))
            if len > 1.0e-6 { angular = mulSub(angular, data.coneImpulse, coneAxis) }
        }

        stateA.linearVelocity = mulSub(stateA.linearVelocity, base.invMassA, linear)
        stateA.angularVelocity = stateA.angularVelocity - mulMV(base.invIA, angularA + angular)
        stateB.linearVelocity = mulAdd(stateB.linearVelocity, base.invMassB, linear)
        stateB.angularVelocity = stateB.angularVelocity + mulMV(base.invIB, angularB + angular)
        World.setStateAt(states, joint.indexA, stateA)
        World.setStateAt(states, joint.indexB, stateB)
    }

    // MARK: Solve

    func solveSixDOFJoint(
        _ base: inout JointSim, _ context: StepContext, _ useBias: Bool, _ states: UnsafeMutableBufferPointer<BodyState>
    ) {
        let joint = base.sixDOF
        let slot = Int(joint.slot)
        var data = sixDOFStore[slot]
        var stateA = World.stateAt(states, joint.indexA)
        var stateB = World.stateAt(states, joint.indexB)
        let mA = base.invMassA
        let mB = base.invMassB
        let iA = base.invIA
        let iB = base.invIB
        let iSum = addMM(iA, iB)
        let h = context.h
        let soft = base.constraintSoftness

        var vA = stateA.linearVelocity
        var wA = stateA.angularVelocity
        var vB = stateB.linearVelocity
        var wB = stateB.angularVelocity

        let f = sixDOFFrame(joint, stateA, stateB)

        // Bias terms for positional constraints: speculative when separated, soft when violated.
        func limitTerms(_ c: Float, _ limitSoftness: Softness, _ springy: Bool) -> (Float, Float, Float) {
            if c > 0 { return (c * context.inv_h, 1, 0) }
            if springy { return (limitSoftness.biasRate * c, limitSoftness.massScale, limitSoftness.impulseScale) }
            if useBias { return (soft.biasRate * c, soft.massScale, soft.impulseScale) }
            return (0, 1, 0)
        }

        // MARK: Angular

        // Velocity motors and friction about the frame A axes.
        for i in 0..<3 {
            var state = data.axes[3 + i]
            let s = state.settings
            guard s.motorMode != .off || s.maxFriction > 0 else { continue }
            let axis = i == 0 ? f.axes.cx : (i == 1 ? f.axes.cy : f.axes.cz)
            let k = dot(axis, mulMV(iSum, axis))
            guard k > 0 else { continue }
            let mass = 1 / k

            if s.motorMode == .velocity {
                let cdot = dot(wB - wA, axis) - s.motorTarget
                let maxImpulse = s.maxMotorForce * h
                let old = state.motorImpulse
                state.motorImpulse = clampFloat(old - mass * cdot, -maxImpulse, maxImpulse)
                let impulse = state.motorImpulse - old
                wA = wA - mulMV(iA, impulse * axis)
                wB = wB + mulMV(iB, impulse * axis)
            } else if s.motorMode == .position {
                // Spring the axis component of the rotation error toward the target.
                var error = invMulQuat(data.targetRotation, f.rel)
                if error.s < 0 { error = negateQuat(error) }
                // Twice the quaternion component is the angle for small errors, so the impulse is
                // a torque impulse and `maxMotorForce` is a true torque limit.
                let frame = mulQuat(f.quatA, data.targetRotation)
                let jacobian = 2 * quaternionJacobian(frame, error, unitAxis(i))
                let kj = dot(jacobian, mulMV(iSum, jacobian))
                if kj > 0 {
                    let c = 2 * component(error.v, i)
                    let cdot = dot(wB - wA, jacobian)
                    let ms = state.motorSoftness
                    let maxImpulse = s.maxMotorForce * h
                    let old = state.motorImpulse
                    let delta = -ms.massScale / kj * (cdot + ms.biasRate * c) - ms.impulseScale * old
                    state.motorImpulse = clampFloat(old + delta, -maxImpulse, maxImpulse)
                    let impulse = state.motorImpulse - old
                    wA = wA - mulMV(iA, impulse * jacobian)
                    wB = wB + mulMV(iB, impulse * jacobian)
                }
            }

            if s.maxFriction > 0 {
                let cdot = dot(wB - wA, axis)
                let maxImpulse = s.maxFriction * h
                let old = state.frictionImpulse
                state.frictionImpulse = clampFloat(old - mass * cdot, -maxImpulse, maxImpulse)
                let impulse = state.frictionImpulse - old
                wA = wA - mulMV(iA, impulse * axis)
                wB = wB + mulMV(iB, impulse * axis)
            }
            data.axes[3 + i] = state
        }

        // Swing cone.
        if data.useCone && (data.axes[4].settings.motion == .limited || data.axes[5].settings.motion == .limited) {
            let xA = rotateVector(f.quatA, .axisX)
            let xB = rotateVector(f.quatB, .axisX)
            var len: Float = 0
            let coneAxis = getLengthAndNormalize(&len, cross(xA, xB))
            if len > 1.0e-6 {
                let angle = atan2Approx(len, dot(xA, xB))
                let c = data.coneAngle - angle
                let (bias, massScale, impulseScale) = limitTerms(c, data.axes[4].limitSoftness, data.axes[4].settings.limitHertz > 0)
                let k = dot(coneAxis, mulMV(iSum, coneAxis))
                if k > 0 {
                    let cdot = -dot(wB - wA, coneAxis)
                    let old = data.coneImpulse
                    let delta = maxFloat(-massScale / k * (cdot + bias) - impulseScale * old, -old)
                    data.coneImpulse = old + delta
                    wA = wA + mulMV(iA, delta * coneAxis)
                    wB = wB - mulMV(iB, delta * coneAxis)
                }
            }
        } else {
            data.coneImpulse = 0
        }

        // Locks and limits on the relative rotation components.
        let relQ = f.rel
        if data.angularLocked {
            let jx = quaternionJacobian(f.quatA, relQ, .axisX)
            let jy = quaternionJacobian(f.quatA, relQ, .axisY)
            let jz = quaternionJacobian(f.quatA, relQ, .axisZ)
            let gx = mulMV(iSum, jx)
            let gy = mulMV(iSum, jy)
            let gz = mulMV(iSum, jz)
            let k = Matrix3(Vec3(dot(jx, gx), dot(jy, gx), dot(jz, gx)),
                            Vec3(dot(jx, gy), dot(jy, gy), dot(jz, gy)),
                            Vec3(dot(jx, gz), dot(jy, gz), dot(jz, gz)))
            let wRel = wB - wA
            let cdot = Vec3(dot(wRel, jx), dot(wRel, jy), dot(wRel, jz))
            var bias = Vec3()
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias {
                bias = soft.biasRate * relQ.v
                massScale = soft.massScale
                impulseScale = soft.impulseScale
            }
            let solution = solve3(k, cdot + bias)
            let delta = (-massScale) * solution - impulseScale * data.angularBlockImpulse
            data.angularBlockImpulse = data.angularBlockImpulse + delta
            let angular = delta.x * jx + delta.y * jy + delta.z * jz
            wA = wA - mulMV(iA, angular)
            wB = wB + mulMV(iB, angular)
        } else {
            for i in 0..<3 {
                var state = data.axes[3 + i]
                let s = state.settings
                let swing = i > 0
                if s.motion == .free || (swing && data.useCone && s.motion == .limited) { continue }
                let jacobian = quaternionJacobian(f.quatA, relQ, unitAxis(i))
                let k = dot(jacobian, mulMV(iSum, jacobian))
                guard k > 0 else { continue }
                let value = component(relQ.v, i)
                let cdot = dot(wB - wA, jacobian)

                if s.motion == .locked {
                    var bias: Float = 0
                    var massScale: Float = 1
                    var impulseScale: Float = 0
                    if useBias {
                        bias = soft.biasRate * value
                        massScale = soft.massScale
                        impulseScale = soft.impulseScale
                    }
                    let delta = -massScale / k * (cdot + bias) - impulseScale * state.lockImpulse
                    state.lockImpulse += delta
                    wA = wA - mulMV(iA, delta * jacobian)
                    wB = wB + mulMV(iB, delta * jacobian)
                } else {
                    let springy = s.limitHertz > 0
                    let lower = computeCosSin(0.5 * s.lowerLimit).sine
                    let upper = computeCosSin(0.5 * s.upperLimit).sine
                    do {
                        let (bias, massScale, impulseScale) = limitTerms(value - lower, state.limitSoftness, springy)
                        let old = state.lowerImpulse
                        let delta = maxFloat(-massScale / k * (cdot + bias) - impulseScale * old, -old)
                        state.lowerImpulse = old + delta
                        wA = wA - mulMV(iA, delta * jacobian)
                        wB = wB + mulMV(iB, delta * jacobian)
                    }
                    do {
                        let cdotUpper = -dot(wB - wA, jacobian)
                        let (bias, massScale, impulseScale) = limitTerms(upper - value, state.limitSoftness, springy)
                        let old = state.upperImpulse
                        let delta = maxFloat(-massScale / k * (cdotUpper + bias) - impulseScale * old, -old)
                        state.upperImpulse = old + delta
                        wA = wA + mulMV(iA, delta * jacobian)
                        wB = wB - mulMV(iB, delta * jacobian)
                    }
                }
                data.axes[3 + i] = state
            }
        }

        // MARK: Linear

        let d = f.d
        for i in 0..<3 {
            var state = data.axes[i]
            let s = state.settings
            let axis = i == 0 ? f.axes.cx : (i == 1 ? f.axes.cy : f.axes.cz)
            let sA = cross(d + f.rA, axis)
            let sB = cross(f.rB, axis)
            let k = mA + mB + dot(sA, mulMV(iA, sA)) + dot(sB, mulMV(iB, sB))
            guard k > 0 else { continue }
            let mass = 1 / k
            let translation = dot(d, axis)

            func apply(_ impulse: Float) {
                vA = mulSub(vA, mA * impulse, axis)
                wA = wA - mulMV(iA, impulse * sA)
                vB = mulAdd(vB, mB * impulse, axis)
                wB = wB + mulMV(iB, impulse * sB)
            }
            func cdot() -> Float { dot(vB - vA, axis) + dot(wB, sB) - dot(wA, sA) }

            if s.motorMode == .velocity {
                let maxImpulse = s.maxMotorForce * h
                let old = state.motorImpulse
                state.motorImpulse = clampFloat(old - mass * (cdot() - s.motorTarget), -maxImpulse, maxImpulse)
                apply(state.motorImpulse - old)
            } else if s.motorMode == .position {
                let ms = state.motorSoftness
                let c = translation - s.motorTarget
                let maxImpulse = s.maxMotorForce * h
                let old = state.motorImpulse
                let delta = -ms.massScale * mass * (cdot() + ms.biasRate * c) - ms.impulseScale * old
                state.motorImpulse = clampFloat(old + delta, -maxImpulse, maxImpulse)
                apply(state.motorImpulse - old)
            }
            if s.maxFriction > 0 {
                let maxImpulse = s.maxFriction * h
                let old = state.frictionImpulse
                state.frictionImpulse = clampFloat(old - mass * cdot(), -maxImpulse, maxImpulse)
                apply(state.frictionImpulse - old)
            }

            if data.linearLocked == false {
                if s.motion == .locked {
                    var bias: Float = 0
                    var massScale: Float = 1
                    var impulseScale: Float = 0
                    if useBias {
                        bias = soft.biasRate * translation
                        massScale = soft.massScale
                        impulseScale = soft.impulseScale
                    }
                    let delta = -massScale * mass * (cdot() + bias) - impulseScale * state.lockImpulse
                    state.lockImpulse += delta
                    apply(delta)
                } else if s.motion == .limited {
                    let springy = s.limitHertz > 0
                    do {
                        let (bias, massScale, impulseScale) = limitTerms(translation - s.lowerLimit, state.limitSoftness, springy)
                        let old = state.lowerImpulse
                        let delta = maxFloat(-massScale * mass * (cdot() + bias) - impulseScale * old, -old)
                        state.lowerImpulse = old + delta
                        apply(delta)
                    }
                    do {
                        let (bias, massScale, impulseScale) = limitTerms(s.upperLimit - translation, state.limitSoftness, springy)
                        let old = state.upperImpulse
                        let delta = maxFloat(-massScale * mass * (-cdot() + bias) - impulseScale * old, -old)
                        state.upperImpulse = old + delta
                        apply(-delta)
                    }
                }
            }
            data.axes[i] = state
        }

        if data.linearLocked {
            // Point to point, as a 3-by-3 block.
            let cdot = (vB + cross(wB, f.rB)) - vA - cross(wA, f.rA)
            var bias = Vec3()
            var massScale: Float = 1
            var impulseScale: Float = 0
            if useBias {
                bias = soft.biasRate * d
                massScale = soft.massScale
                impulseScale = soft.impulseScale
            }
            let b = solvePointConstraint(mA, mB, iA, iB, f.rA, f.rB, cdot + bias)
            let impulse = (-massScale) * b - impulseScale * data.linearBlockImpulse
            data.linearBlockImpulse = data.linearBlockImpulse + impulse
            vA = vA - mA * impulse
            wA = wA - mulMV(iA, cross(f.rA, impulse))
            vB = vB + mB * impulse
            wB = wB + mulMV(iB, cross(f.rB, impulse))
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
        sixDOFStore[slot] = data
    }

    // MARK: Reactions

    /// World space linear and angular impulse of the last substep.
    func sixDOFImpulses(_ sim: JointSim) -> (linear: Vec3, angular: Vec3) {
        let slot = Int(sim.sixDOF.slot)
        guard slot >= 0 && slot < sixDOFStore.count else { return (Vec3(), Vec3()) }
        let data = sixDOFStore[slot]
        let q = mulQuat(bodyTransformQuick(sim.bodyIdA).q, sim.localFrameA.q)
        let axes = makeMatrixFromQuat(q)
        var linear = data.linearBlockImpulse
        var angular = Vec3()
        for i in 0..<3 {
            let axis = i == 0 ? axes.cx : (i == 1 ? axes.cy : axes.cz)
            linear = mulAdd(linear, data.axes[i].positionalImpulse + data.axes[i].axialImpulse, axis)
            angular = mulAdd(angular, data.axes[3 + i].axialImpulse, axis)
            // Quaternion component impulses act about roughly half the axis.
            angular = mulAdd(angular, 0.5 * (data.axes[3 + i].positionalImpulse + component(data.angularBlockImpulse, i)), axis)
        }
        return (linear, angular)
    }

    // MARK: Accessors

    private func sixDOFSlot(_ jointId: JointID) -> Int? {
        let index = jointId.index1 - 1
        guard jointId.world0 == worldId, index >= 0, Int(index) < joints.count else { return nil }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.generation == jointId.generation, joint.type == .sixDOFJoint
        else { return nil }
        let slot = Int(readJointSim(index).sixDOF.slot)
        return slot >= 0 && slot < sixDOFStore.count ? slot : nil
    }

    public func sixDOFJointAxis(_ jointId: JointID, _ axis: SixDOFAxis) -> SixDOFAxisSettings? {
        sixDOFSlot(jointId).map { sixDOFStore[$0].axes[axis.rawValue].settings }
    }

    /// Change one axis at run time, for example to unlock a door or retarget a motor.
    @discardableResult
    public func setSixDOFJointAxis(_ jointId: JointID, _ axis: SixDOFAxis, _ settings: SixDOFAxisSettings) -> Bool {
        guard let slot = sixDOFSlot(jointId) else { return false }
        var s = settings
        if s.motion == .limited && s.upperLimit < s.lowerLimit { swap(&s.lowerLimit, &s.upperLimit) }
        sixDOFStore[slot].axes[axis.rawValue].settings = s
        wakeJointBodies(jointId)
        return true
    }

    @discardableResult
    public func setSixDOFJointMotorTarget(_ jointId: JointID, _ axis: SixDOFAxis, _ target: Float) -> Bool {
        guard let slot = sixDOFSlot(jointId) else { return false }
        sixDOFStore[slot].axes[axis.rawValue].settings.motorTarget = target
        wakeJointBodies(jointId)
        return true
    }

    @discardableResult
    public func setSixDOFJointTargetRotation(_ jointId: JointID, _ rotation: Quat) -> Bool {
        guard let slot = sixDOFSlot(jointId), isValidQuat(rotation) else { return false }
        sixDOFStore[slot].targetRotation = normalizeQuat(rotation)
        wakeJointBodies(jointId)
        return true
    }

    /// Current translation of frame B in frame A, and the rotation of frame B relative to frame A.
    public func sixDOFJointPose(_ jointId: JointID) -> Transform? {
        guard let sim = jointSim(jointId), sim.type == .sixDOFJoint else { return nil }
        let frameA = mulWorldTransforms(bodyTransformQuick(sim.bodyIdA), sim.localFrameA)
        let frameB = mulWorldTransforms(bodyTransformQuick(sim.bodyIdB), sim.localFrameB)
        return invMulWorldTransforms(frameA, frameB)
    }
}
