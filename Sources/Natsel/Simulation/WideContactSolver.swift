// Persistent four-contact SoA batches. Only graph colors with disjoint dynamic bodies
// are packed; the overflow color always uses the sequential solver.

struct ConstraintPointW {
    var rA = Vec3W()
    var rB = Vec3W()
    var baseSeparation = FloatW.zero
    var relativeVelocity = FloatW.zero
    var normalImpulse = FloatW.zero
    var totalNormalImpulse = FloatW.zero
    var restitutionImpulse = FloatW.zero
    var normalMass = FloatW.zero
    var leverArm = FloatW.zero
}

struct ManifoldConstraintW {
    var points = InlineArray<4, ConstraintPointW>(repeating: ConstraintPointW())
    // Distinguish an absent lane from a real manifold containing zero active points.
    var exists = SIMD4<Int32>.zero
    var pointCounts = SIMD4<Int32>.zero
    var pointCount = 0
    var normal = Vec3W()
    var tangent1 = Vec3W()
    var tangent2 = Vec3W()
    var centerA = Vec3W()
    var centerB = Vec3W()
    var twistMass = FloatW.zero
    var twistImpulse = FloatW.zero
    var tangentMass = SymMatrix2W()
    var frictionImpulse = Vec2W()
    var rollingImpulse = Vec3W()
    var tangentVelocity1 = FloatW.zero
    var tangentVelocity2 = FloatW.zero
}

struct ContactConstraintW {
    var indexA = SIMD4<Int32>(repeating: -1)
    var indexB = SIMD4<Int32>(repeating: -1)
    var contactIds = SIMD4<Int32>(repeating: -1)
    var laneManifoldCounts = SIMD4<Int32>.zero
    var invMassA = FloatW.zero
    var invMassB = FloatW.zero
    var invIA = SymMatrix3W()
    var invIB = SymMatrix3W()
    var rollingMass = SymMatrix3W()
    var biasRate = FloatW.zero
    var massScale = FloatW.zero
    var impulseScale = FloatW.zero
    var friction = FloatW.zero
    var restitution = FloatW.zero
    var rollingResistance = FloatW.zero
    var anyRolling = false
    var anyRestitution = false
    var manifoldStart = 0
    var manifoldCount = 0
}

extension SymMatrix3W {
    @inline(__always)
    mutating func setLane(_ lane: Int, _ m: SymMatrix3) {
        xx[lane] = m.cxx
        yy[lane] = m.cyy
        zz[lane] = m.czz
        xy[lane] = m.cxy
        xz[lane] = m.cxz
        yz[lane] = m.cyz
    }
}

extension World {
    @inline(__always)
    func usesWideContacts(_ color: Int32, _ mesh: Bool) -> Bool {
        enableWideSolver && color != overflowIndex
            && (mesh ? solverContacts.wideMesh[Int(color)] : solverContacts.wideConvex[Int(color)])
    }


    /// Lend the wide constraints of one color and kind with fresh pointers (the serial path).
    @inline(__always)
    private func withWideContacts(
        _ color: Int32, _ mesh: Bool,
        _ body: (
            UnsafePointer<ContactConstraintW>, Range<Int>, UnsafeMutableBufferPointer<BodyState>,
            UnsafeMutablePointer<ManifoldConstraintW>
        ) -> Void
    ) {
        let ci = Int(color)
        let list = solverContacts.wideConstraints[ci]
        let split = solverContacts.wideConvexCounts[ci]
        let range = mesh ? split..<list.count : 0..<split
        guard range.isEmpty == false else { return }
        withSolverStates { states in
            solverContacts.wideManifolds.withUnsafeMutableBufferPointer { pool in
                list.withUnsafeBufferPointer { listBuffer in
                    body(listBuffer.baseAddress!, range, states, pool.baseAddress!)
                }
            }
        }
    }

    // The kernels below follow contact_solver_wide.inl without lane masks: prepare zeroes padded
    // points and absent lanes, so their updates are exact no-ops as in C.

    func warmStartContactsWide(_ color: Int32, _ mesh: Bool) {
        withWideContacts(color, mesh) { list, range, states, pool in
            warmStartContactsWideRange(list, range, states, pool)
        }
    }

    func warmStartContactsWideRange(
        _ list: UnsafePointer<ContactConstraintW>, _ range: Range<Int>,
        _ states: UnsafeMutableBufferPointer<BodyState>, _ pool: UnsafeMutablePointer<ManifoldConstraintW>
    ) {
        for i in range {
            let c = list + i
            let anyRolling = c.pointee.anyRolling
            var linear = Vec3W()
            var angularA = Vec3W()
            var angularB = Vec3W()
            for offset in 0..<c.pointee.manifoldCount {
                let m = pool + c.pointee.manifoldStart + offset
                var total = FloatW.zero
                var momentA = Vec3W()
                var momentB = Vec3W()
                for pi in 0..<m.pointee.pointCount {
                    let p = m.point(pi)
                    total += p.pointee.normalImpulse
                    momentA = mulAddW(momentA, p.pointee.normalImpulse, p.pointee.rA)
                    momentB = mulAddW(momentB, p.pointee.normalImpulse, p.pointee.rB)
                }
                let normal = m.pointee.normal
                var friction = mulW(m.pointee.frictionImpulse.x, m.pointee.tangent1)
                friction = mulAddW(friction, m.pointee.frictionImpulse.y, m.pointee.tangent2)
                let manifoldLinear = mulAddW(friction, total, normal)
                let twist = mulW(m.pointee.twistImpulse, normal)
                var a = addW(addW(crossW(momentA, normal), crossW(m.pointee.centerA, friction)), twist)
                var b = addW(addW(crossW(momentB, normal), crossW(m.pointee.centerB, friction)), twist)
                if anyRolling {
                    a = addW(a, m.pointee.rollingImpulse)
                    b = addW(b, m.pointee.rollingImpulse)
                }
                linear = addW(linear, manifoldLinear)
                angularA = addW(angularA, a)
                angularB = addW(angularB, b)
            }
            var a = Self.gatherStatesW(states, c.pointee.indexA)
            var b = Self.gatherStatesW(states, c.pointee.indexB)
            a.w = subW(a.w, mulMVW(c.pointee.invIA, angularA))
            a.v = subW(a.v, mulW(c.pointee.invMassA, linear))
            b.w = addW(b.w, mulMVW(c.pointee.invIB, angularB))
            b.v = addW(b.v, mulW(c.pointee.invMassB, linear))
            Self.scatterStatesW(states, c.pointee.indexA, a)
            Self.scatterStatesW(states, c.pointee.indexB, b)
        }
    }

    func pushContactsWide(_ color: Int32, _ context: StepContext, _ mesh: Bool) {
        withWideContacts(color, mesh) { list, range, states, pool in
            pushContactsWideRange(list, range, context, states, pool)
        }
    }

    func pushContactsWideRange(
        _ list: UnsafePointer<ContactConstraintW>, _ range: Range<Int>, _ context: StepContext, 
        _ states: UnsafeMutableBufferPointer<BodyState>, _ pool: UnsafeMutablePointer<ManifoldConstraintW>
    ) {
        let invH = FloatW(repeating: context.inv_h)
        let minSpeed = FloatW(repeating: -contactSpeed)
        for i in range {
            let c = list + i
            let indexA = c.pointee.indexA
            let indexB = c.pointee.indexB
            var a = Self.gatherStatesW(states, indexA)
            var b = Self.gatherStatesW(states, indexB)
            let biasRate = c.pointee.biasRate
            let massScale = c.pointee.massScale
            let impulseScale = c.pointee.impulseScale
            let dp = subW(b.dp, a.dp)
            let invMass = c.pointee.invMassA + c.pointee.invMassB
            for offset in 0..<c.pointee.manifoldCount {
                let m = pool + c.pointee.manifoldStart + offset
                let separation = dotW(m.pointee.normal, dp)
                let normalA = invRotateVecW(a.dq, m.pointee.normal)
                let normalB = invRotateVecW(b.dq, m.pointee.normal)
                var vnLinear = dotW(m.pointee.normal, subW(b.v, a.v))
                var total = FloatW.zero
                for pi in 0..<m.pointee.pointCount {
                    let p = m.point(pi)
                    let rA = p.pointee.rA
                    let rB = p.pointee.rB
                    let s = (separation + (dotW(normalB, rB) - dotW(normalA, rA))) + p.pointee.baseSeparation
                    let separated = s .> .zero
                    let bias = selectW(separated, s * invH, maxW(biasRate * s, minSpeed))
                    let pointMassScale = selectW(separated, FloatW(repeating: 1), massScale)
                    let pointImpulseScale = selectW(separated, .zero, impulseScale)
                    let uA = crossW(rA, m.pointee.normal)
                    let uB = crossW(rB, m.pointee.normal)
                    let gA = mulMVW(c.pointee.invIA, uA)
                    let gB = mulMVW(c.pointee.invIB, uB)
                    let vn = vnLinear + (dotW(uB, b.w) - dotW(uA, a.w))
                    let neg = p.pointee.normalMass * (pointMassScale * vn + bias) + pointImpulseScale * p.pointee.normalImpulse
                    let impulse = maxW(negW(neg), negW(p.pointee.normalImpulse))
                    p.pointee.normalImpulse += impulse
                    vnLinear += invMass * impulse
                    total += impulse
                    a.w = subW(a.w, mulW(impulse, gA))
                    b.w = addW(b.w, mulW(impulse, gB))
                }
                let impulse = mulW(total, m.pointee.normal)
                a.v = subW(a.v, mulW(c.pointee.invMassA, impulse))
                b.v = addW(b.v, mulW(c.pointee.invMassB, impulse))
            }
            Self.scatterStatesW(states, indexA, a)
            Self.scatterStatesW(states, indexB, b)
        }
    }

    func solveContactsWide(_ color: Int32, _ context: StepContext, _ mesh: Bool) {
        withWideContacts(color, mesh) { list, range, states, pool in
            solveContactsWideRange(list, range, context, states, pool)
        }
    }

    func solveContactsWideRange(
        _ list: UnsafePointer<ContactConstraintW>, _ range: Range<Int>, _ context: StepContext, 
        _ states: UnsafeMutableBufferPointer<BodyState>, _ pool: UnsafeMutablePointer<ManifoldConstraintW>
    ) {
        let invH = FloatW(repeating: context.inv_h)
        let epsilon = FloatW(repeating: Float.ulpOfOne)
        for i in range {
            let c = list + i
            let indexA = c.pointee.indexA
            let indexB = c.pointee.indexB
            var a = Self.gatherStatesW(states, indexA)
            var b = Self.gatherStatesW(states, indexB)
            let anyRolling = c.pointee.anyRolling
            let dp = subW(b.dp, a.dp)
            let invMass = c.pointee.invMassA + c.pointee.invMassB
            for offset in 0..<c.pointee.manifoldCount {
                let m = pool + c.pointee.manifoldStart + offset
                let separation = dotW(m.pointee.normal, dp)
                let normalA = invRotateVecW(a.dq, m.pointee.normal)
                let normalB = invRotateVecW(b.dq, m.pointee.normal)
                let dv = subW(b.v, a.v)
                var vnLinear = dotW(m.pointee.normal, dv)
                let vt1 = dotW(m.pointee.tangent1, dv) - m.pointee.tangentVelocity1
                let vt2 = dotW(m.pointee.tangent2, dv) - m.pointee.tangentVelocity2
                var total = FloatW.zero
                var totalNormal = FloatW.zero
                var twistLimit = FloatW.zero
                for pi in 0..<m.pointee.pointCount {
                    let p = m.point(pi)
                    let rA = p.pointee.rA
                    let rB = p.pointee.rB
                    let s = (separation + (dotW(normalB, rB) - dotW(normalA, rA))) + p.pointee.baseSeparation
                    let bias = maxW(.zero, s * invH)
                    let uA = crossW(rA, m.pointee.normal)
                    let uB = crossW(rB, m.pointee.normal)
                    let gA = mulMVW(c.pointee.invIA, uA)
                    let gB = mulMVW(c.pointee.invIB, uB)
                    let vn = vnLinear + (dotW(uB, b.w) - dotW(uA, a.w))
                    let neg = p.pointee.normalMass * (vn + bias)
                    let impulse = maxW(negW(neg), negW(p.pointee.normalImpulse))
                    let newImpulse = p.pointee.normalImpulse + impulse
                    p.pointee.normalImpulse = newImpulse
                    p.pointee.totalNormalImpulse += newImpulse
                    totalNormal += newImpulse
                    twistLimit += p.pointee.leverArm * newImpulse
                    vnLinear += invMass * impulse
                    total += impulse
                    a.w = subW(a.w, mulW(impulse, gA))
                    b.w = addW(b.w, mulW(impulse, gB))
                }
                if anyRolling {
                    // Flip A/B order to negate
                    let old = m.pointee.rollingImpulse
                    let new = addW(old, mulMVW(c.pointee.rollingMass, subW(a.w, b.w)))
                    let limit = c.pointee.rollingResistance * totalNormal
                    let lengthSquared = dotW(new, new)
                    let scale = selectW(
                        lengthSquared .> (epsilon + limit * limit),
                        limit / (lengthSquared.squareRoot() + epsilon), FloatW(repeating: 1))
                    let rollingMask = (c.pointee.rollingResistance .> .zero) .& (m.pointee.exists .> .zero)
                    let clamped = selectW(rollingMask, mulW(scale, new), Vec3W())
                    m.pointee.rollingImpulse = clamped
                    let delta = subW(clamped, old)
                    a.w = subW(a.w, mulMVW(c.pointee.invIA, delta))
                    b.w = addW(b.w, mulMVW(c.pointee.invIB, delta))
                }
                // Central twist friction
                do {
                    let normal = m.pointee.normal
                    let twistSpeed = dotW(normal, subW(b.w, a.w))
                    let maxLambda = c.pointee.friction * twistLimit
                    let oldTwist = m.pointee.twistImpulse
                    var twist = negW(m.pointee.twistMass * twistSpeed)
                    twist = maxW(negW(maxLambda) - oldTwist, minW(twist, maxLambda - oldTwist))
                    m.pointee.twistImpulse = oldTwist + twist
                    a.w = subW(a.w, mulW(twist, mulMVW(c.pointee.invIA, normal)))
                    b.w = addW(b.w, mulW(twist, mulMVW(c.pointee.invIB, normal)))
                }
                // Central friction
                let t1 = m.pointee.tangent1
                let t2 = m.pointee.tangent2
                let rA = m.pointee.centerA
                let rB = m.pointee.centerB
                let angular = subW(crossW(b.w, rB), crossW(a.w, rA))
                let tangentVelocity = Vec2W(x: vt1 + dotW(angular, t1), y: vt2 + dotW(angular, t2))
                var delta = mulMV2W(m.pointee.tangentMass, tangentVelocity)
                delta = Vec2W(x: negW(delta.x), y: negW(delta.y))
                let oldFriction = m.pointee.frictionImpulse
                var newFriction = Vec2W(x: oldFriction.x + delta.x, y: oldFriction.y + delta.y)
                let limit = c.pointee.friction * totalNormal
                let lengthSquared = newFriction.x * newFriction.x + newFriction.y * newFriction.y
                let scale = selectW(
                    lengthSquared .> (limit * limit), limit / (lengthSquared.squareRoot() + epsilon),
                    FloatW(repeating: 1))
                newFriction = Vec2W(x: scale * newFriction.x, y: scale * newFriction.y)
                delta = Vec2W(x: newFriction.x - oldFriction.x, y: newFriction.y - oldFriction.y)
                m.pointee.frictionImpulse = newFriction
                var impulse = addW(mulW(delta.x, t1), mulW(delta.y, t2))
                a.w = subW(a.w, mulMVW(c.pointee.invIA, crossW(rA, impulse)))
                b.w = addW(b.w, mulMVW(c.pointee.invIB, crossW(rB, impulse)))
                impulse = addW(impulse, mulW(total, m.pointee.normal))
                a.v = subW(a.v, mulW(c.pointee.invMassA, impulse))
                b.v = addW(b.v, mulW(c.pointee.invMassB, impulse))
            }
            Self.scatterStatesW(states, indexA, a)
            Self.scatterStatesW(states, indexB, b)
        }
    }

    func applyRestitutionWide(_ color: Int32, _ context: StepContext, _ mesh: Bool) {
        withWideContacts(color, mesh) { list, range, states, pool in
            applyRestitutionWideRange(list, range, context, states, pool)
        }
    }

    func applyRestitutionWideRange(
        _ list: UnsafePointer<ContactConstraintW>, _ range: Range<Int>, _ context: StepContext, 
        _ states: UnsafeMutableBufferPointer<BodyState>, _ pool: UnsafeMutablePointer<ManifoldConstraintW>
    ) {
        let invH = FloatW(repeating: context.inv_h)
        let threshold = FloatW(repeating: -restitutionThreshold)
        let propagate = enableRestitutionPropagation
        for i in range {
            let c = list + i
            if !propagate && !c.pointee.anyRestitution { continue }
            let indexA = c.pointee.indexA
            let indexB = c.pointee.indexB
            let invIA = c.pointee.invIA
            let invIB = c.pointee.invIB
            let invMassA = c.pointee.invMassA
            let invMassB = c.pointee.invMassB
            var a = Self.gatherStatesW(states, indexA)
            var b = Self.gatherStatesW(states, indexB)
            let originalA = a
            let originalB = b
            let restitutionMask = c.pointee.restitution .> .zero
            let dp = subW(b.dp, a.dp)
            for offset in 0..<c.pointee.manifoldCount {
                let m = pool + c.pointee.manifoldStart + offset
                let normal = m.pointee.normal
                let separation = dotW(normal, dp)
                let normalA = invRotateVecW(a.dq, normal)
                let normalB = invRotateVecW(b.dq, normal)
                for pi in 0..<m.pointee.pointCount {
                    withUnsafeMutablePointer(to: &m.pointee.points[pi]) { p in
                        var active = m.pointee.pointCounts .> SIMD4<Int32>(repeating: Int32(pi))
                        if !propagate { active = active .& restitutionMask }
                        let s =
                            (separation + (dotW(normalB, p.pointee.rB) - dotW(normalA, p.pointee.rA)))
                            + p.pointee.baseSeparation
                        let mass =
                            propagate
                            ? p.pointee.normalMass : selectW(restitutionMask, p.pointee.normalMass, .zero)
                        let compression = p.pointee.totalNormalImpulse - p.pointee.restitutionImpulse
                        let armed =
                            restitutionMask .& (p.pointee.relativeVelocity .< threshold) .& (compression .> .zero)
                        let bias = selectW(
                            armed, c.pointee.restitution * p.pointee.relativeVelocity, maxW(.zero, s * invH))
                        let vrA = addW(a.v, crossW(a.w, p.pointee.rA))
                        let vrB = addW(b.v, crossW(b.w, p.pointee.rB))
                        let vn = dotW(subW(vrB, vrA), normal)
                        let neg = mass * (vn + bias)
                        var impulse = maxW(p.pointee.normalImpulse - neg, .zero) - p.pointee.normalImpulse
                        let approach = minW(maxW(negW(mass) * vn, .zero), maxW(impulse, .zero))
                        let allowance = c.pointee.restitution * (compression + approach) - p.pointee.restitutionImpulse
                        impulse = selectW(
                            active, selectW(armed, minW(impulse, approach + maxW(allowance, .zero)), impulse),
                            .zero)
                        p.pointee.normalImpulse += impulse
                        p.pointee.restitutionImpulse += impulse - approach
                        p.pointee.totalNormalImpulse += impulse
                        let linear = mulW(impulse, normal)
                        a.w = selectW(active, subW(a.w, mulMVW(invIA, crossW(p.pointee.rA, linear))), a.w)
                        a.v = selectW(active, subW(a.v, mulW(invMassA, linear)), a.v)
                        b.w = selectW(active, addW(b.w, mulMVW(invIB, crossW(p.pointee.rB, linear))), b.w)
                        b.v = selectW(active, addW(b.v, mulW(invMassB, linear)), b.v)
                    }
                }
            }
            if !propagate {
                a.v = selectW(restitutionMask, a.v, originalA.v)
                a.w = selectW(restitutionMask, a.w, originalA.w)
                b.v = selectW(restitutionMask, b.v, originalB.v)
                b.w = selectW(restitutionMask, b.w, originalB.w)
            }
            Self.scatterStatesW(states, indexA, a)
            Self.scatterStatesW(states, indexB, b)
        }
    }

    /// Persist directly from SoA lanes without first copying back into scalar constraints.
    func storeImpulsesWide(_ color: Int32, _ mesh: Bool) {
        let ci = Int(color)
        let split = solverContacts.wideConvexCounts[ci]
        solverContacts.wideConstraints[ci].withUnsafeBufferPointer { wide in
            let range = mesh ? split..<wide.count : 0..<split
            guard range.isEmpty == false else { return }
            solverContacts.wideManifolds.withUnsafeBufferPointer { pool in
                contacts.withUnsafeMutableBufferPointer { store in
                    storeImpulsesWideRange(wide.baseAddress!, range, pool.baseAddress!, store.baseAddress!)
                }
            }
        }
    }

    func storeImpulsesWideRange(
        _ wideBase: UnsafePointer<ContactConstraintW>, _ range: Range<Int>,
        _ poolBase: UnsafePointer<ManifoldConstraintW>, _ storeBase: UnsafeMutablePointer<Contact>
    ) {
        for i in range {
            let c = wideBase + i
            for lane in 0..<4 {
                let id = Int(c.pointee.contactIds[lane])
                guard id >= 0 else { continue }
                let laneCount = Int(c.pointee.laneManifoldCounts[lane])
                guard laneCount > 0 else { continue }
                let hitEvents = (storeBase[id].flags & ContactFlags.simEnableHitEvent.rawValue) != 0
                let manifoldBase = UnsafeMutablePointer(mutating: storeBase[id].manifolds.buffer.baseAddress!)
                for offset in 0..<laneCount {
                    let m = poolBase + c.pointee.manifoldStart + offset
                    let out = manifoldBase + offset
                    for pi in 0..<Int(out.pointee.pointCount) {
                        out.pointee.points[pi].normalImpulse = m.pointee.points[pi].normalImpulse[lane]
                        out.pointee.points[pi].totalNormalImpulse = m.pointee.points[pi].totalNormalImpulse[lane]
                        if hitEvents {
                            out.pointee.points[pi].normalVelocity = m.pointee.points[pi].relativeVelocity[lane]
                        }
                    }
                    out.pointee.twistImpulse = m.pointee.twistImpulse[lane]
                    out.pointee.frictionImpulse =
                        m.pointee.frictionImpulse.x[lane] * m.pointee.tangent1[lane]
                        + m.pointee.frictionImpulse.y[lane] * m.pointee.tangent2[lane]
                    out.pointee.rollingImpulse = m.pointee.rollingImpulse[lane]
                }
            }
        }
    }

    /// Scatter accumulated impulses into the scalar constraints. Test support: requires
    /// `prepareScalarForWideColors` so every wide lane has a scalar twin.
    func unpackWideImpulses() {
        guard enableWideSolver else { return }
        for ci in 0..<Int(overflowIndex) {
            let list = solverContacts.constraints[ci]
            let wide = solverContacts.wideConstraints[ci]
            let manifolds = solverContacts.wideManifolds
            solverContacts.manifoldPool.withUnsafeMutableBufferPointer { pool in
                for c in wide {
                    for lane in 0..<4 {
                        let id = c.contactIds[lane]
                        guard id >= 0, let index = list.firstIndex(where: { $0.contactId == id }) else { continue }
                        let s = list[index]
                        for offset in 0..<Int(s.manifoldCount) {
                            let m = manifolds[c.manifoldStart + offset]
                            let out = pool.baseAddress! + Int(s.manifoldStart) + offset
                            out.pointee.twistImpulse = m.twistImpulse[lane]
                            out.pointee.frictionImpulse = m.frictionImpulse[lane]
                            out.pointee.rollingImpulse = m.rollingImpulse[lane]
                            for pi in 0..<Int(out.pointee.pointCount) {
                                out.pointee.points[pi].normalImpulse = m.points[pi].normalImpulse[lane]
                                out.pointee.points[pi].totalNormalImpulse = m.points[pi].totalNormalImpulse[lane]
                            }
                        }
                    }
                }
            }
        }
    }

    /// Load four body states as rows and transpose them into lanes (b3GatherBodies). A null index
    /// reads the identity state: zero velocity and delta position, identity delta rotation.
    @inline(__always)
    static func gatherStatesW(
        _ states: UnsafeMutableBufferPointer<BodyState>, _ indices: SIMD4<Int32>
    ) -> BodyStateW {
        let base = UnsafeRawPointer(states.baseAddress!)
        @inline(__always) func rows(_ index: Int32) -> (FloatW, FloatW, FloatW, FloatW) {
            if index < 0 { return (.zero, .zero, .zero, FloatW(0, 0, 0, 1)) }
            let p = base + Int(index) &* 64
            return (
                p.loadUnaligned(as: FloatW.self), p.loadUnaligned(fromByteOffset: 16, as: FloatW.self),
                p.loadUnaligned(fromByteOffset: 32, as: FloatW.self), p.loadUnaligned(fromByteOffset: 48, as: FloatW.self)
            )
        }
        let a = rows(indices[0])
        let b = rows(indices[1])
        let c = rows(indices[2])
        let d = rows(indices[3])
        let v = transpose4(a.0, b.0, c.0, d.0)
        let w = transpose4(a.1, b.1, c.1, d.1)
        let dp = transpose4(a.2, b.2, c.2, d.2)
        let dq = transpose4(a.3, b.3, c.3, d.3)
        return BodyStateW(
            v: Vec3W(v.0, v.1, v.2),
            w: Vec3W(w.0, w.1, w.2),
            dp: Vec3W(dp.0, dp.1, dp.2),
            dq: QuatW(v: Vec3W(dq.0, dq.1, dq.2), s: dq.3))
    }

    /// Write only the velocities back, as two 16-byte rows per dynamic body (b3ScatterBodies).
    @inline(__always)
    static func scatterStatesW(
        _ states: UnsafeMutableBufferPointer<BodyState>, _ indices: SIMD4<Int32>, _ value: BodyStateW
    ) {
        let base = UnsafeMutableRawPointer(states.baseAddress!)
        let dynamicFlag = BodyFlags.dynamicFlag.rawValue
        let v = transpose4(value.v.x, value.v.y, value.v.z, .zero)
        let w = transpose4(value.w.x, value.w.y, value.w.z, .zero)
        @inline(__always) func store(_ index: Int32, _ v: FloatW, _ w: FloatW) {
            guard index >= 0 else { return }
            let p = base + Int(index) &* 64
            guard (p.load(fromByteOffset: 44, as: UInt32.self) & dynamicFlag) != 0 else { return }
            p.storeBytes(of: v, as: FloatW.self)
            p.storeBytes(of: w, toByteOffset: 16, as: FloatW.self)
        }
        store(indices[0], v.0, w.0)
        store(indices[1], v.1, w.1)
        store(indices[2], v.2, w.2)
        store(indices[3], v.3, w.3)
    }
}

extension UnsafeMutablePointer where Pointee == ManifoldConstraintW {
    /// One constraint point in place; going through `points[i]` would copy or outline the access.
    @inline(__always)
    func point(_ index: Int) -> UnsafeMutablePointer<ConstraintPointW> {
        (UnsafeMutableRawPointer(self) + MemoryLayout<ManifoldConstraintW>.offset(of: \.points)!)
            .assumingMemoryBound(to: ConstraintPointW.self) + index
    }
}
