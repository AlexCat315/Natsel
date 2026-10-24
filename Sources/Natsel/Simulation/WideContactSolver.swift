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
    var scalarIndices = SIMD4<Int32>(repeating: -1)
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
        enableWideSolver && color != overflowIndex && solverContacts.range(color, mesh).count >= 4
    }

    /// Pack each independent color once, rather than gathering constraints on every iteration.
    func prepareWideContacts() {
        solverContacts.wideManifolds.removeAll(keepingCapacity: true)
        for ci in 0..<Int(B3.graphColorCount) {
            solverContacts.wideConstraints[ci].removeAll(keepingCapacity: true)
            solverContacts.wideConvexCounts[ci] = 0
            for mesh in [false, true] {
                guard usesWideContacts(Int32(ci), mesh) else { continue }
                let list = solverContacts.constraints[ci]
                let range = solverContacts.range(Int32(ci), mesh)
                for start in stride(from: range.lowerBound, to: range.upperBound, by: 4) {
                    var c = ContactConstraintW()
                    for lane in 0..<min(4, range.upperBound - start) {
                        let s = list[start + lane]
                        c.scalarIndices[lane] = Int32(start + lane)
                        c.indexA[lane] = s.indexA
                        c.indexB[lane] = s.indexB
                        c.invMassA[lane] = s.invMassA
                        c.invMassB[lane] = s.invMassB
                        c.invIA.setLane(lane, s.invIA)
                        c.invIB.setLane(lane, s.invIB)
                        c.rollingMass.setLane(lane, s.rollingMass)
                        c.biasRate[lane] = s.softness.massScale * s.softness.biasRate
                        c.massScale[lane] = s.softness.massScale
                        c.impulseScale[lane] = s.softness.impulseScale
                        c.friction[lane] = s.friction
                        c.restitution[lane] = s.restitution
                        c.rollingResistance[lane] = s.rollingResistance
                        c.anyRolling = c.anyRolling || s.rollingResistance > 0
                        c.anyRestitution = c.anyRestitution || s.restitution > 0
                        c.manifoldCount = max(c.manifoldCount, Int(s.manifoldCount))
                    }
                    c.manifoldStart = solverContacts.wideManifolds.count
                    for offset in 0..<c.manifoldCount {
                        var m = ManifoldConstraintW()
                        for lane in 0..<4 {
                            let index = c.scalarIndices[lane]
                            guard index >= 0 else { continue }
                            let s = list[Int(index)]
                            guard offset < s.manifoldCount else { continue }
                            let sm = solverContacts.manifoldPool[Int(s.manifoldStart) + offset]
                            m.exists[lane] = 1
                            m.pointCounts[lane] = sm.pointCount
                            m.pointCount = max(m.pointCount, Int(sm.pointCount))
                            m.normal[lane] = sm.normal
                            m.tangent1[lane] = sm.tangent1
                            m.tangent2[lane] = sm.tangent2
                            m.centerA[lane] = sm.centerA
                            m.centerB[lane] = sm.centerB
                            m.twistMass[lane] = sm.twistMass
                            m.twistImpulse[lane] = sm.twistImpulse
                            m.tangentMass.xx[lane] = sm.tangentMass.cx.x
                            m.tangentMass.yy[lane] = sm.tangentMass.cy.y
                            m.tangentMass.xy[lane] = sm.tangentMass.cx.y
                            m.frictionImpulse[lane] = sm.frictionImpulse
                            m.rollingImpulse[lane] = sm.rollingImpulse
                            m.tangentVelocity1[lane] = sm.tangentVelocity1
                            m.tangentVelocity2[lane] = sm.tangentVelocity2
                            for pi in 0..<Int(sm.pointCount) {
                                let sp = sm.points[pi]
                                m.points[pi].rA[lane] = sp.rA
                                m.points[pi].rB[lane] = sp.rB
                                m.points[pi].baseSeparation[lane] = sp.baseSeparation
                                m.points[pi].relativeVelocity[lane] = sp.relativeVelocity
                                m.points[pi].normalImpulse[lane] = sp.normalImpulse
                                m.points[pi].normalMass[lane] = sp.normalMass
                                m.points[pi].leverArm[lane] = sp.leverArm
                            }
                        }
                        solverContacts.wideManifolds.append(m)
                    }
                    solverContacts.wideConstraints[ci].append(c)
                }
                if !mesh { solverContacts.wideConvexCounts[ci] = solverContacts.wideConstraints[ci].count }
            }
        }
    }

    @inline(__always)
    private func withWideContacts(
        _ color: Int32, _ mesh: Bool,
        _ body: (
            ContactConstraintW, UnsafeMutableBufferPointer<BodyState>,
            UnsafeMutablePointer<ManifoldConstraintW>
        ) -> Void
    ) {
        let ci = Int(color)
        let list = solverContacts.wideConstraints[ci]
        let split = solverContacts.wideConvexCounts[ci]
        let range = mesh ? split..<list.count : 0..<split
        withSolverStates { states in
            solverContacts.wideManifolds.withUnsafeMutableBufferPointer { pool in
                for i in range { body(list[i], states, pool.baseAddress!) }
            }
        }
    }

    func warmStartContactsWide(_ color: Int32, _ mesh: Bool) {
        withWideContacts(color, mesh) { c, states, pool in
            var linear = Vec3W()
            var angularA = Vec3W()
            var angularB = Vec3W()
            for offset in 0..<c.manifoldCount {
                let m = pool + c.manifoldStart + offset
                var total = FloatW.zero
                var momentA = Vec3W()
                var momentB = Vec3W()
                for pi in 0..<m.pointee.pointCount {
                    let p = m.pointee.points[pi]
                    total += p.normalImpulse
                    momentA = mulAddW(momentA, p.normalImpulse, p.rA)
                    momentB = mulAddW(momentB, p.normalImpulse, p.rB)
                }
                let normal = m.pointee.normal
                var friction = mulW(m.pointee.frictionImpulse.x, m.pointee.tangent1)
                friction = mulAddW(friction, m.pointee.frictionImpulse.y, m.pointee.tangent2)
                let twist = mulW(m.pointee.twistImpulse, normal)
                var a = addW(addW(crossW(momentA, normal), crossW(m.pointee.centerA, friction)), twist)
                var b = addW(addW(crossW(momentB, normal), crossW(m.pointee.centerB, friction)), twist)
                if c.anyRolling {
                    let rolling = selectW(c.rollingResistance .> .zero, m.pointee.rollingImpulse, Vec3W())
                    a = addW(a, rolling)
                    b = addW(b, rolling)
                }
                // A shorter mesh contact has no manifold in this lane. Its rolling mass
                // still belongs to the contact, so mask the entire manifold contribution.
                let active = m.pointee.exists .> .zero
                linear = addW(linear, selectW(active, mulAddW(friction, total, normal), Vec3W()))
                angularA = addW(angularA, selectW(active, a, Vec3W()))
                angularB = addW(angularB, selectW(active, b, Vec3W()))
            }
            var a = Self.gatherStatesW(states, c.indexA)
            var b = Self.gatherStatesW(states, c.indexB)
            a.w = subW(a.w, mulMVW(c.invIA, angularA))
            a.v = subW(a.v, mulW(c.invMassA, linear))
            b.w = addW(b.w, mulMVW(c.invIB, angularB))
            b.v = addW(b.v, mulW(c.invMassB, linear))
            Self.scatterStatesW(states, c.indexA, a)
            Self.scatterStatesW(states, c.indexB, b)
        }
    }

    func pushContactsWide(_ color: Int32, _ context: StepContext, _ mesh: Bool) {
        let invH = FloatW(repeating: context.inv_h)
        let minSpeed = FloatW(repeating: -contactSpeed)
        withWideContacts(color, mesh) { c, states, pool in
            var a = Self.gatherStatesW(states, c.indexA)
            var b = Self.gatherStatesW(states, c.indexB)
            let dp = subW(b.dp, a.dp)
            let invMass = c.invMassA + c.invMassB
            for offset in 0..<c.manifoldCount {
                let m = pool + c.manifoldStart + offset
                let normal = m.pointee.normal
                let separation = dotW(normal, dp)
                let normalA = invRotateVecW(a.dq, normal)
                let normalB = invRotateVecW(b.dq, normal)
                var vnLinear = dotW(normal, subW(b.v, a.v))
                var total = FloatW.zero
                for pi in 0..<m.pointee.pointCount {
                    withUnsafeMutablePointer(to: &m.pointee.points[pi]) { p in
                        let active = m.pointee.pointCounts .> SIMD4<Int32>(repeating: Int32(pi))
                        let s =
                            (separation + (dotW(normalB, p.pointee.rB) - dotW(normalA, p.pointee.rA)))
                            + p.pointee.baseSeparation
                        let separated = s .> .zero
                        let bias = selectW(separated, s * invH, maxW(c.biasRate * s, minSpeed))
                        let mass = selectW(separated, FloatW(repeating: 1), c.massScale)
                        let impulseScale = selectW(separated, .zero, c.impulseScale)
                        let uA = crossW(p.pointee.rA, normal)
                        let uB = crossW(p.pointee.rB, normal)
                        let vn = vnLinear + (dotW(uB, b.w) - dotW(uA, a.w))
                        let neg =
                            p.pointee.normalMass * (mass * vn + bias) + impulseScale * p.pointee.normalImpulse
                        let impulse = selectW(active, maxW(negW(neg), negW(p.pointee.normalImpulse)), .zero)
                        p.pointee.normalImpulse += impulse
                        vnLinear += invMass * impulse
                        total += impulse
                        a.w = selectW(active, subW(a.w, mulW(impulse, mulMVW(c.invIA, uA))), a.w)
                        b.w = selectW(active, addW(b.w, mulW(impulse, mulMVW(c.invIB, uB))), b.w)
                    }
                }
                let active = m.pointee.exists .> .zero
                let impulse = mulW(total, normal)
                a.v = selectW(active, subW(a.v, mulW(c.invMassA, impulse)), a.v)
                b.v = selectW(active, addW(b.v, mulW(c.invMassB, impulse)), b.v)
            }
            Self.scatterStatesW(states, c.indexA, a)
            Self.scatterStatesW(states, c.indexB, b)
        }
    }

    func solveContactsWide(_ color: Int32, _ context: StepContext, _ mesh: Bool) {
        let invH = FloatW(repeating: context.inv_h)
        let epsilon = FloatW(repeating: Float.ulpOfOne)
        withWideContacts(color, mesh) { c, states, pool in
            var a = Self.gatherStatesW(states, c.indexA)
            var b = Self.gatherStatesW(states, c.indexB)
            let dp = subW(b.dp, a.dp)
            let invMass = c.invMassA + c.invMassB
            for offset in 0..<c.manifoldCount {
                let m = pool + c.manifoldStart + offset
                let oldA = a
                let oldB = b
                let normal = m.pointee.normal
                let t1 = m.pointee.tangent1
                let t2 = m.pointee.tangent2
                let separation = dotW(normal, dp)
                let normalA = invRotateVecW(a.dq, normal)
                let normalB = invRotateVecW(b.dq, normal)
                let dv = subW(b.v, a.v)
                var vnLinear = dotW(normal, dv)
                let vt1 = dotW(t1, dv) - m.pointee.tangentVelocity1
                let vt2 = dotW(t2, dv) - m.pointee.tangentVelocity2
                var total = FloatW.zero
                var totalNormal = FloatW.zero
                var twistLimit = FloatW.zero
                for pi in 0..<m.pointee.pointCount {
                    withUnsafeMutablePointer(to: &m.pointee.points[pi]) { p in
                        let active = m.pointee.pointCounts .> SIMD4<Int32>(repeating: Int32(pi))
                        let s =
                            (separation + (dotW(normalB, p.pointee.rB) - dotW(normalA, p.pointee.rA)))
                            + p.pointee.baseSeparation
                        let bias = maxW(.zero, s * invH)
                        let uA = crossW(p.pointee.rA, normal)
                        let uB = crossW(p.pointee.rB, normal)
                        let vn = vnLinear + (dotW(uB, b.w) - dotW(uA, a.w))
                        let neg = p.pointee.normalMass * (vn + bias)
                        let impulse = selectW(active, maxW(negW(neg), negW(p.pointee.normalImpulse)), .zero)
                        let newImpulse = p.pointee.normalImpulse + impulse
                        p.pointee.normalImpulse = newImpulse
                        p.pointee.totalNormalImpulse += newImpulse
                        totalNormal += newImpulse
                        twistLimit += p.pointee.leverArm * newImpulse
                        vnLinear += invMass * impulse
                        total += impulse
                        a.w = selectW(active, subW(a.w, mulW(impulse, mulMVW(c.invIA, uA))), a.w)
                        b.w = selectW(active, addW(b.w, mulW(impulse, mulMVW(c.invIB, uB))), b.w)
                    }
                }
                if c.anyRolling {
                    var delta = mulMVW(c.rollingMass, subW(a.w, b.w))
                    let old = m.pointee.rollingImpulse
                    var new = addW(old, delta)
                    let limit = c.rollingResistance * totalNormal
                    let lengthSquared = dotW(new, new)
                    let scale = selectW(
                        lengthSquared .> (epsilon + limit * limit),
                        limit / (lengthSquared.squareRoot() + epsilon), FloatW(repeating: 1))
                    new = selectW(c.rollingResistance .> .zero, mulW(scale, new), Vec3W())
                    m.pointee.rollingImpulse = new
                    delta = subW(new, old)
                    let rolling = c.rollingResistance .> .zero
                    a.w = selectW(rolling, subW(a.w, mulMVW(c.invIA, delta)), a.w)
                    b.w = selectW(rolling, addW(b.w, mulMVW(c.invIB, delta)), b.w)
                }
                let twistSpeed = dotW(normal, subW(b.w, a.w))
                let maxLambda = c.friction * twistLimit
                let oldTwist = m.pointee.twistImpulse
                var twist = negW(m.pointee.twistMass * twistSpeed)
                twist = maxW(negW(maxLambda) - oldTwist, minW(twist, maxLambda - oldTwist))
                m.pointee.twistImpulse = oldTwist + twist
                a.w = subW(a.w, mulW(twist, mulMVW(c.invIA, normal)))
                b.w = addW(b.w, mulW(twist, mulMVW(c.invIB, normal)))
                let rA = m.pointee.centerA
                let rB = m.pointee.centerB
                let angular = subW(crossW(b.w, rB), crossW(a.w, rA))
                let tangentVelocity = Vec2W(x: vt1 + dotW(angular, t1), y: vt2 + dotW(angular, t2))
                var delta = mulMV2W(m.pointee.tangentMass, tangentVelocity)
                delta = Vec2W(x: negW(delta.x), y: negW(delta.y))
                let oldFriction = m.pointee.frictionImpulse
                var newFriction = Vec2W(x: oldFriction.x + delta.x, y: oldFriction.y + delta.y)
                let limit = c.friction * totalNormal
                let lengthSquared = newFriction.x * newFriction.x + newFriction.y * newFriction.y
                let scale = selectW(
                    lengthSquared .> (limit * limit), limit / (lengthSquared.squareRoot() + epsilon),
                    FloatW(repeating: 1))
                newFriction = Vec2W(x: scale * newFriction.x, y: scale * newFriction.y)
                delta = Vec2W(x: newFriction.x - oldFriction.x, y: newFriction.y - oldFriction.y)
                m.pointee.frictionImpulse = newFriction
                var impulse = addW(mulW(delta.x, t1), mulW(delta.y, t2))
                a.w = subW(a.w, mulMVW(c.invIA, crossW(rA, impulse)))
                b.w = addW(b.w, mulMVW(c.invIB, crossW(rB, impulse)))
                impulse = addW(impulse, mulW(total, normal))
                let active = m.pointee.exists .> .zero
                a.v = selectW(active, subW(a.v, mulW(c.invMassA, impulse)), oldA.v)
                b.v = selectW(active, addW(b.v, mulW(c.invMassB, impulse)), oldB.v)
                a.w = selectW(active, a.w, oldA.w)
                b.w = selectW(active, b.w, oldB.w)
            }
            Self.scatterStatesW(states, c.indexA, a)
            Self.scatterStatesW(states, c.indexB, b)
        }
    }

    func applyRestitutionWide(_ color: Int32, _ context: StepContext, _ mesh: Bool) {
        let invH = FloatW(repeating: context.inv_h)
        let threshold = FloatW(repeating: -restitutionThreshold)
        let propagate = enableRestitutionPropagation
        withWideContacts(color, mesh) { c, states, pool in
            if !propagate && !c.anyRestitution { return }
            var a = Self.gatherStatesW(states, c.indexA)
            var b = Self.gatherStatesW(states, c.indexB)
            let originalA = a
            let originalB = b
            let restitutionMask = c.restitution .> .zero
            let dp = subW(b.dp, a.dp)
            for offset in 0..<c.manifoldCount {
                let m = pool + c.manifoldStart + offset
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
                            armed, c.restitution * p.pointee.relativeVelocity, maxW(.zero, s * invH))
                        let vrA = addW(a.v, crossW(a.w, p.pointee.rA))
                        let vrB = addW(b.v, crossW(b.w, p.pointee.rB))
                        let vn = dotW(subW(vrB, vrA), normal)
                        let neg = mass * (vn + bias)
                        var impulse = maxW(p.pointee.normalImpulse - neg, .zero) - p.pointee.normalImpulse
                        let approach = minW(maxW(negW(mass) * vn, .zero), maxW(impulse, .zero))
                        let allowance = c.restitution * (compression + approach) - p.pointee.restitutionImpulse
                        impulse = selectW(
                            active, selectW(armed, minW(impulse, approach + maxW(allowance, .zero)), impulse),
                            .zero)
                        p.pointee.normalImpulse += impulse
                        p.pointee.restitutionImpulse += impulse - approach
                        p.pointee.totalNormalImpulse += impulse
                        let linear = mulW(impulse, normal)
                        a.w = selectW(active, subW(a.w, mulMVW(c.invIA, crossW(p.pointee.rA, linear))), a.w)
                        a.v = selectW(active, subW(a.v, mulW(c.invMassA, linear)), a.v)
                        b.w = selectW(active, addW(b.w, mulMVW(c.invIB, crossW(p.pointee.rB, linear))), b.w)
                        b.v = selectW(active, addW(b.v, mulW(c.invMassB, linear)), b.v)
                    }
                }
            }
            if !propagate {
                a.v = selectW(restitutionMask, a.v, originalA.v)
                a.w = selectW(restitutionMask, a.w, originalA.w)
                b.v = selectW(restitutionMask, b.v, originalB.v)
                b.w = selectW(restitutionMask, b.w, originalB.w)
            }
            Self.scatterStatesW(states, c.indexA, a)
            Self.scatterStatesW(states, c.indexB, b)
        }
    }

    /// Persist directly from SoA lanes without first copying back into scalar constraints.
    func storeImpulsesWide(_ color: Int32, _ mesh: Bool) {
        let ci = Int(color)
        let list = solverContacts.constraints[ci]
        let wide = solverContacts.wideConstraints[ci]
        let split = solverContacts.wideConvexCounts[ci]
        let range = mesh ? split..<wide.count : 0..<split
        solverContacts.wideManifolds.withUnsafeBufferPointer { pool in
            contacts.withUnsafeMutableBufferPointer { store in
                for i in range {
                    let c = wide[i]
                    for lane in 0..<4 {
                        let index = c.scalarIndices[lane]
                        guard index >= 0 else { continue }
                        let s = list[Int(index)]
                        let id = Int(s.contactId)
                        let hitEvents = (store[id].flags & ContactFlags.simEnableHitEvent.rawValue) != 0
                        store[id].manifolds.withUnsafeMutableBufferPointer { manifolds in
                            for offset in 0..<Int(s.manifoldCount) {
                                let m = pool.baseAddress! + c.manifoldStart + offset
                                let out = manifolds.baseAddress! + offset
                                for pi in 0..<Int(out.pointee.pointCount) {
                                    out.pointee.points[pi].normalImpulse = m.pointee.points[pi].normalImpulse[lane]
                                    out.pointee.points[pi].totalNormalImpulse =
                                        m.pointee.points[pi].totalNormalImpulse[lane]
                                    if hitEvents {
                                        out.pointee.points[pi].normalVelocity =
                                            m.pointee.points[pi].relativeVelocity[lane]
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
            }
        }
    }

    /// Scatter accumulated impulses once, after all substeps, for contact persistence and events.
    func unpackWideImpulses() {
        guard enableWideSolver else { return }
        for ci in 0..<Int(overflowIndex) {
            let list = solverContacts.constraints[ci]
            let wide = solverContacts.wideConstraints[ci]
            let manifolds = solverContacts.wideManifolds
            solverContacts.manifoldPool.withUnsafeMutableBufferPointer { pool in
                for c in wide {
                    for lane in 0..<4 {
                        let index = c.scalarIndices[lane]
                        guard index >= 0 else { continue }
                        let s = list[Int(index)]
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

    @inline(__always)
    static func gatherStatesW(
        _ states: UnsafeMutableBufferPointer<BodyState>, _ indices: SIMD4<Int32>
    ) -> BodyStateW {
        let a = stateAt(states, indices[0])
        let b = stateAt(states, indices[1])
        let c = stateAt(states, indices[2])
        let d = stateAt(states, indices[3])
        var out = BodyStateW()
        out.v = Vec3W(a.linearVelocity, b.linearVelocity, c.linearVelocity, d.linearVelocity)
        out.w = Vec3W(a.angularVelocity, b.angularVelocity, c.angularVelocity, d.angularVelocity)
        out.dp = Vec3W(a.deltaPosition, b.deltaPosition, c.deltaPosition, d.deltaPosition)
        out.dq = QuatW(
            v: Vec3W(a.deltaRotation.v, b.deltaRotation.v, c.deltaRotation.v, d.deltaRotation.v),
            s: FloatW(a.deltaRotation.s, b.deltaRotation.s, c.deltaRotation.s, d.deltaRotation.s))
        return out
    }

    @inline(__always)
    static func scatterStatesW(
        _ states: UnsafeMutableBufferPointer<BodyState>, _ indices: SIMD4<Int32>, _ value: BodyStateW
    ) {
        for lane in 0..<4 {
            let index = indices[lane]
            if index >= 0 && (states[Int(index)].flags & BodyFlags.dynamicFlag.rawValue) != 0 {
                states[Int(index)].linearVelocity = value.v[lane]
                states[Int(index)].angularVelocity = value.w[lane]
            }
        }
    }
}
