// Direct SoA preparation for independent colors. Lanes are filled straight
// from the contacts, so wide colors no longer build scalar constraints only to repack them.

extension World {
    /// Prepare one batch of up to four contacts into its preassigned slots. Its manifolds go to
    /// `manifoldOut`, uninitialized memory with room for the largest lane manifold count.
    func prepareWideBatch(
        _ ids: UnsafePointer<Int32>, _ laneCount: Int, _ out: UnsafeMutablePointer<ContactConstraintW>,
        _ manifoldOut: UnsafeMutablePointer<ManifoldConstraintW>, _ manifoldStart: Int,
        _ contactBase: UnsafePointer<Contact>, _ sims: UnsafePointer<BodySim>,
        _ states: UnsafeMutableBufferPointer<BodyState>,
        _ softness: Softness, _ staticSoftness: Softness, _ warmStartScale: Float
    ) {
        let invTau = FloatW(repeating: 1 / B3.speculativeDistance)
        let minFrictionWeight = FloatW(repeating: B3.minFrictionWeight)
        let one = FloatW(repeating: 1)
        let two = FloatW(repeating: 2)
        let minDet = FloatW(repeating: 1000 * Float.leastNormalMagnitude)
        let ws = FloatW(repeating: warmStartScale)
        let dynamicBias = softness.massScale * softness.biasRate
        let staticBias = staticSoftness.massScale * staticSoftness.biasRate
        let hitFlag = ContactFlags.simEnableHitEvent.rawValue
        let staticFlag = ContactFlags.staticFlag.rawValue

        var c = ContactConstraintW()
        var manifolds = InlineArray<4, UnsafePointer<Manifold>?>(repeating: nil)
        var manifoldCounts = SIMD4<Int32>.zero
        var needVelocity = SIMDMask<SIMD4<Int32>>(repeating: false)
        var tangentVelocity = Vec3W()
        var simLanesA = InlineArray<4, UnsafeRawPointer?>(repeating: nil)
        var simLanesB = InlineArray<4, UnsafeRawPointer?>(repeating: nil)
        // Constant lane indices let the compiler insert each scalar directly
        // into its vector instead of spilling through a dynamic SIMD subscript.
        @inline(__always) func gatherLane(_ lane: Int) {
            guard lane < laneCount else { return }
            let id = ids[lane]
            let contact = contactBase + Int(id)
            let indexA = decodeAwakeIndex(contact.pointee.encodedBodySimA)
            let indexB = decodeAwakeIndex(contact.pointee.encodedBodySimB)
            c.contactIds[lane] = id
            c.indexA[lane] = indexA
            c.indexB[lane] = indexB
            if indexA >= 0 {
                let sim = sims + Int(indexA)
                c.invMassA[lane] = sim.pointee.invMass
                simLanesA[lane] = UnsafeRawPointer(sim)
            }
            if indexB >= 0 {
                let sim = sims + Int(indexB)
                c.invMassB[lane] = sim.pointee.invMass
                simLanesB[lane] = UnsafeRawPointer(sim)
            }
            let isStatic = (contact.pointee.flags & staticFlag) != 0
            let soft = isStatic ? staticSoftness : softness
            c.biasRate[lane] = isStatic ? staticBias : dynamicBias
            c.massScale[lane] = soft.massScale
            c.impulseScale[lane] = soft.impulseScale
            let restitution = contact.pointee.restitution
            let rolling = contact.pointee.rollingResistance
            c.friction[lane] = contact.pointee.friction
            c.restitution[lane] = restitution
            c.rollingResistance[lane] = rolling
            c.anyRolling = c.anyRolling || rolling > 0
            c.anyRestitution = c.anyRestitution || restitution > 0
            needVelocity[lane] =
                restitution > 0 || (contact.pointee.flags & hitFlag) != 0
            tangentVelocity[lane] = contact.pointee.tangentVelocity
            let mc = contact.pointee.manifoldCount
            manifoldCounts[lane] = mc
            c.manifoldCount = max(c.manifoldCount, Int(mc))
            if mc > 0 {
                manifolds[lane] = UnsafePointer(contact.pointee.manifolds.buffer.baseAddress!)
            }
        }
        gatherLane(0)
        gatherLane(1)
        gatherLane(2)
        gatherLane(3)
        c.invIA = gatherInvInertiaW(simLanesA)
        c.invIB = gatherInvInertiaW(simLanesB)
        c.laneManifoldCounts = manifoldCounts

        let mA = c.invMassA
        let mB = c.invMassB
        let iA = c.invIA
        let iB = c.invIB
        let invIAB = addSymW(iA, iB)
        let rollingMask = c.rollingResistance .> .zero
        if c.anyRolling {
            let inv = invertSymW(invIAB)
            c.rollingMass.xx = selectW(rollingMask, inv.xx, .zero)
            c.rollingMass.yy = selectW(rollingMask, inv.yy, .zero)
            c.rollingMass.zz = selectW(rollingMask, inv.zz, .zero)
            c.rollingMass.xy = selectW(rollingMask, inv.xy, .zero)
            c.rollingMass.xz = selectW(rollingMask, inv.xz, .zero)
            c.rollingMass.yz = selectW(rollingMask, inv.yz, .zero)
        }

        var stateA = BodyStateW()
        var stateB = BodyStateW()
        let anyVelocity = any(needVelocity)
        if anyVelocity {
            stateA = Self.gatherStatesW(states, c.indexA)
            stateB = Self.gatherStatesW(states, c.indexB)
        }

        c.manifoldStart = manifoldStart
        for offset in 0..<c.manifoldCount {
            var m = ManifoldConstraintW()
            var lanes = InlineArray<4, UnsafePointer<Manifold>?>(repeating: nil)
            for lane in 0..<4 where offset < Int(manifoldCounts[lane]) {
                let sm = manifolds[lane]! + offset
                lanes[lane] = sm
                m.exists[lane] = 1
                let pc = sm.pointee.pointCount
                m.pointCounts[lane] = pc
                m.pointCount = max(m.pointCount, Int(pc))
            }
            let header = gatherManifoldHeaderW(lanes)
            let normal = header.normal
            let twistImpulse = header.twistImpulse
            let frictionImpulse = header.frictionImpulse
            let rollingImpulse = header.rollingImpulse

            let tangent1 = perpW(normal)
            let tangent2 = crossW(tangent1, normal)
            m.normal = normal
            m.tangent1 = tangent1
            m.tangent2 = tangent2
            m.tangentVelocity1 = dotW(tangentVelocity, tangent1)
            m.tangentVelocity2 = dotW(tangentVelocity, tangent2)
            m.twistImpulse = ws * twistImpulse
            let rollingKeep = rollingMask .& (m.exists .> .zero)
            m.rollingImpulse = selectW(rollingKeep, mulW(ws, rollingImpulse), Vec3W())
            m.frictionImpulse = Vec2W(
                x: ws * dotW(frictionImpulse, tangent1), y: ws * dotW(frictionImpulse, tangent2))

            var centerA = Vec3W()
            var centerB = Vec3W()
            var totalWeight = FloatW.zero
            for pi in 0..<m.pointCount {
                let point = gatherManifoldPointW(lanes, pi)
                let rA = point.rA
                let rB = point.rB
                let separation = point.separation
                let normalImpulse = point.normalImpulse
                let pointMask = m.pointCounts .> SIMD4<Int32>(repeating: Int32(pi))
                var weight = minW(maxW(two - separation * invTau, minFrictionWeight), one)
                weight = selectW(pointMask, weight, .zero)
                centerA = mulAddW(centerA, weight, rA)
                centerB = mulAddW(centerB, weight, rB)
                totalWeight += weight

                var cp = ConstraintPointW()
                cp.rA = rA
                cp.rB = rB
                cp.baseSeparation = separation - dotW(subW(rB, rA), normal)
                cp.normalImpulse = ws * normalImpulse

                let rnA = crossW(rA, normal)
                let rnB = crossW(rB, normal)
                var kNormal = mA + mB
                kNormal += dotW(rnA, mulMVW(iA, rnA))
                kNormal += dotW(rnB, mulMVW(iB, rnB))
                cp.normalMass = selectW((kNormal .> .zero) .& pointMask, one / kNormal, .zero)
                m.points[pi] = cp
            }

            let frictionValid = totalWeight .> .zero
            let invWeight = selectW(frictionValid, one / totalWeight, .zero)
            centerA = mulW(invWeight, centerA)
            centerB = mulW(invWeight, centerB)
            m.centerA = centerA
            m.centerB = centerB

            for pi in 0..<m.pointCount {
                let pointMask = m.pointCounts .> SIMD4<Int32>(repeating: Int32(pi))
                let d = subW(m.points[pi].rA, centerA)
                m.points[pi].leverArm = selectW(pointMask, dotW(d, d).squareRoot(), .zero)
            }

            do {
                let rtA1 = crossW(centerA, tangent1)
                let rtA2 = crossW(centerA, tangent2)
                let rtB1 = crossW(centerB, tangent1)
                let rtB2 = crossW(centerB, tangent2)
                let iArtA1 = mulMVW(iA, rtA1)
                let iArtA2 = mulMVW(iA, rtA2)
                let iBrtB1 = mulMVW(iB, rtB1)
                let iBrtB2 = mulMVW(iB, rtB2)
                let mSum = mA + mB
                let kxx = (mSum + dotW(rtA1, iArtA1)) + dotW(rtB1, iBrtB1)
                let kyy = (mSum + dotW(rtA2, iArtA2)) + dotW(rtB2, iBrtB2)
                let kxy = dotW(rtA1, iArtA2) + dotW(rtB1, iBrtB2)
                let det = kxx * kyy - kxy * kxy
                let valid = (absW(det) .> minDet) .& frictionValid
                let invDet = selectW(valid, one / det, .zero)
                m.tangentMass.xx = invDet * kyy
                m.tangentMass.xy = negW(invDet * kxy)
                m.tangentMass.yy = invDet * kxx
            }

            let kTwist = dotW(normal, mulMVW(invIAB, normal))
            m.twistMass = selectW(kTwist .> .zero, one / kTwist, .zero)

            if anyVelocity {
                for pi in 0..<m.pointCount {
                    let rA = m.points[pi].rA
                    let rB = m.points[pi].rB
                    let vrA = addW(stateA.v, crossW(stateA.w, rA))
                    let vrB = addW(stateB.v, crossW(stateB.w, rB))
                    let vn = dotW(normal, subW(vrB, vrA))
                    let pointMask = m.pointCounts .> SIMD4<Int32>(repeating: Int32(pi))
                    m.points[pi].relativeVelocity = selectW(needVelocity .& pointMask, vn, .zero)
                }
            }
            (manifoldOut + offset).initialize(to: m)
        }
        out.initialize(to: c)
    }
}

@inline(__always) func absW(_ a: FloatW) -> FloatW {
    let bits = unsafeBitCast(a, to: SIMD4<UInt32>.self)
    return unsafeBitCast(bits & SIMD4<UInt32>(repeating: 0x7FFF_FFFF), to: FloatW.self)
}

@inline(__always) func addSymW(_ a: SymMatrix3W, _ b: SymMatrix3W) -> SymMatrix3W {
    var m = SymMatrix3W()
    m.xx = a.xx + b.xx
    m.xy = a.xy + b.xy
    m.xz = a.xz + b.xz
    m.yy = a.yy + b.yy
    m.yz = a.yz + b.yz
    m.zz = a.zz + b.zz
    return m
}

/// Lane-wise `invertSym`.
@inline(__always) func invertSymW(_ m: SymMatrix3W) -> SymMatrix3W {
    let cxx = m.yy * m.zz - m.yz * m.yz
    let cxy = m.xz * m.yz - m.xy * m.zz
    let cxz = m.xy * m.yz - m.xz * m.yy
    let cyy = m.xx * m.zz - m.xz * m.xz
    let cyz = m.xy * m.xz - m.xx * m.yz
    let czz = m.xx * m.yy - m.xy * m.xy
    let det = m.xx * cxx + (m.xy * cxy + m.xz * cxz)
    let valid = absW(det) .> FloatW(repeating: 1000 * Float.leastNormalMagnitude)
    let invDet = selectW(valid, FloatW(repeating: 1) / det, .zero)
    var r = SymMatrix3W()
    r.xx = invDet * cxx
    r.xy = invDet * cxy
    r.xz = invDet * cxz
    r.yy = invDet * cyy
    r.yz = invDet * cyz
    r.zz = invDet * czz
    return r
}

/// Lane-wise `perp`.
@inline(__always) func perpW(_ a: Vec3W) -> Vec3W {
    let half = FloatW(repeating: 0.5)
    let mask = (a.x .< negW(half)) .| (a.x .> half)
    let p = Vec3W(
        selectW(mask, a.y, .zero),
        selectW(mask, negW(a.x), a.z),
        selectW(mask, .zero, negW(a.y)))
    let lengthSquared = dotW(p, p)
    let valid = lengthSquared .> FloatW(repeating: 1000 * Float.leastNormalMagnitude)
    let s = selectW(valid, FloatW(repeating: 1) / lengthSquared.squareRoot(), .zero)
    return mulW(s, p)
}

// MARK: - Row gathers

@inline(__always) private func row(_ p: UnsafeRawPointer?, _ offset: Int) -> FloatW {
    guard let p else { return .zero }
    return p.loadUnaligned(fromByteOffset: offset, as: FloatW.self)
}

@inline(__always) private func scalar(_ p: UnsafeRawPointer?, _ offset: Int) -> Float {
    guard let p else { return 0 }
    return p.load(fromByteOffset: offset, as: Float.self)
}

private let invInertiaOffset = MemoryLayout<BodySim>.offset(of: \.invInertiaWorld)!

/// The upper triangle of each lane's world inverse inertia. Absent lanes are zero.
@inline(__always) func gatherInvInertiaW(_ sims: InlineArray<4, UnsafeRawPointer?>) -> SymMatrix3W {
    let o = invInertiaOffset
    let r0 = transpose4(row(sims[0], o), row(sims[1], o), row(sims[2], o), row(sims[3], o))
    let r1 = transpose4(row(sims[0], o + 16), row(sims[1], o + 16), row(sims[2], o + 16), row(sims[3], o + 16))
    var m = SymMatrix3W()
    m.xx = r0.0
    m.xy = r0.1
    m.xz = r0.2
    m.yy = r1.0
    m.yz = r1.1
    m.zz = FloatW(scalar(sims[0], o + 32), scalar(sims[1], o + 32), scalar(sims[2], o + 32), scalar(sims[3], o + 32))
    return m
}

private let manifoldNormalOffset = MemoryLayout<Manifold>.offset(of: \.normal)!
private let manifoldPointStride = MemoryLayout<ManifoldPoint>.stride

struct ManifoldHeaderW {
    var normal: Vec3W
    var twistImpulse: FloatW
    var frictionImpulse: Vec3W
    var rollingImpulse: Vec3W
}

/// normal, twist, friction, and rolling impulses are contiguous after the points. Absent lanes are zero.
@inline(__always) func gatherManifoldHeaderW(_ lanes: InlineArray<4, UnsafePointer<Manifold>?>) -> ManifoldHeaderW {
    let p0 = lanes[0].map(UnsafeRawPointer.init), p1 = lanes[1].map(UnsafeRawPointer.init)
    let p2 = lanes[2].map(UnsafeRawPointer.init), p3 = lanes[3].map(UnsafeRawPointer.init)
    let o = manifoldNormalOffset
    let a = transpose4(row(p0, o), row(p1, o), row(p2, o), row(p3, o))
    let b = transpose4(row(p0, o + 16), row(p1, o + 16), row(p2, o + 16), row(p3, o + 16))
    return ManifoldHeaderW(
        normal: Vec3W(a.0, a.1, a.2), twistImpulse: a.3,
        frictionImpulse: Vec3W(b.0, b.1, b.2),
        rollingImpulse: Vec3W(
            b.3,
            FloatW(scalar(p0, o + 32), scalar(p1, o + 32), scalar(p2, o + 32), scalar(p3, o + 32)),
            FloatW(scalar(p0, o + 36), scalar(p1, o + 36), scalar(p2, o + 36), scalar(p3, o + 36))))
}

struct ManifoldPointW {
    var rA: Vec3W
    var rB: Vec3W
    var separation: FloatW
    var normalImpulse: FloatW
}

/// anchorA, anchorB, separation, and normal impulse of point `index`. Lanes without the point are zero.
@inline(__always) func gatherManifoldPointW(_ lanes: InlineArray<4, UnsafePointer<Manifold>?>, _ index: Int) -> ManifoldPointW {
    @inline(__always) func point(_ lane: Int) -> UnsafeRawPointer? {
        guard let m = lanes[lane], index < Int(m.pointee.pointCount) else { return nil }
        return UnsafeRawPointer(m) + index * manifoldPointStride
    }
    let p0 = point(0), p1 = point(1), p2 = point(2), p3 = point(3)
    let a = transpose4(row(p0, 0), row(p1, 0), row(p2, 0), row(p3, 0))
    let b = transpose4(row(p0, 16), row(p1, 16), row(p2, 16), row(p3, 16))
    return ManifoldPointW(rA: Vec3W(a.0, a.1, a.2), rB: Vec3W(a.3, b.0, b.1), separation: b.2, normalImpulse: b.3)
}
