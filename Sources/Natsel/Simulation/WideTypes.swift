

// 4-wide solver types (simd_wide.h, contact_solver_wide.inl).
//
// The C solver runs four contacts at a time through NEON when the target is arm64. Its results are
// bit-identical to the scalar path (verified against `b3World_EnableSSE2Fallback`), so the wide
// port only has to preserve the scalar arithmetic order within each lane.

/// Four floats, one per contact lane.
typealias FloatW = SIMD4<Float>

/// Four 3-vectors, one per lane. `x`, `y`, and `z` each hold four lanes.
struct Vec3W {
    var x = FloatW(repeating: 0)
    var y = FloatW(repeating: 0)
    var z = FloatW(repeating: 0)

    init() {}

    init(x: FloatW, y: FloatW, z: FloatW) {
        self.x = x
        self.y = y
        self.z = z
    }

    init(_ x: FloatW, _ y: FloatW, _ z: FloatW) {
        self.x = x
        self.y = y
        self.z = z
    }

    /// Replicate one vector across every lane.
    init(splat v: Vec3) {
        x = FloatW(repeating: v.x)
        y = FloatW(repeating: v.y)
        z = FloatW(repeating: v.z)
    }

    /// Gather one vector per lane from four sources.
    init(_ a: Vec3, _ b: Vec3, _ c: Vec3, _ d: Vec3) {
        x = FloatW(a.x, b.x, c.x, d.x)
        y = FloatW(a.y, b.y, c.y, d.y)
        z = FloatW(a.z, b.z, c.z, d.z)
    }

    /// Lane `i` of the wide vector.
    @inline(__always)
    subscript(_ i: Int) -> Vec3 {
        get { Vec3(x[i], y[i], z[i]) }
        set {
            x[i] = newValue.x
            y[i] = newValue.y
            z[i] = newValue.z
        }
    }
}

/// Four 2-vectors, one per lane.
struct Vec2W {
    var x = FloatW(repeating: 0)
    var y = FloatW(repeating: 0)

    init() {}

    init(x: FloatW, y: FloatW) {
        self.x = x
        self.y = y
    }

    init(_ a: Vec2, _ b: Vec2, _ c: Vec2, _ d: Vec2) {
        x = FloatW(a.x, b.x, c.x, d.x)
        y = FloatW(a.y, b.y, c.y, d.y)
    }

    @inline(__always)
    subscript(_ i: Int) -> Vec2 {
        get { Vec2(x[i], y[i]) }
        set {
            x[i] = newValue.x
            y[i] = newValue.y
        }
    }
}

/// Four quaternions, one per lane.
struct QuatW {
    var v = Vec3W()
    var s = FloatW(repeating: 0)

    init() {}

    init(v: Vec3W, s: FloatW) {
        self.v = v
        self.s = s
    }
}

/// A symmetric 3x3 matrix, four lanes. Only the six unique entries are stored.
struct SymMatrix3W {
    var xx = FloatW(repeating: 0)
    var yy = FloatW(repeating: 0)
    var zz = FloatW(repeating: 0)
    var xy = FloatW(repeating: 0)
    var xz = FloatW(repeating: 0)
    var yz = FloatW(repeating: 0)

    init() {}

    init(_ a: SymMatrix3, _ b: SymMatrix3, _ c: SymMatrix3, _ d: SymMatrix3) {
        xx = FloatW(a.cxx, b.cxx, c.cxx, d.cxx)
        yy = FloatW(a.cyy, b.cyy, c.cyy, d.cyy)
        zz = FloatW(a.czz, b.czz, c.czz, d.czz)
        xy = FloatW(a.cxy, b.cxy, c.cxy, d.cxy)
        xz = FloatW(a.cxz, b.cxz, c.cxz, d.cxz)
        yz = FloatW(a.cyz, b.cyz, c.cyz, d.cyz)
    }

}

/// A symmetric 2x2 matrix, four lanes.
struct SymMatrix2W {
    var xx = FloatW(repeating: 0)
    var yy = FloatW(repeating: 0)
    var xy = FloatW(repeating: 0)

    init() {}

    init(_ a: Matrix2, _ b: Matrix2, _ c: Matrix2, _ d: Matrix2) {
        xx = FloatW(a.cx.x, b.cx.x, c.cx.x, d.cx.x)
        yy = FloatW(a.cy.y, b.cy.y, c.cy.y, d.cy.y)
        xy = FloatW(a.cx.y, b.cx.y, c.cx.y, d.cx.y)
    }

}

// MARK: - Lane-wise operations

@inline(__always) func addW(_ a: Vec3W, _ b: Vec3W) -> Vec3W {
    Vec3W(x: a.x + b.x, y: a.y + b.y, z: a.z + b.z)
}

@inline(__always) func subW(_ a: Vec3W, _ b: Vec3W) -> Vec3W {
    Vec3W(x: a.x - b.x, y: a.y - b.y, z: a.z - b.z)
}

@inline(__always) func dotW(_ a: Vec3W, _ b: Vec3W) -> FloatW {
    a.x * b.x + a.y * b.y + a.z * b.z
}

@inline(__always) func crossW(_ a: Vec3W, _ b: Vec3W) -> Vec3W {
    Vec3W(
        x: a.y * b.z - a.z * b.y,
        y: a.z * b.x - a.x * b.z,
        z: a.x * b.y - a.y * b.x)
}

@inline(__always) func mulAddW(_ a: Vec3W, _ b: FloatW, _ c: Vec3W) -> Vec3W {
    Vec3W(x: a.x + b * c.x, y: a.y + b * c.y, z: a.z + b * c.z)
}

@inline(__always) func mulMVW(_ m: SymMatrix3W, _ a: Vec3W) -> Vec3W {
    Vec3W(
        x: m.xx * a.x + m.xy * a.y + m.xz * a.z,
        y: m.xy * a.x + m.yy * a.y + m.yz * a.z,
        z: m.xz * a.x + m.yz * a.y + m.zz * a.z)
}

@inline(__always) func mulMV2W(_ m: SymMatrix2W, _ a: Vec2W) -> Vec2W {
    Vec2W(x: m.xx * a.x + m.xy * a.y, y: m.xy * a.x + m.yy * a.y)
}

@inline(__always) func negW(_ a: Vec3W) -> Vec3W {
    Vec3W(x: -a.x, y: -a.y, z: -a.z)
}

// MARK: - Body state lanes

/// Four body states, gathered for the wide solver. Lane `i` mirrors `states[index[i] - 1]`, or the
/// identity state when the index is zero (static).
struct BodyStateW {
    var v = Vec3W()
    var w = Vec3W()
    var dp = Vec3W()
    var dq = QuatW()
}

extension World {
    /// Gather four body states. Index zero means static and yields the identity state, matching C's
    /// `b3GatherBodies`.
    func gatherBodyStates(_ indices: SIMD4<Int32>) -> BodyStateW {
        var states = [BodyState](repeating: BodyState.identity, count: 4)
        for lane in 0..<4 {
            let index = indices[lane]
            if index != 0 {
                states[lane] = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(index - 1)]
            }
        }

        var out = BodyStateW()
        out.v = Vec3W(
            FloatW(states[0].linearVelocity.x, states[1].linearVelocity.x,
                   states[2].linearVelocity.x, states[3].linearVelocity.x),
            FloatW(states[0].linearVelocity.y, states[1].linearVelocity.y,
                   states[2].linearVelocity.y, states[3].linearVelocity.y),
            FloatW(states[0].linearVelocity.z, states[1].linearVelocity.z,
                   states[2].linearVelocity.z, states[3].linearVelocity.z))
        out.w = Vec3W(
            FloatW(states[0].angularVelocity.x, states[1].angularVelocity.x,
                   states[2].angularVelocity.x, states[3].angularVelocity.x),
            FloatW(states[0].angularVelocity.y, states[1].angularVelocity.y,
                   states[2].angularVelocity.y, states[3].angularVelocity.y),
            FloatW(states[0].angularVelocity.z, states[1].angularVelocity.z,
                   states[2].angularVelocity.z, states[3].angularVelocity.z))
        out.dp = Vec3W(
            FloatW(states[0].deltaPosition.x, states[1].deltaPosition.x,
                   states[2].deltaPosition.x, states[3].deltaPosition.x),
            FloatW(states[0].deltaPosition.y, states[1].deltaPosition.y,
                   states[2].deltaPosition.y, states[3].deltaPosition.y),
            FloatW(states[0].deltaPosition.z, states[1].deltaPosition.z,
                   states[2].deltaPosition.z, states[3].deltaPosition.z))
        out.dq = QuatW(
            v: Vec3W(
                FloatW(states[0].deltaRotation.v.x, states[1].deltaRotation.v.x,
                       states[2].deltaRotation.v.x, states[3].deltaRotation.v.x),
                FloatW(states[0].deltaRotation.v.y, states[1].deltaRotation.v.y,
                       states[2].deltaRotation.v.y, states[3].deltaRotation.v.y),
                FloatW(states[0].deltaRotation.v.z, states[1].deltaRotation.v.z,
                       states[2].deltaRotation.v.z, states[3].deltaRotation.v.z)),
            s: FloatW(states[0].deltaRotation.s, states[1].deltaRotation.s,
                      states[2].deltaRotation.s, states[3].deltaRotation.s))
        return out
    }

    /// Write the gathered velocities back. Lanes that are static or non-dynamic are skipped.
    func scatterBodyStates(_ indices: SIMD4<Int32>, _ wide: BodyStateW) {
        let setIndex = Int(SetType.awakeSet.rawValue)
        for lane in 0..<4 {
            let index = indices[lane]
            if index == 0 { continue }
            let local = Int(index - 1)
            var state = solverSets[setIndex].bodyStates[local]
            guard (state.flags & BodyFlags.dynamicFlag.rawValue) != 0 else { continue }
            state.linearVelocity = wide.v[lane]
            state.angularVelocity = wide.w[lane]
            solverSets[setIndex].bodyStates[local] = state
        }
    }

    /// Inverse-rotate four vectors by four quaternions, matching the wide scalar helper
    /// `invRotateVectorWide`. That helper is deliberately not bit-equivalent to the generic
    /// `invRotateVector`, so the lane math has to follow it exactly.
    @inline(__always)
    func invRotateVecW(_ q: QuatW, _ a: Vec3W) -> Vec3W {
        var t = crossW(q.v, a)
        t = Vec3W(t.x + t.x, t.y + t.y, t.z + t.z)
        let u = crossW(q.v, t)
        return Vec3W(
            (a.x - q.s * t.x) + u.x,
            (a.y - q.s * t.y) + u.y,
            (a.z - q.s * t.z) + u.z)
    }
}

/// Select `a` where `mask` is non-zero, else `b`. A float mask keeps the signature expressible:
/// Swift's `SIMDMask` only covers integer vectors.
@inline(__always) func selectW(_ mask: FloatW, _ a: FloatW, _ b: FloatW) -> FloatW {
    var out = FloatW(repeating: 0)
    for lane in 0..<4 {
        out[lane] = mask[lane] != 0 ? a[lane] : b[lane]
    }
    return out
}

/// Signed comparison used as a float mask: 1 where `a > b`, else 0.
@inline(__always) func greaterW(_ a: FloatW, _ b: FloatW) -> FloatW {
    var out = FloatW(repeating: 0)
    for lane in 0..<4 {
        out[lane] = a[lane] > b[lane] ? 1 : 0
    }
    return out
}

@inline(__always) func maxW(_ a: FloatW, _ b: FloatW) -> FloatW {
    var out = FloatW(repeating: 0)
    for lane in 0..<4 {
        out[lane] = maxFloat(a[lane], b[lane])
    }
    return out
}

@inline(__always) func mulSVW(_ s: FloatW, _ a: Vec3W) -> Vec3W {
    Vec3W(a.x * s, a.y * s, a.z * s)
}

@inline(__always) func mulW(_ s: FloatW, _ a: Vec3W) -> Vec3W {
    Vec3W(x: a.x * s, y: a.y * s, z: a.z * s)
}
