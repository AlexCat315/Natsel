

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
        x: m.xx * a.x + (m.xy * a.y + m.xz * a.z),
        y: m.xy * a.x + (m.yy * a.y + m.yz * a.z),
        z: m.xz * a.x + (m.yz * a.y + m.zz * a.z))
}

@inline(__always) func mulMV2W(_ m: SymMatrix2W, _ a: Vec2W) -> Vec2W {
    Vec2W(x: m.xx * a.x + m.xy * a.y, y: m.xy * a.x + m.yy * a.y)
}

@inline(__always) func negW(_ a: Vec3W) -> Vec3W {
    Vec3W(x: negW(a.x), y: negW(a.y), z: negW(a.z))
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

/// Selection preserves the scalar operand order, including signed zeros and NaNs.
@inline(__always) func selectW(_ mask: SIMDMask<SIMD4<Int32>>, _ a: FloatW, _ b: FloatW) -> FloatW {
    b.replacing(with: a, where: mask)
}

@inline(__always) func selectW(_ mask: SIMDMask<SIMD4<Int32>>, _ a: Vec3W, _ b: Vec3W) -> Vec3W {
    Vec3W(selectW(mask, a.x, b.x), selectW(mask, a.y, b.y), selectW(mask, a.z, b.z))
}

@inline(__always) func maxW(_ a: FloatW, _ b: FloatW) -> FloatW {
    selectW(a .> b, a, b)
}

@inline(__always) func minW(_ a: FloatW, _ b: FloatW) -> FloatW {
    selectW(a .< b, a, b)
}

@inline(__always) func mulSVW(_ s: FloatW, _ a: Vec3W) -> Vec3W {
    Vec3W(s * a.x, s * a.y, s * a.z)
}

@inline(__always) func mulW(_ s: FloatW, _ a: Vec3W) -> Vec3W { mulSVW(s, a) }

/// SIMD's generic unary minus uses zero subtraction, which loses the sign of +0.
/// Flip the IEEE sign bit to match scalar Float negation, including zero and NaNs.
@inline(__always) func negW(_ a: FloatW) -> FloatW {
    let bits = unsafeBitCast(a, to: SIMD4<UInt32>.self)
    return unsafeBitCast(bits ^ SIMD4<UInt32>(repeating: 0x8000_0000), to: FloatW.self)
}
