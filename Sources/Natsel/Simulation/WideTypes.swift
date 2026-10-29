import Builtin
#if canImport(simd)
private import simd
#endif

// Four-wide solver types. Each lane preserves the scalar arithmetic order.

/// Four floats, one per contact lane.
///
/// Swift 6.4 lowers `SIMD4<Float>` arithmetic to scalar lane operations. Keeping the lane vector
/// as a `Builtin.Vec4xFPIEEE32` lets the four-wide helpers below emit single NEON instructions.
/// All `Builtin` use in the engine lives in this type and its helpers.
struct FloatW: @unchecked Sendable {
    var storage: Builtin.Vec4xFPIEEE32

    @inline(__always) init() { storage = Builtin.zeroInitializer() }
    @inline(__always) init(storage: Builtin.Vec4xFPIEEE32) { self.storage = storage }
    @inline(__always) init(_ a: SIMD4<Float>) { storage = Self.load(a) }
    @inline(__always) init(repeating value: Float) {
        storage = Self.load(SIMD4<Float>(repeating: value))
    }
    @inline(__always) init(_ x: Float, _ y: Float, _ z: Float, _ w: Float) {
        storage = Self.load(SIMD4<Float>(x, y, z, w))
    }

    @inline(__always) private static func load(_ value: SIMD4<Float>) -> Builtin.Vec4xFPIEEE32 {
        var value = value
        return Builtin.loadRaw(Builtin.addressof(&value))
    }

    @inline(__always) var simd: SIMD4<Float> {
        var out = SIMD4<Float>()
        Builtin.storeRaw(storage, Builtin.addressof(&out))
        return out
    }

    @inline(__always) var x: Float {
        get { simd[0] }
        set { var value = simd; value[0] = newValue; storage = Self.load(value) }
    }
    @inline(__always) var y: Float {
        get { simd[1] }
        set { var value = simd; value[1] = newValue; storage = Self.load(value) }
    }
    @inline(__always) var z: Float {
        get { simd[2] }
        set { var value = simd; value[2] = newValue; storage = Self.load(value) }
    }
    @inline(__always) var w: Float {
        get { simd[3] }
        set { var value = simd; value[3] = newValue; storage = Self.load(value) }
    }

    @inline(__always) subscript(_ i: Int) -> Float {
        get { simd[i] }
        set { var value = simd; value[i] = newValue; storage = Self.load(value) }
    }

    @inline(__always) func squareRoot() -> FloatW { FloatW(simd.squareRoot()) }

    @inline(__always) static var zero: FloatW { FloatW(repeating: 0) }
    @inline(__always) static var one: FloatW { FloatW(repeating: 1) }

    @inline(__always) static func + (a: FloatW, b: FloatW) -> FloatW { wideAdd(a, b) }
    @inline(__always) static func - (a: FloatW, b: FloatW) -> FloatW { wideSub(a, b) }
    @inline(__always) static func * (a: FloatW, b: FloatW) -> FloatW { wideMul(a, b) }
    @inline(__always) static func / (a: FloatW, b: FloatW) -> FloatW { wideDiv(a, b) }
    @inline(__always) static prefix func - (a: FloatW) -> FloatW { wideNeg(a) }

    @inline(__always) static func + (a: FloatW, b: Float) -> FloatW { wideAdd(a, FloatW(repeating: b)) }
    @inline(__always) static func + (a: Float, b: FloatW) -> FloatW { wideAdd(FloatW(repeating: a), b) }
    @inline(__always) static func - (a: FloatW, b: Float) -> FloatW { wideSub(a, FloatW(repeating: b)) }
    @inline(__always) static func - (a: Float, b: FloatW) -> FloatW { wideSub(FloatW(repeating: a), b) }
    @inline(__always) static func * (a: FloatW, b: Float) -> FloatW { wideMul(a, FloatW(repeating: b)) }
    @inline(__always) static func * (a: Float, b: FloatW) -> FloatW { wideMul(FloatW(repeating: a), b) }
    @inline(__always) static func / (a: FloatW, b: Float) -> FloatW { wideDiv(a, FloatW(repeating: b)) }
    @inline(__always) static func / (a: Float, b: FloatW) -> FloatW { wideDiv(FloatW(repeating: a), b) }

    @inline(__always) static func += (a: inout FloatW, b: FloatW) { a = wideAdd(a, b) }
    @inline(__always) static func -= (a: inout FloatW, b: FloatW) { a = wideSub(a, b) }
    @inline(__always) static func *= (a: inout FloatW, b: FloatW) { a = wideMul(a, b) }
    @inline(__always) static func /= (a: inout FloatW, b: FloatW) { a = wideDiv(a, b) }

    @inline(__always) static func .> (a: FloatW, b: FloatW) -> SIMDMask<SIMD4<Int32>> {
        a.simd .> b.simd
    }
    @inline(__always) static func .< (a: FloatW, b: FloatW) -> SIMDMask<SIMD4<Int32>> {
        a.simd .< b.simd
    }
    @inline(__always) static func .>= (a: FloatW, b: FloatW) -> SIMDMask<SIMD4<Int32>> {
        a.simd .>= b.simd
    }
    @inline(__always) static func .<= (a: FloatW, b: FloatW) -> SIMDMask<SIMD4<Int32>> {
        a.simd .<= b.simd
    }
    @inline(__always) static func .== (a: FloatW, b: FloatW) -> SIMDMask<SIMD4<Int32>> {
        a.simd .== b.simd
    }
    @inline(__always) static func .!= (a: FloatW, b: FloatW) -> SIMDMask<SIMD4<Int32>> {
        a.simd .!= b.simd
    }
    @inline(__always) static func .> (a: FloatW, b: Float) -> SIMDMask<SIMD4<Int32>> {
        a.simd .> b
    }
    @inline(__always) static func .< (a: FloatW, b: Float) -> SIMDMask<SIMD4<Int32>> {
        a.simd .< b
    }
    @inline(__always) static func .>= (a: FloatW, b: Float) -> SIMDMask<SIMD4<Int32>> {
        a.simd .>= b
    }
    @inline(__always) static func .<= (a: FloatW, b: Float) -> SIMDMask<SIMD4<Int32>> {
        a.simd .<= b
    }
    @inline(__always) static func .> (a: Float, b: FloatW) -> SIMDMask<SIMD4<Int32>> {
        a .> b.simd
    }
    @inline(__always) static func .< (a: Float, b: FloatW) -> SIMDMask<SIMD4<Int32>> {
        a .< b.simd
    }
}

/// The only place the engine touches `Builtin` vector arithmetic.
@inline(__always) func wideAdd(_ a: FloatW, _ b: FloatW) -> FloatW {
    FloatW(storage: Builtin.fadd_Vec4xFPIEEE32(a.storage, b.storage))
}

@inline(__always) func wideSub(_ a: FloatW, _ b: FloatW) -> FloatW {
    FloatW(storage: Builtin.fsub_Vec4xFPIEEE32(a.storage, b.storage))
}

@inline(__always) func wideMul(_ a: FloatW, _ b: FloatW) -> FloatW {
    FloatW(storage: Builtin.fmul_Vec4xFPIEEE32(a.storage, b.storage))
}

@inline(__always) func wideDiv(_ a: FloatW, _ b: FloatW) -> FloatW {
    FloatW(storage: Builtin.fdiv_Vec4xFPIEEE32(a.storage, b.storage))
}

@inline(__always) func wideNeg(_ a: FloatW) -> FloatW {
    FloatW(storage: Builtin.fneg_Vec4xFPIEEE32(a.storage))
}

/// Transpose four rows into four lane columns with NEON zips. Building the columns from scalar
/// lanes makes LLVM split the downstream SIMD math into partial vectors.
@inline(__always) func transpose4(
    _ a: FloatW, _ b: FloatW, _ c: FloatW, _ d: FloatW
) -> (FloatW, FloatW, FloatW, FloatW) {
    #if canImport(simd)
    let t = simd_transpose(simd_float4x4(a.simd, b.simd, c.simd, d.simd))
    return (FloatW(t.columns.0), FloatW(t.columns.1), FloatW(t.columns.2), FloatW(t.columns.3))
    #else
    return portableTranspose4(a, b, c, d)
    #endif
}

/// Lane-only transpose for platforms without Apple's simd matrix module.
@inline(__always) func portableTranspose4(
    _ a: FloatW, _ b: FloatW, _ c: FloatW, _ d: FloatW
) -> (FloatW, FloatW, FloatW, FloatW) {
    (FloatW(a.x, b.x, c.x, d.x), FloatW(a.y, b.y, c.y, d.y),
     FloatW(a.z, b.z, c.z, d.z), FloatW(a.w, b.w, c.w, d.w))
}

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
    var v: Vec3W
    var w: Vec3W
    var dp: Vec3W
    var dq: QuatW

    @inline(__always)
    init() {
        v = Vec3W()
        w = Vec3W()
        dp = Vec3W()
        dq = QuatW()
    }

    /// Build a gathered state directly. The default initializer can be outlined into a call
    /// that clears all thirteen vectors before the gather overwrites every one of them.
    @inline(__always)
    init(v: Vec3W, w: Vec3W, dp: Vec3W, dq: QuatW) {
        self.v = v
        self.w = w
        self.dp = dp
        self.dq = dq
    }
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

/// Selection preserves the scalar operand order, including signed zeros and NaNs. A bitwise blend
/// (NEON `bsl`): the generic `replacing(with:where:)` is lowered lane by lane.
@inline(__always) func selectW(_ mask: SIMDMask<SIMD4<Int32>>, _ a: FloatW, _ b: FloatW) -> FloatW {
    let m = unsafeBitCast(mask, to: SIMD4<UInt32>.self)
    let bitsA = unsafeBitCast(a.simd, to: SIMD4<UInt32>.self)
    let bitsB = unsafeBitCast(b.simd, to: SIMD4<UInt32>.self)
    return FloatW(unsafeBitCast((bitsA & m) | (bitsB & ~m), to: SIMD4<Float>.self))
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
@inline(__always) func negW(_ a: FloatW) -> FloatW { wideNeg(a) }
