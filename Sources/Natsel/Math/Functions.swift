// Scalar helpers keep the C operand order. Swift's `min`/`max` return the other
// operand on ties, which changes signed zeros and NaN propagation.

/// The minimum of two integers.
@inlinable public func minInt(_ a: Int32, _ b: Int32) -> Int32 { a < b ? a : b }

/// The maximum of two integers.
@inlinable public func maxInt(_ a: Int32, _ b: Int32) -> Int32 { a > b ? a : b }

/// An integer clamped between a lower and upper bound.
@inlinable public func clampInt(_ a: Int32, _ lower: Int32, _ upper: Int32) -> Int32 {
    a < lower ? lower : (upper < a ? upper : a)
}

/// The absolute value of a float.
@inlinable public func absFloat(_ a: Float) -> Float { a < 0 ? -a : a }

/// The minimum of two floats.
@inlinable public func minFloat(_ a: Float, _ b: Float) -> Float { a < b ? a : b }

/// The maximum of two floats.
@inlinable public func maxFloat(_ a: Float, _ b: Float) -> Float { a > b ? a : b }

/// A float clamped between a lower and upper bound.
@inlinable public func clampFloat(_ a: Float, _ lower: Float, _ upper: Float) -> Float {
    a < lower ? lower : (upper < a ? upper : a)
}

/// Interpolate a scalar.
@inlinable public func lerpFloat(_ a: Float, _ b: Float, _ alpha: Float) -> Float {
    (1 - alpha) * a + alpha * b
}

/// Compute an approximate arctangent in the range [-pi, pi].
/// This is hand coded for cross-platform determinism. Accurate to around 0.0023 degrees.
public func atan2Approx(_ y: Float, _ x: Float) -> Float {
    // Added check for (0,0) to match atan2f and avoid NaN
    if x == 0 && y == 0 {
        return 0
    }

    let ax = absFloat(x)
    let ay = absFloat(y)
    let mx = maxFloat(ay, ax)
    let mn = minFloat(ay, ax)
    let a = mn / mx

    // Minimax polynomial approximation to atan(a) on [0,1]
    let s = a * a
    let c = s * a
    let q = s * s
    var r: Float = 0.024840285 * q + 0.18681418
    let t: Float = -0.094097948 * q - 0.33213072
    r = r * s + t
    r = r * c + a

    // Map to full circle
    if ay > ax {
        r = 1.57079637 - r
    }

    if x < 0 {
        r = 3.14159274 - r
    }

    if y < 0 {
        r = -r
    }

    return r
}

/// Compute the cosine and sine of an angle in radians. Implemented for cross-platform determinism.
/// https://en.wikipedia.org/wiki/Bh%C4%81skara_I%27s_sine_approximation_formula
public func computeCosSin(_ radians: Float) -> CosSin {
    let x = unwindAngle(radians)
    let pi2 = B3.pi * B3.pi

    // cosine needs angle in [-pi/2, pi/2]
    let c: Float
    if x < -0.5 * B3.pi {
        let y = x + B3.pi
        let y2 = y * y
        c = -(pi2 - 4 * y2) / (pi2 + y2)
    } else if x > 0.5 * B3.pi {
        let y = x - B3.pi
        let y2 = y * y
        c = -(pi2 - 4 * y2) / (pi2 + y2)
    } else {
        let y2 = x * x
        c = (pi2 - 4 * y2) / (pi2 + y2)
    }

    // sine needs angle in [0, pi]
    let s: Float
    if x < 0 {
        let y = x + B3.pi
        s = -16 * y * (B3.pi - y) / (5 * pi2 - 4 * y * (B3.pi - y))
    } else {
        s = 16 * x * (B3.pi - x) / (5 * pi2 - 4 * x * (B3.pi - x))
    }

    let mag = (s * s + c * c).squareRoot()
    let invMag: Float = mag > 0 ? 1 / mag : 0
    return CosSin(cosine: c * invMag, sine: s * invMag)
}

/// Convert any angle into the range [-pi, pi].
@inlinable public func unwindAngle(_ radians: Float) -> Float {
    // IEEE remainder, the same operation as C remainderf
    radians.remainder(dividingBy: 2 * B3.pi)
}

// MARK: - Vector

@inlinable public func + (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x + b.x, a.y + b.y, a.z + b.z) }
@inlinable public func - (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x - b.x, a.y - b.y, a.z - b.z) }
/// Vector component-wise multiplication.
@inlinable public func * (a: Vec3, b: Vec3) -> Vec3 { Vec3(a.x * b.x, a.y * b.y, a.z * b.z) }
@inlinable public prefix func - (a: Vec3) -> Vec3 { Vec3(-a.x, -a.y, -a.z) }
@inlinable public func * (s: Float, a: Vec3) -> Vec3 { Vec3(s * a.x, s * a.y, s * a.z) }
@inlinable public func * (a: Vec3, s: Float) -> Vec3 { Vec3(s * a.x, s * a.y, s * a.z) }
@inlinable public func += (a: inout Vec3, b: Vec3) { a = a + b }
@inlinable public func -= (a: inout Vec3, b: Vec3) { a = a - b }
@inlinable public func *= (a: inout Vec3, s: Float) { a = s * a }

/// Vector dot product.
@inlinable public func dot(_ a: Vec3, _ b: Vec3) -> Float { a.x * b.x + a.y * b.y + a.z * b.z }

/// Vector length.
@inlinable public func length(_ v: Vec3) -> Float { dot(v, v).squareRoot() }

/// Vector length squared.
@inlinable public func lengthSquared(_ a: Vec3) -> Float { a.x * a.x + a.y * a.y + a.z * a.z }

/// Distance between two points.
@inlinable public func distance(_ a: Vec3, _ b: Vec3) -> Float {
    length(Vec3(b.x - a.x, b.y - a.y, b.z - a.z))
}

/// Squared distance between two points.
@inlinable public func distanceSquared(_ a: Vec3, _ b: Vec3) -> Float {
    let dv = Vec3(b.x - a.x, b.y - a.y, b.z - a.z)
    return dv.x * dv.x + dv.y * dv.y + dv.z * dv.z
}

/// Normalize a vector. Returns a zero vector if the input vector is very small.
@inlinable @inline(__always) public func normalize(_ a: Vec3) -> Vec3 {
    let lengthSquared = a.x * a.x + a.y * a.y + a.z * a.z
    if lengthSquared > 1000 * Float.leastNormalMagnitude {
        let s = 1 / lengthSquared.squareRoot()
        return Vec3(s * a.x, s * a.y, s * a.z)
    }
    return Vec3(0, 0, 0)
}

/// Normalize a vector and return the length. Returns a zero vector if the input is very small.
@inlinable public func getLengthAndNormalize(_ length: inout Float, _ a: Vec3) -> Vec3 {
    let lengthSquared = a.x * a.x + a.y * a.y + a.z * a.z
    if lengthSquared > 1000 * Float.leastNormalMagnitude {
        length = lengthSquared.squareRoot()
        let s = 1 / length
        return Vec3(s * a.x, s * a.y, s * a.z)
    }

    length = 0
    return Vec3(0, 0, 0)
}

/// Get a unit vector that is perpendicular to the supplied vector.
@inlinable public func perp(_ a: Vec3) -> Vec3 {
    // Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
    // Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
    // of a unit vector must be greater or equal to 0.57735.
    let p: Vec3
    if a.x < -0.5 || 0.5 < a.x {
        p = Vec3(a.y, -a.x, 0)
    } else {
        p = Vec3(0, a.z, -a.y)
    }

    return normalize(p)
}

/// Is a vector normalized? In other words, does it have unit length?
@inlinable public func isNormalized(_ a: Vec3) -> Bool {
    let aa = dot(a, a)
    return absFloat(1 - aa) < 100 * Float.ulpOfOne
}

/// a + s * b
@inlinable public func mulAdd(_ a: Vec3, _ s: Float, _ b: Vec3) -> Vec3 {
    Vec3(a.x + s * b.x, a.y + s * b.y, a.z + s * b.z)
}

/// a - s * b
@inlinable public func mulSub(_ a: Vec3, _ s: Float, _ b: Vec3) -> Vec3 {
    Vec3(a.x - s * b.x, a.y - s * b.y, a.z - s * b.z)
}

/// https://en.wikipedia.org/wiki/Cross_product
@inlinable public func cross(_ a: Vec3, _ b: Vec3) -> Vec3 {
    Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x)
}

/// Linearly interpolate between two vectors.
@inlinable public func lerp(_ a: Vec3, _ b: Vec3, _ alpha: Float) -> Vec3 {
    b3Assert(0 <= alpha && alpha <= 1)
    return Vec3(
        (1 - alpha) * a.x + alpha * b.x,
        (1 - alpha) * a.y + alpha * b.y,
        (1 - alpha) * a.z + alpha * b.z
    )
}

/// Blend two vectors: s * a + t * b
@inlinable public func blend2(_ s: Float, _ a: Vec3, _ t: Float, _ b: Vec3) -> Vec3 {
    Vec3(s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z)
}

/// Component-wise absolute value.
@inlinable public func absVec3(_ a: Vec3) -> Vec3 { Vec3(absFloat(a.x), absFloat(a.y), absFloat(a.z)) }

/// Component-wise -1 or 1 (1 if zero).
@inlinable public func signVec3(_ a: Vec3) -> Vec3 {
    Vec3(a.x >= 0 ? 1 : -1, a.y >= 0 ? 1 : -1, a.z >= 0 ? 1 : -1)
}

/// Component-wise minimum value.
@inlinable public func minVec3(_ a: Vec3, _ b: Vec3) -> Vec3 {
    Vec3(minFloat(a.x, b.x), minFloat(a.y, b.y), minFloat(a.z, b.z))
}

/// Component-wise maximum value.
@inlinable public func maxVec3(_ a: Vec3, _ b: Vec3) -> Vec3 {
    Vec3(maxFloat(a.x, b.x), maxFloat(a.y, b.y), maxFloat(a.z, b.z))
}

/// Component-wise clamped value.
@inlinable public func clampVec3(_ a: Vec3, _ lower: Vec3, _ upper: Vec3) -> Vec3 {
    Vec3(clampFloat(a.x, lower.x, upper.x), clampFloat(a.y, lower.y, upper.y), clampFloat(a.z, lower.z, upper.z))
}

/// Create a safe scaling value for scaling collision. This allows negative scale, but keeps
/// scale sufficiently far from zero.
@inlinable public func safeScale(_ a: Vec3) -> Vec3 {
    let absScale = absVec3(a)
    let minScale = Vec3(B3.minScale, B3.minScale, B3.minScale)
    return signVec3(a) * maxVec3(absScale, minScale)
}

// MARK: - Quaternion

/// Does the supplied quaternion have unit length?
@inlinable public func isNormalizedQuat(_ q: Quat) -> Bool {
    let qq = q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z + q.s * q.s
    return 1 - 20 * Float.ulpOfOne < qq && qq < 1 + 20 * Float.ulpOfOne
}

/// Rotate a vector.
@inlinable public func rotateVector(_ q: Quat, _ v: Vec3) -> Vec3 {
    // v + 2 * cross(q.v, cross(q.v, v) + q.s * v)
    let t1 = cross(q.v, v)
    let t2 = mulAdd(t1, q.s, v)
    let t3 = cross(q.v, t2)
    return mulAdd(v, 2, t3)
}

/// Inverse rotate a vector.
@inlinable public func invRotateVector(_ q: Quat, _ v: Vec3) -> Vec3 {
    // v + 2 * cross(q.v, cross(q.v, v) - q.s * v)
    let t1 = cross(q.v, v)
    let t2 = mulSub(t1, q.s, v)
    let t3 = cross(q.v, t2)
    return mulAdd(v, 2, t3)
}

/// Compute dot product of two quaternions. Useful for polarity tests.
@inlinable public func dotQuat(_ a: Quat, _ b: Quat) -> Float {
    a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s
}

/// Multiply two quaternions.
@inlinable public func mulQuat(_ q1: Quat, _ q2: Quat) -> Quat {
    let t1 = cross(q1.v, q2.v)
    let t2 = mulAdd(t1, q1.s, q2.v)
    let t3 = mulAdd(t2, q2.s, q1.v)
    return Quat(t3, q1.s * q2.s - dot(q1.v, q2.v))
}

/// Compute a relative quaternion: inv(q1) * q2
@inlinable public func invMulQuat(_ q1: Quat, _ q2: Quat) -> Quat {
    let t1 = cross(q2.v, q1.v)
    let t2 = mulAdd(t1, q1.s, q2.v)
    let t3 = mulSub(t2, q2.s, q1.v)
    return Quat(t3, q1.s * q2.s + dot(q1.v, q2.v))
}

/// Quaternion conjugate (cheap inverse).
@inlinable public func conjugate(_ q: Quat) -> Quat { Quat(Vec3(-q.v.x, -q.v.y, -q.v.z), q.s) }

/// Component-wise quaternion negation.
@inlinable public func negateQuat(_ q: Quat) -> Quat { Quat(Vec3(-q.v.x, -q.v.y, -q.v.z), -q.s) }

/// Normalize a quaternion.
@inlinable public func normalizeQuat(_ q: Quat) -> Quat {
    let lengthSq = dotQuat(q, q)
    if lengthSq > 1000 * Float.leastNormalMagnitude {
        let s = 1 / lengthSq.squareRoot()
        return Quat(Vec3(s * q.v.x, s * q.v.y, s * q.v.z), s * q.s)
    }

    return Quat.identity
}

/// Make a quaternion that is equivalent to rotating around an axis by a specified angle.
@inlinable public func makeQuatFromAxisAngle(_ axis: Vec3, _ radians: Float) -> Quat {
    b3Assert(isNormalized(axis))
    let cs = computeCosSin(0.5 * radians)
    return Quat(Vec3(cs.sine * axis.x, cs.sine * axis.y, cs.sine * axis.z), cs.cosine)
}

/// Get the axis and angle from a quaternion. Assumes the quaternion is normalized.
@inlinable public func getAxisAngle(_ radians: inout Float, _ q: Quat) -> Vec3 {
    let length = (q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z).squareRoot()
    radians = 2 * atan2Approx(length, q.s)
    if length > 0 {
        let invLength = 1 / length
        return Vec3(invLength * q.v.x, invLength * q.v.y, invLength * q.v.z)
    }

    return Vec3.zero
}

/// Get the angle for a quaternion in radians.
@inlinable public func getQuatAngle(_ q: Quat) -> Float {
    let length = (q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z).squareRoot()
    return 2 * atan2Approx(length, q.s)
}

/// Twist angle around the z-axis, used for twist limit and revolute angle limit.
@inlinable public func getTwistAngle(_ q: Quat) -> Float {
    // Account for polarity to keep the twist angle in range.
    var twist = q.s < 0 ? atan2Approx(-q.v.z, -q.s) : atan2Approx(q.v.z, q.s)
    twist *= 2
    b3Assert(-B3.pi <= twist && twist <= B3.pi)
    return twist
}

/// Swing angle used for cone limit.
@inlinable @inline(__always) public func getSwingAngle(_ q: Quat) -> Float {
    // Polarity should not matter because all terms are squared.
    let x = (q.v.z * q.v.z + q.s * q.s).squareRoot()
    let y = (q.v.x * q.v.x + q.v.y * q.v.y).squareRoot()
    let swing = 2 * atan2Approx(y, x)
    b3Assert(0 <= swing && swing <= B3.pi)
    return swing
}

/// Linearly interpolate and normalize between two quaternions.
@inlinable public func nlerp(_ q1: Quat, _ q2: Quat, _ alpha: Float) -> Quat {
    var q1 = q1
    if dotQuat(q1, q2) < 0 {
        q1 = Quat(Vec3(-q1.v.x, -q1.v.y, -q1.v.z), -q1.s)
    }

    var q = Quat()
    q.v = lerp(q1.v, q2.v, alpha)
    q.s = (1 - alpha) * q1.s + alpha * q2.s

    return normalizeQuat(q)
}

// MARK: - Transform

/// Multiply two transforms. If the result is applied to a point p local to frame B,
/// the transform would first convert p to a point local to frame A, then into a point
/// in the world frame. This is useful if frame B is a child of frame A.
@inlinable public func mulTransforms(_ a: Transform, _ b: Transform) -> Transform {
    Transform(rotateVector(a.q, b.p) + a.p, mulQuat(a.q, b.q))
}

/// Creates a transform that converts a local point in frame B to a local point in frame A.
@inlinable @inline(__always) public func invMulTransforms(_ a: Transform, _ b: Transform) -> Transform {
    Transform(invRotateVector(a.q, b.p - a.p), invMulQuat(a.q, b.q))
}

/// Get the inverse of a transform.
@inlinable public func invertTransform(_ t: Transform) -> Transform {
    Transform(invRotateVector(t.q, -t.p), conjugate(t.q))
}

/// Transform a point.
@inlinable @inline(__always) public func transformPoint(_ t: Transform, _ v: Vec3) -> Vec3 {
    rotateVector(t.q, v) + t.p
}

/// Inverse transform a point.
@inlinable public func invTransformPoint(_ t: Transform, _ v: Vec3) -> Vec3 {
    invRotateVector(t.q, v - t.p)
}

// MARK: - World position boundary (single precision)

/// Convert a vector to a world position.
@inlinable public func toPos(_ v: Vec3) -> Pos { v }

/// Lossy conversion of a world position to a float vector.
@inlinable public func toVec3(_ p: Pos) -> Vec3 { p }

/// Narrow a world coordinate to float, rounding toward negative infinity.
@inlinable public func roundDownFloat(_ x: Float) -> Float { x }

/// Narrow a world coordinate to float, rounding toward positive infinity.
@inlinable public func roundUpFloat(_ x: Float) -> Float { x }

/// a - b, demoted to float. The primary precision boundary operation.
@inlinable public func subPos(_ a: Pos, _ b: Pos) -> Vec3 { Vec3(a.x - b.x, a.y - b.y, a.z - b.z) }

/// p + d
@inlinable public func offsetPos(_ p: Pos, _ d: Vec3) -> Pos { Pos(p.x + d.x, p.y + d.y, p.z + d.z) }

/// World position interpolation for sweeps and sampling.
@inlinable public func lerpPosition(_ a: Pos, _ b: Pos, _ t: Float) -> Pos {
    Pos((1 - t) * a.x + t * b.x, (1 - t) * a.y + t * b.y, (1 - t) * a.z + t * b.z)
}

/// Transform a local point to a world position.
@inlinable public func transformWorldPoint(_ t: WorldTransform, _ p: Vec3) -> Pos {
    let r = rotateVector(t.q, p)
    return Pos(t.p.x + r.x, t.p.y + r.y, t.p.z + r.z)
}

/// Transform a world position to a local point.
@inlinable public func invTransformWorldPoint(_ t: WorldTransform, _ p: Pos) -> Vec3 {
    let d = Vec3(p.x - t.p.x, p.y - t.p.y, p.z - t.p.z)
    return invRotateVector(t.q, d)
}

/// Compose a world transform with a local transform.
@inlinable @inline(__always) public func mulWorldTransforms(_ a: WorldTransform, _ b: Transform) -> WorldTransform {
    var c = WorldTransform()
    c.q = mulQuat(a.q, b.q)
    let r = rotateVector(a.q, b.p)
    c.p = Pos(a.p.x + r.x, a.p.y + r.y, a.p.z + r.z)
    return c
}

/// Relative transform of frame B in frame A. The narrow phase boundary.
@inlinable @inline(__always) public func invMulWorldTransforms(_ a: WorldTransform, _ b: WorldTransform) -> Transform {
    var c = Transform()
    c.q = invMulQuat(a.q, b.q)
    let d = Vec3(b.p.x - a.p.x, b.p.y - a.p.y, b.p.z - a.p.z)
    c.p = invRotateVector(a.q, d)
    return c
}

/// Shift a world transform into the frame of a base position.
@inlinable public func toRelativeTransform(_ t: WorldTransform, _ base: Pos) -> Transform {
    Transform(Vec3(t.p.x - base.x, t.p.y - base.y, t.p.z - base.z), t.q)
}

/// Promote a float transform to a world transform. Lossless.
@inlinable public func makeWorldTransform(_ t: Transform) -> WorldTransform { t }

/// Translate a local AABB by a world position, rounding outward.
@inlinable public func offsetAABB(_ localBox: AABB, _ origin: Pos) -> AABB {
    var out = AABB()
    out.lowerBound.x = roundDownFloat(origin.x + localBox.lowerBound.x)
    out.lowerBound.y = roundDownFloat(origin.y + localBox.lowerBound.y)
    out.lowerBound.z = roundDownFloat(origin.z + localBox.lowerBound.z)
    out.upperBound.x = roundUpFloat(origin.x + localBox.upperBound.x)
    out.upperBound.y = roundUpFloat(origin.y + localBox.upperBound.y)
    out.upperBound.z = roundUpFloat(origin.z + localBox.upperBound.z)
    return out
}

// MARK: - Matrix

/// Compute the determinant of a 3-by-3 matrix.
@inlinable public func det(_ m: Matrix3) -> Float { dot(m.cx, cross(m.cy, m.cz)) }

/// Multiply a matrix times a column vector.
@inlinable public func mulMV(_ m: Matrix3, _ a: Vec3) -> Vec3 {
    Vec3(
        m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
        m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
        m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z
    )
}

/// Negate a matrix.
@inlinable public func negateMat3(_ a: Matrix3) -> Matrix3 { Matrix3(-a.cx, -a.cy, -a.cz) }

/// Matrix addition: a + b
@inlinable public func addMM(_ a: Matrix3, _ b: Matrix3) -> Matrix3 { Matrix3(a.cx + b.cx, a.cy + b.cy, a.cz + b.cz) }

/// Matrix subtraction: a - b
@inlinable public func subMM(_ a: Matrix3, _ b: Matrix3) -> Matrix3 { Matrix3(a.cx - b.cx, a.cy - b.cy, a.cz - b.cz) }

/// Multiply a matrix by a scalar, component-wise.
@inlinable public func mulSM(_ s: Float, _ a: Matrix3) -> Matrix3 { Matrix3(s * a.cx, s * a.cy, s * a.cz) }

/// Matrix multiplication: a * b
@inlinable public func mulMM(_ a: Matrix3, _ b: Matrix3) -> Matrix3 {
    Matrix3(mulMV(a, b.cx), mulMV(a, b.cy), mulMV(a, b.cz))
}

/// Matrix transpose.
@inlinable public func transpose(_ m: Matrix3) -> Matrix3 {
    Matrix3(Vec3(m.cx.x, m.cy.x, m.cz.x), Vec3(m.cx.y, m.cy.y, m.cz.y), Vec3(m.cx.z, m.cy.z, m.cz.z))
}

/// General matrix inverse.
@inlinable public func invertMatrix(_ m: Matrix3) -> Matrix3 {
    let d = det(m)
    if absFloat(d) > 1000 * Float.leastNormalMagnitude {
        let invDet = 1 / d
        let out = Matrix3(invDet * cross(m.cy, m.cz), invDet * cross(m.cz, m.cx), invDet * cross(m.cx, m.cy))
        return transpose(out)
    }

    return Matrix3.zero
}

/// Solve a matrix equation: inv(m) * a
@inlinable @inline(__always) public func solve3(_ m: Matrix3, _ a: Vec3) -> Vec3 {
    let d = det(m)
    if absFloat(d) > 1000 * Float.leastNormalMagnitude {
        let invDet = 1 / d
        let sx = cross(m.cy, m.cz)
        let sy = cross(m.cz, m.cx)
        let sz = cross(m.cx, m.cy)
        return Vec3(invDet * dot(sx, a), invDet * dot(sy, a), invDet * dot(sz, a))
    }

    return Vec3.zero
}

/// Invert a matrix and return the transpose of the inverse.
@inlinable public func invertT(_ m: Matrix3) -> Matrix3 {
    let d = det(m)
    if absFloat(d) > 1000 * Float.leastNormalMagnitude {
        let invDet = 1 / d
        return Matrix3(invDet * cross(m.cy, m.cz), invDet * cross(m.cz, m.cx), invDet * cross(m.cx, m.cy))
    }

    return Matrix3.zero
}

/// Get the component-wise absolute value of a matrix.
@inlinable public func absMatrix3(_ m: Matrix3) -> Matrix3 { Matrix3(absVec3(m.cx), absVec3(m.cy), absVec3(m.cz)) }

/// Make a matrix from a quaternion. This is useful if you need to rotate many vectors.
@inlinable @inline(__always) public func makeMatrixFromQuat(_ q: Quat) -> Matrix3 {
    let xx = q.v.x * q.v.x
    let yy = q.v.y * q.v.y
    let zz = q.v.z * q.v.z
    let xy = q.v.x * q.v.y
    let xz = q.v.x * q.v.z
    let xw = q.v.x * q.s
    let yz = q.v.y * q.v.z
    let yw = q.v.y * q.s
    let zw = q.v.z * q.s

    return Matrix3(
        Vec3(1 - 2 * (yy + zz), 2 * (xy + zw), 2 * (xz - yw)),
        Vec3(2 * (xy - zw), 1 - 2 * (xx + zz), 2 * (yz + xw)),
        Vec3(2 * (xz + yw), 2 * (yz - xw), 1 - 2 * (xx + yy))
    )
}

// MARK: - AABB

/// Get the AABB of a point cloud.
@inlinable public func makeAABB(_ points: UnsafePointer<Vec3>, _ count: Int32, _ radius: Float) -> AABB {
    b3Assert(count > 0)
    var a = AABB(points[0], points[0])
    var i: Int32 = 1
    while i < count {
        a.lowerBound = minVec3(a.lowerBound, points[Int(i)])
        a.upperBound = maxVec3(a.upperBound, points[Int(i)])
        i += 1
    }

    let r = Vec3(radius, radius, radius)
    a.lowerBound = a.lowerBound - r
    a.upperBound = a.upperBound + r

    return a
}

/// Does a fully contain b?
@inlinable public func aabbContains(_ a: AABB, _ b: AABB) -> Bool {
    if a.lowerBound.x > b.lowerBound.x || b.upperBound.x > a.upperBound.x { return false }
    if a.lowerBound.y > b.lowerBound.y || b.upperBound.y > a.upperBound.y { return false }
    if a.lowerBound.z > b.lowerBound.z || b.upperBound.z > a.upperBound.z { return false }
    return true
}

/// Get the surface area of an axis-aligned bounding box.
@inlinable public func aabbArea(_ a: AABB) -> Float {
    let delta = a.upperBound - a.lowerBound
    return 2 * (delta.x * delta.y + delta.y * delta.z + delta.z * delta.x)
}

/// Get the center of an axis-aligned bounding box.
@inlinable public func aabbCenter(_ a: AABB) -> Vec3 { 0.5 * (a.upperBound + a.lowerBound) }

/// Get the extents (half-widths) of an axis-aligned bounding box.
@inlinable public func aabbExtents(_ a: AABB) -> Vec3 { 0.5 * (a.upperBound - a.lowerBound) }

/// Get the union of two axis-aligned bounding boxes.
@inlinable public func aabbUnion(_ a: AABB, _ b: AABB) -> AABB {
    AABB(minVec3(a.lowerBound, b.lowerBound), maxVec3(a.upperBound, b.upperBound))
}

/// Add uniform padding to an axis-aligned bounding box.
@inlinable public func aabbInflate(_ a: AABB, _ extension: Float) -> AABB {
    let radius = Vec3(`extension`, `extension`, `extension`)
    return AABB(a.lowerBound - radius, a.upperBound + radius)
}

/// Do two axis-aligned boxes overlap?
@inlinable public func aabbOverlaps(_ a: AABB, _ b: AABB) -> Bool {
    // No intersection if separated along one axis
    if a.upperBound.x < b.lowerBound.x || a.lowerBound.x > b.upperBound.x { return false }
    if a.upperBound.y < b.lowerBound.y || a.lowerBound.y > b.upperBound.y { return false }
    if a.upperBound.z < b.lowerBound.z || a.lowerBound.z > b.upperBound.z { return false }

    // Overlapping on all axis means bounds are intersecting
    return true
}

/// Transform an axis-aligned bounding box. This can create a larger box than if you
/// recomputed the AABB of the original shape with the transform applied.
@inlinable @inline(__always) public func aabbTransform(_ transform: Transform, _ a: AABB) -> AABB {
    let center = transformPoint(transform, aabbCenter(a))
    let m = makeMatrixFromQuat(transform.q)
    let extent = mulMV(absMatrix3(m), aabbExtents(a))
    return AABB(center - extent, center + extent)
}

/// Get the closest point on an axis-aligned bounding box.
@inlinable public func closestPointToAABB(_ point: Vec3, _ a: AABB) -> Vec3 {
    clampVec3(point, a.lowerBound, a.upperBound)
}

// MARK: - Validation and geometry (math_functions.c)

/// Is this a valid number? Not NaN or infinity.
@inlinable public func isValidFloat(_ a: Float) -> Bool { a.isFinite }

/// Is this a valid vector? Not NaN or infinity.
@inlinable public func isValidVec3(_ a: Vec3) -> Bool { a.x.isFinite && a.y.isFinite && a.z.isFinite }

/// Is this a valid quaternion? Not NaN or infinity. Is normalized.
public func isValidQuat(_ a: Quat) -> Bool {
    if !(a.v.x.isFinite && a.v.y.isFinite && a.v.z.isFinite && a.s.isFinite) {
        return false
    }

    return isNormalizedQuat(a)
}

/// Is this a valid transform? Not NaN or infinity. Is normalized.
public func isValidTransform(_ a: Transform) -> Bool { isValidVec3(a.p) && isValidQuat(a.q) }

/// Is this a valid matrix? Not NaN or infinity.
public func isValidMatrix3(_ a: Matrix3) -> Bool { isValidVec3(a.cx) && isValidVec3(a.cy) && isValidVec3(a.cz) }

/// Is this a valid bounding box? Not NaN or infinity. Upper bound greater than or equal to lower bound.
public func isValidAABB(_ a: AABB) -> Bool {
    if isValidVec3(a.lowerBound) == false { return false }
    if isValidVec3(a.upperBound) == false { return false }
    if a.lowerBound.x > a.upperBound.x { return false }
    if a.lowerBound.y > a.upperBound.y { return false }
    if a.lowerBound.z > a.upperBound.z { return false }
    return true
}

/// Is this AABB reasonably close to the origin? See `B3.huge`.
public func isBoundedAABB(_ a: AABB) -> Bool {
    let huge = B3.huge
    if a.lowerBound.x < -huge || a.lowerBound.y < -huge || a.lowerBound.z < -huge { return false }
    if a.upperBound.x > huge || a.upperBound.y > huge || a.upperBound.z > huge { return false }
    return true
}

/// Is this AABB valid and reasonable?
public func isSaneAABB(_ a: AABB) -> Bool {
    if isValidAABB(a) == false { return false }
    return isBoundedAABB(a)
}

/// Is this a valid plane? Normal is a unit vector. Not NaN or infinity.
public func isValidPlane(_ a: Plane) -> Bool {
    if isValidVec3(a.normal) == false { return false }
    if isNormalized(a.normal) == false { return false }
    return isValidFloat(a.offset)
}

/// Is this a valid world position? Not NaN or infinity.
public func isValidPosition(_ p: Pos) -> Bool { isValidVec3(p) }

/// Is this a valid world transform? Not NaN or infinity. Rotation is normalized.
public func isValidWorldTransform(_ t: WorldTransform) -> Bool { isValidPosition(t.p) && isValidQuat(t.q) }

/// Extract a quaternion from a rotation matrix.
public func makeQuatFromMatrix(_ m: Matrix3) -> Quat {
    let c1 = m.cx
    let c2 = m.cy
    let c3 = m.cz

    var q = Quat()

    let trace = m.cx.x + m.cy.y + m.cz.z
    if trace >= 0 {
        q.v.x = c2.z - c3.y
        q.v.y = c3.x - c1.z
        q.v.z = c1.y - c2.x
        q.s = trace + 1
    } else {
        if c1.x > c2.y && c1.x > c3.z {
            q.v.x = c1.x - c2.y - c3.z + 1
            q.v.y = c2.x + c1.y
            q.v.z = c3.x + c1.z
            q.s = c2.z - c3.y
        } else if c2.y > c3.z {
            q.v.x = c1.y + c2.x
            q.v.y = c2.y - c3.z - c1.x + 1
            q.v.z = c3.y + c2.z
            q.s = c3.x - c1.z
        } else {
            q.v.x = c1.z + c3.x
            q.v.y = c2.z + c3.y
            q.v.z = c3.z - c1.x - c2.y + 1
            q.s = c1.y - c2.x
        }
    }

    // The algorithm is simplified and made more accurate by normalizing at the end
    return normalizeQuat(q)
}

/// Find a quaternion that rotates one vector to another.
public func computeQuatBetweenUnitVectors(_ v1: Vec3, _ v2: Vec3) -> Quat {
    b3Assert(isNormalized(v1))
    b3Assert(isNormalized(v2))

    var out = Quat()

    let m = lerp(v1, v2, 0.5)
    let tolerance = 100 * Float.ulpOfOne
    if lengthSquared(m) > tolerance * tolerance {
        out.v = cross(v1, m)
        out.s = dot(v1, m)
    } else {
        // Anti-parallel: Use a perpendicular vector
        if absFloat(v1.x) > 0.5 {
            out.v.x = v1.y
            out.v.y = -v1.x
            out.v.z = 0
        } else {
            out.v.x = 0
            out.v.y = v1.z
            out.v.z = -v1.y
        }

        out.s = 0
    }

    // The algorithm is simplified and made more accurate by normalizing at the end
    return normalizeQuat(out)
}

/// Compute the closest points on two infinite lines.
public func lineDistance(_ p1: Vec3, _ d1: Vec3, _ p2: Vec3, _ d2: Vec3) -> SegmentDistanceResult {
    var result = SegmentDistanceResult()

    // Solve A*x = b
    let a11 = dot(d1, d1)
    let a12 = -dot(d1, d2)
    let a21 = dot(d2, d1)
    let a22 = -dot(d2, d2)

    let w = p1 - p2
    let b1 = -dot(d1, w)
    let b2 = -dot(d2, w)

    let d = a11 * a22 - a12 * a21
    if d * d < 1000 * Float.leastNormalMagnitude {
        // Lines are parallel - project p2 onto line L1: x1 = p1 + s1 * d1
        let s1 = dot(p2 - p1, d1) / dot(d1, d1)
        let s2: Float = 0

        result.point1 = mulAdd(p1, s1, d1)
        result.fraction1 = s1
        result.point2 = mulAdd(p2, s2, d2)
        result.fraction2 = s2

        return result
    }

    let s1 = (a22 * b1 - a12 * b2) / d
    let s2 = (a11 * b2 - a21 * b1) / d

    result.point1 = mulAdd(p1, s1, d1)
    result.fraction1 = s1
    result.point2 = mulAdd(p2, s2, d2)
    result.fraction2 = s2
    return result
}

/// Compute the closest points on two line segments.
public func segmentDistance(_ p1: Vec3, _ q1: Vec3, _ p2: Vec3, _ q2: Vec3) -> SegmentDistanceResult {
    var result = SegmentDistanceResult()

    let d1 = q1 - p1
    let d2 = q2 - p2
    let r = p1 - p2

    let a = dot(d1, d1)
    let b = dot(d1, d2)
    let c = dot(d1, r)
    let e = dot(d2, d2)
    let f = dot(d2, r)

    // Check if one of the segments degenerates into a point
    if a < 100 * Float.ulpOfOne && e < 100 * Float.ulpOfOne {
        // Both segments degenerate into points
        result.point1 = p1
        result.fraction1 = 0
        result.point2 = p2
        result.fraction2 = 0
        return result
    }

    if a < 100 * Float.ulpOfOne {
        // First segment degenerates into a point
        let s2 = clampFloat(f / e, 0, 1)

        result.point1 = p1
        result.fraction1 = 0
        result.point2 = mulAdd(p2, s2, d2)
        result.fraction2 = s2
        return result
    }

    if e < 100 * Float.ulpOfOne {
        // Second segment degenerates into a point
        let s1 = clampFloat(-c / a, 0, 1)

        result.point1 = mulAdd(p1, s1, d1)
        result.fraction1 = s1
        result.point2 = p2
        result.fraction2 = 0
        return result
    }

    // Non-degenerate case
    let denom = a * e - b * b
    var s1: Float = denom > 1000 * Float.leastNormalMagnitude ? clampFloat((b * f - c * e) / denom, 0, 1) : 0
    var s2 = (b * s1 + f) / e

    // Clamp lambda2 and recompute lambda1 if necessary
    if s2 < 0 {
        s1 = clampFloat(-c / a, 0, 1)
        s2 = 0
    } else if s2 > 1 {
        s1 = clampFloat((b - c) / a, 0, 1)
        s2 = 1
    }

    result.point1 = mulAdd(p1, s1, d1)
    result.fraction1 = s1
    result.point2 = mulAdd(p2, s2, d2)
    result.fraction2 = s2
    return result
}

/// Compute the closest point on the segment a-b to the target q.
public func pointToSegmentDistance(_ a: Vec3, _ b: Vec3, _ q: Vec3) -> Vec3 {
    let ab = b - a
    let aq = q - a

    var alpha = dot(ab, aq)

    if alpha <= 0 {
        // q projects outside interval [a, b] on the side of a
        return a
    } else {
        let denominator = dot(ab, ab)
        if alpha > denominator {
            // q projects outside interval [a, b] on the side of b
            return b
        } else {
            // q projects inside interval [a, b]
            alpha /= denominator
            return mulAdd(a, alpha, ab)
        }
    }
}

/// Get the inertia tensor of an offset point.
/// https://en.wikipedia.org/wiki/Parallel_axis_theorem
public func steiner(_ mass: Float, _ origin: Vec3) -> Matrix3 {
    // Usage: Io = Ic + Is and Ic = Io - Is
    let ixx = mass * (origin.y * origin.y + origin.z * origin.z)
    let iyy = mass * (origin.x * origin.x + origin.z * origin.z)
    let izz = mass * (origin.x * origin.x + origin.y * origin.y)
    let ixy = -mass * origin.x * origin.y
    let ixz = -mass * origin.x * origin.z
    let iyz = -mass * origin.y * origin.z

    var out = Matrix3()
    out.cx.x = ixx
    out.cy.x = ixy
    out.cz.x = ixz
    out.cx.y = ixy
    out.cy.y = iyy
    out.cz.y = iyz
    out.cx.z = ixz
    out.cy.z = iyz
    out.cz.z = izz

    return out
}

/// Is this a valid ray? Origin and translation are finite and the max fraction is in range.
public func isValidRay(_ input: RayCastInput) -> Bool {
    isValidVec3(input.origin) && isValidVec3(input.translation)
        && isValidFloat(input.maxFraction) && 0 <= input.maxFraction
        && input.maxFraction < B3.huge
}
