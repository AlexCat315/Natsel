// Internal geometry helpers from `math_internal.h`. These are the scalar versions so the
// operand order can be preserved exactly; the SIMD paths in C evaluate the same expressions.

// MARK: - Internal types

/// A 2x2 matrix stored as two column vectors.
struct Matrix2: Sendable, Equatable, BitwiseCopyable {
    var cx = Vec2()
    var cy = Vec2()

    init() {}

    init(_ cx: Vec2, _ cy: Vec2) {
        self.cx = cx
        self.cy = cy
    }
}

/// A triangle with vertex indices and edge flags.
struct Triangle: Sendable, BitwiseCopyable {
    var vertices = InlineArray<3, Vec3>(repeating: Vec3())
    var i1: Int32 = 0
    var i2: Int32 = 0
    var i3: Int32 = 0
    var flags: Int32 = 0

    init() {}
}

/// A point on a triangle along with the feature it belongs to.
struct TrianglePoint: Sendable, Equatable, BitwiseCopyable {
    var point = Vec3()
    var feature: TriangleFeature = .featureNone

    init() {}

    init(point: Vec3, feature: TriangleFeature) {
        self.point = point
        self.feature = feature
    }
}

/// Minimum extent and the axis along which it occurs.
struct ShapeExtent: Sendable, Equatable, BitwiseCopyable {
    var minExtent: Float = 0
    var maxExtent = Vec3()

    init() {}
}

/// The empty bounds sentinel used by the C `B3_BOUNDS3_EMPTY` constant.
let bounds3Empty = AABB(
    Vec3(Float.greatestFiniteMagnitude, Float.greatestFiniteMagnitude, Float.greatestFiniteMagnitude),
    Vec3(-Float.greatestFiniteMagnitude, -Float.greatestFiniteMagnitude, -Float.greatestFiniteMagnitude)
)

// MARK: - Scalar helpers

/// Align a byte count up to 8 bytes.
func alignUp8(_ x: Int) -> Int { (x + 7) & ~7 }

/// Integer ceiling division. The denominator must be positive.
func ceilingInt(_ numerator: Int32, _ denominator: Int32) -> Int32 {
    b3Validate(denominator > 0)
    return (numerator + denominator - 1) / denominator
}

/// Integer ceiling division where the denominator is a power of two.
func ceilingPow2(_ numerator: Int32, _ denominator: Int32, _ exponent: Int32) -> Int32 {
    b3Validate(exponent > 0 && denominator == 1 << exponent)
    return (numerator + denominator - 1) >> exponent
}

// MARK: - Vec2 helpers

func dot2(_ v1: Vec2, _ v2: Vec2) -> Float { v1.x * v2.x + v1.y * v2.y }

func length2(_ v: Vec2) -> Float { dot2(v, v).squareRoot() }

func lengthSquared2(_ v: Vec2) -> Float { dot2(v, v) }

func minVec2(_ v1: Vec2, _ v2: Vec2) -> Vec2 {
    Vec2(minFloat(v1.x, v2.x), minFloat(v1.y, v2.y))
}

func maxVec2(_ v1: Vec2, _ v2: Vec2) -> Vec2 {
    Vec2(maxFloat(v1.x, v2.x), maxFloat(v1.y, v2.y))
}

func add2(_ a: Vec2, _ b: Vec2) -> Vec2 { Vec2(a.x + b.x, a.y + b.y) }

func sub2(_ a: Vec2, _ b: Vec2) -> Vec2 { Vec2(a.x - b.x, a.y - b.y) }

func neg2(_ v: Vec2) -> Vec2 { Vec2(-v.x, -v.y) }

func mulSV2(_ s: Float, _ v: Vec2) -> Vec2 { Vec2(s * v.x, s * v.y) }

/// a + s * b
func mulAdd2(_ a: Vec2, _ s: Float, _ b: Vec2) -> Vec2 { Vec2(a.x + s * b.x, a.y + s * b.y) }

/// a - s * b
func mulSub2(_ a: Vec2, _ s: Float, _ b: Vec2) -> Vec2 { Vec2(a.x - s * b.x, a.y - s * b.y) }

func cross2(_ a: Vec2, _ b: Vec2) -> Float { a.x * b.y - a.y * b.x }

func distanceSquared2(_ a: Vec2, _ b: Vec2) -> Float {
    let dx = b.x - a.x
    let dy = b.y - a.y
    return dx * dx + dy * dy
}

func mulMV2(_ m: Matrix2, _ a: Vec2) -> Vec2 {
    Vec2(m.cx.x * a.x + m.cy.x * a.y, m.cx.y * a.x + m.cy.y * a.y)
}

func mulMM2(_ m1: Matrix2, _ m2: Matrix2) -> Matrix2 {
    Matrix2(mulMV2(m1, m2.cx), mulMV2(m1, m2.cy))
}

func det2(_ m: Matrix2) -> Float { m.cx.x * m.cy.y - m.cx.y * m.cy.x }

func invert2(_ m: Matrix2) -> Matrix2 {
    let det = det2(m)
    if absFloat(det) > 1000 * Float.leastNormalMagnitude {
        let invDet = 1 / det
        return Matrix2(
            Vec2(invDet * m.cy.y, -invDet * m.cx.y),
            Vec2(-invDet * m.cy.x, invDet * m.cx.x)
        )
    }

    return Matrix2(Vec2(0, 0), Vec2(0, 0))
}

/// Solve a positive semi-definite 2x2 system.
func solve2(_ m: Matrix2, _ b: Vec2) -> Vec2 {
    let det = det2(m)
    if det > 1000 * Float.leastNormalMagnitude {
        let invDet = 1 / det
        return Vec2(
            invDet * m.cy.y * b.x - invDet * m.cy.x * b.y,
            -invDet * m.cx.y * b.x + invDet * m.cx.x * b.y
        )
    }

    return Vec2(0, 0)
}

// MARK: - Vec3 helpers

/// Get a component by index without relying on `(&v.x)[index]` behavior.
func getByIndex(_ v: Vec3, _ index: Int) -> Float {
    b3Validate(0 <= index && index < 3)
    switch index {
    case 0: return v.x
    case 1: return v.y
    default: return v.z
    }
}

/// The index of the largest component.
func majorAxis(_ v: Vec3) -> Int32 {
    v.x < v.y ? (v.y < v.z ? 2 : 1) : (v.x < v.z ? 2 : 0)
}

func minElement(_ v: Vec3) -> Float { minFloat(v.x, minFloat(v.y, v.z)) }

func maxElement(_ v: Vec3) -> Float { maxFloat(v.x, maxFloat(v.y, v.z)) }

func maxElementIndex(_ v: Vec3) -> Int32 { majorAxis(v) }

/// Convenience function: s * a + t * b + u * c
func blend3(_ s: Float, _ a: Vec3, _ t: Float, _ b: Vec3, _ u: Float, _ c: Vec3) -> Vec3 {
    Vec3(
        s * a.x + t * b.x + u * c.x,
        s * a.y + t * b.y + u * c.y,
        s * a.z + t * b.z + u * c.z
    )
}

/// Cross product that adds equal components instead of subtracting them.
func modifiedCross(_ a: Vec3, _ b: Vec3) -> Vec3 {
    Vec3(
        a.y * b.z + a.z * b.y,
        a.z * b.x + a.x * b.z,
        a.x * b.y + a.y * b.x
    )
}

func makeDiagonalMatrix(_ a: Float, _ b: Float, _ c: Float) -> Matrix3 {
    Matrix3(Vec3(a, 0, 0), Vec3(0, b, 0), Vec3(0, 0, c))
}

/// Clamp the length of a vector while keeping its direction.
func clampLength(_ v: Vec3, _ maxLength: Float) -> Vec3 {
    let lengthSq = lengthSquared(v)
    if lengthSq <= maxLength * maxLength {
        return v
    }

    let length = lengthSq.squareRoot()
    return (maxLength / length) * v
}

/// Compute a unit vector perpendicular to the given unit vector.
func arbitraryPerp(_ v: Vec3) -> Vec3 {
    // Suppose vector a has all equal components and is a unit vector: a = (s, s, s)
    // Then 3*s*s = 1, s = sqrt(1/3) = 0.57735. This means that at least one component
    // of a unit vector must be greater or equal to 0.57735.
    let p: Vec3
    if v.x < -0.5 || 0.5 < v.x {
        // x is non-zero and it should not go into the x component
        let a: Float = 0.67
        let b: Float = -0.42
        p = Vec3(a * v.y + b * v.z, -a * v.x, -b * v.x)
    } else if v.y < -0.5 || 0.5 < v.y {
        // y is non-zero and it should not go into the y component
        let a: Float = 0.67
        let c: Float = -0.42
        p = Vec3(a * v.y, -a * v.x + c * v.z, -c * v.y)
    } else {
        // This would trip if the input is not a unit vector
        b3Validate(v.z < -0.5 || 0.5 < v.z)

        // z is non-zero and it should not go into the z component
        let a: Float = 0.67
        let b: Float = -0.42
        p = Vec3(a * v.z, b * v.z, -a * v.x - b * v.y)
    }

    b3Validate(lengthSquared(p) > 0.1)
    b3Validate(absFloat(dot(p, v)) < 100 * Float.ulpOfOne)

    return normalize(p)
}

func scalarTripleProduct(_ a: Vec3, _ b: Vec3, _ c: Vec3) -> Float {
    let d = Vec3(
        b.y * c.z - b.z * c.y,
        b.z * c.x - b.x * c.z,
        b.x * c.y - b.y * c.x
    )
    return a.x * d.x + a.y * d.y + a.z * d.z
}

// MARK: - Rotation helpers

/// Exponential map (Grassia).
func quatFromExponentialMap(_ v: Vec3) -> Quat {
    let threshold: Float = 0.018581361

    let angle = length(v)
    if angle < threshold {
        // Taylor expansion
        var out = Quat()
        out.v = (0.5 + angle * angle / 48.0) * v
        out.s = computeCosSin(0.5 * angle).cosine

        return out
    }

    return makeQuatFromAxisAngle((1.0 / angle) * v, angle)
}

/// Integrate rotation from angular velocity.
/// q2 = q1 + 0.5 * omega * q1
func integrateRotation(_ q1: Quat, _ deltaRotation: Vec3) -> Quat {
    // https://fgiesen.wordpress.com/2012/08/24/quaternion-differentiation/
    var qd = Quat(0.5 * deltaRotation, 0)
    qd = mulQuat(qd, q1)
    var q2 = Quat(q1.v + qd.v, qd.s + q1.s)
    q2 = normalizeQuat(q2)
    return q2
}

/// Pseudo angular velocity from a quaternion target.
/// w = 2 * (target - q) * conj(q)
func deltaQuatToRotation(_ q: Quat, _ target: Quat) -> Vec3 {
    var s = q
    if dotQuat(q, target) < 0 {
        // Correct polarity
        s = negateQuat(q)
    }

    let diff = Quat(target.v - s.v, target.s - s.s)
    let product = mulQuat(diff, conjugate(s))
    return 2.0 * product.v
}

// MARK: - Plane helpers

func normalizePlane(_ plane: Plane) -> Plane {
    let invLength = 1 / length(plane.normal)
    return Plane(invLength * plane.normal, invLength * plane.offset)
}

func makePlaneFromNormalAndPoint(_ normal: Vec3, _ point: Vec3) -> Plane {
    Plane(normal, dot(normal, point))
}

func makePlaneFromPoints(_ point1: Vec3, _ point2: Vec3, _ point3: Vec3) -> Plane {
    var plane = Plane()
    plane.normal = cross(point2 - point1, point3 - point1)
    plane.normal = normalize(plane.normal)
    plane.offset = dot(plane.normal, point1)
    return plane
}

func makeNormalFromPoints(_ point1: Vec3, _ point2: Vec3, _ point3: Vec3) -> Vec3 {
    let normal = cross(point2 - point1, point3 - point1)
    return normalize(normal)
}

/// normal2 = q * normal1
/// offset2 = dot(normal2, p) + offset1
func transformPlane(_ transform: Transform, _ plane: Plane) -> Plane {
    let normal = rotateVector(transform.q, plane.normal)
    return Plane(normal, plane.offset + dot(normal, transform.p))
}

/// Signed separation of a point from a plane.
func planeSeparation(_ plane: Plane, _ point: Vec3) -> Float {
    dot(plane.normal, point) - plane.offset
}

/// Negative if p is below the triangle v1-v2-v3.
func signedVolume(_ v1: Vec3, _ v2: Vec3, _ v3: Vec3, _ p: Vec3) -> Float {
    let e1 = v2 - v1
    let e2 = v3 - v1
    let n = cross(e1, e2)
    return dot(n, p - v1)
}

// MARK: - Inertia helpers

func rotateInertia(_ q: Quat, _ centralInertia: Matrix3) -> Matrix3 {
    let rotationMatrix = makeMatrixFromQuat(q)
    return mulMM(rotationMatrix, mulMM(centralInertia, transpose(rotationMatrix)))
}

func transformInertia(_ transform: Transform, _ centralInertia: Matrix3, _ mass: Float) -> Matrix3 {
    var inertia = rotateInertia(transform.q, centralInertia)
    inertia = addMM(inertia, steiner(mass, transform.p))
    return inertia
}

// MARK: - AABB helpers

/// Get the surface area (perimeter) of an AABB.
func perimeter(_ a: AABB) -> Float {
    let wx = a.upperBound.x - a.lowerBound.x
    let wy = a.upperBound.y - a.lowerBound.y
    let wz = a.upperBound.z - a.lowerBound.z
    return 2 * (wx * wz + wy * wx + wz * wy)
}

/// Enlarge `a` to contain `b`. Returns true if the AABB grew.
func enlargeAABB(_ a: inout AABB, _ b: AABB) -> Bool {
    var changed = false
    if b.lowerBound.x < a.lowerBound.x {
        a.lowerBound.x = b.lowerBound.x
        changed = true
    }

    if b.lowerBound.y < a.lowerBound.y {
        a.lowerBound.y = b.lowerBound.y
        changed = true
    }

    if b.lowerBound.z < a.lowerBound.z {
        a.lowerBound.z = b.lowerBound.z
        changed = true
    }

    if a.upperBound.x < b.upperBound.x {
        a.upperBound.x = b.upperBound.x
        changed = true
    }

    if a.upperBound.y < b.upperBound.y {
        a.upperBound.y = b.upperBound.y
        changed = true
    }

    if a.upperBound.z < b.upperBound.z {
        a.upperBound.z = b.upperBound.z
        changed = true
    }

    return changed
}

/// Get the farthest point on the AABB in the direction of `p`.
func farthestPointOnAABB(_ b: AABB, _ p: Vec3) -> Vec3 {
    Vec3(
        (p.x - b.lowerBound.x) > (b.upperBound.x - p.x) ? b.lowerBound.x : b.upperBound.x,
        (p.y - b.lowerBound.y) > (b.upperBound.y - p.y) ? b.lowerBound.y : b.upperBound.y,
        (p.z - b.lowerBound.z) > (b.upperBound.z - p.z) ? b.lowerBound.z : b.upperBound.z
    )
}

/// Add a point to an AABB.
func aabbAddPoint(_ a: AABB, _ point: Vec3) -> AABB {
    AABB(minVec3(a.lowerBound, point), maxVec3(a.upperBound, point))
}
