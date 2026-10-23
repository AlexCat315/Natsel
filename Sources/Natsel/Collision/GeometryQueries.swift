// MARK: - Triangle queries

/// Find the closest point on triangle `a-b-c` to point `q`.
func closestPointOnTriangle(_ a: Vec3, _ b: Vec3, _ c: Vec3, _ q: Vec3) -> TrianglePoint {
    // Check if P lies in vertex region of A
    let ab = b - a
    let ac = c - a
    let aq = q - a

    let d1 = dot(ab, aq)
    let d2 = dot(ac, aq)
    if d1 <= 0 && d2 <= 0 {
        return TrianglePoint(point: a, feature: .featureVertex1)
    }

    // Check if P lies in vertex region of B
    let bq = q - b

    let d3 = dot(ab, bq)
    let d4 = dot(ac, bq)
    if d3 > 0 && d4 <= d3 {
        return TrianglePoint(point: b, feature: .featureVertex2)
    }

    // Check if P lies in edge region AB
    let vc = d1 * d4 - d3 * d2
    if vc <= 0 && d1 >= 0 && d3 <= 0 {
        let t = d1 / (d1 - d3)
        return TrianglePoint(point: mulAdd(a, t, ab), feature: .featureEdge1)
    }

    // Check if P lies in vertex region of C
    let cq = q - c

    let d5 = dot(ab, cq)
    let d6 = dot(ac, cq)
    if d6 >= 0 && d5 <= d6 {
        return TrianglePoint(point: c, feature: .featureVertex3)
    }

    // Check if P lies in edge region AC
    let vb = d5 * d2 - d1 * d6
    if vb <= 0 && d2 >= 0 && d6 <= 0 {
        let t = d2 / (d2 - d6)
        return TrianglePoint(point: mulAdd(a, t, ac), feature: .featureEdge3)
    }

    // Check if P lies in edge region of BC
    let va = d3 * d6 - d5 * d4
    if va <= 0 && d4 >= d3 && d5 >= d6 {
        let bc = c - b

        let t = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return TrianglePoint(point: mulAdd(b, t, bc), feature: .featureEdge2)
    }

    // P inside face region ABC
    let t1 = vb / (va + vb + vc)
    let t2 = vc / (va + vb + vc)

    var p = mulAdd(a, t1, ab)
    p = mulAdd(p, t2, ac)
    return TrianglePoint(point: p, feature: .featureTriangleFace)
}

// MARK: - Inertia tensors

/// The inertia tensor of a solid sphere about its center.
func sphereInertia(_ mass: Float, _ radius: Float) -> Matrix3 {
    let i = 0.4 * mass * radius * radius
    return makeDiagonalMatrix(i, i, i)
}

/// The inertia tensor of a solid cylinder about its center.
func cylinderInertia(_ mass: Float, _ radius: Float, _ height: Float) -> Matrix3 {
    let ixx = mass * (3 * radius * radius + height * height) / 12.0
    let iyy = 0.5 * mass * radius * radius
    return makeDiagonalMatrix(ixx, iyy, ixx)
}

/// The inertia tensor of a solid box about its center.
func boxInertia(_ mass: Float, _ min: Vec3, _ max: Vec3) -> Matrix3 {
    let delta = max - min
    let ixx = mass * (delta.y * delta.y + delta.z * delta.z) / 12.0
    let iyy = mass * (delta.x * delta.x + delta.z * delta.z) / 12.0
    let izz = mass * (delta.x * delta.x + delta.y * delta.y) / 12.0

    return makeDiagonalMatrix(ixx, iyy, izz)
}
