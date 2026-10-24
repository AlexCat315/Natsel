// Ray casts against soft body surfaces.

extension World {
    /// Closest hit of a ray on one soft body's faces.
    public func castRaySoftBody(_ softBodyId: SoftBodyID, _ origin: Pos, _ translation: Vec3) -> SoftBodyRayResult {
        guard let index = softBodyIndex(softBodyId) else { return SoftBodyRayResult() }
        var result = rayCastSoftBodyRecord(softBodies[index], origin, translation, maxFraction: 1)
        if result.hit { result.softBodyId = softBodyId }
        return result
    }

    /// Closest hit of a ray on any soft body whose filter accepts the query.
    public func castRayClosestSoftBody(
        _ origin: Pos, _ translation: Vec3, _ filter: QueryFilter = .defaultFilter
    ) -> SoftBodyRayResult {
        var best = SoftBodyRayResult()
        var input = RayCastInput()
        input.origin = origin
        input.translation = translation
        input.maxFraction = 1
        for i in softBodies.indices where softBodies[i].id != B3.nullIndex {
            let record = softBodies[i]
            guard shouldQueryCollide(record.filter, filter) else { continue }
            input.maxFraction = best.fraction
            guard rayCastAABB(record.bounds, input) else { continue }
            let hit = rayCastSoftBodyRecord(record, origin, translation, maxFraction: best.fraction)
            if hit.hit && hit.fraction < best.fraction {
                best = hit
                best.softBodyId = SoftBodyID(index1: record.id + 1, world0: worldId, generation: record.generation)
            }
        }
        return best
    }

    private func rayCastSoftBodyRecord(
        _ record: SoftBodyRecord, _ origin: Pos, _ translation: Vec3, maxFraction: Float
    ) -> SoftBodyRayResult {
        var result = SoftBodyRayResult()
        result.fraction = maxFraction
        let doubleSided = record.facesDoubleSided
        record.positions.withUnsafeBufferPointer { x in
            for (f, face) in record.settings.faces.enumerated() {
                let a = x[Int(face.vertex.x)]
                let b = x[Int(face.vertex.y)]
                let c = x[Int(face.vertex.z)]
                guard let (t, normal) = rayTriangle(origin, translation, a, b, c, doubleSided: doubleSided),
                    t < result.fraction
                else { continue }
                result.hit = true
                result.fraction = t
                result.faceIndex = Int32(f)
                result.normal = normal
                result.point = mulAdd(origin, t, translation)
            }
        }
        return result
    }
}

/// Slab test of a ray segment against a box.
func rayCastAABB(_ box: AABB, _ input: RayCastInput) -> Bool {
    var tMin: Float = 0
    var tMax = input.maxFraction
    let p = input.origin
    let d = input.translation
    for axis in 0..<3 {
        let pa = axis == 0 ? p.x : axis == 1 ? p.y : p.z
        let da = axis == 0 ? d.x : axis == 1 ? d.y : d.z
        let lo = axis == 0 ? box.lowerBound.x : axis == 1 ? box.lowerBound.y : box.lowerBound.z
        let hi = axis == 0 ? box.upperBound.x : axis == 1 ? box.upperBound.y : box.upperBound.z
        if absFloat(da) < 1.0e-12 {
            if pa < lo || pa > hi { return false }
            continue
        }
        let inv = 1 / da
        var t1 = (lo - pa) * inv
        var t2 = (hi - pa) * inv
        if t1 > t2 { swap(&t1, &t2) }
        tMin = maxFloat(tMin, t1)
        tMax = minFloat(tMax, t2)
        if tMin > tMax { return false }
    }
    return true
}

/// Möller–Trumbore. Returns the fraction along `translation` and the face normal facing the ray.
@inline(__always)
func rayTriangle(_ origin: Vec3, _ translation: Vec3, _ a: Vec3, _ b: Vec3, _ c: Vec3, doubleSided: Bool) -> (Float, Vec3)? {
    let e1 = b - a
    let e2 = c - a
    let normal = cross(e1, e2)
    let facing = dot(normal, translation)
    if doubleSided == false && facing >= 0 { return nil }
    let p = cross(translation, e2)
    let determinant = dot(e1, p)
    guard absFloat(determinant) > 1.0e-12 else { return nil }
    let invDet = 1 / determinant
    let s = origin - a
    let u = dot(s, p) * invDet
    guard u >= 0 && u <= 1 else { return nil }
    let q = cross(s, e1)
    let v = dot(translation, q) * invDet
    guard v >= 0 && u + v <= 1 else { return nil }
    let t = dot(e2, q) * invDet
    guard t >= 0 && t <= 1 else { return nil }
    var n = normalize(normal)
    if facing > 0 { n = -n }
    return (t, n)
}
