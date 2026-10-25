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
        input.translation = translation
        input.maxFraction = 1
        for i in softBodies.indices where softBodies[i].id != B3.nullIndex {
            let record = softBodies[i]
            guard shouldQueryCollide(record.filter, filter) else { continue }
            input.maxFraction = best.fraction
            input.origin = subPos(origin, record.origin)
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
        let localOrigin = subPos(origin, record.origin)
        var tree: DynamicTree?
        if enableSoftBodyQueryAcceleration && record.settings.faces.count >= 32 {
            if let cached = softBodyRayCaches[record.id], cached.generation == record.generation,
                cached.revision == record.spatialRevision {
                tree = cached.tree
            } else {
                let cache = SoftBodyRayCache(record)
                softBodyRayCaches[record.id] = cache
                tree = cache.tree
            }
        }
        record.positions.withUnsafeBufferPointer { x in
            func testFace(_ f: Int) {
                let face = record.settings.faces[f]
                let a = x[Int(face.vertex.x)]
                let b = x[Int(face.vertex.y)]
                let c = x[Int(face.vertex.z)]
                guard let (t, normal) = rayTriangle(localOrigin, translation, a, b, c, doubleSided: doubleSided),
                    t < result.fraction || (result.hit && t == result.fraction && Int32(f) < result.faceIndex)
                else { return }
                result.hit = true
                result.fraction = t
                result.faceIndex = Int32(f)
                result.normal = normal
                result.point = mulAdd(origin, t, translation)
            }
            if let tree {
                let input = RayCastInput(origin: localOrigin, translation: translation, maxFraction: maxFraction)
                _ = tree.rayCast(input, maskBits: .max) { _, _, data in
                    testFace(Int(data))
                    return result.fraction
                }
            } else {
                for f in record.settings.faces.indices { testFace(f) }
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
