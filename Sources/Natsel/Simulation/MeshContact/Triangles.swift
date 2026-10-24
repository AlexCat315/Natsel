struct MeshTriangleManifold {
    var normal: Vec3
    var triangleNormal: Vec3
    var points: ArraySlice<LocalManifoldPoint>
    var triangleIndex: Int32
    var vertices: InlineArray<3, Int32>
    var squaredDistance: Float
    var feature: TriangleFeature
    var flags: Int32
    var pointRange: Range<Int> = 0..<0
}

struct MeshSurvivor {
    var cacheIndex: Int
    var triangle: Triangle
    var points: InlineArray<3, Vec3>
}

extension World {
    func collideMeshTriangles(
        _ contact: inout Contact, surface: TriangleSurface, transformA: WorldTransform,
        shapeB: Shape, transformB: WorldTransform, isFast: Bool,
        _ scratch: UnsafeMutablePointer<MeshContactScratch>
    ) -> [MeshTriangleManifold] {
        let transform = invMulWorldTransforms(transformB, transformA)
        let matrix = makeMatrixFromQuat(transform.q)
        let speculative = contact.flags & ContactFlags.enableSpeculativePoints.rawValue != 0
        let hull = shapeB.hull
        let distance: Float = speculative ? B3.speculativeDistance : 0
        let margin = Vec3(distance, distance, distance)
        scratch.pointee.survivors.removeAll(keepingCapacity: true)
        for (index, entry) in contact.meshContact.triangles.enumerated() {
            let triangle = surface.triangle(entry.triangleIndex)
            // Each scalar lane uses exactly the nested adds in C's four-wide transform.
            let points: InlineArray<3, Vec3> = [
                mulMV(matrix, triangle.vertices[0]) + transform.p,
                mulMV(matrix, triangle.vertices[1]) + transform.p,
                mulMV(matrix, triangle.vertices[2]) + transform.p,
            ]
            if let hull {
                let lower = minVec3(points[0], minVec3(points[1], points[2])) - margin
                let upper = maxVec3(points[0], maxVec3(points[1], points[2])) + margin
                let bounds = hull.aabb
                if upper.x < bounds.lowerBound.x || lower.x > bounds.upperBound.x
                    || upper.y < bounds.lowerBound.y || lower.y > bounds.upperBound.y
                    || upper.z < bounds.lowerBound.z || lower.z > bounds.upperBound.z
                {
                    contact.meshContact.triangles[index].cache = .sat(SATCache())
                    continue
                }
            }
            scratch.pointee.survivors.append(MeshSurvivor(cacheIndex: index, triangle: triangle, points: points))
        }
        let triangleCount = contact.meshContact.triangles.count
        let capacity = 32 * triangleCount
        var totalPointCount = 0
        scratch.pointee.manifolds.removeAll(keepingCapacity: true)
        if scratch.pointee.points.count < capacity {
            scratch.pointee.points.append(
                contentsOf: repeatElement(
                    LocalManifoldPoint(), count: capacity - scratch.pointee.points.count))
        }
        let pointBuffer = scratch.pointee.points.withUnsafeMutableBufferPointer { $0.baseAddress! }
        do {
            for survivor in scratch.pointee.survivors {
                if totalPointCount + 3 >= capacity { break }
                let index = survivor.cacheIndex
                let triangle = survivor.triangle
                let triangleIndex = contact.meshContact.triangles[index].triangleIndex
                var manifold = LocalManifold()
                manifold.points = pointBuffer + totalPointCount
                manifold.triangleFlags = triangle.flags
                let available = Int32(capacity - totalPointCount)
                switch shapeB.geometry {
                case .capsule(let capsule):
                    var cache = SimplexCache()
                    if case .simplex(let saved) = contact.meshContact.triangles[index].cache { cache = saved }
                    withInlineReadBuffer(survivor.points) { points in
                        collideTriangleAndCapsule(&manifold, available, points.baseAddress!, capsule, &cache)
                    }
                    contact.meshContact.triangles[index].cache = .simplex(cache)
                case .hull(let hull):
                    var cache = SATCache()
                    if case .sat(let saved) = contact.meshContact.triangles[index].cache { cache = saved }
                    if isFast && cache.type == .edgePairAxis { cache = SATCache() }
                    collideTriangleAndHull(
                        &manifold, available, survivor.points[0], survivor.points[1], survivor.points[2],
                        triangle.flags, hull, &cache, speculative)
                    scratch.pointee.satCalls += 1
                    scratch.pointee.satCacheHits += Int32(cache.hit)
                    contact.meshContact.triangles[index].cache = .sat(cache)
                case .sphere(let sphere):
                    withInlineReadBuffer(survivor.points) { points in
                        collideTriangleAndSphere(&manifold, available, points.baseAddress!, sphere)
                    }
                default:
                    b3Assert(false)
                    continue
                }
                if manifold.pointCount > 0 {
                    // Triangle kernels fill point/separation/features. Mesh clustering supplies
                    // the triangle id; initialize it explicitly without clearing the unused tail.
                    for i in 0..<Int(manifold.pointCount) {
                        manifold.points![i].triangleIndex = 0
                    }
                    scratch.pointee.manifolds.append(
                        MeshTriangleManifold(
                            normal: manifold.normal,
                            triangleNormal: makeNormalFromPoints(
                                survivor.points[0], survivor.points[1], survivor.points[2]),
                            points: [], triangleIndex: triangleIndex,
                            vertices: [triangle.i1, triangle.i2, triangle.i3],
                            squaredDistance: manifold.squaredDistance, feature: manifold.feature, flags: triangle.flags,
                            pointRange: totalPointCount..<(totalPointCount + Int(manifold.pointCount))))
                    totalPointCount += Int(manifold.pointCount)
                }
            }
        }
        // Form shared, immutable slices only after writing the whole arena. Taking a slice
        // inside the collide loop would make subsequent point writes copy the arena.
        for i in scratch.pointee.manifolds.indices {
            scratch.pointee.manifolds[i].points =
                scratch.pointee.points[scratch.pointee.manifolds[i].pointRange]
        }
        return scratch.pointee.manifolds
    }
}
