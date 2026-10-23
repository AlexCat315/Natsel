struct MeshTriangleManifold {
    var normal: Vec3
    var triangleNormal: Vec3
    var points: [LocalManifoldPoint]
    var triangleIndex: Int32
    var vertices: InlineArray<3, Int32>
    var squaredDistance: Float
    var feature: TriangleFeature
    var flags: Int32
}

private struct MeshSurvivor {
    var cacheIndex: Int
    var triangle: Triangle
    var points: [Vec3]
}

extension World {
    func collideMeshTriangles(_ contact: inout Contact, surface: TriangleSurface, transformA: WorldTransform,
                              shapeB: Shape, transformB: WorldTransform, isFast: Bool) -> [MeshTriangleManifold] {
        let transform = invMulWorldTransforms(transformB, transformA)
        let matrix = makeMatrixFromQuat(transform.q)
        let speculative = contact.flags & ContactFlags.enableSpeculativePoints.rawValue != 0
        let hull = shapeB.hull
        let distance: Float = speculative ? B3.speculativeDistance : 0
        let margin = Vec3(distance, distance, distance)
        var survivors: [MeshSurvivor] = []
        for (index, entry) in contact.meshContact.triangles.enumerated() {
            let triangle = surface.triangle(entry.triangleIndex)
            // Each scalar lane uses exactly the nested adds in C's four-wide transform.
            let points = (0..<3).map { mulMV(matrix, triangle.vertices[$0]) + transform.p }
            if let hull {
                let lower = minVec3(points[0], minVec3(points[1], points[2])) - margin
                let upper = maxVec3(points[0], maxVec3(points[1], points[2])) + margin
                let bounds = hull.aabb
                if upper.x < bounds.lowerBound.x || lower.x > bounds.upperBound.x
                    || upper.y < bounds.lowerBound.y || lower.y > bounds.upperBound.y
                    || upper.z < bounds.lowerBound.z || lower.z > bounds.upperBound.z {
                    contact.meshContact.triangles[index].cache = .sat(SATCache())
                    continue
                }
            }
            survivors.append(MeshSurvivor(cacheIndex: index, triangle: triangle, points: points))
        }
        let capacity = 32 * contact.meshContact.triangles.count
        var pointBuffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: capacity)
        var totalPointCount = 0
        var manifolds: [MeshTriangleManifold] = []
        pointBuffer.withUnsafeMutableBufferPointer { buffer in
            for survivor in survivors {
                if totalPointCount + 3 >= capacity { break }
                let index = survivor.cacheIndex
                let triangle = survivor.triangle
                let triangleIndex = contact.meshContact.triangles[index].triangleIndex
                var manifold = LocalManifold()
                manifold.points = buffer.baseAddress! + totalPointCount
                manifold.triangleFlags = triangle.flags
                let available = Int32(capacity - totalPointCount)
                switch shapeB.geometry {
                case .capsule(let capsule):
                    var cache = SimplexCache()
                    if case .simplex(let saved) = contact.meshContact.triangles[index].cache { cache = saved }
                    survivor.points.withUnsafeBufferPointer { points in
                        collideTriangleAndCapsule(&manifold, available, points.baseAddress!, capsule, &cache)
                    }
                    contact.meshContact.triangles[index].cache = .simplex(cache)
                case .hull(let hull):
                    var cache = SATCache()
                    if case .sat(let saved) = contact.meshContact.triangles[index].cache { cache = saved }
                    if isFast && cache.type == .edgePairAxis { cache = SATCache() }
                    collideTriangleAndHull(&manifold, available, survivor.points[0], survivor.points[1], survivor.points[2],
                                           triangle.flags, hull, &cache, speculative)
                    satCallCount += 1
                    satCacheHitCount += Int32(cache.hit)
                    contact.meshContact.triangles[index].cache = .sat(cache)
                case .sphere(let sphere):
                    survivor.points.withUnsafeBufferPointer { points in
                        collideTriangleAndSphere(&manifold, available, points.baseAddress!, sphere)
                    }
                default:
                    b3Assert(false)
                    continue
                }
                if manifold.pointCount > 0 {
                    let points = Array(UnsafeBufferPointer(start: manifold.points!, count: Int(manifold.pointCount)))
                    manifolds.append(MeshTriangleManifold(normal: manifold.normal,
                        triangleNormal: makeNormalFromPoints(survivor.points[0], survivor.points[1], survivor.points[2]),
                        points: points, triangleIndex: triangleIndex, vertices: [triangle.i1, triangle.i2, triangle.i3],
                        squaredDistance: manifold.squaredDistance, feature: manifold.feature, flags: triangle.flags))
                    totalPointCount += Int(manifold.pointCount)
                }
            }
        }
        return manifolds
    }
}
