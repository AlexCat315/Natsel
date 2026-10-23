func collideMoverAndMesh(_ mesh: Mesh, _ mover: Capsule, capacity: Int) -> [PlaneResult] {
    guard capacity > 0, let data = mesh.data else { return [] }
    let center = lerp(mover.center1, mover.center2, 0.5)
    let radius = Vec3(mover.radius, mover.radius, mover.radius)
    let bounds = MeshQueryBounds(bounds: AABB(minVec3(mover.center1, mover.center2) - radius,
                                              maxVec3(mover.center1, mover.center2) + radius), scale: mesh.scale)
    let ccw = mesh.scale.x * mesh.scale.y * mesh.scale.z > 0
    let vertices = data.vertices
    let materials = data.materialIndices
    var results: [PlaneResult] = []
    let moverPoints = [mover.center1, mover.center2]
    moverPoints.withUnsafeBufferPointer { points in
        var input = DistanceInput()
        input.proxyB = ShapeProxy(points: points.baseAddress, count: 2, radius: 0)
        input.transform = .identity
        input.useRadii = false
        var cache = SimplexCache()
        visitMeshCandidates(data, bounds) { index, triangle in
            let a = vertices[Int(triangle.index1)]
            let b = vertices[Int(ccw ? triangle.index2 : triangle.index3)]
            let c = vertices[Int(ccw ? triangle.index3 : triangle.index2)]
            if !boundsTriangleOverlap(bounds.center, bounds.extent, a, b, c) { return true }
            let scaled = [mesh.scale * a, mesh.scale * b, mesh.scale * c]
            if signedVolume(scaled[0], scaled[1], scaled[2], center) < 0 { return true }
            let output = scaled.withUnsafeBufferPointer { trianglePoints in
                input.proxyA = ShapeProxy(points: trianglePoints.baseAddress, count: 3, radius: 0)
                cache.count = 0
                return shapeDistance(input, &cache)
            }
            if output.distance != 0 && output.distance <= mover.radius {
                var result = PlaneResult()
                result.plane = Plane(output.normal, mover.radius - output.distance)
                result.point = output.pointA
                result.triangleIndex = Int32(index)
                result.materialIndex = Int32(materials[index])
                results.append(result)
            }
            return results.count < capacity
        }
    }
    return results
}
