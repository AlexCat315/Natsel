func overlapMesh(_ mesh: Mesh, _ transform: Transform, _ proxy: ShapeProxy) -> Bool {
    guard let data = mesh.data else { return false }
    b3Assert(proxy.count > 0)
    var localPoints = [Vec3](repeating: .zero, count: Int(B3.maxShapeCastPoints))
    return localPoints.withUnsafeMutableBufferPointer { points in
        let localProxy = makeLocalProxy(proxy, transform, points.baseAddress!)
        let bounds = MeshQueryBounds(bounds: computeProxyAABB(localProxy), scale: mesh.scale)
        var input = DistanceInput()
        input.proxyB = localProxy
        input.transform = .identity
        input.useRadii = true
        let vertices = data.vertices
        var touching = false
        var cache = SimplexCache()
        visitMeshCandidates(data, bounds) { _, triangle in
            let a = vertices[Int(triangle.index1)]
            let b = vertices[Int(triangle.index2)]
            let c = vertices[Int(triangle.index3)]
            if boundsTriangleOverlap(bounds.center, bounds.extent, a, b, c) {
                let trianglePoints = [mesh.scale * a, mesh.scale * b, mesh.scale * c]
                touching = trianglePoints.withUnsafeBufferPointer { buffer in
                    input.proxyA = ShapeProxy(points: buffer.baseAddress, count: 3, radius: 0)
                    cache.count = 0
                    return shapeDistance(input, &cache).distance < 0.1 * B3.linearSlop
                }
            }
            return !touching
        }
        return touching
    }
}

/// Query scaled mesh triangles overlapping a local-space box. Return false to stop.
public func queryMesh(_ mesh: Mesh, bounds: AABB, _ callback: (Vec3, Vec3, Vec3, Int32) -> Bool) {
    guard let data = mesh.data else { return }
    let localBounds = MeshQueryBounds(bounds: bounds, scale: mesh.scale)
    let vertices = data.vertices
    let ccw = mesh.scale.x * mesh.scale.y * mesh.scale.z > 0
    visitMeshCandidates(data, localBounds) { index, triangle in
        let a = vertices[Int(triangle.index1)]
        let b = vertices[Int(triangle.index2)]
        let c = vertices[Int(triangle.index3)]
        if boundsTriangleOverlap(localBounds.center, localBounds.extent, a, b, c) {
            return callback(mesh.scale * a, mesh.scale * (ccw ? b : c), mesh.scale * (ccw ? c : b), Int32(index))
        }
        return true
    }
}
