/// Transient triangle caches and the world-space bounds that own their validity.
struct MeshContactState {
    var triangles: [TriangleCache] = []
    var queryBounds = AABB()

    mutating func refresh(surface: TriangleSurface, transform: WorldTransform, bounds: AABB) {
        if aabbContains(queryBounds, bounds) { return }
        let radius = B3.maxAABBMargin + B3.speculativeDistance
        let extensionVector = Vec3(radius, radius, radius)
        queryBounds = AABB(bounds.lowerBound - extensionVector, bounds.upperBound + extensionVector)
        let meshTransform = toRelativeTransform(transform, .zero)
        let localBounds = aabbTransform(invertTransform(meshTransform), queryBounds)
        var indices: [Int32] = []
        surface.query(localBounds) { _, _, _, index in
            if indices.count == Int(B3.maxMeshContactTriangles) { return false }
            indices.append(index)
            return indices.count < Int(B3.maxMeshContactTriangles)
        }
        var updated: [TriangleCache] = []
        var oldIndex = 0
        for index in indices {
            while oldIndex < triangles.count && triangles[oldIndex].triangleIndex < index { oldIndex += 1 }
            var entry = TriangleCache()
            entry.triangleIndex = index
            if oldIndex < triangles.count && triangles[oldIndex].triangleIndex == index {
                entry.cache = triangles[oldIndex].cache
            }
            updated.append(entry)
        }
        triangles = updated
    }
}

