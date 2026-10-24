/// Transient triangle caches and the world-space bounds that own their validity.
struct MeshContactState {
    var triangles: [TriangleCache] = []
    var queryBounds = AABB()
    /// Reused query scratch. Keeping the old buffer as a second array lets the next refresh read
    /// cached triangles and fill the other buffer without copying the array.
    var triangleScratch: [TriangleCache] = []
    var indexScratch: [Int32] = []

    mutating func refresh(surface: TriangleSurface, transform: WorldTransform, bounds: AABB) {
        if aabbContains(queryBounds, bounds) { return }
        let radius = B3.maxAABBMargin + B3.speculativeDistance
        let extensionVector = Vec3(radius, radius, radius)
        queryBounds = AABB(bounds.lowerBound - extensionVector, bounds.upperBound + extensionVector)
        let meshTransform = toRelativeTransform(transform, .zero)
        let localBounds = aabbTransform(invertTransform(meshTransform), queryBounds)
        indexScratch.removeAll(keepingCapacity: true)
        surface.query(localBounds) { _, _, _, index in
            if indexScratch.count == Int(B3.maxMeshContactTriangles) { return false }
            indexScratch.append(index)
            return indexScratch.count < Int(B3.maxMeshContactTriangles)
        }
        triangleScratch.removeAll(keepingCapacity: true)
        var oldIndex = 0
        for index in indexScratch {
            while oldIndex < triangles.count && triangles[oldIndex].triangleIndex < index { oldIndex += 1 }
            var entry = TriangleCache()
            entry.triangleIndex = index
            if oldIndex < triangles.count && triangles[oldIndex].triangleIndex == index {
                entry.cache = triangles[oldIndex].cache
            }
            triangleScratch.append(entry)
        }
        swap(&triangles, &triangleScratch)
    }
}

