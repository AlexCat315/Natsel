public func createGridMesh(xCount: Int, zCount: Int, cellWidth: Float,
                           materialCount: Int = 0, identifyEdges: Bool = false) -> TriangleMesh? {
    guard xCount > 0, zCount > 0, cellWidth.isFinite, cellWidth > 0,
          0...255 ~= materialCount else { return nil }
    var vertices: [Vec3] = []
    vertices.reserveCapacity((xCount + 1) * (zCount + 1))
    let xWidth = cellWidth * Float(xCount)
    let zWidth = cellWidth * Float(zCount)
    var x: Float = -0.5 * xWidth
    for _ in 0...xCount {
        var z: Float = -0.5 * zWidth
        for _ in 0...zCount {
            vertices.append(Vec3(x, 0, z))
            z += cellWidth
        }
        x += cellWidth
    }
    let indices = meshGridIndices(xCount: xCount, zCount: zCount)
    var materials: [UInt8] = []
    materials.reserveCapacity(2 * xCount * zCount)
    var materialIndex = 0
    for _ in 0..<xCount {
        for _ in 0..<zCount {
            if materialCount > 0 {
                materials.append(contentsOf: [UInt8(materialIndex % materialCount), UInt8(materialIndex % materialCount)])
            }
            materialIndex += 1
        }
    }
    var options = MeshBuildOptions()
    options.useMedianSplit = true
    options.identifyEdges = identifyEdges
    return TriangleMesh(vertices: vertices, indices: indices,
                        materials: materialCount > 0 ? materials : nil, options: options)
}
