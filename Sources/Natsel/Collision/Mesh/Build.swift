/// Mesh construction choices. A zero welding tolerance preserves all input vertices.
public struct MeshBuildOptions: Sendable {
    public var weldTolerance: Float = 0
    public var useMedianSplit = false
    public var identifyEdges = false
    public var clockwiseWinding = false

    public init() {}
}

struct PreparedMesh {
    var vertices: [Vec3]
    var triangles: [MeshTriangle]
    var nodes: [MeshNode]
    var materials: [UInt8]
    var flags: [UInt8]
    var bounds: AABB
    var surfaceArea: Float
    var treeHeight: Int
    var degenerateCount: Int
    var materialCount: Int
}

func prepareMesh(vertices source: [Vec3], indices sourceIndices: [Int32],
                 materials: [UInt8]?, options: MeshBuildOptions) -> PreparedMesh? {
    guard source.count >= 3, !sourceIndices.isEmpty, sourceIndices.count % 3 == 0,
          materials == nil || materials!.count == sourceIndices.count / 3,
          sourceIndices.allSatisfy({ $0 >= 0 && Int($0) < source.count }) else { return nil }
    let vertices: [Vec3]
    let indices: [Int32]
    if options.weldTolerance > 0 {
        let welded = weldMeshVertices(source, indices: sourceIndices, tolerance: options.weldTolerance)
        vertices = welded.vertices
        indices = welded.indices
    } else {
        vertices = source
        indices = sourceIndices
    }
    var bounds = bounds3Empty
    var primitives: [MeshPrimitive] = []
    var degenerateCount = 0
    var surfaceArea: Float = 0
    var materialCount = 1
    let minArea: Float = 0.01 * B3.linearSlop * B3.linearSlop
    for index in 0..<(indices.count / 3) {
        let i1 = indices[3 * index]
        let i2 = indices[3 * index + 1]
        let i3 = indices[3 * index + 2]
        let v1 = vertices[Int(i1)]
        let v2 = vertices[Int(i2)]
        let v3 = vertices[Int(i3)]
        let normal = cross(v2 - v1, v3 - v1)
        let area: Float = 0.5 * length(normal)
        if area < minArea {
            if i1 != i2 && i1 != i3 && i2 != i3 { degenerateCount += 1 }
            continue
        }
        surfaceArea += area
        let box = AABB(minVec3(v1, minVec3(v2, v3)), maxVec3(v1, maxVec3(v2, v3)))
        primitives.append(MeshPrimitive(bounds: box, center: aabbCenter(box), triangleIndex: index))
        if let materials { materialCount = max(materialCount, Int(materials[index]) + 1) }
        bounds = aabbUnion(bounds, box)
    }
    guard !primitives.isEmpty, isSaneAABB(bounds) else { return nil }
    let tree = MeshTreeBuild(primitives: &primitives, useMedianSplit: options.useMedianSplit)
    var triangles: [MeshTriangle] = []
    var sortedMaterials: [UInt8] = []
    for primitive in primitives {
        let offset = 3 * primitive.triangleIndex
        var triangle = MeshTriangle()
        triangle.index1 = indices[offset]
        triangle.index2 = indices[offset + (options.clockwiseWinding ? 2 : 1)]
        triangle.index3 = indices[offset + (options.clockwiseWinding ? 1 : 2)]
        triangles.append(triangle)
        sortedMaterials.append(materials?[primitive.triangleIndex] ?? 0)
    }
    // The builder emits leaves in DFS order with contiguous ranges, so a subsequent
    // DFS sorting pass copies the same triangle and material order without permuting it.
    guard meshTreeFitsTraversalStack(tree.nodes) else { return nil }
    let flags = options.identifyEdges
        ? identifyMeshEdges(vertices: vertices, triangles: triangles)
        : [UInt8](repeating: 0, count: triangles.count)
    return PreparedMesh(vertices: vertices, triangles: triangles, nodes: tree.nodes,
                        materials: sortedMaterials, flags: flags, bounds: bounds,
                        surfaceArea: surfaceArea, treeHeight: tree.height,
                        degenerateCount: degenerateCount, materialCount: materialCount)
}

private func meshTreeFitsTraversalStack(_ nodes: [MeshNode]) -> Bool {
    var stack = [0]
    while let index = stack.popLast() {
        let node = nodes[index]
        if !node.isLeaf {
            if stack.count >= 254 { return false }
            stack.append(index + node.childOffset)
            stack.append(index + 1)
        }
    }
    return true
}
