private struct MeshAdjacencyEdge {
    var vertex1: Int32
    var vertex2: Int32
    var triangle1: Int
    var triangle2 = -1
    var triangleCount: UInt16 = 1
    var edge1: Int
    var edge2 = -1
}

func identifyMeshEdges(vertices: [Vec3], triangles: [MeshTriangle]) -> [UInt8] {
    var edges: [MeshAdjacencyEdge] = []
    var normals: [Vec3] = []
    edges.reserveCapacity(3 * triangles.count)
    normals.reserveCapacity(triangles.count)
    for (i, triangle) in triangles.enumerated() {
        let indices = [triangle.index1, triangle.index2, triangle.index3]
        for j in 0..<3 {
            let a = indices[j]
            let b = indices[(j + 1) % 3]
            edges.append(MeshAdjacencyEdge(vertex1: min(a, b), vertex2: max(a, b), triangle1: i, edge1: j))
        }
        let v1 = vertices[Int(triangle.index1)]
        let v2 = vertices[Int(triangle.index2)]
        let v3 = vertices[Int(triangle.index3)]
        normals.append(normalize(cross(v2 - v1, v3 - v1)))
    }
    var map: [UInt64: Int] = [:]
    map.reserveCapacity(edges.count)
    for i in edges.indices {
        let key = UInt64(UInt32(bitPattern: edges[i].vertex1)) << 32 | UInt64(UInt32(bitPattern: edges[i].vertex2))
        if let other = map[key] {
            if edges[other].triangleCount == 1 {
                edges[other].triangle2 = edges[i].triangle1
                edges[other].edge2 = edges[i].edge1
            }
            edges[other].triangleCount &+= 1
        } else {
            map[key] = i
        }
    }
    var flags = [UInt8](repeating: 0, count: triangles.count)
    for edge in edges where edge.triangleCount == 2 {
        let a = triangles[edge.triangle1]
        let b = triangles[edge.triangle2]
        let opposite = edge.edge2 == 0 ? b.index3 : (edge.edge2 == 1 ? b.index1 : b.index2)
        let volume = signedVolume(vertices[Int(a.index1)], vertices[Int(a.index2)], vertices[Int(a.index3)], vertices[Int(opposite)])
        let cosAngle = dot(normals[edge.triangle1], normals[edge.triangle2])
        if volume > 0 || cosAngle > 0.9962 {
            flags[edge.triangle1] |= UInt8(1 << edge.edge1)
            flags[edge.triangle2] |= UInt8(1 << edge.edge2)
        }
        if volume < 0 || cosAngle > 0.9962 {
            flags[edge.triangle1] |= UInt8(0x10 << edge.edge1)
            flags[edge.triangle2] |= UInt8(0x10 << edge.edge2)
        }
    }
    return flags
}
