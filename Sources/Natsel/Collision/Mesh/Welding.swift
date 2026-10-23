/// mesh.c's spatial welding. Map iteration order is irrelevant: cells are probed in
/// C's dx/dy/dz order and each cell owns an explicit newest-first linked list.
private struct VertexSpatialHash {
    private struct Node {
        var vertexIndex: Int
        var next: Int?
    }

    let vertices: [Vec3]
    let tolerance: Float
    private var nodes: [Node] = []
    private var cells: [UInt64: Int] = [:]

    init(vertices: [Vec3], tolerance: Float) {
        precondition(tolerance > 0)
        self.vertices = vertices
        self.tolerance = tolerance
        nodes.reserveCapacity(vertices.count)
        cells.reserveCapacity(vertices.count)
    }

    private func key(_ x: Int32, _ y: Int32, _ z: Int32) -> UInt64 {
        var key: UInt64 = 0
        for coordinate in [x, y, z] {
            key ^= UInt64(truncatingIfNeeded: Int64(coordinate)) &+ 0x9e3779b9
                &+ (key << 6) &+ (key >> 2)
        }
        return key
    }

    mutating func findDuplicate(_ current: Int) -> Int? {
        let vertex = vertices[current]
        let cellSize: Float = 2 * tolerance
        let x = Int32((vertex.x / cellSize).rounded(.down))
        let y = Int32((vertex.y / cellSize).rounded(.down))
        let z = Int32((vertex.z / cellSize).rounded(.down))
        for dx: Int32 in -1...1 {
            for dy: Int32 in -1...1 {
                for dz: Int32 in -1...1 {
                    var index = cells[key(x + dx, y + dy, z + dz)]
                    while let i = index {
                        let node = nodes[i]
                        let other = vertices[node.vertexIndex]
                        if abs(vertex.x - other.x) <= tolerance
                            && abs(vertex.y - other.y) <= tolerance
                            && abs(vertex.z - other.z) <= tolerance {
                            return node.vertexIndex
                        }
                        index = node.next
                    }
                }
            }
        }
        let cell = key(x, y, z)
        let next = cells[cell]
        cells[cell] = nodes.count
        nodes.append(Node(vertexIndex: current, next: next))
        return nil
    }
}

struct WeldedMeshVertices {
    var vertices: [Vec3]
    var indices: [Int32]
}

func weldMeshVertices(_ vertices: [Vec3], indices: [Int32], tolerance: Float) -> WeldedMeshVertices {
    var spatialHash = VertexSpatialHash(vertices: vertices, tolerance: tolerance)
    var mapping = [Int32](repeating: 0, count: vertices.count)
    var unique: [Vec3] = []
    unique.reserveCapacity(vertices.count)
    for i in vertices.indices {
        if let duplicate = spatialHash.findDuplicate(i) {
            mapping[i] = mapping[duplicate]
        } else {
            mapping[i] = Int32(unique.count)
            unique.append(vertices[i])
        }
    }
    return WeldedMeshVertices(vertices: unique, indices: indices.map { mapping[Int($0)] })
}
