/// Immutable triangle geometry shared by scaled mesh instances.
public final class TriangleMesh: @unchecked Sendable, Equatable {
    let storage: GeometryStorage
    var base: UnsafeMutableRawPointer { storage.base }
    public var byteCount: Int32 { storage.byteCount }

    init(storage: GeometryStorage) { self.storage = storage }

    public static func == (lhs: TriangleMesh, rhs: TriangleMesh) -> Bool { lhs === rhs }

    public convenience init?(vertices: [Vec3], indices: [Int32], materials: [UInt8]? = nil,
                             options: MeshBuildOptions = MeshBuildOptions()) {
        guard let prepared = prepareMesh(vertices: vertices, indices: indices, materials: materials, options: options) else { return nil }
        self.init(prepared: prepared)
    }

    init(prepared: PreparedMesh) {
        let layout = MeshStorageLayout(mesh: prepared)
        storage = GeometryStorage(takingOwnershipOf: b3AllocZero(layout.byteCount)!, byteCount: Int32(layout.byteCount))
        let header = base.bindMemory(to: MeshData.self, capacity: 1)
        header.initialize(to: MeshData())
        header.pointee.version = B3.meshVersion
        header.pointee.byteCount = byteCount
        header.pointee.bounds = prepared.bounds
        header.pointee.surfaceArea = prepared.surfaceArea
        header.pointee.treeHeight = Int32(prepared.treeHeight)
        header.pointee.degenerateCount = Int32(prepared.degenerateCount)
        header.pointee.nodeOffset = Int32(layout.nodes)
        header.pointee.nodeCount = Int32(prepared.nodes.count)
        header.pointee.vertexOffset = Int32(layout.vertices)
        header.pointee.vertexCount = Int32(prepared.vertices.count)
        header.pointee.triangleOffset = Int32(layout.triangles)
        header.pointee.triangleCount = Int32(prepared.triangles.count)
        header.pointee.materialOffset = Int32(layout.materials)
        header.pointee.materialCount = Int32(prepared.materialCount)
        header.pointee.flagsOffset = Int32(layout.flags)
        Self.store(prepared.nodes, at: base + layout.nodes)
        Self.store(prepared.vertices, at: base + layout.vertices)
        Self.store(prepared.triangles, at: base + layout.triangles)
        Self.store(prepared.materials, at: base + layout.materials)
        Self.store(prepared.flags, at: base + layout.flags)
        header.pointee.hash = hash64NonZero(base.assumingMemoryBound(to: UInt8.self), byteCount)
    }


    var header: MeshData { base.assumingMemoryBound(to: MeshData.self).pointee }
    var nodes: UnsafeBufferPointer<MeshNode> { buffer(offset: header.nodeOffset, count: header.nodeCount) }
    var vertices: UnsafeBufferPointer<Vec3> { buffer(offset: header.vertexOffset, count: header.vertexCount) }
    var triangles: UnsafeBufferPointer<MeshTriangle> { buffer(offset: header.triangleOffset, count: header.triangleCount) }
    var materialIndices: UnsafeBufferPointer<UInt8> { buffer(offset: header.materialOffset, count: header.triangleCount) }
    var edgeFlags: UnsafeBufferPointer<UInt8> { buffer(offset: header.flagsOffset, count: header.triangleCount) }

    public var vertexCount: Int { Int(header.vertexCount) }
    public var triangleCount: Int { Int(header.triangleCount) }
    public var bounds: AABB { header.bounds }
    public var surfaceArea: Float { header.surfaceArea }
    public var hash: UInt64 { header.hash }
    public var degenerateTriangleCount: Int { Int(header.degenerateCount) }

    private func buffer<T>(offset: Int32, count: Int32) -> UnsafeBufferPointer<T> {
        UnsafeBufferPointer(start: (base + Int(offset)).assumingMemoryBound(to: T.self), count: Int(count))
    }

    private static func store<T: BitwiseCopyable>(_ values: [T], at address: UnsafeMutableRawPointer) {
        values.withUnsafeBufferPointer { buffer in
            address.bindMemory(to: T.self, capacity: buffer.count).initialize(from: buffer.baseAddress!, count: buffer.count)
        }
    }
}

private struct MeshStorageLayout {
    var nodes: Int
    var vertices: Int
    var triangles: Int
    var materials: Int
    var flags: Int
    var byteCount: Int

    init(mesh: PreparedMesh) {
        func aligned(_ count: Int) -> Int { (count + 7) & ~7 }
        nodes = aligned(MemoryLayout<MeshData>.stride)
        vertices = nodes + aligned(mesh.nodes.count * MemoryLayout<MeshNode>.stride)
        triangles = vertices + aligned(mesh.vertices.count * MemoryLayout<Vec3>.stride)
        materials = triangles + aligned(mesh.triangles.count * MemoryLayout<MeshTriangle>.stride)
        flags = materials + aligned(mesh.materials.count)
        byteCount = flags + aligned(mesh.flags.count)
    }
}

public func computeMeshAABB(_ mesh: TriangleMesh, _ transform: Transform, scale: Vec3) -> AABB {
    let lower = scale * mesh.bounds.lowerBound
    let upper = scale * mesh.bounds.upperBound
    return aabbTransform(transform, AABB(minVec3(lower, upper), maxVec3(lower, upper)))
}
