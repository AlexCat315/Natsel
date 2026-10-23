public enum CompoundChildGeometry: Sendable {
    case capsule(Capsule)
    case hull(Hull)
    case mesh(Mesh)
    case sphere(Sphere)
}

public struct CompoundChild: Sendable {
    public let geometry: CompoundChildGeometry
    public let transform: Transform
    public let materialIndices: [UInt16]
}

/// A baked, immutable compound with shared child geometry and materials.
public final class Compound: @unchecked Sendable {
    let storage: GeometryStorage
    private var hullViews: [UInt32: Hull] = [:]
    private var meshViews: [UInt32: TriangleMesh] = [:]

    public convenience init?(_ definition: CompoundDef) {
        guard let builder = CompoundBuilder(definition) else { return nil }
        self.init(storage: bakeCompound(builder))
    }

    public convenience init?(serializedBytes: [UInt8]) {
        guard validateCompoundBytes(serializedBytes) else { return nil }
        let base = b3Alloc(serializedBytes.count)!
        serializedBytes.withUnsafeBytes { base.copyMemory(from: $0.baseAddress!, byteCount: $0.count) }
        self.init(storage: GeometryStorage(takingOwnershipOf: base, byteCount: Int32(serializedBytes.count)))
    }

    private init(storage: GeometryStorage) {
        self.storage = storage
        for i in 0..<Int(header.hullCount) {
            let instance: PackedHullInstance = value(offset: header.hullOffset, index: i)
            let offset = Int(instance.hullOffset)
            if hullViews[instance.hullOffset] == nil {
                let data = (storage.base + offset).assumingMemoryBound(to: HullData.self).pointee
                hullViews[instance.hullOffset] = Hull(storage: storage.slice(offset: offset, byteCount: data.byteCount))
            }
        }
        for i in 0..<Int(header.meshCount) {
            let instance: PackedMeshInstance = value(offset: header.meshOffset, index: i)
            let offset = Int(instance.meshOffset)
            if meshViews[instance.meshOffset] == nil {
                let data = (storage.base + offset).assumingMemoryBound(to: MeshData.self).pointee
                meshViews[instance.meshOffset] = TriangleMesh(storage: storage.slice(offset: offset, byteCount: data.byteCount))
            }
        }
    }

    var header: CompoundData { storage.base.assumingMemoryBound(to: CompoundData.self).pointee }
    public var byteCount: Int32 { storage.byteCount }
    public var childCount: Int { Int(header.capsuleCount + header.hullCount + header.meshCount + header.sphereCount) }
    public var bounds: AABB { header.bounds }
    public var materials: [SurfaceMaterial] {
        Array(UnsafeBufferPointer(start: (storage.base + Int(header.materialOffset)).assumingMemoryBound(to: SurfaceMaterial.self), count: Int(header.materialCount)))
    }

    public func serializedBytes() -> [UInt8] { geometryBytes(storage.base, byteCount) }

    public func child(at index: Int) -> CompoundChild {
        precondition(index >= 0 && index < childCount)
        var index = index
        if index < Int(header.capsuleCount) {
            let instance: PackedCapsule = value(offset: header.capsuleOffset, index: index)
            return CompoundChild(geometry: .capsule(instance.capsule), transform: .identity, materialIndices: [instance.materialIndex])
        }
        index -= Int(header.capsuleCount)
        if index < Int(header.hullCount) {
            let instance: PackedHullInstance = value(offset: header.hullOffset, index: index)
            return CompoundChild(geometry: .hull(hullViews[instance.hullOffset]!), transform: instance.transform, materialIndices: [instance.materialIndex])
        }
        index -= Int(header.hullCount)
        if index < Int(header.meshCount) {
            let instance: PackedMeshInstance = value(offset: header.meshOffset, index: index)
            let indices = UnsafeBufferPointer(start: (storage.base + Int(instance.materialOffset)).assumingMemoryBound(to: UInt16.self), count: Int(instance.materialCount))
            return CompoundChild(geometry: .mesh(Mesh(data: meshViews[instance.meshOffset], scale: instance.scale)),
                                 transform: instance.transform, materialIndices: Array(indices))
        }
        index -= Int(header.meshCount)
        let instance: PackedSphere = value(offset: header.sphereOffset, index: index)
        return CompoundChild(geometry: .sphere(instance.sphere), transform: .identity, materialIndices: [instance.materialIndex])
    }

    func withTreeView<R>(_ body: (TreeView) -> R) -> R {
        let nodes = UnsafeBufferPointer(start: (storage.base + Int(header.nodeOffset)).assumingMemoryBound(to: TreeNode.self), count: Int(header.nodeCount))
        return body(TreeView(nodes: nodes, proxies: nil))
    }

    private func value<T>(offset: Int32, index: Int) -> T {
        (storage.base + Int(offset)).assumingMemoryBound(to: T.self)[index]
    }
}

public func computeCompoundAABB(_ compound: Compound, _ transform: Transform) -> AABB {
    aabbTransform(transform, compound.bounds)
}
