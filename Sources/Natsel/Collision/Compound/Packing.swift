struct CompoundData: Sendable, BitwiseCopyable {
    /// The compound version is always first.
    var version: UInt64 = 0

    /// The total number of bytes for this compound.
    var byteCount: Int32 = 0

    /// Bounds of the tree.
    var bounds = AABB()

    /// Tree height for diagnostics.
    var treeHeight: Int32 = 0

    /// Offset of the tree node array in bytes from the struct address.
    var nodeOffset: Int32 = 0

    /// The number of tree nodes.
    var nodeCount: Int32 = 0

    /// Offset of the material array in bytes from the struct address.
    var materialOffset: Int32 = 0

    /// The number of materials.
    var materialCount: Int32 = 0

    /// Offset of the capsule array in bytes from the struct address.
    var capsuleOffset: Int32 = 0

    /// The number of capsules.
    var capsuleCount: Int32 = 0

    /// Offset of the hull instance array in bytes from the struct address.
    var hullOffset: Int32 = 0

    /// The number of hull instances.
    var hullCount: Int32 = 0

    /// The number of unique hulls. Diagnostic.
    var sharedHullCount: Int32 = 0

    /// Offset of the mesh instance array in bytes from the struct address.
    var meshOffset: Int32 = 0

    /// The number of mesh instances.
    var meshCount: Int32 = 0

    /// The number of unique meshes. Diagnostic.
    var sharedMeshCount: Int32 = 0

    /// Offset of the sphere array in bytes from the struct address.
    var sphereOffset: Int32 = 0

    /// The number of spheres.
    var sphereCount: Int32 = 0

    init() {}
}

struct PackedCapsule: BitwiseCopyable {
    var capsule = Capsule()
    var materialIndex: UInt16 = 0
    var padding: UInt16 = 0
}

struct PackedSphere: BitwiseCopyable {
    var sphere = Sphere()
    var materialIndex: UInt16 = 0
    var padding: UInt16 = 0
}

struct PackedHullInstance: BitwiseCopyable {
    var transform = Transform()
    var hullOffset: UInt32 = 0
    var materialIndex: UInt16 = 0
    var padding: UInt16 = 0
}

struct PackedMeshInstance: BitwiseCopyable {
    var transform = Transform()
    var scale = Vec3()
    var meshOffset: UInt32 = 0
    var materialOffset: UInt32 = 0
    var materialCount: UInt32 = 0
}
