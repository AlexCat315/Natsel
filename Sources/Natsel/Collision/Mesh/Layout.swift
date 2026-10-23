// Internal header for the owned geometry allocation.
struct MeshData: Sendable, BitwiseCopyable {
    /// Version must be first.
    var version: UInt64 = 0

    /// Hash of this mesh
    var hash: UInt64 = 0

    /// The total number of bytes for this mesh.
    var byteCount: Int32 = 0

    /// Local axis-aligned box.
    var bounds = AABB()

    /// Combined surface area of all triangles. Single-sided.
    var surfaceArea: Float = 0

    /// The height of the bounding volume hierarchy.
    var treeHeight: Int32 = 0

    /// The number of degenerate triangles. Diagnostic.
    var degenerateCount: Int32 = 0

    /// Offset of the node array in bytes from the struct address.
    var nodeOffset: Int32 = 0

    /// The number of BVH nodes.
    var nodeCount: Int32 = 0

    /// Offset of the vertex array in bytes from the struct address.
    var vertexOffset: Int32 = 0

    /// The number of vertices.
    var vertexCount: Int32 = 0

    /// Offset of the triangle array in bytes from the struct address.
    var triangleOffset: Int32 = 0

    /// The number of triangles.
    var triangleCount: Int32 = 0

    /// Offset of the material array in bytes from the struct address.
    var materialOffset: Int32 = 0

    /// The number of materials.
    var materialCount: Int32 = 0

    /// Offset of the triangle flag array in bytes from the struct address.
    var flagsOffset: Int32 = 0

    /// Explicit padding.
    var padding: Int32 = 0

    init() {}
}

