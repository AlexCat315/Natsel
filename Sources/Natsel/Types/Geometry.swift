

// MARK: - Sphere

/// A solid sphere
public struct Sphere: Sendable, Equatable, BitwiseCopyable {
    /// The local center
    public var center = Vec3()

    /// The radius
    public var radius: Float = 0

    public init() {}

    public init(center: Vec3, radius: Float) {
        self.center = center
        self.radius = radius
    }
}

// MARK: - Capsule

/// A solid capsule can be viewed as two hemispheres connected by a cylinder.
public struct Capsule: Sendable, Equatable, BitwiseCopyable {
    /// Local center of the first hemisphere
    public var center1 = Vec3()

    /// Local center of the second hemisphere
    public var center2 = Vec3()

    /// The radius of the hemispheres
    public var radius: Float = 0

    public init() {}

    public init(center1: Vec3, center2: Vec3, radius: Float) {
        self.center1 = center1
        self.center2 = center2
        self.radius = radius
    }
}

// MARK: - Hull

/// A hull vertex. Identified by a half-edge with this vertex as its tail.
public struct HullVertex: Sendable, Equatable, BitwiseCopyable {
    /// A half-edge that has this vertex as the origin
    public var edge: UInt8 = 0

    public init() {}
}

/// Half-edge for hull data structure
public struct HullHalfEdge: Sendable, Equatable, BitwiseCopyable {
    /// Next edge index CCW
    public var next: UInt8 = 0

    /// Twin edge index
    public var twin: UInt8 = 0

    /// index of origin vertex and point
    public var origin: UInt8 = 0

    /// Face to the left of this edge
    public var face: UInt8 = 0

    public init() {}
}

/// A hull face. Hulls use a half-edge data structure, so a face can be determined from a single half-edge index.
public struct HullFace: Sendable, Equatable, BitwiseCopyable {
    /// An arbitrary half-edge on this face
    public var edge: UInt8 = 0

    public init() {}
}

/// A convex hull.
/// @note This data structure has data hanging off the end and cannot be directly copied.
public struct HullData: Sendable, BitwiseCopyable {
    /// Version must be first and match B3_HULL_VERSION
    public var version: UInt64 = 0

    /// Hash of this hull
    public var hash: UInt64 = 0

    /// Axis-aligned box in local space.
    public var aabb = AABB()

    /// Surface area, typically in squared meters.
    public var surfaceArea: Float = 0

    /// Volume, typically in m^3.
    public var volume: Float = 0

    /// The radius of the largest sphere at the center.
    public var innerRadius: Float = 0

    /// The local centroid.
    public var center = Vec3()

    /// The inertia tensor about the centroid.
    public var centralInertia = Matrix3()

    /// The vertex count.
    public var vertexCount: Int32 = 0

    /// Offset of the vertex array in bytes from the struct address.
    public var vertexOffset: Int32 = 0

    /// Offset of the point array in bytes from the struct address.
    public var pointOffset: Int32 = 0

    /// This is the half-edge count (double the edge count).
    public var edgeCount: Int32 = 0

    /// Offset of the edge array in bytes from the struct address.
    public var edgeOffset: Int32 = 0

    /// The face count. Hulls faces are convex polygons.
    public var faceCount: Int32 = 0

    /// Offset of the face plane array in bytes from the struct address.
    public var planeOffset: Int32 = 0

    /// Offset of the face array in bytes from the struct address.
    public var faceOffset: Int32 = 0

    /// Offset of structure of array (SOA) vertices.
    public var soaVertexOffset: Int32 = 0

    /// Offset of structure of array (SOA) unit normal vectors.
    public var soaNormalOffset: Int32 = 0

    /// Offset of dot(n1, n2) for each full edge.
    public var edgeCosineOffset: Int32 = 0

    /// The total number of bytes for this hull.
    public var byteCount: Int32 = 0

    /// Explicit padding for determinism.
    public var padding: Int32 = 0

    public init() {}
}

// MARK: - Mesh

/// Mesh edge flags.
public struct MeshEdgeFlags: OptionSet, Sendable, BitwiseCopyable {
    public var rawValue: UInt8

    public init(rawValue: UInt8) {
        self.rawValue = rawValue
    }

    public static let concaveEdge1 = MeshEdgeFlags(rawValue: 0x01)
    public static let concaveEdge2 = MeshEdgeFlags(rawValue: 0x02)
    public static let concaveEdge3 = MeshEdgeFlags(rawValue: 0x04)
    public static let inverseConcaveEdge1 = MeshEdgeFlags(rawValue: 0x10)
    public static let inverseConcaveEdge2 = MeshEdgeFlags(rawValue: 0x20)
    public static let inverseConcaveEdge3 = MeshEdgeFlags(rawValue: 0x40)
}

/// A mesh triangle.
public struct MeshTriangle: Sendable, Equatable, BitwiseCopyable {
    public var index1: Int32 = 0
    public var index2: Int32 = 0
    public var index3: Int32 = 0

    public init() {}
}

/// A mesh BVH node.
public struct MeshNode: Sendable, Equatable, BitwiseCopyable {
    /// The lower bound of the node AABB.
    public var lowerBound = Vec3()

    /// Union of node/leaf data.
    public var data: UInt32 = 0

    /// The upper bound of the node AABB.
    public var upperBound = Vec3()

    /// The index of the leaf triangles.
    public var triangleOffset: UInt32 = 0

    public init() {}
}


/// This allows mesh data to be re-used with different scales.
public struct Mesh: Sendable, Equatable {
    /// Shared immutable triangle geometry.
    public var data: TriangleMesh?

    /// This scale may be non-uniform and have negative components.
    public var scale = Vec3(1, 1, 1)

    public init() {}

    public init(data: TriangleMesh?, scale: Vec3) {
        self.data = data
        self.scale = scale
    }
}

// MARK: - Height Field

/// Data used to create a height field.
public struct HeightFieldDef: Sendable, Equatable {
    /// Grid point heights
    public var heights: [Float] = []

    /// Grid cell material. A value of 0xFF is reserved for holes.
    public var materialIndices: [UInt8] = []

    /// The height field scale. All components must be positive values.
    public var scale = Vec3(1, 1, 1)

    /// The number of grid lines along the x-axis.
    public var countX: Int32 = 0

    /// The number of grid lines along the z-axis.
    public var countZ: Int32 = 0

    /// Global minimum and maximum heights used for quantization.
    public var globalMinimumHeight: Float = 0

    /// The maximum.
    public var globalMaximumHeight: Float = 0

    /// Use clock-wise winding.
    public var clockwiseWinding = false

    public init() {}
}


// MARK: - Compound

/// Definition for a capsule in a compound shape.
public struct CompoundCapsuleDef: Sendable, Equatable, BitwiseCopyable {
    /// Local capsule.
    public var capsule = Capsule()

    /// Material properties.
    public var material = SurfaceMaterial()

    public init() {}
}

/// Definition for a convex hull in a compound shape.
public struct CompoundHullDef:  @unchecked Sendable {
    /// Shared hull.
    public var hull: Hull?

    /// Transform of the shared hull into compound local space.
    public var transform = Transform()

    /// Material properties.
    public var material = SurfaceMaterial()

    public init() {}
}

/// Definition for a triangle mesh in a compound shape.
public struct CompoundMeshDef:  @unchecked Sendable, Equatable {
    /// Shared mesh.
    public var meshData: TriangleMesh?

    /// Transform of the shared mesh into compound local space.
    public var transform = Transform()

    /// Local space non-uniform mesh scale. May have negative components.
    public var scale = Vec3(1, 1, 1)

    /// Material properties.
    public var materials: [SurfaceMaterial] = []

    public init() {}
}

/// Definition for a sphere in a compound shape.
public struct CompoundSphereDef: Sendable, Equatable, BitwiseCopyable {
    /// Local sphere.
    public var sphere = Sphere()

    /// Material properties.
    public var material = SurfaceMaterial()

    public init() {}
}

/// Definition for creating a compound shape.
public struct CompoundDef: Sendable {
    /// Capsule instances.
    public var capsules: [CompoundCapsuleDef] = []

    /// Hull instances.
    public var hulls: [CompoundHullDef] = []

    /// Mesh instances.
    public var meshes: [CompoundMeshDef] = []

    /// Sphere instances.
    public var spheres: [CompoundSphereDef] = []

    public init() {}
}

// MARK: - Mass Data

/// This holds the mass data computed for a shape.
public struct MassData: Sendable, Equatable, BitwiseCopyable {
    /// The shape mass
    public var mass: Float = 0

    /// The local center of mass position.
    public var center = Vec3()

    /// The inertia tensor about the shape center of mass.
    public var inertia = Matrix3()

    public init() {}
}
