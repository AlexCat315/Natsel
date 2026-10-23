

// MARK: - Manifold

/// A manifold point is a contact point belonging to a contact manifold.
public struct ManifoldPoint: Sendable, Equatable, BitwiseCopyable {
    /// Location of the contact point relative to the bodyA center of mass in world space.
    public var anchorA = Vec3()

    /// Location of the contact point relative to the bodyB center of mass in world space.
    public var anchorB = Vec3()

    /// The separation of the contact point, negative if penetrating
    public var separation: Float = 0

    /// The impulse along the manifold normal vector.
    public var normalImpulse: Float = 0

    /// The total normal impulse applied across sub-stepping and restitution.
    public var totalNormalImpulse: Float = 0

    /// Relative normal velocity pre-solve. Negative when approaching.
    public var normalVelocity: Float = 0

    /// Cached separation used for contact recycling
    public var baseSeparation: Float = 0

    /// Local point for matching. Uniquely identifies a contact point between two shapes
    public var featureId: UInt32 = 0

    /// Triangle index if one of the shapes is a mesh or height field
    public var triangleIndex: Int32 = 0

    /// Did this contact point exist in the previous step?
    public var persisted = false

    public init() {}
}

/// A contact manifold describes the contact points between colliding shapes.
public struct Manifold: Sendable, Equatable, BitwiseCopyable {
    /// The manifold points. There may be 1 to 4 valid points.
    public var points = InlineArray<4, ManifoldPoint>(repeating: ManifoldPoint())

    /// The unit normal vector in world space, points from shape A to shape B
    public var normal = Vec3()

    /// Central friction angular impulse (applied about the normal)
    public var twistImpulse: Float = 0

    /// Central friction linear impulse
    public var frictionImpulse = Vec3()

    /// Rolling resistance angular impulse
    public var rollingImpulse = Vec3()

    /// The number of contact points, will be 0 to 4
    public var pointCount: Int32 = 0

    public init() {}

    public static func == (lhs: Manifold, rhs: Manifold) -> Bool {
        guard lhs.normal == rhs.normal,
              lhs.twistImpulse == rhs.twistImpulse,
              lhs.frictionImpulse == rhs.frictionImpulse,
              lhs.rollingImpulse == rhs.rollingImpulse,
              lhs.pointCount == rhs.pointCount
        else {
            return false
        }
        for i in 0..<lhs.points.count where lhs.points[i] != rhs.points[i] {
            return false
        }
        return true
    }
}

// MARK: - Separating Feature

/// Cached separating axis feature.
public enum SeparatingFeature: UInt8, Sendable, BitwiseCopyable {
    case invalidAxis = 0
    case backsideAxis
    case faceAxisA
    case faceAxisB
    case edgePairAxis
    case closestPointsAxis

    /// These are for testing
    case manualFaceAxisA
    case manualFaceAxisB
    case manualEdgePairAxis
}

/// Cached triangle feature.
public enum TriangleFeature: UInt8, Sendable, BitwiseCopyable {
    case featureNone = 0
    case featureTriangleFace
    case featureHullFace
    /// v1-v2
    case featureEdge1
    /// v2-v3
    case featureEdge2
    /// v3-v1
    case featureEdge3
    case featureVertex1
    case featureVertex2
    case featureVertex3
}

/// Separating axis test cache. Provides temporal acceleration of collision routines.
public struct SATCache: Sendable, Equatable, BitwiseCopyable {
    /// The separation when the cache is populated. Negative for overlap.
    public var separation: Float = 0

    /// SeparatingFeature.
    public var type: SeparatingFeature = .invalidAxis

    /// Index of the feature on shape A.
    public var indexA: UInt8 = 0

    /// Index of the feature on shape B.
    public var indexB: UInt8 = 0

    /// Was the cache re-used?
    public var hit: UInt8 = 0

    public init() {}
}

/// Contact points are always the result of two edges intersecting.
public struct FeaturePair: Sendable, Equatable, BitwiseCopyable {
    /// Incoming type (either edge on shape A or shape B)
    public var owner1: UInt8 = 0
    /// Incoming edge index (into associated shape array)
    public var index1: UInt8 = 0
    /// Outgoing type (either edge on shape A or shape B)
    public var owner2: UInt8 = 0
    /// Outgoing edge index (into associated shape array)
    public var index2: UInt8 = 0

    public init() {}
}

/// A local manifold point and normal in frame A.
public struct LocalManifoldPoint: Sendable, Equatable, BitwiseCopyable {
    /// Local point in frame A.
    public var point = Vec3()

    /// The contact point separation. Negative for overlap.
    public var separation: Float = 0

    /// The feature pair for this point.
    public var pair = FeaturePair()

    /// The triangle index when collide with a mesh or height-field.
    public var triangleIndex: Int32 = 0

    public init() {}
}

/// A local manifold with no dynamic information. Used by collide functions.
///
/// The `points` buffer is owned by the caller and only borrowed for the duration of a collide
/// call, matching the C API where the buffer is supplied by the shape's manifold storage.
public struct LocalManifold: @unchecked Sendable {
    /// Local normal in frame A.
    public var normal = Vec3()

    /// The triangle normal.
    public var triangleNormal = Vec3()

    /// The manifold points. From a point buffer.
    public var points: UnsafeMutablePointer<LocalManifoldPoint>?

    /// The number of manifold points. Only bounded by the buffer capacity.
    public var pointCount: Int32 = 0

    /// The index of the triangle.
    public var triangleIndex: Int32 = 0

    public var i1: Int32 = 0
    public var i2: Int32 = 0
    public var i3: Int32 = 0

    /// The squared distance of a sphere from a triangle. For ghost collision reduction.
    public var squaredDistance: Float = 0

    /// The triangle feature involved.
    public var feature: TriangleFeature = .featureNone

    /// MeshEdgeFlags.
    public var triangleFlags: Int32 = 0

    public init() {}
}
