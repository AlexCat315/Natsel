

/// Low level ray cast input data.
public struct RayCastInput: Sendable, Equatable, BitwiseCopyable {
    /// Start point of the ray cast.
    public var origin = Vec3()

    /// Translation of the ray cast. end = start + translation.
    public var translation = Vec3()

    /// The maximum fraction of the translation to consider, typically 1
    public var maxFraction: Float = 0

    public init() {}

    public init(origin: Vec3, translation: Vec3, maxFraction: Float) {
        self.origin = origin
        self.translation = translation
        self.maxFraction = maxFraction
    }
}

/// Result from the closest ray cast.
public struct RayResult: Sendable, Equatable {
    /// The shape hit.
    public var shapeId = ShapeID()

    /// The world point of the hit.
    public var point = Pos()

    /// The world normal of the shape surface at the hit point.
    public var normal = Vec3()

    /// The user material id at the hit point.
    public var userMaterialId: UInt64 = 0

    /// The fraction of the input ray.
    public var fraction: Float = 0

    /// The triangle index if the shape is a mesh, height-field, or compound with child mesh.
    public var triangleIndex: Int32 = 0

    /// The child index if the shape is a compound.
    public var childIndex: Int32 = 0

    /// The number of BVH nodes visited. Diagnostic.
    public var nodeVisits: Int32 = 0

    /// The number of BVH leaves visited. Diagnostic.
    public var leafVisits: Int32 = 0

    /// Did the ray hit? If false, all other data is invalid.
    public var hit = false

    public init() {}
}

/// A shape proxy is used by the GJK algorithm. It can represent a convex shape.
public struct ShapeProxy: BitwiseCopyable {
    /// The point cloud.
    public var points: UnsafePointer<Vec3>?

    /// The number of points. Do not exceed the maximum shape cast point count.
    public var count: Int32 = 0

    /// The external radius of the point cloud.
    public var radius: Float = 0

    public init() {}

    public init(points: UnsafePointer<Vec3>?, count: Int32, radius: Float) {
        self.points = points
        self.count = count
        self.radius = radius
    }
}

/// Low level shape cast input in generic form. This allows casting an arbitrary point cloud wrapped
/// with a radius. A sphere is a single point with a non-zero radius. A capsule is two points with
/// a non-zero radius. A box is four points with a zero radius.
public struct ShapeCastInput: BitwiseCopyable {
    /// A generic query shape.
    public var proxy = ShapeProxy()

    /// The translation of the shape cast.
    public var translation = Vec3()

    /// The maximum fraction of the translation to consider, typically 1.
    public var maxFraction: Float = 0

    /// Allow shape cast to encroach when initially touching. This only works if the radius is
    /// greater than zero.
    public var canEncroach = false

    public init() {}
}

/// Input for sweeping an AABB through a dynamic tree. The box is in the tree's world float frame.
public struct BoxCastInput: Sendable, Equatable, BitwiseCopyable {
    /// The AABB to cast, in the tree's frame.
    public var box = AABB()

    /// The sweep translation.
    public var translation = Vec3()

    /// The maximum fraction of the translation to consider, typically 1.
    public var maxFraction: Float = 0

    public init() {}
}

/// Low level ray cast or shape-cast output data.
public struct CastOutput: Sendable, Equatable, BitwiseCopyable {
    /// The surface normal at the hit point.
    public var normal = Vec3()

    /// The surface hit point.
    public var point = Vec3()

    /// The fraction of the input translation at collision.
    public var fraction: Float = 0

    /// The number of iterations used.
    public var iterations: Int32 = 0

    /// The index of the mesh or height field triangle hit.
    public var triangleIndex: Int32 = 0

    /// The index of the compound child shape.
    public var childIndex: Int32 = 0

    /// The material index. May be -1 for null.
    public var materialIndex: Int32 = 0

    /// Did the cast hit?
    public var hit = false

    public init() {}
}


/// Body cast result for ray and shape casts.
public struct BodyCastResult: Sendable, Equatable {
    /// The shape hit.
    public var shapeId = ShapeID()

    /// The world point on the shape surface.
    public var point = Pos()

    /// The world normal vector on the shape surface.
    public var normal = Vec3()

    /// The fraction along the ray hit. hit point = origin + fraction * translation
    public var fraction: Float = 0

    /// The triangle index if the shape is a mesh or height-field.
    public var triangleIndex: Int32 = 0

    /// The user material id at the hit point.
    public var userMaterialId: UInt64 = 0

    /// The number of iterations used. Diagnostic.
    public var iterations: Int32 = 0

    /// Did the cast hit? If false, all other fields are invalid.
    public var hit = false

    public init() {}
}

/// Used to warm start the GJK simplex. If you call this function multiple times with nearby
/// transforms this might improve performance. Otherwise you can zero initialize this.
public struct SimplexCache: Sendable, Equatable, BitwiseCopyable {
    /// Value use to compare length, area, volume of two simplexes.
    public var metric: Float = 0

    /// The number of stored simplex points
    public var count: UInt16 = 0

    /// The cached simplex indices on shape A
    public var indexA = InlineArray<4, UInt8>(repeating: 0)

    /// The cached simplex indices on shape B
    public var indexB = InlineArray<4, UInt8>(repeating: 0)

    public init() {}

    public static func == (lhs: SimplexCache, rhs: SimplexCache) -> Bool {
        lhs.metric == rhs.metric && lhs.count == rhs.count
            && (0..<4).allSatisfy { lhs.indexA[$0] == rhs.indexA[$0] && lhs.indexB[$0] == rhs.indexB[$0] }
    }
}

/// Input parameters for shape casts between two proxies.
public struct ShapeCastPairInput: BitwiseCopyable {
    /// The proxy for shape A
    public var proxyA = ShapeProxy()
    /// The proxy for shape B
    public var proxyB = ShapeProxy()
    /// Transform of shape B in shape A's frame, the relative pose B in A
    public var transform = Transform()
    /// The translation of shape B, in A's frame
    public var translationB = Vec3()
    /// The fraction of the translation to consider, typically 1
    public var maxFraction: Float = 0
    /// Allows shapes with a radius to move slightly closer if already touching
    public var canEncroach = false

    public init() {}
}

/// Input for shape distance.
public struct DistanceInput: BitwiseCopyable {
    /// The proxy for shape A
    public var proxyA = ShapeProxy()

    /// The proxy for shape B
    public var proxyB = ShapeProxy()

    /// Transform of shape B in shape A's frame, the relative pose B in A. The query is origin
    /// independent and runs in frame A.
    public var transform = Transform()

    /// Should the proxy radius be considered?
    public var useRadii = false

    public init() {}
}

/// Output for shape distance.
public struct DistanceOutput: Sendable, Equatable, BitwiseCopyable {
    /// Closest point on shapeA, in shape A's frame
    public var pointA = Vec3()
    /// Closest point on shapeB, in shape A's frame
    public var pointB = Vec3()
    /// A to B normal in shape A's frame. Invalid if distance is zero.
    public var normal = Vec3()
    /// The final distance, zero if overlapped
    public var distance: Float = 0
    /// Number of GJK iterations used
    public var iterations: Int32 = 0
    /// The number of simplexes stored in the simplex array
    public var simplexCount: Int32 = 0

    public init() {}
}

/// Simplex vertex for debugging the GJK algorithm.
public struct SimplexVertex: Sendable, Equatable, BitwiseCopyable {
    /// support point in proxyA
    public var wA = Vec3()
    /// support point in proxyB
    public var wB = Vec3()
    /// wB - wA
    public var w = Vec3()
    /// barycentric coordinates
    public var a: Float = 0
    /// wA index
    public var indexA: Int32 = 0
    /// wB index
    public var indexB: Int32 = 0

    public init() {}
}

/// Simplex from the GJK algorithm.
public struct Simplex: Sendable, BitwiseCopyable {
    /// vertices
    public var vertices = InlineArray<4, SimplexVertex>(repeating: SimplexVertex())
    /// number of valid vertices
    public var count: Int32 = 0

    public init() {}
}

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect
/// to the body origin, which may not coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
public struct Sweep: Sendable, Equatable, BitwiseCopyable {
    /// Local center of mass position
    public var localCenter = Vec3()
    /// Starting center of mass world position
    public var c1 = Vec3()
    /// Ending center of mass world position
    public var c2 = Vec3()
    /// Starting world rotation
    public var q1 = Quat()
    /// Ending world rotation
    public var q2 = Quat()

    public init() {}

    public init(localCenter: Vec3, c1: Vec3, c2: Vec3, q1: Quat, q2: Quat) {
        self.localCenter = localCenter
        self.c1 = c1
        self.c2 = c2
        self.q1 = q1
        self.q2 = q2
    }
}

/// Time of impact input.
public struct TOIInput: BitwiseCopyable {
    /// The proxy for shape A
    public var proxyA = ShapeProxy()
    /// The proxy for shape B
    public var proxyB = ShapeProxy()
    /// The movement of shape A
    public var sweepA = Sweep()
    /// The movement of shape B
    public var sweepB = Sweep()
    /// Defines the sweep interval [0, tMax]
    public var maxFraction: Float = 0

    public init() {}
}

/// Describes the TOI output.
public enum TOIState: Int32, Sendable, BitwiseCopyable {
    case unknown
    case failed
    case overlapped
    case hit
    case separated
}

/// Time of impact output.
public struct TOIOutput: Sendable, Equatable, BitwiseCopyable {
    /// The type of result.
    public var state: TOIState = .unknown

    /// The hit point. A shared point if overlapped.
    public var point = Vec3()

    /// The hit normal. Zero if overlapped.
    public var normal = Vec3()

    /// The sweep time of the collision. 0 if overlapped.
    public var fraction: Float = 0

    /// The final distance. 0 if overlapped.
    public var distance: Float = 0

    /// Number of outer iterations.
    public var distanceIterations: Int32 = 0

    /// Total number of push back iterations.
    public var pushBackIterations: Int32 = 0

    /// Total number of root iterations.
    public var rootIterations: Int32 = 0

    /// Indicates that the time of impact detected initial overlap and used a fallback sphere as a
    /// last ditch effort to prevent tunneling.
    public var usedFallback = false

    public init() {}
}

// MARK: - Character mover

/// The plane between a character mover and a shape.
public struct PlaneResult: Sendable, Equatable, BitwiseCopyable {
    /// Outward pointing plane.
    public var plane = Plane()

    /// Closest point on the shape. May not be unique.
    public var point = Vec3()

    /// The index of the mesh or height field triangle hit.
    public var triangleIndex: Int32 = 0

    /// The index of the compound child shape.
    public var childIndex: Int32 = 0

    /// The material index.
    public var materialIndex: Int32 = 0

    public init() {}
}

/// Collision planes that can be fed to `solvePlanes`. Normally this is assembled by the user from
/// plane results.
public struct CollisionPlane: Sendable, Equatable, BitwiseCopyable {
    /// The collision plane between the mover and some shape.
    public var plane = Plane()

    /// Setting this to Float.greatestFiniteMagnitude makes the plane as rigid as possible. Lower
    /// values can make the plane collision soft. Usually in meters.
    public var pushLimit: Float = 0

    /// The push on the mover determined by `solvePlanes`. Usually in meters.
    public var push: Float = 0

    /// Indicates if `clipVector` should clip against this plane. Should be false for soft collision.
    public var clipVelocity = false

    public init() {}

    public init(plane: Plane, pushLimit: Float, push: Float, clipVelocity: Bool) {
        self.plane = plane
        self.pushLimit = pushLimit
        self.push = push
        self.clipVelocity = clipVelocity
    }
}

/// Result returned by `solvePlanes`.
public struct PlaneSolverResult: Sendable, Equatable {
    /// The final relative translation.
    public var delta = Vec3()

    /// The number of iterations used by the plane solver. For diagnostics.
    public var iterationCount: Int32 = 0

    public init() {}
}

/// Body plane result for movers.
public struct BodyPlaneResult: Sendable, Equatable, BitwiseCopyable {
    /// The shape id on the body.
    public var shapeId = ShapeID()

    /// The plane result.
    public var result = PlaneResult()

    public init() {}
}

/// Body time of impact result for movers.
public struct BodyTOIResult: Sendable, Equatable {
    /// The hit point in world space.
    public var point = Pos()

    /// The hit normal. Points from the body to the mover.
    public var normal = Vec3()

    /// The sweep time of the collision.
    public var fraction: Float = 0

    /// The hit shape.
    public var shapeId = ShapeID()

    public init() {}
}

/// Used to collect collision planes for character movers. Return true to continue gathering planes.
public typealias PlaneResultCallback = (_ shapeId: ShapeID, _ planes: UnsafeBufferPointer<PlaneResult>) -> Bool

/// Used to filter shapes for shape casting character movers. Return true to accept the collision.
public typealias MoverFilterCallback = (_ shapeId: ShapeID) -> Bool

/// A cast against a world shape, preserving the double precision hit position.
public struct WorldCastOutput: Sendable, Equatable, BitwiseCopyable {
    public var normal = Vec3()
    public var point = Pos()
    public var fraction: Float = 0
    public var iterations: Int32 = 0
    public var triangleIndex: Int32 = 0
    public var childIndex: Int32 = 0
    public var materialIndex: Int32 = 0
    public var hit = false
    public init() {}
    internal init(_ local: CastOutput, origin: Pos) {
        normal = local.normal; point = offsetPos(origin, local.point)
        fraction = local.fraction; iterations = local.iterations
        triangleIndex = local.triangleIndex; childIndex = local.childIndex
        materialIndex = local.materialIndex; hit = local.hit
    }
}
