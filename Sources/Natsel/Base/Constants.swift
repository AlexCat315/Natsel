/// Compile-time engine constants. Values that scale with the
/// length unit are computed so `setLengthUnitsPerMeter` takes effect.
@usableFromInline
enum B3 {
    /// https://en.wikipedia.org/wiki/Pi
    /// This literal rounds to the nearest float; `Float.pi` rounds toward zero.
    @usableFromInline static let pi: Float = 3.14159265359

    /// Convert from degrees to radians.
    static let degToRad: Float = 0.01745329251

    /// Convert from radians to degrees.
    static let radToDeg: Float = 57.2957795131

    /// Minimum scale used for scaling collision meshes, etc.
    @usableFromInline static let minScale: Float = 0.01

    static let twoPi: Float = 6.283185307
    static let piOverTwo: Float = 1.570796327
    static let piOverFour: Float = 0.785398163
    static let sqrt3: Float = 1.732050808

    /// Used to indicate null for interfaces that work with indices instead of pointers.
    static let nullIndex: Int32 = -1

    /// Used to detect bad values. In float mode positions greater than about 16km have
    /// precision problems, so 100km is a safe limit.
    @usableFromInline static var huge: Float { 1.0e5 * lengthUnitsPerMeter }

    /// Maximum parallel workers. Used for some fixed size arrays.
    static let maxWorkers: Int32 = 32

    /// Maximum number of tasks queued per world step.
    static let maxTasks: Int32 = 256

    /// Maximum number of colors in the constraint graph. Constraints that cannot find a
    /// color are added to the overflow set which are solved single-threaded.
    static let graphColorCount: Int32 = 24

    /// Number of contact point buckets for counting the number of contact points per
    /// shape contact pair. This is just for reporting and doesn't affect simulation.
    static let contactManifoldCountBuckets: Int32 = 8

    /// A small length used as a collision and constraint tolerance. Usually it is chosen to be
    /// numerically significant, but visually insignificant. In meters.
    static var linearSlop: Float { 0.005 * lengthUnitsPerMeter }

    /// The minimum length of a capsules. Very short capsules should be created as spheres.
    static var minCapsuleLength: Float { linearSlop }

    /// Minimum contact point friction weight, lower bound for speculative points.
    static let minFrictionWeight: Float = 1e-10

    /// The distance between shapes where they are considered overlapped.
    static var overlapSlop: Float { 0.1 * linearSlop }

    /// Maximum number of simultaneous worlds that can be allocated.
    static let maxWorlds: Int32 = 128

    /// The maximum rotation of a body per time step.
    static let maxRotation: Float = 0.25 * pi

    static var speculativeDistance: Float { 4 * linearSlop }

    /// The rest offset is used for mesh contact to reduce ghost collisions and assist with CCD.
    static var meshRestOffset: Float { 1 * linearSlop }

    /// The default contact recycling distance.
    static var contactRecycleDistance: Float { 10 * linearSlop }
    static let contactRecycleAngularDistance: Float = 0.99240388

    /// The default contact recycling world angle threshold, cos(angle/2)^2 for 10 degrees.

    /// This is used to fatten AABBs in the dynamic tree.
    static var maxAABBMargin: Float { 0.05 * lengthUnitsPerMeter }

    /// Per-shape AABB margin is a fraction of the shape extent (capped by maxAABBMargin).
    static let aabbMarginFraction: Float = 0.125

    /// The time that a body must be still before it will go to sleep. In seconds.
    static let timeToSleep: Float = 0.5

    /// The maximum number of restitution iterations. Needed to avoid a solver overflow.
    static let maxRestitutionIterations: Int32 = 63

    /// The maximum number of contact points between two touching shapes.
    static let maxManifoldPoints: Int32 = 4

    /// The number of iterations for gyroscopic torques.
    static let gyroscopicIterations: Int32 = 1

    /// The maximum number of convex hull vertices. This is fixed for performance reasons.
    static let maxHullVertices: Int32 = 128

    /// The maximum number of convex hull faces.
    static let maxHullFaces: Int32 = 128

    /// The maximum number of convex hull edges. Full edges, not half-edges.
    static let maxHullEdges: Int32 = 128

    /// Relative tolerance used to determine if two edges are parallel.
    static let parallelEdgeTolerance: Float = 0.005

    /// The maximum number points to use for shape cast proxies (swept point cloud).
    static let maxShapeCastPoints: Int32 = maxHullVertices

    /// These generous limits allow for easy hashing. See shapePairKey.
    static let shapePower: Int32 = 22
    static let childPower: Int32 = 64 - 2 * shapePower
    static let maxShapes: Int32 = 1 << shapePower
    static let maxChildShapes: Int64 = 1 << Int64(childPower)

    /// The limit on how many mesh or heightfield triangles a single convex shape can collide with.
    static let maxMeshContactTriangles: Int32 = 512

    /// Minimum memory alignment used for all allocations.
    static let alignment: Int = 32

    /// Initial value for the djb2 hash used in determinism testing.
    static let hashInit: UInt32 = 5381

    // Serialized data versions. These are part of the on-disk format.

    /// Dynamic tree version.
    static let dynamicTreeVersion: UInt64 = 0x1D6F_4C2A_73B8_0E91

    /// Hull version. Useful for validating serialized data.
    static let hullVersion: UInt64 = 0x6B1E_39D4_A87C_25F3

    /// Mesh version. Useful for validating serialized data.
    static let meshVersion: UInt64 = 0xAAAB_9A00_F1A8_AAF7

    static let compoundVersion: UInt64 = 0x9E4B_17D3_A25C_68F1 ^ meshVersion ^ hullVersion

    /// Height field version. Useful for validating serialized data.
    static let heightFieldVersion: UInt64 = 0x8E41_E5FB_0848_48F8
}

nonisolated(unsafe) private var lengthUnits: Float = 1

/// Natsel bases all length units on meters, but you may need different units for your game.
/// This must be set at application startup, before any other call into Natsel.
public func setLengthUnitsPerMeter(_ lengthUnits: Float) {
    b3Assert(isValidFloat(lengthUnits) && lengthUnits > 0)
    Natsel.lengthUnits = lengthUnits
}

/// The current length units per meter.
@usableFromInline var lengthUnitsPerMeter: Float { lengthUnits }

/// Get the current length units per meter.
public func getLengthUnitsPerMeter() -> Float { lengthUnits }
