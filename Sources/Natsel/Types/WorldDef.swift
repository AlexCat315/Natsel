

public let defaultCategoryBits: UInt64 = .max
public let defaultMaskBits: UInt64 = .max

/// A task the scheduler must run on a worker thread exactly once per enqueue, passing back the
/// same task context supplied to `EnqueueTaskCallback`.
public typealias TaskCallback = @Sendable (_ taskContext: UnsafeMutableRawPointer?) -> Void

/// Invoke a task system. Returns the user's task object, or nil to indicate the work was executed
/// serially within the callback and there is no need to call `FinishTaskCallback`.
public typealias EnqueueTaskCallback = @Sendable (
    _ task: @escaping TaskCallback,
    _ taskContext: UnsafeMutableRawPointer?,
    _ taskName: StaticString
) -> UnsafeMutableRawPointer?

/// Finishes a user task object that wraps a task. This must block until the task has completed.
public typealias FinishTaskCallback = @Sendable (_ userTask: UnsafeMutableRawPointer) -> Void

/// Create a user debug draw shape. Called when a shape is first drawn.
public typealias CreateDebugShapeCallback = (_ debugShape: DebugShape) -> UnsafeMutableRawPointer?

/// Destroy a user debug draw shape. Called when a shape is modified or destroyed.
public typealias DestroyDebugShapeCallback = (_ userShape: UnsafeMutableRawPointer?) -> Void

/// Optional friction mixing callback. Called from a worker thread.
/// This function should not attempt to modify engine state or user application state.
public typealias FrictionCallback = @Sendable (
    _ frictionA: Float, _ userMaterialIdA: UInt64, _ frictionB: Float, _ userMaterialIdB: UInt64
) -> Float

/// Optional restitution mixing callback. Called from a worker thread.
/// This function should not attempt to modify engine state or user application state.
public typealias RestitutionCallback = @Sendable (
    _ restitutionA: Float, _ userMaterialIdA: UInt64, _ restitutionB: Float, _ userMaterialIdB: UInt64
) -> Float

/// Contact filter callback. Called when a contact pair is considered for collision, only if one of
/// the two shapes has custom filtering enabled. Must be thread-safe. Return false to disable the
/// collision. Do not attempt to modify the world inside this callback.
public typealias CustomFilterCallback = @Sendable (_ shapeIdA: ShapeID, _ shapeIdB: ShapeID) -> Bool

/// Pre-solve callback. Called after a contact is updated, only for shapes that enabled pre-solve
/// events. Must be thread-safe. Return false to disable the contact this step.
public typealias PreSolveCallback = @Sendable (_ shapeIdA: ShapeID, _ shapeIdB: ShapeID, _ point: Pos, _ normal: Vec3) -> Bool

/// Overlap query callback. Called for each shape found in the query. Return false to terminate.
public typealias OverlapResultCallback = (_ shapeId: ShapeID) -> Bool

/// Ray cast callback. Return -1 to filter, 0 to terminate, fraction to clip the ray for closest
/// hit, 1 to continue.
public typealias CastResultCallback = (
    _ shapeId: ShapeID, _ point: Pos, _ normal: Vec3, _ fraction: Float,
    _ userMaterialId: UInt64, _ triangleIndex: Int32, _ childIndex: Int32
) -> Float

/// Optional world capacities that can be used to avoid run-time allocations.
public struct Capacity: Sendable, Equatable {
    /// Number of expected static shapes.
    public var staticShapeCount: Int32 = 0
    /// Number of expected dynamic and kinematic shapes.
    public var dynamicShapeCount: Int32 = 0
    /// Number of expected static bodies.
    public var staticBodyCount: Int32 = 0
    /// Number of expected dynamic and kinematic bodies.
    public var dynamicBodyCount: Int32 = 0
    /// Number of expected contacts.
    public var contactCount: Int32 = 0

    public init() {}
}

/// World definition used to create a simulation world.
public struct WorldDef {
    /// Gravity vector. There is no up-vector defined.
    public var gravity = Vec3(0, -10, 0)

    /// Restitution speed threshold, usually in m/s. Collisions above this speed have
    /// restitution applied (will bounce).
    public var restitutionThreshold: Float = 1 * lengthUnitsPerMeter

    /// Number of iterations of the restitution solver. More iterations can lead to less box spinning.
    public var restitutionIterations: Int32 = 2

    /// Enable full contact propagation in the restitution solver. Expensive.
    public var enableRestitutionPropagation = false

    /// Hit event speed threshold, usually in m/s.
    public var hitEventThreshold: Float = 1 * lengthUnitsPerMeter

    /// Contact stiffness. Cycles per second.
    public var contactHertz: Float = 30

    /// Contact bounciness. Non-dimensional.
    public var contactDampingRatio: Float = 10

    /// This parameter controls how fast overlap is resolved and usually has units of meters per second.
    public var contactSpeed: Float = 3 * lengthUnitsPerMeter

    /// Maximum linear speed. Usually meters per second. 400 meters per second, faster than the
    /// speed of sound.
    public var maximumLinearSpeed: Float = 400 * lengthUnitsPerMeter

    /// Use this to disable gyroscopic torque on sphere-like objects automatically.
    /// Range is [0, 1]. A sphere is 0 and a 1x1x2 box is 0.5.
    public var isotropicInertiaTolerance: Float = 0.2

    /// Optional mixing callback for friction. The default uses sqrt(frictionA * frictionB).
    public var frictionCallback: FrictionCallback?

    /// Optional mixing callback for restitution. The default uses max(restitutionA, restitutionB).
    public var restitutionCallback: RestitutionCallback?

    /// Optional user filter. Both shapes must allow custom filtering for this to be called.
    public var customFilterCallback: CustomFilterCallback?

    /// Optional pre-solve callback. Returning false disables the contact for this step.
    public var preSolveCallback: PreSolveCallback?

    /// Can bodies go to sleep to improve performance
    public var enableSleep = true

    /// Enable continuous collision
    public var enableContinuous = true

    /// Number of workers to use with the provided task system. This is clamped to the range
    /// [1, 32]. Using a value above 1 will turn on multithreading. If task callbacks are provided
    /// then the user task system is used. Otherwise threads are created with an internal scheduler.
    public var workerCount: UInt32 = 0

    /// Function to spawn task
    public var enqueueTask: EnqueueTaskCallback?

    /// Function to finish a task
    public var finishTask: FinishTaskCallback?

    /// User data associated with a world
    public var userData: UnsafeMutableRawPointer?

    /// Used to create debug draw shapes.
    public var createDebugShape: CreateDebugShapeCallback?

    /// Used to destroy debug draw shapes.
    public var destroyDebugShape: DestroyDebugShapeCallback?

    /// Optional initial capacities
    public var capacity = Capacity()

    public init() {}
}

/// The explosion definition is used to configure options for explosions. Explosions consider
/// shape geometry when computing the impulse.
public struct ExplosionDef: Sendable, Equatable {
    /// Mask bits to filter shapes
    public var maskBits: UInt64 = defaultMaskBits

    /// The center of the explosion in world space
    public var position = Pos()

    /// The radius of the explosion
    public var radius: Float = 0

    /// The falloff distance beyond the radius. Impulse is reduced to zero at this distance.
    public var falloff: Float = 0

    /// Impulse per unit area. Explosions only apply to spheres, capsules, and hulls. This may be
    /// negative for implosions.
    public var impulsePerArea: Float = 0

    public init() {}
}

/// Profiling data. Times are in milliseconds.
public struct Profile: Sendable, Equatable {
    public var step: Float = 0
    public var pairs: Float = 0
    public var collide: Float = 0
    public var solve: Float = 0
    public var solverSetup: Float = 0
    public var constraints: Float = 0
    public var prepareConstraints: Float = 0
    public var integrateVelocities: Float = 0
    public var warmStart: Float = 0
    public var solveImpulses: Float = 0
    public var integratePositions: Float = 0
    public var relaxImpulses: Float = 0
    public var restitution: Float = 0
    public var storeImpulses: Float = 0
    public var splitIslands: Float = 0
    public var transforms: Float = 0
    public var sensorHits: Float = 0
    public var jointEvents: Float = 0
    public var hitEvents: Float = 0
    public var refit: Float = 0
    public var bullets: Float = 0
    public var sleepIslands: Float = 0
    public var sensors: Float = 0

    public init() {}
}

/// Counters that give details of the simulation size.
public struct Counters: Sendable {
    public var byteCount: Int64 = 0
    public var bodyCount: Int32 = 0
    public var shapeCount: Int32 = 0
    public var contactCount: Int32 = 0
    public var jointCount: Int32 = 0
    public var islandCount: Int32 = 0
    public var stackUsed: Int32 = 0
    public var arenaCapacity: Int32 = 0
    public var staticTreeHeight: Int32 = 0
    public var treeHeight: Int32 = 0
    public var satCallCount: Int32 = 0
    public var satCacheHitCount: Int32 = 0
    public var taskCount: Int32 = 0
    public var colorCounts = InlineArray<24, Int32>(repeating: 0)
    public var manifoldCounts = InlineArray<8, Int32>(repeating: 0)

    /// Number of contacts touched by the collide pass: graph contacts + awake-set non-touching
    public var awakeContactCount: Int32 = 0

    /// Number of contacts recycled in the most recent step.
    public var recycledContactCount: Int32 = 0

    /// Maximum number of time of impact outer iterations
    public var distanceIterations: Int32 = 0

    /// Maximum TOI push backs.
    public var pushBackIterations: Int32 = 0

    /// Maximum TOI root solver iterations.
    public var rootIterations: Int32 = 0

    public init() {}
}
