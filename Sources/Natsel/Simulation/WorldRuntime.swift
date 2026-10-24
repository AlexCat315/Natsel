

// Runtime data model and core body management (body.c, solver_set.h, island.h).

// MARK: - Body flags

/// Body flags (body.h).
public struct BodyFlags: OptionSet, Sendable, BitwiseCopyable {
    public var rawValue: UInt32

    public init(rawValue: UInt32) {
        self.rawValue = rawValue
    }

    public static let lockLinearX = BodyFlags(rawValue: 0x0000_0001)
    public static let lockLinearY = BodyFlags(rawValue: 0x0000_0002)
    public static let lockLinearZ = BodyFlags(rawValue: 0x0000_0004)
    public static let lockAngularX = BodyFlags(rawValue: 0x0000_0008)
    public static let lockAngularY = BodyFlags(rawValue: 0x0000_0010)
    public static let lockAngularZ = BodyFlags(rawValue: 0x0000_0020)

    /// Only lives on BodySim.
    public static let isFast = BodyFlags(rawValue: 0x0000_0040)
    public static let isBullet = BodyFlags(rawValue: 0x0000_0080)
    public static let isSpeedCapped = BodyFlags(rawValue: 0x0000_0100)
    public static let hadTimeOfImpact = BodyFlags(rawValue: 0x0000_0200)
    public static let allowFastRotation = BodyFlags(rawValue: 0x0000_0400)
    public static let enlargeBulletBounds = BodyFlags(rawValue: 0x0000_0800)
    public static let dynamicFlag = BodyFlags(rawValue: 0x0000_1000)
    public static let enableSleep = BodyFlags(rawValue: 0x0000_2000)
    public static let enableContactRecycling = BodyFlags(rawValue: 0x0000_4000)
    public static let dirtyMass = BodyFlags(rawValue: 0x0000_8000)
    public static let enableGyroscopicTorque = BodyFlags(rawValue: 0x0001_0000)

    public static let allLocks: BodyFlags = [
        .lockLinearX, .lockLinearY, .lockLinearZ, .lockAngularX, .lockAngularY, .lockAngularZ,
    ]

    public static let fixedRotation: BodyFlags = [.lockAngularX, .lockAngularY, .lockAngularZ]

    public static let bodyTransientFlags: BodyFlags = [.isSpeedCapped, .hadTimeOfImpact]
}

// MARK: - Solver set kinds

enum SetType: Int32 {
    case staticSet = 0
    case disabledSet = 1
    case awakeSet = 2
    case firstSleepingSet = 3
}

// MARK: - Body state

/// Body state used in the constraint solver. 64 bytes.
struct BodyState: BitwiseCopyable {
    var linearVelocity = Vec3()
    var padding1: Float = 0

    var angularVelocity = Vec3()
    var padding2: Float = 0

    /// Using delta position reduces round-off error far from the origin.
    var deltaPosition = Vec3()

    /// BodyFlags.
    var flags: UInt32 = 0

    /// The delta rotation. Static bodies use identity.
    var deltaRotation = Quat()

    init() {
        deltaRotation = Quat.identity
    }

    static let identity = BodyState()
}

// MARK: - Body simulation

/// Body simulation data used for integration of position and velocity.
struct BodySim: BitwiseCopyable {
    /// Transform for body origin.
    var transform = WorldTransform()

    /// Center of mass position in world space.
    var center = Pos()

    /// BodyFlags.
    var flags: UInt32 = 0

    var minExtent: Float = 0
    var maxExtent = Vec3()

    /// Previous rotation and COM for TOI.
    var rotation0 = Quat()
    var center0 = Pos()

    /// Location of center of mass relative to the body origin.
    var localCenter = Vec3()

    var force = Vec3()
    var torque = Vec3()

    var invMass: Float = 0

    /// Local space inertia.
    var inertia = Matrix3()

    /// Rotational inertia about the center of mass.
    var invInertiaLocal = Matrix3()
    var invInertiaWorld = Matrix3()

    var linearDamping: Float = 0
    var angularDamping: Float = 0
    var gravityScale: Float = 0

    /// Index of Body.
    var bodyId: Int32 = 0

    init() {}
}

// MARK: - Body

/// Body organizational details that are not used in the solver.
struct Body {
    var userData: UnsafeMutableRawPointer?

    /// Index of solver set stored in World. May be `B3.nullIndex`.
    var setIndex: Int32 = B3.nullIndex

    /// Body sim and state index within set. May be `B3.nullIndex`.
    var localIndex: Int32 = B3.nullIndex

    /// [31 : contactId | 1 : edgeIndex]
    var headContactKey: Int32 = B3.nullIndex
    var contactCount: Int32 = 0

    var headShapeId: Int32 = B3.nullIndex
    var shapeCount: Int32 = 0

    var headChainId: Int32 = B3.nullIndex

    /// [31 : jointId | 1 : edgeIndex]
    var headJointKey: Int32 = B3.nullIndex
    var jointCount: Int32 = 0

    /// All enabled dynamic and kinematic bodies are in an island.
    var islandId: Int32 = B3.nullIndex

    /// Index into the island's bodies array for O(1) swap-removal.
    var islandIndex: Int32 = B3.nullIndex

    var sleepThreshold: Float = 0
    var sleepTime: Float = 0
    var sleepVelocity: Float = 0
    var safetyFactor: Float = 0
    var mass: Float = 0

    var bodyMoveIndex: Int32 = B3.nullIndex

    var id: Int32 = B3.nullIndex

    /// BodyFlags.
    var flags: UInt32 = 0
    var nameId: UInt32 = 0

    var type = BodyType.staticBody

    /// Monotonically advanced when a body is allocated in this slot.
    var generation: UInt16 = 0

    init() {}
}

// MARK: - Islands

/// Cached contact data stored in the island for fast contiguous iteration.
struct ContactLink: BitwiseCopyable {
    var contactId: Int32 = 0
    var bodyIdA: Int32 = 0
    var bodyIdB: Int32 = 0

    init() {}
}

/// Cached joint data stored in the island for fast contiguous iteration.
struct JointLink: BitwiseCopyable {
    var jointId: Int32 = 0
    var bodyIdA: Int32 = 0
    var bodyIdB: Int32 = 0

    init() {}
}

/// Persistent island for awake bodies, joints, and touching contacts.
struct Island {
    /// Index of solver set stored in World. May be `B3.nullIndex`.
    var setIndex: Int32 = B3.nullIndex

    /// Island index within set. May be `B3.nullIndex`.
    var localIndex: Int32 = B3.nullIndex

    var islandId: Int32 = B3.nullIndex

    /// Keeps track of how many contacts have been removed from this island.
    var constraintRemoveCount: Int32 = 0

    var bodies: [Int32] = []
    var contacts: [ContactLink] = []
    var joints: [JointLink] = []

    init() {}
}

/// Used to move islands across solver sets.
struct IslandSim: BitwiseCopyable {
    var islandId: Int32 = 0

    init() {}
}

// MARK: - Solver set

/// Solvers sets hold contiguous arrays of body sims, states, joints, contacts, and islands.
struct SolverSet {
    var bodySims: [BodySim] = []
    var bodyStates: [BodyState] = []
    var jointSims: [JointSim] = []
    var contactIndices: [Int32] = []
    var islandSims: [IslandSim] = []
    var setIndex: Int32 = B3.nullIndex

    init() {}
}

// MARK: - Contacts

struct ContactEdge: BitwiseCopyable {
    var bodyId: Int32 = 0
    var prevKey: Int32 = 0
    var nextKey: Int32 = 0

    init() {}
}

/// The union of SAT and simplex caches.
enum ContactCache {
    case sat(SATCache)
    case simplex(SimplexCache)
}

struct TriangleCache {
    var triangleIndex: Int32 = 0
    var cache = ContactCache.sat(SATCache())

    init() {}
}

enum ContactCacheKind: Int32, BitwiseCopyable {
    case convex = 0
    case mesh = 1
}

/// The persistent interaction between two shapes.
struct Contact: BitwiseCopyable {
    var setIndex: Int32 = B3.nullIndex
    var colorIndex: Int32 = B3.nullIndex
    var localIndex: Int32 = B3.nullIndex

    var edges = InlineArray<2, ContactEdge>(repeating: ContactEdge())
    var shapeIdA: Int32 = B3.nullIndex
    var shapeIdB: Int32 = B3.nullIndex
    var childIndex: Int32 = 0

    var islandId: Int32 = B3.nullIndex
    var islandIndex: Int32 = B3.nullIndex
    var contactId: Int32 = B3.nullIndex

    var encodedBodySimA: Int32 = B3.nullIndex
    var encodedBodySimB: Int32 = B3.nullIndex

    var flags: UInt32 = 0

    var payload: UnsafeMutablePointer<ContactPayload>?
    var manifoldCount: Int32 = 0

    var cachedRotationA = Quat()
    var cachedRotationB = Quat()
    var cachedRelativePose = Transform()

    var friction: Float = 0

    /// Convex contact cache. The C union is represented as two fields because a contact's shape
    /// pair fixes which one is written.
    var satCache = SATCache()
    var simplexCache = SimplexCache()

    var restitution: Float = 0
    var rollingResistance: Float = 0
    var tangentVelocity = Vec3()

    var generation: UInt32 = 0

    init() {}
}

// MARK: - Joints

struct JointEdge: BitwiseCopyable {
    var bodyId: Int32 = 0
    var prevKey: Int32 = 0
    var nextKey: Int32 = 0

    init() {}
}

/// Map from JointID to Joint in the solver sets.
struct Joint {
    var userData: UnsafeMutableRawPointer?

    var setIndex: Int32 = B3.nullIndex
    var colorIndex: Int32 = B3.nullIndex
    var localIndex: Int32 = B3.nullIndex

    var edges = InlineArray<2, JointEdge>(repeating: JointEdge())

    var jointId: Int32 = B3.nullIndex
    var islandId: Int32 = B3.nullIndex
    var islandIndex: Int32 = B3.nullIndex

    var drawScale: Float = 1

    var type = JointType.distanceJoint

    var generation: UInt16 = 0
    var collideConnected = false

    init() {}
}

/// The discriminant and only the active joint's POD data share one allocation-free value.
private enum JointPayload: BitwiseCopyable {
    case distance(DistanceJointSim)
    case revolute(RevoluteJointSim)
    case weld(WeldJointSim)
    case parallel(ParallelJointSim)
    case motor(MotorJointSim)
    case prismatic(PrismaticJointSim)
    case spherical(SphericalJointSim)
    case wheel(WheelJointSim)
}

/// Solver data for a joint.
struct JointSim: BitwiseCopyable {
    var jointId: Int32 = B3.nullIndex
    var bodyIdA: Int32 = B3.nullIndex
    var bodyIdB: Int32 = B3.nullIndex
    var type = JointType.distanceJoint

    // Joint frames local to body origin
    var localFrameA = Transform()
    var localFrameB = Transform()

    var invMassA: Float = 0
    var invMassB: Float = 0
    var invIA = Matrix3()
    var invIB = Matrix3()

    var constraintHertz: Float = 0
    var constraintDampingRatio: Float = 0
    var constraintSoftness = Softness()

    var forceThreshold: Float = 0
    var torqueThreshold: Float = 0

    var fixedRotation = false

    // A joint owns exactly one variant. Keeping all eight payloads increased each solver
    // record to over 1.7 KB and copied unused state through every API and solver stage.
    private var payload = JointPayload.distance(DistanceJointSim())

    var distance: DistanceJointSim {
        @inline(__always) get {
            if case .distance(let value) = payload { return value }
            return DistanceJointSim()
        }
        @inline(__always) set { payload = .distance(newValue) }
    }

    var revolute: RevoluteJointSim {
        @inline(__always) get {
            if case .revolute(let value) = payload { return value }
            return RevoluteJointSim()
        }
        @inline(__always) set { payload = .revolute(newValue) }
    }

    var weld: WeldJointSim {
        @inline(__always) get {
            if case .weld(let value) = payload { return value }
            return WeldJointSim()
        }
        @inline(__always) set { payload = .weld(newValue) }
    }

    var parallel: ParallelJointSim {
        @inline(__always) get {
            if case .parallel(let value) = payload { return value }
            return ParallelJointSim()
        }
        @inline(__always) set { payload = .parallel(newValue) }
    }

    var motor: MotorJointSim {
        @inline(__always) get {
            if case .motor(let value) = payload { return value }
            return MotorJointSim()
        }
        @inline(__always) set { payload = .motor(newValue) }
    }

    var prismatic: PrismaticJointSim {
        @inline(__always) get {
            if case .prismatic(let value) = payload { return value }
            return PrismaticJointSim()
        }
        @inline(__always) set { payload = .prismatic(newValue) }
    }

    var spherical: SphericalJointSim {
        @inline(__always) get {
            if case .spherical(let value) = payload { return value }
            return SphericalJointSim()
        }
        @inline(__always) set { payload = .spherical(newValue) }
    }

    var wheel: WheelJointSim {
        @inline(__always) get {
            if case .wheel(let value) = payload { return value }
            return WheelJointSim()
        }
        @inline(__always) set { payload = .wheel(newValue) }
    }
    init() {}
}

// MARK: - Body helpers

@inline(__always)
func encodeBodySimIndex(_ body: Body) -> Int32 {
    if body.setIndex == SetType.awakeSet.rawValue {
        return body.localIndex
    }

    if body.setIndex == SetType.staticSet.rawValue {
        return -(body.localIndex + 2)
    }

    return B3.nullIndex
}

@inline(__always)
func isStaticSimIndex(_ encoded: Int32) -> Bool {
    encoded < B3.nullIndex
}

@inline(__always)
func decodeAwakeIndex(_ encoded: Int32) -> Int32 {
    encoded >= 0 ? encoded : B3.nullIndex
}

@inline(__always)
func sleepBodySimIndex(_ encoded: Int32) -> Int32 {
    encoded >= 0 ? B3.nullIndex : encoded
}

/// Make a sweep relative to a base position to keep TOI in float precision far from the origin.
@inline(__always)
func makeRelativeSweep(_ bodySim: BodySim, _ base: Pos) -> Sweep {
    var s = Sweep()
    s.c1 = subPos(bodySim.center0, base)
    s.c2 = subPos(bodySim.center, base)
    s.q1 = bodySim.rotation0
    s.q2 = bodySim.transform.q
    s.localCenter = bodySim.localCenter
    return s
}

// MARK: - World

/// The world owns all bodies, shapes, contacts, joints, and solver sets.
public final class World {
    var worldId: UInt16 = 0
    var generation: UInt16 = 0

    var broadPhase: BroadPhase
    let constraintGraph: ConstraintGraph

    /// Reused broad-phase scratch. These arrays stay alive across steps so pair generation does
    /// not allocate and regrow a fresh list every frame.
    var pairKeyScratch: [UInt64] = []
    var movedSiblingScratch: [Int32] = []
    var crossSeedQueue: [NodePair] = []
    var staticSeedScratch: [NodePair] = []
    var kinematicSeedScratch: [NodePair] = []
    var broadPhasePairStack: [TraversalPair] = []
    var broadPhaseProxyStack: [Int32] = []
    var contactIndexScratch: [Int32] = []

    var bodies: [Body] = []
    var bodyIdPool = IDPool()

    var solverSets: [SolverSet] = []
    var solverSetIdPool = IDPool()

    var islands: [Island] = []
    var islandIdPool = IDPool()

    var shapes: [Shape] = []
    var shapeIdPool = IDPool()

    var fatAABBs: [AABB] = []

    var contacts: ContiguousArray<Contact> = []
    let contactPayloads = ContactPayloadPool()
    var contactIdPool = IDPool()

    var joints: [Joint] = []
    var jointIdPool = IDPool()

    var names = NameCache()

    /// Move events captured during a step.
    var bodyMoveEvents: [BodyMoveEvent] = []

    /// Profiling data for the most recent step.
    public var profile = Profile()

    /// Sensors
    var sensors: [Sensor] = []
    var sensorBeginEvents: [SensorBeginTouchEvent] = []
    var sensorEndEvents = EndEventBuffer<SensorEndTouchEvent>()

    /// Contact touch events
    var contactBeginEvents: [ContactBeginTouchEvent] = []
    var contactEndEvents = EndEventBuffer<ContactEndTouchEvent>()
    var contactHitEvents: [ContactHitEvent] = []
    var jointEvents: [JointEvent] = []

    /// Per-thread task storage.
    var taskContexts: [TaskContext] = [TaskContext()]

    /// Reused serial mesh narrow-phase scratch.
    var meshContactScratch = MeshContactScratch()

    /// Serial CCD query state. Reusing it avoids allocating a class and a sensor-hit array for
    /// every fast body every step.
    let continuousContext = ContinuousContext()

    /// Number of worker threads for the parallel phases (broad-phase pair generation and the
    /// collide pass). The solver itself runs serially by color, as C does. Results are independent
    /// of this value: each worker owns a disjoint slice and a private contact-state bit set, and the
    /// slices are merged in contact id order.
    public var workerCount: Int32 = 1

    /// Solve independent contact batches with four SIMD lanes. Disable to use the equivalent
    /// scalar path. Overflow constraints always remain sequential.
    public var enableWideSolver = true

    /// Per-step solver constraint data.
    var solverContacts = SolverContactData()
    var solverJoints = SolverJointData()

    /// True when any prepared contact has restitution.
    var solveAnyRestitution = false

    /// Reference counted store of shared hull data keyed by content.
    let hullDatabase = HullDatabase()

    public var gravity = Vec3(0, -10, 0)
    public var hitEventThreshold: Float = 1
    public var restitutionThreshold: Float = 1
    public var restitutionIterations: Int32 = 2
    public var enableRestitutionPropagation = false
    public var maxLinearSpeed: Float = 400
    public var isotropicInertiaTolerance: Float = 0.2
    public var contactSpeed: Float = 3
    public var contactHertz: Float = 30
    public var contactDampingRatio: Float = 10
    public var contactRecycleDistance: Float = B3.contactRecycleDistance
    var simdWidth: Int32 = 4
    var satCallCount: Int32 = 0
    var satCacheHitCount: Int32 = 0

    // nil selects the inline default mixer. A custom callback still runs for every update.
    var frictionCallback: FrictionCallback?
    var restitutionCallback: RestitutionCallback?

    @inline(__always)
    func mixFriction(_ a: Float, _ idA: UInt64, _ b: Float, _ idB: UInt64) -> Float {
        if let frictionCallback { return frictionCallback(a, idA, b, idB) }
        return (a * b).squareRoot()
    }

    @inline(__always)
    func mixRestitution(_ a: Float, _ idA: UInt64, _ b: Float, _ idB: UInt64) -> Float {
        if let restitutionCallback { return restitutionCallback(a, idA, b, idB) }
        return maxFloat(a, b)
    }

    var customFilter: CustomFilterCallback?

    /// Optional pre-solve callback. Returning false disables the contact.
    var preSolve: PreSolveCallback?

    /// Set the custom shape filter. Both shapes must enable custom filtering.
    public func setCustomFilter(_ callback: CustomFilterCallback?) {
        customFilter = callback
    }

    /// Set the pre-solve callback. Returning false disables the contact for that step.
    public func setPreSolveCallback(_ callback: PreSolveCallback?) {
        preSolve = callback
    }

    public var enableSleep = true
    public var enableWarmStarting = true
    public var enableContinuous = true
    public var enableSpeculative = true
    var stepIndex: UInt64 = 0
    var inv_h: Float = 0
    var inv_dt: Float = 0
    var splitIslandId: Int32 = B3.nullIndex
    var compoundShapeCount: Int32 = 0

    var locked = false
    var inUse = true

    public init(capacity: Capacity = Capacity()) {
        broadPhase = BroadPhase(capacity: capacity)
        constraintGraph = ConstraintGraph(
            bodyCapacity: maxInt(16, capacity.staticBodyCount + capacity.dynamicBodyCount)
        )

        // Add empty static, disabled, and awake body sets.
        let staticSet = SolverSet()
        solverSets.append(staticSet)
        solverSets[0].setIndex = solverSetIdPool.alloc()

        var disabledSet = SolverSet()
        disabledSet.setIndex = solverSetIdPool.alloc()
        solverSets.append(disabledSet)

        var awakeSet = SolverSet()
        awakeSet.setIndex = solverSetIdPool.alloc()
        solverSets.append(awakeSet)

        bodies.reserveCapacity(Int(maxInt(16, capacity.staticBodyCount + capacity.dynamicBodyCount)))
    }

    // MARK: Element access

    @inline(__always)
    func solverSet(_ index: Int32) -> SolverSet {
        solverSets[Int(index)]
    }

    @inline(__always)
    func bodySim(_ body: Body) -> BodySim {
        solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]
    }

    @inline(__always)
    func bodyState(_ body: Body) -> BodyState? {
        if body.setIndex == SetType.awakeSet.rawValue {
            return solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)]
        }
        return nil
    }

    @inline(__always)
    func mutateBodySim(_ body: Body, _ body_: (inout BodySim) -> Void) {
        var sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]
        body_(&sim)
        solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)] = sim
    }

    @inline(__always)
    func mutateBodyState(_ body: Body, _ body_: (inout BodyState) -> Void) {
        if body.setIndex == SetType.awakeSet.rawValue {
            var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)]
            body_(&state)
            solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)] = state
        }
    }

    // MARK: Ids

    func makeBodyID(_ bodyId: Int32) -> BodyID {
        BodyID(index1: bodyId + 1, world0: worldId, generation: bodies[Int(bodyId)].generation)
    }

    func isValidBody(_ id: BodyID) -> Bool {
        if id.world0 != worldId {
            return false
        }
        if id.index1 < 1 || bodies.count < Int(id.index1) {
            return false
        }
        let body = bodies[Int(id.index1 - 1)]
        if body.setIndex == B3.nullIndex {
            // this was freed
            return false
        }
        b3Assert(body.localIndex != B3.nullIndex)
        return body.generation == id.generation
    }

    func body(_ id: BodyID) -> Body {
        b3Assert(isValidBody(id))
        return bodies[Int(id.index1 - 1)]
    }

    func bodyIndex(_ id: BodyID) -> Int32 {
        id.index1 - 1
    }

    // MARK: Islands

    @discardableResult
    func createIsland(_ setIndex: Int32) -> Int32 {
        b3Assert(setIndex == SetType.awakeSet.rawValue || setIndex >= SetType.firstSleepingSet.rawValue)

        let islandId = islandIdPool.alloc()

        if Int(islandId) == islands.count {
            islands.append(Island())
        } else {
            b3Assert(islands[Int(islandId)].setIndex == B3.nullIndex)
        }

        var island = Island()
        island.setIndex = setIndex
        island.localIndex = Int32(solverSets[Int(setIndex)].islandSims.count)
        island.islandId = islandId
        islands[Int(islandId)] = island

        var islandSim = IslandSim()
        islandSim.islandId = islandId
        solverSets[Int(setIndex)].islandSims.append(islandSim)

        return islandId
    }

    func destroyIsland(_ islandId: Int32) {
        if splitIslandId == islandId {
            splitIslandId = B3.nullIndex
        }

        var island = islands[Int(islandId)]
        let setIndex = island.setIndex
        let localIndex = island.localIndex
        let lastIndex = Int32(solverSets[Int(setIndex)].islandSims.count - 1)
        b3Assert(0 <= localIndex && localIndex <= lastIndex)
        let moveIslandId = solverSets[Int(setIndex)].islandSims[Int(lastIndex)].islandId
        solverSets[Int(setIndex)].islandSims[Int(localIndex)] = solverSets[Int(setIndex)].islandSims[Int(lastIndex)]
        islands[Int(moveIslandId)].localIndex = localIndex
        solverSets[Int(setIndex)].islandSims.removeLast()

        island.bodies.removeAll()
        island.contacts.removeAll()
        island.joints.removeAll()
        island.constraintRemoveCount = 0
        island.localIndex = B3.nullIndex
        island.islandId = B3.nullIndex
        island.setIndex = B3.nullIndex
        islands[Int(islandId)] = island

        islandIdPool.free(islandId)
    }

    // MARK: Body flags

    func syncBodyFlags(_ bodyIndex: Int32) {
        var body = bodies[Int(bodyIndex)]
        var sim = bodySim(body)

        // Preserve the sim only flags.
        sim.flags = (sim.flags & (BodyFlags.isFast.rawValue | BodyFlags.hadTimeOfImpact.rawValue))
            | (body.flags & ~BodyFlags.bodyTransientFlags.rawValue)
        solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)] = sim

        if body.setIndex == SetType.awakeSet.rawValue {
            var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)]
            state.flags = body.flags & ~BodyFlags.bodyTransientFlags.rawValue
            solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)] = state
        }

        body = bodies[Int(bodyIndex)]
    }

    // MARK: Body creation

    /// Create a body. Returns a null id if the world cannot accept it.
    @discardableResult
    public func createBody(_ def: BodyDef) -> BodyID {
        b3Assert(isValidPosition(def.position))
        b3Assert(isValidQuat(def.rotation))
        b3Assert(isValidVec3(def.linearVelocity))
        b3Assert(isValidVec3(def.angularVelocity))
        b3Assert(isValidFloat(def.linearDamping) && def.linearDamping >= 0)
        b3Assert(isValidFloat(def.angularDamping) && def.angularDamping >= 0)
        b3Assert(isValidFloat(def.sleepThreshold) && def.sleepThreshold >= 0)
        b3Assert(isValidFloat(def.safetyFactor) && def.safetyFactor >= 0)
        b3Assert(isValidFloat(def.gravityScale))

        let isAwake = (def.isAwake || def.enableSleep == false) && def.isEnabled

        // determine the solver set
        let setId: Int32
        if def.isEnabled == false {
            setId = SetType.disabledSet.rawValue
        } else if def.type == .staticBody {
            setId = SetType.staticSet.rawValue
        } else if isAwake {
            setId = SetType.awakeSet.rawValue
        } else {
            // new set for a sleeping body in its own island
            setId = solverSetIdPool.alloc()
            if Int(setId) == solverSets.count {
                solverSets.append(SolverSet())
            } else {
                b3Assert(solverSets[Int(setId)].setIndex == B3.nullIndex)
            }

            solverSets[Int(setId)].setIndex = setId
        }

        b3Assert(0 <= setId && Int(setId) < solverSets.count)

        let bodyId = bodyIdPool.alloc()

        var lockFlags: UInt32 = 0
        lockFlags |= def.motionLocks.linearX ? BodyFlags.lockLinearX.rawValue : 0
        lockFlags |= def.motionLocks.linearY ? BodyFlags.lockLinearY.rawValue : 0
        lockFlags |= def.motionLocks.linearZ ? BodyFlags.lockLinearZ.rawValue : 0
        lockFlags |= def.motionLocks.angularX ? BodyFlags.lockAngularX.rawValue : 0
        lockFlags |= def.motionLocks.angularY ? BodyFlags.lockAngularY.rawValue : 0
        lockFlags |= def.motionLocks.angularZ ? BodyFlags.lockAngularZ.rawValue : 0

        var sim = BodySim()
        sim.transform.p = def.position
        sim.transform.q = def.rotation
        sim.center = def.position
        sim.rotation0 = sim.transform.q
        sim.center0 = sim.center
        sim.localCenter = Vec3()
        sim.force = Vec3()
        sim.torque = Vec3()
        sim.invMass = 0
        sim.inertia = Matrix3()
        sim.invInertiaLocal = Matrix3()
        sim.minExtent = B3.huge
        sim.maxExtent = Vec3()
        sim.linearDamping = def.linearDamping
        sim.angularDamping = def.angularDamping
        sim.gravityScale = def.gravityScale
        sim.bodyId = bodyId
        sim.flags = lockFlags
        sim.flags |= def.isBullet ? BodyFlags.isBullet.rawValue : 0
        sim.flags |= def.allowFastRotation ? BodyFlags.allowFastRotation.rawValue : 0
        sim.flags |= def.type == .dynamicBody ? BodyFlags.dynamicFlag.rawValue : 0
        sim.flags |= def.enableSleep ? BodyFlags.enableSleep.rawValue : 0
        sim.flags |= def.enableContactRecycling ? BodyFlags.enableContactRecycling.rawValue : 0

        var set = solverSets[Int(setId)]
        set.bodySims.append(sim)
        let localIndex = Int32(set.bodySims.count - 1)

        if setId == SetType.awakeSet.rawValue {
            var state = BodyState()
            state.linearVelocity = def.linearVelocity
            state.angularVelocity = def.angularVelocity
            state.deltaRotation = Quat.identity
            state.flags = sim.flags
            set.bodyStates.append(state)
        }

        solverSets[Int(setId)] = set

        if Int(bodyId) == bodies.count {
            bodies.append(Body())
        } else {
            b3Assert(bodies[Int(bodyId)].id == B3.nullIndex)
        }

        var body = bodies[Int(bodyId)]
        body.userData = def.userData
        body.setIndex = setId
        body.localIndex = localIndex
        body.generation &+= 1
        body.headShapeId = B3.nullIndex
        body.shapeCount = 0
        body.headChainId = B3.nullIndex
        body.headContactKey = B3.nullIndex
        body.contactCount = 0
        body.headJointKey = B3.nullIndex
        body.jointCount = 0
        body.islandId = B3.nullIndex
        body.islandIndex = B3.nullIndex
        body.bodyMoveIndex = B3.nullIndex
        body.id = bodyId
        body.sleepThreshold = def.sleepThreshold
        body.sleepTime = 0
        body.sleepVelocity = 0
        body.safetyFactor = def.safetyFactor
        body.mass = 0
        body.nameId = names.add(def.name)
        body.type = def.type
        body.flags = sim.flags
        bodies[Int(bodyId)] = body

        // dynamic and kinematic bodies that are enabled need an island
        if setId >= SetType.awakeSet.rawValue {
            createIslandForBody(setId, bodyId)
        }

        return makeBodyID(bodyId)
    }

    func createIslandForBody(_ setIndex: Int32, _ bodyId: Int32) {
        var body = bodies[Int(bodyId)]
        b3Assert(body.islandId == B3.nullIndex)
        b3Assert(setIndex != SetType.disabledSet.rawValue)

        let islandId = createIsland(setIndex)
        islands[Int(islandId)].bodies.append(bodyId)
        body.islandId = islandId
        body.islandIndex = 0
        bodies[Int(bodyId)] = body
    }

    func removeBodyFromIsland(_ bodyId: Int32) {
        var body = bodies[Int(bodyId)]

        if body.islandId == B3.nullIndex {
            b3Assert(body.islandIndex == B3.nullIndex)
            return
        }

        let islandId = body.islandId
        do {
            let localIndex = Int(body.islandIndex)
            let islandBodyCount = islands[Int(islandId)].bodies.count
            let movedBodyId = islands[Int(islandId)].bodies[islandBodyCount - 1]
            islands[Int(islandId)].bodies[localIndex] = movedBodyId
            b3Validate(bodies[Int(movedBodyId)].islandIndex == Int32(islandBodyCount - 1))
            bodies[Int(movedBodyId)].islandIndex = Int32(localIndex)
            islands[Int(islandId)].bodies.removeLast()
        }

        if islands[Int(islandId)].bodies.count == 0 {
            b3Assert(islands[Int(islandId)].contacts.count == 0)
            b3Assert(islands[Int(islandId)].joints.count == 0)
            destroyIsland(islandId)
        }

        body = bodies[Int(bodyId)]
        body.islandId = B3.nullIndex
        body.islandIndex = B3.nullIndex
        bodies[Int(bodyId)] = body
    }

    // MARK: Body destruction

    public func destroyBody(_ bodyId: BodyID) {
        b3Assert(isValidBody(bodyId))

        let id = bodyIndex(bodyId)

        var jointKey = bodies[Int(id)].headJointKey
        while jointKey != B3.nullIndex {
            let jointId = jointKey >> 1
            let edgeIndex = Int(jointKey & 1)
            jointKey = joints[Int(jointId)].edges[edgeIndex].nextKey
            destroyJoint(jointId, true)
        }
        destroyBodyContacts(id, true)
        // Destroy the attached shapes and their broad-phase proxies.
        var shapeId = bodies[Int(id)].headShapeId
        while shapeId != B3.nullIndex {
            var shape = shapes[Int(shapeId)]

            if shape.type == .compoundShape {
                b3Assert(compoundShapeCount > 0)
                compoundShapeCount -= 1
            }

            if shape.sensorIndex != B3.nullIndex { destroySensor(shapeId) }
            destroyShapeProxy(&shape, broadPhase)
            destroyShapeAllocations(self, &shape)

            shapes[Int(shapeId)] = shape

            // Return shape to free list.
            shapeIdPool.free(shapeId)
            shapes[Int(shapeId)].id = B3.nullIndex

            shapeId = shape.nextShapeId
        }

        removeBodyFromIsland(id)

        // Remove body sim from solver set that owns it
        var body = bodies[Int(id)]
        var set = solverSets[Int(body.setIndex)]
        let movedIndex = removeSwapArray(&set.bodySims, Int(body.localIndex))
        if movedIndex != B3.nullIndex {
            // Fix moved body index
            let movedSim = set.bodySims[Int(body.localIndex)]
            let movedId = movedSim.bodyId
            b3Assert(bodies[Int(movedId)].localIndex == movedIndex)
            bodies[Int(movedId)].localIndex = body.localIndex
            refreshBodyContactIndices(movedId)
        }

        // Remove body state from awake set
        if body.setIndex == SetType.awakeSet.rawValue {
            let result = removeSwapArray(&set.bodyStates, Int(body.localIndex))
            b3Assert(result == movedIndex)
        }
        solverSets[Int(body.setIndex)] = set
        if set.setIndex >= SetType.firstSleepingSet.rawValue && set.bodySims.count == 0 {
            // Commit the removal before destroying the now-orphaned solver set.
            destroySolverSet(set.setIndex)
        }

        // Free body and id (preserve body revision)
        bodyIdPool.free(body.id)

        body.setIndex = B3.nullIndex
        body.localIndex = B3.nullIndex
        body.id = B3.nullIndex
        bodies[Int(id)] = body
    }

    func destroySolverSet(_ setIndex: Int32) {
        var set = solverSets[Int(setIndex)]
        set.bodySims.removeAll()
        set.bodyStates.removeAll()
        set.contactIndices.removeAll()
        set.jointSims.removeAll()
        set.islandSims.removeAll()

        solverSetIdPool.free(setIndex)
        set.setIndex = B3.nullIndex
        solverSets[Int(setIndex)] = set
    }

    // MARK: Body state

    func isBodyAwake(_ body: Body) -> Bool {
        body.setIndex == SetType.awakeSet.rawValue
    }

    @discardableResult
    func wakeBody(_ bodyId: Int32) -> Bool {
        if bodies[Int(bodyId)].setIndex >= SetType.firstSleepingSet.rawValue {
            wakeSolverSet(bodies[Int(bodyId)].setIndex)
            return true
        }

        return false
    }

}

/// Swap-remove an element from a Swift array. Returns the moved index or `B3.nullIndex`.
@discardableResult
func removeSwapArray<T>(_ array: inout [T], _ index: Int) -> Int32 {
    b3Assert(index >= 0 && index < array.count)
    let last = array.count - 1
    if index != last {
        array[index] = array[last]
    }
    array.removeLast()
    if index != last {
        return Int32(last)
    }
    return B3.nullIndex
}
