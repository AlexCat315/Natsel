// World state snapshots for rollback, replays, and lockstep resynchronization (Jolt's
// StateRecorder with all state flags). A snapshot holds every piece of state that influences
// later steps: bodies and solver sets, shapes and broad-phase trees, contacts with their manifolds
// and warm starting impulses, the constraint graph, islands, joints and their side stores,
// sensors, vehicles, soft bodies, id pools, pending events, and the simulation settings. Restoring
// it and stepping gives the same bits as stepping from the moment it was taken, even after bodies,
// shapes, or joints were created or destroyed in between.
//
// Not part of the state: shape geometry objects (hulls, meshes, height fields, compounds and
// constraint paths are shared by reference and must not be edited in place), user callbacks,
// user data pointers' targets, `workerCount` (results do not depend on it), and character
// controllers, which live outside the world and have their own `saveState`.

/// Clones of manually managed containers. The copies own new storage.
extension DynamicArray {
    func cloned() -> DynamicArray {
        var copy = DynamicArray(capacity: capacity)
        if count > 0, let data, let target = copy.data {
            target.update(from: data, count: Int(count))
        }
        copy.count = count
        return copy
    }
}

extension IDPool {
    func cloned() -> IDPool {
        var copy = self
        copy.freeArray = freeArray.cloned()
        return copy
    }
}

extension BitSet {
    func cloned() -> BitSet {
        var copy = BitSet()
        guard blockCapacity > 0, let bits else { return copy }
        copy = BitSet(bitCapacity: blockCapacity * 64)
        copy.blockCount = blockCount
        copy.bits?.update(from: bits, count: Int(blockCapacity))
        return copy
    }
}

extension HashSet {
    func cloned() -> HashSet {
        var copy = HashSet()
        guard capacity > 0, let items else { return copy }
        copy.capacity = capacity
        copy.count = count
        copy.items = b3AllocTyped(SetItem.self, Int32(capacity))
        copy.items?.initialize(from: items, count: Int(capacity))
        return copy
    }
}

/// The persistent part of a dynamic tree. Rebuild scratch is left out.
struct DynamicTreeState {
    var nodes: [TreeNode]
    var parents: [Int32]
    var proxies: [TreeProxy]
    var nodeEnd: Int32
    var nodeCapacity: Int32
    var pairFreeList: Int32
    var proxyCount: Int32
    var proxyCapacity: Int32
    var proxyFreeList: Int32
    var dfsOrdered: Bool
}

extension DynamicTree {
    var state: DynamicTreeState {
        DynamicTreeState(
            nodes: nodes, parents: parents, proxies: proxies, nodeEnd: nodeEnd, nodeCapacity: nodeCapacity,
            pairFreeList: pairFreeList, proxyCount: proxyCount, proxyCapacity: proxyCapacity,
            proxyFreeList: proxyFreeList, dfsOrdered: dfsOrdered)
    }

    func restore(_ state: DynamicTreeState) {
        nodes = state.nodes
        parents = state.parents
        proxies = state.proxies
        nodeEnd = state.nodeEnd
        nodeCapacity = state.nodeCapacity
        pairFreeList = state.pairFreeList
        proxyCount = state.proxyCount
        proxyCapacity = state.proxyCapacity
        proxyFreeList = state.proxyFreeList
        dfsOrdered = state.dfsOrdered
    }
}

/// A saved world state. Holds its own copies, so it can be restored any number of times. Only
/// the world that saved it can restore it.
public final class WorldState {
    fileprivate weak var world: World?
    /// Step count of the world when the state was saved.
    public let stepIndex: UInt64

    // Bodies, shapes, and solver sets.
    fileprivate var bodies: [Body] = []
    fileprivate var bodyIdPool = IDPool()
    fileprivate var solverSets: [SolverSet] = []
    fileprivate var solverSetIdPool = IDPool()
    fileprivate var islands: [Island] = []
    fileprivate var islandIdPool = IDPool()
    fileprivate var shapes: [Shape] = []
    fileprivate var shapeIdPool = IDPool()
    fileprivate var fatAABBs: [AABB] = []
    fileprivate var hullDatabase: Any = ()
    fileprivate var names = NameCache()

    // Broad phase.
    fileprivate var trees: [DynamicTreeState] = []
    fileprivate var movedSiblings: [Int32] = []
    fileprivate var pairSet = HashSet()

    // Contacts and the constraint graph.
    fileprivate var contacts: ContiguousArray<Contact> = []
    fileprivate var payloads: [(Int, ContactPayload)] = []
    fileprivate var contactIdPool = IDPool()
    fileprivate var colors: [GraphColor] = []

    // Joints.
    fileprivate var joints: [Joint] = []
    fileprivate var jointIdPool = IDPool()
    fileprivate var sixDOF: [SixDOFJointData] = []
    fileprivate var sixDOFPool = IDPool()
    fileprivate var pathJoints: [PathJointData] = []
    fileprivate var pathJointPaths: [ConstraintPath?] = []
    fileprivate var pathJointPool = IDPool()

    // Other systems.
    fileprivate var sensors: [Sensor] = []
    fileprivate var vehicles: [VehicleRecord] = []
    fileprivate var vehicleIdPool = IDPool()
    fileprivate var softBodies: [SoftBodyRecord] = []
    fileprivate var softBodyIdPool = IDPool()

    // Events of the last step and end events queued since.
    fileprivate var bodyMoveEvents: [BodyMoveEvent] = []
    fileprivate var sensorBeginEvents: [SensorBeginTouchEvent] = []
    fileprivate var sensorEndEvents = EndEventBuffer<SensorEndTouchEvent>()
    fileprivate var contactBeginEvents: [ContactBeginTouchEvent] = []
    fileprivate var contactEndEvents = EndEventBuffer<ContactEndTouchEvent>()
    fileprivate var contactHitEvents: [ContactHitEvent] = []
    fileprivate var jointEvents: [JointEvent] = []

    // Step scalars.
    fileprivate var inv_h: Float = 0
    fileprivate var inv_dt: Float = 0
    fileprivate var splitIslandId: Int32 = B3.nullIndex
    fileprivate var compoundShapeCount: Int32 = 0

    // Settings.
    fileprivate var settings = WorldSettingsState()

    fileprivate init(stepIndex: UInt64) {
        self.stepIndex = stepIndex
    }

    deinit {
        bodyIdPool.destroy()
        solverSetIdPool.destroy()
        islandIdPool.destroy()
        shapeIdPool.destroy()
        contactIdPool.destroy()
        jointIdPool.destroy()
        sixDOFPool.destroy()
        pathJointPool.destroy()
        vehicleIdPool.destroy()
        softBodyIdPool.destroy()
        pairSet.destroy()
        for i in 0..<colors.count { colors[i].bodySet.destroy() }
    }

    /// Approximate heap size of the snapshot's own copies, in bytes.
    public var byteCount: Int {
        var total = bodies.count * MemoryLayout<Body>.stride + shapes.count * MemoryLayout<Shape>.stride
        total += fatAABBs.count * MemoryLayout<AABB>.stride + contacts.count * MemoryLayout<Contact>.stride
        total += payloads.count * MemoryLayout<ContactPayload>.stride + joints.count * MemoryLayout<Joint>.stride
        for set in solverSets {
            total += set.bodySims.count * MemoryLayout<BodySim>.stride
            total += set.bodyStates.count * MemoryLayout<BodyState>.stride
            total += set.jointSims.count * MemoryLayout<JointSim>.stride
            total += set.contactIndices.count * 4 + set.islandSims.count * MemoryLayout<IslandSim>.stride
        }
        for tree in trees {
            total += tree.nodes.count * MemoryLayout<TreeNode>.stride + tree.parents.count * 4
            total += tree.proxies.count * MemoryLayout<TreeProxy>.stride
        }
        for color in colors {
            total += color.jointSims.count * MemoryLayout<JointSim>.stride
            total += color.contacts.count * MemoryLayout<ContactSpec>.stride + color.convexContacts.count * 4
        }
        for island in islands {
            total += island.bodies.count * 4 + island.contacts.count * MemoryLayout<ContactLink>.stride
            total += island.joints.count * MemoryLayout<JointLink>.stride
        }
        return total
    }
}

/// Settings that change results, saved with the state.
struct WorldSettingsState {
    var gravity = Vec3()
    var hitEventThreshold: Float = 0
    var restitutionThreshold: Float = 0
    var restitutionIterations: Int32 = 0
    var enableRestitutionPropagation = false
    var maxLinearSpeed: Float = 0
    var isotropicInertiaTolerance: Float = 0
    var contactSpeed: Float = 0
    var contactHertz: Float = 0
    var contactDampingRatio: Float = 0
    var contactRecycleDistance: Float = 0
    var enableSleep = true
    var enableWarmStarting = true
    var enableContinuous = true
    var enableSpeculative = true
    var enableWideSolver = true
}

extension World {
    /// Save everything that influences later steps. Not allowed during a step or a callback.
    public func saveState() -> WorldState {
        b3Assert(locked == false)
        let state = WorldState(stepIndex: stepIndex)
        state.world = self

        state.bodies = bodies
        state.bodyIdPool = bodyIdPool.cloned()
        state.solverSets = solverSets
        state.solverSetIdPool = solverSetIdPool.cloned()
        state.islands = islands
        state.islandIdPool = islandIdPool.cloned()
        state.shapes = shapes
        state.shapeIdPool = shapeIdPool.cloned()
        state.fatAABBs = fatAABBs
        state.hullDatabase = hullDatabase.snapshot
        state.names = names

        state.trees = broadPhase.trees.map { $0.state }
        state.movedSiblings = broadPhase.movedSiblings
        state.pairSet = broadPhase.pairSet.cloned()

        state.contacts = contacts
        state.payloads.reserveCapacity(contacts.count)
        for (index, contact) in contacts.enumerated() where contact.contactId != B3.nullIndex {
            if let payload = contact.payload { state.payloads.append((index, payload.pointee)) }
        }
        state.contactIdPool = contactIdPool.cloned()
        state.colors = constraintGraph.colors
        for i in 0..<state.colors.count { state.colors[i].bodySet = constraintGraph.colors[i].bodySet.cloned() }

        state.joints = joints
        state.jointIdPool = jointIdPool.cloned()
        state.sixDOF = sixDOFStore.elements
        state.sixDOFPool = sixDOFPool.cloned()
        state.pathJoints = pathJointStore.elements
        state.pathJointPaths = pathJointPaths
        state.pathJointPool = pathJointPool.cloned()

        state.sensors = sensors
        state.vehicles = vehicles
        state.vehicleIdPool = vehicleIdPool.cloned()
        state.softBodies = softBodies
        state.softBodyIdPool = softBodyIdPool.cloned()

        state.bodyMoveEvents = bodyMoveEvents
        state.sensorBeginEvents = sensorBeginEvents
        state.sensorEndEvents = sensorEndEvents
        state.contactBeginEvents = contactBeginEvents
        state.contactEndEvents = contactEndEvents
        state.contactHitEvents = contactHitEvents
        state.jointEvents = jointEvents

        state.inv_h = inv_h
        state.inv_dt = inv_dt
        state.splitIslandId = splitIslandId
        state.compoundShapeCount = compoundShapeCount

        var settings = WorldSettingsState()
        settings.gravity = gravity
        settings.hitEventThreshold = hitEventThreshold
        settings.restitutionThreshold = restitutionThreshold
        settings.restitutionIterations = restitutionIterations
        settings.enableRestitutionPropagation = enableRestitutionPropagation
        settings.maxLinearSpeed = maxLinearSpeed
        settings.isotropicInertiaTolerance = isotropicInertiaTolerance
        settings.contactSpeed = contactSpeed
        settings.contactHertz = contactHertz
        settings.contactDampingRatio = contactDampingRatio
        settings.contactRecycleDistance = contactRecycleDistance
        settings.enableSleep = enableSleep
        settings.enableWarmStarting = enableWarmStarting
        settings.enableContinuous = enableContinuous
        settings.enableSpeculative = enableSpeculative
        settings.enableWideSolver = enableWideSolver
        state.settings = settings
        return state
    }

    /// Return the world to a saved state. Returns false when the state belongs to another world or
    /// the world is locked. Ids created after the save become invalid; ids destroyed after the save
    /// are valid again.
    @discardableResult
    public func restoreState(_ state: WorldState) -> Bool {
        guard state.world === self, locked == false else { return false }

        bodies = state.bodies
        replacePool(&bodyIdPool, state.bodyIdPool)
        solverSets = state.solverSets
        replacePool(&solverSetIdPool, state.solverSetIdPool)
        islands = state.islands
        replacePool(&islandIdPool, state.islandIdPool)
        shapes = state.shapes
        replacePool(&shapeIdPool, state.shapeIdPool)
        fatAABBs = state.fatAABBs
        hullDatabase.restore(state.hullDatabase)
        names = state.names

        for (tree, saved) in zip(broadPhase.trees, state.trees) { tree.restore(saved) }
        broadPhase.movedSiblings = state.movedSiblings
        broadPhase.pairSet.destroy()
        broadPhase.pairSet = state.pairSet.cloned()

        contacts = state.contacts
        // Payload slots are keyed by contact id and never move, so the saved pointers are valid.
        for (index, payload) in state.payloads {
            contacts[index].payload?.pointee = payload
        }
        replacePool(&contactIdPool, state.contactIdPool)
        for i in 0..<constraintGraph.colors.count { constraintGraph.colors[i].bodySet.destroy() }
        constraintGraph.colors = state.colors
        for i in 0..<state.colors.count { constraintGraph.colors[i].bodySet = state.colors[i].bodySet.cloned() }

        joints = state.joints
        replacePool(&jointIdPool, state.jointIdPool)
        sixDOFStore.replace(with: state.sixDOF)
        replacePool(&sixDOFPool, state.sixDOFPool)
        pathJointStore.replace(with: state.pathJoints)
        pathJointPaths = state.pathJointPaths
        replacePool(&pathJointPool, state.pathJointPool)

        sensors = state.sensors
        vehicles = state.vehicles
        replacePool(&vehicleIdPool, state.vehicleIdPool)
        softBodies = state.softBodies
        replacePool(&softBodyIdPool, state.softBodyIdPool)

        bodyMoveEvents = state.bodyMoveEvents
        sensorBeginEvents = state.sensorBeginEvents
        sensorEndEvents = state.sensorEndEvents
        contactBeginEvents = state.contactBeginEvents
        contactEndEvents = state.contactEndEvents
        contactHitEvents = state.contactHitEvents
        jointEvents = state.jointEvents

        stepIndex = state.stepIndex
        inv_h = state.inv_h
        inv_dt = state.inv_dt
        splitIslandId = state.splitIslandId
        compoundShapeCount = state.compoundShapeCount

        let settings = state.settings
        gravity = settings.gravity
        hitEventThreshold = settings.hitEventThreshold
        restitutionThreshold = settings.restitutionThreshold
        restitutionIterations = settings.restitutionIterations
        enableRestitutionPropagation = settings.enableRestitutionPropagation
        maxLinearSpeed = settings.maxLinearSpeed
        isotropicInertiaTolerance = settings.isotropicInertiaTolerance
        contactSpeed = settings.contactSpeed
        contactHertz = settings.contactHertz
        contactDampingRatio = settings.contactDampingRatio
        contactRecycleDistance = settings.contactRecycleDistance
        enableSleep = settings.enableSleep
        enableWarmStarting = settings.enableWarmStarting
        enableContinuous = settings.enableContinuous
        enableSpeculative = settings.enableSpeculative
        enableWideSolver = settings.enableWideSolver
        return true
    }

    private func replacePool(_ pool: inout IDPool, _ saved: IDPool) {
        pool.destroy()
        pool = saved.cloned()
    }
}
