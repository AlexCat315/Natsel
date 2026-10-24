// Soft body instances: creation, state access, tuning, and sleeping.

/// Soft body id references a soft body instance. This should be treated as an opaque handle.
public struct SoftBodyID: Sendable, Hashable, BitwiseCopyable {
    var index1: Int32 = 0
    var world0: UInt16 = 0
    var generation: UInt16 = 0

    public init() {}

    init(index1: Int32, world0: UInt16, generation: UInt16) {
        self.index1 = index1
        self.world0 = world0
        self.generation = generation
    }

    public static let null = SoftBodyID()

    public var isNull: Bool { index1 == 0 }
}

/// Used to create a soft body.
public struct SoftBodyDef {
    public var settings = SoftBodySettings()

    /// Placement of the settings' rest vertices.
    public var position = Pos()
    public var rotation = Quat.identity

    /// XPBD substeps per world step. More substeps make the body stiffer.
    public var iterationCount: Int32 = 5

    /// Linear velocity damping: dv/dt = -c * v.
    public var linearDamping: Float = 0.1

    /// Vertices never move faster than this.
    public var maxLinearVelocity: Float = 500 * lengthUnitsPerMeter

    public var friction: Float = 0.2
    public var restitution: Float = 0

    /// n * R * T for closed surfaces (N m). Positive values inflate the body.
    public var pressure: Float = 0

    public var gravityScale: Float = 1

    /// Collision radius of each vertex.
    public var vertexRadius: Float = 0

    /// Collision filter against rigid shapes.
    public var filter = Filter.defaultFilter

    /// Interaction with rigid bodies. With `twoWayCoupling` off, rigid bodies push the soft body but
    /// do not feel it.
    public var collideWithRigidBodies = true
    public var twoWayCoupling = true

    public var enableSleep = true

    /// The soft body falls asleep when every vertex is slower than this for `timeToSleep`.
    public var sleepThreshold: Float = 0.03 * lengthUnitsPerMeter

    /// Ray casts hit both sides of the faces.
    public var facesDoubleSided = false

    public var userData: UnsafeMutableRawPointer?

    public init() {}

    public init(settings: SoftBodySettings, position: Pos = Pos(), rotation: Quat = .identity) {
        self.settings = settings
        self.position = position
        self.rotation = rotation
    }
}

/// A rigid shape touched by a soft body during the last step.
public struct SoftBodyContact: Sendable, Equatable {
    public var shapeId = ShapeID()
    public var bodyId = BodyID()
    /// Number of vertices touching the shape.
    public var vertexCount: Int32 = 0

    public init() {}
}

/// Ray hit on a soft body surface.
public struct SoftBodyRayResult: Sendable {
    public var softBodyId = SoftBodyID()
    public var point = Pos()
    public var normal = Vec3()
    public var fraction: Float = 1
    public var faceIndex: Int32 = -1
    public var hit = false

    public init() {}
}

/// A rigid shape near the soft body, with the body's velocity tracked through the iterations so
/// later vertices see the impulses applied by earlier ones.
struct SoftBodyCollider {
    var shapeIndex: Int32 = 0
    var bodyIndex: Int32 = 0
    var awakeIndex: Int32 = B3.nullIndex
    var isDynamic = false
    var invMass: Float = 0
    var invInertia = Matrix3()
    var center = Pos()
    var linearVelocity = Vec3()
    var angularVelocity = Vec3()
    var originalLinearVelocity = Vec3()
    var originalAngularVelocity = Vec3()
    var friction: Float = 0
    var restitution: Float = 0
    var updated = false
    var contactCount: Int32 = 0

    @inline(__always)
    func pointVelocity(_ r: Vec3) -> Vec3 {
        linearVelocity + cross(angularVelocity, r)
    }
}

struct SoftBodyRecord {
    var id: Int32 = B3.nullIndex
    var generation: UInt16 = 0
    var userData: UnsafeMutableRawPointer?

    var settings = SoftBodySettings()

    // Particle state, structure of arrays.
    var positions: [Vec3] = []
    var previousPositions: [Vec3] = []
    var velocities: [Vec3] = []
    var invMasses: [Float] = []

    // Collision state per vertex.
    var planeNormals: [Vec3] = []
    var planeOffsets: [Float] = []
    var colliderIndices: [Int32] = []
    var hasContact: [Bool] = []
    var colliders: [SoftBodyCollider] = []

    // Four wide constraint batches built from the settings.
    var edgeBatches: [EdgeBatch] = []
    var bendBatches: [BendBatch] = []
    var volumeBatches: [VolumeBatch] = []

    // Skinning
    var jointTransforms: [Transform] = []
    var skinnedPositions: [Vec3] = []
    var skinnedNormals: [Vec3] = []
    var hasSkinPose = false

    // Tuning
    var iterationCount: Int32 = 5
    var linearDamping: Float = 0.1
    var maxLinearVelocity: Float = 500
    var friction: Float = 0.2
    var restitution: Float = 0
    var pressure: Float = 0
    var gravityScale: Float = 1
    var vertexRadius: Float = 0
    var filter = Filter.defaultFilter
    var collideWithRigidBodies = true
    var twoWayCoupling = true
    var enableSleep = true
    var sleepThreshold: Float = 0.03
    var facesDoubleSided = false

    // State
    var awake = true
    var sleepTime: Float = 0
    var bounds = AABB()
    var volume: Float = 0

    init() {}

    init(_ def: SoftBodyDef) {
        var settings = def.settings
        if settings.patches.isEmpty { settings.buildPatches() }
        self.settings = settings
        userData = def.userData

        let transform = Transform(def.position, normalizeQuat(def.rotation))
        let count = settings.vertices.count
        positions = settings.vertices.map { transformPoint(transform, $0.position) }
        previousPositions = positions
        velocities = settings.vertices.map { rotateVector(transform.q, $0.velocity) }
        invMasses = settings.vertices.map { $0.invMass }
        planeNormals = [Vec3](repeating: Vec3(), count: count)
        planeOffsets = [Float](repeating: 0, count: count)
        colliderIndices = [Int32](repeating: B3.nullIndex, count: count)
        hasContact = [Bool](repeating: false, count: count)
        if settings.skinned.isEmpty == false {
            skinnedPositions = [Vec3](repeating: Vec3(), count: settings.skinned.count)
            skinnedNormals = [Vec3](repeating: Vec3(0, 1, 0), count: settings.skinned.count)
        }

        iterationCount = max(1, def.iterationCount)
        linearDamping = max(0, def.linearDamping)
        maxLinearVelocity = def.maxLinearVelocity
        friction = def.friction
        restitution = def.restitution
        pressure = def.pressure
        gravityScale = def.gravityScale
        vertexRadius = max(0, def.vertexRadius)
        filter = def.filter
        collideWithRigidBodies = def.collideWithRigidBodies
        twoWayCoupling = def.twoWayCoupling
        enableSleep = def.enableSleep
        sleepThreshold = def.sleepThreshold
        facesDoubleSided = def.facesDoubleSided
        buildBatches()
        updateBounds()
    }

    mutating func updateBounds() {
        guard let first = positions.first else {
            bounds = AABB()
            return
        }
        var lower = first
        var upper = first
        positions.withUnsafeBufferPointer { p in
            for i in 1..<p.count {
                lower = minVec3(lower, p[i])
                upper = maxVec3(upper, p[i])
            }
        }
        let r = Vec3(vertexRadius, vertexRadius, vertexRadius)
        bounds = AABB(lower - r, upper + r)
    }

    var vertexCount: Int { positions.count }
}

extension World {
    // MARK: Creation

    /// Create a soft body. Returns a null id when the settings are invalid.
    @discardableResult
    public func createSoftBody(_ def: SoftBodyDef) -> SoftBodyID {
        b3Assert(locked == false)
        guard def.settings.validate() == nil, isValidPosition(def.position), isValidQuat(def.rotation) else {
            return SoftBodyID()
        }

        var record = SoftBodyRecord(def)
        let id = softBodyIdPool.alloc()
        if Int(id) == softBodies.count {
            softBodies.append(record)
        } else {
            record.generation = softBodies[Int(id)].generation &+ 1
            softBodies[Int(id)] = record
        }
        softBodies[Int(id)].id = id
        return SoftBodyID(index1: id + 1, world0: worldId, generation: softBodies[Int(id)].generation)
    }

    public func destroySoftBody(_ softBodyId: SoftBodyID) {
        guard let index = softBodyIndex(softBodyId) else { return }
        let generation = softBodies[index].generation
        softBodies[index] = SoftBodyRecord()
        softBodies[index].generation = generation
        softBodyIdPool.free(Int32(index))
    }

    public func softBodyIsValid(_ softBodyId: SoftBodyID) -> Bool {
        guard softBodyId.world0 == worldId, softBodyId.index1 >= 1, Int(softBodyId.index1) <= softBodies.count
        else { return false }
        let record = softBodies[Int(softBodyId.index1 - 1)]
        return record.id != B3.nullIndex && record.generation == softBodyId.generation
    }

    public var softBodyCount: Int32 { softBodyIdPool.idCount }

    @inline(__always)
    func softBodyIndex(_ softBodyId: SoftBodyID) -> Int? {
        softBodyIsValid(softBodyId) ? Int(softBodyId.index1 - 1) : nil
    }

    // MARK: State

    public func softBodyVertexCount(_ softBodyId: SoftBodyID) -> Int32 {
        softBodyIndex(softBodyId).map { Int32(softBodies[$0].vertexCount) } ?? 0
    }

    /// Current vertex positions, in the order of the (optimized) settings.
    public func softBodyVertexPositions(_ softBodyId: SoftBodyID) -> [Pos] {
        softBodyIndex(softBodyId).map { softBodies[$0].positions } ?? []
    }

    /// Borrow the vertex positions without copying, for example to upload a render mesh.
    public func withSoftBodyVertexPositions<R>(_ softBodyId: SoftBodyID, _ body: (UnsafeBufferPointer<Pos>) -> R) -> R? {
        guard let index = softBodyIndex(softBodyId) else { return nil }
        return softBodies[index].positions.withUnsafeBufferPointer(body)
    }

    public func softBodyVertexVelocities(_ softBodyId: SoftBodyID) -> [Vec3] {
        softBodyIndex(softBodyId).map { softBodies[$0].velocities } ?? []
    }

    public func softBodyVertexPosition(_ softBodyId: SoftBodyID, _ vertex: Int32) -> Pos? {
        guard let i = softBodyIndex(softBodyId), vertex >= 0, Int(vertex) < softBodies[i].vertexCount else { return nil }
        return softBodies[i].positions[Int(vertex)]
    }

    /// Move a vertex. Kinematic vertices (inverse mass zero) are animated this way.
    public func setSoftBodyVertexPosition(_ softBodyId: SoftBodyID, _ vertex: Int32, _ position: Pos) {
        guard let i = softBodyIndex(softBodyId), vertex >= 0, Int(vertex) < softBodies[i].vertexCount,
            isValidPosition(position)
        else { return }
        softBodies[i].positions[Int(vertex)] = position
        softBodies[i].previousPositions[Int(vertex)] = position
        wakeSoftBody(i)
    }

    public func setSoftBodyVertexVelocity(_ softBodyId: SoftBodyID, _ vertex: Int32, _ velocity: Vec3) {
        guard let i = softBodyIndex(softBodyId), vertex >= 0, Int(vertex) < softBodies[i].vertexCount,
            isValidVec3(velocity)
        else { return }
        softBodies[i].velocities[Int(vertex)] = velocity
        wakeSoftBody(i)
    }

    /// Pin (zero) or release a vertex.
    public func setSoftBodyVertexInverseMass(_ softBodyId: SoftBodyID, _ vertex: Int32, _ invMass: Float) {
        guard let i = softBodyIndex(softBodyId), vertex >= 0, Int(vertex) < softBodies[i].vertexCount,
            invMass >= 0
        else { return }
        softBodies[i].invMasses[Int(vertex)] = invMass
        wakeSoftBody(i)
    }

    /// Add a velocity to every dynamic vertex.
    public func addSoftBodyVelocity(_ softBodyId: SoftBodyID, _ velocity: Vec3) {
        guard let i = softBodyIndex(softBodyId), isValidVec3(velocity) else { return }
        for v in softBodies[i].velocities.indices where softBodies[i].invMasses[v] > 0 {
            softBodies[i].velocities[v] += velocity
        }
        wakeSoftBody(i)
    }

    /// Apply a linear impulse distributed by mass, so the change of momentum equals `impulse`.
    public func applySoftBodyLinearImpulse(_ softBodyId: SoftBodyID, _ impulse: Vec3) {
        guard softBodyIsValid(softBodyId), isValidVec3(impulse) else { return }
        let mass = softBodyMass(softBodyId)
        guard mass > 0 else { return }
        addSoftBodyVelocity(softBodyId, (1 / mass) * impulse)
    }

    /// Teleport the whole soft body by a rigid transform about its center of mass.
    public func setSoftBodyTransform(_ softBodyId: SoftBodyID, _ position: Pos, _ rotation: Quat) {
        guard let i = softBodyIndex(softBodyId), isValidPosition(position), isValidQuat(rotation) else { return }
        let center = softBodyCenterOfMass(softBodyId)
        let q = normalizeQuat(rotation)
        for v in softBodies[i].positions.indices {
            let p = position + rotateVector(q, softBodies[i].positions[v] - center)
            softBodies[i].positions[v] = p
            softBodies[i].previousPositions[v] = p
            softBodies[i].velocities[v] = rotateVector(q, softBodies[i].velocities[v])
        }
        softBodies[i].updateBounds()
        wakeSoftBody(i)
    }

    /// Total mass of the dynamic vertices.
    public func softBodyMass(_ softBodyId: SoftBodyID) -> Float {
        guard let i = softBodyIndex(softBodyId) else { return 0 }
        return softBodies[i].invMasses.reduce(0) { $0 + ($1 > 0 ? 1 / $1 : 0) }
    }

    /// Mass weighted center of the dynamic vertices, or the vertex average when all are kinematic.
    public func softBodyCenterOfMass(_ softBodyId: SoftBodyID) -> Pos {
        guard let i = softBodyIndex(softBodyId) else { return Pos() }
        let record = softBodies[i]
        var sum = Vec3()
        var mass: Float = 0
        for v in record.positions.indices where record.invMasses[v] > 0 {
            let m = 1 / record.invMasses[v]
            sum = mulAdd(sum, m, record.positions[v])
            mass += m
        }
        if mass > 0 { return (1 / mass) * sum }
        for p in record.positions { sum += p }
        return record.positions.isEmpty ? Pos() : (1 / Float(record.positions.count)) * sum
    }

    public func softBodyLinearVelocity(_ softBodyId: SoftBodyID) -> Vec3 {
        guard let i = softBodyIndex(softBodyId) else { return Vec3() }
        let record = softBodies[i]
        var sum = Vec3()
        var mass: Float = 0
        for v in record.velocities.indices where record.invMasses[v] > 0 {
            let m = 1 / record.invMasses[v]
            sum = mulAdd(sum, m, record.velocities[v])
            mass += m
        }
        return mass > 0 ? (1 / mass) * sum : Vec3()
    }

    public func softBodyBounds(_ softBodyId: SoftBodyID) -> AABB {
        softBodyIndex(softBodyId).map { softBodies[$0].bounds } ?? AABB()
    }

    /// Enclosed volume of a closed surface after the last step.
    public func softBodyVolume(_ softBodyId: SoftBodyID) -> Float {
        guard let i = softBodyIndex(softBodyId) else { return 0 }
        return softBodies[i].positions.withUnsafeBufferPointer { closedVolume(softBodies[i].settings.faces, $0) }
    }

    public func softBodySettings(_ softBodyId: SoftBodyID) -> SoftBodySettings? {
        softBodyIndex(softBodyId).map { softBodies[$0].settings }
    }

    /// Rigid shapes touched during the last step.
    public func softBodyContacts(_ softBodyId: SoftBodyID) -> [SoftBodyContact] {
        guard let i = softBodyIndex(softBodyId) else { return [] }
        return softBodies[i].colliders.compactMap { collider in
            guard collider.contactCount > 0 else { return nil }
            let shape = shapes[Int(collider.shapeIndex)]
            var contact = SoftBodyContact()
            contact.shapeId = ShapeID(index1: shape.id + 1, world0: worldId, generation: shape.generation)
            contact.bodyId = makeBodyID(collider.bodyIndex)
            contact.vertexCount = collider.contactCount
            return contact
        }
    }

    /// True when vertex `vertex` touched a rigid shape in the last step.
    public func softBodyVertexHasContact(_ softBodyId: SoftBodyID, _ vertex: Int32) -> Bool {
        guard let i = softBodyIndex(softBodyId), vertex >= 0, Int(vertex) < softBodies[i].vertexCount else { return false }
        return softBodies[i].hasContact[Int(vertex)]
    }

    public func softBodyUserData(_ softBodyId: SoftBodyID) -> UnsafeMutableRawPointer? {
        softBodyIndex(softBodyId).flatMap { softBodies[$0].userData }
    }

    // MARK: Skinning

    /// Pose of the skeleton driving skinned vertices, one world transform per joint. Kinematic
    /// skinned vertices follow the skin; dynamic ones are constrained near it.
    public func setSoftBodySkinTransforms(_ softBodyId: SoftBodyID, _ transforms: [Transform], teleport: Bool = false) {
        guard let i = softBodyIndex(softBodyId), transforms.count >= softBodies[i].settings.inverseBindMatrices.count
        else { return }
        softBodies[i].jointTransforms = transforms
        softBodies[i].hasSkinPose = true
        softBodies[i].updateSkin()
        if teleport {
            for (k, s) in softBodies[i].settings.skinned.enumerated() {
                let v = Int(s.vertex)
                softBodies[i].positions[v] = softBodies[i].skinnedPositions[k]
                softBodies[i].previousPositions[v] = softBodies[i].skinnedPositions[k]
                softBodies[i].velocities[v] = Vec3()
            }
        }
        wakeSoftBody(i)
    }

    // MARK: Tuning

    public func setSoftBodyIterationCount(_ softBodyId: SoftBodyID, _ count: Int32) {
        guard let i = softBodyIndex(softBodyId) else { return }
        softBodies[i].iterationCount = max(1, count)
    }

    public func setSoftBodyPressure(_ softBodyId: SoftBodyID, _ pressure: Float) {
        guard let i = softBodyIndex(softBodyId) else { return }
        softBodies[i].pressure = pressure
        wakeSoftBody(i)
    }

    public func softBodyPressure(_ softBodyId: SoftBodyID) -> Float {
        softBodyIndex(softBodyId).map { softBodies[$0].pressure } ?? 0
    }

    public func setSoftBodyFriction(_ softBodyId: SoftBodyID, _ friction: Float) {
        guard let i = softBodyIndex(softBodyId) else { return }
        softBodies[i].friction = max(0, friction)
    }

    public func setSoftBodyRestitution(_ softBodyId: SoftBodyID, _ restitution: Float) {
        guard let i = softBodyIndex(softBodyId) else { return }
        softBodies[i].restitution = max(0, restitution)
    }

    public func setSoftBodyLinearDamping(_ softBodyId: SoftBodyID, _ damping: Float) {
        guard let i = softBodyIndex(softBodyId) else { return }
        softBodies[i].linearDamping = max(0, damping)
    }

    public func setSoftBodyGravityScale(_ softBodyId: SoftBodyID, _ scale: Float) {
        guard let i = softBodyIndex(softBodyId) else { return }
        softBodies[i].gravityScale = scale
        wakeSoftBody(i)
    }

    public func setSoftBodyFilter(_ softBodyId: SoftBodyID, _ filter: Filter) {
        guard let i = softBodyIndex(softBodyId) else { return }
        softBodies[i].filter = filter
    }

    // MARK: Sleep

    public func softBodyIsAwake(_ softBodyId: SoftBodyID) -> Bool {
        softBodyIndex(softBodyId).map { softBodies[$0].awake } ?? false
    }

    public func setSoftBodyAwake(_ softBodyId: SoftBodyID, _ awake: Bool) {
        guard let i = softBodyIndex(softBodyId) else { return }
        if awake {
            wakeSoftBody(i)
        } else {
            softBodies[i].awake = false
            for v in softBodies[i].velocities.indices { softBodies[i].velocities[v] = Vec3() }
        }
    }

    func wakeSoftBody(_ index: Int) {
        softBodies[index].awake = true
        softBodies[index].sleepTime = 0
    }
}

// MARK: - Skinning

extension SoftBodyRecord {
    /// Linear blend skinning of the skinned vertices and their normals.
    mutating func updateSkin() {
        guard hasSkinPose, settings.skinned.isEmpty == false else { return }
        let binds = settings.inverseBindMatrices
        let joints = jointTransforms
        guard joints.count >= binds.count else { return }

        for (k, s) in settings.skinned.enumerated() {
            let rest = settings.vertices[Int(s.vertex)].position
            var p = Vec3()
            var weightSum: Float = 0
            for j in 0..<4 where s.weights[j] > 0 {
                let joint = Int(s.joints[j])
                let skin = mulTransforms(joints[joint], binds[joint])
                p = mulAdd(p, s.weights[j], transformPoint(skin, rest))
                weightSum += s.weights[j]
            }
            skinnedPositions[k] = weightSum > 0 ? (1 / weightSum) * p : rest
        }

        // Skinned normals from the skinned neighbourhood. Unskinned neighbours use their position.
        var skinnedOfVertex = [Int32](repeating: -1, count: positions.count)
        for (k, s) in settings.skinned.enumerated() { skinnedOfVertex[Int(s.vertex)] = Int32(k) }
        func skinnedPosition(_ v: UInt32) -> Vec3 {
            let k = skinnedOfVertex[Int(v)]
            return k >= 0 ? skinnedPositions[Int(k)] : positions[Int(v)]
        }
        for (k, s) in settings.skinned.enumerated() {
            var n = Vec3()
            for f in s.normalFaces {
                let face = settings.faces[Int(f)]
                let a = skinnedPosition(face.vertex.x)
                let b = skinnedPosition(face.vertex.y)
                let c = skinnedPosition(face.vertex.z)
                n += cross(b - a, c - a)
            }
            skinnedNormals[k] = lengthSquared(n) > 0 ? normalize(n) : Vec3(0, 1, 0)
        }
    }
}

/// Volume enclosed by a closed, outward wound triangle surface.
func closedVolume(_ faces: [SoftBodyFace], _ positions: UnsafeBufferPointer<Vec3>) -> Float {
    var sixVolume: Float = 0
    for face in faces {
        let a = positions[Int(face.vertex.x)]
        let b = positions[Int(face.vertex.y)]
        let c = positions[Int(face.vertex.z)]
        sixVolume += dot(a, cross(b, c))
    }
    return sixVolume / 6
}
