


@testable import Natsel


struct SceneRandom {
    var seed: UInt32 = 12_345

    // XorShift32, mapped to [0, 32767].
    mutating func nextInt() -> Int {
        var x = seed
        x ^= x << 13
        x ^= x >> 17
        x ^= x << 5
        seed = x
        return Int(x % 32_768)
    }

    mutating func floatRange(_ lo: Float, _ hi: Float) -> Float {
        var r = Float(nextInt() & 32_767)
        r /= 32_767
        r = (hi - lo) * r + lo
        return r
    }

    mutating func vec3Uniform(_ lo: Float, _ hi: Float) -> Vec3 {
        Vec3(floatRange(lo, hi), floatRange(lo, hi), floatRange(lo, hi))
    }

    mutating func unitVector() -> Vec3 {
        let u1 = floatRange(0, 1)
        let u2 = floatRange(0, 2 * B3.pi)
        let u3 = floatRange(0, 2 * B3.pi)
        let sqrt1MinusU1 = (1 - u1).squareRoot()
        let sqrtU1 = u1.squareRoot()
        let cs2 = computeCosSin(u2)
        let cs3 = computeCosSin(u3)
        return Vec3(sqrt1MinusU1 * cs2.sine, sqrt1MinusU1 * cs2.cosine, sqrtU1 * cs3.sine)
    }

    mutating func quat() -> Quat {
        let u1 = floatRange(0, 1)
        let u2 = floatRange(0, 2 * B3.pi)
        let u3 = floatRange(0, 2 * B3.pi)
        let sqrt1MinusU1 = (1 - u1).squareRoot()
        let sqrtU1 = u1.squareRoot()
        let cs2 = computeCosSin(u2)
        let cs3 = computeCosSin(u3)
        return Quat(
            Vec3(sqrt1MinusU1 * cs2.sine, sqrt1MinusU1 * cs2.cosine, sqrtU1 * cs3.sine),
            sqrtU1 * cs3.cosine
        )
    }
}

// MARK: - Transform hashing

func hashTransform(_ hash: UInt32, _ transform: WorldTransform) -> UInt32 {
    var transform = transform
    return withUnsafeBytes(of: &transform) { raw in
        Natsel.hash(hash, raw.baseAddress!, Int32(raw.count))
    }
}


struct WavePileScene {
    static let bodyCount = 100

    var bodies: [BodyID] = []
    var heightField: HeightField
    var random = SceneRandom(seed: 52_977)
}

func createWavePile(_ world: World) -> WavePileScene {
    var scene: WavePileScene
    let fieldCount = 21
    let fieldScale = Vec3(1, 0.6, 1)
    let heightField = createWaveHeightField(
        rowCount: fieldCount, columnCount: fieldCount, scale: fieldScale,
        rowFrequency: 0.08, columnFrequency: 0.06
    )!
    scene = WavePileScene(bodies: [], heightField: heightField)

    let extent = fieldScale.x * Float(fieldCount - 1)
    var groundDef = BodyDef()
    groundDef.position = Pos(-0.5 * extent, 0, -0.5 * extent)
    let ground = world.createBody(groundDef)
    _ = world.createHeightFieldShape(ground, ShapeDef(), heightField)

    let rock = createRock(0.55)!
    let box = makeBoxHull(0.45, 0.3, 0.55)
    let sphere = Sphere(center: Vec3(), radius: 0.5)
    let capsule = Capsule(center1: Vec3(0, -0.3, 0), center2: Vec3(0, 0.3, 0), radius: 0.35)

    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody

    var shapeDef = ShapeDef()
    shapeDef.baseMaterial.rollingResistance = 0.3

    let spacing: Float = 1.7
    var index = 0
    for layer in 0..<4 {
        for i in 0..<5 {
            for j in 0..<5 {
                let jitter = scene.random.vec3Uniform(-0.3, 0.3)
                bodyDef.position.x = Double(spacing * (Float(i) - 0.5 * 4) + jitter.x)
                bodyDef.position.y = Double(2.5 + 1.6 * Float(layer) + 0.3 * jitter.y)
                bodyDef.position.z = Double(spacing * (Float(j) - 0.5 * 4) + jitter.z)
                bodyDef.rotation = scene.random.quat()

                let body = world.createBody(bodyDef)
                scene.bodies.append(body)

                switch index % 4 {
                case 0: _ = world.createSphereShape(body, shapeDef, sphere)
                case 1: _ = world.createCapsuleShape(body, shapeDef, capsule)
                case 2: _ = world.createHullShape(body, shapeDef, box)
                default: _ = world.createHullShape(body, shapeDef, rock)
                }
                index += 1
            }
        }
    }
    return scene
}

func updateWavePile(_ world: World, _ scene: inout WavePileScene, stepCount: inout Int,
                    sleepStep: inout Int, hash: inout UInt32) -> Bool {
    if hash == 0 && world.awakeBodyCount() == 0 {
        hash = hashInit
        for body in scene.bodies {
            hash = hashTransform(hash, world.bodyTransform(body))
        }
        sleepStep = stepCount
    }
    stepCount += 1
    return hash != 0
}


struct MeshDropScene {
    static let gridCount = 20

    var mesh: TriangleMesh
    var bodies: [BodyID] = []
}

func createMeshDrop(_ world: World, _ origin: Pos) -> MeshDropScene {
    let mesh = createWaveMesh(
        xCount: 40, zCount: 40, cellWidth: 1, amplitude: 0.5,
        rowFrequency: 0.1, columnFrequency: 0.2
    )!
    let ground = world.createBody(BodyDef())
    var groundShape = ShapeDef()
    groundShape.filter.categoryBits = 1
    _ = world.createMeshShape(ground, groundShape, mesh)

    let box = makeBoxHull(0.02, 0.2, 0.04)
    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody

    var shapeDef = ShapeDef()
    shapeDef.baseMaterial.rollingResistance = 0.1
    shapeDef.filter.categoryBits = 2
    shapeDef.filter.maskBits = 1

    var random = SceneRandom(seed: 3_963_634_789)
    var scene = MeshDropScene(mesh: mesh)
    for i in 0..<MeshDropScene.gridCount {
        for j in 0..<MeshDropScene.gridCount {
            bodyDef.linearVelocity = random.vec3Uniform(-1, 1)
            bodyDef.angularVelocity = random.vec3Uniform(-5, 5)
            bodyDef.position = offsetPos(
                origin, Vec3(0.5 * (Float(i) - 0.5 * Float(MeshDropScene.gridCount)), 5,
                             0.5 * (Float(j) - 0.5 * Float(MeshDropScene.gridCount)))
            )
            let body = world.createBody(bodyDef)
            scene.bodies.append(body)
            _ = world.createHullShape(body, shapeDef, box)
        }
    }
    return scene
}

func updateMeshDrop(_ world: World, _ scene: inout MeshDropScene, stepCount: inout Int,
                    sleepStep: inout Int, hash: inout UInt32) -> Bool {
    if hash == 0 && world.awakeBodyCount() == 0 {
        hash = hashInit
        for body in scene.bodies {
            hash = hashTransform(hash, world.bodyTransform(body))
        }
        sleepStep = stepCount
    }
    stepCount += 1
    return hash != 0
}


struct QuerySpawnScene {
    static let count = 50
    static let castRadius: Float = 0.5

    var bodies: [BodyID] = []
    var spawnCount = 0
    var queryHitCount = 0
    var queryHash: UInt32 = 0
}

func createQuerySpawn(_ world: World) -> QuerySpawnScene {
    world.gravity = Vec3()
    return QuerySpawnScene()
}

func querySpawnOnce(_ world: World, _ scene: inout QuerySpawnScene, _ random: inout SceneRandom) {
    let filter = QueryFilter.defaultFilter

    let rayOrigin = Pos(
        random.floatRange(-12, 12), random.floatRange(-12, 12), random.floatRange(-12, 12)
    )
    let rayTranslation = random.unitVector() * 30

    let spawnPosition: Pos
    let ray = world.castRayClosest(rayOrigin, rayTranslation, filter)
    if ray.hit {
        scene.queryHitCount += 1
        var fraction = ray.fraction
        scene.queryHash = withUnsafeBytes(of: &fraction) {
            hash(scene.queryHash, $0.baseAddress!, 4)
        }
        var normal = ray.normal
        scene.queryHash = withUnsafeBytes(of: &normal) {
            hash(scene.queryHash, $0.baseAddress!, 12)
        }
        spawnPosition = offsetPos(ray.point, ray.normal * 1.2)
    } else {
        spawnPosition = Pos(
            random.floatRange(-6, 6), random.floatRange(-6, 6), random.floatRange(-6, 6)
        )
    }

    let center = random.vec3Uniform(-10, 10)
    let extent = random.floatRange(1, 4)
    let aabb = AABB(
        Vec3(center.x - extent, center.y - extent, center.z - extent),
        Vec3(center.x + extent, center.y + extent, center.z + extent)
    )

    var overlapCount = 0
    _ = world.overlapAABB(aabb, filter) { id in
        overlapCount += 1
        scene.queryHitCount += 1
        var index1 = id.index1
        scene.queryHash = withUnsafeBytes(of: &index1) {
            hash(scene.queryHash, $0.baseAddress!, 4)
        }
        return true
    }

    var fraction: Float = 1
    var proxyPoint = Vec3()
    let proxy = withUnsafePointer(to: &proxyPoint) {
        ShapeProxy(points: $0, count: 1, radius: QuerySpawnScene.castRadius)
    }
    var castInput = ShapeCastInput()
    castInput.proxy = proxy
    castInput.translation = rayTranslation
    castInput.maxFraction = 1
    _ = world.shapeCast(rayOrigin, castInput, filter) {
        _, _, _, castFraction, _, _, _ in
        fraction = castFraction
        return castFraction
    }
    if fraction < 1 {
        scene.queryHitCount += 1
        var value = fraction
        scene.queryHash = withUnsafeBytes(of: &value) {
            hash(scene.queryHash, $0.baseAddress!, 4)
        }
    }

    let size: Float = 0.3 + 0.2 * fraction

    var bodyDef = BodyDef()
    bodyDef.type = .dynamicBody
    bodyDef.position = spawnPosition
    bodyDef.rotation = random.quat()
    bodyDef.linearVelocity = random.vec3Uniform(-0.2, 0.2)
    bodyDef.angularVelocity = random.vec3Uniform(-0.5, 0.5)
    bodyDef.linearDamping = 1
    bodyDef.angularDamping = 1

    let body = world.createBody(bodyDef)

    var shapeDef = ShapeDef()
    shapeDef.baseMaterial.rollingResistance = 0.2

    switch (scene.spawnCount + overlapCount) % 3 {
    case 0:
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: size))
    case 1:
        _ = world.createCapsuleShape(
            body, shapeDef,
            Capsule(center1: Vec3(0, -size, 0), center2: Vec3(0, size, 0), radius: 0.7 * size)
        )
    default:
        _ = world.createHullShape(body, shapeDef, makeBoxHull(size, 0.7 * size, 0.5 * size))
    }

    scene.bodies.append(body)
    scene.spawnCount += 1
}

func updateQuerySpawn(_ world: World, _ scene: inout QuerySpawnScene, _ random: inout SceneRandom,
                      stepCount: inout Int, sleepStep: inout Int, hash: inout UInt32) -> Bool {
    if scene.spawnCount < QuerySpawnScene.count {
        querySpawnOnce(world, &scene, &random)
    } else if hash == 0 && world.awakeBodyCount() == 0 {
        hash = hashInit
        for body in scene.bodies {
            hash = hashTransform(hash, world.bodyTransform(body))
        }
        sleepStep = stepCount
    }
    stepCount += 1
    return hash != 0
}
