// Scenes for the systems beyond the Box3D port: vehicles, cloth, and volumetric soft bodies.

import Natsel

nonisolated(unsafe) var benchmarkVehicles: [VehicleID] = []

private func makeTerrain(_ world: World) {
    let ground = world.createBody(BodyDef())
    if let mesh = createWaveMesh(xCount: 120, zCount: 120, cellWidth: 2, amplitude: 0.4, rowFrequency: 0.15, columnFrequency: 0.15) {
        _ = world.createMeshShape(ground, ShapeDef(), mesh)
    }
}

// MARK: - vehicles

/// 64 four wheel drive cars with anti-roll bars and cylinder wheel casts on wavy terrain.
func createVehicles(_ world: World) {
    makeTerrain(world)
    benchmarkVehicles = []
    for row in 0..<8 {
        for column in 0..<8 {
            var bodyDef = BodyDef()
            bodyDef.type = .dynamicBody
            bodyDef.position = Pos(Float(column - 4) * 12, 3, Float(row - 4) * 12)
            let body = world.createBody(bodyDef)
            var shapeDef = ShapeDef()
            shapeDef.density = 1500 / (8 * 0.9 * 2 * 0.3)
            _ = world.createHullShape(body, shapeDef, makeBoxHull(0.9, 0.3, 2))

            var def = VehicleDef()
            def.bodyId = body
            def.collisionTester = .cylinder(convexRadiusFraction: 0.5)
            def.maxPitchRollAngle = 60 * 0.0174533
            for (i, p) in [Vec3(0.9, -0.1, 1.5), Vec3(-0.9, -0.1, 1.5), Vec3(0.9, -0.1, -1.5), Vec3(-0.9, -0.1, -1.5)].enumerated() {
                var wheel = WheelSettings()
                wheel.position = p
                wheel.radius = 0.35
                wheel.width = 0.25
                wheel.suspensionMinLength = 0.1
                wheel.suspensionMaxLength = 0.4
                wheel.maxSteerAngle = i < 2 ? 0.5 : 0
                def.wheels.append(wheel)
            }
            def.antiRollBars = [VehicleAntiRollBar(leftWheel: 0, rightWheel: 1), VehicleAntiRollBar(leftWheel: 2, rightWheel: 3)]
            var controller = WheeledVehicleControllerSettings()
            controller.differentials = [
                VehicleDifferentialSettings(leftWheel: 0, rightWheel: 1, engineTorqueRatio: 0.5),
                VehicleDifferentialSettings(leftWheel: 2, rightWheel: 3, engineTorqueRatio: 0.5),
            ]
            def.controller = .wheeled(controller)
            benchmarkVehicles.append(world.createVehicle(def))
        }
    }
}

func stepVehicles(_ world: World, _ stepIndex: Int) {
    for (i, vehicle) in benchmarkVehicles.enumerated() {
        let steer: Float = (stepIndex / 120 + i) % 2 == 0 ? 0.4 : -0.4
        world.setVehicleDriverInput(vehicle, forward: 0.6, right: steer)
    }
}

func destroyVehicles() {
    benchmarkVehicles = []
}

// MARK: - cloth

/// Eight 40x40 cloths falling over static spheres and a pile of dynamic boxes.
func createCloth(_ world: World) {
    let ground = world.createBody(BodyDef())
    _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(50, 1, 50, Vec3(0, -1, 0)))
    let attributes = SoftBodyVertexAttributes(compliance: 1.0e-6, shearCompliance: 1.0e-5, bendCompliance: 1.0e-3)
    let settings = SoftBodySettings.makeCloth(columns: 40, rows: 40, spacing: 0.05, attributes: attributes, bendType: benchmarkEnvironment["NATSEL_BEND"] == "distance" ? .distance : .dihedral)

    for k in 0..<8 {
        let x = Float(k % 4) * 3 - 4.5
        let z = Float(k / 4) * 3 - 1.5
        let obstacle = world.createBody(BodyDef())
        _ = world.createSphereShape(obstacle, ShapeDef(), Sphere(center: Vec3(x, 0.6, z), radius: 0.6))

        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(x + 0.6, 1.6, z)
        let box = world.createBody(bodyDef)
        _ = world.createHullShape(box, ShapeDef(), makeBoxHull(0.15, 0.15, 0.15))

        var def = SoftBodyDef(settings: settings, position: Pos(x, 2, z))
        def.vertexRadius = 0.01
        def.enableSleep = false
        _ = world.createSoftBody(def)
    }
}

// MARK: - soft_cubes

/// GuavaStudio's 64x64 cloth acceptance scene, with its authored vertex order and self collision.
nonisolated(unsafe) private var selfCollidingCloth64: (World, SoftBodyID)?

func createSelfCollidingCloth64(_ world: World) {
    world.gravity = Vec3(0, -9.81, 0)
    var settings = SoftBodySettings()
    for z in 0..<64 {
        for x in 0..<64 {
            settings.vertices.append(SoftBodyVertex(position: Vec3((Float(x) - 31.5) * 0.05, 0, (Float(z) - 31.5) * 0.05),
                invMass: z == 0 ? 0 : 20))
        }
    }
    for z in 0..<63 {
        for x in 0..<63 {
            let a = UInt32(z * 64 + x), b = a + 64, c = b + 1, d = a + 1
            settings.faces.append(SoftBodyFace(a, b, c))
            settings.faces.append(SoftBodyFace(a, c, d))
        }
    }
    settings.createConstraints([SoftBodyVertexAttributes(compliance: 1.0e-5, shearCompliance: 1.0e-5, bendCompliance: 1.0e-5)], bendType: .distance)
    settings.calculateRestValues()
    var def = SoftBodyDef(settings: settings, position: Pos(0, 8, 0))
    def.iterationCount = 5
    def.linearDamping = 0.05
    def.vertexRadius = 0.02
    def.maxLinearVelocity = 500
    def.enableSleep = false
    def.enableSelfCollision = true
    def.facesDoubleSided = true
    selfCollidingCloth64 = (world, world.createSoftBody(def))
}

func destroySelfCollidingCloth64() {
    if recordHash, let (world, id) = selfCollidingCloth64 {
        var hash: UInt64 = 0xcbf29ce484222325
        for p in world.softBodyVertexPositions(id) {
            for bits in [p.x.bitPattern, p.y.bitPattern, p.z.bitPattern] { hash = (hash ^ bits) &* 0x100000001b3 }
        }
        print("cloth_positions_hash: \(String(hash, radix: 16))")
    }
    selfCollidingCloth64 = nil
}

/// 27 volumetric cubes dropped in a pile.
func createSoftCubes(_ world: World) {
    let ground = world.createBody(BodyDef())
    _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(50, 1, 50, Vec3(0, -1, 0)))
    let settings = SoftBodySettings.makeCube(count: 6, spacing: 0.1, compliance: 1.0e-5)
    for i in 0..<27 {
        let p = Pos(Float(i % 3) * 0.7 - 0.7, 1 + Float(i / 9) * 0.7, Float((i / 3) % 3) * 0.7 - 0.7)
        var def = SoftBodyDef(settings: settings, position: p)
        def.vertexRadius = 0.01
        def.enableSleep = false
        _ = world.createSoftBody(def)
    }
}

// MARK: - cloth_stack

/// Six 30x30 cloths dropped onto each other over a sphere: soft body contact and self collision.
func createClothStack(_ world: World) {
    let ground = world.createBody(BodyDef())
    _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(50, 1, 50, Vec3(0, -1, 0)))
    _ = world.createSphereShape(ground, ShapeDef(), Sphere(center: Vec3(0, 0.5, 0), radius: 0.5))
    let attributes = SoftBodyVertexAttributes(compliance: 1.0e-6, shearCompliance: 1.0e-5, bendCompliance: 1.0e-3)
    let settings = SoftBodySettings.makeCloth(columns: 30, rows: 30, spacing: 0.05, attributes: attributes)
    for k in 0..<6 {
        var def = SoftBodyDef(settings: settings, position: Pos(0.05 * Float(k), 1.4 + 0.25 * Float(k), 0),
                              rotation: makeQuatFromAxisAngle(.axisY, 0.3 * Float(k)))
        def.enableSelfCollision = true
        def.enableSleep = false
        def.friction = 0.4
        _ = world.createSoftBody(def)
    }
}

// MARK: - path_beads

/// 64 tilted loops with 16 beads each: path joints with motors, friction, and rotation constraints.
func createPathBeads(_ world: World) {
    let ground = world.createBody(BodyDef())
    _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(80, 1, 80, Vec3(0, -1, 0)))
    let modes: [PathRotationConstraint] = [.free, .constrainAroundTangent, .constrainToPath, .fullyConstrained]
    for loop in 0..<64 {
        let center = Vec3(Float(loop % 8) * 7 - 24, 4, Float(loop / 8) * 7 - 24)
        let positions = (0..<12).map { i -> Vec3 in
            let cs = computeCosSin(2 * Float.pi * Float(i) / 12)
            return Vec3(2.5 * cs.cosine, 0.8 * cs.sine, 2.5 * cs.sine)
        }
        let path = ConstraintPath(through: positions, isLooping: true)!
        for bead in 0..<16 {
            let fraction = Float(bead) * 0.75
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(center + path.point(at: fraction).position)
            let body = world.createBody(def)
            _ = world.createHullShape(body, ShapeDef(), makeCubeHull(0.2))
            var joint = PathJointDef()
            joint.base.bodyIdA = ground
            joint.base.bodyIdB = body
            joint.base.localFrameA = Transform(center, .identity)
            joint.base.collideConnected = true
            joint.path = path
            joint.pathFraction = fraction
            joint.rotationConstraint = modes[bead % modes.count]
            if bead % 2 == 0 {
                joint.motorMode = .velocity
                joint.targetVelocity = 2
                joint.maxMotorForce = 40
            } else {
                joint.maxFrictionForce = 3
            }
            _ = world.createPathJoint(joint)
        }
    }
}

// MARK: - ragdoll_motors

nonisolated(unsafe) var benchmarkRagdolls: [Ragdoll] = []
nonisolated(unsafe) var benchmarkWalk = SkeletalAnimation()

private func ragdollSkeleton() -> Skeleton {
    var skeleton = Skeleton()
    skeleton.addJoint("pelvis")
    skeleton.addJoint("spine", parentName: "pelvis")
    skeleton.addJoint("head", parentName: "spine")
    for side in ["L", "R"] {
        skeleton.addJoint("upperArm\(side)", parentName: "spine")
        skeleton.addJoint("lowerArm\(side)", parentName: "upperArm\(side)")
        skeleton.addJoint("thigh\(side)", parentName: "pelvis")
        skeleton.addJoint("calf\(side)", parentName: "thigh\(side)")
    }
    return skeleton
}

/// 100 skeleton ragdolls that follow a looping walk cycle with joint motors.
func createRagdollMotors(_ world: World) {
    let ground = world.createBody(BodyDef())
    _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(80, 1, 80, Vec3(0, -1, 0)))
    let skeleton = ragdollSkeleton()
    let down = makeQuatFromAxisAngle(.axisZ, -0.5 * Float.pi)
    let up = makeQuatFromAxisAngle(.axisZ, 0.5 * Float.pi)
    var bind = SkeletonPose(skeleton: skeleton)
    func set(_ name: String, _ p: Vec3, _ q: Quat) { bind.jointMatrices[skeleton.jointIndex(name)!] = Transform(p, q) }
    set("pelvis", Vec3(0, 1.0, 0), up)
    set("spine", Vec3(0, 1.25, 0), up)
    set("head", Vec3(0, 1.65, 0), up)
    for (side, x) in [("L", Float(-1)), ("R", Float(1))] {
        set("upperArm\(side)", Vec3(0.25 * x, 1.55, 0), down)
        set("lowerArm\(side)", Vec3(0.25 * x, 1.25, 0), down)
        set("thigh\(side)", Vec3(0.12 * x, 0.95, 0), down)
        set("calf\(side)", Vec3(0.12 * x, 0.5, 0), down)
    }
    bind.calculateJointStates()

    func capsule(_ length: Float, _ radius: Float) -> RagdollShape {
        .capsule(Capsule(center1: Vec3(radius, 0, 0), center2: Vec3(length - radius, 0, 0), radius: radius))
    }
    let hinge = Transform(Vec3(), makeQuatFromAxisAngle(.axisZ, 0.5 * Float.pi))
    var parts: [RagdollPart] = []
    for joint in skeleton.joints {
        var part = RagdollPart()
        part.jointFriction = 0.5
        switch joint.name {
        case "pelvis": part.shapes = [.box(halfExtents: Vec3(0.12, 0.15, 0.1), transform: Transform(Vec3(0.1, 0, 0), .identity))]
        case "spine": part.shapes = [capsule(0.38, 0.13)]; part.joint = .swingTwist(coneAngle: 0.4, twistMin: -0.3, twistMax: 0.3)
        case "head": part.shapes = [.sphere(Sphere(center: Vec3(0.12, 0, 0), radius: 0.11))]
        case let n where n.hasPrefix("upperArm"): part.shapes = [capsule(0.3, 0.05)]; part.joint = .swingTwist(coneAngle: 1.2, twistMin: -0.5, twistMax: 0.5)
        case let n where n.hasPrefix("lowerArm"): part.shapes = [capsule(0.3, 0.045)]; part.joint = .hinge(lower: -2, upper: 0); part.jointFrame = hinge
        case let n where n.hasPrefix("thigh"): part.shapes = [capsule(0.45, 0.07)]; part.joint = .pyramid(twist: -0.3...0.3, swingY: -0.4...0.4, swingZ: -1.2...0.4)
        default: part.shapes = [capsule(0.45, 0.055)]; part.joint = .hinge(lower: 0, upper: 2); part.jointFrame = hinge
        }
        parts.append(part)
    }
    let settings = RagdollSettings(skeleton: skeleton, parts: parts)

    // Walk cycle: thighs and knees swing out of phase.
    func swing(_ angle: Float, _ axis: Vec3, _ base: Quat) -> Quat { mulQuat(base, makeQuatFromAxisAngle(axis, angle)) }
    var tracks: [SkeletalAnimation.Track] = []
    for (side, phase) in [("L", Float(0)), ("R", Float.pi)] {
        let thigh = bind.jointStates[skeleton.jointIndex("thigh\(side)")!]
        let calf = bind.jointStates[skeleton.jointIndex("calf\(side)")!]
        var thighKeys: [SkeletalAnimation.Keyframe] = []
        var calfKeys: [SkeletalAnimation.Keyframe] = []
        for k in 0...8 {
            let t = Float(k) / 8
            let s = computeCosSin(2 * Float.pi * t + phase).sine
            thighKeys.append(.init(time: t, translation: thigh.translation, rotation: swing(0.5 * s, .axisZ, thigh.rotation)))
            calfKeys.append(.init(time: t, translation: calf.translation, rotation: swing(0.6 + 0.5 * s, .axisY, calf.rotation)))
        }
        tracks.append(.init(jointName: "thigh\(side)", keyframes: thighKeys))
        tracks.append(.init(jointName: "calf\(side)", keyframes: calfKeys))
    }
    benchmarkWalk = SkeletalAnimation(tracks: tracks, isLooping: true)

    benchmarkRagdolls = []
    for i in 0..<100 {
        var pose = bind
        pose.rootOffset = Pos(Float(i % 10) * 2 - 9, 0.2, Float(i / 10) * 2 - 9)
        benchmarkRagdolls.append(Ragdoll(world: world, settings: settings, pose: pose)!)
    }
}

func stepRagdollMotors(_ world: World, _ stepIndex: Int) {
    guard let first = benchmarkRagdolls.first else { return }
    var pose = SkeletonPose(skeleton: first.settings.skeleton)
    let current = first.pose()
    pose.jointStates = current.jointStates
    benchmarkWalk.sample(Float(stepIndex) / 60, &pose)
    pose.calculateJointMatrices()
    for ragdoll in benchmarkRagdolls {
        ragdoll.driveToPose(usingMotors: pose, maxTorque: 150, hertz: 6)
    }
}

func destroyRagdollMotors() {
    benchmarkRagdolls = []
}

// MARK: - rollback

nonisolated(unsafe) var rollbackStates: [WorldState] = []

/// A large pyramid saved every step; every 30 steps the last 8 are rolled back and replayed,
/// as a rollback network game does.
func createRollback(_ world: World) {
    createLargePyramid(world)
    rollbackStates = []
}

func stepRollback(_ world: World, _ stepIndex: Int) {
    rollbackStates.append(world.saveState())
    if rollbackStates.count > 8 { rollbackStates.removeFirst() }
    if stepIndex % 30 == 29, let oldest = rollbackStates.first {
        world.restoreState(oldest)
        for _ in 0..<7 { world.step(1.0 / 60.0, subStepCount: 4) }
        rollbackStates.removeAll()
    }
}

func destroyRollback() {
    rollbackStates = []
}
