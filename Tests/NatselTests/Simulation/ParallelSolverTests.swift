import Testing

@testable import Natsel

/// Scenes that exercise every solver stage. Each must give the same bits for any worker count.
enum ParallelScene: String, CaseIterable {
    case pyramid
    case jointGrid
    case meshMix
    case restitution
    case mixedJoints
    case ragdolls
    case fastAndSleepy
    case paths

    func make(_ workers: Int32) -> World {
        let world: World
        switch self {
        case .meshMix:
            world = ImpulseDeterminismTests().makeMeshMix(true).0
        default:
            world = World()
        }
        world.workerCount = workers
        switch self {
        case .pyramid: buildPyramid(world)
        case .jointGrid: buildJointGrid(world)
        case .meshMix: break
        case .restitution: buildRestitution(world)
        case .mixedJoints: buildMixedJoints(world)
        case .ragdolls: buildRagdolls(world)
        case .fastAndSleepy: buildFastAndSleepy(world)
        case .paths: buildPaths(world)
        }
        return world
    }
}

private func ground(_ world: World) -> BodyID {
    var def = BodyDef()
    def.position = Vec3(0, -1, 0)
    let body = world.createBody(def)
    _ = world.createHullShape(body, ShapeDef(), makeBoxHull(100, 1, 100))
    return body
}

private func buildPyramid(_ world: World) {
    world.enableSleep = false
    _ = ground(world)
    let box = makeCubeHull(0.5)
    let base = 24
    for row in 0..<base {
        for column in 0..<(base - row) {
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Vec3(Float(column) - 0.5 * Float(base - row - 1), 0.5 + Float(row), 0)
            let body = world.createBody(def)
            _ = world.createHullShape(body, ShapeDef(), box)
        }
    }
}

private func buildJointGrid(_ world: World) {
    world.gravity = Vec3(0, -10, 0)
    let n = 30
    var bodies = [BodyID]()
    for k in 0..<n {
        for i in 0..<n {
            var def = BodyDef()
            def.type = i == 0 ? .staticBody : .dynamicBody
            def.position = Vec3(Float(k), -Float(i), 0)
            let body = world.createBody(def)
            _ = world.createSphereShape(body, ShapeDef(), Sphere(center: Vec3(), radius: 0.4))
            var joint = SphericalJointDef()
            if i > 0 {
                joint.base.bodyIdA = bodies[bodies.count - 1]
                joint.base.bodyIdB = body
                joint.base.localFrameA.p = Vec3(0, -0.5, 0)
                joint.base.localFrameB.p = Vec3(0, 0.5, 0)
                _ = world.createSphericalJoint(joint)
            }
            if k > 0 {
                joint.base.bodyIdA = bodies[bodies.count - n]
                joint.base.bodyIdB = body
                joint.base.localFrameA.p = Vec3(0.5, 0, 0)
                joint.base.localFrameB.p = Vec3(-0.5, 0, 0)
                _ = world.createSphericalJoint(joint)
            }
            bodies.append(body)
        }
    }
}

private func buildRestitution(_ world: World) {
    _ = ground(world)
    var shapeDef = ShapeDef()
    shapeDef.baseMaterial.restitution = 0.8
    for i in 0..<400 {
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(Float(i % 20) * 1.1 - 10, 1 + Float(i / 20) * 1.1, Float(i % 7) * 0.05)
        let body = world.createBody(def)
        if i % 2 == 0 {
            _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 0.5))
        } else {
            _ = world.createHullShape(body, shapeDef, makeCubeHull(0.45))
        }
    }
}

private func buildMixedJoints(_ world: World) {
    let anchor = ground(world)
    for chain in 0..<12 {
        var previous = anchor
        for link in 0..<12 {
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Vec3(Float(chain) * 2, 12 - Float(link), 0)
            let body = world.createBody(def)
            _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.4, 0.4, 0.4))
            let pivot = Vec3(Float(chain) * 2, 12.5 - Float(link), 0)
            switch (chain + link) % 4 {
            case 0:
                var joint = RevoluteJointDef()
                joint.base.bodyIdA = previous
                joint.base.bodyIdB = body
                joint.base.localFrameA = Transform(world.bodyLocalPoint(previous, pivot), .identity)
                joint.base.localFrameB = Transform(world.bodyLocalPoint(body, pivot), .identity)
                joint.enableMotor = true
                joint.motorSpeed = 1
                joint.maxMotorTorque = 50
                _ = world.createRevoluteJoint(joint)
            case 1:
                var joint = SixDOFJointDef()
                joint.base.bodyIdA = previous
                joint.base.bodyIdB = body
                joint.base.localFrameA = Transform(world.bodyLocalPoint(previous, pivot), .identity)
                joint.base.localFrameB = Transform(world.bodyLocalPoint(body, pivot), .identity)
                joint.lockAll(except: [.rotationX, .rotationZ])
                joint[.rotationZ] = SixDOFAxisSettings(.limited, lower: -0.5, upper: 0.5)
                _ = world.createSixDOFJoint(joint)
            case 2:
                var joint = WeldJointDef()
                joint.base.bodyIdA = previous
                joint.base.bodyIdB = body
                joint.base.localFrameA = Transform(world.bodyLocalPoint(previous, pivot), .identity)
                joint.base.localFrameB = Transform(world.bodyLocalPoint(body, pivot), .identity)
                joint.linearHertz = 5
                joint.angularHertz = 5
                _ = world.createWeldJoint(joint)
            default:
                var joint = DistanceJointDef()
                joint.base.bodyIdA = previous
                joint.base.bodyIdB = body
                joint.base.localFrameA = Transform(world.bodyLocalPoint(previous, pivot), .identity)
                joint.base.localFrameB = Transform(world.bodyLocalPoint(body, pivot), .identity)
                joint.length = 0.1
                _ = world.createDistanceJoint(joint)
            }
            previous = body
        }
    }
}

private func buildRagdolls(_ world: World) {
    _ = ground(world)
    for i in 0..<16 {
        var human = Human()
        createHuman(&human, world, Vec3(Float(i % 4) * 1.5, 2 + Float(i / 4) * 2, 0), 0.05, 5, 0.5, i + 1, nil, false)
    }
}

/// Enough awake bodies to finalize in parallel: a grid of boxes that settles and falls asleep, plus
/// fast bodies and bullets fired into static walls and the boxes, so the continuous pass, the bullet
/// list, island sleep, and proxy enlargement all run.
private func buildFastAndSleepy(_ world: World) {
    _ = ground(world)
    let wall = makeBoxHull(0.25, 4, 20)
    for x: Float in [-16, 16] {
        var def = BodyDef()
        def.position = Vec3(x, 4, 0)
        let body = world.createBody(def)
        _ = world.createHullShape(body, ShapeDef(), wall)
    }
    let box = makeCubeHull(0.4)
    for i in 0..<640 {
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(Float(i % 16) - 7.5, 0.4 + Float(i / 256) * 0.8, Float((i / 16) % 16) - 7.5)
        let body = world.createBody(def)
        _ = world.createHullShape(body, ShapeDef(), box)
    }
    for i in 0..<24 {
        var def = BodyDef()
        def.type = .dynamicBody
        def.isBullet = i % 3 == 0
        def.position = Vec3(i % 2 == 0 ? -12 : 12, 1 + Float(i / 2) * 0.5, Float(i % 5) - 2)
        def.linearVelocity = Vec3(i % 2 == 0 ? 300 : -300, 0, Float(i % 3) - 1)
        let body = world.createBody(def)
        _ = world.createSphereShape(body, ShapeDef(), Sphere(center: Vec3(), radius: 0.1))
    }
}

/// Beads on tilted loops with every rotation mode and motor kind; neighbors collide.
private func buildPaths(_ world: World) {
    let anchor = ground(world)
    let modes: [PathRotationConstraint] = [.free, .constrainAroundTangent, .constrainAroundNormal,
                                           .constrainAroundBinormal, .constrainToPath, .fullyConstrained]
    for loop in 0..<24 {
        let center = Vec3(Float(loop % 6) * 7 - 17, 4, Float(loop / 6) * 7 - 10)
        let positions = (0..<10).map { i -> Vec3 in
            let cs = computeCosSin(2 * Float.pi * Float(i) / 10)
            return Vec3(2.5 * cs.cosine, 0.8 * cs.sine, 2.5 * cs.sine)
        }
        let path = ConstraintPath(through: positions, isLooping: true)!
        for bead in 0..<8 {
            let at = path.point(at: Float(bead) * 1.25).position
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = center + at
            let body = world.createBody(def)
            _ = world.createHullShape(body, ShapeDef(), makeCubeHull(0.3))
            var joint = PathJointDef()
            joint.base.bodyIdA = anchor
            joint.base.bodyIdB = body
            joint.base.localFrameA = Transform(center, .identity)
            joint.base.collideConnected = true
            joint.path = path
            joint.pathFraction = Float(bead) * 1.25
            joint.rotationConstraint = modes[(loop + bead) % modes.count]
            switch bead % 3 {
            case 0: joint.maxFrictionForce = 2
            case 1:
                joint.motorMode = .velocity
                joint.targetVelocity = Float(loop % 4) - 1.5
                joint.maxMotorForce = 30
            default:
                joint.motorMode = .position
                joint.targetFraction = Float(loop % 10)
                joint.maxMotorForce = 20
            }
            _ = world.createPathJoint(joint)
        }
    }
}

@Suite("Parallel solver")
struct ParallelSolverTests {
    @Test("bit identical for any worker count", arguments: ParallelScene.allCases)
    func identical(_ scene: ParallelScene) {
        let serial = scene.make(1)
        let worlds = [scene.make(4), scene.make(8)]
        for step in 0..<90 {
            serial.step(1.0 / 60.0, subStepCount: 4)
            let expected = serial.worldStateHash()
            for world in worlds {
                world.step(1.0 / 60.0, subStepCount: 4)
                #expect(world.worldStateHash() == expected, "\(scene) step \(step) workers \(world.workerCount)")
            }
        }
        #expect(serial.awakeBodyCount() > 0)
    }

    @Test("changing the worker count between steps keeps the result")
    func changeWorkers() {
        let a = ParallelScene.pyramid.make(1)
        let b = ParallelScene.pyramid.make(1)
        for step in 0..<60 {
            b.workerCount = Int32([1, 3, 8, 2][step % 4])
            a.step(1.0 / 60.0, subStepCount: 4)
            b.step(1.0 / 60.0, subStepCount: 4)
            #expect(a.worldStateHash() == b.worldStateHash(), "step \(step)")
        }
    }

    @Test("vehicles, soft bodies, and joints with worker threads")
    func systems() {
        func run(_ workers: Int32) -> UInt64 {
            let world = World()
            world.workerCount = workers
            _ = ground(world)
            for k in 0..<6 {
                var def = BodyDef()
                def.type = .dynamicBody
                def.position = Vec3(Float(k) * 4, 1.2, 0)
                let body = world.createBody(def)
                var shapeDef = ShapeDef()
                shapeDef.density = 400
                _ = world.createHullShape(body, shapeDef, makeBoxHull(0.9, 0.25, 2))
                var vehicle = VehicleDef()
                vehicle.bodyId = body
                for p in [Vec3(0.9, -0.1, 1.6), Vec3(-0.9, -0.1, 1.6), Vec3(0.9, -0.1, -1.6), Vec3(-0.9, -0.1, -1.6)] {
                    var wheel = WheelSettings()
                    wheel.position = p
                    vehicle.wheels.append(wheel)
                }
                var controller = WheeledVehicleControllerSettings()
                controller.differentials = [VehicleDifferentialSettings(leftWheel: 0, rightWheel: 1)]
                vehicle.controller = .wheeled(controller)
                let id = world.createVehicle(vehicle)
                world.setVehicleDriverInput(id, forward: 1, right: k % 2 == 0 ? 0.3 : -0.3)
            }
            _ = world.createSoftBody(SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 12, rows: 12, spacing: 0.1),
                                                 position: Vec3(0, 3, 4)))
            buildPyramid(world)
            for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
            return world.worldStateHash()
        }
        #expect(run(1) == run(6))
    }
}
