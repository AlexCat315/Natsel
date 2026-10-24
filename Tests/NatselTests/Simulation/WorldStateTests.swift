import Testing
@testable import Natsel

/// A scene with every system: a sleeping pyramid, ragdolls, joints of every kind, a sensor,
/// vehicles, cloth, and a bullet.
private final class StateScene {
    let world = World()
    var vehicles: [VehicleID] = []
    var softBody = SoftBodyID()
    var boxes: [BodyID] = []
    var joints: [JointID] = []
    let anchor: BodyID
    var character: CharacterVirtual!

    init(workers: Int32 = 1) {
        world.workerCount = workers
        var groundDef = BodyDef()
        groundDef.position = Vec3(0, -1, 0)
        anchor = world.createBody(groundDef)
        _ = world.createHullShape(anchor, ShapeDef(), makeBoxHull(60, 1, 60))

        // Sensor over the pyramid.
        var sensorDef = ShapeDef()
        sensorDef.isSensor = true
        sensorDef.enableSensorEvents = true
        _ = world.createHullShape(anchor, sensorDef, makeBoxHull(3, 1, 1))

        let box = makeCubeHull(0.4)
        var shapeDef = ShapeDef()
        shapeDef.enableSensorEvents = true
        for row in 0..<8 {
            for column in 0..<(8 - row) {
                var def = BodyDef()
                def.type = .dynamicBody
                def.position = Vec3(Float(column) * 0.85 - 0.425 * Float(7 - row), 1.4 + Float(row) * 0.8, 0)
                let body = world.createBody(def)
                _ = world.createHullShape(body, shapeDef, box)
                boxes.append(body)
            }
        }

        for i in 0..<3 {
            var human = Human()
            createHuman(&human, world, Vec3(Float(i) * 2 - 8, 2, 6), 0.05, 5, 0.5, i + 1, nil, false)
        }

        // A chain mixing joint kinds, with a path joint and a six DOF joint at the end.
        var previous = anchor
        for link in 0..<6 {
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Vec3(10, 6 - Float(link), 0)
            let body = world.createBody(def)
            _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.3, 0.3, 0.3))
            let pivot = Vec3(10, 6.5 - Float(link), 0)
            if link % 2 == 0 {
                var joint = RevoluteJointDef()
                joint.base.bodyIdA = previous
                joint.base.bodyIdB = body
                joint.base.localFrameA = Transform(world.bodyLocalPoint(previous, pivot), .identity)
                joint.base.localFrameB = Transform(world.bodyLocalPoint(body, pivot), .identity)
                joints.append(world.createRevoluteJoint(joint))
            } else {
                var joint = SixDOFJointDef()
                joint.base.bodyIdA = previous
                joint.base.bodyIdB = body
                joint.base.localFrameA = Transform(world.bodyLocalPoint(previous, pivot), .identity)
                joint.base.localFrameB = Transform(world.bodyLocalPoint(body, pivot), .identity)
                joint.lockAll(except: [.rotationX, .rotationY])
                joint[.rotationX].maxFriction = 2
                joints.append(world.createSixDOFJoint(joint))
            }
            previous = body
        }
        var bead = BodyDef()
        bead.type = .dynamicBody
        bead.position = Vec3(-10, 1, -5)
        let beadBody = world.createBody(bead)
        _ = world.createSphereShape(beadBody, ShapeDef(), Sphere(center: Vec3(), radius: 0.2))
        var path = PathJointDef()
        path.base.bodyIdA = anchor
        path.base.bodyIdB = beadBody
        path.path = ConstraintPath(through: [Vec3(-10, 2, -5), Vec3(-6, 3, -5), Vec3(-2, 2, -5), Vec3(2, 4, -5)])
        path.motorMode = .velocity
        path.targetVelocity = 2
        path.maxMotorForce = 50
        path.rotationConstraint = .constrainToPath
        joints.append(world.createPathJoint(path))

        for k in 0..<2 {
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Vec3(Float(k) * 5 - 20, 1.2, -12)
            let body = world.createBody(def)
            var chassis = ShapeDef()
            chassis.density = 400
            _ = world.createHullShape(body, chassis, makeBoxHull(0.9, 0.25, 2))
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
            world.setVehicleDriverInput(id, forward: 1, right: k == 0 ? 0.4 : -0.2)
            vehicles.append(id)
        }

        softBody = world.createSoftBody(SoftBodyDef(
            settings: SoftBodySettings.makeCloth(columns: 10, rows: 10, spacing: 0.15), position: Vec3(0, 6, 3)))

        var bullet = BodyDef()
        bullet.type = .dynamicBody
        bullet.isBullet = true
        bullet.position = Vec3(-25, 2, 0)
        bullet.linearVelocity = Vec3(250, 0, 0)
        _ = world.createSphereShape(world.createBody(bullet), ShapeDef(), Sphere(center: Vec3(), radius: 0.1))

        var characterDef = CharacterDef()
        characterDef.position = Vec3(-4, 0.05, 0)
        character = CharacterVirtual(world: world, characterDef)
    }

    func step() {
        // Walk into the pyramid, pushing boxes.
        character.linearVelocity = Vec3(1.5, character.linearVelocity.y - 10.0 / 60.0, 0)
        character.update(1.0 / 60.0)
        world.step(1.0 / 60.0, subStepCount: 4)
    }

    func save() -> (WorldState, CharacterVirtual.State) { (world.saveState(), character.saveState()) }

    @discardableResult
    func restore(_ state: (WorldState, CharacterVirtual.State)) -> Bool {
        character.restoreState(state.1)
        return world.restoreState(state.0)
    }

    /// Everything observable that should repeat after a restore.
    func fingerprint() -> [UInt64] {
        var values: [UInt64] = [world.worldStateHash(), UInt64(world.awakeBodyCount())]
        var hash: UInt64 = 14_695_981_039_346_656_037
        func mix(_ f: Float) { hash = (hash ^ UInt64(f.bitPattern)) &* 1_099_511_628_211 }
        for p in world.softBodyVertexPositions(softBody) { mix(p.x); mix(p.y); mix(p.z) }
        for vehicle in vehicles {
            let state = world.vehicleDrivetrainState(vehicle)!
            mix(state.engineRPM)
            mix(Float(state.currentGear))
            mix(state.clutchFriction)
        }
        for joint in joints {
            if let force = world.jointConstraintForce(joint) { mix(force.x); mix(force.y); mix(force.z) }
        }
        let p = character.position
        mix(p.x); mix(p.y); mix(p.z)
        mix(character.linearVelocity.x)
        values.append(hash)
        let events = world.contactEvents()
        values.append(UInt64(events.beginEvents.count * 1000 + events.endEvents.count))
        let sensorEvents = world.sensorEvents()
        values.append(UInt64(sensorEvents.begin.count * 1000 + sensorEvents.end.count))
        return values
    }
}

@Suite("World state")
struct WorldStateTests {
    @Test("restoring repeats the following steps bit for bit, any number of times")
    func rollback() {
        let scene = StateScene()
        for _ in 0..<40 { scene.step() }
        let state = scene.save()
        #expect(state.0.stepIndex == scene.world.stepIndex)
        #expect(state.0.byteCount > 0)

        var expected: [[UInt64]] = []
        for _ in 0..<90 {
            scene.step()
            expected.append(scene.fingerprint())
        }
        for round in 0..<2 {
            #expect(scene.restore(state))
            for (i, values) in expected.enumerated() {
                scene.step()
                #expect(scene.fingerprint() == values, "round \(round) step \(i)")
            }
        }
    }

    @Test("bodies, shapes, joints, vehicles, and soft bodies created or destroyed after the save are undone")
    func structuralChanges() {
        let scene = StateScene()
        for _ in 0..<30 { scene.step() }
        let state = scene.save()
        var expected: [[UInt64]] = []
        for _ in 0..<60 {
            scene.step()
            expected.append(scene.fingerprint())
        }

        // Rewind, then change the world heavily.
        scene.restore(state)
        let world = scene.world
        let destroyed = scene.boxes[3]
        world.destroyBody(destroyed)
        world.destroyJoint(scene.joints[1].index1 - 1, true)
        world.destroyJoint(scene.joints.last!.index1 - 1, true)
        world.destroyVehicle(scene.vehicles[0])
        world.destroySoftBody(scene.softBody)
        world.destroyShapeInternal(world.bodies[Int(scene.boxes[5].index1 - 1)].headShapeId, true)
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(0, 8, 0)
        let added = world.createBody(def)
        _ = world.createHullShape(added, ShapeDef(), makeBoxHull(1, 0.2, 0.7))
        var weld = WeldJointDef()
        weld.base.bodyIdA = added
        weld.base.bodyIdB = scene.boxes[0]
        _ = world.createWeldJoint(weld)
        var six = SixDOFJointDef()
        six.base.bodyIdA = scene.anchor
        six.base.bodyIdB = added
        _ = world.createSixDOFJoint(six)
        for _ in 0..<25 { scene.step() }
        #expect(world.bodyIsValid(destroyed) == false)

        #expect(scene.restore(state))
        #expect(world.bodyIsValid(destroyed))
        #expect(world.bodyIsValid(added) == false)
        #expect(world.vehicleIsValid(scene.vehicles[0]))
        #expect(world.softBodyIsValid(scene.softBody))
        for (i, values) in expected.enumerated() {
            scene.step()
            #expect(scene.fingerprint() == values, "step \(i)")
        }
    }

    @Test("worker threads restore the same continuation")
    func threads() {
        let serial = StateScene(workers: 1)
        let parallel = StateScene(workers: 4)
        for _ in 0..<30 {
            serial.step()
            parallel.step()
        }
        let state = parallel.save()
        for _ in 0..<30 {
            serial.step()
            parallel.step()
        }
        let expected = serial.fingerprint()
        #expect(parallel.fingerprint() == expected)
        parallel.restore(state)
        for _ in 0..<30 { parallel.step() }
        #expect(parallel.fingerprint() == expected)
    }

    @Test("a state only restores into its own world")
    func ownership() {
        let a = World()
        let b = World()
        let state = a.saveState()
        #expect(b.restoreState(state) == false)
        #expect(a.restoreState(state))
    }

    @Test("saving and restoring many times keeps the world consistent")
    func cycles() {
        let scene = StateScene()
        for _ in 0..<10 { scene.step() }
        let saved = scene.save()
        scene.step()
        let expected = scene.fingerprint()
        for _ in 0..<20 {
            let temporary = scene.save()
            scene.restore(saved)
            scene.step()
            #expect(scene.fingerprint() == expected)
            scene.restore(temporary)
        }
    }
}
