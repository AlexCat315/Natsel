import Testing
@testable import Natsel

@Suite("Moving pair CCD")
struct DynamicContinuousTests {
    private func ball(_ world: World, _ x: Double, _ velocity: Float, bullet: Bool,
                      type: BodyType = .dynamicBody, customFilter: Bool = false,
                      density: Float = 1000, restitution: Float = 0) -> BodyID {
        var def = BodyDef()
        def.type = type
        def.position = Pos(x, 0, 0)
        def.linearVelocity = Vec3(velocity, 0, 0)
        def.isBullet = bullet
        let id = world.createBody(def)
        var shape = ShapeDef()
        shape.enableCustomFiltering = customFilter
        shape.density = density
        shape.baseMaterial.restitution = restitution
        _ = world.createSphereShape(id, shape, Sphere(center: .zero, radius: 0.1))
        return id
    }

    @Test("precision capability agrees with fractional world integration")
    func precisionCapability() {
        #expect(isDoublePrecision())
        let world = World()
        world.gravity = .zero
        let id = ball(world, 1.0e12 + 0.125, 0.25, bullet: false)
        world.step(0.5, subStepCount: 1)
        #expect(abs(world.bodyPosition(id).x - (1.0e12 + 0.25)) < 0.001)
    }

    @Test("crossing swept paths stop both movers", arguments: [false, true])
    func crossing(_ bullet: Bool) {
        for reverse in [false, true] {
            let world = World()
            world.gravity = .zero
            world.enableSleep = false
            world.enableDynamicContinuous = true
            let first = ball(world, reverse ? 2 : -2, reverse ? -240 : 240, bullet: bullet)
            let second = ball(world, reverse ? -2 : 2, reverse ? 240 : -240, bullet: bullet)
            let left = reverse ? second : first
            let right = reverse ? first : second
            for _ in 0..<8 {
                world.step(1 / 60, subStepCount: 1)
                #expect(world.bodyPosition(left).x < world.bodyPosition(right).x)
            }
        }
    }

    @Test("moving kinematic body is included in the sweep")
    func kinematic() {
        let world = World()
        world.gravity = .zero
        world.enableDynamicContinuous = true
        let dynamic = ball(world, -2, 240, bullet: false)
        let platform = ball(world, 2, -240, bullet: false, type: .kinematicBody)
        world.step(1 / 60, subStepCount: 1)
        #expect(world.bodyPosition(dynamic).x < world.bodyPosition(platform).x)
        #expect(abs(world.bodyPosition(platform).x + 2) < 0.001)
    }

    @Test("dynamic CCD respects filters and survives rollback")
    func filteringAndRollback() {
        let world = World()
        world.gravity = .zero
        world.enableDynamicContinuous = true
        let left = ball(world, -2, 240, bullet: true, customFilter: true)
        let right = ball(world, 2, -240, bullet: true, customFilter: true)
        world.setCustomFilter { _, _ in false }
        let state = world.saveState()
        world.step(1 / 60, subStepCount: 1)
        #expect(world.bodyPosition(left).x > world.bodyPosition(right).x)
        world.enableDynamicContinuous = false
        #expect(world.restoreState(state))
        #expect(world.enableDynamicContinuous)
    }

    @Test("CCD reports more than eight crossed sensors")
    func manySensors() {
        let world = World()
        world.gravity = .zero
        for i in 0..<12 {
            var def = BodyDef()
            def.position = Pos(-1.65 + Double(i) * 0.3, 0, 0)
            let sensor = world.createBody(def)
            var shape = ShapeDef()
            shape.isSensor = true
            shape.enableSensorEvents = true
            _ = world.createHullShape(sensor, shape, makeBoxHull(0.01, 1, 1))
        }
        let mover = ball(world, -2, 240, bullet: true)
        let shape = world.makeShapeID(world.body(mover).headShapeId)
        world.enableShapeSensorEvents(shape, true)
        world.step(1 / 60, subStepCount: 1)
        #expect(Set(world.sensorBeginEvents.map { $0.sensorShapeId }).count == 12)
    }

    @Test("rotating thin hull queries the interior of its arc")
    func rotationArc() {
        let world = World()
        world.gravity = .zero
        world.enableDynamicContinuous = true
        let obstacle = world.createBody(BodyDef())
        _ = world.createSphereShape(obstacle, ShapeDef(), Sphere(center: Vec3(0, 0.9, 0), radius: 0.03))
        var def = BodyDef()
        def.type = .dynamicBody
        def.rotation = makeQuatFromAxisAngle(.axisZ, Float.pi / 4)
        def.angularVelocity = Vec3(0, 0, 120)
        def.allowFastRotation = true
        let rotor = world.createBody(def)
        _ = world.createHullShape(rotor, ShapeDef(), makeBoxHull(1, 0.01, 0.01))
        world.step(1 / 60, subStepCount: 1)
        #expect((world.bodySim(world.body(rotor)).flags & BodyFlags.hadTimeOfImpact.rawValue) != 0)
    }

    @Test("impact conserves momentum and wakes a sleeping target", arguments: [false, true])
    func momentum(_ sleeping: Bool) {
        let world = World()
        world.gravity = .zero
        world.enableDynamicContinuous = true
        let left = ball(world, -2, 240, bullet: true, restitution: 1)
        let right = ball(world, 0, 0, bullet: true, density: 2000, restitution: 1)
        if sleeping { world.setBodyAwake(right, false) }
        world.enableShapeHitEvents(world.makeShapeID(world.body(left).headShapeId), true)
        let ma = world.bodyMass(left)
        let mb = world.bodyMass(right)
        let momentum = ma * 240
        let energy = 0.5 * ma * 240 * 240
        world.step(1 / 60, subStepCount: 1)
        #expect(world.isBodyAwake(right))
        let va = world.bodyLinearVelocity(left).x
        let vb = world.bodyLinearVelocity(right).x
        #expect(abs(ma * va + mb * vb - momentum) < momentum * 0.0001)
        #expect(abs(0.5 * (ma * va * va + mb * vb * vb) - energy) < energy * 0.0001)
        #expect(va < 0 && vb > 0)
        #expect(world.contactHitEvents.count == 1)
    }

    @Test("moving-pair results agree across workers at a distant origin")
    func workerParity() {
        func run(_ workers: Int32) -> [Pos] {
            let world = World()
            world.gravity = .zero
            world.workerCount = workers
            world.enableDynamicContinuous = true
            var bodies: [BodyID] = []
            for i in 0..<32 {
                let offset = 1.0e12 + Double(i) * 10
                bodies.append(ball(world, offset - 2, 240, bullet: true))
                bodies.append(ball(world, offset + 2, -240, bullet: true))
            }
            for _ in 0..<10 { world.step(1 / 60, subStepCount: 1) }
            return bodies.map { world.bodyPosition($0) }
        }
        #expect(run(1) == run(4))
    }
}
