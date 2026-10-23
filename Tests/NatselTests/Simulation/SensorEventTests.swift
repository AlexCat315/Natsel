import Testing
@testable import Natsel

// A bullet passing through a sensor shape must produce exactly one begin and one end event.
// Ported from test/test_world.c TestSensor.
@Suite("Sensor events")
struct SensorEventTests {
    @Test("bullet through a sensor produces one begin and one end event")
    func bulletThroughSensor() {
        let world = World()

        // Wall from x = 1 to x = 2
        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(1.5, 11, 0)
        let wall = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.enableSensorEvents = true
        _ = world.createHullShape(wall, shapeDef, makeBoxHull(0.5, 10, 1))

        // Bullet fired towards the wall, itself a sensor so events fire.
        def = BodyDef()
        def.type = .dynamicBody
        def.isBullet = true
        def.gravityScale = 0
        def.position = Pos(7.39814, 4, 0)
        def.linearVelocity = Vec3(-20, 0, 0)
        let bullet = world.createBody(def)

        shapeDef = ShapeDef()
        shapeDef.isSensor = true
        shapeDef.enableSensorEvents = true
        _ = world.createSphereShape(bullet, shapeDef, Sphere(center: Vec3(), radius: 0.1))

        var beginCount = 0
        var endCount = 0

        while true {
            world.step(1.0 / 60.0, subStepCount: 4)

            let bulletPos = world.bodyTransform(bullet).p
            let events = world.sensorEvents()

            if events.begin.isEmpty == false { beginCount += 1 }
            if events.end.isEmpty == false { endCount += 1 }

            if bulletPos.x < -1 {
                break
            }
        }

        #expect(beginCount == 1)
        #expect(endCount == 1)
    }

    @Test("a resting shape inside a sensor produces a single begin")
    func restingInsideSensor() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let sensorBody = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.isSensor = true
        shapeDef.enableSensorEvents = true
        _ = world.createHullShape(sensorBody, shapeDef, makeBoxHull(2, 2, 2))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0, 0)
        let ball = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        shapeDef.enableSensorEvents = true
        _ = world.createSphereShape(ball, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        var beginCount = 0
        var endCount = 0
        for _ in 0..<30 {
            world.step(1.0 / 60.0, subStepCount: 4)
            let events = world.sensorEvents()
            beginCount += events.begin.count
            endCount += events.end.count
        }

        // Overlapping from the start: one begin, no end while it stays inside.
        #expect(beginCount == 1)
        #expect(endCount == 0)
    }
}
