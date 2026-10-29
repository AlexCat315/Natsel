import Testing
@testable import Natsel

@Suite("Sensor repeatability")
struct SensorRepeatabilityTests {
    @Test("all six sensor geometry types preserve ordered begin/end events", arguments: 0..<2)
    func repeatability(scenario: Int) throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let (world, origin) = try makeTestQueryScene(scenario, sensors: true)
            var bodyDef = BodyDef()
            bodyDef.type = .kinematicBody
            let visitor = world.createBody(bodyDef)
            var shapeDef = ShapeDef()
            shapeDef.enableSensorEvents = true
            _ = world.createSphereShape(visitor, shapeDef, Sphere(center: .zero, radius: 0.3))
            var digest = CollisionDigest()
            for i in 0..<120 {
                let position = Vec3(Float((i * 7) % 19) * 0.25 - 1, Float(i % 9) * 0.25 - 0.5,
                                    Float((i * 11) % 17) * 0.25 - 0.75)
                world.setBodyTransform(visitor, origin + position, .identity)
                world.step(1 / 60, subStepCount: 4)
                let events = world.sensorEvents()
                digest.add(UInt32(events.begin.count)); digest.add(UInt32(events.end.count))
                for event in events.begin {
                    digest.add(UInt32(event.sensorShapeId.index1)); digest.add(UInt32(event.visitorShapeId.index1))
                }
                for event in events.end {
                    digest.add(UInt32(event.sensorShapeId.index1)); digest.add(UInt32(event.visitorShapeId.index1))
                }
            }
            samples.append(UInt64(digest.value))
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}

@Suite("Sensor end-event lifetime")
struct SensorEndEventLifetimeTests {
    @Test("removing a sensor preserves the moved sensor index and subsequent destruction")
    func swapRemoval() {
        let world = World()
        let body = world.createBody(BodyDef())
        var definition = ShapeDef()
        definition.isSensor = true
        let ids = (0..<4).map { _ in world.createSphereShape(body, definition, Sphere(center: .zero, radius: 1)) }
        world.destroyShape(ids[0], updateBodyMass: true)
        for (index, sensor) in world.sensors.enumerated() {
            #expect(world.shapes[Int(sensor.shapeId)].sensorIndex == Int32(index))
        }
        world.destroyShape(ids[2], updateBodyMass: true)
        world.destroyShape(ids[3], updateBodyMass: true)
        #expect(world.sensors.count == 1)
        #expect(world.shape(ids[1]).sensorIndex == 0)
    }

    @Test("destroyed sensor publishes its end event on the next step only")
    func destruction() {
        let world = World()
        let ground = world.createBody(BodyDef())
        var shapeDef = ShapeDef()
        shapeDef.isSensor = true
        shapeDef.enableSensorEvents = true
        let sensor = world.createSphereShape(ground, shapeDef, Sphere(center: .zero, radius: 1))
        var definition = BodyDef()
        definition.type = .kinematicBody
        let visitorBody = world.createBody(definition)
        shapeDef.isSensor = false
        let visitor = world.createSphereShape(visitorBody, shapeDef, Sphere(center: .zero, radius: 0.5))
        world.step(1 / 60, subStepCount: 4)
        #expect(world.sensorEvents().begin.count == 1)
        world.destroyShape(sensor, updateBodyMass: true)
        #expect(world.sensorEvents().end.isEmpty)
        world.step(1 / 60, subStepCount: 4)
        let events = world.sensorEvents()
        #expect(events.begin.isEmpty)
        #expect(events.end.count == 1)
        #expect(events.end.first?.sensorShapeId == sensor)
        #expect(events.end.first?.visitorShapeId == visitor)
        world.step(1 / 60, subStepCount: 4)
        #expect(world.sensorEvents().end.isEmpty)
    }
}
