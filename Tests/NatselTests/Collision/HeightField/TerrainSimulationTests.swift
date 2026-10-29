import Testing
@testable import Natsel

@Suite("Height-field simulation repeatability")
struct HeightFieldSimulationTests {
    @Test("sphere, capsule, and hull trajectories and all contact frames are repeatable", arguments: 0..<54)
    func simulation(scenario: Int) throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            world.enableSleep = scenario >= 36
            var bodyDef = BodyDef()
            bodyDef.rotation = makeQuatFromAxisAngle(.axisY, 0.2)
            bodyDef.position = Pos(Float(-2.25), 0, Float(-1.25))
            let ground = world.createBody(bodyDef)
            let field = try #require(makeTestHeightField((scenario % 18) / 3))
            var shapeDef = ShapeDef()
            shapeDef.materials = (0..<3).map { i in
                var material = SurfaceMaterial.defaultMaterial
                material.friction = 0.2 + 0.2 * Float(i)
                material.restitution = 0.05 * Float(i)
                material.tangentVelocity = Vec3(Float(i - 1) * 0.05, 0, 0.02 * Float(i))
                material.userMaterialId = UInt64(10 + i)
                return material
            }
            _ = world.createHeightFieldShape(ground, shapeDef, field)
            bodyDef = BodyDef()
            bodyDef.type = .dynamicBody
            bodyDef.position = Pos(Float(0.13), 2, Float(0.17))
            bodyDef.rotation = makeQuatFromAxisAngle(.axisY, 0.3)
            bodyDef.linearVelocity = Vec3(0.35, -0.1, 0.2)
            bodyDef.angularVelocity = Vec3(0.1, 0.2, 0.15)
            if (18..<36).contains(scenario) { bodyDef.linearVelocity.y = -150; bodyDef.isBullet = true }
            let body = world.createBody(bodyDef)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            shapeDef.baseMaterial.friction = 0.7
            shapeDef.baseMaterial.restitution = 0.15
            shapeDef.baseMaterial.rollingResistance = 0.05
            switch scenario % 3 {
            case 0: _ = world.createSphereShape(body, shapeDef, Sphere(center: .zero, radius: 0.4))
            case 1: _ = world.createCapsuleShape(body, shapeDef, Capsule(center1: Vec3(0, -0.35, 0), center2: Vec3(0, 0.35, 0), radius: 0.3))
            default: _ = world.createHullShape(body, shapeDef, makeBoxHull(0.4, 0.4, 0.4))
            }
            var bodies = PhysicsDigest()
            var contacts = PhysicsDigest()
            for _ in 0..<180 {
                world.step(1 / 60, subStepCount: 4)
                let transform = world.bodyTransform(body)
                bodies.add(transform.p); bodies.add(transform.q.v); bodies.add(transform.q.s)
                bodies.add(world.bodyLinearVelocity(body)); bodies.add(world.bodyAngularVelocity(body))
                contacts.addContacts(world)
                samples.append(bodies.value)
                samples.append(contacts.value)
            }
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(first.count == 360)
        #expect(first == replay)
    }
}

