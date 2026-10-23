import Testing
@testable import Natsel

@Suite("Mesh simulation C parity")
struct MeshSimulationTests {
    @Test("sphere, capsule, and hull trajectories and all contact frames match C", arguments: 0..<27)
    func simulation(scenario: Int) throws {
        let world = World()
        world.enableSleep = scenario >= 18
        var bodyDef = BodyDef()
        bodyDef.rotation = makeQuatFromAxisAngle(.axisY, 0.2)
        let ground = world.createBody(bodyDef)
        let geometry: TriangleMesh?
        switch (scenario % 9) / 3 {
        case 0: geometry = createGridMesh(xCount: 8, zCount: 8, cellWidth: 0.5, materialCount: 3, identifyEdges: true)
        case 1: geometry = createWaveMesh(xCount: 8, zCount: 8, cellWidth: 0.5, amplitude: 0.25, rowFrequency: 0.4, columnFrequency: 0.3)
        default: geometry = createPlatformMesh(center: Vec3(0, -0.25, 0), height: 0.5, topWidth: 3, bottomWidth: 4)
        }
        let mesh = try #require(geometry)
        var shapeDef = ShapeDef()
        shapeDef.materials = (0..<3).map { i in
            var material = SurfaceMaterial.defaultMaterial
            material.friction = 0.2 + 0.2 * Float(i)
            material.restitution = 0.05 * Float(i)
            material.tangentVelocity = Vec3(Float(i - 1) * 0.05, 0, 0.02 * Float(i))
            material.userMaterialId = UInt64(10 + i)
            return material
        }
        _ = world.createMeshShape(ground, shapeDef, mesh, scale: (scenario % 9) / 3 == 2 ? Vec3(-1.25, 1, 0.75) : .one)
        bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Vec3(0.13, 2, 0.17)
        bodyDef.rotation = makeQuatFromAxisAngle(.axisY, 0.3)
        bodyDef.linearVelocity = Vec3(0.35, -0.1, 0.2)
        bodyDef.angularVelocity = Vec3(0.1, 0.2, 0.15)
        if (9..<18).contains(scenario) { bodyDef.linearVelocity.y = -150; bodyDef.isBullet = true }
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
        var bodies = PhysicsReferenceDigest()
        var contacts = PhysicsReferenceDigest()
        for step in 0..<180 {
            world.step(1 / 60, subStepCount: 4)
            let transform = world.bodyTransform(body)
            bodies.add(transform.p); bodies.add(transform.q.v); bodies.add(transform.q.s)
            bodies.add(world.bodyLinearVelocity(body)); bodies.add(world.bodyAngularVelocity(body))
            contacts.addContacts(world)
            if (step + 1) % 30 == 0 {
                let expected = meshSimulationReference[scenario][step / 30]
                #expect(bodies.value == expected.body, "scenario \(scenario), step \(step + 1)")
                #expect(contacts.value == expected.contact, "scenario \(scenario), step \(step + 1)")
            }
        }
    }
}

