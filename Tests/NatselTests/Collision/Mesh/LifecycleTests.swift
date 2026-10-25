import Testing
@testable import Natsel

@Suite("Mesh contact lifecycle")
struct MeshContactLifecycleTests {
    @Test("disjoint, shape removal, and body removal unlink mesh constraints", arguments: 0..<5)
    func removal(action: Int) throws {
        let world = World()
        world.enableSleep = action >= 3
        let ground = world.createBody(BodyDef())
        let geometry = try #require(createGridMesh(xCount: 4, zCount: 4, cellWidth: 1, identifyEdges: true))
        _ = world.createMeshShape(ground, ShapeDef(), geometry)
        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(Float(0.13), Float(0.39), Float(0.17))
        let body = world.createBody(bodyDef)
        let shape = world.createSphereShape(body, ShapeDef(), Sphere(center: .zero, radius: 0.4))
        world.step(1 / 60, subStepCount: 4)
        #expect(world.contacts.contains { $0.contactId != B3.nullIndex && $0.manifoldCount > 0 })
        if action >= 3 {
            for _ in 0..<180 { world.step(1 / 60, subStepCount: 4) }
            #expect(!world.isBodyAwake(world.body(body)))
        }
        switch action {
        case 0: world.setBodyTransform(body, Pos(20, 20, 20), .identity)
        case 1, 3: world.destroyShape(shape, updateBodyMass: true)
        default: world.destroyBody(body)
        }
        world.step(1 / 60, subStepCount: 4)
        #expect(world.contacts.allSatisfy { $0.contactId == B3.nullIndex })
        #expect(world.constraintGraph.colors.allSatisfy { $0.contacts.isEmpty && $0.convexContacts.isEmpty })
        // Reuse the freed IDs and advance again to catch stale graph/island references.
        bodyDef.position = Pos(10, 10, 10)
        let replacement = world.createBody(bodyDef)
        _ = world.createSphereShape(replacement, ShapeDef(), Sphere(center: .zero, radius: 0.4))
        world.step(1 / 60, subStepCount: 4)
        #expect(world.contacts.allSatisfy { $0.contactId == B3.nullIndex })
    }
}
