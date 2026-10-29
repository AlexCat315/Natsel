import Testing
@testable import Natsel

@Suite("Wide equivalence")
struct WideEquivalenceTests {
    private func scene(_ wide: Bool) -> UInt64 {
        let world = World()
        world.enableSleep = false
        world.enableWideSolver = wide

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, -1, 0)
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(10, 1, 10))

        // A small grid so several contacts share a color.
        for x in 0..<3 {
            for z in 0..<3 {
                def = BodyDef()
                def.type = .dynamicBody
                def.position = Pos(Float(x) * 1.02 - 1, 0.6 + Float(z) * 0.1, Float(z) * 1.02 - 1)
                let box = world.createBody(def)
                var shapeDef = ShapeDef()
                shapeDef.density = 1000
                _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
            }
        }

        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
        return world.worldStateHash()
    }

    @Test("wide and scalar paths agree")
    func wideMatchesScalar() {
        let scalar = scene(false)
        let wide = scene(true)
        #expect(scalar == wide, "scalar \(String(scalar, radix: 16)) wide \(String(wide, radix: 16))")
    }
}

@Suite("Wide multi-manifold equivalence")
struct WideMultiManifoldEquivalenceTests {
    @Test("terrain and mesh batches agree at every step", arguments: [false, true])
    func multiManifold(_ mesh: Bool) throws {
        let scalar = World(), wide = World()
        scalar.enableWideSolver = false
        wide.enableWideSolver = true
        if mesh {
            _ = createMeshDrop(scalar, Pos())
            _ = createMeshDrop(wide, Pos())
        } else {
            _ = createWavePile(scalar)
            _ = createWavePile(wide)
        }
        for step in 0..<300 {
            scalar.step(1 / 60, subStepCount: 4)
            wide.step(1 / 60, subStepCount: 4)
            try #require(scalar.worldStateHash() == wide.worldStateHash(), "step \(step), mesh \(mesh)")
        }
    }
}

@Suite("Wide restitution and kinematic equivalence")
struct WideRestitutionEquivalenceTests {
    @Test("mixed bounce, friction, warm starting, and partial batches agree", arguments: [false, true])
    func mixedContacts(_ propagation: Bool) throws {
        func makeWorld(_ enableWide: Bool) -> World {
            let world = World()
            world.enableWideSolver = enableWide
            world.enableSleep = false
            world.enableRestitutionPropagation = propagation
            var groundDef = BodyDef()
            groundDef.type = .kinematicBody
            groundDef.position = Pos(0, -1, 0)
            groundDef.linearVelocity = Vec3(0.15, 0, -0.1)
            let ground = world.createBody(groundDef)
            _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(20, 1, 20))
            for i in 0..<11 {
                var def = BodyDef()
                def.type = .dynamicBody
                def.position = Pos(Float(i % 4) * 2 - 3, 0.6 + Float(i % 3) * 0.05, Float(i / 4) * 2 - 2)
                def.linearVelocity = Vec3(0.1, -3, -0.2)
                def.angularVelocity = Vec3(0.2, 0.1, -0.1)
                let body = world.createBody(def)
                var shape = ShapeDef()
                shape.baseMaterial.restitution = Float(i % 3) * 0.4
                shape.baseMaterial.rollingResistance = i % 2 == 0 ? 0.2 : 0
                shape.baseMaterial.tangentVelocity = Vec3(0.3, 0, -0.2)
                if i % 2 == 0 {
                    _ = world.createSphereShape(body, shape, Sphere(center: Vec3(), radius: 0.5))
                } else {
                    _ = world.createHullShape(body, shape, makeBoxHull(0.5, 0.5, 0.5))
                }
            }
            return world
        }
        let scalar = makeWorld(false), wide = makeWorld(true)
        var sawWide = false, sawPartialBatch = false
        for step in 0..<100 {
            scalar.enableWarmStarting = step % 15 != 0
            wide.enableWarmStarting = scalar.enableWarmStarting
            scalar.step(1 / 60, subStepCount: 4)
            wide.step(1 / 60, subStepCount: 4)
            for color in wide.solverContacts.wideConstraints {
                sawWide = sawWide || !color.isEmpty
                sawPartialBatch = sawPartialBatch || color.contains { $0.contactIds[3] < 0 }
            }
            try #require(scalar.worldStateHash() == wide.worldStateHash(), "step \(step)")
            for id in scalar.contacts.indices {
                try #require(scalar.contacts[id].manifolds == wide.contacts[id].manifolds, "step \(step) contact \(id)")
            }
        }
        #expect(sawWide)
        #expect(sawPartialBatch)
    }
}
