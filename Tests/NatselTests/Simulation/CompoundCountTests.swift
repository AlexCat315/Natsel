import Testing
@testable import Natsel

// The compound shape count lives on the world and is decremented from two different teardown
// paths. A leak pins the broad-phase on the slow per-candidate compound path forever.
@Suite("Compound shape count")
struct CompoundCountTests {
    private func makeCompound() -> Compound? {
        var definition = CompoundDef()
        var item = CompoundHullDef()
        item.hull = makeBoxHull(1, 1, 1)
        item.transform = Transform(Vec3(), Quat.identity)
        item.material = SurfaceMaterial.defaultMaterial
        definition.hulls.append(item)
        return Compound(definition)
    }

    @Test("destroying a compound shape releases the count")
    func destroyShapeReleases() {
        let world = World()
        #expect(world.compoundShapeCountValue == 0)

        guard let compound = makeCompound() else { return }

        var def = BodyDef()
        def.type = .staticBody
        let body = world.createBody(def)
        let shapeDef = ShapeDef()

        // Shape destroyed on its own
        let shape = world.createCompoundShape(body, shapeDef, compound)
        #expect(world.compoundShapeCountValue == 1)

        world.destroyShape(shape, updateBodyMass: false)
        #expect(world.compoundShapeCountValue == 0)

        world.destroyBody(body)
    }

    @Test("destroying the owning body releases the count")
    func destroyBodyReleases() {
        let world = World()
        guard let compound = makeCompound() else { return }

        var def = BodyDef()
        def.type = .staticBody
        let body = world.createBody(def)
        let shapeDef = ShapeDef()

        // Shape carried away by its body
        _ = world.createCompoundShape(body, shapeDef, compound)
        #expect(world.compoundShapeCountValue == 1)

        world.destroyBody(body)
        #expect(world.compoundShapeCountValue == 0)
    }

    @Test("count tracks several compounds across bodies")
    func multipleCompounds() {
        let world = World()
        guard let compound = makeCompound() else { return }

        var bodies: [BodyID] = []
        for _ in 0..<3 {
            var def = BodyDef()
            def.type = .staticBody
            let body = world.createBody(def)
            _ = world.createCompoundShape(body, ShapeDef(), compound)
            bodies.append(body)
        }
        #expect(world.compoundShapeCountValue == 3)

        world.destroyBody(bodies[1])
        #expect(world.compoundShapeCountValue == 2)

        for body in [bodies[0], bodies[2]] {
            world.destroyBody(body)
        }
        #expect(world.compoundShapeCountValue == 0)
    }
}
