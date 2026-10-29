import Testing
@testable import Natsel

@Suite("Dynamic compound instances")
struct DynamicInstanceTests {
    private func body(_ world: World) -> BodyID {
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 2, 0)
        return world.createBody(def)
    }

    @Test("convex compound mass, transforms, and materials match explicit fixtures")
    func massAndQueries() throws {
        let world = World()
        let id = body(world)
        let compound = try #require(makeTestCompound(2))
        let instance = try #require(world.createDynamicCompound(id, ShapeDef(), compound))
        #expect(world.dynamicCompoundIsValid(instance))
        #expect(instance.childShapeIds.count == compound.childCount)
        var expectedMass: Float = 0
        for i in 0..<compound.childCount {
            let child = compound.child(at: i)
            guard case .hull(let hull) = child.geometry else { continue }
            expectedMass += hull.volume * 1000
            let local = world.shapeHull(instance.childShapeIds[i])!
            #expect(distance(local.center, transformPoint(child.transform, hull.center)) < 1.0e-5)
            let material = world.shape(instance.childShapeIds[i]).material(0)
            #expect(material == compound.materials[Int(child.materialIndices[0])])
        }
        #expect(abs(world.bodyMass(id) - expectedMass) < expectedMass * 1.0e-5)
        let hit = world.castRayClosest(Pos(-1, 5, -0.5), Vec3(0, -6, 0))
        #expect(hit.hit)
        #expect(instance.childIndex(of: hit.shapeId) != nil)
    }

    @Test("compound fixtures collide, replace, and restore across structural changes")
    func lifecycle() throws {
        let world = World()
        let ground = world.createBody(BodyDef())
        _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(10, 0.5, 10, Vec3(0, -0.5, 0)))
        let id = body(world)
        let original = try #require(makeTestCompound(2))
        let instance = try #require(world.createDynamicCompound(id, ShapeDef(), original))
        for _ in 0..<120 { world.step(1 / 60, subStepCount: 4) }
        #expect(world.bodyPosition(id).y > -0.5)
        #expect(world.touchingContactCount() > 0)
        let saved = world.saveState()
        let sphere = try #require(makeTestCompound(0))
        let replacement = try #require(world.replaceDynamicCompound(instance, ShapeDef(), sphere))
        #expect(!world.dynamicCompoundIsValid(instance))
        #expect(world.dynamicCompoundIsValid(replacement))
        world.destroyDynamicCompound(replacement)
        #expect(world.body(id).shapeCount == 0)
        #expect(world.restoreState(saved))
        #expect(world.dynamicCompoundIsValid(instance))
        world.step(1 / 60, subStepCount: 4)
    }

    @Test("mesh children are rejected without attaching any shapes")
    func unsupportedGeometryIsAtomic() throws {
        let world = World()
        let id = body(world)
        let mixed = try #require(makeTestCompound(4))
        let convex = try #require(makeTestCompound(0))
        #expect(world.createDynamicCompound(id, ShapeDef(), mixed) == nil)
        #expect(world.body(id).shapeCount == 0)
        let instance = try #require(world.createDynamicCompound(id, ShapeDef(), convex))
        let mass = world.bodyMass(id)
        #expect(world.replaceDynamicCompound(instance, ShapeDef(), mixed) == nil)
        #expect(world.dynamicCompoundIsValid(instance))
        #expect(world.bodyMass(id) == mass)
    }
}
