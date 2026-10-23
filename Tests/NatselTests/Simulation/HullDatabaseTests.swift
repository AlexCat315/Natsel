import Testing
@testable import Natsel

// Identical hull data is shared through a reference counted world database, ported from
// test/test_world.c TestHullDatabase.
@Suite("Hull database")
struct HullDatabaseTests {
    @Test("identical hulls share one interned copy")
    func deduplication() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        let bodyA = world.createBody(def)
        let bodyB = world.createBody(def)
        let shapeDef = ShapeDef()

        // Two shapes built from identical data share one owned copy.
        let shapeA = world.createHullShape(bodyA, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        let shapeB = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        #expect(world.hullDatabaseCount == 1)

        // A box built on an independent stack frame must de-duplicate to the same shared copy.
        // This only holds if content hashing sees deterministic padding bytes.
        let bodyC = world.createBody(def)
        let shapeC = world.createHullShape(bodyC, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        #expect(world.hullDatabaseCount == 1)

        world.destroyShape(shapeC, updateBodyMass: false)
        #expect(world.hullDatabaseCount == 1)

        // A different size is a separate entry.
        let bodyD = world.createBody(def)
        let shapeD = world.createHullShape(bodyD, shapeDef, makeBoxHull(0.3, 0.3, 0.3))
        #expect(world.hullDatabaseCount == 2)
        world.destroyShape(shapeD, updateBodyMass: false)
        #expect(world.hullDatabaseCount == 1)

        // Releasing one reference keeps the other alive.
        world.destroyShape(shapeA, updateBodyMass: false)
        #expect(world.hullDatabaseCount == 1)

        world.destroyShape(shapeB, updateBodyMass: false)
        #expect(world.hullDatabaseCount == 0)
    }

    @Test("destroying bodies drains the database")
    func destroyBodyDrains() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        let bodies = (0..<4).map { _ in world.createBody(def) }

        let shapeDef = ShapeDef()
        for body in bodies {
            _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        }
        #expect(world.hullDatabaseCount == 1)

        for body in bodies {
            world.destroyBody(body)
        }
        #expect(world.hullDatabaseCount == 0)
    }

    @Test("interning is content keyed across many distinct hulls")
    func distinctHulls() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        let shapeDef = ShapeDef()

        // Ten different sizes, each added twice.
        for i in 1...10 {
            let half = Float(i) * 0.1
            for _ in 0..<2 {
                let body = world.createBody(def)
                _ = world.createHullShape(body, shapeDef, makeBoxHull(half, half, half))
            }
        }

        #expect(world.hullDatabaseCount == 10)
    }
}
