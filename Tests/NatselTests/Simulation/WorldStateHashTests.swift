import Testing
@testable import Natsel

// worldStateHash uses FNV-1a over every live body's transform and velocities.
@Suite("World state hash")
struct WorldStateHashTests {
    private func makeScene() -> World {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        for i in 0..<3 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0.3 * Float(i), 0.6 + Float(i) * 1.1, 0.1 * Float(i))
            let box = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        }

        return world
    }

    @Test("independent scenes produce the same state hash at every step")
    func stateHash() {
        let first = makeScene()
        let replay = makeScene()
        #expect(first.worldStateHash() == replay.worldStateHash())
        for _ in 0..<60 {
            first.step(1.0 / 60.0, subStepCount: 4)
            replay.step(1.0 / 60.0, subStepCount: 4)
            #expect(first.worldStateHash() == replay.worldStateHash())
        }
    }

    @Test("state hash covers positions to double precision")
    func stateHashPrecision() {
        let world = makeScene()
        let before = world.worldStateHash()

        // A one-ULP change in a single body position must change the hash.
        let id = world.makeBodyID(3)
        let transform = world.bodyTransform(id)
        var moved = transform.p
        moved.x = moved.x.nextUp
        world.setBodyTransform(id, moved, transform.q)

        #expect(world.worldStateHash() != before)
    }
}

extension WorldStateHashTests {
    @Test("an empty world hashes to the FNV-1a seed")
    func emptyWorld() {
        #expect(World().worldStateHash() == 0xcbf2_9ce4_8422_2325)
    }
}
