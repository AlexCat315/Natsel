import Testing
@testable import Natsel

// worldStateHash mirrors b3HashWorldState: FNV-1a over every live body's transform and velocities.
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

    @Test("state hash matches the C reference across a step sequence")
    func stateHash() {
        let world = makeScene()
        #expect(world.worldStateHash() == 0x5fe4_b47a_1d93_4f3f)

        for step in 0..<60 {
            world.step(1.0 / 60.0, subStepCount: 4)
            if step % 20 == 19 {
                let hash = world.worldStateHash()
                switch step {
                case 19: #expect(hash == 0xfbfa_a631_882b_8a7d)
                case 39: #expect(hash == 0xda0c_1b91_6e3d_8327)
                default: #expect(hash == 0xe1c4_3efd_2746_7992)
                }
            }
        }
    }

    @Test("state hash covers positions to float precision")
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

// MARK: - Golden values regenerated from the C reference build

extension WorldStateHashTests {
    @Test("stack scene hashes match the C reference at every checkpoint")
    func stackGolden() {
        let world = World()

        var def = BodyDef()
        def.position = Pos(0, -1, 0)
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(10, 1, 10))

        for i in 0..<3 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 0.6 + 1.05 * Float(i), 0)
            let box = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        }

        #expect(world.worldStateHash() == 0x37cb_0b80_904a_c026)

        var checkpoints: [Int: UInt64] = [
            20: 0x949b_1293_7583_efcd,
            40: 0x59d9_2287_c7d3_f35d,
            60: 0x7e95_9f62_9051_030a,
        ]

        for step in 1...60 {
            world.step(1.0 / 60.0, subStepCount: 4)
            if let expected = checkpoints[step] {
                #expect(world.worldStateHash() == expected, "step \(step)")
                checkpoints.removeValue(forKey: step)
            }
        }

        #expect(checkpoints.isEmpty)
    }

    @Test("an empty world hashes to the seed")
    func emptyWorldGolden() {
        let world = World()
        #expect(world.worldStateHash() == 0xcbf2_9ce4_8422_2325)
    }
}
