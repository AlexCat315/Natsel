import Testing
@testable import Natsel

private func fnv(_ h: UInt64, _ bytes: UnsafeRawPointer, _ n: Int) -> UInt64 {
    var h = h
    let p = bytes.assumingMemoryBound(to: UInt8.self)
    for i in 0..<n {
        h ^= UInt64(p[i])
        h = h &* 1_099_511_628_211
    }
    return h
}

private func islandHash(_ world: World) -> UInt64 {
    var ih: UInt64 = 14_695_981_039_346_656_037
    for island in world.islands where island.islandId != B3.nullIndex {
        var islandId = island.islandId
        var setIndex = island.setIndex
        ih = fnv(ih, &islandId, 4)
        ih = fnv(ih, &setIndex, 4)
        for bodyId in island.bodies { var b = bodyId; ih = fnv(ih, &b, 4) }
        for link in island.contacts { var c = link.contactId; ih = fnv(ih, &c, 4) }
    }
    return ih
}

@Suite("Island split")
struct IslandSplitTests {
    @Test("union-find split matches the C reference")
    func splitIsland() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        var boxes: [BodyID] = []
        for i in 0..<2 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 0.55 + Float(i) * 1.01, 0)
            let box = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
            boxes.append(box)
        }

        for _ in 0..<30 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        #expect(world.islands.count == 2)
        let before = islandHash(world)
        #expect(before == 0x5949_14da_eac8_6cc4)

        // Unlink the first touching contact from its island.
        var targetContact: Int32 = B3.nullIndex
        var targetIsland: Int32 = B3.nullIndex
        for c in world.contacts where c.contactId != B3.nullIndex {
            if c.islandId != B3.nullIndex {
                targetContact = c.contactId
                targetIsland = c.islandId
                break
            }
        }
        #expect(targetContact == 0)
        #expect(targetIsland == 0)

        world.unlinkContact(targetContact)
        world.splitIsland(targetIsland)

        var h: UInt64 = islandHash(world)
        for b in world.bodies where b.id != B3.nullIndex {
            var islandId = b.islandId
            var islandIndex = b.islandIndex
            h = fnv(h, &islandId, 4)
            h = fnv(h, &islandIndex, 4)
        }

        #expect(h == 0x7e3c_3db3_a79b_2a9e)
    }
}
