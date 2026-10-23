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

private func contactHash(_ world: World) -> (UInt64, Int) {
    var h: UInt64 = 14_695_981_039_346_656_037
    var count = 0
    for c in world.contacts where c.contactId != B3.nullIndex {
        count += 1
        var contactId = c.contactId
        var shapeIdA = c.shapeIdA
        var shapeIdB = c.shapeIdB
        var childIndex = c.childIndex
        var setIndex = c.setIndex
        var flags = c.flags
        var encodedA = c.encodedBodySimA
        var encodedB = c.encodedBodySimB
        var rolling = c.rollingResistance
        var pairKey = shapePairKey(c.shapeIdA, c.shapeIdB, c.childIndex)

        h = fnv(h, &contactId, 4)
        h = fnv(h, &shapeIdA, 4)
        h = fnv(h, &shapeIdB, 4)
        h = fnv(h, &childIndex, 4)
        h = fnv(h, &setIndex, 4)
        h = fnv(h, &flags, 4)
        h = fnv(h, &encodedA, 4)
        h = fnv(h, &encodedB, 4)
        h = fnv(h, &rolling, 4)
        h = fnv(h, &pairKey, 8)
    }
    return (h, count)
}

private func bodyHash(_ world: World) -> UInt64 {
    var h: UInt64 = 14_695_981_039_346_656_037
    for b in world.bodies where b.id != B3.nullIndex {
        var id = b.id
        var head = b.headContactKey
        var count = b.contactCount
        h = fnv(h, &id, 4)
        h = fnv(h, &head, 4)
        h = fnv(h, &count, 4)
    }
    return h
}

@Suite("Broad phase pairs")
struct PairTests {
    @Test("pair generation matches the C reference")
    func generatePairs() {
        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 0
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        let positions = [Vec3(0, 1, 0), Vec3(0.5, 1, 0), Vec3(0, 2, 0)]
        var dynamicBodies: [BodyID] = []
        for p in positions {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = p
            let body = world.createBody(def)
            dynamicBodies.append(body)

            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 0.5))
        }

        world.updateBroadPhasePairs()

        var (h, count) = contactHash(world)
        #expect(count == 5)
        #expect(h == 0x0332_3114_2496_bd0a)
        #expect(bodyHash(world) == 0x559c_bc2b_0bc6_6950)

        for (i, bodyId) in dynamicBodies.enumerated() {
            let b = world.body(bodyId)
            switch i {
            case 0:
                #expect(b.headContactKey == 6)
                #expect(b.contactCount == 3)
            case 1:
                #expect(b.headContactKey == 8)
                #expect(b.contactCount == 3)
            default:
                #expect(b.headContactKey == 9)
                #expect(b.contactCount == 2)
            }
        }

        #expect(world.solverSets[2].contactIndices.count == 5)
        #expect(world.solverSets[1].contactIndices.count == 0)

        // A second update must not create duplicate contacts.
        world.updateBroadPhasePairs()
        (h, count) = contactHash(world)
        #expect(count == 5)
        #expect(h == 0x0332_3114_2496_bd0a)
        #expect(bodyHash(world) == 0x559c_bc2b_0bc6_6950)
    }
}
