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

@Suite("Constraint graph")
struct GraphTests {
    @Test("graph coloring and waking match the C reference")
    func graphAndWake() {
        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 0
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        let positions = [Vec3(0, 0.9, 0), Vec3(0.6, 0.9, 0), Vec3(0, 2.0, 0)]
        for p in positions {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = p
            let body = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 0.5))
        }

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(-0.9, 0.9, 0)
        let boxBody = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(boxBody, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        world.updateBroadPhasePairs()

        for cid in world.solverSets[2].contactIndices {
            let contact = world.contacts[Int(cid)]
            let shapeA = world.shapes[Int(contact.shapeIdA)]
            let shapeB = world.shapes[Int(contact.shapeIdB)]
            let simA = world.bodySimFor(world.makeBodyID(shapeA.bodyId))
            let simB = world.bodySimFor(world.makeBodyID(shapeB.bodyId))
            var store = ContactStore(&world.contacts)
            world.updateContact(cid, simA.localCenter, simA.transform, simB.localCenter, simB.transform, false, &store)
        }

        for cid in world.solverSets[2].contactIndices {
            let c = world.contacts[Int(cid)]
            if (c.flags & ContactFlags.simTouching.rawValue) != 0 {
                world.linkContact(cid)
                world.addContactToGraph(cid)
            }
        }

        var gh: UInt64 = 14_695_981_039_346_656_037
        for c in world.contacts where c.contactId != B3.nullIndex {
            var colorIndex = c.colorIndex
            var localIndex = c.localIndex
            var encodedA = c.encodedBodySimA
            var encodedB = c.encodedBodySimB
            gh = fnv(gh, &colorIndex, 4)
            gh = fnv(gh, &localIndex, 4)
            gh = fnv(gh, &encodedA, 4)
            gh = fnv(gh, &encodedB, 4)
        }
        for i in 0..<Int(B3.graphColorCount) {
            let color = world.constraintGraph.colors[i]
            var convexCount = Int32(color.convexContacts.count)
            var scalarCount = Int32(color.contacts.count)
            var jointCount = Int32(color.jointSims.count)
            gh = fnv(gh, &convexCount, 4)
            gh = fnv(gh, &scalarCount, 4)
            gh = fnv(gh, &jointCount, 4)
            for var id in color.convexContacts { gh = fnv(gh, &id, 4) }
            for spec in color.contacts { var id = spec.contactId; gh = fnv(gh, &id, 4) }
        }
        #expect(gh == 0x68d8_2c8c_bf4a_f21b)

        // Verify per-contact colors and indices.
        var touchingContacts: [(Int32, Int32, Int32, Int32, Int32, Int32)] = []
        for c in world.contacts where c.contactId != B3.nullIndex {
            touchingContacts.append(
                (c.contactId, c.colorIndex, c.localIndex, c.encodedBodySimA, c.encodedBodySimB,
                    (c.flags & ContactFlags.simTouching.rawValue) != 0 ? 1 : 0)
            )
        }
        #expect(touchingContacts.count == 8)
        #expect(touchingContacts[0] == (0, 22, 0, -2, 0, 1))
        #expect(touchingContacts[1] == (1, 22, 1, -2, 1, 1))
        #expect(touchingContacts[2] == (2, 22, 2, -2, 3, 1))
        #expect(touchingContacts[3] == (3, 0, 0, 0, 1, 1))
        #expect(touchingContacts[4] == (4, -1, 4, 0, 2, 0))
        #expect(touchingContacts[5] == (5, 1, 0, 3, 0, 1))
        #expect(touchingContacts[6] == (6, -1, 6, 1, 2, 0))
        #expect(touchingContacts[7] == (7, -1, 7, 3, 2, 0))

        var nonzero = 0
        for i in 0..<Int(B3.graphColorCount) {
            let c = world.constraintGraph.colors[i]
            if c.convexContacts.count + c.contacts.count + c.jointSims.count > 0 {
                nonzero += 1
            }
        }
        #expect(nonzero == 3)
        #expect(world.constraintGraph.colors[0].convexContacts.count == 1)
        #expect(world.constraintGraph.colors[1].convexContacts.count == 1)
        #expect(world.constraintGraph.colors[22].convexContacts.count == 3)

        // Wake a sleeping body.
        def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(20, 1, 0)
        def.isAwake = false
        let sleeper = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createSphereShape(sleeper, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        var sb = world.body(sleeper)
        #expect(sb.setIndex == 3)
        #expect(sb.localIndex == 0)
        #expect(world.solverSets.count == 4)
        #expect(world.solverSets[2].bodySims.count == 4)

        world.wakeBody(world.bodyIndex(sleeper))

        sb = world.body(sleeper)
        #expect(sb.setIndex == 2)
        #expect(sb.localIndex == 4)
        #expect(world.solverSets.count == 4)
        #expect(world.solverSets[2].bodySims.count == 5)
        #expect(world.solverSets[2].bodyStates.count == 5)
        #expect(world.solverSets[2].islandSims.count == 3)
        #expect(sb.islandId == 3)
        #expect(world.islands[3].setIndex == 2)
        #expect(world.solverSets[3].setIndex == B3.nullIndex)
    }
}
