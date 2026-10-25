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

private func hashManifolds(_ world: World) -> (UInt64, Int, Int) {
    var h: UInt64 = 14_695_981_039_346_656_037
    var touchCount = 0

    for c in world.contacts where c.contactId != B3.nullIndex {
        var flags = c.flags
        var manifoldCount = c.manifoldCount
        var friction = c.friction
        var restitution = c.restitution
        var rolling = c.rollingResistance
        var tangentVelocity = c.tangentVelocity

        h = fnv(h, &flags, 4)
        h = fnv(h, &manifoldCount, 4)
        h = fnv(h, &friction, 4)
        h = fnv(h, &restitution, 4)
        h = fnv(h, &rolling, 4)
        h = fnv(h, &tangentVelocity, 12)

        for m in 0..<Int(c.manifoldCount) {
            var manifold = c.manifolds[m]
            var normal = manifold.normal
            var pointCount = manifold.pointCount
            h = fnv(h, &normal, 12)
            h = fnv(h, &pointCount, 4)

            for p in 0..<Int(manifold.pointCount) {
                var mp = manifold.points[p]
                var anchorA = mp.anchorA
                var anchorB = mp.anchorB
                var separation = mp.separation
                var featureId = mp.featureId
                var normalImpulse = mp.normalImpulse
                var persisted: UInt8 = mp.persisted ? 1 : 0
                h = fnv(h, &anchorA, 12)
                h = fnv(h, &anchorB, 12)
                h = fnv(h, &separation, 4)
                h = fnv(h, &featureId, 4)
                h = fnv(h, &normalImpulse, 4)
                h = fnv(h, &persisted, 1)
            }
        }

        if (c.flags & ContactFlags.simTouching.rawValue) != 0 {
            touchCount += 1
        }
    }

    return (h, touchCount, Int(world.contacts.filter { $0.contactId != B3.nullIndex }.count))
}

private func runUpdates(_ world: World) -> Int {
    var touchCount = 0
    for cid in world.solverSets[2].contactIndices {
        let contact = world.contacts[Int(cid)]
        let shapeA = world.shapes[Int(contact.shapeIdA)]
        let shapeB = world.shapes[Int(contact.shapeIdB)]
        let bodyA = world.bodyIndex(world.makeBodyID(shapeA.bodyId))
        let bodyB = world.bodyIndex(world.makeBodyID(shapeB.bodyId))
        _ = bodyA
        _ = bodyB

        let simA = world.bodySimFor(world.makeBodyID(shapeA.bodyId))
        let simB = world.bodySimFor(world.makeBodyID(shapeB.bodyId))

        var store = ContactStore(&world.contacts)
        let touching = world.updateContact(
            cid, simA.localCenter, simA.transform, simB.localCenter, simB.transform, false, &store
        )
        if touching {
            touchCount += 1
        }
    }
    return touchCount
}

@Suite("Contact update")
struct ContactUpdateTests {
    @Test("convex manifolds match the C reference")
    func manifolds() {
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
            def.position = Pos(p)
            let body = world.createBody(def)

            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 0.5))
        }

        // Dynamic hull box sitting on the ground and touching sphere A.
        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(Float(-0.9), Float(0.9), 0)
        let boxBody = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(boxBody, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        world.updateBroadPhasePairs()

        var touching = runUpdates(world)
        #expect(touching == 5)

        var (h, touchCount, contactCount) = hashManifolds(world)
        #expect(touchCount == 5)
        #expect(contactCount == 8)
        #expect(h == 0xcb5a_bfdf_be37_e76a)

        // Link touching contacts into islands in awake set order.
        for cid in world.solverSets[2].contactIndices {
            let c = world.contacts[Int(cid)]
            if (c.flags & ContactFlags.simTouching.rawValue) != 0 {
                world.linkContact(cid)
            }
        }

        var ih: UInt64 = 14_695_981_039_346_656_037
        for island in world.islands where island.islandId != B3.nullIndex {
            var islandId = island.islandId
            var setIndex = island.setIndex
            ih = fnv(ih, &islandId, 4)
            ih = fnv(ih, &setIndex, 4)
            for bodyId in island.bodies { var b = bodyId; ih = fnv(ih, &b, 4) }
            for link in island.contacts { var c = link.contactId; ih = fnv(ih, &c, 4) }
        }
        for b in world.bodies where b.id != B3.nullIndex {
            var islandId = b.islandId
            var islandIndex = b.islandIndex
            ih = fnv(ih, &islandId, 4)
            ih = fnv(ih, &islandIndex, 4)
        }
        #expect(ih == 0xb3de_0b31_27da_08cf)

        // Inject impulses to test recycling by feature id.
        for i in 0..<world.contacts.count {
            var c = world.contacts[i]
            if c.contactId == B3.nullIndex { continue }
            for m in 0..<Int(c.manifoldCount) {
                var manifold = c.manifolds[m]
                for p in 0..<Int(manifold.pointCount) {
                    var mp = manifold.points[p]
                    mp.normalImpulse = Float(10 + p)
                    manifold.points[p] = mp
                }
                c.manifolds[m] = manifold
            }
            world.contacts[i] = c
        }

        touching = runUpdates(world)
        #expect(touching == 5)
        (h, touchCount, contactCount) = hashManifolds(world)
        #expect(h == 0x5118_9ed1_fb59_6f52)

        // A further update carries the recycled impulses.
        touching = runUpdates(world)
        #expect(touching == 5)
        (h, touchCount, contactCount) = hashManifolds(world)
        #expect(h == 0x5118_9ed1_fb59_6f52)
    }
}
