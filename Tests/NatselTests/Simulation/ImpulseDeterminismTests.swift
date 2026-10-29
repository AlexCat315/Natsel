import Testing
@testable import Natsel

@Suite("Impulse determinism repeatability")
struct ImpulseDeterminismTests {
    private func hashFloats(_ h: UInt32, _ value: inout some BitwiseCopyable) -> UInt32 {
        withUnsafeBytes(of: &value) { raw in
            hash(h, raw.baseAddress!, Int32(raw.count))
        }
    }

    private func hashContacts(_ world: World, _ body: BodyID, into h: inout UInt32, capacity: Int) {
        var contactKey = world.bodies[Int(world.bodyIndex(body))].headContactKey
        var count = 0
        while contactKey != B3.nullIndex && count < capacity {
            let contactId = contactKey >> 1
            let edgeIndex = Int(contactKey & 1)
            let contact = world.contacts[Int(contactId)]
            if (contact.flags & ContactFlags.touching.rawValue) != 0 {
                for m in 0..<Int(contact.manifoldCount) {
                    var manifold = contact.manifolds[m]
                    h = hashFloats(h, &manifold.rollingImpulse)
                    h = hashFloats(h, &manifold.frictionImpulse)
                    h = hashFloats(h, &manifold.twistImpulse)
                    for p in 0..<Int(manifold.pointCount) {
                        var point = manifold.points[p]
                        h = hashFloats(h, &point.normalImpulse)
                        h = hashFloats(h, &point.totalNormalImpulse)
                    }
                }
                count += 1
            }
            contactKey = contact.edges[edgeIndex].nextKey
        }
    }

    @Test("rolling resistance mix is repeatable", arguments: [false, true])
    func rollingMix(_ wide: Bool) {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            world.enableWideSolver = wide
            let ground = world.createBody(BodyDef())
            _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(40, 1, 40))

            var bodyIds: [BodyID] = []
            var seed: UInt32 = 271_828
            let box = makeBoxHull(0.4, 0.4, 0.4)
            let sphere = Sphere(center: Vec3(), radius: 0.4)
            let capsule = Capsule(center1: Vec3(-0.3, 0, 0), center2: Vec3(0.3, 0, 0), radius: 0.25)

            for i in 0..<64 {
                let row = i / 8
                let column = i % 8
                var def = BodyDef()
                def.type = .dynamicBody
                def.position = Pos(2 * Float(column) - 7, Float(1.45), 2 * Float(row) - 7)
                let id = world.createBody(def)
                bodyIds.append(id)

                seed = seed &* 1_664_525 &+ 1_013_904_223
                var shapeDef = ShapeDef()
                shapeDef.density = 1
                shapeDef.baseMaterial.rollingResistance = (seed >> 28) < 5 ? 0.2 : 0

                switch i % 3 {
                case 0: _ = world.createSphereShape(id, shapeDef, sphere)
                case 1: _ = world.createCapsuleShape(id, shapeDef, capsule)
                default: _ = world.createHullShape(id, shapeDef, box)
                }

                let sign: Float = (i & 1) != 0 ? -1 : 1
                world.setBodyLinearVelocity(id, Vec3(-0.0, -0.5, sign * 0.5))
                world.setBodyAngularVelocity(id, Vec3(sign * 3, -0.0, -2 * sign))
            }

            var h = hashInit
            for _ in 0..<120 {
                world.step(1.0 / 60.0, subStepCount: 4)
                for id in bodyIds {
                    h = hashTransform(h, world.bodyTransform(id))
                    hashContacts(world, id, into: &h, capacity: 4)
                }
            }

            samples.append(UInt64(h))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("mesh mix is repeatable", arguments: [false, true])
    func meshMix(_ wide: Bool) {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let (world, bodyIds) = makeMeshMix(wide)

            var h = hashInit
            for _ in 0..<180 {
                world.step(1.0 / 60.0, subStepCount: 4)
                for id in bodyIds {
                    h = hashTransform(h, world.bodyTransform(id))
                    var v = world.bodyLinearVelocity(id)
                    var w = world.bodyAngularVelocity(id)
                    h = hashFloats(h, &v)
                    h = hashFloats(h, &w)
                    hashContacts(world, id, into: &h, capacity: 8)
                }
            }

            samples.append(UInt64(h))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
    func makeMeshMix(_ wide: Bool) -> (World, [BodyID]) {
        let world = World()
        world.enableWideSolver = wide
        let mesh = createWaveMesh(
            xCount: 40, zCount: 40, cellWidth: 0.5, amplitude: 0.3,
            rowFrequency: 0.2, columnFrequency: 0.3
        )!
        var groundDef = BodyDef()
        groundDef.position = Pos(-10, 0, -10)
        let ground = world.createBody(groundDef)
        _ = world.createMeshShape(ground, ShapeDef(), mesh)

        var bodyIds: [BodyID] = []
        let smallBox = makeBoxHull(0.2, 0.2, 0.2)
        let largeBox = makeBoxHull(0.9, 0.3, 0.9)
        let sphere = Sphere(center: Vec3(), radius: 0.3)
        let capsule = Capsule(center1: Vec3(-0.5, 0, 0), center2: Vec3(0.5, 0, 0), radius: 0.2)

        var seed: UInt32 = 314_159
        for i in 0..<64 {
            let row = i / 8
            let column = i % 8
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(2.2 * Float(column) - 7.7, Float(1.2), 2.2 * Float(row) - 7.7)
            let id = world.createBody(def)
            bodyIds.append(id)

            seed = seed &* 1_664_525 &+ 1_013_904_223
            var shapeDef = ShapeDef()
            shapeDef.density = 1
            shapeDef.baseMaterial.rollingResistance = (seed >> 28) < 5 ? 0.2 : 0
            switch (seed >> 24) & 3 {
            case 0: _ = world.createHullShape(id, shapeDef, largeBox)
            case 1: _ = world.createHullShape(id, shapeDef, smallBox)
            case 2: _ = world.createSphereShape(id, shapeDef, sphere)
            default: _ = world.createCapsuleShape(id, shapeDef, capsule)
            }

            let sign: Float = (i & 1) != 0 ? -1 : 1
            world.setBodyLinearVelocity(id, Vec3(-0.0, -0.5, -0.0))
            world.setBodyAngularVelocity(id, Vec3(-0.0, sign * 0.5, -0.0))
        }

        return (world, bodyIds)
    }

    @Test("mesh mix preserves every impulse bit")
    func meshMixPathsAgree() throws {
        let (scalar, _) = makeMeshMix(false)
        let (wide, _) = makeMeshMix(true)
        func bits(_ v: Vec3) -> [UInt32] { [v.x.bitPattern, v.y.bitPattern, v.z.bitPattern] }
        for step in 0..<180 {
            scalar.step(1 / 60, subStepCount: 4)
            wide.step(1 / 60, subStepCount: 4)
            try #require(scalar.worldStateHash() == wide.worldStateHash(), "state at step \(step)")
            for id in scalar.contacts.indices {
                let a = scalar.contacts[id], b = wide.contacts[id]
                try #require(a.manifoldCount == b.manifoldCount)
                for mi in 0..<Int(a.manifoldCount) {
                    let x = a.manifolds[mi], y = b.manifolds[mi]
                    try #require(bits(x.frictionImpulse) == bits(y.frictionImpulse), "friction step \(step) id \(id) manifold \(mi) \(x.frictionImpulse) != \(y.frictionImpulse)")
                    try #require(bits(x.rollingImpulse) == bits(y.rollingImpulse), "rolling step \(step) id \(id) manifold \(mi) \(x.rollingImpulse) != \(y.rollingImpulse)")
                    try #require(x.twistImpulse.bitPattern == y.twistImpulse.bitPattern, "twist step \(step) id \(id) manifold \(mi)")
                    for pi in 0..<Int(x.pointCount) {
                        try #require(x.points[pi].normalImpulse.bitPattern == y.points[pi].normalImpulse.bitPattern, "normal step \(step) id \(id) manifold \(mi) point \(pi)")
                        try #require(x.points[pi].totalNormalImpulse.bitPattern == y.points[pi].totalNormalImpulse.bitPattern, "total step \(step) id \(id) manifold \(mi) point \(pi)")
                    }
                }
            }
        }
    }

}
