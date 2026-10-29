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

@Suite("Collision step")
struct CollisionStepTests {
    @Test("stacked drop is repeatable")
    func stackedDrop() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            world.enableSleep = false

            var def = BodyDef()
            def.type = .staticBody
            let ground = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = 0
            _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, Float(1.5), 0)
            let box = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(Float(0.6), Float(2.2), 0)
            let sphere = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createSphereShape(sphere, shapeDef, Sphere(center: Vec3(), radius: 0.5))

            for _ in 0..<60 {
                world.step(1.0 / 60.0, subStepCount: 4)
            }

            var h: UInt64 = 14_695_981_039_346_656_037
            for id in [ground, box, sphere] {
                var transform = world.bodyTransform(id)
                var v = world.bodyLinearVelocity(id)
                var w = world.bodyAngularVelocity(id)
                h = fnv(h, &transform.p, MemoryLayout<Pos>.size)
                h = fnv(h, &transform.q, 16)
                h = fnv(h, &v, 12)
                h = fnv(h, &w, 12)
            }

            var count = 0
            var ch: UInt64 = 14_695_981_039_346_656_037
            for c in world.contacts where c.contactId != B3.nullIndex {
                count += 1
                var flags = c.flags
                var manifoldCount = c.manifoldCount
                var friction = c.friction
                var restitution = c.restitution
                ch = fnv(ch, &flags, 4)
                ch = fnv(ch, &manifoldCount, 4)
                ch = fnv(ch, &friction, 4)
                ch = fnv(ch, &restitution, 4)

                for m in 0..<Int(c.manifoldCount) {
                    var manifold = c.manifolds[m]
                    var normal = manifold.normal
                    var pointCount = manifold.pointCount
                    var twistImpulse = manifold.twistImpulse
                    var frictionImpulse = manifold.frictionImpulse
                    var rollingImpulse = manifold.rollingImpulse
                    ch = fnv(ch, &normal, 12)
                    ch = fnv(ch, &pointCount, 4)
                    ch = fnv(ch, &twistImpulse, 4)
                    ch = fnv(ch, &frictionImpulse, 12)
                    ch = fnv(ch, &rollingImpulse, 12)

                    for p in 0..<Int(manifold.pointCount) {
                        var mp = manifold.points[p]
                        var anchorA = mp.anchorA
                        var anchorB = mp.anchorB
                        var separation = mp.separation
                        var featureId = mp.featureId
                        var normalImpulse = mp.normalImpulse
                        var totalNormalImpulse = mp.totalNormalImpulse
                        var normalVelocity = mp.normalVelocity
                        var baseSeparation = mp.baseSeparation
                        ch = fnv(ch, &anchorA, 12)
                        ch = fnv(ch, &anchorB, 12)
                        ch = fnv(ch, &separation, 4)
                        ch = fnv(ch, &featureId, 4)
                        ch = fnv(ch, &normalImpulse, 4)
                        ch = fnv(ch, &totalNormalImpulse, 4)
                        ch = fnv(ch, &normalVelocity, 4)
                        ch = fnv(ch, &baseSeparation, 4)
                    }
                }
            }

            for (i, id) in [ground, box, sphere].enumerated() {
                let t = world.bodyTransform(id)
                let v = world.bodyLinearVelocity(id)

            }

            #expect(count == 2)
            samples.append(UInt64(ch))
            samples.append(UInt64(h))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
