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

@Suite("Continuous collision")
struct CcdTests {
    func run(bullet: Bool) -> (UInt64, Int) {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let wall = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(wall, shapeDef, makeBoxHull(0.1, 2, 2))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(-5, 0, 0)
        def.linearVelocity = Vec3(200, 0, 0)
        def.isBullet = bullet
        let body = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        for _ in 0..<30 {
            world.step(1.0 / 60.0, subStepCount: 1)
        }

        var transform = world.bodyTransform(body)
        var v = world.bodyLinearVelocity(body)
        var av = world.bodyAngularVelocity(body)
        var flags = world.body(body).flags
        #expect(transform.p.x < 0, "the fast body must remain on the approach side of the wall")
        #expect(v.x.isFinite && v.y.isFinite && v.z.isFinite)
        #expect(av.x.isFinite && av.y.isFinite && av.z.isFinite)

        var h: UInt64 = 14_695_981_039_346_656_037
        h = fnv(h, &transform.p, MemoryLayout<Pos>.size)
        h = fnv(h, &transform.q, 16)
        h = fnv(h, &v, 12)
        h = fnv(h, &av, 12)
        h = fnv(h, &flags, 4)

        var count = 0
        for c in world.contacts where c.contactId != B3.nullIndex { count += 1 }
        return (h, count)
    }

    @Test("bullet does not tunnel")
    func bullet() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let (h, count) = run(bullet: true)
            #expect(count == 0)
            samples.append(UInt64(h))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("non-bullet fast body does not tunnel")
    func fastBody() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let (h, count) = run(bullet: false)
            #expect(count == 0)
            samples.append(UInt64(h))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
