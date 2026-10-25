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

@Suite("Box stack")
struct StackTests {
    @Test("three box stack matches the C reference")
    func boxStack() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        var boxes: [BodyID] = []
        for i in 0..<3 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 0.55 + Float(i) * 1.05, 0)
            let box = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
            boxes.append(box)
        }

        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        var h: UInt64 = 14_695_981_039_346_656_037
        for id in boxes {
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
            ch = fnv(ch, &flags, 4)
            ch = fnv(ch, &manifoldCount, 4)
            for m in 0..<Int(c.manifoldCount) {
                var manifold = c.manifolds[m]
                var normal = manifold.normal
                var pointCount = manifold.pointCount
                var frictionImpulse = manifold.frictionImpulse
                var twistImpulse = manifold.twistImpulse
                ch = fnv(ch, &normal, 12)
                ch = fnv(ch, &pointCount, 4)
                ch = fnv(ch, &frictionImpulse, 12)
                ch = fnv(ch, &twistImpulse, 4)
                for p in 0..<Int(manifold.pointCount) {
                    var mp = manifold.points[p]
                    var anchorA = mp.anchorA
                    var anchorB = mp.anchorB
                    var separation = mp.separation
                    var normalImpulse = mp.normalImpulse
                    var totalNormalImpulse = mp.totalNormalImpulse
                    ch = fnv(ch, &anchorA, 12)
                    ch = fnv(ch, &anchorB, 12)
                    ch = fnv(ch, &separation, 4)
                    ch = fnv(ch, &normalImpulse, 4)
                    ch = fnv(ch, &totalNormalImpulse, 4)
                }
            }
        }

        print("STACK bodyhash \(String(h, radix: 16)) contacts \(count) chash \(String(ch, radix: 16))")
        #expect(count == 3)
        #expect(ch == 0x3b5f182b45a9977a)
        #expect(h == 0x8992527b78b1c587)

        let p1 = world.bodyTransform(boxes[1]).p
        #expect(Float(p1.x).bitPattern == 0x3df298e1)
        #expect(Float(p1.y).bitPattern == 0x4000555c)
    }
}
