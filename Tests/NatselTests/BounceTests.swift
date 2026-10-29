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

@Suite("Restitution")
struct BounceTests {
    @Test("bouncing ball is repeatable")
    func bouncingBall() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            world.enableSleep = false

            var def = BodyDef()
            def.type = .staticBody
            let ground = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.baseMaterial.restitution = 0.8
            _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, Float(3.0), 0)
            let ball = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            shapeDef.baseMaterial.restitution = 0.8
            _ = world.createSphereShape(ball, shapeDef, Sphere(center: Vec3(), radius: 0.5))

            for _ in 0..<240 {
                world.step(1.0 / 60.0, subStepCount: 4)
            }

            var transform = world.bodyTransform(ball)
            var v = world.bodyLinearVelocity(ball)
            var w = world.bodyAngularVelocity(ball)

            var h: UInt64 = 14_695_981_039_346_656_037
            h = fnv(h, &transform.p, MemoryLayout<Pos>.size)
            h = fnv(h, &transform.q, 16)
            h = fnv(h, &v, 12)
            h = fnv(h, &w, 12)

            samples.append(UInt64(h))
            samples.append(UInt64(Float(transform.p.y).bitPattern))
            samples.append(UInt64(v.y.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
