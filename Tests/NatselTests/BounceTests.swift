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
    @Test("bouncing ball matches the C reference")
    func bouncingBall() {
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
        def.position = Vec3(0, 3.0, 0)
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
        h = fnv(h, &transform.p, 12)
        h = fnv(h, &transform.q, 16)
        h = fnv(h, &v, 12)
        h = fnv(h, &w, 12)

        print("BOUNCE p \(transform.p.y) v \(v.y) hash \(String(h, radix: 16))")
        #expect(h == 0xffee_0af1_588c_ad6e)
        #expect(transform.p.y.bitPattern == 0x3f82_e893)
        #expect(v.y.bitPattern == 0xbfde_ac09)
    }
}
