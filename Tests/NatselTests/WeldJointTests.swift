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

@Suite("Weld joint")
struct WeldJointTests {
    @Test("welded body matches the C reference")
    func weldedBody() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 3, 0)
        let bodyA = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(Float(0.8), 3, 0)
        def.angularVelocity = Vec3(0.5, 0.2, 0)
        let bodyB = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jd = WeldJointDef()
        jd.base.bodyIdA = bodyA
        jd.base.bodyIdB = bodyB
        jd.base.localFrameA.p = Vec3(1, 0, 0)
        jd.base.localFrameB.p = Vec3(0.2, 0, 0)
        let joint = world.createWeldJoint(jd)

        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        let transform = world.bodyTransform(bodyB)
        let v = world.bodyLinearVelocity(bodyB)
        let av = world.bodyAngularVelocity(bodyB)
        let sim = world.readJointSim(joint.index1 - 1).weldJoint

        var h: UInt64 = 14_695_981_039_346_656_037
        var p = transform.p
        var q = transform.q
        var vv = v
        var avv = av
        var linear = sim.linearImpulse
        var angular = sim.angularImpulse
        h = fnv(h, &p, MemoryLayout<Pos>.size)
        h = fnv(h, &q, 16)
        h = fnv(h, &vv, 12)
        h = fnv(h, &avv, 12)
        h = fnv(h, &linear, 12)
        h = fnv(h, &angular, 12)

        #expect(h == 0x303bd3ed3a2b458b)
        #expect(sim.linearImpulse.y.bitPattern == 0x402aaaab)
        #expect(sim.angularImpulse.z.bitPattern == 0xbf088888)
    }
}
