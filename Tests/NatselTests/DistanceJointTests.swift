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

@Suite("Distance joint")
struct DistanceJointTests {
    @Test("spring, limit, and motor match the C reference")
    func springPendulum() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 3, 0)
        let bodyA = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 3, 0)
        let bodyB = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jd = DistanceJointDef()
        jd.base.bodyIdA = bodyA
        jd.base.bodyIdB = bodyB
        jd.length = 1.0
        jd.enableSpring = true
        jd.hertz = 5.0
        jd.dampingRatio = 0.7
        jd.enableLimit = true
        jd.minLength = 0.5
        jd.maxLength = 1.5
        jd.enableMotor = true
        jd.maxMotorForce = 100.0
        jd.motorSpeed = 2.0
        let joint = world.createDistanceJoint(jd)

        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        let transform = world.bodyTransform(bodyB)
        let v = world.bodyLinearVelocity(bodyB)
        let av = world.bodyAngularVelocity(bodyB)
        let sim = world.distanceJointSim(joint)!

        var h: UInt64 = 14_695_981_039_346_656_037
        var p = transform.p
        var q = transform.q
        var vv = v
        var avv = av
        var impulse = sim.impulse
        var lower = sim.lowerImpulse
        var upper = sim.upperImpulse
        var motor = sim.motorImpulse
        h = fnv(h, &p, MemoryLayout<Pos>.size)
        h = fnv(h, &q, 16)
        h = fnv(h, &vv, 12)
        h = fnv(h, &avv, 12)
        h = fnv(h, &impulse, 4)
        h = fnv(h, &lower, 4)
        h = fnv(h, &upper, 4)
        h = fnv(h, &motor, 4)

        #expect(h == 0xe74092bba8a6ae98)
        #expect(sim.motorImpulse.bitPattern == 0x3ed5_5556)
    }

    @Test("rigid pendulum matches the C reference")
    func rigidPendulum() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 3, 0)
        let bodyA = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 3, 0)
        let bodyB = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jd = DistanceJointDef()
        jd.base.bodyIdA = bodyA
        jd.base.bodyIdB = bodyB
        jd.length = 1.0
        let joint = world.createDistanceJoint(jd)

        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        let transform = world.bodyTransform(bodyB)
        let v = world.bodyLinearVelocity(bodyB)
        let av = world.bodyAngularVelocity(bodyB)
        let sim = world.distanceJointSim(joint)!

        var h: UInt64 = 14_695_981_039_346_656_037
        var p = transform.p
        var q = transform.q
        var vv = v
        var avv = av
        var impulse = sim.impulse
        h = fnv(h, &p, MemoryLayout<Pos>.size)
        h = fnv(h, &q, 16)
        h = fnv(h, &vv, 12)
        h = fnv(h, &avv, 12)
        h = fnv(h, &impulse, 4)

        print("JOINT impulse \(sim.impulse) p \(p) hash \(String(h, radix: 16))")
        #expect(sim.impulse.bitPattern == 0xc08f3e67)
        #expect(h == 0xe093de5a5a98dd52)
        #expect(Float(p.x).bitPattern == 0x3f558449)
        #expect(Float(p.y).bitPattern == 0x401ca3d5)
    }
}
