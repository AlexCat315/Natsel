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

@Suite("Spherical joint")
struct SphericalJointTests {
    @Test("ball socket with limits matches the C reference")
    func ballSocket() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Vec3(0, 3, 0)
        let bodyA = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(1.5, 3, 0)
        def.angularVelocity = Vec3(1, 0.5, 0.3)
        let bodyB = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jd = SphericalJointDef()
        jd.base.bodyIdA = bodyA
        jd.base.bodyIdB = bodyB
        jd.base.localFrameA.p = Vec3(1, 0, 0)
        jd.base.localFrameB.p = Vec3(-0.5, 0, 0)
        jd.enableSpring = true
        jd.hertz = 4.0
        jd.dampingRatio = 0.6
        jd.enableMotor = true
        jd.maxMotorTorque = 30.0
        jd.motorVelocity = Vec3(0.5, 0, 1)
        jd.enableConeLimit = true
        jd.coneAngle = 0.6
        jd.enableTwistLimit = true
        jd.lowerTwistAngle = -0.4
        jd.upperTwistAngle = 0.4
        let joint = world.createSphericalJoint(jd)

        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        let transform = world.bodyTransform(bodyB)
        let v = world.bodyLinearVelocity(bodyB)
        let av = world.bodyAngularVelocity(bodyB)
        let sim = world.readJointSim(joint.index1 - 1).sphericalJoint

        var h: UInt64 = 14_695_981_039_346_656_037
        var p = transform.p
        var q = transform.q
        var vv = v
        var avv = av
        var linear = sim.linearImpulse
        var spring = sim.springImpulse
        var motor = sim.motorImpulse
        var lower = sim.lowerTwistImpulse
        var upper = sim.upperTwistImpulse
        var swing = sim.swingImpulse
        h = fnv(h, &p, 12)
        h = fnv(h, &q, 16)
        h = fnv(h, &vv, 12)
        h = fnv(h, &avv, 12)
        h = fnv(h, &linear, 12)
        h = fnv(h, &spring, 12)
        h = fnv(h, &motor, 12)
        h = fnv(h, &lower, 4)
        h = fnv(h, &upper, 4)
        h = fnv(h, &swing, 4)

        #expect(h == 0xe54e_d4c3_e3fc_77a0)
        #expect(sim.linearImpulse.y.bitPattern == 0x402e_4bf3)
        #expect(sim.motorImpulse.x.bitPattern == 0x3d62_9f3b)
    }
}
