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

@Suite("Wheel joint")
struct WheelJointTests {
    @Test("suspension, spin, and steering match the C reference")
    func wheel() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Vec3(0, 3, 0)
        let bodyA = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(0.5, 3, 0)
        def.angularVelocity = Vec3(0, 0, 1)
        let bodyB = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createSphereShape(bodyB, shapeDef, Sphere(center: Vec3(), radius: 0.3))

        var jd = WheelJointDef()
        jd.base.bodyIdA = bodyA
        jd.base.bodyIdB = bodyB
        jd.enableSpinMotor = true
        jd.maxSpinTorque = 50.0
        jd.spinSpeed = 2.0
        jd.enableSuspensionSpring = true
        jd.suspensionHertz = 4.0
        jd.suspensionDampingRatio = 0.7
        jd.enableSuspensionLimit = true
        jd.lowerSuspensionLimit = -0.2
        jd.upperSuspensionLimit = 0.2
        jd.enableSteering = true
        jd.steeringHertz = 4.0
        jd.steeringDampingRatio = 0.7
        jd.targetSteeringAngle = 0.3
        jd.maxSteeringTorque = 30.0
        jd.enableSteeringLimit = true
        jd.lowerSteeringLimit = -0.5
        jd.upperSteeringLimit = 0.5
        let joint = world.createWheelJoint(jd)

        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        let transform = world.bodyTransform(bodyB)
        let v = world.bodyLinearVelocity(bodyB)
        let av = world.bodyAngularVelocity(bodyB)
        let sim = world.readJointSim(joint.index1 - 1).wheelJoint

        var h: UInt64 = 14_695_981_039_346_656_037
        var p = transform.p
        var q = transform.q
        var vv = v
        var avv = av
        var linear = sim.linearImpulse
        var angular = sim.angularImpulse
        var spin = sim.spinImpulse
        var sus = sim.suspensionSpringImpulse
        var lowSus = sim.lowerSuspensionImpulse
        var upSus = sim.upperSuspensionImpulse
        var steer = sim.steeringSpringImpulse
        var lowSteer = sim.lowerSteeringImpulse
        var upSteer = sim.upperSteeringImpulse
        h = fnv(h, &p, 12)
        h = fnv(h, &q, 16)
        h = fnv(h, &vv, 12)
        h = fnv(h, &avv, 12)
        h = fnv(h, &linear, 8)
        h = fnv(h, &angular, 8)
        h = fnv(h, &spin, 4)
        h = fnv(h, &sus, 4)
        h = fnv(h, &lowSus, 4)
        h = fnv(h, &upSus, 4)
        h = fnv(h, &steer, 4)
        h = fnv(h, &lowSteer, 4)
        h = fnv(h, &upSteer, 4)

        #expect(h == 0x15f6_c4c3_aaff_79cb)
        #expect(sim.spinImpulse.bitPattern == 0x348c_939e)
        #expect(sim.steeringSpringImpulse.bitPattern == 0x3435_40c6)
    }
}
