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

@Suite("Motor joint")
struct MotorJointTests {
    @Test("velocity and spring control match the C reference")
    func motorControl() {
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

        var jd = MotorJointDef()
        jd.base.bodyIdA = bodyA
        jd.base.bodyIdB = bodyB
        jd.linearVelocity = Vec3(0.5, 0, 0)
        jd.maxVelocityForce = 100.0
        jd.angularVelocity = Vec3(1, 0, 0)
        jd.maxVelocityTorque = 50.0
        jd.linearHertz = 4.0
        jd.linearDampingRatio = 0.5
        jd.maxSpringForce = 200.0
        jd.angularHertz = 4.0
        jd.angularDampingRatio = 0.5
        jd.maxSpringTorque = 100.0
        let joint = world.createMotorJoint(jd)

        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        let transform = world.bodyTransform(bodyB)
        let v = world.bodyLinearVelocity(bodyB)
        let av = world.bodyAngularVelocity(bodyB)
        let sim = world.readJointSim(joint.index1 - 1).motorJoint

        var h: UInt64 = 14_695_981_039_346_656_037
        var p = transform.p
        var q = transform.q
        var vv = v
        var avv = av
        var lv = sim.linearVelocityImpulse
        var angV = sim.angularVelocityImpulse
        var ls = sim.linearSpringImpulse
        var angS = sim.angularSpringImpulse
        h = fnv(h, &p, MemoryLayout<Pos>.size)
        h = fnv(h, &q, 16)
        h = fnv(h, &vv, 12)
        h = fnv(h, &avv, 12)
        h = fnv(h, &lv, 12)
        h = fnv(h, &angV, 12)
        h = fnv(h, &ls, 12)
        h = fnv(h, &angS, 12)

        #expect(h == 0x69cf509b57a20931)
        #expect(sim.linearVelocityImpulse.x.bitPattern == 0x3d2f3d33)
        #expect(sim.angularVelocityImpulse.x.bitPattern == 0x3e55_5555)
    }
}
