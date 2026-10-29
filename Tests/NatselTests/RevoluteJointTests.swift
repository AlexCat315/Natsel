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

@Suite("Revolute joint")
struct RevoluteJointTests {
    @Test("spring, limit, and motor are repeatable")
    func springRevolute() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            world.enableSleep = false

            var def = BodyDef()
            def.type = .staticBody
            def.position = Pos(0, 3, 0)
            let bodyA = world.createBody(def)

            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(Float(1.5), 3, 0)
            def.angularVelocity = Vec3(1, 0.5, 0)
            let bodyB = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

            var jd = RevoluteJointDef()
            jd.base.bodyIdA = bodyA
            jd.base.bodyIdB = bodyB
            jd.base.localFrameA.p = Vec3(1, 0, 0)
            jd.base.localFrameB.p = Vec3(-0.5, 0, 0)
            jd.enableSpring = true
            jd.hertz = 4.0
            jd.dampingRatio = 0.6
            jd.enableMotor = true
            jd.maxMotorTorque = 50.0
            jd.motorSpeed = 1.0
            jd.enableLimit = true
            jd.lowerAngle = -0.5
            jd.upperAngle = 0.5
            let joint = world.createRevoluteJoint(jd)

            for _ in 0..<120 {
                world.step(1.0 / 60.0, subStepCount: 4)
            }

            let transform = world.bodyTransform(bodyB)
            let v = world.bodyLinearVelocity(bodyB)
            let av = world.bodyAngularVelocity(bodyB)
            let sim = world.readJointSim(joint.index1 - 1).revoluteJoint

            var h: UInt64 = 14_695_981_039_346_656_037
            var p = transform.p
            var q = transform.q
            var vv = v
            var avv = av
            var linear = sim.linearImpulse
            var perp = sim.perpImpulse
            var spring = sim.springImpulse
            var motor = sim.motorImpulse
            var lower = sim.lowerImpulse
            var upper = sim.upperImpulse
            h = fnv(h, &p, MemoryLayout<Pos>.size)
            h = fnv(h, &q, 16)
            h = fnv(h, &vv, 12)
            h = fnv(h, &avv, 12)
            h = fnv(h, &linear, 12)
            h = fnv(h, &perp, 8)
            h = fnv(h, &spring, 4)
            h = fnv(h, &motor, 4)
            h = fnv(h, &lower, 4)
            h = fnv(h, &upper, 4)

            samples.append(UInt64(h))
            samples.append(UInt64(sim.motorImpulse.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("hinged pendulum is repeatable")
    func hingedPendulum() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let world = World()
            world.enableSleep = false

            var def = BodyDef()
            def.type = .staticBody
            def.position = Pos(0, 3, 0)
            let bodyA = world.createBody(def)

            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(Float(1.5), 3, 0)
            let bodyB = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

            var jd = RevoluteJointDef()
            jd.base.bodyIdA = bodyA
            jd.base.bodyIdB = bodyB
            jd.base.localFrameA.p = Vec3(1, 0, 0)
            jd.base.localFrameB.p = Vec3(-0.5, 0, 0)
            let joint = world.createRevoluteJoint(jd)

            for _ in 0..<120 {
                world.step(1.0 / 60.0, subStepCount: 4)
            }

            let transform = world.bodyTransform(bodyB)
            let v = world.bodyLinearVelocity(bodyB)
            let av = world.bodyAngularVelocity(bodyB)
            let sim = world.readJointSim(joint.index1 - 1).revoluteJoint

            var h: UInt64 = 14_695_981_039_346_656_037
            var p = transform.p
            var q = transform.q
            var vv = v
            var avv = av
            var linear = sim.linearImpulse
            var perp = sim.perpImpulse
            var spring = sim.springImpulse
            var motor = sim.motorImpulse
            var lower = sim.lowerImpulse
            var upper = sim.upperImpulse
            h = fnv(h, &p, MemoryLayout<Pos>.size)
            h = fnv(h, &q, 16)
            h = fnv(h, &vv, 12)
            h = fnv(h, &avv, 12)
            h = fnv(h, &linear, 12)
            h = fnv(h, &perp, 8)
            h = fnv(h, &spring, 4)
            h = fnv(h, &motor, 4)
            h = fnv(h, &lower, 4)
            h = fnv(h, &upper, 4)

            samples.append(UInt64(h))
            samples.append(UInt64(sim.linearImpulse.x.bitPattern))
            samples.append(UInt64(sim.linearImpulse.y.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
