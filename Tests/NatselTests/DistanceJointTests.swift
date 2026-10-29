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
    @Test("spring, limit, and motor are repeatable")
    func springPendulum() {
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

            samples.append(UInt64(h))
            samples.append(UInt64(sim.motorImpulse.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("rigid pendulum is repeatable")
    func rigidPendulum() {
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

            samples.append(UInt64(sim.impulse.bitPattern))
            samples.append(UInt64(h))
            samples.append(UInt64(Float(p.x).bitPattern))
            samples.append(UInt64(Float(p.y).bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
