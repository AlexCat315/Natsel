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

@Suite("Prismatic joint")
struct PrismaticJointTests {
    @Test("slider with lateral velocity is repeatable")
    func sliderLateral() {
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
            def.position = Pos(Float(0.5), 3, 0)
            def.linearVelocity = Vec3(1, 0.5, 1)
            def.angularVelocity = Vec3(0.3, 0.2, 1)
            let bodyB = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

            var jd = PrismaticJointDef()
            jd.base.bodyIdA = bodyA
            jd.base.bodyIdB = bodyB
            jd.enableSpring = true
            jd.hertz = 5.0
            jd.dampingRatio = 0.7
            jd.enableMotor = true
            jd.maxMotorForce = 100.0
            jd.motorSpeed = 1.0
            jd.enableLimit = true
            jd.lowerTranslation = -0.5
            jd.upperTranslation = 1.5
            let joint = world.createPrismaticJoint(jd)

            for _ in 0..<120 {
                world.step(1.0 / 60.0, subStepCount: 4)
            }

            let transform = world.bodyTransform(bodyB)
            let v = world.bodyLinearVelocity(bodyB)
            let av = world.bodyAngularVelocity(bodyB)
            let sim = world.readJointSim(joint.index1 - 1).prismaticJoint

            var h: UInt64 = 14_695_981_039_346_656_037
            var p = transform.p
            var q = transform.q
            var vv = v
            var avv = av
            var perp = sim.perpImpulse
            var angular = sim.angularImpulse
            var spring = sim.springImpulse
            var motor = sim.motorImpulse
            var lower = sim.lowerImpulse
            var upper = sim.upperImpulse
            h = fnv(h, &p, MemoryLayout<Pos>.size)
            h = fnv(h, &q, 16)
            h = fnv(h, &vv, 12)
            h = fnv(h, &avv, 12)
            h = fnv(h, &perp, 8)
            h = fnv(h, &angular, 12)
            h = fnv(h, &spring, 4)
            h = fnv(h, &motor, 4)
            h = fnv(h, &lower, 4)
            h = fnv(h, &upper, 4)

            samples.append(UInt64(h))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("slider is repeatable")
    func slider() {
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
            def.position = Pos(Float(0.5), 3, 0)
            def.linearVelocity = Vec3(1, 0.5, 0)
            def.angularVelocity = Vec3(0, 0, 1)
            let bodyB = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

            var jd = PrismaticJointDef()
            jd.base.bodyIdA = bodyA
            jd.base.bodyIdB = bodyB
            jd.enableSpring = true
            jd.hertz = 5.0
            jd.dampingRatio = 0.7
            jd.enableMotor = true
            jd.maxMotorForce = 100.0
            jd.motorSpeed = 1.0
            jd.enableLimit = true
            jd.lowerTranslation = -0.5
            jd.upperTranslation = 1.5
            let joint = world.createPrismaticJoint(jd)

            for _ in 0..<120 {
                world.step(1.0 / 60.0, subStepCount: 4)
            }

            let transform = world.bodyTransform(bodyB)
            let v = world.bodyLinearVelocity(bodyB)
            let av = world.bodyAngularVelocity(bodyB)
            let sim = world.readJointSim(joint.index1 - 1).prismaticJoint

            var h: UInt64 = 14_695_981_039_346_656_037
            var p = transform.p
            var q = transform.q
            var vv = v
            var avv = av
            var perp = sim.perpImpulse
            var angular = sim.angularImpulse
            var spring = sim.springImpulse
            var motor = sim.motorImpulse
            var lower = sim.lowerImpulse
            var upper = sim.upperImpulse
            h = fnv(h, &p, MemoryLayout<Pos>.size)
            h = fnv(h, &q, 16)
            h = fnv(h, &vv, 12)
            h = fnv(h, &avv, 12)
            h = fnv(h, &perp, 8)
            h = fnv(h, &angular, 12)
            h = fnv(h, &spring, 4)
            h = fnv(h, &motor, 4)
            h = fnv(h, &lower, 4)
            h = fnv(h, &upper, 4)

            samples.append(UInt64(h))
            samples.append(UInt64(sim.perpImpulse.x.bitPattern))
            samples.append(UInt64(sim.springImpulse.bitPattern))
            samples.append(UInt64(sim.motorImpulse.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
