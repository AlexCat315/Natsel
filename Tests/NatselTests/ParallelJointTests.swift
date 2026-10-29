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

@Suite("Parallel joint")
struct ParallelJointTests {
    @Test("parallel axes are repeatable")
    func parallelAxes() {
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
            def.angularVelocity = Vec3(1, 0.5, 0.3)
            let bodyB = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

            var jd = ParallelJointDef()
            jd.base.bodyIdA = bodyA
            jd.base.bodyIdB = bodyB
            jd.hertz = 10.0
            jd.dampingRatio = 0.5
            jd.maxTorque = 100.0
            let joint = world.createParallelJoint(jd)

            for _ in 0..<120 {
                world.step(1.0 / 60.0, subStepCount: 4)
            }

            let transform = world.bodyTransform(bodyB)
            let v = world.bodyLinearVelocity(bodyB)
            let av = world.bodyAngularVelocity(bodyB)
            let sim = world.readJointSim(joint.index1 - 1).parallelJoint

            var h: UInt64 = 14_695_981_039_346_656_037
            var p = transform.p
            var q = transform.q
            var vv = v
            var avv = av
            var perp = sim.perpImpulse
            h = fnv(h, &p, MemoryLayout<Pos>.size)
            h = fnv(h, &q, 16)
            h = fnv(h, &vv, 12)
            h = fnv(h, &avv, 12)
            h = fnv(h, &perp, 8)

            samples.append(UInt64(h))
            samples.append(UInt64(Float(transform.p.y).bitPattern))
            samples.append(UInt64(transform.q.v.z.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
