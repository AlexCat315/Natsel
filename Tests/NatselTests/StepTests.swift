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

@Suite("Step integration")
struct StepTests {
    @Test("contact-free integration matches the C reference")
    func contactFreeStep() {
        let world = World()
        world.enableSleep = false
        world.enableContinuous = false

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 5, 0)
        def.linearVelocity = Vec3(1, 2, 3)
        def.angularVelocity = Vec3(0.1, 0.2, 0.3)
        def.linearDamping = 0.1
        def.angularDamping = 0.2
        def.gravityScale = 0.5
        let b0 = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(b0, shapeDef, makeBoxHull(0.5, 1.0, 1.5))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(-1, 3, 0)
        def.linearVelocity = Vec3(0.5, -0.5, 1.0)
        def.angularVelocity = Vec3(1, 0, 0)
        def.motionLocks.linearX = true
        def.motionLocks.angularY = true
        def.motionLocks.angularZ = true
        let b1 = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 500
        _ = world.createSphereShape(b1, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        def = BodyDef()
        def.type = .kinematicBody
        def.position = Pos(2, 0, 0)
        def.linearVelocity = Vec3(0, 0, 1)
        def.angularVelocity = Vec3(0.3, 0, 0)
        let b2 = world.createBody(def)
        shapeDef = ShapeDef()
        _ = world.createSphereShape(b2, shapeDef, Sphere(center: Vec3(), radius: 0.25))

        def = BodyDef()
        def.type = .staticBody
        def.position = Pos(10, 10, 10)
        let b3s = world.createBody(def)
        shapeDef = ShapeDef()
        _ = world.createSphereShape(b3s, shapeDef, Sphere(center: Vec3(), radius: 1.0))

        world.applyBodyForce(b0, Vec3(100, 0, 0), Pos(Float(0.2), Float(5.2), 0), true)
        world.applyBodyTorque(b0, Vec3(0, 10, 0), true)

        for _ in 0..<5 {
            world.step(1.0 / 60.0, subStepCount: 4)
        }

        var h: UInt64 = 14_695_981_039_346_656_037
        let ids = [b0, b1, b2, b3s]
        for id in ids {
            var transform = world.bodyTransform(id)
            var v = world.bodyLinearVelocity(id)
            var w = world.bodyAngularVelocity(id)
            var center = world.bodyWorldCenter(id)
            h = fnv(h, &transform.p, MemoryLayout<Pos>.size)
            h = fnv(h, &transform.q, 16)
            h = fnv(h, &v, 12)
            h = fnv(h, &w, 12)
            h = fnv(h, &center, 12)
            var flags = world.body(id).flags
            h = fnv(h, &flags, 4)
        }
        #expect(h == 0x32c481b36da0e179)

        let t0 = world.bodyTransform(b0)
        #expect(Float(t0.p.x).bitPattern == 0x3da9_f74e)
        #expect(Float(t0.p.y).bitPattern == 0x40a4_ba70)
        #expect(Float(t0.p.z).bitPattern == 0x3e7e_e22e)
        #expect(t0.q.v.x.bitPattern == 0x3b88_b2ee)
        #expect(t0.q.s.bitPattern == 0x3f7f_f82d)

        #expect(world.bodyLinearVelocity(b0).x.bitPattern == 0x3f7d_f25d)
        #expect(world.bodyLinearVelocity(b0).y.bitPattern == 0x3fc8_c0d2)
        #expect(world.bodyLinearVelocity(b0).z.bitPattern == 0x403e_6839)

        // Motion locks zero the x translation.
        #expect(Float(world.bodyTransform(b1).p.x).bitPattern == 0xbf80_0000)
        #expect(world.bodyLinearVelocity(b1).x.bitPattern == 0)
        #expect(world.body(b1).flags == 0x0000_7031)

        // Kinematic body moves but ignores gravity.
        #expect(Float(world.bodyTransform(b2).p.z).bitPattern == 0x3daa_aaab)
        #expect(world.body(b2).flags == 0x0000_6000)

        // Static body is untouched.
        #expect(world.bodyTransform(b3s).p == Pos(10, 10, 10))
    }
}
