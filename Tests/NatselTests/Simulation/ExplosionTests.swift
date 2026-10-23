import Testing
@testable import Natsel

// Radial impulse field, ported from the C explosion path in physics_world.c.
@Suite("Explosion")
struct ExplosionTests {
    private func makeScene() -> (World, [BodyID]) {
        let world = World()
        world.enableSleep = false

        var ids: [BodyID] = []
        for i in 0..<5 {
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(Float(i) * 2 - 4, 0, 0)
            let body = world.createBody(def)

            var shapeDef = ShapeDef()
            shapeDef.density = 1000
            if i % 2 == 0 {
                _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 0.5))
            } else {
                _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
            }
            ids.append(body)
        }
        return (world, ids)
    }

    @Test("radial impulse matches the C reference")
    func explosion() {
        let (world, ids) = makeScene()

        var def = ExplosionDef()
        def.position = Pos(0, 0, 0)
        def.radius = 3.0
        def.falloff = 2.0
        def.impulsePerArea = 50.0
        world.explode(def)

        var hash: UInt64 = 14_695_981_039_346_656_037
        func feed<T>(_ value: T) {
            withUnsafeBytes(of: value) { raw in
                for byte in raw { hash = (hash ^ UInt64(byte)) &* 1_099_511_628_211 }
            }
        }

        for id in ids {
            feed(world.bodyLinearVelocity(id))
            feed(world.bodyAngularVelocity(id))
        }

        #expect(hash == 0xb6b6_a037_3c55_4fbd)

        // Bodies left of the centre are pushed away from it, symmetric about the origin.
        #expect(world.bodyLinearVelocity(ids[0]).x < 0)
        #expect(world.bodyLinearVelocity(ids[2]).x > 0)
        #expect(world.bodyLinearVelocity(ids[4]).x > 0)
    }

    @Test("shapes beyond radius plus falloff are untouched")
    func outOfRange() {
        let (world, ids) = makeScene()

        var def = ExplosionDef()
        def.position = Pos(0, 0, 0)
        def.radius = 0.1
        def.falloff = 0.1
        def.impulsePerArea = 50.0
        world.explode(def)

        // Only the body at the explosion centre is within range; the others are far outside.
        let velocities = ids.map { world.bodyLinearVelocity($0) }
        #expect(velocities[0] == Vec3())
        #expect(velocities[1] == Vec3())
        #expect(velocities[2].x.bitPattern == 0x3d99_9999)
        #expect(velocities[3] == Vec3())
        #expect(velocities[4] == Vec3())
    }

    @Test("zero explosion scale opts a shape out")
    func zeroScale() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        shapeDef.explosionScale = 0
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        var explosion = ExplosionDef()
        explosion.position = Pos(0, 0, 0)
        explosion.radius = 5
        explosion.falloff = 1
        explosion.impulsePerArea = 100
        world.explode(explosion)

        #expect(world.bodyLinearVelocity(body) == Vec3())
    }
}
