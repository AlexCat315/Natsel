import Testing
@testable import Natsel

@Suite("World tuning")
struct WorldTuningTests {
    @Test("gravity drives a falling body")
    func gravity() {
        let world = World()
        world.enableSleep = false
        world.gravity = Vec3(0, -20, 0)

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 10, 0)
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        world.step(1.0 / 60.0, subStepCount: 1)

        // After one step the velocity is g * dt.
        let v = world.bodyLinearVelocity(body)
        #expect(absFloat(v.y + 20.0 / 60.0) < 1e-4)
    }

    @Test("zero gravity leaves a free body still")
    func zeroGravity() {
        let world = World()
        world.enableSleep = false
        world.gravity = Vec3()

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 10, 0)
        def.linearVelocity = Vec3(3, 0, 0)
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 1) }

        // Constant velocity, no acceleration.
        let v = world.bodyLinearVelocity(body)
        #expect(absFloat(v.x - 3) < 1e-4)
        #expect(absFloat(v.y) < 1e-6)
    }

    @Test("contact tuning round trips")
    func contactTuning() {
        let world = World()
        world.setContactTuning(hertz: 45, dampingRatio: 6, contactSpeed: 2.5)

        #expect(world.contactHertz == 45)
        #expect(world.contactDampingRatio == 6)
        #expect(world.contactSpeed == 2.5)

        // A negative contact speed clamps to zero.
        world.setContactTuning(hertz: 30, dampingRatio: 10, contactSpeed: -5)
        #expect(world.contactSpeed == 0)
    }

    @Test("world toggles are readable back")
    func toggles() {
        let world = World()

        #expect(world.isSleepingEnabled())
        #expect(world.isWarmStartingEnabled())
        #expect(world.isContinuousEnabled())

        world.enableSleeping(false)
        #expect(world.isSleepingEnabled() == false)

        world.enableWarmStarting(false)
        #expect(world.isWarmStartingEnabled() == false)

        world.enableContinuous(false)
        #expect(world.isContinuousEnabled() == false)

        // Continuous collision off lets a fast body tunnel.
        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(5, 0, 0)
        let wall = world.createBody(def)
        _ = world.createHullShape(wall, ShapeDef(), makeBoxHull(0.05, 5, 5))

        def = BodyDef()
        def.type = .dynamicBody
        def.isBullet = true
        def.gravityScale = 0
        def.linearVelocity = Vec3(200, 0, 0)
        let bullet = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = world.createSphereShape(bullet, shapeDef, Sphere(center: Vec3(), radius: 0.1))

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.bodyTransform(bullet).p.x > 5)
    }

    @Test("counters and profile reflect the step")
    func countersAndProfile() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 2, 0)
        let box = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        let counters = world.counters()
        #expect(counters.bodyCount == 2)
        #expect(counters.contactCount > 0)
        // A single resting box on the ground is one leaf in each tree.
        #expect(counters.shapeCount == 2)

        #expect(world.awakeBodyCountPublic() == 1)

        let profile = world.profileData()
        #expect(profile.step >= 0)
    }

    @Test("the maximum linear speed clamps velocity")
    func maxLinearSpeed() {
        let world = World()
        world.enableSleep = false
        world.gravity = Vec3()
        world.maxLinearSpeed = 50

        var def = BodyDef()
        def.type = .dynamicBody
        def.linearVelocity = Vec3(500, 0, 0)
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        // An impulse beyond the cap is clamped.
        world.applyBodyLinearImpulseToCenter(body, Vec3(1000, 0, 0))
        let v = world.bodyLinearVelocity(body)
        #expect(length(v) <= 50 + 1e-3)
    }
}
