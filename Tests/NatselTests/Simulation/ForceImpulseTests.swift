import Testing
@testable import Natsel

// Force and impulse application, matching the C semantics in body.c.
@Suite("Forces and impulses")
struct ForceImpulseTests {
    private func makeBox(_ world: World, _ position: Pos = Pos()) -> BodyID {
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = position
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        return body
    }

    @Test("a force through the center accelerates the body")
    func forceToCenter() {
        let world = World()
        world.gravity = Vec3()
        world.enableSleep = false
        let body = makeBox(world)

        let mass = world.bodyMass(body)
        world.applyBodyForceToCenter(body, Vec3(0, mass * 10, 0))
        world.step(1.0 / 60.0, subStepCount: 1)

        // a = F/m = 10, so after one 1/60 step v = 10/60.
        let v = world.bodyLinearVelocity(body)
        #expect(absFloat(v.y - 10.0 / 60.0) < 1e-4)
        #expect(absFloat(v.x) < 1e-6)
    }

    @Test("an off-center force also produces torque")
    func forceOffCenter() {
        let world = World()
        world.gravity = Vec3()
        world.enableSleep = false
        let body = makeBox(world)

        let mass = world.bodyMass(body)
        // Push along +x at a point above the center: torque = r x F points along -z.
        world.applyBodyForce(body, Vec3(mass * 10, 0, 0), Pos(0, Float(0.5), 0))
        world.step(1.0 / 60.0, subStepCount: 1)

        let v = world.bodyLinearVelocity(body)
        let w = world.bodyAngularVelocity(body)
        #expect(absFloat(v.x - 10.0 / 60.0) < 1e-4)
        #expect(w.z < 0)
    }

    @Test("a pure torque spins the body without translating it")
    func torque() {
        let world = World()
        world.gravity = Vec3()
        world.enableSleep = false
        let body = makeBox(world)

        world.applyBodyTorque(body, Vec3(0, 0, 50))
        world.step(1.0 / 60.0, subStepCount: 1)

        #expect(length(world.bodyLinearVelocity(body)) < 1e-6)
        #expect(world.bodyAngularVelocity(body).z > 0)
    }

    @Test("forces are cleared after each step")
    func forcesClear() {
        let world = World()
        world.gravity = Vec3()
        world.enableSleep = false
        let body = makeBox(world)

        let mass = world.bodyMass(body)
        world.applyBodyForceToCenter(body, Vec3(0, mass * 6, 0))
        world.step(1.0 / 60.0, subStepCount: 1)
        let v1 = world.bodyLinearVelocity(body)

        // No new force: the velocity must not keep growing.
        world.step(1.0 / 60.0, subStepCount: 1)
        let v2 = world.bodyLinearVelocity(body)
        #expect(absFloat(v2.y - v1.y) < 1e-6)
    }

    @Test("a center impulse changes velocity immediately")
    func impulseToCenter() {
        let world = World()
        world.gravity = Vec3()
        world.enableSleep = false
        let body = makeBox(world)

        let mass = world.bodyMass(body)
        world.applyBodyLinearImpulseToCenter(body, Vec3(0, mass * 3, 0))

        // The velocity changes without stepping.
        let v = world.bodyLinearVelocity(body)
        #expect(absFloat(v.y - 3) < 1e-5)
        #expect(length(world.bodyAngularVelocity(body)) < 1e-6)
    }

    @Test("an off-center impulse also adds spin")
    func impulseOffCenter() {
        let world = World()
        world.gravity = Vec3()
        world.enableSleep = false
        let body = makeBox(world)

        let mass = world.bodyMass(body)
        world.applyBodyLinearImpulse(body, Vec3(mass * 3, 0, 0), Pos(0, Float(0.5), 0))

        let v = world.bodyLinearVelocity(body)
        #expect(absFloat(v.x - 3) < 1e-5)
        // r x F with r = +y and F = +x gives -z.
        #expect(world.bodyAngularVelocity(body).z < 0)
    }

    @Test("an angular impulse only spins")
    func angularImpulse() {
        let world = World()
        world.gravity = Vec3()
        world.enableSleep = false
        let body = makeBox(world)

        world.applyBodyAngularImpulse(body, Vec3(0, 0, 5))

        #expect(length(world.bodyLinearVelocity(body)) < 1e-6)
        #expect(world.bodyAngularVelocity(body).z > 0)
    }

    @Test("applying to a sleeping body wakes it")
    func wakesBody() {
        let world = World()
        world.gravity = Vec3(0, -10, 0)

        var groundDef = BodyDef()
        groundDef.type = .staticBody
        let ground = world.createBody(groundDef)
        var groundShape = ShapeDef()
        _ = world.createHullShape(ground, groundShape, makeBoxHull(10, 0.5, 10))

        let body = makeBox(world, Pos(0, 2, 0))

        // Let it settle and sleep.
        for _ in 0..<400 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.isBodyAwake(body) == false)

        world.applyBodyForceToCenter(body, Vec3(0, 1000, 0))
        #expect(world.isBodyAwake(body))
    }

    @Test("body shape count and joint list reflect the body")
    func bodyAssociations() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.3, 0.3, 0.3))
        #expect(world.bodyShapeCount(body) == 2)
        #expect(world.bodyJoints(body).isEmpty)

        def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)

        var jointDef = RevoluteJointDef()
        jointDef.base.bodyIdA = ground
        jointDef.base.bodyIdB = body
        let joint = world.createRevoluteJoint(jointDef)

        let joints = world.bodyJoints(body)
        #expect(joints.count == 1)
        #expect(joints[0].index1 == joint.index1)
    }
}
