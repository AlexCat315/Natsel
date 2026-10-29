import Testing
@testable import Natsel

@Suite("Body extras")
struct BodyExtraTests {
    @Test("contact data reports only touching contacts")
    func contactData() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, Float(-0.5), 0)
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, Float(0.5), 0)
        let box = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        // Before settling there is no touching contact.
        #expect(world.bodyContactData(box).isEmpty)

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        let contacts = world.bodyContactData(box)
        #expect(contacts.count == 1)
        guard let first = contacts.first else { return }
        #expect(first.contactId.index1 != 0)
        #expect(first.manifolds.count > 0)

        // The capacity caps the result.
        #expect(world.bodyContactData(box, capacity: 0).isEmpty)
    }

    @Test("compute AABB covers every shape")
    func computeAABB() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0, 0)
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(-2, 0, 0), radius: 0.5))
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(2, 0, 0), radius: 0.5))

        world.step(1.0 / 60.0, subStepCount: 1)

        let aabb = world.bodyComputeAABB(body)
        #expect(aabb.lowerBound.x <= -2.5 + 1e-3)
        #expect(aabb.upperBound.x >= 2.5 - 1e-3)

        // A body with no shapes collapses to its origin.
        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(7, 8, 9)
        let empty = world.createBody(def)
        let emptyAABB = world.bodyComputeAABB(empty)
        #expect(absFloat(emptyAABB.lowerBound.x - 7) < 1e-5)
        #expect(absFloat(emptyAABB.upperBound.z - 9) < 1e-5)
    }

    @Test("applying mass from shapes recomputes the body mass")
    func applyMass() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)

        // Add a shape with density but do not let creation compute the mass.
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        shapeDef.updateBodyMass = false
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        world.applyBodyMassFromShapes(body)
        // A 1m cube at 1000 kg/m³ weighs 1000 kg.
        #expect(absFloat(world.bodyMass(body) - 1000) < 1)
    }

    @Test("closest point on a body finds the near shape")
    func closestPoint() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(-3, 0, 0), radius: 1))
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(3, 0, 0), radius: 1))

        // The query runs core-on-core (useRadii is false), so a sphere contributes its center.
        // A target on the far right lands on the right sphere's center.
        let right = world.bodyClosestPoint(body, Vec3(10, 0, 0))
        #expect(absFloat(right.point.x - 3) < 1e-3)
        #expect(absFloat(right.distance - 7) < 1e-3)

        // A target on the far left lands on the left sphere's center.
        let left = world.bodyClosestPoint(body, Vec3(-10, 0, 0))
        #expect(absFloat(left.point.x + 3) < 1e-3)
        #expect(absFloat(left.distance - 7) < 1e-3)
    }

    @Test("fast rotation flag and user data round trip")
    func flagsAndUserData() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)

        #expect(world.isBodyFastRotationAllowed(body) == false)
        world.allowBodyFastRotation(body, true)
        #expect(world.isBodyFastRotationAllowed(body))
        world.allowBodyFastRotation(body, false)
        #expect(world.isBodyFastRotationAllowed(body) == false)

        var token = 7
        let pointer = withUnsafeMutablePointer(to: &token) { UnsafeMutableRawPointer($0) }
        world.setBodyUserData(body, pointer)
        #expect(world.bodyUserData(body) == pointer)
        world.setBodyUserData(body, nil)
        #expect(world.bodyUserData(body) == nil)
    }

    @Test("a target transform drives the body toward the goal")
    func targetTransform() {
        let world = World()
        world.enableSleep = false
        world.gravity = Vec3()

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        // Ask the body to reach (6, 0, 0) in one second.
        world.setBodyTargetTransform(
            body, WorldTransform(Vec3(6, 0, 0), Quat.identity), timeStep: 1.0, wake: true)

        // The velocity is the center difference over the time step.
        let v = world.bodyLinearVelocity(body)
        #expect(absFloat(v.x - 6) < 1e-3)
        #expect(absFloat(v.y) < 1e-5)

        // Stepping moves it toward the goal.
        world.step(1.0 / 60.0, subStepCount: 1)
        #expect(world.bodyTransform(body).p.x > 0)

        // A static body is ignored.
        var staticDef = BodyDef()
        staticDef.type = .staticBody
        staticDef.position = Pos(20, 0, 0)
        let staticBody = world.createBody(staticDef)
        world.setBodyTargetTransform(
            staticBody, WorldTransform(Vec3(0, 0, 0), Quat.identity), timeStep: 1.0)
        #expect(world.bodyLinearVelocity(staticBody) == Vec3())
    }
}
