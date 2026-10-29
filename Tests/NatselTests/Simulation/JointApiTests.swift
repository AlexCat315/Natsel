import Testing
@testable import Natsel

@Suite("Joint runtime API")
struct JointApiTests {
    private struct Fixture {
        var world = World()
        var ground = BodyID()
        var body = BodyID()
    }

    private func makeFixture() -> Fixture {
        var fixture = Fixture()

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 0, 0)
        fixture.ground = fixture.world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 0, 0)
        fixture.body = fixture.world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = fixture.world.createHullShape(fixture.body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        return fixture
    }

    @Test("distance joint setters round trip")
    func distanceJointApi() {
        let f = makeFixture()

        var def = DistanceJointDef()
        def.base.bodyIdA = f.ground
        def.base.bodyIdB = f.body
        def.length = 2.0
        let joint = f.world.createDistanceJoint(def)

        f.world.setDistanceJointLength(joint, 3.0)
        #expect(f.world.distanceJointLength(joint) == 3.0)

        f.world.enableDistanceJointSpring(joint, true)
        #expect(f.world.isDistanceJointSpringEnabled(joint) == true)

        f.world.setDistanceJointSpringForceRange(joint, -50.0, 75.0)
        let range = f.world.distanceJointSpringForceRange(joint)!
        #expect(range.lower == -50.0 && range.upper == 75.0)

        f.world.setDistanceJointSpringHertz(joint, 4.0)
        #expect(f.world.distanceJointSpringHertz(joint) == 4.0)

        f.world.setDistanceJointSpringDampingRatio(joint, 0.6)
        #expect(f.world.distanceJointSpringDampingRatio(joint) == 0.6)

        f.world.enableDistanceJointLimit(joint, true)
        #expect(f.world.isDistanceJointLimitEnabled(joint) == true)

        f.world.setDistanceJointLengthRange(joint, 1.0, 5.0)
        #expect(f.world.distanceJointMinLength(joint) == 1.0)
        #expect(f.world.distanceJointMaxLength(joint) == 5.0)
        #expect(f.world.distanceJointCurrentLength(joint) != nil)

        f.world.enableDistanceJointMotor(joint, true)
        #expect(f.world.isDistanceJointMotorEnabled(joint) == true)

        f.world.setDistanceJointMotorSpeed(joint, 1.5)
        #expect(f.world.distanceJointMotorSpeed(joint) == 1.5)

        f.world.setDistanceJointMaxMotorForce(joint, 25.0)
        #expect(f.world.distanceJointMaxMotorForce(joint) == 25.0)

        f.world.destroyJoint(joint.index1 - 1, false)
        #expect(f.world.distanceJointSim(joint) == nil)
    }

    @Test("length clamps to the linear slop")
    func lengthClamp() {
        let f = makeFixture()

        var def = DistanceJointDef()
        def.base.bodyIdA = f.ground
        def.base.bodyIdB = f.body
        let joint = f.world.createDistanceJoint(def)

        f.world.setDistanceJointLength(joint, -5)
        #expect(f.world.distanceJointLength(joint) == B3.linearSlop)

        // The range normalizes so min <= max regardless of the argument order.
        f.world.setDistanceJointLengthRange(joint, 5.0, 1.0)
        #expect(f.world.distanceJointMinLength(joint) == 1.0)
        #expect(f.world.distanceJointMaxLength(joint) == 5.0)
    }

    @Test("filter joint prevents contact but keeps both bodies")
    func filterJoint() {
        let f = makeFixture()

        var def = FilterJointDef()
        def.base.bodyIdA = f.ground
        def.base.bodyIdB = f.body
        let joint = f.world.createFilterJoint(def)

        // The filter joint connects the bodies, so moving the body must keep it valid.
        f.world.step(1.0 / 60.0, subStepCount: 4)

        var contactsBetween = 0
        for c in f.world.contacts where c.contactId != B3.nullIndex {
            let shapeA = f.world.shapes[Int(c.shapeIdA)]
            let shapeB = f.world.shapes[Int(c.shapeIdB)]
            if (shapeA.bodyId == f.ground.index1 - 1 && shapeB.bodyId == f.body.index1 - 1)
                || (shapeA.bodyId == f.body.index1 - 1 && shapeB.bodyId == f.ground.index1 - 1)
            {
                contactsBetween += 1
            }
        }
        #expect(contactsBetween == 0)

        f.world.destroyJoint(joint.index1 - 1, true)
        #expect(f.world.joints[Int(joint.index1 - 1)].jointId == B3.nullIndex)
    }
}
