import Testing
@testable import Natsel

// Round-trip checks for the per-type joint tuning APIs.
@Suite("Joint API coverage")
struct JointApiCoverageTests {
    private struct Fixture {
        var world = World()
        var ground = BodyID()
        var body = BodyID()
    }

    private func makeFixture() -> Fixture {
        var f = Fixture()

        var def = BodyDef()
        def.type = .staticBody
        f.ground = f.world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 0, 0)
        f.body = f.world.createBody(def)

        var sd = ShapeDef()
        sd.density = 1
        _ = f.world.createHullShape(f.body, sd, makeBoxHull(0.4, 0.4, 0.4))
        return f
    }

    @Test("common joint accessors")
    func commonAccessors() {
        let f = makeFixture()

        var def = RevoluteJointDef()
        def.base.bodyIdA = f.ground
        def.base.bodyIdB = f.body
        let joint = f.world.createRevoluteJoint(def)

        #expect(f.world.jointIsValid(joint))
        #expect(f.world.jointType(joint) == .revoluteJoint)
        let bodies = f.world.jointBodies(joint)!
        #expect(bodies.a.index1 == f.ground.index1)
        #expect(bodies.b.index1 == f.body.index1)

        f.world.destroyJoint(joint.index1 - 1, false)
        #expect(f.world.jointIsValid(joint) == false)
    }

    @Test("revolute joint setters round trip")
    func revoluteApi() {
        let f = makeFixture()

        var def = RevoluteJointDef()
        def.base.bodyIdA = f.ground
        def.base.bodyIdB = f.body
        let joint = f.world.createRevoluteJoint(def)

        #expect(f.world.setRevoluteJointLimits(joint, -0.5, 0.75))
        let limits = f.world.revoluteJointLimits(joint)!
        #expect(limits.lower == -0.5 && limits.upper == 0.75)

        #expect(f.world.enableRevoluteJointLimit(joint, true))
        #expect(f.world.isRevoluteJointLimitEnabled(joint))

        #expect(f.world.enableRevoluteJointMotor(joint, true))
        #expect(f.world.isRevoluteJointMotorEnabled(joint))
        #expect(f.world.setRevoluteJointMotorSpeed(joint, 2.5))
        #expect(f.world.revoluteJointMotorSpeed(joint) == 2.5)
        #expect(f.world.setRevoluteJointMaxMotorTorque(joint, 40))
        #expect(f.world.revoluteJointMaxMotorTorque(joint) == 40)

        #expect(f.world.enableRevoluteJointSpring(joint, true))
        #expect(f.world.isRevoluteJointSpringEnabled(joint))
        #expect(f.world.setRevoluteJointSpringHertz(joint, 3))
        #expect(f.world.revoluteJointSpringHertz(joint) == 3)
        #expect(f.world.setRevoluteJointSpringDampingRatio(joint, 0.7))
        #expect(f.world.revoluteJointSpringDampingRatio(joint) == 0.7)

        #expect(f.world.revoluteJointAngle(joint) != nil)
    }

    @Test("prismatic joint setters round trip")
    func prismaticApi() {
        let f = makeFixture()

        var def = PrismaticJointDef()
        def.base.bodyIdA = f.ground
        def.base.bodyIdB = f.body
        let joint = f.world.createPrismaticJoint(def)

        #expect(f.world.setPrismaticJointLimits(joint, -1, 2))
        let limits = f.world.prismaticJointLimits(joint)!
        #expect(limits.lower == -1 && limits.upper == 2)

        #expect(f.world.enablePrismaticJointLimit(joint, true))
        #expect(f.world.isPrismaticJointLimitEnabled(joint))

        #expect(f.world.enablePrismaticJointMotor(joint, true))
        #expect(f.world.isPrismaticJointMotorEnabled(joint))
        #expect(f.world.setPrismaticJointMotorSpeed(joint, 1.5))
        #expect(f.world.prismaticJointMotorSpeed(joint) == 1.5)
        #expect(f.world.setPrismaticJointMaxMotorForce(joint, 30))
        #expect(f.world.prismaticJointMaxMotorForce(joint) == 30)

        #expect(f.world.enablePrismaticJointSpring(joint, true))
        #expect(f.world.isPrismaticJointSpringEnabled(joint))
        #expect(f.world.setPrismaticJointSpringHertz(joint, 5))
        #expect(f.world.prismaticJointSpringHertz(joint) == 5)
        #expect(f.world.setPrismaticJointSpringDampingRatio(joint, 0.5))
        #expect(f.world.prismaticJointSpringDampingRatio(joint) == 0.5)
        #expect(f.world.setPrismaticJointTargetTranslation(joint, 0.8))
        #expect(f.world.prismaticJointTargetTranslation(joint) == 0.8)
    }

    @Test("wheel joint setters round trip")
    func wheelApi() {
        let f = makeFixture()

        var def = WheelJointDef()
        def.base.bodyIdA = f.ground
        def.base.bodyIdB = f.body
        let joint = f.world.createWheelJoint(def)

        #expect(f.world.enableWheelJointSpinMotor(joint, true))
        #expect(f.world.isWheelJointSpinMotorEnabled(joint))
        #expect(f.world.setWheelJointSpinSpeed(joint, 3))
        #expect(f.world.wheelJointSpinSpeed(joint) == 3)
        #expect(f.world.setWheelJointMaxSpinTorque(joint, 25))
        #expect(f.world.wheelJointMaxSpinTorque(joint) == 25)

        #expect(f.world.setWheelJointSuspensionSpring(joint, hertz: 4, dampingRatio: 0.6))
        let suspension = f.world.wheelJointSuspensionSpring(joint)!
        #expect(suspension.hertz == 4 && suspension.dampingRatio == 0.6)

        #expect(f.world.setWheelJointSuspensionLimits(joint, -0.2, 0.2))
        let susLimits = f.world.wheelJointSuspensionLimits(joint)!
        #expect(susLimits.lower == -0.2 && susLimits.upper == 0.2)

        #expect(f.world.enableWheelJointSteering(joint, true))
        #expect(f.world.isWheelJointSteeringEnabled(joint))
        #expect(f.world.setWheelJointTargetSteeringAngle(joint, 0.3))
        #expect(f.world.wheelJointTargetSteeringAngle(joint) == 0.3)
        #expect(f.world.setWheelJointMaxSteeringTorque(joint, 15))
        #expect(f.world.wheelJointMaxSteeringTorque(joint) == 15)

        #expect(f.world.setWheelJointSteeringSpring(joint, hertz: 2, dampingRatio: 0.4))
        let steering = f.world.wheelJointSteeringSpring(joint)!
        #expect(steering.hertz == 2 && steering.dampingRatio == 0.4)

        #expect(f.world.setWheelJointSteeringLimits(joint, -0.5, 0.5))
        let steerLimits = f.world.wheelJointSteeringLimits(joint)!
        #expect(steerLimits.lower == -0.5 && steerLimits.upper == 0.5)
    }

    @Test("spherical, motor, weld and parallel setters round trip")
    func otherJointApis() {
        let f = makeFixture()

        // Spherical
        var sdef = SphericalJointDef()
        sdef.base.bodyIdA = f.ground
        sdef.base.bodyIdB = f.body
        let spherical = f.world.createSphericalJoint(sdef)

        #expect(f.world.setSphericalJointConeLimit(spherical, 0.6))
        #expect(f.world.enableSphericalJointConeLimit(spherical, true))
        let cone = f.world.sphericalJointConeLimit(spherical)!
        #expect(cone.enabled && cone.coneAngle == 0.6)

        #expect(f.world.setSphericalJointTwistLimits(spherical, -0.4, 0.4))
        let twist = f.world.sphericalJointTwistLimits(spherical)!
        #expect(twist.lower == -0.4 && twist.upper == 0.4)

        #expect(f.world.enableSphericalJointMotor(spherical, true))
        #expect(f.world.setSphericalJointMotorVelocity(spherical, Vec3(0.5, 0, 1)))
        #expect(f.world.sphericalJointMotorVelocity(spherical) == Vec3(0.5, 0, 1))
        #expect(f.world.setSphericalJointMaxMotorTorque(spherical, 20))
        #expect(f.world.sphericalJointMaxMotorTorque(spherical) == 20)

        // Motor
        var mdef = MotorJointDef()
        mdef.base.bodyIdA = f.ground
        mdef.base.bodyIdB = f.body
        let motor = f.world.createMotorJoint(mdef)

        #expect(f.world.setMotorJointLinearVelocity(motor, Vec3(1, 0, 0)))
        #expect(f.world.motorJointLinearVelocity(motor) == Vec3(1, 0, 0))
        #expect(f.world.setMotorJointAngularVelocity(motor, Vec3(0, 2, 0)))
        #expect(f.world.motorJointAngularVelocity(motor) == Vec3(0, 2, 0))
        #expect(f.world.setMotorJointMaxVelocityForce(motor, 50))
        #expect(f.world.motorJointMaxVelocityForce(motor) == 50)
        #expect(f.world.setMotorJointMaxVelocityTorque(motor, 60))
        #expect(f.world.motorJointMaxVelocityTorque(motor) == 60)
        #expect(f.world.setMotorJointLinearSpring(motor, hertz: 3, dampingRatio: 0.5))
        let linearSpring = f.world.motorJointLinearSpring(motor)!
        #expect(linearSpring.hertz == 3 && linearSpring.dampingRatio == 0.5)
        #expect(f.world.setMotorJointAngularSpring(motor, hertz: 2, dampingRatio: 0.3))
        let angularSpring = f.world.motorJointAngularSpring(motor)!
        #expect(angularSpring.hertz == 2 && angularSpring.dampingRatio == 0.3)
        #expect(f.world.setMotorJointMaxSpringForce(motor, 80))
        #expect(f.world.motorJointMaxSpringForce(motor) == 80)
        #expect(f.world.setMotorJointMaxSpringTorque(motor, 90))
        #expect(f.world.motorJointMaxSpringTorque(motor) == 90)

        // Weld
        var wdef = WeldJointDef()
        wdef.base.bodyIdA = f.ground
        wdef.base.bodyIdB = f.body
        let weld = f.world.createWeldJoint(wdef)

        #expect(f.world.setWeldJointLinearSpring(weld, hertz: 6, dampingRatio: 0.8))
        let weldLinear = f.world.weldJointLinearSpring(weld)!
        #expect(weldLinear.hertz == 6 && weldLinear.dampingRatio == 0.8)
        #expect(f.world.setWeldJointAngularSpring(weld, hertz: 7, dampingRatio: 0.9))
        let weldAngular = f.world.weldJointAngularSpring(weld)!
        #expect(weldAngular.hertz == 7 && weldAngular.dampingRatio == 0.9)

        // Parallel
        var pdef = ParallelJointDef()
        pdef.base.bodyIdA = f.ground
        pdef.base.bodyIdB = f.body
        let parallel = f.world.createParallelJoint(pdef)

        #expect(f.world.setParallelJointSpring(parallel, hertz: 9, dampingRatio: 0.2))
        let parallelSpring = f.world.parallelJointSpring(parallel)!
        #expect(parallelSpring.hertz == 9 && parallelSpring.dampingRatio == 0.2)
        #expect(f.world.setParallelJointMaxTorque(parallel, 100))
        #expect(f.world.parallelJointMaxTorque(parallel) == 100)
    }

    @Test("type mismatches are rejected")
    func typeMismatch() {
        let f = makeFixture()

        var def = RevoluteJointDef()
        def.base.bodyIdA = f.ground
        def.base.bodyIdB = f.body
        let joint = f.world.createRevoluteJoint(def)

        // Prismatic setters must not touch a revolute joint.
        #expect(f.world.setPrismaticJointMotorSpeed(joint, 5) == false)
        #expect(f.world.prismaticJointMotorSpeed(joint) == nil)
        #expect(f.world.wheelJointSpinSpeed(joint) == nil)
    }
}
