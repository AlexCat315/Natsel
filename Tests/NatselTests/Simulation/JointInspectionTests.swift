import Testing
@testable import Natsel

// Joint inspection API matching b3Joint_* in the C header.
@Suite("Joint inspection")
struct JointInspectionTests {
    private func makePendulum(jointType: JointType) -> (World, JointID, BodyID, BodyID) {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 5, 0)
        let anchor = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 5, 0)
        let bob = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bob, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jointDef = DistanceJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.base.localFrameA = Transform(Vec3(), Quat.identity)
        jointDef.base.localFrameB = Transform(Vec3(-1, 0, 0), Quat.identity)
        jointDef.length = 1
        let joint = world.createDistanceJoint(jointDef)

        return (world, joint, anchor, bob)
    }

    @Test("collide connected round trips")
    func collideConnected() {
        var (world, joint, _, _) = makePendulum(jointType: .distanceJoint)

        #expect(world.jointCollideConnected(joint) == false)
        world.setJointCollideConnected(joint, true)
        #expect(world.jointCollideConnected(joint))
        world.setJointCollideConnected(joint, false)
        #expect(world.jointCollideConnected(joint) == false)
    }

    @Test("constraint tuning round trips")
    func constraintTuning() {
        var (world, joint, _, _) = makePendulum(jointType: .distanceJoint)

        #expect(world.setJointConstraintTuning(joint, hertz: 12, dampingRatio: 0.3))
        let tuning = world.jointConstraintTuning(joint)!
        #expect(absFloat(tuning.hertz - 12) < 1e-6)
        #expect(absFloat(tuning.dampingRatio - 0.3) < 1e-6)
    }

    @Test("force and torque thresholds round trip")
    func thresholds() {
        var (world, joint, _, _) = makePendulum(jointType: .distanceJoint)

        #expect(world.setJointForceThreshold(joint, 250))
        #expect(absFloat(world.jointForceThreshold(joint)! - 250) < 1e-6)

        #expect(world.setJointTorqueThreshold(joint, 75))
        #expect(absFloat(world.jointTorqueThreshold(joint)! - 75) < 1e-6)
    }

    @Test("local frames round trip")
    func localFrames() {
        var (world, joint, _, _) = makePendulum(jointType: .distanceJoint)

        let frameA = Transform(Vec3(0.1, 0.2, 0.3), Quat.identity)
        let frameB = Transform(Vec3(-0.4, 0.5, 0.6), Quat.identity)
        #expect(world.setJointLocalFrameA(joint, frameA))
        #expect(world.setJointLocalFrameB(joint, frameB))

        let backA = world.jointLocalFrameA(joint)!
        let backB = world.jointLocalFrameB(joint)!
        #expect(absFloat(backA.p.x - 0.1) < 1e-6)
        #expect(absFloat(backB.p.x + 0.4) < 1e-6)
    }

    @Test("a satisfied distance joint reports no linear separation")
    func separation() {
        var (world, joint, _, _) = makePendulum(jointType: .distanceJoint)

        // The anchors coincide when the joint is built.
        #expect(absFloat(world.jointLinearSeparation(joint)!) < 1e-5)
        #expect(absFloat(world.jointAngularSeparation(joint)!) < 1e-5)
    }

    @Test("a stretched joint reports a positive separation")
    func stretchedSeparation() {
        var (world, joint, _, bob) = makePendulum(jointType: .distanceJoint)

        // Move the bob away from the anchor; the joint is now violated.
        world.setBodyTransform(bob, Pos(3, 5, 0), Quat.identity)
        let separation = world.jointLinearSeparation(joint)!
        #expect(separation > 1.5)
    }

    @Test("a distance joint carries a constraint force under load")
    func constraintForce() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 5, 0)
        let anchor = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 4, 0)
        let bob = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bob, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jointDef = DistanceJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.length = 1
        let joint = world.createDistanceJoint(jointDef)

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        // The bob hangs from the joint, so the force must support its weight.
        let force = world.jointConstraintForce(joint)!
        let weight = world.bodyMass(bob) * 10
        #expect(force.y > weight * 0.5)

        // A distance joint cannot transmit torque.
        let torque = world.jointConstraintTorque(joint)!
        #expect(length(torque) < 1e-3)
    }

    @Test("waking the joint wakes its bodies")
    func wakeBodies() {
        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 5, 0)
        let anchor = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 4, 0)
        let bob = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bob, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jointDef = DistanceJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.length = 1
        let joint = world.createDistanceJoint(jointDef)

        for _ in 0..<400 { world.step(1.0 / 60.0, subStepCount: 4) }
        world.wakeJointBodies(joint)
        #expect(world.isBodyAwake(bob))
        #expect(world.jointIsAwake(joint))
    }
}

// MARK: - Per-type scalar accessors

extension JointInspectionTests {
    @Test("suspension and steering scalars round trip independently")
    func wheelScalars() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let chassis = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, -1, 0)
        let wheel = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(wheel, shapeDef, makeBoxHull(0.3, 0.3, 0.1))

        var jointDef = WheelJointDef()
        jointDef.base.bodyIdA = chassis
        jointDef.base.bodyIdB = wheel
        let joint = world.createWheelJoint(jointDef)

        world.setWheelJointSuspensionHertz(joint, 7)
        #expect(absFloat(world.wheelJointSuspensionHertz(joint)! - 7) < 1e-6)

        world.setWheelJointSuspensionDampingRatio(joint, 0.45)
        #expect(absFloat(world.wheelJointSuspensionDampingRatio(joint)! - 0.45) < 1e-6)

        world.setWheelJointSteeringHertz(joint, 3)
        #expect(absFloat(world.wheelJointSteeringHertz(joint)! - 3) < 1e-6)

        world.setWheelJointSteeringDampingRatio(joint, 0.25)
        #expect(absFloat(world.wheelJointSteeringDampingRatio(joint)! - 0.25) < 1e-6)

        world.setWheelJointSpinMotorSpeed(joint, 12)
        #expect(absFloat(world.wheelJointSpinMotorSpeed(joint)! - 12) < 1e-6)

        // The two hertz values stay independent.
        #expect(world.wheelJointSuspensionHertz(joint) != world.wheelJointSteeringHertz(joint))
    }

    @Test("motor and weld hertz/damping pairs round trip")
    func springPairs() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let base = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        let other = world.createBody(def)

        var motorDef = MotorJointDef()
        motorDef.base.bodyIdA = base
        motorDef.base.bodyIdB = other
        let motor = world.createMotorJoint(motorDef)

        world.setMotorJointLinearHertz(motor, 4)
        world.setMotorJointLinearDampingRatio(motor, 0.6)
        world.setMotorJointAngularHertz(motor, 2)
        world.setMotorJointAngularDampingRatio(motor, 0.8)

        #expect(absFloat(world.motorJointLinearHertz(motor)! - 4) < 1e-6)
        #expect(absFloat(world.motorJointLinearDampingRatio(motor)! - 0.6) < 1e-6)
        #expect(absFloat(world.motorJointAngularHertz(motor)! - 2) < 1e-6)
        #expect(absFloat(world.motorJointAngularDampingRatio(motor)! - 0.8) < 1e-6)

        var weldDef = WeldJointDef()
        weldDef.base.bodyIdA = base
        weldDef.base.bodyIdB = other
        let weld = world.createWeldJoint(weldDef)

        world.setWeldJointLinearHertz(weld, 9)
        world.setWeldJointLinearDampingRatio(weld, 0.1)
        world.setWeldJointAngularHertz(weld, 5)
        world.setWeldJointAngularDampingRatio(weld, 0.9)

        #expect(absFloat(world.weldJointLinearHertz(weld)! - 9) < 1e-6)
        #expect(absFloat(world.weldJointLinearDampingRatio(weld)! - 0.1) < 1e-6)
        #expect(absFloat(world.weldJointAngularHertz(weld)! - 5) < 1e-6)
        #expect(absFloat(world.weldJointAngularDampingRatio(weld)! - 0.9) < 1e-6)
    }

    @Test("limit accessors report the configured bounds")
    func limitAccessors() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let base = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 0, 0)
        let other = world.createBody(def)

        var revDef = RevoluteJointDef()
        revDef.base.bodyIdA = base
        revDef.base.bodyIdB = other
        let revolute = world.createRevoluteJoint(revDef)

        world.setRevoluteJointLimits(revolute, -0.4, 0.6)
        #expect(absFloat(world.revoluteJointLowerLimit(revolute)! + 0.4) < 1e-6)
        #expect(absFloat(world.revoluteJointUpperLimit(revolute)! - 0.6) < 1e-6)

        world.enableRevoluteJointLimit(revolute, true)
        #expect(world.isRevoluteJointLimitEnabled(revolute))

        world.setRevoluteJointTargetAngle(revolute, 0.25)
        #expect(absFloat(world.revoluteJointTargetAngle(revolute)! - 0.25) < 1e-6)

        var prismDef = PrismaticJointDef()
        prismDef.base.bodyIdA = base
        prismDef.base.bodyIdB = other
        let prismatic = world.createPrismaticJoint(prismDef)

        world.setPrismaticJointLimits(prismatic, -1.5, 2.5)
        #expect(absFloat(world.prismaticJointLowerLimit(prismatic)! + 1.5) < 1e-6)
        #expect(absFloat(world.prismaticJointUpperLimit(prismatic)! - 2.5) < 1e-6)

        world.enablePrismaticJointMotor(prismatic, true)
        #expect(world.isPrismaticJointMotorEnabled(prismatic))

        world.enablePrismaticJointSpring(prismatic, true)
        #expect(world.isPrismaticJointSpringEnabled(prismatic))
    }

    @Test("prismatic translation and speed track the body pose")
    func prismaticKinematics() {
        let world = World()
        world.enableSleep = false
        world.gravity = Vec3()

        var def = BodyDef()
        def.type = .staticBody
        let base = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0, 0)
        let slider = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(slider, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jointDef = PrismaticJointDef()
        jointDef.base.bodyIdA = base
        jointDef.base.bodyIdB = slider
        let joint = world.createPrismaticJoint(jointDef)

        // Resting at the origin: no translation.
        #expect(absFloat(world.prismaticJointTranslationValue(joint)!) < 1e-5)

        // Drive it along +x and let it move.
        world.applyBodyForceToCenter(slider, Vec3(0, 0, 0))
        world.setBodyLinearVelocity(slider, Vec3(2, 0, 0))
        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        let translation = world.prismaticJointTranslationValue(joint)!
        #expect(translation > 0.5)

        // The speed follows the linear velocity along the axis.
        let speed = world.prismaticJointSpeedValue(joint)!
        #expect(absFloat(speed - 2) < 1e-3)
    }

    @Test("spherical spring and limit scalars round trip")
    func sphericalScalars() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let base = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, -1, 0)
        let bob = world.createBody(def)

        var jointDef = SphericalJointDef()
        jointDef.base.bodyIdA = base
        jointDef.base.bodyIdB = bob
        let joint = world.createSphericalJoint(jointDef)

        world.setSphericalJointSpringHertz(joint, 6)
        world.setSphericalJointSpringDampingRatio(joint, 0.35)
        #expect(absFloat(world.sphericalJointSpringHertz(joint)! - 6) < 1e-6)
        #expect(absFloat(world.sphericalJointSpringDampingRatio(joint)! - 0.35) < 1e-6)

        world.enableSphericalJointSpring(joint, true)
        #expect(world.isSphericalJointSpringEnabled(joint))

        world.enableSphericalJointMotor(joint, true)
        #expect(world.isSphericalJointMotorEnabled(joint))

        world.setSphericalJointConeLimit(joint, 0.5)
        world.enableSphericalJointConeLimit(joint, true)
        #expect(absFloat(world.sphericalJointConeAngle(joint)! - 0.5) < 1e-6)
        #expect(world.isSphericalJointConeLimitEnabled(joint))

        world.setSphericalJointTwistLimits(joint, -0.3, 0.3)
        #expect(absFloat(world.sphericalJointLowerTwistLimit(joint)! + 0.3) < 1e-6)
        #expect(absFloat(world.sphericalJointUpperTwistLimit(joint)! - 0.3) < 1e-6)
    }

    @Test("parallel spring scalars round trip")
    func parallelScalars() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let base = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        let other = world.createBody(def)

        var jointDef = ParallelJointDef()
        jointDef.base.bodyIdA = base
        jointDef.base.bodyIdB = other
        let joint = world.createParallelJoint(jointDef)

        world.setParallelJointHertz(joint, 8)
        world.setParallelJointDampingRatio(joint, 0.55)
        #expect(absFloat(world.parallelJointHertz(joint)! - 8) < 1e-6)
        #expect(absFloat(world.parallelJointDampingRatio(joint)! - 0.55) < 1e-6)
    }
}

// MARK: - Joint lifecycle interaction

extension JointInspectionTests {
    /// Creating a joint with collideConnected off must drop contacts that already exist between the
    /// attached bodies.
    @Test("creating a non-colliding joint clears existing contacts")
    func createJointClearsContacts() {
        let world = World()
        world.gravity = Vec3()

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0, 0)
        let bodyA = world.createBody(def)

        def.position = Pos(0.5, 0, 0)
        let bodyB = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = world.createHullShape(bodyA, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        world.step(1.0 / 60.0, subStepCount: 4)
        #expect(world.bodyContactData(bodyA).isEmpty == false)

        var jointDef = RevoluteJointDef()
        jointDef.base.bodyIdA = bodyA
        jointDef.base.bodyIdB = bodyB
        jointDef.base.collideConnected = false
        _ = world.createRevoluteJoint(jointDef)

        // The contact is gone immediately; the joint does not wait for the next step.
        #expect(world.bodyContactData(bodyA).isEmpty)

        // Stepping keeps them apart.
        world.step(1.0 / 60.0, subStepCount: 4)
        #expect(world.bodyContactData(bodyA).isEmpty)
    }

    /// A weld across a static and a kinematic body must survive type changes and a disable/enable
    /// cycle on one of the bodies.
    @Test("a non-dynamic joint survives type changes and disable cycles")
    func nonDynamicJointSurvives() {
        let world = World()
        world.gravity = Vec3()

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0, 0)
        let bodyA = world.createBody(def)

        def.position = Pos(2, 0, 0)
        let bodyB = world.createBody(def)

        var shapeDef = ShapeDef()
        _ = world.createHullShape(bodyA, shapeDef, makeCubeHull(0.5))
        _ = world.createHullShape(bodyB, shapeDef, makeCubeHull(0.5))

        var jointDef = WeldJointDef()
        jointDef.base.bodyIdA = bodyA
        jointDef.base.bodyIdB = bodyB
        let weld = world.createWeldJoint(jointDef)

        world.setBodyType(bodyA, .staticBody)
        world.setBodyType(bodyB, .kinematicBody)
        world.step(1.0 / 60.0, subStepCount: 4)

        world.disableBody(bodyB)
        world.step(1.0 / 60.0, subStepCount: 4)

        world.enableBody(bodyB)
        world.step(1.0 / 60.0, subStepCount: 4)

        #expect(world.jointIsValid(weld))
        #expect(world.jointType(weld) == .weldJoint)
    }

    /// Creating a colliding joint leaves the contact in place.
    @Test("creating a colliding joint keeps existing contacts")
    func createJointKeepsContacts() {
        let world = World()
        world.gravity = Vec3()

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0, 0)
        let bodyA = world.createBody(def)

        def.position = Pos(0.5, 0, 0)
        let bodyB = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = world.createHullShape(bodyA, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        _ = world.createHullShape(bodyB, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        world.step(1.0 / 60.0, subStepCount: 4)
        #expect(world.bodyContactData(bodyA).isEmpty == false)

        var jointDef = RevoluteJointDef()
        jointDef.base.bodyIdA = bodyA
        jointDef.base.bodyIdB = bodyB
        jointDef.base.collideConnected = true
        _ = world.createRevoluteJoint(jointDef)

        #expect(world.bodyContactData(bodyA).isEmpty == false)
    }
}
