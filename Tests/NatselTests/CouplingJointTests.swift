import Testing
@testable import Natsel

private func disk(_ world: World, _ position: Vec3, _ radius: Float) -> BodyID {
    var def = BodyDef()
    def.type = .dynamicBody
    def.position = Pos(position)
    let body = world.createBody(def)
    _ = world.createCapsuleShape(body, ShapeDef(), Capsule(center1: Vec3(0, 0, -0.05), center2: Vec3(0, 0, 0.05), radius: radius))
    return body
}

/// A revolute joint about world z at the body's position.
private func hinge(_ world: World, _ ground: BodyID, _ body: BodyID, motorSpeed: Float? = nil) -> JointID {
    var def = RevoluteJointDef()
    def.base.bodyIdA = ground
    def.base.bodyIdB = body
    def.base.localFrameA = Transform(subPos(world.bodyPosition(body), .zero), .identity)
    def.base.localFrameB = Transform(Vec3(), .identity)
    if let motorSpeed {
        def.enableMotor = true
        def.motorSpeed = motorSpeed
        def.maxMotorTorque = 1000
    }
    return world.createRevoluteJoint(def)
}

/// Unwrapped rotation about z.
private struct AngleTracker {
    var angle: Float = 0
    var last: Float = 0
    mutating func update(_ q: Quat) {
        let twist = getTwistAngle(q)
        angle += unwindAngle(twist - last)
        last = twist
    }
}

@Suite("Coupling joints")
struct CouplingJointTests {
    @Test("gears keep their ratio over many revolutions", arguments: [true, false])
    func gear(_ useReferences: Bool) {
        let world = World()
        world.gravity = Vec3()
        let ground = world.createBody(BodyDef())
        let a = disk(world, Vec3(0, 0, 0), 0.5)
        let b = disk(world, Vec3(1.5, 0, 0), 1.0)
        let hingeA = hinge(world, ground, a, motorSpeed: 6)
        let hingeB = hinge(world, ground, b)

        var def = GearJointDef()
        def.base.bodyIdA = a
        def.base.bodyIdB = b
        def.ratio = 2
        if useReferences {
            def.referenceJointA = hingeA
            def.referenceJointB = hingeB
        }
        let gear = world.createGearJoint(def)
        #expect(world.jointType(gear) == .gearJoint)
        #expect(world.gearJointRatio(gear) == 2)

        var angleA = AngleTracker()
        var angleB = AngleTracker()
        for _ in 0..<600 {
            world.step(1.0 / 60.0, subStepCount: 4)
            angleA.update(world.bodyRotation(a))
            angleB.update(world.bodyRotation(b))
        }
        // A turned many times; B turned half as fast the other way.
        #expect(angleA.angle > 50)
        #expect(abs(angleA.angle + 2 * angleB.angle) < 0.02)
        #expect(world.jointConstraintTorque(gear).map { length($0) > 0 } == true)
    }

    @Test("rack follows the pinion")
    func rackAndPinion() {
        let world = World()
        world.gravity = Vec3()
        let ground = world.createBody(BodyDef())
        let pinion = disk(world, Vec3(0, 0, 0), 0.25)
        let pinionHinge = hinge(world, ground, pinion, motorSpeed: 2)

        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(0, Float(-0.4), 0)
        let rack = world.createBody(bodyDef)
        _ = world.createHullShape(rack, ShapeDef(), makeBoxHull(2, 0.1, 0.1))
        var slide = PrismaticJointDef()
        slide.base.bodyIdA = ground
        slide.base.bodyIdB = rack
        slide.base.localFrameA = Transform(Vec3(0, -0.4, 0), .identity)
        let rackSlide = world.createPrismaticJoint(slide)

        var def = RackAndPinionJointDef()
        def.base.bodyIdA = pinion
        def.base.bodyIdB = rack
        def.ratio = 0.25
        def.referencePinionJoint = pinionHinge
        def.referenceRackJoint = rackSlide
        let joint = world.createRackAndPinionJoint(def)
        #expect(joint.isNull == false)

        var angle = AngleTracker()
        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
            angle.update(world.bodyRotation(pinion))
        }
        let translation = world.bodyPosition(rack).x
        #expect(angle.angle > 3)
        #expect(abs(translation - Double(0.25 * angle.angle)) < 0.01)
        #expect(abs(world.bodyPosition(rack).y + 0.4) < 1.0e-3)
    }

    @Test("pulley keeps the rope length and the heavier side descends")
    func pulley() {
        let world = World()
        let ground = world.createBody(BodyDef())
        func box(_ x: Float, _ density: Float) -> BodyID {
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(x, 0, 0)
            let body = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = density
            _ = world.createHullShape(body, shapeDef, makeBoxHull(0.2, 0.2, 0.2))
            return body
        }
        let light = box(-1, 100)
        let heavy = box(1, 200)

        var def = PulleyJointDef()
        def.base.bodyIdA = light
        def.base.bodyIdB = heavy
        def.base.localFrameA = Transform(Vec3(0, 0.2, 0), .identity)
        def.base.localFrameB = Transform(Vec3(0, 0.2, 0), .identity)
        def.groundAnchorA = Pos(-1, 3, 0)
        def.groundAnchorB = Pos(1, 3, 0)
        def.ratio = 1
        def.base.collideConnected = true
        let pulley = world.createPulleyJoint(def)
        let range = world.pulleyJointLengthRange(pulley)!
        #expect(abs(range.min - 5.6) < 1.0e-4)
        _ = ground

        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
        let lengths = world.pulleyJointCurrentLengths(pulley)!
        #expect(abs(lengths.a + lengths.b - 5.6) < 0.01)
        #expect(world.bodyPosition(heavy).y < -0.5)
        #expect(world.bodyPosition(light).y > 0.5)
        // Expected acceleration (m2 - m1) / (m1 + m2) g = g / 3.
        let expected = 0.5 * (10.0 / 3.0) * 1.0
        #expect(abs(world.bodyPosition(light).y - Double(expected)) < 0.1)
        #expect(world.jointConstraintForce(pulley).map { length($0) > 0 } == true)

        // Slack rope: the light box falls freely until the rope tightens again.
        world.setPulleyJointLengthRange(pulley, 4, 8)
        world.setBodyLinearVelocity(light, Vec3())
        world.setBodyLinearVelocity(heavy, Vec3())
        for _ in 0..<10 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.bodyLinearVelocity(light).y < -1)
    }

    @Test("coupling joints draw and report through the generic joint API")
    func genericApi() {
        let world = World()
        let ground = world.createBody(BodyDef())
        let a = disk(world, Vec3(0, 0, 0), 0.5)
        let b = disk(world, Vec3(1.5, 0, 0), 1.0)
        _ = hinge(world, ground, a)
        _ = hinge(world, ground, b)
        var def = GearJointDef()
        def.base.bodyIdA = a
        def.base.bodyIdB = b
        let gear = world.createGearJoint(def)
        world.step(1.0 / 60.0, subStepCount: 4)
        var segments = 0
        var options = DebugDraw()
        options.drawJoints = true
        options.drawSegment = { _, _, _ in segments += 1 }
        world.draw(options)
        #expect(segments > 0)
        world.destroyJoint(gear.index1 - 1, true)
        #expect(world.jointIsValid(gear) == false)
    }

    @Test("a recycled joint slot invalidates the old id")
    func staleJointIds() {
        let world = World()
        let ground = world.createBody(BodyDef())
        let a = disk(world, Vec3(0, 0, 0), 0.5)
        let first = hinge(world, ground, a)
        world.destroyJoint(first.index1 - 1, false)
        let second = hinge(world, ground, a)
        #expect(second.index1 == first.index1)
        #expect(second.generation != first.generation)
        #expect(world.jointIsValid(first) == false)
        #expect(world.jointIsValid(second))
    }
}
