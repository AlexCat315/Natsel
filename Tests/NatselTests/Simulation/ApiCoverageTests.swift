import Testing
@testable import Natsel

// Covers the accessors added alongside the port that do not have a dedicated test elsewhere.
@Suite("API coverage")
struct ApiCoverageTests {
    private func makeBody(_ world: World, _ type: BodyType = .dynamicBody) -> BodyID {
        var def = BodyDef()
        def.type = type
        return world.createBody(def)
    }

    // MARK: Filtering

    @Test("shape filters gate pair creation")
    func filters() {
        var a = Filter()
        a.categoryBits = 0b0001
        a.maskBits = 0b0010

        var b = Filter()
        b.categoryBits = 0b0010
        b.maskBits = 0b0001

        // Mutual interest.
        #expect(shouldShapesCollide(a, b))

        // One side loses interest.
        var c = Filter()
        c.categoryBits = 0b0010
        c.maskBits = 0
        #expect(shouldShapesCollide(a, c) == false)
    }

    @Test("setting a shape filter is observable")
    func shapeFilter() {
        let world = World()
        let body = makeBody(world)

        var shapeDef = ShapeDef()
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        var filter = Filter()
        filter.categoryBits = 0b1000
        filter.maskBits = 0b0111
        world.setShapeFilter(shape, filter)

        #expect(world.shapeFilter(shape).categoryBits == 0b1000)
        #expect(world.shapeFilter(shape).maskBits == 0b0111)
    }

    // MARK: Shape geometry accessors

    @Test("typed geometry accessors agree with shapeType")
    func geometryAccessors() {
        let world = World()
        world.enableSleep = false
        let body = makeBody(world)

        let shapeDef = ShapeDef()
        let hullShape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        #expect(world.shapeHull(hullShape) != nil)
        #expect(world.shapeMesh(hullShape) == nil)
        #expect(world.shapeCompound(hullShape) == nil)
        #expect(world.shapeHeightField(hullShape) == nil)

        var definition = CompoundDef()
        var item = CompoundHullDef()
        item.hull = makeBoxHull(0.5, 0.5, 0.5)
        item.transform = .identity
        item.material = SurfaceMaterial.defaultMaterial
        definition.hulls.append(item)
        guard let compound = Compound(definition) else {
            Issue.record("compound creation failed")
            return
        }
        // Compound shapes must live on a static body.
        let staticBody = makeBody(world, .staticBody)
        let compoundShape = world.createCompoundShape(staticBody, shapeDef, compound)
        #expect(compoundShape.index1 != 0)
        #expect(world.shapeCompound(compoundShape) != nil)
        #expect(world.shapeHull(compoundShape) == nil)
    }

    // MARK: Body state accessors

    @Test("body min extent and enabled state")
    func bodyState() {
        let world = World()
        world.enableSleep = false
        let body = makeBody(world)

        var shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(1, 2, 3))

        // The smallest half dimension is the x extent.
        #expect(absFloat(world.bodyMinExtentValue(body) - 1) < 1e-4)
        #expect(world.isBodyEnabled(body))

        world.disableBody(body)
        #expect(world.isBodyEnabled(body) == false)
        world.enableBody(body)
        #expect(world.isBodyEnabled(body))
    }

    @Test("touching contact count tracks the resting pair")
    func touchingContacts() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, -0.5, 0)
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0.5, 0)
        let box = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        #expect(world.touchingContactCount() == 0)
        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.touchingContactCount() > 0)
    }

    // MARK: Joint world anchors

    @Test("a distance joint's world anchor follows body A")
    func jointWorldAnchor() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(2, 3, 4)
        let anchor = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(2, 2, 4)
        let bob = world.createBody(def)

        var jointDef = DistanceJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.length = 1
        let joint = world.createDistanceJoint(jointDef)

        let worldAnchor = world.jointWorldAnchorA(joint)!
        #expect(absFloat(worldAnchor.x - 2) < 1e-4)
        #expect(absFloat(worldAnchor.y - 3) < 1e-4)
        #expect(absFloat(worldAnchor.z - 4) < 1e-4)
    }

    // MARK: Wheel scalar limits

    @Test("wheel suspension and steering limits round trip")
    func wheelLimits() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let chassis = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, -1, 0)
        let wheel = world.createBody(def)

        var jointDef = WheelJointDef()
        jointDef.base.bodyIdA = chassis
        jointDef.base.bodyIdB = wheel
        let joint = world.createWheelJoint(jointDef)

        world.setWheelJointSuspensionLimits(joint, -0.2, 0.3)
        #expect(absFloat(world.wheelJointLowerSuspensionLimit(joint)! + 0.2) < 1e-6)
        #expect(absFloat(world.wheelJointUpperSuspensionLimit(joint)! - 0.3) < 1e-6)

        world.setWheelJointSteeringLimits(joint, -0.4, 0.5)
        #expect(absFloat(world.wheelJointLowerSteeringLimit(joint)! + 0.4) < 1e-6)
        #expect(absFloat(world.wheelJointUpperSteeringLimit(joint)! - 0.5) < 1e-6)

        world.enableWheelJointSuspension(joint, true)
        #expect(world.isWheelJointSuspensionEnabled(joint))
        world.enableWheelJointSuspensionLimit(joint, true)
        #expect(world.isWheelJointSuspensionLimitEnabled(joint))
        world.enableWheelJointSteeringLimit(joint, true)
        #expect(world.isWheelJointSteeringLimitEnabled(joint))

        // The reaction queries are defined even before stepping.
        #expect(world.wheelJointSpinTorqueValue(joint) != nil)
        #expect(world.wheelJointSteeringTorque(joint) != nil)
        #expect(world.wheelJointSteeringAngle(joint) != nil)
        #expect(world.wheelJointSpinSpeedValue(joint) != nil)
        #expect(world.wheelJointMotorTorque(joint) != nil)
    }

    // MARK: Spherical extras

    @Test("spherical spring, target rotation and twist limit round trip")
    func sphericalExtras() {
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

        world.setSphericalJointSpring(joint, hertz: 4, dampingRatio: 0.4)
        let spring = world.sphericalJointSpring(joint)!
        #expect(absFloat(spring.hertz - 4) < 1e-6)
        #expect(absFloat(spring.dampingRatio - 0.4) < 1e-6)

        let rotation = makeQuatFromAxisAngle(.axisY, 0.5)
        world.setSphericalJointTargetRotation(joint, rotation)
        #expect(world.sphericalJointTargetRotation(joint) != nil)

        world.enableSphericalJointTwistLimit(joint, true)
        #expect(world.isSphericalJointTwistLimitEnabled(joint))

        #expect(world.sphericalJointTwistAngle(joint) != nil)
        #expect(world.sphericalJointMotorTorque(joint) != nil)
    }

    @Test("prismatic and revolute reaction queries return values")
    func jointReactions() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let base = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 0, 0)
        let other = world.createBody(def)

        var prismDef = PrismaticJointDef()
        prismDef.base.bodyIdA = base
        prismDef.base.bodyIdB = other
        let prismatic = world.createPrismaticJoint(prismDef)
        world.enablePrismaticJointMotor(prismatic, true)
        #expect(world.prismaticJointMotorForce(prismatic) != nil)

        var revDef = RevoluteJointDef()
        revDef.base.bodyIdA = base
        revDef.base.bodyIdB = other
        let revolute = world.createRevoluteJoint(revDef)
        world.enableRevoluteJointMotor(revolute, true)
        #expect(world.revoluteJointMotorTorque(revolute) != nil)
    }

    @Test("parallel spring aliases agree with the base accessors")
    func parallelAliases() {
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

        world.setParallelJointSpringHertz(joint, 7)
        #expect(absFloat(world.parallelJointSpringHertz(joint)! - 7) < 1e-6)
        #expect(world.parallelJointHertz(joint) == world.parallelJointSpringHertz(joint))

        world.setParallelJointSpringDampingRatio(joint, 0.6)
        #expect(absFloat(world.parallelJointSpringDampingRatio(joint)! - 0.6) < 1e-6)
    }

    // MARK: Shape name

    @Test("shape names round trip")
    func shapeName() {
        let world = World()
        let body = makeBody(world)
        var shapeDef = ShapeDef()
        shapeDef.name = "hull"
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        #expect(world.shapeName(shape) == "hull")
        world.setShapeName(shape, "renamed")
        #expect(world.shapeName(shape) == "renamed")
    }
}
