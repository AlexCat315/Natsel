import Testing
@testable import Natsel

// Joint events and shape-level contact data.
@Suite("Joint events")
struct JointEventTests {
    @Test("a joint under load reports an event when its threshold is low")
    func jointEventFires() {
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

        // With a zero force threshold any awake loaded joint is reported.
        world.setJointForceThreshold(joint, 0)

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        let events = world.jointEventList()
        #expect(events.isEmpty == false)
        #expect(events.contains { $0.jointId.index1 == joint.index1 })
    }

    @Test("a high threshold suppresses the event")
    func highThresholdSuppresses() {
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

        world.setJointForceThreshold(joint, 1.0e9)
        world.setJointTorqueThreshold(joint, 1.0e9)

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.jointEventList().isEmpty)
    }

    @Test("joint events are cleared each step")
    func eventsClearEachStep() {
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
        world.setJointForceThreshold(joint, 0)

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }
        #expect(world.jointEventList().isEmpty == false)

        // Destroying the joint removes it from the next step's events.
        world.destroyJoint(joint.index1 - 1, false)
        world.step(1.0 / 60.0, subStepCount: 4)
        #expect(world.jointEventList().isEmpty)
    }

    @Test("shape contact data reports only its own touching contacts")
    func shapeContactData() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, -0.5, 0)
        let ground = world.createBody(def)
        let groundShapeDef = ShapeDef()
        let groundShape = world.createHullShape(
            ground, groundShapeDef, makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0.5, 0)
        let box = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        let boxShape = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        let contacts = world.shapeContactData(boxShape)
        #expect(contacts.count == 1)
        guard let first = contacts.first else { return }

        // The contact lists both shapes.
        let involvesBox = first.shapeIdA.index1 == boxShape.index1
            || first.shapeIdB.index1 == boxShape.index1
        let involvesGround = first.shapeIdA.index1 == groundShape.index1
            || first.shapeIdB.index1 == groundShape.index1
        #expect(involvesBox)
        #expect(involvesGround)

        // The same contact is reported from the ground's side.
        #expect(world.shapeContactData(groundShape).count == 1)
        #expect(world.contactIsValid(first.contactId))

        // A detached shape reports nothing.
        world.destroyShape(boxShape, updateBodyMass: false)
        #expect(world.shapeContactData(groundShape).isEmpty)
    }

    @Test("a sensor shape reports no contact data")
    func sensorHasNoContacts() {
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
        let probe = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.isSensor = true
        shapeDef.enableSensorEvents = true
        let sensor = world.createHullShape(probe, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        // Sensors never surface through contact data.
        #expect(world.shapeContactData(sensor).isEmpty)
    }
}
