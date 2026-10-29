import Testing
@testable import Natsel

@Suite("Contact events")
struct ContactEventTests {
    @Test("bouncing sphere produces begin and end events with valid ids")
    func beginAndEndEvents() {
        let world = World()

        // Static ground
        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, Float(-0.5), 0)
        let ground = world.createBody(def)
        var groundShapeDef = ShapeDef()
        let groundShape = world.createHullShape(ground, groundShapeDef, makeBoxHull(10, 0.5, 10))

        // Dynamic sphere dropped onto the ground; restitution causes it to bounce so we get
        // end events as well.
        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 5, 0)
        let sphereBody = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.enableContactEvents = true
        shapeDef.baseMaterial.restitution = 0.6
        let sphereShape = world.createSphereShape(
            sphereBody, shapeDef, Sphere(center: Vec3(), radius: 0.5)
        )

        var beginCount = 0
        var endCount = 0
        var idsChecked = false

        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)

            let events = world.contactEvents()

            if events.beginEvents.isEmpty == false && idsChecked == false {
                let begin = events.beginEvents[0]
                let aIsSphere = begin.shapeIdA.index1 == sphereShape.index1
                    && begin.shapeIdA.generation == sphereShape.generation
                let bIsSphere = begin.shapeIdB.index1 == sphereShape.index1
                    && begin.shapeIdB.generation == sphereShape.generation
                let aIsGround = begin.shapeIdA.index1 == groundShape.index1
                let bIsGround = begin.shapeIdB.index1 == groundShape.index1

                #expect((aIsSphere && bIsGround) || (aIsGround && bIsSphere))
                idsChecked = true
            }

            beginCount += events.beginEvents.count
            endCount += events.endEvents.count
        }

        #expect(idsChecked)
        #expect(beginCount >= 1)
        #expect(endCount >= 1)
    }

    @Test("shapes without contact events enabled report nothing")
    func eventsDisabled() {
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
        let body = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        shapeDef.enableContactEvents = false
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        var total = 0
        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
            let events = world.contactEvents()
            total += events.beginEvents.count + events.endEvents.count
        }
        #expect(total == 0)
    }
}

// MARK: - Hit events

@Suite("Contact hit events")
struct ContactHitEventTests {
    @Test("fast impact produces a hit event with speed, normal and material")
    func hitEvent() {
        let world = World()
        world.hitEventThreshold = 1.0

        // Static ground
        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, Float(-0.5), 0)
        let ground = world.createBody(def)
        var groundShapeDef = ShapeDef()
        _ = world.createHullShape(ground, groundShapeDef, makeBoxHull(10, 0.5, 10))

        // Sphere driven into the ground fast enough to clear the hit threshold.
        def = BodyDef()
        def.type = .dynamicBody
        def.gravityScale = 0
        def.position = Pos(0, 2, 0)
        def.linearVelocity = Vec3(0, -30, 0)
        let sphereBody = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        shapeDef.enableHitEvents = true
        shapeDef.baseMaterial.userMaterialId = 7
        _ = world.createSphereShape(sphereBody, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        var hitCount = 0
        var capturedSpeed: Float = 0
        var capturedNormal = Vec3()
        var capturedMaterialA: UInt64 = 0
        var capturedMaterialB: UInt64 = 0

        for _ in 0..<30 {
            world.step(1.0 / 60.0, subStepCount: 4)

            let events = world.contactEvents()
            if events.hitEvents.isEmpty == false && hitCount == 0 {
                let hit = events.hitEvents[0]
                capturedSpeed = hit.approachSpeed
                capturedNormal = hit.normal
                capturedMaterialA = hit.userMaterialIdA
                capturedMaterialB = hit.userMaterialIdB
            }
            hitCount += events.hitEvents.count
        }

        #expect(hitCount >= 1)
        #expect(capturedSpeed > 1)
        // Head-on vertical impact: the normal lies along Y.
        #expect(absFloat(capturedNormal.x) < 0.01)
        #expect(absFloat(capturedNormal.z) < 0.01)
        // One side of the contact carries the sphere's user material.
        #expect(capturedMaterialA == 7 || capturedMaterialB == 7)
    }

    @Test("slow contacts do not report a hit")
    func noHitBelowThreshold() {
        let world = World()
        world.enableSleep = false
        world.hitEventThreshold = 100

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 2, 0)
        let body = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        shapeDef.enableHitEvents = true
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        var total = 0
        for _ in 0..<120 {
            world.step(1.0 / 60.0, subStepCount: 4)
            total += world.contactEvents().hitEvents.count
        }
        #expect(total == 0)
    }
}
