import Testing
@testable import Natsel

// The move event written during finalization must match the pose the body reports, including any
// CCD rewind. Ported from test/test_world.c TestContinuousMoveEvent.
@Suite("Continuous move event")
struct ContinuousMoveEventTests {
    @Test("move events agree with the body pose across a CCD impact")
    func moveEventMatchesPose() {
        let world = World()

        // Thin static wall, near face at x = 0.1
        var def = BodyDef()
        def.type = .staticBody
        let wall = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(wall, shapeDef, makeBoxHull(0.1, 5, 5))

        // Fast dynamic sphere fired at the wall.
        def = BodyDef()
        def.type = .dynamicBody
        def.gravityScale = 0
        def.position = Pos(3, 0, 0)
        def.linearVelocity = Vec3(-30, 0, 0)
        let ball = world.createBody(def)

        shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = world.createSphereShape(ball, shapeDef, Sphere(center: Vec3(), radius: 0.25))

        let timeStep: Float = 1.0 / 60.0
        var haveMove = false

        for _ in 0..<30 {
            world.step(timeStep, subStepCount: 4)

            let transform = world.bodyTransform(ball)

            for event in world.bodyEvents() {
                if event.bodyId != ball {
                    continue
                }

                haveMove = true

                // The move event must carry the same pose the body reports, CCD rewind included.
                #expect(event.transform.p == transform.p)
                #expect(event.transform.q == transform.q)
            }
        }

        #expect(haveMove)

        // Tunnel check
        let finalPos = world.bodyTransform(ball).p
        #expect(finalPos.x > 0.2 && finalPos.x < 0.8)
    }

    @Test("move events report the body ids and transforms of moving bodies")
    func moveEventsAreConsistent() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 3, 0)
        let box = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        var sawBox = false
        for _ in 0..<30 {
            world.step(1.0 / 60.0, subStepCount: 4)
            let transform = world.bodyTransform(box)
            for event in world.bodyEvents() where event.bodyId == box {
                sawBox = true
                #expect(event.transform.p == transform.p)
                #expect(event.transform.q == transform.q)
                #expect(event.fellAsleep == false)
            }
        }
        #expect(sawBox)
    }
}
