import Testing
@testable import Natsel

@Suite("World create and destroy")
struct WorldLifecycleTests {
    @Test("empty world steps without complaint")
    func emptyWorld() {
        let world = World()
        for _ in 0..<60 {
            world.step(1.0 / 60.0, subStepCount: 1)
        }
        #expect(world.bodyCount == 0)
        #expect(world.contactCount == 0)
    }

    @Test("creating and destroying bodies leaves nothing behind")
    func destroyAllBodies() {
        let world = World()
        let bodyCount = 10
        let cube = makeCubeHull(0.5)

        var bodyIds: [BodyID?] = Array(repeating: nil, count: bodyCount)
        var count = 0
        var creating = true

        // Create ten boxes, then tear them down one at a time, stepping throughout.
        for _ in 0..<(2 * bodyCount + 10) {
            if creating {
                if count < bodyCount {
                    var def = BodyDef()
                    def.type = .dynamicBody
                    let body = world.createBody(def)

                    var shapeDef = ShapeDef()
                    _ = world.createHullShape(body, shapeDef, cube)
                    bodyIds[count] = body
                    count += 1
                } else {
                    creating = false
                }
            } else if count > 0 {
                world.destroyBody(bodyIds[count - 1]!)
                bodyIds[count - 1] = nil
                count -= 1
            }

            world.step(1.0 / 60.0, subStepCount: 3)
        }

        #expect(world.bodyCount == 0)
        #expect(world.contactCount == 0)
        #expect(world.awakeBodyCount() == 0)

        for id in bodyIds.compactMap({ $0 }) {
            #expect(world.bodyIsValid(id) == false)
        }
    }

    @Test("destroyed body ids are rejected")
    func destroyedBodyIsInvalid() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeCubeHull(0.5))

        #expect(world.bodyIsValid(body))
        world.step(1.0 / 60.0, subStepCount: 1)

        world.destroyBody(body)
        #expect(world.bodyIsValid(body) == false)
        #expect(world.bodyCount == 0)
    }

    @Test("id pool recycles slots in last-in-first-out order")
    func idRecycling() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        let first = world.createBody(def)
        let second = world.createBody(def)

        world.destroyBody(second)
        world.destroyBody(first)

        // The free list is a stack: the slot freed last is handed out first.
        let recycled = world.createBody(def)
        #expect(recycled.index1 == first.index1)

        let recycled2 = world.createBody(def)
        #expect(recycled2.index1 == second.index1)
    }

    @Test("bodies accumulate contacts then release them when destroyed")
    func contactsReleasedOnDestroy() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        var boxes: [BodyID] = []
        for i in 0..<3 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 0.6 + Float(i) * 1.05, 0)
            let box = world.createBody(def)
            shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
            boxes.append(box)
        }

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        let contactCount = world.contactCount
        #expect(contactCount > 0)

        // Removing every box must also remove every contact that referenced them.
        for box in boxes {
            world.destroyBody(box)
        }

        #expect(world.bodyCount == 1)
        #expect(world.contactCount == 0)
    }
}
