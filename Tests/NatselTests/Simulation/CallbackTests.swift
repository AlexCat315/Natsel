import Testing
internal import Foundation
@testable import Natsel

private final class CallCounter: @unchecked Sendable {
    private let lock = NSLock()
    private var value = 0
    var count: Int {
        lock.lock(); defer { lock.unlock() }
        return value
    }
    func increment() {
        lock.lock(); defer { lock.unlock() }
        value += 1
    }
}

@Suite("Custom filter and pre-solve")
struct CallbackTests {
    private func makeScene(_ world: World, customFiltering: Bool = false) -> (BodyID, BodyID) {
        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(0, 1.0, 0)
        let box = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        shapeDef.enableCustomFiltering = customFiltering
        shapeDef.enablePreSolveEvents = true
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        return (ground, box)
    }

    @Test("custom filter can reject a pair")
    func customFilter() {
        let world = World()
        world.enableSleep = false
        let calls = CallCounter()
        world.setCustomFilter { _, _ in
            calls.increment()
            return false
        }
        let (_, box) = makeScene(world, customFiltering: true)

        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }

        #expect(calls.count > 0)
        // The filter removed all contacts, so the box falls through the ground.
        #expect(world.bodyTransform(box).p.y < -1)
        var contactCount = 0
        for c in world.contacts where c.contactId != B3.nullIndex { contactCount += 1 }
        #expect(contactCount == 0)
    }

    @Test("pre-solve can disable a contact")
    func preSolve() {
        let world = World()
        world.enableSleep = false
        let calls = CallCounter()
        world.setPreSolveCallback { _, _, _, _ in
            calls.increment()
            return false
        }
        let (_, box) = makeScene(world)

        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }

        #expect(calls.count > 0)
        #expect(world.bodyTransform(box).p.y < -1)
    }

    @Test("pre-solve returning true keeps the contact")
    func preSolveAllows() {
        let world = World()
        world.enableSleep = false
        let calls = CallCounter()
        world.setPreSolveCallback { _, _, _, _ in
            calls.increment()
            return true
        }
        let (_, box) = makeScene(world)

        for _ in 0..<120 { world.step(1.0 / 60.0, subStepCount: 4) }

        #expect(calls.count > 0)
        // The box rests on the ground.
        #expect(world.bodyTransform(box).p.y > 0.9)
    }
}
