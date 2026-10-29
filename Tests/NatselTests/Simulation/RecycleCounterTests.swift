import Testing
internal import Foundation
@testable import Natsel

private final class RejectFlag: @unchecked Sendable {
    private let lock = NSLock()
    private var value = true
    var reject: Bool {
        get { lock.lock(); defer { lock.unlock() }; return value }
        set { lock.lock(); defer { lock.unlock() }; value = newValue }
    }
}

@Suite("Recycle counters")
struct RecycleCounterTests {
    private func countRecycledResting(enableSpeculative: Bool) -> Int32 {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, Float(-0.5), 0)
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, Float(0.5), 0)
        let box = world.createBody(def)
        shapeDef.enableSpeculativeContact = enableSpeculative
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        var recycledCount: Int32 = 0
        for _ in 0..<60 {
            world.step(1.0 / 60.0, subStepCount: 4)
            recycledCount += world.counters().recycledContactCount
        }
        return recycledCount
    }

    @Test("speculative points enable recycling; disabling them stops it")
    func speculativeRecycle() {
        #expect(countRecycledResting(enableSpeculative: true) > 0)
        #expect(countRecycledResting(enableSpeculative: true) == 59)
        #expect(countRecycledResting(enableSpeculative: false) == 0)
    }

    /// A rejected pre-solve frees the manifold while the shapes may already touch. The next step
    /// must rebuild the contact and ask again instead of recycling the empty pair.
    @Test("pre-solve rejection does not let the body sink through")
    func preSolveRejectRecycle() {
        let world = World()
        world.enableSleep = false

        let flag = RejectFlag()
        world.setPreSolveCallback { _, _, _, _ in !flag.reject }

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, Float(-0.5), 0)
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        let radius: Float = 0.25
        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, radius, 0)
        let body = world.createBody(def)

        shapeDef.enablePreSolveEvents = true
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: radius))

        // Reject while the shapes first meet.
        world.step(1.0 / 60.0, subStepCount: 4)
        flag.reject = false

        var minSeparation = Float.greatestFiniteMagnitude
        for _ in 0..<30 {
            world.step(1.0 / 60.0, subStepCount: 4)
            let p = world.bodyTransform(body).p
            minSeparation = minFloat(minSeparation, Float(p.y) - radius)
        }

        // The sphere must stay out of the ground within the linear slop.
        #expect(minSeparation > -B3.linearSlop)
        #expect(absFloat(minSeparation - (-0.00105443597)) < 1e-6)
    }
}
