import Testing

@testable import Natsel

@Suite("Wide solver stages")
struct WideStageTests {
    @Test("mesh stages preserve every body velocity and impulse")
    func stages() throws {
        let (w, _) = ImpulseDeterminismTests().makeMeshMix(false)
        w.prepareScalarForWideColors = true
        for _ in 0..<69 { w.step(1 / 60, subStepCount: 4) }
        w.enableWideSolver = true
        w.updateBroadPhasePairs()
        var ctx = World.StepContext()
        ctx.dt = 1 / 60
        ctx.h = ctx.dt / 4
        ctx.inv_dt = 60
        ctx.inv_h = 240
        ctx.subStepCount = 4
        ctx.enableWarmStarting = true
        ctx.maxLinearVelocity = w.maxLinearSpeed
        let hertz = minFloat(w.contactHertz, 0.125 * ctx.inv_h)
        ctx.contactSoftness = makeSoft(hertz, w.contactDampingRatio, ctx.h)
        ctx.staticSoftness = makeSoft(2 * hertz, 0.5 * w.contactDampingRatio, ctx.h)
        w.collide(&ctx)
        w.prepareAllContacts(ctx)
        func compare(_ phase: String, _ action: () -> Void) throws {
            let states = w.solverSets[Int(SetType.awakeSet.rawValue)].bodyStates
            let pool = w.solverContacts.manifoldPool
            w.enableWideSolver = false
            action()
            let expected = w.solverSets[Int(SetType.awakeSet.rawValue)].bodyStates
            let expectedPool = w.solverContacts.manifoldPool
            w.solverSets[Int(SetType.awakeSet.rawValue)].bodyStates = states
            w.solverContacts.manifoldPool = pool
            w.enableWideSolver = true
            action()
            w.unpackWideImpulses()
            let actual = w.solverSets[Int(SetType.awakeSet.rawValue)].bodyStates
            for i in expected.indices {
                for (a, b) in [
                    (expected[i].linearVelocity, actual[i].linearVelocity),
                    (expected[i].angularVelocity, actual[i].angularVelocity),
                ] {
                    try #require(
                        a.x.bitPattern == b.x.bitPattern && a.y.bitPattern == b.y.bitPattern
                            && a.z.bitPattern == b.z.bitPattern, "\(phase) body \(i) \(a) != \(b)")
                }
            }
            for i in expectedPool.indices {
                let a = expectedPool[i]
                let b = w.solverContacts.manifoldPool[i]
                try #require(
                    a.frictionImpulse.x.bitPattern == b.frictionImpulse.x.bitPattern
                        && a.frictionImpulse.y.bitPattern == b.frictionImpulse.y.bitPattern
                        && a.twistImpulse.bitPattern == b.twistImpulse.bitPattern
                        && a.rollingImpulse == b.rollingImpulse,
                    "\(phase) manifold \(i) friction \(a.frictionImpulse) != \(b.frictionImpulse), rolling \(a.rollingImpulse) != \(b.rollingImpulse)"
                )
                for pi in 0..<Int(a.pointCount) {
                    try #require(
                        a.points[pi].normalImpulse == b.points[pi].normalImpulse,
                        "\(phase) manifold \(i) point \(pi) normal")
                }
            }
        }
        for substep in 0..<4 {
            w.integrateVelocities(ctx.h)
            for ci in [overflowIndex] + Array(0..<overflowIndex) {
                try compare("warm \(substep) color \(ci)") { w.warmStartJoints(ci, ctx) }
                for mesh in [false, true] {
                    try compare("warm \(substep) color \(ci) mesh \(mesh)") { w.warmStartContacts(ci, mesh) }
                }
            }
            try compare("push \(substep)") { w.pushAll(ctx) }
            w.integratePositions(ctx.h, ctx.inv_dt)
            try compare("solve \(substep)") { w.solveAll(ctx) }
        }
    }
}
