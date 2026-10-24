import Testing

@testable import Natsel

@Suite("Wide prepare")
struct WidePrepareTests {
    @Test("direct SoA prepare matches the scalar prepare lane by lane")
    func lanesMatchScalar() throws {
        let w = World()
        var ground = BodyDef()
        let groundId = w.createBody(ground)
        _ = w.createHullShape(groundId, ShapeDef(), makeBoxHull(50, 1, 50))
        for i in 0..<12 {
            for j in 0..<4 {
                var def = BodyDef()
                def.type = .dynamicBody
                def.position = Vec3(Float(j) * 1.1 - 1.5, 1.5 + Float(i), Float(i % 3) * 0.3)
                let body = w.createBody(def)
                _ = w.createHullShape(body, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))
            }
        }
        ground.type = .staticBody
        w.prepareScalarForWideColors = true
        var checked = 0
        for _ in 0..<40 {
            w.step(1 / 60, subStepCount: 4)
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
            for ci in 0..<Int(overflowIndex) {
                let list = w.solverContacts.constraints[ci]
                for c in w.solverContacts.wideConstraints[ci] {
                    for lane in 0..<4 {
                        let id = c.contactIds[lane]
                        guard id >= 0, let s = list.first(where: { $0.contactId == id }) else { continue }
                        func eq(_ a: Float, _ b: Float, _ what: String) throws {
                            try #require(a.bitPattern == b.bitPattern, "\(what) contact \(id): \(a) vs \(b)")
                        }
                        try eq(c.invMassA[lane], s.invMassA, "invMassA")
                        try eq(c.invIA.xy[lane], s.invIA.cxy, "invIA.xy")
                        try eq(c.rollingMass.xx[lane], s.rollingMass.cxx, "rollingMass")
                        for offset in 0..<Int(s.manifoldCount) {
                            let m = w.solverContacts.wideManifolds[c.manifoldStart + offset]
                            let sm = w.solverContacts.manifoldPool[Int(s.manifoldStart) + offset]
                            try eq(m.tangent1.x[lane], sm.tangent1.x, "t1x")
                            try eq(m.tangent1.y[lane], sm.tangent1.y, "t1y")
                            try eq(m.tangent1.z[lane], sm.tangent1.z, "t1z")
                            try eq(m.tangent2.x[lane], sm.tangent2.x, "t2x")
                            try eq(m.tangentVelocity1[lane], sm.tangentVelocity1, "tv1")
                            try eq(m.twistImpulse[lane], sm.twistImpulse, "twist")
                            try eq(m.frictionImpulse.x[lane], sm.frictionImpulse.x, "fx")
                            try eq(m.frictionImpulse.y[lane], sm.frictionImpulse.y, "fy")
                            try eq(m.centerA.x[lane], sm.centerA.x, "cAx")
                            try eq(m.centerB.y[lane], sm.centerB.y, "cBy")
                            try eq(m.twistMass[lane], sm.twistMass, "twistMass")
                            try eq(m.tangentMass.xx[lane], sm.tangentMass.cx.x, "tmxx")
                            try eq(m.tangentMass.xy[lane], sm.tangentMass.cx.y, "tmxy")
                            try eq(m.tangentMass.yy[lane], sm.tangentMass.cy.y, "tmyy")
                            for pi in 0..<Int(sm.pointCount) {
                                let p = m.points[pi], sp = sm.points[pi]
                                try eq(p.rA.x[lane], sp.rA.x, "rAx \(pi)")
                                try eq(p.baseSeparation[lane], sp.baseSeparation, "baseSep \(pi)")
                                try eq(p.normalImpulse[lane], sp.normalImpulse, "normalImpulse \(pi)")
                                try eq(p.normalMass[lane], sp.normalMass, "normalMass \(pi)")
                                try eq(p.leverArm[lane], sp.leverArm, "leverArm \(pi)")
                                try eq(p.relativeVelocity[lane], sp.relativeVelocity, "relVel \(pi)")
                                checked += 1
                            }
                        }
                    }
                }
            }
        }
        try #require(checked > 0)
    }
}

extension WidePrepareTests {
    @Test("skipping scalar twins does not change the simulation")
    func scalarTwinsAreInert() throws {
        func make(_ twins: Bool) -> World {
            let w = World()
            let groundId = w.createBody(BodyDef())
            _ = w.createHullShape(groundId, ShapeDef(), makeBoxHull(50, 1, 50))
            for i in 0..<12 {
                for j in 0..<4 {
                    var def = BodyDef()
                    def.type = .dynamicBody
                    def.position = Vec3(Float(j) * 1.1 - 1.5, 1.5 + Float(i), Float(i % 3) * 0.3)
                    let body = w.createBody(def)
                    _ = w.createHullShape(body, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))
                }
            }
            w.prepareScalarForWideColors = twins
            return w
        }
        let a = make(false), b = make(true)
        for step in 0..<120 {
            a.step(1 / 60, subStepCount: 4)
            b.step(1 / 60, subStepCount: 4)
            try #require(a.worldStateHash() == b.worldStateHash(), "step \(step)")
        }
    }
}
