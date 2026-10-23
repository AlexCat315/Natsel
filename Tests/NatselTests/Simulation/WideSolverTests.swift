import Testing
@testable import Natsel

// The wide (4-lane) solver must reproduce the scalar path exactly. C guarantees the same
// equivalence between its NEON path and `b3World_EnableSSE2Fallback`.
@Suite("Wide solver")
struct WideSolverTests {
    private func makeStack(_ world: World) {
        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, -1, 0)
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(10, 1, 10))

        for i in 0..<6 {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 0.6 + Float(i) * 1.05, 0)
            let box = world.createBody(def)
            var shapeDef = ShapeDef()
            shapeDef.density = 1000
            _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
        }
    }

    /// Lane-wise arithmetic must match the scalar expressions for a single contact.
    @Test("lane arithmetic matches scalar")
    func laneArithmetic() {
        let a = Vec3W(Vec3(1, 2, 3), Vec3(-1, 0.5, 2), Vec3(0, 0, 0), Vec3(4, 5, 6))
        let b = Vec3W(Vec3(0.5, 1, 1.5), Vec3(2, -1, 0.25), Vec3(1, 1, 1), Vec3(-4, 0, 1))

        let sum = addW(a, b)
        let diff = subW(a, b)
        let cross = crossW(a, b)
        let dot = dotW(a, b)

        for lane in 0..<4 {
            let sa = a[lane]
            let sb = b[lane]
            #expect(sum[lane] == sa + sb)
            #expect(diff[lane] == sa - sb)
            #expect(cross[lane] == Natsel.cross(sa, sb))
            #expect(dot[lane] == Natsel.dot(sa, sb))
        }
    }

    @Test("symmetric matrix product matches scalar")
    func matrixProduct() {
        let m = SymMatrix3(cxx: 2, cxy: 0.5, cxz: -1, cyy: 3, cyz: 0.25, czz: 4)
        let wide = SymMatrix3W(m, m, m, m)
        let v = Vec3W(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1, 1, 1))

        let out = mulMVW(wide, v)
        for lane in 0..<4 {
            #expect(out[lane] == m.mulMV(v[lane]))
        }
    }

    @Test("inverse rotation matches scalar")
    func inverseRotation() {
        let q = makeQuatFromAxisAngle(normalize(Vec3(1, 2, 3)), 0.7)
        let wideQuat = QuatW(
            v: Vec3W(splat: q.v), s: FloatW(repeating: q.s))
        let v = Vec3W(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1, 2, 3))

        let world = World()
        let out = world.invRotateVecW(wideQuat, v)
        for lane in 0..<4 {
            #expect(out[lane] == invRotateVectorWide(q, v[lane]))
        }
    }

    /// A stack run through the scalar path twice must agree, establishing the baseline the wide
    /// path has to match.
    @Test("stack settles deterministically")
    func stackDeterministic() {
        var hashes: [UInt64] = []
        for _ in 0..<2 {
            let world = World()
            world.enableSleep = false
            makeStack(world)
            for _ in 0..<90 { world.step(1.0 / 60.0, subStepCount: 4) }
            hashes.append(world.worldStateHash())
        }
        #expect(hashes[0] == hashes[1])
    }
}
