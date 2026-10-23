import Testing
@testable import Natsel

// The wide solver must reproduce the scalar path bit for bit, the same way C's NEON path matches
// its scalar fallback.
@Suite("Wide equivalence")
struct WideEquivalenceTests {
    private func scene(_ wide: Bool) -> UInt64 {
        let world = World()
        world.enableSleep = false
        world.enableWideSolver = wide

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, -1, 0)
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(10, 1, 10))

        // A small grid so several contacts share a color.
        for x in 0..<3 {
            for z in 0..<3 {
                def = BodyDef()
                def.type = .dynamicBody
                def.position = Pos(Float(x) * 1.02 - 1, 0.6 + Float(z) * 0.1, Float(z) * 1.02 - 1)
                let box = world.createBody(def)
                var shapeDef = ShapeDef()
                shapeDef.density = 1000
                _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
            }
        }

        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }
        return world.worldStateHash()
    }

    @Test("wide and scalar paths agree")
    func wideMatchesScalar() {
        let scalar = scene(false)
        let wide = scene(true)
        #expect(scalar == wide, "scalar \(String(scalar, radix: 16)) wide \(String(wide, radix: 16))")
    }
}
