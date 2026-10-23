import Testing
@testable import Natsel

private func fnv(_ h: UInt64, _ bytes: UnsafeRawPointer, _ n: Int) -> UInt64 {
    var h = h
    let p = bytes.assumingMemoryBound(to: UInt8.self)
    for i in 0..<n {
        h ^= UInt64(p[i])
        h = h &* 1_099_511_628_211
    }
    return h
}

@Suite("Island sleeping")
struct SleepTests {
    @Test("box falls asleep like the C reference")
    func boxSleeps() {
        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        let ground = world.createBody(def)
        var shapeDef = ShapeDef()
        _ = world.createHullShape(ground, shapeDef, makeBoxHull(10, 0.5, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Vec3(0, 0.6, 0)
        let box = world.createBody(def)
        shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        for i in 0..<180 {
            world.step(1.0 / 60.0, subStepCount: 4)
            if i == 59 {
                // After two seconds of resting the body should be in a sleeping set.
                #expect(world.body(box).setIndex >= SetType.firstSleepingSet.rawValue)
                #expect(world.bodyMoveEvents.isEmpty)
            }
        }

        let b = world.body(box)
        let sim = world.solverSets[Int(b.setIndex)].bodySims[Int(b.localIndex)]
        let v = world.bodyLinearVelocity(box)

        var h: UInt64 = 14_695_981_039_346_656_037
        var p = sim.transform.p
        var q = sim.transform.q
        var vv = v
        var flags = b.flags
        var sleepTime = b.sleepTime
        h = fnv(h, &p, 12)
        h = fnv(h, &q, 16)
        h = fnv(h, &vv, 12)
        h = fnv(h, &flags, 4)
        h = fnv(h, &sleepTime, 4)

        #expect(b.setIndex == 3)
        #expect(b.localIndex == 0)
        #expect(b.sleepTime.bitPattern == 0x3f00_0003)
        #expect(h == 0xe35c_04b9_0db7_7bb6)
    }
}
