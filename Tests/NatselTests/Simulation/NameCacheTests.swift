import Testing
@testable import Natsel

// Name interning ported from test/test_name_cache.c.
@Suite("Name cache")
struct NameCacheTests {
    @Test("empty and nil names map to the null id")
    func cacheUnit() {
        var cache = NameCache()

        // Empty and nil map to the null id, which resolves to no string.
        #expect(cache.add("") == nullName)
        #expect(cache.add(nil) == nullName)
        #expect(cache.find(nullName) == nil)

        // Distinct strings get distinct ids and round-trip.
        let crate = cache.add("crate")
        let barrel = cache.add("barrel")
        #expect(crate != nullName)
        #expect(barrel != nullName)
        #expect(crate != barrel)
        #expect(cache.find(crate) == "crate")
        #expect(cache.find(barrel) == "barrel")

        // Re-interning the same bytes returns the same id.
        #expect(cache.add("crate") == crate)

        // A name far longer than any inline buffer round-trips exactly.
        let longName = String(repeating: "abcdefghijklmnopqrstuvwxyz", count: 150)
        let longId = cache.add(longName)
        #expect(cache.find(longId) == longName)

        // A missing id resolves to nil, and to the caller's default.
        #expect(cache.find(12345) == nil)
        #expect(cache.find(12345, default: "fallback") == "fallback")
    }

    @Test("bodies and shapes carry interned names through the world")
    func worldNames() {
        let bodyNames = [
            "crate",
            "barrel",
            "crate",
            "a_very_long_body_name_that_exceeds_any_inline_name_buffer",
        ]
        let shapeNames = [
            "box_hull",
            "box_hull",
            "crate",
            "a_very_long_shape_name_that_never_fit_the_old_inline_buffer",
        ]

        let world = World()
        world.gravity = Vec3(0, -10, 0)

        // Ground is ordinal 0, boxes 1..N.
        var groundDef = BodyDef()
        groundDef.type = .staticBody
        let ground = world.createBody(groundDef)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(20, 1, 20))

        var bodies: [BodyID] = []
        var shapes: [ShapeID] = []
        for i in 0..<bodyNames.count {
            var def = BodyDef()
            def.type = .dynamicBody
            def.position = Pos(0, 1 + 1.1 * Float(i), 0)
            def.name = bodyNames[i]
            let body = world.createBody(def)

            var shapeDef = ShapeDef()
            shapeDef.density = 1
            shapeDef.name = shapeNames[i]
            let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

            bodies.append(body)
            shapes.append(shape)
        }

        for i in 0..<bodyNames.count {
            #expect(world.bodyName(bodies[i]) == bodyNames[i], "body \(i)")
            #expect(world.shapeName(shapes[i]) == shapeNames[i], "shape \(i)")
        }

        // Shape names intern independently: a shape may share a body's name.
        #expect(world.bodyName(bodies[0]) == "crate")
        #expect(world.shapeName(shapes[2]) == "crate")

        // Setting a name replaces it; clearing removes it.
        world.setBodyName(bodies[0], "renamed")
        #expect(world.bodyName(bodies[0]) == "renamed")

        world.setBodyName(bodies[0], nil)
        #expect(world.bodyName(bodies[0]) == nil)

        // The unnamed ground stays unnamed.
        #expect(world.bodyName(ground) == nil)
    }

    @Test("dynamic body and shape names survive stepping")
    func namesSurviveStepping() {
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
        def.name = "falling_box"
        let box = world.createBody(def)

        shapeDef = ShapeDef()
        shapeDef.density = 1000
        shapeDef.name = "box_shape"
        let shape = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        for _ in 0..<60 { world.step(1.0 / 60.0, subStepCount: 4) }

        #expect(world.bodyName(box) == "falling_box")
        #expect(world.shapeName(shape) == "box_shape")
    }
}
