import Testing
@testable import Natsel

// Debug draw walks the world and reports what the caller asked to see.
@Suite("Debug draw")
struct DebugDrawTests {
    private func makeScene() -> (World, BodyID) {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, -1, 0)
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(10, 1, 10))

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, Float(0.6), 0)
        let box = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }
        return (world, box)
    }

    @Test("shapes are reported with a factory-produced handle")
    func shapes() {
        let (world, _) = makeScene()

        var created = 0
        var drawn = 0
        var boundsDrawn = 0
        var lastTransform = WorldTransform()

        var options = DebugDraw()
        options.createDebugShape = { _ in
            created += 1
            // Any non-nil token stands in for a renderer handle.
            return UnsafeMutableRawPointer(bitPattern: 0x1000 + created)
        }
        options.drawShape = { _, transform, _ in
            drawn += 1
            lastTransform = transform
        }
        options.drawBounds = { _, _ in boundsDrawn += 1 }

        world.draw(options)

        // Two shapes: the ground and the box.
        #expect(created == 2)
        #expect(drawn == 2)

        // The handle is cached on the shape, so a second pass reuses it.
        created = 0
        drawn = 0
        world.draw(options)
        #expect(created == 0)
        #expect(drawn == 2)

        // Bounds are only emitted when the option is on.
        options.drawBoundsOption = true
        boundsDrawn = 0
        world.draw(options)
        #expect(boundsDrawn == 2)
    }

    @Test("culling bounds limit what is reported")
    func culling() {
        let (world, _) = makeScene()

        var drawn = 0
        var options = DebugDraw()
        options.createDebugShape = { _ in UnsafeMutableRawPointer(bitPattern: 0x1000) }
        options.drawShape = { _, _, _ in drawn += 1 }

        // The ground spans ±10, so a tight box around the upper body still only touches the box
        // if the ground's fat AABB reaches it. Use bounds well above the ground.
        options.drawingBounds = AABB(Vec3(-1, 0.2, -1), Vec3(1, 2, 1))
        world.draw(options)
        #expect(drawn == 1)

        // Far away bounds see nothing.
        drawn = 0
        options.drawingBounds = AABB(Vec3(100, 100, 100), Vec3(110, 110, 110))
        world.draw(options)
        #expect(drawn == 0)
    }

    @Test("contact normals are emitted when requested")
    func contacts() {
        let (world, _) = makeScene()

        var segments = 0
        var strings: [String] = []

        var options = DebugDraw()
        options.drawShapes = false
        options.drawContacts = true
        options.drawContactNormals = true
        options.drawSegment = { _, _, _ in segments += 1 }
        options.drawString = { _, s, _ in strings.append(s) }

        world.draw(options)

        #expect(segments > 0)
        // Normal labels carry the separation in cm.
        #expect(strings.contains { $0.contains(".") })
    }

    @Test("graph colors emit one point per contact point")
    func graphColors() {
        let (world, _) = makeScene()

        var points = 0
        var options = DebugDraw()
        options.drawShapes = false
        options.drawContacts = true
        options.drawGraphColors = true
        options.drawPoint = { _, _, _ in points += 1 }

        world.draw(options)
        #expect(points > 0)
    }

    @Test("body names and mass annotations are reported")
    func annotations() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        def.name = "test_body"
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        var strings: [String] = []
        var transforms = 0

        var options = DebugDraw()
        options.drawShapes = false
        options.drawBodyNames = true
        options.drawMass = true
        options.drawString = { _, s, _ in strings.append(s) }
        options.drawTransform = { _ in transforms += 1 }

        world.draw(options)

        #expect(strings.contains("test_body"))
        #expect(strings.contains { $0.contains("1000") })
        #expect(transforms == 1)
    }

    @Test("joints connect their anchors with segments")
    func joints() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 5, 0)
        let anchor = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 5, 0)
        let bob = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bob, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        var jointDef = DistanceJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.length = 1
        _ = world.createDistanceJoint(jointDef)

        var segments = 0
        var points = 0
        var options = DebugDraw()
        options.drawShapes = false
        options.drawJoints = true
        options.drawSegment = { _, _, _ in segments += 1 }
        options.drawPoint = { _, _, _ in points += 1 }

        world.draw(options)
        // A distance joint draws its own axis plus anchor markers.
        #expect(segments == 1)
        #expect(points == 2)
    }

    @Test("material presets pack into the high byte")
    func colorPacking() {
        let rgb: HexColor = 0xFF8000
        let plain = makeDebugColor(rgb, .default)
        #expect(plain == rgb)

        let glossy = makeDebugColor(rgb, .glossy)
        #expect(glossy & 0x00FF_FFFF == rgb)
        #expect(glossy >> 24 == DebugMaterial.glossy.rawValue)

        let metallic = makeDebugColor(rgb, .metallic)
        #expect(metallic >> 24 == 5)
    }
}

// MARK: - Per-type joint drawing

extension DebugDrawTests {
    private struct Counts {
        var segments = 0
        var points = 0
        var boxes = 0
    }

    private func drawCounts(_ world: World) -> Counts {
        var counts = Counts()
        var options = DebugDraw()
        options.drawShapes = false
        options.drawJoints = true
        options.drawSegment = { _, _, _ in counts.segments += 1 }
        options.drawPoint = { _, _, _ in counts.points += 1 }
        options.drawBox = { _, _, _ in counts.boxes += 1 }
        world.draw(options)
        return counts
    }

    private func makeJointWorld() -> (World, BodyID, BodyID) {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(0, 5, 0)
        let anchor = world.createBody(def)

        def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 5, 0)
        let bob = world.createBody(def)
        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        _ = world.createHullShape(bob, shapeDef, makeBoxHull(0.2, 0.2, 0.2))

        return (world, anchor, bob)
    }

    @Test("a distance joint draws its limit markers and spring rest point")
    func distanceJointDetail() {
        let (world, anchor, bob) = makeJointWorld()

        var jointDef = DistanceJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.length = 1
        jointDef.enableLimit = true
        jointDef.minLength = 0.5
        jointDef.maxLength = 2
        jointDef.enableSpring = true
        jointDef.hertz = 4
        _ = world.createDistanceJoint(jointDef)

        let counts = drawCounts(world)
        // Limit points (2) + anchor points (2) + spring rest point (1).
        #expect(counts.points == 5)
        // The limit segment plus the main axis.
        #expect(counts.segments == 2)
    }

    @Test("a revolute joint draws frame axes and the limit wedge")
    func revoluteJointDetail() {
        let (world, anchor, bob) = makeJointWorld()
        let (plainWorld, plainAnchor, plainBob) = makeJointWorld()

        var jointDef = RevoluteJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.enableLimit = true
        jointDef.lowerAngle = -0.5
        jointDef.upperAngle = 0.5
        _ = world.createRevoluteJoint(jointDef)

        // Same joint without a limit for comparison.
        var plainDef = RevoluteJointDef()
        plainDef.base.bodyIdA = plainAnchor
        plainDef.base.bodyIdB = plainBob
        _ = plainWorld.createRevoluteJoint(plainDef)

        let limited = drawCounts(world)
        let plain = drawCounts(plainWorld)

        // Three frame axes either way.
        #expect(plain.segments == 3)
        // The wedge adds its 16 slices plus the two closing spokes plus the twist marker.
        #expect(limited.segments > plain.segments + 15)
    }

    @Test("a prismatic joint draws its axis and limits")
    func prismaticJointDetail() {
        let (world, anchor, bob) = makeJointWorld()

        var jointDef = PrismaticJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.enableLimit = true
        jointDef.lowerTranslation = -0.5
        jointDef.upperTranslation = 0.5
        _ = world.createPrismaticJoint(jointDef)

        let counts = drawCounts(world)
        // Two perpendicular axes plus the limit axis.
        #expect(counts.segments == 3)
        // Two limit points plus the frame B marker.
        #expect(counts.points == 3)
    }

    @Test("a spherical joint draws its cone and twist wedges")
    func sphericalJointDetail() {
        let (world, anchor, bob) = makeJointWorld()

        var jointDef = SphericalJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.enableConeLimit = true
        jointDef.coneAngle = 0.4
        jointDef.enableTwistLimit = true
        jointDef.lowerTwistAngle = -0.3
        jointDef.upperTwistAngle = 0.3
        _ = world.createSphericalJoint(jointDef)

        let counts = drawCounts(world)
        // Three frame axes, the cone axis, the twist wedge and the cone fan all contribute.
        #expect(counts.segments > 40)
    }

    @Test("a weld joint draws two boxes")
    func weldJointDetail() {
        let (world, anchor, bob) = makeJointWorld()

        var jointDef = WeldJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        _ = world.createWeldJoint(jointDef)

        let counts = drawCounts(world)
        #expect(counts.boxes == 2)
        #expect(counts.segments == 0)
    }

    @Test("a parallel joint draws one axis per frame")
    func parallelJointDetail() {
        let (world, anchor, bob) = makeJointWorld()

        var jointDef = ParallelJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        _ = world.createParallelJoint(jointDef)

        let counts = drawCounts(world)
        #expect(counts.segments == 2)
    }

    @Test("a wheel joint draws its suspension and spin axle")
    func wheelJointDetail() {
        let (world, anchor, bob) = makeJointWorld()

        var jointDef = WheelJointDef()
        jointDef.base.bodyIdA = anchor
        jointDef.base.bodyIdB = bob
        jointDef.enableSuspensionLimit = true
        jointDef.lowerSuspensionLimit = -0.3
        jointDef.upperSuspensionLimit = 0.3
        _ = world.createWheelJoint(jointDef)

        let counts = drawCounts(world)
        // Link, suspension axis, two limit ticks, spin axle.
        #expect(counts.segments >= 5)
        #expect(counts.points == 2)
    }
}
