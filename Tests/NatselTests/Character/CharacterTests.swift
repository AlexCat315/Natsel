import Testing
@testable import Natsel

private func makeWorld() -> World {
    let world = World()
    let ground = world.createBody(BodyDef())
    _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(50, 1, 50, Vec3(0, -1, 0)))
    return world
}

/// Typical game loop: gravity when airborne, walking velocity when grounded.
private func drive(_ world: World, _ character: CharacterVirtual, _ walk: Vec3, _ steps: Int, extended: Bool = true) {
    let dt: Float = 1.0 / 60.0
    for _ in 0..<steps {
        var v = character.linearVelocity
        if character.groundState == .onGround {
            v = walk
        } else {
            v.x = walk.x
            v.z = walk.z
            v = mulAdd(v, dt, world.gravity)
        }
        character.linearVelocity = v
        if extended {
            character.extendedUpdate(dt, gravity: world.gravity)
        } else {
            character.update(dt)
        }
        world.step(dt, subStepCount: 4)
    }
}

private func addStaticBox(_ world: World, _ center: Vec3, _ half: Vec3, rotation: Quat = .identity) {
    let body = world.createBody(BodyDef())
    _ = world.createHullShape(body, ShapeDef(), makeTransformedBoxHull(half.x, half.y, half.z, Transform(center, rotation)))
}

@Suite("Character controller")
struct CharacterTests {
    @Test("falls, lands, and walks on flat ground")
    func walk() {
        let world = makeWorld()
        var def = CharacterDef()
        def.position = Pos(0, 2, 0)
        let character = CharacterVirtual(world: world, def)!
        drive(world, character, Vec3(), 120)
        #expect(character.groundState == .onGround)
        #expect(character.groundNormal.y > 0.99)
        #expect(abs(character.position.y) < 0.05)

        let start = character.position
        drive(world, character, Vec3(2, 0, 0), 60)
        #expect(abs(character.position.x - start.x - 2) < 0.1)
        #expect(abs(character.position.y - start.y) < 0.02)
        #expect(character.groundState == .onGround)
    }

    @Test("walls stop the character and it slides along them")
    func wall() {
        let world = makeWorld()
        addStaticBox(world, Vec3(2, 1, 0), Vec3(0.2, 1, 5))
        var def = CharacterDef()
        def.position = Pos(0, Float(0.05), 0)
        let character = CharacterVirtual(world: world, def)!
        drive(world, character, Vec3(), 30)
        drive(world, character, Vec3(2, 0, 2), 120)
        // Stopped by the wall face at x = 1.8, minus radius and padding.
        #expect(character.position.x < 1.8 - 0.3 + 0.01)
        #expect(character.position.x > 1.8 - 0.3 - 0.06)
        // Kept sliding along z.
        #expect(character.position.z > 3)
        #expect(character.contacts.contains { $0.normal.x < -0.9 })
    }

    @Test("walkable slopes are climbed and steep slopes are not", arguments: [(Float(25), true), (Float(65), false)])
    func slopes(_ degrees: Float, _ climbable: Bool) {
        let world = makeWorld()
        let angle = degrees * B3.degToRad
        // A ramp rising toward +x, starting at x = 1.
        let rotation = makeQuatFromAxisAngle(Vec3(0, 0, 1), angle)
        let cs = computeCosSin(angle)
        addStaticBox(world, Vec3(1 + 3 * cs.cosine, 3 * cs.sine - 0.5 * cs.cosine, 0), Vec3(3, 0.5, 3), rotation: rotation)
        var def = CharacterDef()
        def.position = Pos(-1, Float(0.05), 0)
        def.enableStairStepping = false
        let character = CharacterVirtual(world: world, def)!
        drive(world, character, Vec3(), 30)
        drive(world, character, Vec3(2, 0, 0), 180)
        if climbable {
            #expect(character.position.y > 1)
        } else {
            #expect(character.position.y < 0.3)
        }
    }

    @Test("stairs are climbed with extended update, tall ledges are not")
    func stairs() {
        for (stepHeight, climbs) in [(Float(0.25), true), (Float(0.7), false)] {
            let world = makeWorld()
            for k in 0..<4 {
                addStaticBox(world, Vec3(2 + 0.5 * Float(k) + 2.5, stepHeight * Float(k + 1) * 0.5, 0),
                             Vec3(2.5, stepHeight * Float(k + 1) * 0.5, 3))
            }
            var def = CharacterDef()
            def.position = Pos(0, Float(0.05), 0)
            let character = CharacterVirtual(world: world, def)!
            drive(world, character, Vec3(), 30)
            drive(world, character, Vec3(2, 0, 0), 180)
            if climbs {
                #expect(character.position.y > Double(3 * stepHeight), "step \(stepHeight)")
                #expect(character.groundState == .onGround)
            } else {
                #expect(character.position.y < 0.1, "step \(stepHeight)")
                #expect(character.position.x < 2)
            }
        }
    }

    @Test("sticks to the floor walking down a slope")
    func stickToFloor() {
        let world = makeWorld()
        let angle: Float = 20 * B3.degToRad
        let rotation = makeQuatFromAxisAngle(Vec3(0, 0, 1), -angle)
        // Down ramp from x = 0 toward +x, top at y = 2.
        let cs = computeCosSin(angle)
        addStaticBox(world, Vec3(4 * cs.cosine, 2 - 4 * cs.sine - 0.5 * cs.cosine, 0), Vec3(4, 0.5, 3), rotation: rotation)
        addStaticBox(world, Vec3(-2, 1, 0), Vec3(2, 1, 3))
        var def = CharacterDef()
        def.position = Pos(-1, Float(2.05), 0)
        let character = CharacterVirtual(world: world, def)!
        drive(world, character, Vec3(), 30)
        var airborne = 0
        let dt: Float = 1.0 / 60.0
        for _ in 0..<90 {
            character.linearVelocity = character.groundState == .onGround
                ? Vec3(4, 0, 0) : mulAdd(character.linearVelocity, dt, world.gravity)
            character.extendedUpdate(dt, gravity: world.gravity)
            world.step(dt, subStepCount: 4)
            if character.groundState != .onGround && character.position.x > 0.3 && character.position.x < 6 {
                airborne += 1
            }
        }
        #expect(airborne <= 2)
        #expect(character.position.x > 3)
    }

    @Test("rides a moving platform")
    func platform() {
        let world = makeWorld()
        var bodyDef = BodyDef()
        bodyDef.type = .kinematicBody
        bodyDef.position = Pos(0, 1, 0)
        bodyDef.linearVelocity = Vec3(1, 0, 0)
        let platform = world.createBody(bodyDef)
        _ = world.createHullShape(platform, ShapeDef(), makeBoxHull(2, 0.1, 2))
        var def = CharacterDef()
        def.position = Pos(0, Float(1.15), 0)
        let character = CharacterVirtual(world: world, def)!
        drive(world, character, Vec3(), 120)
        #expect(character.groundState == .onGround)
        #expect(character.groundBodyId == platform)
        #expect(abs(character.groundVelocity.x - 1) < 1.0e-3)
        let offset = character.position.x - world.bodyPosition(platform).x
        #expect(abs(offset) < 0.1)
    }

    @Test("pushes dynamic bodies and is seen through its inner body")
    func pushing() {
        let world = makeWorld()
        var bodyDef = BodyDef()
        bodyDef.type = .dynamicBody
        bodyDef.position = Pos(Float(1.5), Float(0.25), 0)
        let box = world.createBody(bodyDef)
        var shapeDef = ShapeDef()
        shapeDef.density = 100
        shapeDef.baseMaterial.friction = 0.2
        _ = world.createHullShape(box, shapeDef, makeBoxHull(0.25, 0.25, 0.25))

        var def = CharacterDef()
        def.position = Pos(0, Float(0.05), 0)
        let character = CharacterVirtual(world: world, def)!
        #expect(character.innerBodyId.isNull == false)
        drive(world, character, Vec3(), 10)
        drive(world, character, Vec3(1.5, 0, 0), 180)
        #expect(world.bodyPosition(box).x > 2)

        // A ray finds the inner body.
        let hit = world.castRayClosest(mulAdd(character.position, 1, Vec3(0, 1, 0)) + Vec3(-3, 0, 0), Vec3(6, 0, 0))
        #expect(hit.hit)
        #expect(world.shapeBody(hit.shapeId) == character.innerBodyId)
        character.destroy()
        #expect(world.bodyIsValid(character.innerBodyId) == false)
    }

    @Test("crouching shape changes are refused under a low ceiling")
    func crouch() {
        let world = makeWorld()
        addStaticBox(world, Vec3(3, 1.5, 0), Vec3(1, 0.2, 2))
        var def = CharacterDef()
        def.position = Pos(0, Float(0.05), 0)
        let character = CharacterVirtual(world: world, def)!
        drive(world, character, Vec3(), 30)
        #expect(character.setShape(radius: 0.3, height: 1.0))
        drive(world, character, Vec3(1.5, 0, 0), 120)
        #expect(character.position.x > 2.5)
        // Under the ceiling (bottom at 1.3) standing up would overlap.
        #expect(character.setShape(radius: 0.3, height: 1.8) == false)
        #expect(character.height == 1.0)
    }
}
