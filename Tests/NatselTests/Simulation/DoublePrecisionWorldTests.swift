import Testing
@testable import Natsel

@Suite("Double precision large worlds")
struct DoublePrecisionWorldTests {
    @Test("world position boundaries retain small offsets", arguments: [1.0e9, -1.0e9, 1.0e12])
    func positions(_ offset: Double) {
        let p = Pos(offset + 0.125, offset - 0.25, offset + 0.5)
        let d = Vec3(0.25, -0.125, 0.5)
        #expect(subPos(offsetPos(p, d), p) == d)
        #expect(lerpPosition(p, offsetPos(p, d), 0.5) == offsetPos(p, 0.5 * d))
        let box = offsetAABB(AABB(Vec3(-0.1, -0.2, -0.3), Vec3(0.1, 0.2, 0.3)), p)
        #expect(Double(box.lowerBound.x) <= p.x - 0.1)
        #expect(Double(box.upperBound.x) >= p.x + 0.1)
    }

    @Test("queries, integration and rollback retain fractional coordinates", arguments: [1.0e9, -1.0e9, 1.0e12])
    func queriesAndRollback(_ offset: Double) {
        let world = World()
        world.gravity = .zero
        var def = BodyDef()
        def.position = Pos(offset + 0.125, offset, offset)
        let ground = world.createBody(def)
        let shape = world.createSphereShape(ground, ShapeDef(), Sphere(center: Vec3(), radius: 0.5))
        let start = Pos(offset - 2, offset, offset)
        let result = world.castRayClosest(start, Vec3(4, 0, 0))
        #expect(result.hit)
        #expect(abs(result.point.x - (offset - 0.375)) < 0.001)
        let single = world.shapeRayCast(shape, start, Vec3(4, 0, 0))
        #expect(single.hit)
        #expect(single.point == result.point)
        let closest = world.shapeClosestPoint(shape, Pos(offset + 2, offset, offset))
        #expect(abs(closest.x - (offset + 0.625)) < 0.001)
        def.type = .dynamicBody
        def.position = Pos(offset, offset + 3, offset)
        def.linearVelocity = Vec3(0.125, 0, 0)
        def.enableSleep = false
        let body = world.createBody(def)
        _ = world.createSphereShape(body, ShapeDef(), Sphere(center: Vec3(), radius: 0.1))
        let state = world.saveState()
        for _ in 0..<60 { world.step(1.0 / 60, subStepCount: 4) }
        let position = world.bodyPosition(body)
        #expect(abs(position.x - offset - 0.125) < 0.01)
        let hash = world.worldStateHash()
        #expect(world.restoreState(state))
        for _ in 0..<60 { world.step(1.0 / 60, subStepCount: 4) }
        #expect(world.worldStateHash() == hash)
    }

    @Test("rigid and soft contact remain stable far from the origin")
    func contacts() {
        let world = World()
        let origin = Pos(1.0e9 + 0.125, 1.0e9, -1.0e9)
        var def = BodyDef()
        def.position = origin
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(10, 1, 10, Vec3(0, -1, 0)))
        def.type = .dynamicBody
        def.position = offsetPos(origin, Vec3(0, 3, 0))
        let body = world.createBody(def)
        _ = world.createSphereShape(body, ShapeDef(), Sphere(center: Vec3(), radius: 0.5))
        var soft = SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 3, rows: 3, spacing: 0.25))
        soft.position = offsetPos(origin, Vec3(2, 2, 0))
        let cloth = world.createSoftBody(soft)
        for _ in 0..<180 { world.step(1.0 / 60, subStepCount: 4) }
        #expect(abs(world.bodyPosition(body).y - origin.y - 0.5) < 0.03)
        for p in world.softBodyVertexPositions(cloth) {
            #expect(p.y - origin.y > -0.05)
            #expect(p.y - origin.y < 0.15)
        }
    }
}


extension DoublePrecisionWorldTests {
    @Test("precise bounds reject float broad-phase false positives")
    func preciseBounds() {
        let world = World()
        let origin = Pos(1.0e12 + 0.125, 1.0e12, -1.0e12)
        var def = BodyDef()
        def.position = origin
        let body = world.createBody(def)
        let shape = world.createSphereShape(body, ShapeDef(), Sphere(center: .zero, radius: 0.25))
        let bounds = world.shapeWorldAABB(shape)
        #expect(bounds.contains(offsetPos(origin, Vec3(0.25, 0, 0))))
        #expect(bounds == world.bodyWorldAABB(body))
        var hits = 0
        _ = world.overlapAABB(WorldAABB(localBounds: AABB(Vec3(-0.1, -0.1, -0.1), Vec3(0.1, 0.1, 0.1)), origin: origin)) {
            _ in hits += 1; return true
        }
        #expect(hits == 1)
        hits = 0
        _ = world.overlapAABB(WorldAABB(localBounds: AABB(Vec3(1, 1, 1), Vec3(2, 2, 2)), origin: origin)) {
            _ in hits += 1; return true
        }
        #expect(hits == 0)
    }

    @Test("joint anchors and character ground queries work at distant coordinates")
    func distantSystems() {
        let world = World()
        let origin = Pos(1.0e9 + 0.125, -1.0e9, 1.0e9 + 0.375)
        var def = BodyDef()
        def.position = origin
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeOffsetBoxHull(20, 1, 20, Vec3(0, -1, 0)))
        def.type = .dynamicBody
        def.position = offsetPos(origin, Vec3(0, 4, 0))
        let body = world.createBody(def)
        _ = world.createSphereShape(body, ShapeDef(), Sphere(center: .zero, radius: 0.2))
        var joint = DistanceJointDef()
        joint.base.bodyIdA = ground
        joint.base.bodyIdB = body
        joint.base.localFrameA.p = Vec3(0, 5, 0)
        joint.length = 1
        _ = world.createDistanceJoint(joint)
        var characterDef = CharacterDef()
        characterDef.position = offsetPos(origin, Vec3(3, 0.01, 0))
        let character = CharacterVirtual(world: world, characterDef)!
        for _ in 0..<120 {
            world.step(1.0 / 60, subStepCount: 4)
            character.linearVelocity = Vec3(0, character.linearVelocity.y - 10.0 / 60, 0)
            character.extendedUpdate(1.0 / 60, gravity: world.gravity)
        }
        #expect(abs(world.bodyPosition(body).y - origin.y - 4) < 0.01)
        #expect(character.groundState == .onGround)
        #expect(abs(character.position.y - origin.y) < 0.05)
    }
}


extension DoublePrecisionWorldTests {
    @Test("fluid volume keeps its precise surface height far from the origin")
    func distantFluid() {
        let world = World()
        let origin = Pos(1.0e12 + 0.125, 1.0e12, -1.0e12)
        var def = BodyDef()
        def.type = .dynamicBody
        def.position = origin
        let body = world.createBody(def)
        _ = world.createSphereShape(body, ShapeDef(), Sphere(center: .zero, radius: 0.5))
        let submerged = world.bodySubmergedVolume(body, surfacePosition: origin, surfaceNormal: .axisY)
        #expect(abs(submerged.submergedVolume / submerged.totalVolume - 0.5) < 1.0e-5)
        let bounds = WorldAABB(localBounds: AABB(Vec3(-1, -1, -1), Vec3(1, 0, 1)), origin: origin)
        #expect(world.applyBuoyancy(in: bounds, buoyancy: 2, linearDrag: 0, angularDrag: 0, timeStep: 1.0 / 60) == 1)
        #expect(world.bodyLinearVelocity(body).y > 0)
    }
}
