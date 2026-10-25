import Testing
@testable import Natsel

// Large-world behaviour ported from the C collision and large-world tests.
@Suite("Large world")
struct LargeWorldTests {
    /// Two cubes overlapping by 0.1 along x produce a four point face manifold at the origin.
    @Test("a two face overlap yields four points at the origin")
    func manifoldAtOrigin() {
        let boxA = makeBoxHull(0.5, 0.5, 0.5)
        let boxB = makeBoxHull(0.5, 0.5, 0.5)

        let xfA = WorldTransform(Vec3(), Quat.identity)
        let xfB = WorldTransform(Vec3(0.9, 0, 0), Quat.identity)

        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 8)
        var manifold = LocalManifold()
        var cache = SATCache()
        buffer.withUnsafeMutableBufferPointer { pts in
            manifold.points = pts.baseAddress
            collideHulls(&manifold, 8, boxA, boxB, invMulWorldTransforms(xfA, xfB), &cache)
        }

        // Two cube faces overlap, so the clipped manifold has four points.
        #expect(manifold.pointCount == 4)
        for i in 0..<Int(manifold.pointCount) {
            #expect(absFloat(buffer[i].separation + 0.1) < 0.01, "point \(i)")
        }
    }

    /// The tight AABB of a unit cube is exactly its half extent, with the margin folded in.
    @Test("a unit cube keeps its exact extent in the fat AABB")
    func aabbAtOrigin() {
        let box = makeBoxHull(0.5, 0.5, 0.5)
        var shape = Shape()
        shape.geometry = .hull(box)
        shape.type = .hullShape

        let aabb = computeFatShapeAABB(shape, WorldTransform(Vec3(), Quat.identity), 0)
        let eps = Float.ulpOfOne

        #expect(absFloat(aabb.lowerBound.x + 0.5) <= eps)
        #expect(absFloat(aabb.lowerBound.y + 0.5) <= eps)
        #expect(absFloat(aabb.lowerBound.z + 0.5) <= eps)
        #expect(absFloat(aabb.upperBound.x - 0.5) <= eps)
        #expect(absFloat(aabb.upperBound.y - 0.5) <= eps)
        #expect(absFloat(aabb.upperBound.z - 0.5) <= eps)
    }

    /// A shape placed far from the origin must still be found by its own AABB query.
    @Test("a shape far from the origin stays inside its own bounds")
    func farShapeBounds() {
        let world = World()
        world.enableSleep = false

        let farFar: Float = 1.0e5
        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(farFar, -farFar, farFar)
        let body = world.createBody(def)
        _ = world.createHullShape(body, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))

        world.step(1.0 / 60.0, subStepCount: 1)

        // A tiny query point at the body centre must overlap the shape.
        var point = Vec3(farFar, -farFar, farFar)
        var hits = 0
        withUnsafePointer(to: &point) { ptr in
            let proxy = ShapeProxy(points: ptr, count: 1, radius: 0)
            _ = world.overlapShape(Pos(), proxy, .defaultFilter) { _ in
                hits += 1
                return true
            }
        }
        #expect(hits == 1)

        // A ray straight down must hit the top face at the expected height.
        let result = world.castRayClosest(
            Pos(farFar, -farFar + 3, farFar), Vec3(0, -6, 0)
        )
        #expect(result.hit)
        if result.hit {
            let gap = result.point.y - Double(-farFar + 0.5)
            #expect(absFloat(gap) < 0.01)
        }
    }
}

// MARK: - Scene ported from test/test_large_world.c

extension LargeWorldTests {
    private static let stackCount = 6
    private static let maxSteps = 400

    /// Drop a short stack onto a ground box centered at `baseX`. Returns each body's final
    /// position relative to the base and the step on which the stack settles.
    private func runStack(_ baseX: Float) -> (positions: [Vec3], sleepStep: Int) {
        let base = Pos(baseX, 0, 0)

        let world = World()

        var def = BodyDef()
        def.position = base
        let ground = world.createBody(def)
        _ = world.createHullShape(ground, ShapeDef(), makeBoxHull(10, 1, 10))

        var bodies: [BodyID] = []
        for i in 0..<Self.stackCount {
            def = BodyDef()
            def.type = .dynamicBody
            def.position = offsetPos(base, Vec3(0, 2 + 1.05 * Float(i), 0))
            let body = world.createBody(def)

            var shapeDef = ShapeDef()
            shapeDef.density = 1
            _ = world.createHullShape(body, shapeDef, makeCubeHull(0.5))
            bodies.append(body)
        }

        var sleepStep = -1
        for step in 0..<Self.maxSteps {
            world.step(1.0 / 60.0, subStepCount: 4)
            if world.isBodyAwake(bodies[Self.stackCount - 1]) == false {
                sleepStep = step
                break
            }
        }

        let positions = bodies.map { subPos(world.bodyTransform($0).p, base) }
        return (positions, sleepStep)
    }

    @Test("a six box stack settles at the origin")
    func stackSettles() {
        let result = runStack(0)
        #expect(result.sleepStep >= 0)
        // The stack must not sink into the ground or scatter.
        for (i, p) in result.positions.enumerated() {
            #expect(p.y > 0.4, "body \(i) y \(p.y)")
            #expect(absFloat(p.x) < 0.5, "body \(i) x \(p.x)")
            #expect(absFloat(p.z) < 0.5, "body \(i) z \(p.z)")
        }
    }

    /// Fire a fast bullet at a thin wall. Returns the bullet's final x. If continuous collision
    /// works the bullet stops at the wall instead of tunneling past it.
    @Test("a fast bullet does not tunnel through a thin wall", arguments: [0.0, 1.0e9, -1.0e9, 1.0e12])
    func bulletStops(_ offset: Double) {
        let base = Pos(offset, offset, offset)
        let world = World()

        // Thin static wall at x = 5, spanning y and z.
        var def = BodyDef()
        def.type = .staticBody
        def.position = offsetPos(base, Vec3(5, 0, 0))
        let wall = world.createBody(def)
        _ = world.createHullShape(wall, ShapeDef(), makeBoxHull(0.05, 5, 5))

        // Small fast bullet aimed at the wall, no gravity.
        def = BodyDef()
        def.type = .dynamicBody
        def.isBullet = true
        def.gravityScale = 0
        def.position = base
        def.linearVelocity = Vec3(200, 0, 0)
        let bullet = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1
        _ = world.createSphereShape(bullet, shapeDef, Sphere(center: Vec3(), radius: 0.1))

        for _ in 0..<30 { world.step(1.0 / 60.0, subStepCount: 4) }

        let finalX = world.bodyTransform(bullet).p.x - base.x
        // Wall front face is at 5 - 0.05; the radius is 0.1, so a caught bullet stays short of 5.
        #expect(finalX < 5)
    }

    /// The origin-relative queries hit at the origin with the expected geometry.
    @Test("origin relative queries report the expected geometry", arguments: [0.0, 1.0e9, -1.0e9, 1.0e12])
    func queriesAtOrigin(_ offset: Double) {
        let base = Pos(offset, offset, offset)
        let world = World()

        var def = BodyDef()
        def.type = .staticBody
        def.position = base
        let body = world.createBody(def)
        var shapeDef = ShapeDef()
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(1, 1, 1))

        world.step(1.0 / 60.0, subStepCount: 1)

        // Sphere proxy swept from the left into the box, hitting the left face at relative x = -1.
        var castPoint = Vec3(-5, 0, 0)
        var castRelX: Float = 0
        var castHit = false
        withUnsafePointer(to: &castPoint) { ptr in
            let proxy = ShapeProxy(points: ptr, count: 1, radius: 0.25)
            _ = world.castShape(base, proxy, Vec3(10, 0, 0)) { _, point, _, _ in
                castHit = true
                castRelX = Float(point.x - base.x)
                return 0
            }
        }
        #expect(castHit)
        #expect(absFloat(castRelX + 1) < 0.05)

        // Sphere proxy sitting at the box center.
        var overlapPoint = Vec3(0, 0, 0)
        var overlapHit = false
        withUnsafePointer(to: &overlapPoint) { ptr in
            let proxy = ShapeProxy(points: ptr, count: 1, radius: 0.5)
            _ = world.overlapShape(base, proxy, .defaultFilter) { _ in
                overlapHit = true
                return true
            }
        }
        #expect(overlapHit)

        // Capsule swept from the left into the box.
        let moverCast = Capsule(
            center1: Vec3(-5, -0.3, 0), center2: Vec3(-5, 0.3, 0), radius: 0.25)
        let moverFraction = world.castMover(base, moverCast, Vec3(10, 0, 0))
        #expect(moverFraction < 1)

        // Capsule overlapping the left face reports a contact plane.
        let moverCollide = Capsule(
            center1: Vec3(-1.1, -0.3, 0), center2: Vec3(-1.1, 0.3, 0), radius: 0.3)
        var planeCount = 0
        world.collideMover(base, moverCollide) { _, planes in
            planeCount += planes.count
            return true
        }
        #expect(planeCount > 0)

        // World ray cast from the left into the box, hitting the left face at relative x = -1.
        let ray = world.castRayClosest(
            offsetPos(base, Vec3(-5, 0, 0)), Vec3(10, 0, 0))
        #expect(ray.hit)
        if ray.hit {
            #expect(absFloat((ray.point.x - base.x) + 1) < 0.05)
        }

        // Direct shape ray cast against the same box.
        let shapeRay = world.shapeRayCast(shape, offsetPos(base, Vec3(-5, 0, 0)), Vec3(10, 0, 0))
        #expect(shapeRay.hit)
        if shapeRay.hit {
            #expect(absFloat((shapeRay.point.x - base.x) + 1) < 0.05)
        }
    }
}
