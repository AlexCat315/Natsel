import Testing
@testable import Natsel

// Per-body query behaviour, ported from test/test_body_query.c. These pin the origin-relative
// framing: results come back in world space and the supplied transform drives the geometry.
@Suite("Body queries")
struct BodyQueryTests {
    private func makeQueryWorld() -> (World, BodyID) {
        let world = World()
        var bodyDef = BodyDef()
        bodyDef.type = .staticBody
        let body = world.createBody(bodyDef)
        return (world, body)
    }

    private func identityAt(_ x: Float, _ y: Float, _ z: Float) -> WorldTransform {
        WorldTransform(Vec3(x, y, z), Quat.identity)
    }

    private func withProxy<R>(_ point: inout Vec3, _ radius: Float, _ body: (ShapeProxy) -> R) -> R {
        withUnsafePointer(to: &point) { pointer in
            body(ShapeProxy(points: pointer, count: 1, radius: radius))
        }
    }

    @Test("ray hits a sphere")
    func castRayHitsSphere() {
        let (world, body) = makeQueryWorld()
        var shapeDef = ShapeDef()
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 1))

        let result = world.bodyCastRay(
            body, Pos(0, 0, 0), Vec3(10, 0, 0), .defaultFilter, 1, identityAt(5, 0, 0)
        )

        #expect(result.hit)
        #expect(absFloat(result.fraction - 0.4) < 1e-5)
        #expect(absFloat(result.normal.x + 1) < 1e-5)
        #expect(absFloat(result.normal.y) < 1e-5)
        #expect(absFloat(result.normal.z) < 1e-5)

        let point = result.point
        #expect(absFloat(point.x - 4) < 1e-4)
        #expect(absFloat(point.y) < 1e-4)
        #expect(absFloat(point.z) < 1e-4)
    }

    @Test("ray misses when parallel")
    func castRayMiss() {
        let (world, body) = makeQueryWorld()
        var shapeDef = ShapeDef()
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 1))

        let result = world.bodyCastRay(
            body, Pos(0, 0, 0), Vec3(0, 10, 0), .defaultFilter, 1, identityAt(5, 0, 0)
        )
        #expect(result.hit == false)
    }

    @Test("ray reports the closest of several shapes")
    func castRayClosestShape() {
        let (world, body) = makeQueryWorld()
        let shapeDef = ShapeDef()
        let near = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 1))
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(4, 0, 0), radius: 1))

        let result = world.bodyCastRay(
            body, Pos(-5, 0, 0), Vec3(10, 0, 0), .defaultFilter, 1, identityAt(0, 0, 0)
        )

        #expect(result.hit)
        #expect(result.shapeId.index1 == near.index1)
        #expect(result.shapeId.generation == near.generation)
        #expect(absFloat(result.fraction - 0.4) < 1e-5)
    }

    @Test("far from the origin gives the same hit")
    func castRayFarFromOrigin() {
        let (world, body) = makeQueryWorld()
        var shapeDef = ShapeDef()
        _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 1))

        let origin = Pos(1.0e6, -2.0e6, 5.0e5)
        let bodyTransform = WorldTransform(offsetPos(origin, Vec3(5, 0, 0)), Quat.identity)
        let result = world.bodyCastRay(
            body, origin, Vec3(10, 0, 0), .defaultFilter, 1, bodyTransform
        )

        #expect(result.hit)
        #expect(absFloat(result.fraction - 0.4) < 1e-5)
        #expect(absFloat(result.normal.x + 1) < 1e-5)
    }

    @Test("shape cast hits a box")
    func castShapeHitsBox() {
        let (world, body) = makeQueryWorld()
        let shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(1, 1, 1))

        var point = Vec3()
        let result = withProxy(&point, 0.5) { proxy in
            world.bodyCastShape(
                body, Pos(0, 0, 0), proxy, Vec3(10, 0, 0), .defaultFilter, 1, false,
                identityAt(5, 0, 0)
            )
        }

        // Front face at world x = 4. The fraction carries a small shape-cast skin.
        #expect(result.hit)
        #expect(absFloat(result.fraction - 0.35) < 1e-2)
        #expect(absFloat(result.normal.x + 1) < 1e-4)
        #expect(absFloat(result.point.x - 4) < 1e-3)
    }

    @Test("overlap respects the supplied body transform")
    func overlapRespectsBodyTransform() {
        let (world, body) = makeQueryWorld()
        let shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(1, 1, 1))

        var point = Vec3()
        let overlapsHere = withProxy(&point, 0.5) { proxy in
            world.bodyOverlapShape(body, Pos(0, 0, 0), proxy, .defaultFilter, identityAt(0, 0, 0))
        }
        let overlapsFar = withProxy(&point, 0.5) { proxy in
            world.bodyOverlapShape(body, Pos(0, 0, 0), proxy, .defaultFilter, identityAt(20, 0, 0))
        }
        #expect(overlapsHere)
        #expect(overlapsFar == false)
    }

    @Test("overlap is relative to the origin")
    func overlapTrueAndFalse() {
        let (world, body) = makeQueryWorld()
        let shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(1, 1, 1))

        var point = Vec3()
        let touches = withProxy(&point, 0.5) { proxy in
            world.bodyOverlapShape(body, Pos(5, 0, 0), proxy, .defaultFilter, identityAt(5, 0, 0))
        }
        let misses = withProxy(&point, 0.5) { proxy in
            world.bodyOverlapShape(body, Pos(20, 0, 0), proxy, .defaultFilter, identityAt(5, 0, 0))
        }
        #expect(touches)
        #expect(misses == false)
    }

    @Test("mover grazes a box face")
    func moverTouchesBox() {
        let (world, body) = makeQueryWorld()
        let shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        // Mover core runs above the +Y face; its 0.2 radius reaches 0.1 into it.
        let mover = Capsule(
            center1: Vec3(-0.3, 0.6, 0), center2: Vec3(0.3, 0.6, 0), radius: 0.2
        )
        let planes = world.bodyCollideMover(
            body, Pos(0, 0, 0), mover, .defaultFilter, identityAt(0, 0, 0), capacity: 4
        )

        #expect(planes.count == 1)
        #expect(isNormalized(planes[0].result.plane.normal))
        #expect(planes[0].result.plane.normal.y > 0.99)
        #expect(absFloat(planes[0].result.plane.offset - 0.1) < 1e-4)
    }

    @Test("mover respects the capacity limit")
    func moverCapacity() {
        let (world, body) = makeQueryWorld()
        let shapeDef = ShapeDef()
        for i in 0..<4 {
            _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))
            _ = i
        }

        let mover = Capsule(
            center1: Vec3(-0.3, 0.6, 0), center2: Vec3(0.3, 0.6, 0), radius: 0.2
        )
        let planes = world.bodyCollideMover(
            body, Pos(0, 0, 0), mover, .defaultFilter, identityAt(0, 0, 0), capacity: 2
        )
        #expect(planes.count == 2)
    }

    @Test("separated mover reports nothing")
    func moverSeparated() {
        let (world, body) = makeQueryWorld()
        let shapeDef = ShapeDef()
        _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        let mover = Capsule(
            center1: Vec3(-0.3, 5, 0), center2: Vec3(0.3, 5, 0), radius: 0.2
        )
        let planes = world.bodyCollideMover(
            body, Pos(0, 0, 0), mover, .defaultFilter, identityAt(0, 0, 0), capacity: 4
        )
        #expect(planes.isEmpty)
    }
}
