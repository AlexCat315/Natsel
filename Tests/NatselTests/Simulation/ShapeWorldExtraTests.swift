import Testing
@testable import Natsel

@Suite("Shape and world extras")
struct ShapeWorldExtraTests {
    private func makeBody(_ world: World) -> BodyID {
        var def = BodyDef()
        def.type = .dynamicBody
        return world.createBody(def)
    }

    @Test("sensor flag and user data round trip")
    func sensorAndUserData() {
        let world = World()
        let body = makeBody(world)

        var shapeDef = ShapeDef()
        shapeDef.isSensor = true
        let sensor = world.createSphereShape(
            body, shapeDef, Sphere(center: Vec3(), radius: 0.5))
        #expect(world.shapeIsSensor(sensor))

        shapeDef = ShapeDef()
        let solid = world.createSphereShape(
            body, shapeDef, Sphere(center: Vec3(5, 0, 0), radius: 0.5))
        #expect(world.shapeIsSensor(solid) == false)

        // User data is a raw pointer and round trips exactly.
        var token = 42
        let pointer = withUnsafeMutablePointer(to: &token) { UnsafeMutableRawPointer($0) }
        world.setShapeUserData(solid, pointer)
        #expect(world.shapeUserData(solid) == pointer)

        world.setShapeUserData(solid, nil)
        #expect(world.shapeUserData(solid) == nil)
    }

    @Test("shape AABB tracks the body transform")
    func shapeAABB() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(3, 4, 5)
        let body = world.createBody(def)

        let shapeDef = ShapeDef()
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        let aabb = world.shapeAABB(shape)
        #expect(absFloat(aabb.lowerBound.x - 2.5) < 1e-4)
        #expect(absFloat(aabb.lowerBound.y - 3.5) < 1e-4)
        #expect(absFloat(aabb.lowerBound.z - 4.5) < 1e-4)
        #expect(absFloat(aabb.upperBound.x - 3.5) < 1e-4)
        #expect(absFloat(aabb.upperBound.y - 4.5) < 1e-4)
        #expect(absFloat(aabb.upperBound.z - 5.5) < 1e-4)
    }

    @Test("closest point on a sphere sits on the surface toward the target")
    func closestPoint() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(0, 0, 0)
        let body = world.createBody(def)

        let shapeDef = ShapeDef()
        let shape = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 1))

        // A target far along +x projects onto the sphere surface at (1, 0, 0).
        let point = world.shapeClosestPoint(shape, Pos(5, 0, 0))
        #expect(absFloat(point.x - 1) < 1e-4)
        #expect(absFloat(point.y) < 1e-4)
        #expect(absFloat(point.z) < 1e-4)

        // A target inside projects onto the surface too.
        let inside = world.shapeClosestPoint(shape, Pos(Float(0.25), 0, 0))
        #expect(absFloat(length(subPos(inside, .zero)) - 1) < 1e-3 || length(subPos(inside, .zero)) < 1e-3)
    }

    @Test("mesh material count and per-triangle material round trip")
    func meshMaterials() {
        let world = World()
        world.enableSleep = false

        let vertices = [
            Vec3(-3, 0, -1), Vec3(-2, 0, 1), Vec3(-1, 0, -1),
            Vec3(1, 0, -1), Vec3(2, 0, 1), Vec3(3, 0, -1),
        ]
        guard let mesh = TriangleMesh(
            vertices: vertices, indices: [0, 1, 2, 3, 4, 5], materials: [0, 1]
        ) else {
            Issue.record("mesh creation failed")
            return
        }

        let body = makeBody(world)
        var shapeDef = ShapeDef()
        var materialA = SurfaceMaterial.defaultMaterial
        materialA.friction = 0.2
        var materialB = SurfaceMaterial.defaultMaterial
        materialB.friction = 0.8
        shapeDef.materials = [materialA, materialB]

        let shape = world.createMeshShape(body, shapeDef, mesh, scale: Vec3(1, 1, 1))

        #expect(world.shapeMeshMaterialCount(shape) == 2)
        #expect(absFloat(world.shapeMeshSurfaceMaterial(shape, 0)!.friction - 0.2) < 1e-6)
        #expect(absFloat(world.shapeMeshSurfaceMaterial(shape, 1)!.friction - 0.8) < 1e-6)

        // Out of range returns nil.
        #expect(world.shapeMeshSurfaceMaterial(shape, 2) == nil)

        var replacement = SurfaceMaterial.defaultMaterial
        replacement.friction = 0.55
        world.setShapeMeshMaterial(shape, 1, replacement)
        #expect(absFloat(world.shapeMeshSurfaceMaterial(shape, 1)!.friction - 0.55) < 1e-6)
    }

    @Test("event flags can be toggled per shape")
    func eventToggles() {
        let world = World()
        let body = makeBody(world)

        let shapeDef = ShapeDef()
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        #expect(world.shapeAreHitEventsEnabled(shape) == false)
        world.enableShapeHitEvents(shape, true)
        #expect(world.shapeAreHitEventsEnabled(shape))
        world.enableShapeHitEvents(shape, false)
        #expect(world.shapeAreHitEventsEnabled(shape) == false)

        world.enableShapeContactEvents(shape, true)
        #expect(world.shapeAreContactEventsEnabled(shape))

        world.enableShapeSensorEvents(shape, true)
        #expect(world.shapeAreSensorEventsEnabled(shape))

        world.enableShapePreSolveEvents(shape, true)
        #expect(world.shapeArePreSolveEventsEnabled(shape))
    }

    @Test("world bounds cover every shape")
    func worldBounds() {
        let world = World()
        world.enableSleep = false

        // An empty world has no bounds.
        #expect(world.worldBounds() == nil)

        var def = BodyDef()
        def.type = .staticBody
        def.position = Pos(-5, 0, 0)
        let left = world.createBody(def)
        _ = world.createHullShape(left, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))

        def = BodyDef()
        def.type = .staticBody
        def.position = Pos(5, 0, 0)
        let right = world.createBody(def)
        _ = world.createHullShape(right, ShapeDef(), makeBoxHull(0.5, 0.5, 0.5))

        world.step(1.0 / 60.0, subStepCount: 1)

        guard let bounds = world.worldBounds() else {
            Issue.record("bounds missing")
            return
        }
        // Static shapes live in the static tree; the bounds span both.
        #expect(bounds.lowerBound.x <= -5.5 + 1e-3)
        #expect(bounds.upperBound.x >= 5.5 - 1e-3)
    }

    @Test("the speculative and propagation flags are public")
    func publicFlags() {
        let world = World()

        #expect(world.enableSpeculative)
        world.enableSpeculative = false
        #expect(world.enableSpeculative == false)

        #expect(world.enableRestitutionPropagation == false)
        world.enableRestitutionPropagation = true
        #expect(world.enableRestitutionPropagation)
    }
}
