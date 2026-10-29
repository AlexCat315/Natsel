import Testing
@testable import Natsel

@Suite("Shape accessors")
struct ShapeAccessorTests {
    @Test("shape type and owner are reported")
    func typeAndOwner() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)

        let shapeDef = ShapeDef()
        let sphereShape = world.createSphereShape(
            body, shapeDef, Sphere(center: Vec3(), radius: 0.5))
        let capsuleShape = world.createCapsuleShape(
            body, shapeDef,
            Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.2))
        let hullShape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        #expect(world.shapeType(sphereShape) == .sphereShape)
        #expect(world.shapeType(capsuleShape) == .capsuleShape)
        #expect(world.shapeType(hullShape) == .hullShape)

        for shape in [sphereShape, capsuleShape, hullShape] {
            #expect(world.shapeIsValid(shape))
            #expect(world.shapeBody(shape).index1 == body.index1)
        }
    }

    @Test("geometry accessors return the matching type only")
    func geometryAccess() {
        let world = World()
        world.enableSleep = false

        let def = BodyDef()
        let body = world.createBody(def)
        let shapeDef = ShapeDef()

        let sphereShape = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(1, 2, 3), radius: 0.75))
        #expect(world.shapeSphere(sphereShape)?.radius == 0.75)
        #expect(world.shapeCapsule(sphereShape) == nil)
        #expect(world.shapeHull(sphereShape) == nil)

        let capsuleShape = world.createCapsuleShape(
            body, shapeDef,
            Capsule(center1: Vec3(-1, 0, 0), center2: Vec3(1, 0, 0), radius: 0.2))
        #expect(world.shapeCapsule(capsuleShape)?.radius == 0.2)
        #expect(world.shapeSphere(capsuleShape) == nil)

        let hullShape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.4, 0.3))
        #expect(world.shapeHull(hullShape) != nil)
        #expect(world.shapeCapsule(hullShape) == nil)
    }

    @Test("friction and restitution round trip through the base material")
    func materialEditing() {
        let world = World()
        world.enableSleep = false

        let def = BodyDef()
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.baseMaterial.friction = 0.3
        shapeDef.baseMaterial.restitution = 0.1
        shapeDef.density = 500
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        #expect(absFloat(world.shapeFriction(shape) - 0.3) < 1e-6)
        #expect(absFloat(world.shapeRestitution(shape) - 0.1) < 1e-6)
        #expect(absFloat(world.shapeDensity(shape) - 500) < 1e-6)

        world.setShapeFriction(shape, 0.85)
        world.setShapeRestitution(shape, 0.65)
        #expect(absFloat(world.shapeFriction(shape) - 0.85) < 1e-6)
        #expect(absFloat(world.shapeRestitution(shape) - 0.65) < 1e-6)

        // Replacing the whole material carries every field.
        var material = SurfaceMaterial.defaultMaterial
        material.friction = 0.15
        material.restitution = 0.9
        material.rollingResistance = 0.05
        material.userMaterialId = 42
        world.setShapeSurfaceMaterial(shape, material)

        let back = world.shapeSurfaceMaterial(shape)
        #expect(absFloat(back.friction - 0.15) < 1e-6)
        #expect(absFloat(back.restitution - 0.9) < 1e-6)
        #expect(absFloat(back.rollingResistance - 0.05) < 1e-6)
        #expect(back.userMaterialId == 42)
    }

    @Test("setting density marks the body mass dirty")
    func densityMarksDirty() {
        let world = World()
        world.enableSleep = false

        var def = BodyDef()
        def.type = .dynamicBody
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 1000
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        let massBefore = world.bodyMass(body)
        world.setShapeDensity(shape, 2000)

        // The mass is recomputed on the next step.
        world.step(1.0 / 60.0, subStepCount: 1)
        let massAfter = world.bodyMass(body)
        #expect(absFloat(massAfter - massBefore * 2) < massAfter * 1e-3)
    }

    @Test("event flags reflect the shape definition")
    func eventFlags() {
        let world = World()
        world.enableSleep = false

        let def = BodyDef()
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.enableSensorEvents = true
        shapeDef.enableContactEvents = true
        shapeDef.enableHitEvents = false
        shapeDef.enablePreSolveEvents = true
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        #expect(world.shapeAreSensorEventsEnabled(shape))
        #expect(world.shapeAreContactEventsEnabled(shape))
        #expect(world.shapeAreHitEventsEnabled(shape) == false)
        #expect(world.shapeArePreSolveEventsEnabled(shape))
    }

    @Test("the filter is reported")
    func filter() {
        let world = World()
        world.enableSleep = false

        let def = BodyDef()
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.filter.categoryBits = 0b0100
        shapeDef.filter.maskBits = 0b1010
        let shape = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.5, 0.5))

        let filter = world.shapeFilter(shape)
        #expect(filter.categoryBits == 0b0100)
        #expect(filter.maskBits == 0b1010)
    }
}
