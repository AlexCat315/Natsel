import Testing
@testable import Natsel

@Suite("Shape creation")
struct ShapeCreationTests {
    @Test("shape creation and body mass match the C reference")
    func createShapes() {
        let world = World()

        var def = BodyDef()
        def.type = .dynamicBody
        def.position = Pos(1, 2, 3)
        let body = world.createBody(def)

        var shapeDef = ShapeDef()
        shapeDef.density = 2
        let sh1 = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(), radius: 0.5))

        shapeDef = ShapeDef()
        shapeDef.density = 2
        let sh2 = world.createCapsuleShape(
            body, shapeDef, Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
        )

        shapeDef = ShapeDef()
        shapeDef.density = 1
        let boxHull = makeBoxHull(1, 2, 3)
        let sh3 = world.createHullShape(body, shapeDef, boxHull)
        let sh4 = world.createHullShape(body, shapeDef, boxHull)

        #expect(world.shapes.count == 4)
        #expect(world.fatAABBs.count == 4)

        // Sphere
        var shape = world.shape(sh1)
        #expect(shape.id == 0)
        #expect(shape.bodyId == 0)
        #expect(shape.type == .sphereShape)
        #expect(shape.density.bitPattern == 0x4000_0000)
        #expect(shape.aabbMargin.bitPattern == 0x3d4c_cccd)
        #expect(shape.proxyKey == 2)
        #expect(shape.prevShapeId == 1)
        #expect(shape.nextShapeId == -1)
        #expect(shape.materialCount == 1)
        #expect(shape.generation == 1)
        #expect(shape.localCentroid == Vec3())
        #expect(shape.aabb.lowerBound.x.bitPattern == 0x3ef5c290)
        #expect(shape.aabb.lowerBound.y.bitPattern == 0x3fbd_70a4)
        #expect(shape.aabb.upperBound.z.bitPattern == 0x4061_47ae)
        var fat = world.fatAABBs[0]
        #expect(fat.lowerBound.x.bitPattern == 0x3edc28f6)
        #expect(fat.upperBound.z.bitPattern == 0x4064_7ae1)

        // Capsule
        shape = world.shape(sh2)
        #expect(shape.proxyKey == 6)
        #expect(shape.prevShapeId == 2)
        #expect(shape.nextShapeId == 0)
        #expect(shape.aabb.lowerBound.y.bitPattern == 0x3ef5c290)
        #expect(shape.aabb.upperBound.y.bitPattern == 0x4061_47ae)
        fat = world.fatAABBs[1]
        #expect(fat.upperBound.y.bitPattern == 0x4064_7ae1)

        // Hulls
        shape = world.shape(sh3)
        #expect(shape.proxyKey == 10)
        #expect(shape.prevShapeId == 3)
        #expect(shape.nextShapeId == 1)
        #expect(shape.aabb.lowerBound.x.bitPattern == 0xbca3d700)
        #expect(shape.aabb.upperBound.z.bitPattern == 0x40c0_a3d7)
        #expect(shape.hull!.hash == 0xfc96_8302_32f7_676a)

        shape = world.shape(sh4)
        #expect(shape.proxyKey == 14)
        #expect(shape.prevShapeId == -1)
        #expect(shape.nextShapeId == 2)
        #expect(shape.hull!.hash == 0xfc96_8302_32f7_676a)

        // Identical hulls are interned
        #expect(world.shape(sh3).hull! === world.shape(sh4).hull!)

        // Body mass properties
        let bodyIndex = world.bodyIndex(body)
        let b = world.bodies[Int(bodyIndex)]
        #expect(b.mass.bitPattern == 0x42ca_78d3)
        #expect(b.headShapeId == 3)
        #expect(b.shapeCount == 4)
        #expect(b.flags == 0x0001_7000)

        let sim = world.bodySimFor(body)
        #expect(sim.invMass.bitPattern == 0x3c21_d6f6)
        #expect(sim.center == Pos(1, 2, 3))
        #expect(sim.localCenter == Vec3())
        #expect(sim.minExtent.bitPattern == 0x3f00_0000)
        #expect(sim.maxExtent == Vec3(1, 2, 3))
        #expect(sim.inertia.cx.x.bitPattern == 0x43d1_724a)
        #expect(sim.inertia.cy.y.bitPattern == 0x43a0_4d12)
        #expect(sim.inertia.cz.z.bitPattern == 0x4322_e494)

        // Destroy one of the two interned hull shapes.
        world.destroyShape(sh3, updateBodyMass: true)
        #expect(world.shapes.count == 4)
        #expect(world.shapes[2].id == B3.nullIndex)
        var updatedBody = world.bodies[Int(bodyIndex)]
        #expect(updatedBody.headShapeId == 3)
        #expect(updatedBody.shapeCount == 3)
        #expect(updatedBody.mass.bitPattern == 0x4254_f1a7)

        var updatedSim = world.bodySimFor(body)
        #expect(updatedSim.invMass.bitPattern == 0x3c99_e180)
        #expect(updatedSim.inertia.cx.x.bitPattern == 0x4352_e494)

        // The remaining shape still references the interned hull.
        #expect(world.shape(sh4).hull!.hash == 0xfc96_8302_32f7_676a)
        #expect(world.shape(sh4).proxyKey == 14)

        world.destroyShape(sh4, updateBodyMass: true)
        updatedBody = world.bodies[Int(bodyIndex)]
        #expect(updatedBody.mass.bitPattern == 0x40a7_8d36)
        updatedSim = world.bodySimFor(body)
        #expect(updatedSim.invMass.bitPattern == 0x3e43_91d1)
        #expect(updatedBody.shapeCount == 2)
        #expect(world.shapes[3].id == B3.nullIndex)
    }
}
