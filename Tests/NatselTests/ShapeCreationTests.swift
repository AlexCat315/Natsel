import Testing
@testable import Natsel

@Suite("Shape creation")
struct ShapeCreationTests {
    @Test("shape creation and body mass are repeatable")
    func createShapes() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
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
            samples.append(UInt64(shape.density.bitPattern))
            samples.append(UInt64(shape.aabbMargin.bitPattern))
            #expect(shape.proxyKey == 2)
            #expect(shape.prevShapeId == 1)
            #expect(shape.nextShapeId == -1)
            #expect(shape.materialCount == 1)
            #expect(shape.generation == 1)
            #expect(shape.localCentroid == Vec3())
            samples.append(UInt64(shape.aabb.lowerBound.x.bitPattern))
            samples.append(UInt64(shape.aabb.lowerBound.y.bitPattern))
            samples.append(UInt64(shape.aabb.upperBound.z.bitPattern))
            var fat = world.fatAABBs[0]
            samples.append(UInt64(fat.lowerBound.x.bitPattern))
            samples.append(UInt64(fat.upperBound.z.bitPattern))

            // Capsule
            shape = world.shape(sh2)
            #expect(shape.proxyKey == 6)
            #expect(shape.prevShapeId == 2)
            #expect(shape.nextShapeId == 0)
            samples.append(UInt64(shape.aabb.lowerBound.y.bitPattern))
            samples.append(UInt64(shape.aabb.upperBound.y.bitPattern))
            fat = world.fatAABBs[1]
            samples.append(UInt64(fat.upperBound.y.bitPattern))

            // Hulls
            shape = world.shape(sh3)
            #expect(shape.proxyKey == 10)
            #expect(shape.prevShapeId == 3)
            #expect(shape.nextShapeId == 1)
            samples.append(UInt64(shape.aabb.lowerBound.x.bitPattern))
            samples.append(UInt64(shape.aabb.upperBound.z.bitPattern))
            samples.append(UInt64(shape.hull!.hash))

            shape = world.shape(sh4)
            #expect(shape.proxyKey == 14)
            #expect(shape.prevShapeId == -1)
            #expect(shape.nextShapeId == 2)
            samples.append(UInt64(shape.hull!.hash))

            // Identical hulls are interned
            #expect(world.shape(sh3).hull! === world.shape(sh4).hull!)

            // Body mass properties
            let bodyIndex = world.bodyIndex(body)
            let b = world.bodies[Int(bodyIndex)]
            samples.append(UInt64(b.mass.bitPattern))
            #expect(b.headShapeId == 3)
            #expect(b.shapeCount == 4)
            samples.append(UInt64(b.flags))

            let sim = world.bodySimFor(body)
            samples.append(UInt64(sim.invMass.bitPattern))
            #expect(sim.center == Pos(1, 2, 3))
            #expect(sim.localCenter == Vec3())
            samples.append(UInt64(sim.minExtent.bitPattern))
            #expect(sim.maxExtent == Vec3(1, 2, 3))
            samples.append(UInt64(sim.inertia.cx.x.bitPattern))
            samples.append(UInt64(sim.inertia.cy.y.bitPattern))
            samples.append(UInt64(sim.inertia.cz.z.bitPattern))

            // Destroy one of the two interned hull shapes.
            world.destroyShape(sh3, updateBodyMass: true)
            #expect(world.shapes.count == 4)
            #expect(world.shapes[2].id == B3.nullIndex)
            var updatedBody = world.bodies[Int(bodyIndex)]
            #expect(updatedBody.headShapeId == 3)
            #expect(updatedBody.shapeCount == 3)
            samples.append(UInt64(updatedBody.mass.bitPattern))

            var updatedSim = world.bodySimFor(body)
            samples.append(UInt64(updatedSim.invMass.bitPattern))
            samples.append(UInt64(updatedSim.inertia.cx.x.bitPattern))

            // The remaining shape still references the interned hull.
            samples.append(UInt64(world.shape(sh4).hull!.hash))
            #expect(world.shape(sh4).proxyKey == 14)

            world.destroyShape(sh4, updateBodyMass: true)
            updatedBody = world.bodies[Int(bodyIndex)]
            samples.append(UInt64(updatedBody.mass.bitPattern))
            updatedSim = world.bodySimFor(body)
            samples.append(UInt64(updatedSim.invMass.bitPattern))
            #expect(updatedBody.shapeCount == 2)
            #expect(world.shapes[3].id == B3.nullIndex)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
