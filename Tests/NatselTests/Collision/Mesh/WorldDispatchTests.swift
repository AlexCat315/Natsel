import Testing
@testable import Natsel

@Suite("Mesh world query dispatch")
struct MeshWorldDispatchTests {
    @Test("mesh creation, scaled casts, overlap, and destruction use world transforms")
    func queries() throws {
        let world = World()
        var bodyDef = BodyDef()
        bodyDef.position = Pos(1, 2, -1)
        let body = world.createBody(bodyDef)
        let data = try #require(createGridMesh(xCount: 5, zCount: 4, cellWidth: 0.75, materialCount: 3, identifyEdges: true))
        var shapeDef = ShapeDef()
        shapeDef.materials = (0..<3).map { i in
            var material = SurfaceMaterial()
            material.userMaterialId = UInt64(10 * (i + 1))
            return material
        }
        let shape = world.createMeshShape(body, shapeDef, data, scale: Vec3(-2, 0.5, 1.5))
        #expect(world.shapeIsValid(shape))
        let origin = Pos(Float(1.125), 4, Float(-0.75))
        let translation = Vec3(0, -4, 0)
        let ray = world.castRayClosest(origin, translation)
        #expect(ray.hit)
        #expect(ray.shapeId == shape)
        #expect(ray.fraction == 0.5)
        #expect(ray.point == Pos(Float(1.125), 2, Float(-0.75)))
        #expect(ray.normal == .axisY)
        #expect([UInt64(10), 20, 30].contains(ray.userMaterialId))
        var center = Vec3(1.125, 2.1, -0.75)
        withUnsafePointer(to: &center) { pointer in
            var count = 0
            let proxy = ShapeProxy(points: pointer, count: 1, radius: 0.25)
            _ = world.overlapShape(.zero, proxy) { id in
                #expect(id == shape)
                count += 1
                return true
            }
            #expect(count == 1)
        }
        let meshShape = world.shape(shape)
        let mesh = Mesh(data: data, scale: Vec3(-2, 0.5, 1.5))
        let transform = WorldTransform(bodyDef.position, .identity)
        var point = Vec3(1.125, 4, -0.75)
        withUnsafePointer(to: &point) { pointer in
            var input = ShapeCastInput()
            input.proxy = ShapeProxy(points: pointer, count: 1, radius: 0.25)
            input.translation = translation
            input.maxFraction = 1
            let output = shapeCastShape(meshShape, toRelativeTransform(transform, .zero), input)
            var localPoint = subPos(Pos(pointer.pointee), bodyDef.position)
            withUnsafePointer(to: &localPoint) { localPointer in
                input.proxy.points = localPointer
                let local = shapeCastMesh(mesh, input)
                #expect(output.hit)
                #expect(output.fraction.bitPattern == local.fraction.bitPattern)
                #expect(distance(Pos(output.point), offsetPos(bodyDef.position, local.point)) < 1.0e-5)
                #expect(output.triangleIndex == local.triangleIndex)
            }
        }
        world.destroyShape(shape, updateBodyMass: true)
        #expect(!world.shapeIsValid(shape))
        #expect(!world.castRayClosest(origin, translation).hit)
    }
}
