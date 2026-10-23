import Testing
@testable import Natsel

@Suite("Compound world lifecycle")
struct CompoundWorldLifecycleTests {
    @Test("compound queries use body transforms and embedded materials")
    func queries() throws {
        let world = World()
        var bodyDef = BodyDef()
        bodyDef.position = Vec3(2, 3, -1)
        bodyDef.rotation = makeQuatFromAxisAngle(.axisY, 0.3)
        let body = world.createBody(bodyDef)
        let compound = try #require(referenceCompound(4))
        let shapeID = world.createCompoundShape(body, ShapeDef(), compound)
        #expect(world.shapeIsValid(shapeID))
        #expect(world.compoundShapeCount == 1)
        let shape = world.shape(shapeID)
        let transform = Transform(bodyDef.position, bodyDef.rotation)
        for i in 0..<20 {
            let origin = Vec3(Float(i) * 0.15 - 1, 2, 0.5)
            let translation = Vec3(0, -4, 0)
            let worldOrigin = transformPoint(transform, origin)
            let worldTranslation = rotateVector(transform.q, translation)
            let ray = RayCastInput(origin: invTransformPoint(transform, worldOrigin),
                                   translation: invRotateVector(transform.q, worldTranslation), maxFraction: 1)
            let expected = rayCastCompound(compound, ray)
            let actual = world.castRayClosest(worldOrigin, worldTranslation)
            #expect(actual.hit == expected.hit)
            if expected.hit {
                #expect(actual.childIndex == expected.childIndex)
                #expect(actual.triangleIndex == expected.triangleIndex)
                #expect(actual.userMaterialId == compound.materials[Int(expected.materialIndex)].userMaterialId)
                #expect(abs(actual.fraction - expected.fraction) < 0.00001)
            }
            let point = Vec3(origin.x, 0, origin.z)
            withUnsafePointer(to: point) { pointer in
                let proxy = ShapeProxy(points: pointer, count: 1, radius: 0.3)
                let expectedOverlap = overlapCompound(compound, .identity, proxy)
                let worldPoint = transformPoint(transform, point)
                withUnsafePointer(to: worldPoint) { worldPointer in
                    let worldProxy = ShapeProxy(points: worldPointer, count: 1, radius: 0.3)
                    var count = 0
                    _ = world.overlapShape(.zero, worldProxy) { _ in count += 1; return true }
                    #expect((count == 1) == expectedOverlap)
                }
                var input = ShapeCastInput()
                input.proxy = proxy
                input.translation = translation
                input.maxFraction = 1
                input.canEncroach = true
                let cast = shapeCastShape(shape, transform, input)
                let localPoint = invTransformPoint(transform, point)
                let expectedCast = withUnsafePointer(to: localPoint) { localPointer in
                    var local = input
                    local.proxy.points = localPointer
                    local.translation = invRotateVector(transform.q, input.translation)
                    return shapeCastCompound(compound, local)
                }
                #expect(cast.hit == expectedCast.hit)
                #expect(cast.fraction.bitPattern == expectedCast.fraction.bitPattern)
                #expect(cast.childIndex == expectedCast.childIndex)
                #expect(cast.materialIndex == expectedCast.materialIndex)
                #expect(cast.point == transformPoint(transform, expectedCast.point))
            }
        }
        world.destroyShape(shapeID, updateBodyMass: true)
        #expect(world.compoundShapeCount == 0)
        #expect(!world.shapeIsValid(shapeID))
        bodyDef.type = .dynamicBody
        let dynamic = world.createBody(bodyDef)
        #expect(!world.shapeIsValid(world.createCompoundShape(dynamic, ShapeDef(), compound)))
    }

    @Test("destroying a compound removes every child contact")
    func destruction() throws {
        let world = World()
        let ground = world.createBody(BodyDef())
        let compound = try #require(referenceCompound(5))
        let shape = world.createCompoundShape(ground, ShapeDef(), compound)
        var definition = BodyDef()
        definition.type = .dynamicBody
        definition.position = Vec3(-0.4, 0.7, 0.4)
        let body = world.createBody(definition)
        _ = world.createSphereShape(body, ShapeDef(), Sphere(center: .zero, radius: 0.5))
        world.step(1 / 60, subStepCount: 4)
        #expect(world.contacts.contains { $0.contactId != B3.nullIndex })
        world.destroyShape(shape, updateBodyMass: true)
        #expect(!world.contacts.contains { $0.contactId != B3.nullIndex })
        #expect(world.compoundShapeCount == 0)
        world.step(1 / 60, subStepCount: 4)
    }
}
