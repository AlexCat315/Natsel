import Testing
@testable import Natsel

func makeTestQueryScene(_ scenario: Int, sensors: Bool = false) throws -> (World, Pos) {
        let world = World()
        let origin = scenario == 0 ? Vec3.zero : Vec3(1000000, -1000000, 2000000)
        let mesh = try #require(createGridMesh(xCount: 4, zCount: 5, cellWidth: 0.5, materialCount: 3, identifyEdges: true))
        let height = try #require(makeTestHeightField(4))
        let compound = try #require(makeTestCompound(4))
        for j in 0..<6 {
            var bodyDef = BodyDef()
            bodyDef.type = sensors ? .staticBody : j == 0 ? .dynamicBody : j == 1 ? .kinematicBody : .staticBody
            bodyDef.position = Pos(origin + Vec3(Float(j % 3) * 0.75 - 1, 0, 0))
            bodyDef.rotation = makeQuatFromAxisAngle(.axisY, Float(j) * 0.15)
            let body = world.createBody(bodyDef)
            var shapeDef = ShapeDef()
            shapeDef.filter.categoryBits = UInt64(1) << (j % 3)
            shapeDef.isSensor = sensors
            shapeDef.enableSensorEvents = true
            shapeDef.materials = Array(repeating: .defaultMaterial, count: 3)
            switch j {
            case 0: _ = world.createSphereShape(body, shapeDef, Sphere(center: Vec3(0, 0.25, 0), radius: 0.6))
            case 1: _ = world.createCapsuleShape(body, shapeDef, Capsule(center1: Vec3(0, -0.5, 0), center2: Vec3(0, 0.5, 0), radius: 0.3))
            case 2: _ = world.createHullShape(body, shapeDef, makeBoxHull(0.5, 0.25, 0.75))
            case 3: _ = world.createMeshShape(body, shapeDef, mesh)
            case 4: _ = world.createHeightFieldShape(body, shapeDef, height)
            default: _ = world.createCompoundShape(body, shapeDef, compound)
            }
        }
        return (world, Pos(origin))
}
