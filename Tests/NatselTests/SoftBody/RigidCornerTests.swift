import Testing
@testable import Natsel

@Suite("Soft rigid corner contacts")
struct RigidCornerTests {
    private func scene(_ workers: Int32) -> (World, SoftBodyID) {
        let world = World()
        world.gravity = Vec3(-10, -10, 0)
        world.workerCount = workers
        world.enableSleep = false
        let floor = world.createBody(BodyDef())
        let wall = world.createBody(BodyDef())
        var shape = ShapeDef()
        shape.baseMaterial.friction = 0
        _ = world.createHullShape(floor, shape, makeOffsetBoxHull(5, 0.1, 5, Vec3(0, -0.1, 0)))
        _ = world.createHullShape(wall, shape, makeOffsetBoxHull(0.1, 5, 5, Vec3(-0.1, 0, 0)))
        var settings = SoftBodySettings()
        settings.vertices = [SoftBodyVertex(position: Vec3(0.005, 0.005, 0), invMass: 1)]
        var def = SoftBodyDef(settings: settings)
        def.vertexRadius = 0.02
        def.iterationCount = 4
        def.enableSleep = false
        return (world, world.createSoftBody(def))
    }

    @Test("a vertex respects both a wall and a floor", arguments: [Int32(1), Int32(4)])
    func corner(_ workers: Int32) {
        let (world, soft) = scene(workers)
        for _ in 0..<120 {
            world.step(1 / 60, subStepCount: 4)
            let p = world.softBodyVertexPositions(soft)[0]
            #expect(p.x >= 0.019)
            #expect(p.y >= 0.019)
        }
        #expect(world.softBodyContacts(soft).count == 2)
    }

    @Test("multi-plane contact state restores deterministically")
    func rollback() {
        let (world, soft) = scene(1)
        world.step(1 / 60, subStepCount: 4)
        let saved = world.saveState()
        for _ in 0..<20 { world.step(1 / 60, subStepCount: 4) }
        let expected = world.softBodyVertexPositions(soft)
        #expect(world.restoreState(saved))
        world.workerCount = 4
        for _ in 0..<20 { world.step(1 / 60, subStepCount: 4) }
        #expect(world.softBodyVertexPositions(soft) == expected)
    }
}
