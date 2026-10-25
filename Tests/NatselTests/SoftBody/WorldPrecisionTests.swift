import Testing
@testable import Natsel

@Suite("Soft body world precision")
struct SoftBodyWorldPrecisionTests {
    @Test("distant pieces sharing a float bounding cell keep independent precise frames")
    func separatedPieces() {
        let world = World()
        world.workerCount = 4
        var def = SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 3, rows: 3, spacing: 0.001))
        def.gravityScale = 0
        def.enableSleep = false
        def.position = Pos(1.0e12, 1.0e12, -1.0e12)
        let a = world.createSoftBody(def)
        def.position.x += 16384
        let b = world.createSoftBody(def)
        let before = [world.softBodyVertexPositions(a), world.softBodyVertexPositions(b)]
        for _ in 0..<5 { world.step(1.0 / 60, subStepCount: 4) }
        for (index, id) in [a, b].enumerated() {
            let after = world.softBodyVertexPositions(id)
            for vertex in after.indices {
                #expect(length(subPos(after[vertex], before[index][vertex])) < 0.00025)
            }
        }
    }

    @Test("teleport and render buffers retain shape detail across huge distances")
    func teleport() {
        let world = World()
        var def = SoftBodyDef(settings: SoftBodySettings.makeCloth(columns: 3, rows: 3, spacing: 0.125))
        def.position = Pos(0, 2, 0)
        let id = world.createSoftBody(def)
        let oldCenter = world.softBodyCenterOfMass(id)
        let offsets = world.softBodyVertexPositions(id).map { subPos($0, oldCenter) }
        let target = Pos(1.0e12 + 0.375, -1.0e12 + 0.125, 1.0e12 + 0.5)
        world.setSoftBodyTransform(id, target, .identity)
        let positions = world.softBodyVertexPositions(id)
        for vertex in positions.indices {
            #expect(length(subPos(positions[vertex], target) - offsets[vertex]) < 0.00025)
        }
        let matched = world.withSoftBodyLocalVertexPositions(id) { origin, vertices in
            vertices.indices.allSatisfy { offsetPos(origin, vertices[$0]) == positions[$0] }
        }
        #expect(matched == true)
    }
}
