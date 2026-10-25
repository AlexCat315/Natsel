import Testing
@testable import Natsel

@Suite("Soft body ray acceleration")
struct RayAccelerationTests {
    @Test("BVH rays match a linear scan including shared triangle edges")
    func oracle() {
        let world = World()
        world.gravity = .zero
        let settings = SoftBodySettings.makeCloth(columns: 24, rows: 24, spacing: 0.125)
        var def = SoftBodyDef(settings: settings, position: Pos(1.0e12 + 0.125, 0, -1.0e12))
        def.facesDoubleSided = true
        let id = world.createSoftBody(def)
        for i in 0..<160 {
            let origin = Pos(1.0e12 + Double(i % 20) * 0.125 - 1, 2, -1.0e12 + Double(i / 20) * 0.125 - 0.5)
            world.enableSoftBodyQueryAcceleration = false
            let expected = world.castRaySoftBody(id, origin, Vec3(0, -4, 0))
            world.enableSoftBodyQueryAcceleration = true
            let actual = world.castRaySoftBody(id, origin, Vec3(0, -4, 0))
            #expect(actual.hit == expected.hit)
            #expect(actual.faceIndex == expected.faceIndex)
            #expect(actual.fraction == expected.fraction)
            #expect(actual.normal == expected.normal)
            #expect(actual.point == expected.point)
        }
    }

    @Test("vertex edits, rollback, and slot reuse invalidate the face cache")
    func invalidation() {
        let world = World()
        let settings = SoftBodySettings.makeCloth(columns: 6, rows: 6, spacing: 0.25)
        var def = SoftBodyDef(settings: settings)
        def.facesDoubleSided = true
        let id = world.createSoftBody(def)
        let origin = Pos(0, 3, 0)
        let first = world.castRaySoftBody(id, origin, Vec3(0, -6, 0))
        #expect(first.hit)
        let cache = world.softBodyRayCaches[id.index1 - 1]
        let saved = world.saveState()
        for i in 0..<world.softBodyVertexCount(id) {
            let p = world.softBodyVertexPosition(id, i)!
            world.setSoftBodyVertexPosition(id, i, Pos(p.x, 1, p.z))
        }
        let moved = world.castRayClosestSoftBody(origin, Vec3(0, -6, 0))
        #expect(moved.hit)
        #expect(abs(moved.point.y - 1) < 0.00001)
        #expect(world.softBodyRayCaches[id.index1 - 1] !== cache)
        #expect(world.restoreState(saved))
        #expect(world.softBodyRayCaches.isEmpty)
        #expect(world.castRaySoftBody(id, origin, Vec3(0, -6, 0)).point == first.point)
        world.destroySoftBody(id)
        #expect(world.softBodyRayCaches.isEmpty)
        def.position = Pos(0, -1, 0)
        let reused = world.createSoftBody(def)
        #expect(!world.softBodyIsValid(id))
        #expect(abs(world.castRaySoftBody(reused, origin, Vec3(0, -6, 0)).point.y + 1) < 0.00001)
    }
}
