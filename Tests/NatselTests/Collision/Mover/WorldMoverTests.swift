import Testing
@testable import Natsel

@Suite("World mover C parity")
struct WorldMoverTests {
    @Test("mixed shape planes, casts, filters, and large origins match C", arguments: 0..<2)
    func reference(scenario: Int) throws {
        let (world, origin) = try makeReferenceQueryScene(scenario)
        var planes = CollisionReferenceDigest(), casts = CollisionReferenceDigest(), filters = CollisionReferenceDigest()
        for i in 0..<120 {
            let x = Float((i * 7) % 19) * 0.25 - 1
            let z = Float((i * 11) % 17) * 0.25 - 0.75
            let y = Float(i % 9) * 0.25 - 0.5
            var mover = Capsule(center1: Vec3(x, y, z), center2: Vec3(x + 0.1, y + 0.5, z - 0.15), radius: Float(i % 3) * 0.2)
            var filter = QueryFilter.defaultFilter
            filter.maskBits = i % 3 == 0 ? 1 : UInt64.max
            var count = 0
            let limit = i % 4 == 0 ? 1 : 100000
            world.collideMover(origin, mover, filter) { id, results in
                planes.add(UInt32(id.index1)); planes.add(UInt32(results.count))
                for plane in results { planes.add(plane) }
                count += 1
                return count < limit
            }
            planes.add(UInt32(count))
            mover.center1.y = 2
            mover.center2.y = 2.5
            let translation = Vec3(Float(i % 4 - 2) * 1.25, -4, Float(i % 5 - 2) * 0.75)
            let callback: ((ShapeID) -> Bool)? = i % 2 == 0 ? { id in
                filters.add(UInt32(id.index1))
                return id.index1 % 2 == 0
            } : nil
            casts.add(world.castMover(origin, mover, translation, filter, shouldCollide: callback))
        }
        #expect(planes.value == 0xc03a2aa1f9dc48e4)
        #expect(casts.value == 0xfe31bb3eb5168f09)
        #expect(filters.value == [UInt64(0xfb5ce481cec8d1d6), 0x5076a8f43f721a54][scenario])
    }
}
