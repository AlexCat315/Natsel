import Testing
@testable import Natsel

@Suite("Hull collision")
struct ConvexManifoldTests {
    private func collide(_ hullA: Hull, _ hullB: Hull, _ transform: Transform, _ cache: inout SATCache) -> [LocalManifoldPoint] {
        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 4)
        var manifold = LocalManifold()
        return buffer.withUnsafeMutableBufferPointer { points in
            manifold.points = points.baseAddress
            collideHulls(&manifold, 4, hullA, hullB, transform, &cache)
            // Copy out before the buffer goes away.
            return (0..<Int(manifold.pointCount)).map { points[$0] }
        }
    }

    private func manifoldNormal(_ hullA: Hull, _ hullB: Hull, _ transform: Transform, _ cache: inout SATCache) -> Vec3 {
        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: 4)
        var manifold = LocalManifold()
        buffer.withUnsafeMutableBufferPointer { points in
            manifold.points = points.baseAddress
            collideHulls(&manifold, 4, hullA, hullB, transform, &cache)
        }
        return manifold.normal
    }

    @Test("face contact matches the C reference")
    func faceContact() {
        let hullA = makeBoxHull(1, 1, 1)
        let hullB = makeBoxHull(1, 1, 1)
        var cache = SATCache()

        let normal = manifoldNormal(hullA, hullB, Transform(Vec3(1.5, 0, 0), Quat.identity), &cache)
        #expect(normal.x.bitPattern == 0x3f80_0000)
        #expect(normal.y.bitPattern == 0x8000_0000)
        #expect(normal.z.bitPattern == 0x8000_0000)

        #expect(cache.type == .faceAxisB)
        #expect(cache.separation.bitPattern == 0xbf00_0000)
        #expect(cache.indexA == 0)
        #expect(cache.indexB == 0)
        #expect(cache.hit == 0)

        // Re-run with the cache populated: the cached features should reproduce the contact.
        let points = collide(hullA, hullB, Transform(Vec3(1.5, 0, 0), Quat.identity), &cache)
        #expect(cache.hit == 1)
        #expect(points.count == 4)
    }

    @Test("rotated edge contact matches the C reference bit for bit")
    func rotatedContact() {
        let hullA = makeBoxHull(1, 1, 1)
        let hullB = makeBoxHull(1, 1, 1)
        let q = makeQuatFromAxisAngle(Vec3(0, 1, 0), 0.785398163)
        let xf = Transform(Vec3(1.2, 0.7, 0.3), q)

        var cache = SATCache()
        let normal = manifoldNormal(hullA, hullB, xf, &cache)

        #expect(normal.x.bitPattern == 0x3f80_0000)
        #expect(normal.y.bitPattern == 0x0000_0000)
        #expect(normal.z.bitPattern == 0x0000_0000)

        #expect(cache.type == .faceAxisA)
        #expect(cache.separation.bitPattern == 0xbf9b_6b3c)
        #expect(cache.indexA == 1)
        #expect(cache.indexB == 5)

        var cache2 = SATCache()
        let points = collide(hullA, hullB, xf, &cache2)
        #expect(points.count == 4)

        #expect(points[0].point.x.bitPattern == 0x3f85_5216)
        #expect(points[0].point.y.bitPattern == 0x3f80_0000)
        #expect(points[0].point.z.bitPattern == 0xbf80_0000)
        #expect(points[0].separation.bitPattern == 0x3daa_42d0)

        #expect(points[1].point.x.bitPattern == 0x3f85_5216)
        #expect(points[1].point.y.bitPattern == 0xbe99_999a)
        #expect(points[1].separation.bitPattern == 0x3daa_42c0)

        #expect(points[2].point.x.bitPattern == 0x3ec9_2988)
        #expect(points[2].point.y.bitPattern == 0xbe99_999a)
        #expect(points[2].point.z.bitPattern == 0x3e9b_3868)
        #expect(points[2].separation.bitPattern == 0xbf9b_6b3c)

        #expect(points[3].point.x.bitPattern == 0x3ec9_2988)
        #expect(points[3].point.y.bitPattern == 0x3f7f_ffff)
        #expect(points[3].point.z.bitPattern == 0x3e9b_3868)
    }
}
