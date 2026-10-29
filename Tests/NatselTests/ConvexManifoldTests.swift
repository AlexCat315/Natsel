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

    @Test("face contact is repeatable")
    func faceContact() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let hullA = makeBoxHull(1, 1, 1)
            let hullB = makeBoxHull(1, 1, 1)
            var cache = SATCache()

            let normal = manifoldNormal(hullA, hullB, Transform(Vec3(1.5, 0, 0), Quat.identity), &cache)
            samples.append(UInt64(normal.x.bitPattern))
            samples.append(UInt64(normal.y.bitPattern))
            samples.append(UInt64(normal.z.bitPattern))

            #expect(cache.type == .faceAxisB)
            samples.append(UInt64(cache.separation.bitPattern))
            #expect(cache.indexA == 0)
            #expect(cache.indexB == 0)
            #expect(cache.hit == 0)

            // Re-run with the cache populated: the cached features should reproduce the contact.
            let points = collide(hullA, hullB, Transform(Vec3(1.5, 0, 0), Quat.identity), &cache)
            #expect(cache.hit == 1)
            #expect(points.count == 4)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("rotated edge contact is repeatable")
    func rotatedContact() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let hullA = makeBoxHull(1, 1, 1)
            let hullB = makeBoxHull(1, 1, 1)
            let q = makeQuatFromAxisAngle(Vec3(0, 1, 0), 0.785398163)
            let xf = Transform(Vec3(1.2, 0.7, 0.3), q)

            var cache = SATCache()
            let normal = manifoldNormal(hullA, hullB, xf, &cache)

            samples.append(UInt64(normal.x.bitPattern))
            #expect(normal.y.bitPattern == 0x0000_0000)
            #expect(normal.z.bitPattern == 0x0000_0000)

            #expect(cache.type == .faceAxisA)
            samples.append(UInt64(cache.separation.bitPattern))
            #expect(cache.indexA == 1)
            #expect(cache.indexB == 5)

            var cache2 = SATCache()
            let points = collide(hullA, hullB, xf, &cache2)
            #expect(points.count == 4)

            samples.append(UInt64(points[0].point.x.bitPattern))
            samples.append(UInt64(points[0].point.y.bitPattern))
            samples.append(UInt64(points[0].point.z.bitPattern))
            samples.append(UInt64(points[0].separation.bitPattern))

            samples.append(UInt64(points[1].point.x.bitPattern))
            samples.append(UInt64(points[1].point.y.bitPattern))
            samples.append(UInt64(points[1].separation.bitPattern))

            samples.append(UInt64(points[2].point.x.bitPattern))
            samples.append(UInt64(points[2].point.y.bitPattern))
            samples.append(UInt64(points[2].point.z.bitPattern))
            samples.append(UInt64(points[2].separation.bitPattern))

            samples.append(UInt64(points[3].point.x.bitPattern))
            samples.append(UInt64(points[3].point.y.bitPattern))
            samples.append(UInt64(points[3].point.z.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
