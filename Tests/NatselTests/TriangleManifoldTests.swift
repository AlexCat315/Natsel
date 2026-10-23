import Testing
@testable import Natsel

@Suite("Triangle collision")
struct TriangleManifoldTests {
    private let tri: [Vec3] = [Vec3(0, 0, 0), Vec3(0, 0, 1), Vec3(1, 0, 0)]

    private func withManifold(_ capacity: Int32, _ body: (inout LocalManifold) -> Void) -> (Vec3, Int32, [LocalManifoldPoint]) {
        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: Int(capacity))
        var manifold = LocalManifold()
        buffer.withUnsafeMutableBufferPointer { points in
            manifold.points = points.baseAddress
            body(&manifold)
        }
        let count = Int(manifold.pointCount)
        return (manifold.normal, Int32(manifold.feature.rawValue), Array(buffer[0..<count]))
    }

    @Test("triangle and sphere matches the C reference")
    func triangleSphere() {
        let result = withManifold(1) { manifold in
            let sphere = Sphere(center: Vec3(0.2, 0.5, 0.2), radius: 0.5)
            tri.withUnsafeBufferPointer {
                collideTriangleAndSphere(&manifold, 1, $0.baseAddress!, sphere)
            }
        }

        #expect(result.0.x.bitPattern == 0x3300_0000)
        #expect(result.0.y.bitPattern == 0x3f80_0000)
        #expect(result.0.z.bitPattern == 0x3300_0000)
        #expect(result.1 == 1)
        #expect(result.2.count == 1)
        #expect(result.2[0].point.x.bitPattern == 0x3e4c_cccc)
        #expect(result.2[0].point.y.bitPattern == 0)
        #expect(result.2[0].separation.bitPattern == 0)
    }

    @Test("triangle and degenerate capsule matches the C reference")
    func triangleCapsule() {
        let result = withManifold(2) { manifold in
            var cache = SimplexCache()
            let capsule = Capsule(center1: Vec3(0.2, 0.25, 0.2), center2: Vec3(0.2, 0.25, 0.2), radius: 0.3)
            tri.withUnsafeBufferPointer {
                collideTriangleAndCapsule(&manifold, 2, $0.baseAddress!, capsule, &cache)
            }
        }

        #expect(result.0.y.bitPattern == 0x3f80_0000)
        #expect(result.1 == 1)
        #expect(result.2.count == 2)
        #expect(result.2[0].point.x.bitPattern == 0x3e4c_cccd)
        #expect(result.2[0].point.y.bitPattern == 0xbccc_ccd0)
        #expect(result.2[0].separation.bitPattern == 0xbd4c_ccd0)
    }

    @Test("triangle and horizontal capsule clips both ends")
    func triangleCapsule2() {
        let result = withManifold(2) { manifold in
            var cache = SimplexCache()
            let capsule = Capsule(center1: Vec3(0.1, 0.1, 0.1), center2: Vec3(0.6, 0.1, 0.1), radius: 0.2)
            tri.withUnsafeBufferPointer {
                collideTriangleAndCapsule(&manifold, 2, $0.baseAddress!, capsule, &cache)
            }
        }

        #expect(result.2.count == 2)
        #expect(result.2[0].point.x.bitPattern == 0x3dcc_cccd)
        #expect(result.2[0].point.y.bitPattern == 0xbd4c_ccce)
        #expect(result.2[1].point.x.bitPattern == 0x3f19_999a)
        #expect(result.2[1].separation.bitPattern == 0xbdcc_cccd)
    }

    @Test("triangle above a box is a separating axis")
    func triangleHullSeparated() {
        let hull = makeBoxHull(1, 1, 1)
        let triH: [Vec3] = [Vec3(-0.5, 1.2, -0.5), Vec3(0.5, 1.2, -0.5), Vec3(0, 1.2, 0.5)]

        var cache = SATCache()
        let result = withManifold(4) { manifold in
            collideTriangleAndHull(
                &manifold, 4, triH[0], triH[1], triH[2], 0, hull, &cache, true
            )
        }

        #expect(result.2.count == 0)
        #expect(cache.type == .faceAxisA)
        #expect(cache.separation.bitPattern == 0x3e4c_ccd0)
        #expect(cache.hit == 0)
    }

    @Test("triangle intersecting a box matches the C reference")
    func triangleHullContact() {
        let hull = makeBoxHull(1, 1, 1)
        let triH: [Vec3] = [Vec3(-0.4, 1.05, -0.4), Vec3(0.4, 1.0, -0.3), Vec3(0.3, 1.15, 0.4)]

        var cache = SATCache()
        let result = withManifold(4) { manifold in
            collideTriangleAndHull(
                &manifold, 4, triH[0], triH[1], triH[2], 0, hull, &cache, true
            )
        }

        #expect(result.0.x.bitPattern == 0x8000_0000)
        #expect(result.0.y.bitPattern == 0xbf80_0000)
        #expect(result.0.z.bitPattern == 0x8000_0000)
        #expect(result.1 == 2)
        #expect(result.2.count == 3)

        #expect(result.2[0].point.x.bitPattern == 0xbecc_cccd)
        #expect(result.2[0].point.y.bitPattern == 0x3f80_0000)
        #expect(result.2[0].point.z.bitPattern == 0xbecc_cccd)
        #expect(result.2[0].separation.bitPattern == 0x3d4c_ccc0)

        #expect(result.2[1].point.x.bitPattern == 0x3ecc_cccd)
        #expect(result.2[1].point.z.bitPattern == 0xbe99_999a)
        #expect(result.2[1].separation.bitPattern == 0)

        #expect(result.2[2].point.x.bitPattern == 0x3e99_999a)
        #expect(result.2[2].point.z.bitPattern == 0x3ecc_cccd)
        #expect(result.2[2].separation.bitPattern == 0x3e19_9998)

        #expect(cache.type == .faceAxisB)
        #expect(cache.indexA == 1)
        #expect(cache.indexB == 3)
    }
}
