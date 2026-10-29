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

    @Test("triangle and sphere is repeatable")
    func triangleSphere() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let result = withManifold(1) { manifold in
                let sphere = Sphere(center: Vec3(0.2, 0.5, 0.2), radius: 0.5)
                tri.withUnsafeBufferPointer {
                    collideTriangleAndSphere(&manifold, 1, $0.baseAddress!, sphere)
                }
            }

            samples.append(UInt64(result.0.x.bitPattern))
            samples.append(UInt64(result.0.y.bitPattern))
            samples.append(UInt64(result.0.z.bitPattern))
            #expect(result.1 == 1)
            #expect(result.2.count == 1)
            samples.append(UInt64(result.2[0].point.x.bitPattern))
            #expect(result.2[0].point.y.bitPattern == 0)
            #expect(result.2[0].separation.bitPattern == 0)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("triangle and degenerate capsule is repeatable")
    func triangleCapsule() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let result = withManifold(2) { manifold in
                var cache = SimplexCache()
                let capsule = Capsule(center1: Vec3(0.2, 0.25, 0.2), center2: Vec3(0.2, 0.25, 0.2), radius: 0.3)
                tri.withUnsafeBufferPointer {
                    collideTriangleAndCapsule(&manifold, 2, $0.baseAddress!, capsule, &cache)
                }
            }

            samples.append(UInt64(result.0.y.bitPattern))
            #expect(result.1 == 1)
            #expect(result.2.count == 2)
            samples.append(UInt64(result.2[0].point.x.bitPattern))
            samples.append(UInt64(result.2[0].point.y.bitPattern))
            samples.append(UInt64(result.2[0].separation.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("triangle and horizontal capsule clips both ends")
    func triangleCapsule2() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let result = withManifold(2) { manifold in
                var cache = SimplexCache()
                let capsule = Capsule(center1: Vec3(0.1, 0.1, 0.1), center2: Vec3(0.6, 0.1, 0.1), radius: 0.2)
                tri.withUnsafeBufferPointer {
                    collideTriangleAndCapsule(&manifold, 2, $0.baseAddress!, capsule, &cache)
                }
            }

            #expect(result.2.count == 2)
            samples.append(UInt64(result.2[0].point.x.bitPattern))
            samples.append(UInt64(result.2[0].point.y.bitPattern))
            samples.append(UInt64(result.2[1].point.x.bitPattern))
            samples.append(UInt64(result.2[1].separation.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("triangle above a box is a separating axis")
    func triangleHullSeparated() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
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
            samples.append(UInt64(cache.separation.bitPattern))
            #expect(cache.hit == 0)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("triangle intersecting a box is repeatable")
    func triangleHullContact() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let hull = makeBoxHull(1, 1, 1)
            let triH: [Vec3] = [Vec3(-0.4, 1.05, -0.4), Vec3(0.4, 1.0, -0.3), Vec3(0.3, 1.15, 0.4)]

            var cache = SATCache()
            let result = withManifold(4) { manifold in
                collideTriangleAndHull(
                    &manifold, 4, triH[0], triH[1], triH[2], 0, hull, &cache, true
                )
            }

            samples.append(UInt64(result.0.x.bitPattern))
            samples.append(UInt64(result.0.y.bitPattern))
            samples.append(UInt64(result.0.z.bitPattern))
            #expect(result.1 == 2)
            #expect(result.2.count == 3)

            samples.append(UInt64(result.2[0].point.x.bitPattern))
            samples.append(UInt64(result.2[0].point.y.bitPattern))
            samples.append(UInt64(result.2[0].point.z.bitPattern))
            samples.append(UInt64(result.2[0].separation.bitPattern))

            samples.append(UInt64(result.2[1].point.x.bitPattern))
            samples.append(UInt64(result.2[1].point.z.bitPattern))
            #expect(result.2[1].separation.bitPattern == 0)

            samples.append(UInt64(result.2[2].point.x.bitPattern))
            samples.append(UInt64(result.2[2].point.z.bitPattern))
            samples.append(UInt64(result.2[2].separation.bitPattern))

            #expect(cache.type == .faceAxisB)
            #expect(cache.indexA == 1)
            #expect(cache.indexB == 3)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
