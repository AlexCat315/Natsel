import Testing
@testable import Natsel

@Suite("Primitive collisions")
struct ShapeCollisionTests {
    private func withManifold(_ capacity: Int32, _ body: (inout LocalManifold) -> Void) -> (Vec3, [LocalManifoldPoint]) {
        var buffer = [LocalManifoldPoint](repeating: LocalManifoldPoint(), count: Int(capacity))
        var manifold = LocalManifold()
        buffer.withUnsafeMutableBufferPointer { points in
            manifold.points = points.baseAddress
            body(&manifold)
        }
        let count = Int(manifold.pointCount)
        return (manifold.normal, Array(buffer[0..<count]))
    }

    @Test("sphere pair is repeatable")
    func spheres() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let (normal, points) = withManifold(1) { manifold in
                let a = Sphere(center: Vec3(0, 0, 0), radius: 1)
                let b = Sphere(center: Vec3(0, 0, 0), radius: 1)
                collideSpheres(&manifold, 1, a, b, Transform(Vec3(1.5, 0, 0), Quat.identity))
            }

            samples.append(UInt64(normal.x.bitPattern))
            #expect(normal.y.bitPattern == 0)
            #expect(points.count == 1)
            samples.append(UInt64(points[0].point.x.bitPattern))
            #expect(points[0].point.y.bitPattern == 0)
            samples.append(UInt64(points[0].separation.bitPattern))
            #expect(points[0].pair.owner1 == 0 && points[0].pair.index1 == 0)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("parallel capsules produce two points")
    func capsules() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let (normal, points) = withManifold(2) { manifold in
                let a = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
                let b = a
                collideCapsules(&manifold, 2, a, b, Transform(Vec3(0.8, 0, 0), Quat.identity))
            }

            samples.append(UInt64(normal.x.bitPattern))
            #expect(points.count == 2)
            samples.append(UInt64(points[0].point.x.bitPattern))
            samples.append(UInt64(points[0].point.y.bitPattern))
            samples.append(UInt64(points[0].separation.bitPattern))
            samples.append(UInt64(points[1].point.y.bitPattern))
            #expect(points[1].pair.index1 == 1)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("capsule and sphere is repeatable")
    func capsuleSphere() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let (normal, points) = withManifold(1) { manifold in
                let a = Capsule(center1: Vec3(0, 0, 0), center2: Vec3(0, 0, 0), radius: 0.5)
                let b = Sphere(center: Vec3(0, 0, 0), radius: 1)
                collideCapsuleAndSphere(&manifold, 1, a, b, Transform(Vec3(1.2, 0.3, 0), Quat.identity))
            }

            samples.append(UInt64(normal.x.bitPattern))
            samples.append(UInt64(normal.y.bitPattern))
            #expect(points.count == 1)
            samples.append(UInt64(points[0].point.x.bitPattern))
            samples.append(UInt64(points[0].point.y.bitPattern))
            samples.append(UInt64(points[0].separation.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("hull and sphere is repeatable")
    func hullSphere() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let hull = makeBoxHull(1, 1, 1)
            var result: (Vec3, [LocalManifoldPoint])!
            result = withManifold(1) { manifold in
                var cache = SimplexCache()
                let sphere = Sphere(center: Vec3(0, 0, 0), radius: 1)
                collideHullAndSphere(&manifold, 1, hull, sphere, Transform(Vec3(1.2, 0.5, 0), Quat.identity), &cache)
            }

            samples.append(UInt64(result.0.x.bitPattern))
            #expect(result.1.count == 1)
            samples.append(UInt64(result.1[0].point.x.bitPattern))
            samples.append(UInt64(result.1[0].point.y.bitPattern))
            samples.append(UInt64(result.1[0].separation.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("hull and capsule is repeatable")
    func hullCapsule() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let hull = makeBoxHull(1, 1, 1)
            let result = withManifold(2) { manifold in
                var cache = SimplexCache()
                let capsule = Capsule(center1: Vec3(0, 0, 0), center2: Vec3(0, 0, 0), radius: 0.25)
                collideHullAndCapsule(&manifold, 2, hull, capsule, Transform(Vec3(1.1, 0, 0), Quat.identity), &cache)
            }

            samples.append(UInt64(result.0.x.bitPattern))
            #expect(result.1.count == 2)
            samples.append(UInt64(result.1[0].point.x.bitPattern))
            samples.append(UInt64(result.1[0].separation.bitPattern))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
