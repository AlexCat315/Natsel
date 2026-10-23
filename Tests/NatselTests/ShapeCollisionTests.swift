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

    @Test("sphere pair matches the C reference")
    func spheres() {
        let (normal, points) = withManifold(1) { manifold in
            let a = Sphere(center: Vec3(0, 0, 0), radius: 1)
            let b = Sphere(center: Vec3(0, 0, 0), radius: 1)
            collideSpheres(&manifold, 1, a, b, Transform(Vec3(1.5, 0, 0), Quat.identity))
        }

        #expect(normal.x.bitPattern == 0x3f80_0000)
        #expect(normal.y.bitPattern == 0)
        #expect(points.count == 1)
        #expect(points[0].point.x.bitPattern == 0x3f40_0000)
        #expect(points[0].point.y.bitPattern == 0)
        #expect(points[0].separation.bitPattern == 0xbf00_0000)
        #expect(points[0].pair.owner1 == 0 && points[0].pair.index1 == 0)
    }

    @Test("parallel capsules produce two points")
    func capsules() {
        let (normal, points) = withManifold(2) { manifold in
            let a = Capsule(center1: Vec3(0, -1, 0), center2: Vec3(0, 1, 0), radius: 0.5)
            let b = a
            collideCapsules(&manifold, 2, a, b, Transform(Vec3(0.8, 0, 0), Quat.identity))
        }

        #expect(normal.x.bitPattern == 0x3f80_0000)
        #expect(points.count == 2)
        #expect(points[0].point.x.bitPattern == 0x3ecc_cccc)
        #expect(points[0].point.y.bitPattern == 0xbf80_0000)
        #expect(points[0].separation.bitPattern == 0xbe4c_cccc)
        #expect(points[1].point.y.bitPattern == 0x3f80_0000)
        #expect(points[1].pair.index1 == 1)
    }

    @Test("capsule and sphere matches the C reference")
    func capsuleSphere() {
        let (normal, points) = withManifold(1) { manifold in
            let a = Capsule(center1: Vec3(0, 0, 0), center2: Vec3(0, 0, 0), radius: 0.5)
            let b = Sphere(center: Vec3(0, 0, 0), radius: 1)
            collideCapsuleAndSphere(&manifold, 1, a, b, Transform(Vec3(1.2, 0.3, 0), Quat.identity))
        }

        #expect(normal.x.bitPattern == 0x3f78_5b43)
        #expect(normal.y.bitPattern == 0x3e78_5b43)
        #expect(points.count == 1)
        #expect(points[0].point.x.bitPattern == 0x3eb7_0592)
        #expect(points[0].point.y.bitPattern == 0x3db7_0592)
        #expect(points[0].separation.bitPattern == 0xbe86_b0e4)
    }

    @Test("hull and sphere matches the C reference")
    func hullSphere() {
        let hull = makeBoxHull(1, 1, 1)
        var result: (Vec3, [LocalManifoldPoint])!
        result = withManifold(1) { manifold in
            var cache = SimplexCache()
            let sphere = Sphere(center: Vec3(0, 0, 0), radius: 1)
            collideHullAndSphere(&manifold, 1, hull, sphere, Transform(Vec3(1.2, 0.5, 0), Quat.identity), &cache)
        }

        #expect(result.0.x.bitPattern == 0x3f80_0000)
        #expect(result.1.count == 1)
        #expect(result.1[0].point.x.bitPattern == 0x3f19_999a)
        #expect(result.1[0].point.y.bitPattern == 0x3f00_0000)
        #expect(result.1[0].separation.bitPattern == 0xbf4c_cccc)
    }

    @Test("hull and capsule matches the C reference")
    func hullCapsule() {
        let hull = makeBoxHull(1, 1, 1)
        let result = withManifold(2) { manifold in
            var cache = SimplexCache()
            let capsule = Capsule(center1: Vec3(0, 0, 0), center2: Vec3(0, 0, 0), radius: 0.25)
            collideHullAndCapsule(&manifold, 2, hull, capsule, Transform(Vec3(1.1, 0, 0), Quat.identity), &cache)
        }

        #expect(result.0.x.bitPattern == 0x3f80_0000)
        #expect(result.1.count == 2)
        #expect(result.1[0].point.x.bitPattern == 0x3f6c_cccd)
        #expect(result.1[0].separation.bitPattern == 0xbe19_9998)
    }
}
