import Testing
@testable import Natsel

/// A byte-exact content hash over the emitted hull arrays. This mirrors the C probe so the
/// vertex/edge/face ordering can be compared directly.
private func contentHash(_ hull: Hull) -> UInt64 {
    func fnv(_ h: UInt64, _ bytes: UnsafeRawPointer, _ n: Int) -> UInt64 {
        var h = h
        let p = bytes.assumingMemoryBound(to: UInt8.self)
        for i in 0..<n {
            h ^= UInt64(p[i])
            h = h &* 1_099_511_628_211
        }
        return h
    }

    var h: UInt64 = 14_695_981_039_346_656_037

    var vertexCount = hull.vertexCount
    var edgeCount = hull.halfEdgeCount
    var faceCount = hull.faceCount
    h = fnv(h, &vertexCount, 4)
    h = fnv(h, &edgeCount, 4)
    h = fnv(h, &faceCount, 4)

    let points = hull.points!
    for i in 0..<Int(hull.vertexCount) {
        h = fnv(h, points + i, 12)
    }

    let edges = hull.edges!
    for i in 0..<Int(hull.halfEdgeCount) {
        h = fnv(h, edges + i, 4)
    }

    let planes = hull.planes!
    for i in 0..<Int(hull.faceCount) {
        h = fnv(h, planes + i, 16)
    }

    var volume = hull.volume
    var surfaceArea = hull.surfaceArea
    var innerRadius = hull.innerRadius
    var center = hull.center
    var inertia = hull.centralInertia
    h = fnv(h, &volume, 4)
    h = fnv(h, &surfaceArea, 4)
    h = fnv(h, &innerRadius, 4)
    h = fnv(h, &center, 12)
    h = fnv(h, &inertia, 36)

    return h
}

@Suite("Hull builder")
struct HullBuilderTests {
    @Test("cube hull matches the C builder")
    func cube() {
        let c: Float = 1
        let pts: [Vec3] = [
            Vec3(-c, -c, -c), Vec3(c, -c, c), Vec3(c, c, -c), Vec3(-c, c, c),
            Vec3(c, -c, -c), Vec3(-c, c, -c), Vec3(-c, -c, c), Vec3(c, c, c),
        ]

        let hull = createHull(pts, maxVertexCount: 8)
        #expect(hull != nil)
        #expect(hull!.vertexCount == 8)
        #expect(hull!.halfEdgeCount == 24)
        #expect(hull!.faceCount == 6)
        #expect(isValidHull(hull!))
        #expect(contentHash(hull!) == 0xdb56_b283_8b02_bba2)
    }

    @Test("icosahedron hull matches the C builder")
    func icosahedron() {
        let t = (1.0 + Float(5).squareRoot()) / 2.0
        let pts: [Vec3] = [
            Vec3(-1, t, 0), Vec3(1, t, 0), Vec3(-1, -t, 0), Vec3(1, -t, 0),
            Vec3(0, -1, t), Vec3(0, 1, t), Vec3(0, -1, -t), Vec3(0, 1, -t),
            Vec3(t, 0, -1), Vec3(t, 0, 1), Vec3(-t, 0, -1), Vec3(-t, 0, 1),
        ]

        let hull = createHull(pts, maxVertexCount: 12)
        #expect(hull != nil)
        #expect(hull!.vertexCount == 12)
        #expect(hull!.halfEdgeCount == 60)
        #expect(hull!.faceCount == 20)
        #expect(isValidHull(hull!))
        #expect(contentHash(hull!) == 0x1e84_b5f8_5c2d_29ca)
    }

    @Test("random cloud hull matches the C builder")
    func cloud() {
        var pts: [Vec3] = []
        var s: UInt32 = 7
        func next() -> Float {
            s = s &* 2_147_001_325 &+ 715_136_305
            return Float(s & 0xffff) / 65535.0 - 0.5
        }
        for _ in 0..<40 {
            pts.append(Vec3(next(), next(), next()))
        }

        let hull = createHull(pts, maxVertexCount: 40)
        #expect(hull != nil)
        #expect(hull!.vertexCount == 19)
        #expect(hull!.halfEdgeCount == 102)
        #expect(hull!.faceCount == 34)
        #expect(isValidHull(hull!))
        #expect(contentHash(hull!) == 0x87c1_5401_bd9e_44d4)
    }

    @Test("generated primitive hulls match the C builder")
    func primitives() {
        let cylinder = createCylinder(2, 1, 0, 8)
        #expect(cylinder != nil)
        #expect(cylinder!.vertexCount == 16)
        #expect(cylinder!.halfEdgeCount == 48)
        #expect(cylinder!.faceCount == 10)
        #expect(contentHash(cylinder!) == 0x7de7_f150_d047_456a)

        let cone = createCone(2, 1, 0.5, 8)
        #expect(cone != nil)
        #expect(cone!.vertexCount == 16)
        #expect(cone!.halfEdgeCount == 48)
        #expect(cone!.faceCount == 10)
        #expect(contentHash(cone!) == 0xc6e5_9883_1b82_4dac)

        let rock = createRock(1)
        #expect(rock != nil)
        #expect(rock!.vertexCount == 10)
        #expect(rock!.halfEdgeCount == 48)
        #expect(rock!.faceCount == 16)
        #expect(contentHash(rock!) == 0xa9c6_a42d_3728_5ff4)

        let complex = createComplexHull(1)
        #expect(complex != nil)
        #expect(complex!.vertexCount == 32)
        #expect(complex!.halfEdgeCount == 178)
        #expect(complex!.faceCount == 59)
        #expect(contentHash(complex!) == 0x9db2_124d_d887_ad73)
    }
}

@Suite("Hull clone and transform")
struct HullCloneTests {
    @Test("clone and transform matches the C reference")
    func cloneTransform() {
        let base = createCylinder(2, 1, 0, 6)!
        let xf = Transform(Vec3(1, 2, 3), makeQuatFromAxisAngle(Vec3(0, 0, 1), 0.5))

        let positive = cloneAndTransformHull(base, xf, Vec3(1, 2, 0.5))
        #expect(positive != nil)
        #expect(positive!.vertexCount == 12)
        #expect(positive!.halfEdgeCount == 36)
        #expect(positive!.faceCount == 8)
        #expect(isValidHull(positive!))
        #expect(contentHash(positive!) == 0x039e_195e_6d5d_4356)

        let negative = cloneAndTransformHull(base, xf, Vec3(1, 2, -0.5))
        #expect(negative != nil)
        #expect(negative!.vertexCount == 12)
        #expect(negative!.halfEdgeCount == 36)
        #expect(negative!.faceCount == 8)
        #expect(isValidHull(negative!))
        #expect(contentHash(negative!) == 0x0eb3_2d83_7f6c_8553)
    }
}
