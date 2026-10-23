import Testing
@testable import Natsel

@Suite("Mesh query C parity")
struct MeshQueryTests {
    @Test("scaled casts, transformed overlaps, triangle queries, and mover planes match C", arguments: 0..<9)
    func queries(scenario: Int) throws {
        let geometry: TriangleMesh?
        switch scenario / 3 {
        case 0: geometry = createGridMesh(xCount: 5, zCount: 4, cellWidth: 0.75, materialCount: 3, identifyEdges: true)
        case 1: geometry = createWaveMesh(xCount: 5, zCount: 4, cellWidth: 0.75, amplitude: 0.5, rowFrequency: 0.3, columnFrequency: 0.2)
        default: geometry = createTorusMesh(radialResolution: 7, tubularResolution: 9, radius: 1.25, thickness: 0.375)
        }
        let data = try #require(geometry)
        let contentHashes: [UInt64] = [0xdf329f8cf1076350, 0x2385a213834e3724, 0xbf54bc4f51333dee]
        #expect(data.hash == contentHashes[scenario / 3])
        let scales = [Vec3.one, Vec3(-2, 0.5, 1.5), Vec3(1, -1, 1)]
        let mesh = Mesh(data: data, scale: scales[scenario % 3])
        let transform = Transform(Vec3(0.125, -0.25, 0.5), makeQuatFromAxisAngle(.axisY, 0.35))
        var casts = QueryDigest()
        var overlaps = QueryDigest()
        var queries = QueryDigest()
        var movers = QueryDigest()
        for i in 0..<80 {
            let x = Float(i % 10) * 0.5 - 2.25
            let z = Float(i % 8) * 0.5 - 1.75
            let y: Float = i < 40 ? 2 : -2
            var points = [Vec3(x, y, z), Vec3(x + 0.125, y + 0.5, z - 0.25)]
            var input = ShapeCastInput()
            input.translation = Vec3(0.125, i < 40 ? -4 : 4, -0.25)
            input.maxFraction = i % 3 == 0 ? 0.5 : 1
            input.canEncroach = i % 2 == 0
            points.withUnsafeBufferPointer { buffer in
                input.proxy = ShapeProxy(points: buffer.baseAddress, count: 2, radius: 0.25)
                casts.add(shapeCastMesh(mesh, input))
            }
            points[0].y = Float(i % 5) * 0.2 - 0.3
            points[1].y = points[0].y + 0.5
            points.withUnsafeBufferPointer { buffer in
                let proxy = ShapeProxy(points: buffer.baseAddress, count: 2, radius: 0.25)
                overlaps.add(overlapMesh(mesh, transform, proxy) ? UInt32(1) : UInt32(0))
            }
            let center = Vec3(x, Float(i % 3) * 0.3 - 0.3, z)
            let extent = Vec3(0.375, 0.2, 0.375)
            var count = 0
            let limit = i % 4 == 0 ? 3 : 100000
            queryMesh(mesh, bounds: AABB(center - extent, center + extent)) { a, b, c, index in
                queries.add(a); queries.add(b); queries.add(c); queries.add(UInt32(bitPattern: index))
                count += 1
                return count < limit
            }
            queries.add(UInt32(count))
            let capsule = Capsule(center1: points[0], center2: points[1], radius: 0.4)
            let planes = collideMoverAndMesh(mesh, capsule, capacity: i % 5)
            movers.add(UInt32(planes.count))
            for plane in planes {
                movers.add(plane.plane.normal); movers.add(plane.plane.offset)
                movers.add(plane.point); movers.add(UInt32(bitPattern: plane.triangleIndex))
                movers.add(UInt32(bitPattern: plane.childIndex)); movers.add(UInt32(bitPattern: plane.materialIndex))
            }
        }
        let expectedCast: [UInt64] = [0x333638b5857671ac, 0x73741419d845a820, 0x53c49eefebe3bcbe, 0xc5e819bd6c24b190, 0x83efee1240c124dc, 0x8d82f55de0671a64, 0xbef56fbe47997b0e, 0x8086cbe908ff897b, 0x3f27c1c2f477c3fc]
        let expectedOverlap: [UInt64] = [0x61df885eb2103125, 0xca403e3d54abd9a5, 0x61df885eb2103125, 0xdaf7068417d8fb45, 0x03dc9a66ec62e625, 0x63b48146837651a5, 0x83afcb28423311a5, 0x779dfdc7ebf86785, 0x83afcb28423311a5]
        let expectedQuery: [UInt64] = [0x8e648ddc8b32f4dc, 0x8fe55f46fe7ecd94, 0x2bf4eb6debe16c9c, 0x0090194a9331fe52, 0x49ca9aac8edf2e3d, 0x1d806c7a732eed5e, 0xcb5c8585caabb840, 0x032287b318ff80b8, 0x259df953f2ad83b1]
        let expectedMover: [UInt64] = [0x171171c8ec224dad, 0x06808d20b64476b5, 0xf05e74aa1eda9c25, 0x900c03af69022cd1, 0x727fde0d79fa6c01, 0x1675a9410ce7c489, 0x5aed87ce97afe9c1, 0xffe59fca9e53f255, 0x4b7cf7e0f3671ea5]
        #expect(casts.value == expectedCast[scenario])
        #expect(overlaps.value == expectedOverlap[scenario])
        #expect(queries.value == expectedQuery[scenario])
        #expect(movers.value == expectedMover[scenario])
    }
}

private struct QueryDigest {
    var value: UInt64 = 14695981039346656037
    mutating func add(_ word: UInt32) {
        for byte in 0..<4 {
            value ^= UInt64((word >> (8 * byte)) & 255)
            value = value &* 1099511628211
        }
    }
    mutating func add(_ scalar: Float) { add(scalar.bitPattern) }
    mutating func add(_ vector: Vec3) { add(vector.x); add(vector.y); add(vector.z) }
    mutating func add(_ out: CastOutput) {
        add(out.normal); add(out.point); add(out.fraction)
        add(UInt32(bitPattern: out.iterations)); add(UInt32(bitPattern: out.triangleIndex))
        add(UInt32(bitPattern: out.childIndex)); add(UInt32(bitPattern: out.materialIndex)); add(out.hit ? UInt32(1) : UInt32(0))
    }
}
