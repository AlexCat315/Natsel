import Testing
@testable import Natsel

@Suite("Mesh query repeatability")
struct MeshQueryTests {
    @Test("scaled casts, transformed overlaps, triangle queries, and mover planes are repeatable", arguments: 0..<9)
    func queries(scenario: Int) throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let geometry: TriangleMesh?
            switch scenario / 3 {
            case 0: geometry = createGridMesh(xCount: 5, zCount: 4, cellWidth: 0.75, materialCount: 3, identifyEdges: true)
            case 1: geometry = createWaveMesh(xCount: 5, zCount: 4, cellWidth: 0.75, amplitude: 0.5, rowFrequency: 0.3, columnFrequency: 0.2)
            default: geometry = createTorusMesh(radialResolution: 7, tubularResolution: 9, radius: 1.25, thickness: 0.375)
            }
            let data = try #require(geometry)
            samples.append(UInt64(data.hash))
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
            samples.append(UInt64(casts.value))
            samples.append(UInt64(overlaps.value))
            samples.append(UInt64(queries.value))
            samples.append(UInt64(movers.value))
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}

private struct QueryDigest {
    var value: UInt64 = 14695981039346656037
    mutating func add(_ value: Pos) { add(Float(value.x)); add(Float(value.y)); add(Float(value.z)) }
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
