import Testing
@testable import Natsel

@Suite("Mesh ray repeatability")
struct MeshRayTests {
    @Test("grid storage and 240 scaled ray results are repeatable", arguments: 0..<3)
    func rayRepeatability(scenario: Int) throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let data = try #require(createGridMesh(xCount: 5, zCount: 4, cellWidth: 0.75,
                                                  materialCount: 3, identifyEdges: true))
            samples.append(UInt64(data.hash))
            #expect(data.byteCount == 1624)
            let scales = [Vec3.one, Vec3(-2, 0.5, 1.5), Vec3(1, -1, 1)]
            let mesh = Mesh(data: data, scale: scales[scenario])
            var hash: UInt64 = 14695981039346656037
            for i in 0..<80 {
                let ray = RayCastInput(origin: Vec3(Float(i % 10) * 0.5 - 2.25, i < 40 ? 2 : -2, Float(i % 8) * 0.5 - 1.75),
                                       translation: Vec3(0.125, i < 40 ? -4 : 4, -0.25),
                                       maxFraction: i % 3 == 0 ? 0.5 : 1)
                let out = rayCastMesh(mesh, ray)
                for word in [out.normal.x.bitPattern, out.normal.y.bitPattern, out.normal.z.bitPattern,
                             out.point.x.bitPattern, out.point.y.bitPattern, out.point.z.bitPattern,
                             out.fraction.bitPattern, UInt32(bitPattern: out.triangleIndex),
                             UInt32(bitPattern: out.materialIndex), out.hit ? 1 : 0] {
                    for byte in 0..<4 {
                        hash ^= UInt64((word >> (8 * byte)) & 255)
                        hash = hash &* 1099511628211
                    }
                }
            }
            samples.append(UInt64(hash))
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
