import Testing
@testable import Natsel

@Suite("Mesh primitive repeatability")
struct MeshGeneratorTests {
    @Test("box, hollow box, and platform produce repeatable storage hashes", arguments: 0..<3)
    func primitives(scenario: Int) throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let center = Vec3(0.125, -0.25, 0.5)
            let extent = Vec3(1.25, 0.75, 2)
            let geometry: TriangleMesh?
            switch scenario {
            case 0: geometry = createBoxMesh(center: center, extent: extent, identifyEdges: true)
            case 1: geometry = createHollowBoxMesh(center: center, extent: extent)
            default: geometry = createPlatformMesh(center: center, height: 1.5, topWidth: 2, bottomWidth: 3)
            }
            let mesh = try #require(geometry)
            samples.append(UInt64(mesh.hash))
            #expect(mesh.byteCount == 656)
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
