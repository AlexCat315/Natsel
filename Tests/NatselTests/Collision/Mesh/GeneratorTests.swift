import Testing
@testable import Natsel

@Suite("Mesh primitive C parity")
struct MeshGeneratorTests {
    @Test("box, hollow box, and platform match C storage hashes", arguments: 0..<3)
    func primitives(scenario: Int) throws {
        let center = Vec3(0.125, -0.25, 0.5)
        let extent = Vec3(1.25, 0.75, 2)
        let geometry: TriangleMesh?
        switch scenario {
        case 0: geometry = createBoxMesh(center: center, extent: extent, identifyEdges: true)
        case 1: geometry = createHollowBoxMesh(center: center, extent: extent)
        default: geometry = createPlatformMesh(center: center, height: 1.5, topWidth: 2, bottomWidth: 3)
        }
        let mesh = try #require(geometry)
        let hashes: [UInt64] = [0xb17b0b67c676fa07, 0xaaff30fbdf08fbee, 0xaa445a0e1aa6ea86]
        #expect(mesh.hash == hashes[scenario])
        #expect(mesh.byteCount == 656)
    }
}
