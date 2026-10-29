import Testing
@testable import Natsel

@Suite("Mesh storage repeatability")
struct MeshStorageTests {
    @Test("complete zeroed mesh blob hashes are repeatable", arguments: 0..<3)
    func storageRepeatability(scenario: Int) throws {
        func snapshot() throws -> [UInt64] {
            var samples: [UInt64] = []
            let vertices = [Vec3.zero, Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(1, 0, 1),
                            Vec3(0, 1, 0), Vec3.zero, Vec3(2, 0, 0)]
            let indices: [Int32] = [0,2,1, 1,2,3, 0,1,4, 5,2,1, 0,0,1, 0,1,5, 0,1,6]
            var options = MeshBuildOptions()
            options.identifyEdges = scenario > 0
            options.weldTolerance = scenario == 2 ? 0.001 : 0
            options.clockwiseWinding = scenario == 2
            let mesh = try #require(TriangleMesh(vertices: vertices, indices: indices,
                                                materials: [0,1,2,3,4,5,6], options: options))
            samples.append(UInt64(mesh.hash))
            #expect(mesh.byteCount == (scenario == 2 ? 264 : 280))
            #expect(mesh.triangleCount == 4)
            #expect(mesh.vertexCount == (scenario == 2 ? 6 : 7))
            #expect(mesh.surfaceArea.bitPattern == Float(2).bitPattern)
            #expect(mesh.degenerateTriangleCount == (scenario == 2 ? 1 : 2))
            #expect(mesh.header.materialCount == 4)
            let expectedFlags: [[UInt8]] = [[0,0,0,0], [4,0,1,0], [0x44,0,0,0x44]]
            #expect(Array(mesh.edgeFlags) == expectedFlags[scenario])
            #expect(Array(mesh.materialIndices) == [0,1,2,3])
            let again = try #require(TriangleMesh(vertices: vertices, indices: indices,
                                                 materials: [0,1,2,3,4,5,6], options: options))
            #expect(mesh.hash == again.hash)
            return samples
        }

        let first = try snapshot()
        let replay = try snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }

    @Test("invalid and fully degenerate input does not produce geometry")
    func rejectedInputs() {
        let vertices = [Vec3.zero, Vec3(1, 0, 0), Vec3(2, 0, 0)]
        #expect(TriangleMesh(vertices: vertices, indices: [0,1,2]) == nil)
        #expect(TriangleMesh(vertices: vertices, indices: [0,1,3]) == nil)
        #expect(TriangleMesh(vertices: vertices, indices: [-1,1,2]) == nil)
        #expect(TriangleMesh(vertices: vertices, indices: [0,1]) == nil)
    }
}
