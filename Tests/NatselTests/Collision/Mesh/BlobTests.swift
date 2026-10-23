import Testing
@testable import Natsel

@Suite("Mesh storage C parity")
struct MeshStorageTests {
    @Test("complete zeroed mesh blob hashes match C", arguments: 0..<3)
    func storageParity(scenario: Int) throws {
        let vertices = [Vec3.zero, Vec3(1, 0, 0), Vec3(0, 0, 1), Vec3(1, 0, 1),
                        Vec3(0, 1, 0), Vec3.zero, Vec3(2, 0, 0)]
        let indices: [Int32] = [0,2,1, 1,2,3, 0,1,4, 5,2,1, 0,0,1, 0,1,5, 0,1,6]
        var options = MeshBuildOptions()
        options.identifyEdges = scenario > 0
        options.weldTolerance = scenario == 2 ? 0.001 : 0
        options.clockwiseWinding = scenario == 2
        let mesh = try #require(TriangleMesh(vertices: vertices, indices: indices,
                                            materials: [0,1,2,3,4,5,6], options: options))
        let expected: [UInt64] = [0x5d91a1abd310a8fb, 0x96d3eabacf760139, 0x5fb52a8facd95037]
        #expect(mesh.hash == expected[scenario])
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
