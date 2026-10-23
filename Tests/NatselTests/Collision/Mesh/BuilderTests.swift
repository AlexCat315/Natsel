import Testing
@testable import Natsel

@Suite("Mesh builder C parity")
struct MeshBuilderTests {
    @Test("SAH, median, and degenerate centroid fallback match the C tree and permutation",
          arguments: 0..<4)
    func treeParity(scenario: Int) {
        // Expected values are emitted by Tools/Reference/mesh_builder.c, which calls mesh.c directly.
        let counts = [37, 37, 7, 17]
        let nodeCounts = [21, 25, 1, 5]
        let heights = [5, 5, 0, 3]
        let digests: [UInt64] = [0x48a1f637da16c512, 0xeec2e8f28a450a6c, 0xadea3b4159cceae5, 0x4d1354009fd2268e]
        var primitives = (0..<counts[scenario]).map { i in
            let center = scenario < 2
                ? Vec3(Float((i * 17) % 19) * 0.25 - 2, Float((i * 7) % 13) * 0.5, Float((i * 3) % 11) * 0.125)
                : Vec3.zero
            let extent = Vec3(0.125, 0.25, 0.0625)
            return MeshPrimitive(bounds: AABB(center - extent, center + extent), center: center, triangleIndex: i)
        }
        let tree = MeshTreeBuild(primitives: &primitives, useMedianSplit: scenario == 1)
        #expect(tree.nodes.count == nodeCounts[scenario])
        #expect(tree.height == heights[scenario])
        var digest: UInt64 = 14695981039346656037
        for primitive in primitives { mix(&digest, UInt32(primitive.triangleIndex)) }
        for node in tree.nodes {
            mix(&digest, node.lowerBound.x.bitPattern)
            mix(&digest, node.lowerBound.y.bitPattern)
            mix(&digest, node.lowerBound.z.bitPattern)
            mix(&digest, node.data)
            mix(&digest, node.upperBound.x.bitPattern)
            mix(&digest, node.upperBound.y.bitPattern)
            mix(&digest, node.upperBound.z.bitPattern)
            mix(&digest, node.triangleOffset)
        }
        #expect(digest == digests[scenario])
    }

    @Test("welding preserves negative cells, neighbor priority, tolerance, and non-transitive duplicates")
    func weldingParity() {
        let source = [Vec3(-0.375, 0, 0), Vec3(0.375, 0, 0), Vec3.zero,
                      Vec3(-0.75, 0, 0), Vec3(0.75, 0, 0), Vec3(0.125, 0.5, -0.5),
                      Vec3(0.25, 0.625, -0.625), Vec3(0.375, 0.75, -0.75)]
        let welded = weldMeshVertices(source, indices: Array(0...7), tolerance: 0.5)
        #expect(welded.vertices == [source[0], source[1], source[6]])
        #expect(welded.indices == [0, 1, 0, 0, 1, 0, 2, 2])
    }

    private func mix(_ hash: inout UInt64, _ word: UInt32) {
        for i in 0..<4 {
            hash ^= UInt64((word >> (8 * i)) & 255)
            hash = hash &* 1099511628211
        }
    }
}
