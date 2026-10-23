import Testing
@testable import Natsel

@Suite("Mesh manifold reduction C parity")
struct MeshReductionTests {
    @Test("deepest, collinear, and area-based selection preserve C order", arguments: 0..<3)
    func reduction(scenario: Int) {
        var hash: UInt64 = 14695981039346656037
        func word(_ value: UInt32) {
            for byte in 0..<4 {
                hash ^= UInt64((value >> (8 * byte)) & 255)
                hash = hash &* 1099511628211
            }
        }
        for trial in 0..<100 {
            let points = (0..<(trial % 24 + 1)).map { i in
                var x = Float((i * 17 + trial * 3) % 19) - 9
                var y = Float((i * 7 + trial * 5) % 13) - 6
                if scenario == 0 { x *= 0.00001; y *= 0.00001 }
                if scenario == 1 { y = 0 }
                return MeshPatchPoint(p: Vec2(x, y), separation: Float((i * 3 + trial) % 11) * -0.002, originalIndex: i)
            }
            let selected = cullMeshPoints(points)
            word(UInt32(selected.count))
            for point in selected {
                word(point.p.x.bitPattern); word(point.p.y.bitPattern)
                word(point.separation.bitPattern); word(UInt32(point.originalIndex))
            }
        }
        let expected: [UInt64] = [0x3866624d13aec4b4, 0x326f53a2a26e4434, 0x094a6b0accdbe76f]
        #expect(hash == expected[scenario])
    }
}
