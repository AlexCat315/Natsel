import Testing
@testable import Natsel

@Suite("Mesh manifold reduction repeatability")
struct MeshReductionTests {
    @Test("deepest, collinear, and area-based selection preserve stable order", arguments: 0..<3)
    func reduction(scenario: Int) {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
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
                #expect(!selected.isEmpty && selected.count <= 4)
                #expect(Set(selected.map(\.originalIndex)).count == selected.count)
                #expect(selected.map(\.separation).min() == points.map(\.separation).min())
                word(UInt32(selected.count))
                for point in selected {
                    word(point.p.x.bitPattern); word(point.p.y.bitPattern)
                    word(point.separation.bitPattern); word(UInt32(point.originalIndex))
                }
            }
            samples.append(UInt64(hash))
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
