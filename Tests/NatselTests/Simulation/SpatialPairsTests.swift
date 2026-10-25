import Testing
@testable import Natsel

@Suite("Precise spatial pair sweep")
struct SpatialPairsTests {
    @Test("sweep agrees with exhaustive overlap at distant coordinates")
    func oracle() {
        var seed: UInt64 = 17
        func random() -> Double {
            seed = seed &* 6364136223846793005 &+ 1
            return Double(seed >> 40) / Double(1 << 24)
        }
        let bounds = (0..<400).map { _ -> WorldAABB in
            let p = Pos(1.0e12 + random() * 20, -1.0e12 + random() * 20, random() * 20)
            let size = random() * 3
            return WorldAABB(p, Pos(p.x + size, p.y + size, p.z + size))
        }
        var expected: [BoundsPair] = []
        for a in bounds.indices {
            for b in (a + 1)..<bounds.count where bounds[a].overlaps(bounds[b]) {
                expected.append(BoundsPair(a: a, b: b))
            }
        }
        #expect(overlappingBoundsPairs(bounds).pairs == expected)
    }

    @Test("separated objects avoid quadratic candidate work", arguments: [0, 1, 2])
    func sparse(_ axis: Int) {
        let bounds = (0..<5000).map { i -> WorldAABB in
            let d = Double(i) * 4
            let p = Pos(1.0e12 + (axis == 0 ? d : 0), axis == 1 ? d : 0, axis == 2 ? d : 0)
            return WorldAABB(p, Pos(p.x + 1, p.y + 1, p.z + 1))
        }
        let result = overlappingBoundsPairs(bounds)
        #expect(result.pairs.isEmpty)
        #expect(result.candidateCount == 0)
    }

    @Test("touching endpoints and equal sort keys retain every pair")
    func ties() {
        let bounds = [WorldAABB(Pos(), Pos(1, 1, 1)), WorldAABB(Pos(1, 0, 0), Pos(2, 1, 1)), WorldAABB(Pos(), Pos(1, 1, 1))]
        #expect(overlappingBoundsPairs(bounds).pairs == [BoundsPair(a: 0, b: 1), BoundsPair(a: 0, b: 2), BoundsPair(a: 1, b: 2)])
        #expect(overlappingBoundsPairs([]).pairs.isEmpty)
    }
}
