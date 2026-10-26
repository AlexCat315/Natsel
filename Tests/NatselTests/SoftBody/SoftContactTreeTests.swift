import Testing
@testable import Natsel

@Suite("Soft contact feature tree")
struct SoftContactTreeTests {
    @Test("queries match brute-force overlap without duplicate features")
    func completeCandidates() {
        let boxes = (0..<48).map { index in
            let center = Vec3(Float((index * 7) % 8) - 3, Float(index / 24), Float(index / 8) - 2)
            return aabbInflate(AABB(center, center), 0.25)
        }
        var tree = SoftContactTree()
        tree.refit(boxes)
        for x in -5...5 {
            let center = Vec3(Float(x), 0, 0)
            let query = aabbInflate(AABB(center, center), 1.25)
            var found: [Int32] = []
            tree.visit(query) { found.append($0) }
            let expected = boxes.indices.filter { aabbOverlaps(query, boxes[$0]) }.map(Int32.init)
            #expect(found.sorted() == expected)
            #expect(Set(found).count == found.count)
            var later: [Int32] = []
            tree.visit(query, afterFeature: 23) { later.append($0) }
            #expect(later.sorted() == expected.filter { $0 > 23 })
        }
    }

    @Test("refitting moving features updates ancestor bounds and rebuilding changes feature count")
    func refitAndRebuild() {
        var boxes = (0..<16).map { index in
            let p = Vec3(Float(index), 0, 0)
            return aabbInflate(AABB(p, p), 0.1)
        }
        var tree = SoftContactTree()
        tree.refit(boxes)
        let distant = Vec3(-100, 50, 12)
        boxes[3] = AABB(distant, distant)
        tree.refit(boxes)
        var remote: [Int32] = []
        tree.visit(AABB(distant, distant)) { remote.append($0) }
        #expect(remote == [3])
        var old: [Int32] = []
        tree.visit(AABB(Vec3(3, 0, 0), Vec3(3, 0, 0))) { old.append($0) }
        #expect(old.isEmpty)
        tree.refit([AABB(.zero, .zero)])
        var rebuilt: [Int32] = []
        tree.visit(AABB(.zero, .zero)) { rebuilt.append($0) }
        #expect(rebuilt == [0])
        tree.refit([])
        var empty: [Int32] = []
        tree.visit(AABB(.zero, .zero)) { empty.append($0) }
        #expect(empty.isEmpty)
    }

    @Test("coincident features are each visited once")
    func coincidentFeatures() {
        var tree = SoftContactTree()
        tree.refit([AABB](repeating: AABB(.zero, .zero), count: 20))
        var found: [Int32] = []
        tree.visit(AABB(.zero, .zero)) { found.append($0) }
        #expect(found.sorted() == Array(0..<20).map(Int32.init))
    }

    @Test("rest exclusion prunes neighborhoods and preserves distant rest features folded together")
    func referenceExclusion() {
        let current = [AABB](repeating: AABB(.zero, .zero), count: 32)
        let near = Vec3(0.1, 0, 0), far = Vec3(10, 0, 0)
        let reference = (0..<32).map { index in
            let p = index < 16 ? near : far
            return AABB(p, p)
        }
        var tree = SoftContactTree()
        tree.refit(current, referenceBounds: { reference })
        var found: [Int32] = []
        tree.visit(AABB(.zero, .zero), referenceExclusion: (.zero, 1)) { found.append($0) }
        #expect(found.sorted() == Array(16..<32).map(Int32.init))
    }
}
