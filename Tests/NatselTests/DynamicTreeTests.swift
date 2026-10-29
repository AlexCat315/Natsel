private import Foundation
import Testing
@testable import Natsel

private func fnv(_ h: UInt64, _ bytes: UnsafeRawPointer, _ n: Int) -> UInt64 {
    var h = h
    let p = bytes.assumingMemoryBound(to: UInt8.self)
    for i in 0..<n {
        h ^= UInt64(p[i])
        h = h &* 1_099_511_628_211
    }
    return h
}

private func treeHash(_ tree: DynamicTree) -> UInt64 {
    var h: UInt64 = 14_695_981_039_346_656_037
    var nodeEnd = tree.nodeEnd
    var nodeCapacity = tree.nodeCapacity
    var proxyCount = tree.proxyCount
    var proxyCapacity = tree.proxyCapacity
    var pairFreeList = tree.pairFreeList
    var proxyFreeList = tree.proxyFreeList
    var dfsOrdered: UInt8 = tree.dfsOrdered ? 1 : 0

    h = fnv(h, &nodeEnd, 4)
    h = fnv(h, &nodeCapacity, 4)
    h = fnv(h, &proxyCount, 4)
    h = fnv(h, &proxyCapacity, 4)
    h = fnv(h, &pairFreeList, 4)
    h = fnv(h, &proxyFreeList, 4)
    h = fnv(h, &dfsOrdered, 1)

    tree.nodes.withUnsafeBufferPointer { buffer in
        for i in 0..<Int(tree.nodeEnd) {
            h = fnv(h, buffer.baseAddress! + i, 32)
        }
    }
    tree.parents.withUnsafeBufferPointer { buffer in
        for i in 0..<Int(tree.nodeEnd) {
            h = fnv(h, buffer.baseAddress! + i, 4)
        }
    }
    tree.proxies.withUnsafeBufferPointer { buffer in
        for i in 0..<Int(tree.proxyCapacity) {
            h = fnv(h, buffer.baseAddress! + i, 24)
        }
    }

    return h
}

@Suite("Dynamic tree")
struct DynamicTreeTests {
    @Test("tree operations are repeatable")
    func treeRepeatability() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            let tree = DynamicTree(proxyCapacity: 16)
            for i in 0..<20 {
                let x = Float(i % 5)
                let y = Float(i / 5)
                let box = AABB(Vec3(x, y, 0), Vec3(x + 0.9, y + 0.9, 1))
                let category: UInt64 = (i & 1) != 0 ? 1 : 2
                _ = tree.createProxy(box, categoryBits: category, userData: UInt64(100 + i))
            }

            samples.append(UInt64(treeHash(tree)))
            #expect(tree.height == 5)
            samples.append(UInt64(tree.areaRatio.bitPattern))
            #expect(tree.proxyCount == 20)

            let queryBox = AABB(Vec3(0.5, 0.5, 0.5), Vec3(2.5, 1.5, 0.5))

            func runQuery(mask: UInt64, requireAll: Bool) -> (UInt64, Int, TreeStats) {
                var h: UInt64 = 14_695_981_039_346_656_037
                var n = 0
                let stats = tree.query(queryBox, maskBits: mask, requireAllBits: requireAll) { proxyId, userData in
                    var pid = proxyId
                    var ud = userData
                    h = fnv(h, &pid, 4)
                    h = fnv(h, &ud, 8)
                    n += 1
                    return true
                }
                return (h, n, stats)
            }

            var q = runQuery(mask: .max, requireAll: false)
            samples.append(UInt64(q.0))
            #expect(q.1 == 6)
            #expect(q.2.nodeVisits == 10)
            #expect(q.2.leafVisits == 6)

            q = runQuery(mask: 1, requireAll: false)
            samples.append(UInt64(q.0))
            #expect(q.1 == 3)

            q = runQuery(mask: 1, requireAll: true)
            samples.append(UInt64(q.0))
            #expect(q.1 == 3)

            // Ray cast
            var rayInput = RayCastInput()
            rayInput.origin = Vec3(-1, 0.45, 0.5)
            rayInput.translation = Vec3(10, 0, 0)
            rayInput.maxFraction = 1

            var h: UInt64 = 14_695_981_039_346_656_037
            var n = 0
            let rayStats = tree.rayCast(rayInput, maskBits: .max, requireAllBits: false) { input, proxyId, userData in
                var pid = proxyId
                var ud = userData
                h = fnv(h, &pid, 4)
                h = fnv(h, &ud, 8)
                var mf = input.maxFraction
                h = fnv(h, &mf, 4)
                n += 1
                return input.maxFraction
            }
            samples.append(UInt64(h))
            #expect(n == 5)
            #expect(rayStats.nodeVisits == 6)
            #expect(rayStats.leafVisits == 5)

            // Box cast
            var boxInput = BoxCastInput()
            boxInput.box = AABB(Vec3(-0.1, -0.1, 0.4), Vec3(0.1, 0.1, 0.6))
            boxInput.translation = Vec3(10, 0, 0)
            boxInput.maxFraction = 1

            h = 14_695_981_039_346_656_037
            n = 0
            let boxStats = tree.boxCast(boxInput, maskBits: .max, requireAllBits: false) { input, proxyId, userData in
                var pid = proxyId
                var ud = userData
                h = fnv(h, &pid, 4)
                h = fnv(h, &ud, 8)
                var mf = input.maxFraction
                h = fnv(h, &mf, 4)
                n += 1
                return input.maxFraction
            }
            samples.append(UInt64(h))
            #expect(n == 5)
            #expect(boxStats.nodeVisits == 6)

            // Closest query
            h = 14_695_981_039_346_656_037
            n = 0
            let closest = tree.queryClosest(Vec3(0.45, 0.45, 0.5), maskBits: .max, requireAllBits: false, minDistanceSqr: 1e30) {
                minSqr, proxyId, userData in
                var pid = proxyId
                var ud = userData
                h = fnv(h, &pid, 4)
                h = fnv(h, &ud, 8)
                var m = minSqr
                h = fnv(h, &m, 4)
                n += 1
                return minSqr
            }
            samples.append(UInt64(h))
            #expect(n == 20)
            #expect(closest.stats.nodeVisits == 38)
            #expect(closest.stats.leafVisits == 20)
            samples.append(UInt64(closest.minDistanceSqr.bitPattern))

            // Edits
            tree.moveProxy(3, AABB(Vec3(10, 10, 0), Vec3(11, 11, 1)))
            tree.destroyProxy(5)
            _ = tree.createProxy(AABB(Vec3(5, 5, 0), Vec3(5.5, 5.5, 1)), categoryBits: 4, userData: 999)
            samples.append(UInt64(treeHash(tree)))

            let rebuilt = tree.rebuild(fullBuild: false)
            #expect(rebuilt == 1)
            samples.append(UInt64(treeHash(tree)))
            #expect(tree.height == 7)

            // Save and load
            let path = NSTemporaryDirectory() + "natsel_tree.bin"
            tree.save(to: path)
            let loaded = DynamicTree.load(from: path, scale: 1)!
            samples.append(UInt64(treeHash(loaded)))
            try? FileManager.default.removeItem(atPath: path)
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
