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
        for i in 0..<Int(tree.nodeEnd) { h = fnv(h, buffer.baseAddress! + i, 32) }
    }
    tree.parents.withUnsafeBufferPointer { buffer in
        for i in 0..<Int(tree.nodeEnd) { h = fnv(h, buffer.baseAddress! + i, 4) }
    }
    tree.proxies.withUnsafeBufferPointer { buffer in
        for i in 0..<Int(tree.proxyCapacity) { h = fnv(h, buffer.baseAddress! + i, 24) }
    }
    return h
}

@Suite("Broad phase")
struct BroadPhaseTests {
    @Test("proxies are repeatable")
    func proxies() {
        func snapshot() -> [UInt64] {
            var samples: [UInt64] = []
            var capacity = Capacity()
            capacity.staticShapeCount = 8
            capacity.dynamicShapeCount = 8
            capacity.contactCount = 16

            let bp = BroadPhase(capacity: capacity)

            var keys: [Int32] = []
            for i in 0..<4 {
                let box = AABB(Vec3(Float(i), 0, 0), Vec3(Float(i) + 0.5, 1, 1))
                keys.append(
                    bp.createProxy(box, type: .staticBody, categoryBits: 1, shapeIndex: Int32(i), forcePairCreation: false)
                )
            }
            for i in 0..<2 {
                let box = AABB(Vec3(Float(i), 2, 0), Vec3(Float(i) + 0.5, 3, 1))
                keys.append(
                    bp.createProxy(
                        box, type: .kinematicBody, categoryBits: 1, shapeIndex: Int32(10 + i), forcePairCreation: false
                    )
                )
            }
            for i in 0..<6 {
                let box = AABB(Vec3(Float(i) * 0.5, 4, 0), Vec3(Float(i) * 0.5 + 0.4, 4.4, 1))
                keys.append(
                    bp.createProxy(
                        box, type: .dynamicBody, categoryBits: 2, shapeIndex: Int32(20 + i), forcePairCreation: false
                    )
                )
            }

            #expect(keys == [0, 4, 8, 12, 1, 5, 2, 6, 10, 14, 18, 22])
            #expect(bp.getShapeIndex(keys[0]) == 0)
            #expect(bp.getShapeIndex(keys[6]) == 20)
            #expect(bp.getShapeIndex(keys[11]) == 25)

            #expect(!bp.testOverlap(keys[0], keys[1]))
            #expect(!bp.testOverlap(keys[0], keys[6]))
            #expect(!bp.testOverlap(keys[6], keys[7]))

            bp.moveProxy(keys[8], AABB(Vec3(0, 4, 0), Vec3(1, 5, 1)))
            bp.destroyProxy(keys[7])

            #expect(bp.testOverlap(keys[8], keys[6]))
            #expect(!bp.testOverlap(keys[8], keys[9]))

            samples.append(UInt64(treeHash(bp.trees[0])))
            samples.append(UInt64(treeHash(bp.trees[1])))
            samples.append(UInt64(treeHash(bp.trees[2])))

            let queryBox = AABB(Vec3(0, 3.8, 0), Vec3(2, 5, 1))
            var h: UInt64 = 14_695_981_039_346_656_037
            var n = 0
            let stats = bp.trees[2].query(queryBox, maskBits: .max, requireAllBits: false) { proxyId, userData in
                var pid = proxyId
                var ud = userData
                h = fnv(h, &pid, 4)
                h = fnv(h, &ud, 8)
                n += 1
                return true
            }
            samples.append(UInt64(h))
            #expect(n == 4)
            #expect(stats.nodeVisits == 4)
            #expect(stats.leafVisits == 4)

            bp.validate()
            return samples
        }

        let first = snapshot()
        let replay = snapshot()
        #expect(!first.isEmpty)
        #expect(first == replay)
    }
}
