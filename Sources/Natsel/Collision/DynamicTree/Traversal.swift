/// Read-only traversal of dynamic trees or immutable compound tree nodes.
struct TreeView {
    let nodes: UnsafeBufferPointer<TreeNode>
    let proxies: UnsafeBufferPointer<TreeProxy>?

    private var rootPairIndex: Int32 {
        let root = nodes[Int(rootNode)]
        return isLeaf(root) ? rootNode : leftChild(root)
    }

    private func userData(_ node: TreeNode, mask: UInt64, requireAll: Bool) -> UInt64? {
        if let proxies {
            let proxy = proxies[Int(proxyIdOf(node))]
            let passes = requireAll ? (proxy.categoryBits & mask) == mask : (proxy.categoryBits & mask) != 0
            return passes ? proxy.userData : nil
        }
        return UInt64(UInt32(bitPattern: node.shapeIndex))
    }

    func query(
        _ aabb: AABB,
        maskBits: UInt64 = .max,
        requireAllBits: Bool = false,
        _ callback: (Int32, UInt64) -> Bool
    ) -> TreeStats {
        var result = TreeStats()

        if nodes.isEmpty {
            return result
        }

        if isEmptyNode(nodes[Int(rootNode)]) {
            return result
        }

        // Keep this at 1 KB or less: Swift's temporary allocation falls back to the heap above
        // 1 KB, and tree traversal depth is logarithmic in the proxy count.
        return withUnsafeTemporaryAllocation(of: Int32.self, capacity: 256) { stack in
            var stackCount = 0
            stack[stackCount] = rootPairIndex
            stackCount += 1

            while stackCount > 0 {
                stackCount -= 1
                let pair = stack[stackCount]
                result.nodeVisits += 1

                for i in 0..<2 {
                    let nodeIndex = pair + Int32(i)
                    let node = nodes[Int(nodeIndex)]
                    if aabbOverlaps(aabb, node.aabb) == false {
                        continue
                    }

                    if isLeaf(node) {
                        let proxyId = proxyIdOf(node)
                        guard let userData = userData(node, mask: maskBits, requireAll: requireAllBits) else {
                            continue
                        }

                        let proceed = callback(proxyId, userData)
                        result.leafVisits += 1

                        if proceed == false {
                            return result
                        }
                    } else {
                        if stackCount < treeStackSize - 1 {
                            stack[stackCount] = leftChild(node)
                            stackCount += 1
                        } else {
                            b3Assert(stackCount < treeStackSize - 1)
                        }
                    }
                }
            }

            return result
        }
    }

    func queryClosest(
        _ point: Vec3,
        maskBits: UInt64 = .max,
        requireAllBits: Bool = false,
        minDistanceSqr: Float = .greatestFiniteMagnitude,
        _ callback: (Float, Int32, UInt64) -> Float
    ) -> (stats: TreeStats, minDistanceSqr: Float) {
        var result = TreeStats()
        var minSqr = minDistanceSqr

        if nodes.isEmpty {
            return (result, minSqr)
        }

        if isEmptyNode(nodes[Int(rootNode)]) {
            return (result, minSqr)
        }

        func distanceToNodeSqr(_ node: TreeNode) -> Float {
            let r = point - clampVec3(point, node.aabb.lowerBound, node.aabb.upperBound)
            return dot(r, r)
        }

        return withUnsafeTemporaryAllocation(
            of: (nodeIndex: Int32, distanceToNodeSqr: Float).self, capacity: 128
        ) { stack in
            var stackCount = 0

            let root = rootPairIndex
            let seed1 = (nodeIndex: root, distanceToNodeSqr: distanceToNodeSqr(nodes[Int(root)]))

            if root == rootNode {
                stack[stackCount] = seed1
                stackCount += 1
            } else {
                let seed2 = (nodeIndex: root + 1, distanceToNodeSqr: distanceToNodeSqr(nodes[Int(root) + 1]))

                // Ensure we iterate the closest child first as we pop off the stack
                if seed2.distanceToNodeSqr < seed1.distanceToNodeSqr {
                    stack[stackCount] = seed1
                    stack[stackCount + 1] = seed2
                } else {
                    stack[stackCount] = seed2
                    stack[stackCount + 1] = seed1
                }
                stackCount += 2
            }

            while stackCount > 0 {
                stackCount -= 1
                let item = stack[stackCount]
                let node = nodes[Int(item.nodeIndex)]
                result.nodeVisits += 1

                if item.distanceToNodeSqr >= minSqr {
                    continue
                }

                if isLeaf(node) {
                    let proxyId = proxyIdOf(node)
                    guard let userData = userData(node, mask: maskBits, requireAll: requireAllBits) else { continue }

                    // callback to user code with minimum distance squared so far and proxy id
                    let dd = callback(minSqr, proxyId, userData)

                    if dd < minSqr {
                        minSqr = dd
                    }

                    result.leafVisits += 1
                    continue
                }

                if stackCount + 2 > treeStackSize {
                    b3Assert(stackCount + 2 <= treeStackSize)
                    continue
                }

                let pair = leftChild(node)

                let item1 = (nodeIndex: pair, distanceToNodeSqr: distanceToNodeSqr(nodes[Int(pair)]))
                let item2 = (nodeIndex: pair + 1, distanceToNodeSqr: distanceToNodeSqr(nodes[Int(pair) + 1]))

                // Ensure we iterate the closest child first as we pop off the stack
                if item2.distanceToNodeSqr < item1.distanceToNodeSqr {
                    stack[stackCount] = item1
                    stack[stackCount + 1] = item2
                } else {
                    stack[stackCount] = item2
                    stack[stackCount + 1] = item1
                }
                stackCount += 2
            }

            return (result, minSqr)
        }
    }

    func rayCast(
        _ input: RayCastInput,
        maskBits: UInt64 = .max,
        requireAllBits: Bool = false,
        _ callback: (inout RayCastInput, Int32, UInt64) -> Float
    ) -> TreeStats {
        var result = TreeStats()

        if nodes.isEmpty {
            return result
        }

        if isEmptyNode(nodes[Int(rootNode)]) {
            return result
        }

        let p1 = input.origin
        let d = input.translation

        var maxFraction = input.maxFraction

        var p2 = mulAdd(p1, maxFraction, d)

        // Build a bounding box for the segment.
        var segmentAABB = AABB(minVec3(p1, p2), maxVec3(p1, p2))

        // Keep this at 1 KB or less: Swift's temporary allocation falls back to the heap above
        // 1 KB, and tree traversal depth is logarithmic in the proxy count.
        return withUnsafeTemporaryAllocation(of: Int32.self, capacity: 256) { stack in
            var stackCount = 0
            stack[stackCount] = rootPairIndex
            stackCount += 1

            var subInput = input

            while stackCount > 0 {
                stackCount -= 1
                let pair = stack[stackCount]
                result.nodeVisits += 1

                var hit = [TreeNode](repeating: TreeNode(), count: 2)
                var isLeafHit = [false, false]
                var hitCount = 0
                for i in 0..<2 {
                    let node = nodes[Int(pair) + i]

                    if aabbOverlaps(segmentAABB, node.aabb) == false {
                        continue
                    }

                    let nodeAABB = node.aabb
                    if testBoundsRayOverlap(nodeAABB.lowerBound, nodeAABB.upperBound, p1, d) == false {
                        continue
                    }

                    isLeafHit[hitCount] = isLeaf(node)
                    hit[hitCount] = node
                    hitCount += 1
                }

                if hitCount == 2 && isLeafHit[0] == false && isLeafHit[1] == false {
                    let center1 = aabbCenter(hit[0].aabb)
                    let center2 = aabbCenter(hit[1].aabb)
                    let d1 = distanceSquared(center1, p1)
                    let d2 = distanceSquared(center2, p1)

                    // Want to push the closest one last. Both have the same isLeaf, so they don't swap.
                    if d1 < d2 {
                        hit.swapAt(0, 1)
                    }
                }

                for i in 0..<hitCount {
                    if isLeafHit[i] {
                        let proxyId = proxyIdOf(hit[i])
                        guard let userData = userData(hit[i], mask: maskBits, requireAll: requireAllBits) else {
                            continue
                        }

                        subInput.maxFraction = maxFraction

                        let value = callback(&subInput, proxyId, userData)
                        result.leafVisits += 1

                        // The user may return -1 to indicate this shape should be skipped

                        if value == 0 {
                            // The client has terminated the ray cast.
                            return result
                        }

                        if 0 < value && value <= maxFraction {
                            // Update segment bounding box.
                            maxFraction = value
                            p2 = mulAdd(p1, maxFraction, d)
                            segmentAABB.lowerBound = minVec3(p1, p2)
                            segmentAABB.upperBound = maxVec3(p1, p2)
                        }
                    } else {
                        if stackCount < treeStackSize - 1 {
                            stack[stackCount] = leftChild(hit[i])
                            stackCount += 1
                        } else {
                            b3Assert(stackCount < treeStackSize - 1)
                        }
                    }
                }
            }

            return result
        }
    }

    func boxCast(
        _ input: BoxCastInput,
        maskBits: UInt64 = .max,
        requireAllBits: Bool = false,
        _ callback: (inout BoxCastInput, Int32, UInt64) -> Float
    ) -> TreeStats {
        var result = TreeStats()

        if nodes.isEmpty {
            return result
        }

        if isEmptyNode(nodes[Int(rootNode)]) {
            return result
        }

        // The caller folds the shape radius and the world origin into the box
        let originAABB = input.box

        let p1 = aabbCenter(originAABB)
        let extension_ = maxVec3(p1 - originAABB.lowerBound, originAABB.upperBound - p1)

        let d = input.translation

        var maxFraction = input.maxFraction

        // Build total box for the cast
        var t = maxFraction * input.translation
        var totalAABB = AABB(
            minVec3(originAABB.lowerBound, originAABB.lowerBound + t),
            maxVec3(originAABB.upperBound, originAABB.upperBound + t)
        )

        // Keep this at 1 KB or less: Swift's temporary allocation falls back to the heap above
        // 1 KB, and tree traversal depth is logarithmic in the proxy count.
        return withUnsafeTemporaryAllocation(of: Int32.self, capacity: 256) { stack in
            var stackCount = 0
            stack[stackCount] = rootPairIndex
            stackCount += 1

            var subInput = input

            while stackCount > 0 {
                stackCount -= 1
                let pair = stack[stackCount]
                result.nodeVisits += 1

                var hit = [TreeNode](repeating: TreeNode(), count: 2)
                var isLeafHit = [false, false]
                var hitCount = 0
                for i in 0..<2 {
                    let node = nodes[Int(pair) + i]

                    if aabbOverlaps(totalAABB, node.aabb) == false {
                        continue
                    }

                    // radius extension is added to the node in this case
                    let nodeAABB = node.aabb
                    let lower = nodeAABB.lowerBound - extension_
                    let upper = nodeAABB.upperBound + extension_
                    if testBoundsRayOverlap(lower, upper, p1, d) == false {
                        continue
                    }

                    isLeafHit[hitCount] = isLeaf(node)
                    hit[hitCount] = node
                    hitCount += 1
                }

                if hitCount == 2 && isLeafHit[0] == false && isLeafHit[1] == false {
                    let center1 = aabbCenter(hit[0].aabb)
                    let center2 = aabbCenter(hit[1].aabb)
                    let d1 = distanceSquared(center1, p1)
                    let d2 = distanceSquared(center2, p1)

                    // Want to push the closest one last. Both have the same isLeaf, so they don't swap.
                    if d1 < d2 {
                        hit.swapAt(0, 1)
                    }
                }

                for i in 0..<hitCount {
                    if isLeafHit[i] {
                        let proxyId = proxyIdOf(hit[i])
                        guard let userData = userData(hit[i], mask: maskBits, requireAll: requireAllBits) else {
                            continue
                        }

                        subInput.maxFraction = maxFraction

                        let value = callback(&subInput, proxyId, userData)
                        result.leafVisits += 1

                        if value == 0 {
                            // The client has terminated the cast.
                            return result
                        }

                        if 0 < value && value < maxFraction {
                            maxFraction = value
                            t = maxFraction * input.translation
                            totalAABB.lowerBound = minVec3(originAABB.lowerBound, originAABB.lowerBound + t)
                            totalAABB.upperBound = maxVec3(originAABB.upperBound, originAABB.upperBound + t)
                        }
                    } else {
                        if stackCount < treeStackSize - 1 {
                            stack[stackCount] = leftChild(hit[i])
                            stackCount += 1
                        } else {
                            b3Assert(stackCount < treeStackSize - 1)
                        }
                    }
                }
            }

            return result
        }
    }

}
