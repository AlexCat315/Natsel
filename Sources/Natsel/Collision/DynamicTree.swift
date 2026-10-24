// Dynamic AABB tree, inspired by Nathanael Presson's btDbvt. Nodes are pooled and relocatable,
// so indices are used instead of pointers. Sibling nodes are paired at even indices.

private import Foundation

// MARK: - Constants

let treeStackSize = 512

let movedNodeFlag: UInt32 = 1 << 30
let leafNodeFlag: UInt32 = 1 << 31
let nodeIndexMask: UInt32 = 0xFFFF_FFFF & ~(movedNodeFlag | leafNodeFlag)

/// Passes as a leaf so internal node processing naturally skips it.
let emptyNodeFlag: UInt32 = nodeIndexMask | leafNodeFlag

let rootNode: Int32 = 0

// MARK: - Types

/// A node in the dynamic tree.
public struct TreeNode: Sendable, BitwiseCopyable {
    /// The node bounding box.
    public var aabb = AABB()

    /// bit 31: 1 for leaf node
    /// bit 30: 1 for moved flag
    /// bits 0-29: index of the sibling pair node or the proxy id for a leaf
    public var flagIndex: UInt32 = 0

    /// The height of an internal node or the shape index for a leaf. This is the C union.
    var u: Int32 = 0

    public var height: Int32 { get { u } set { u = newValue } }
    public var shapeIndex: Int32 { get { u } set { u = newValue } }

    public init() {}
}

/// Separate storage for tree leaves.
public struct TreeProxy: Sendable, BitwiseCopyable {
    /// User data is an index instead of a pointer because it is used internally as a shape index.
    public var userData: UInt64 = 0

    /// Category bits for collision filtering.
    public var categoryBits: UInt64 = 0

    /// The leaf node. `B3.nullIndex` for a free proxy.
    public var node: Int32 = 0

    /// Next free proxy.
    public var next: Int32 = 0

    public init() {}
}

/// Performance results returned by dynamic tree queries.
public struct TreeStats: Sendable, Equatable, BitwiseCopyable {
    /// Number of internal nodes visited during the query.
    public var nodeVisits: Int32 = 0

    /// Number of leaf nodes visited during the query.
    public var leafVisits: Int32 = 0

    public init() {}
}

// MARK: - Node helpers

@inline(__always)
func isLeaf(_ node: TreeNode) -> Bool {
    (node.flagIndex & leafNodeFlag) == leafNodeFlag
}

@inline(__always)
func isNodeMoved(_ node: TreeNode) -> Bool {
    (node.flagIndex & movedNodeFlag) == movedNodeFlag
}

@inline(__always)
func isEmptyNode(_ node: TreeNode) -> Bool {
    node.flagIndex == emptyNodeFlag
}

@inline(__always)
func leftChild(_ node: TreeNode) -> Int32 {
    Int32(node.flagIndex & nodeIndexMask)
}

@inline(__always)
func proxyIdOf(_ node: TreeNode) -> Int32 {
    Int32(node.flagIndex & nodeIndexMask)
}

@inline(__always)
func rootPair(_ nodes: [TreeNode]) -> Int32 {
    let root = nodes[Int(rootNode)]
    return isLeaf(root) ? rootNode : leftChild(root)
}

@inline(__always)
func makeEmptyNode() -> TreeNode {
    var node = TreeNode()
    node.aabb.lowerBound = Vec3(Float.infinity, Float.infinity, Float.infinity)
    node.aabb.upperBound = Vec3(-Float.infinity, -Float.infinity, -Float.infinity)
    node.flagIndex = emptyNodeFlag
    node.u = 0
    return node
}

/// Test a ray for edge separation with an AABB (Gino, p80).
@inline(__always)
func testBoundsRayOverlap(_ nodeMin: Vec3, _ nodeMax: Vec3, _ rayStart: Vec3, _ rayDelta: Vec3) -> Bool {
    // Setup node
    let nodeCenter = 0.5 * (nodeMin + nodeMax)
    let nodeExtent = nodeMax - nodeCenter

    // Setup ray
    let start = rayStart - nodeCenter

    // SAT: edge separation
    let edgeSeparation = absVec3(cross(rayDelta, start)) - modifiedCross(absVec3(rayDelta), nodeExtent)
    return edgeSeparation.x <= 0 && edgeSeparation.y <= 0 && edgeSeparation.z <= 0
}

// MARK: - Dynamic tree

/// A dynamic AABB tree.
public final class DynamicTree {
    /// The dynamic tree version. Useful if the tree is serialized.
    public private(set) var version: UInt64 = B3.dynamicTreeVersion

    /// Array of nodes. The root is at index zero and index 1 is empty. Otherwise siblings are
    /// paired at even indices. Has holes for free node pairs.
    var nodes: [TreeNode]

    /// Parent index per node. The free list is interweaved.
    var parents: [Int32]

    /// Proxy data split from node array as cold data.
    var proxies: [TreeProxy]

    /// Every allocated node has a lower index than this.
    var nodeEnd: Int32

    /// The allocated node space.
    var nodeCapacity: Int32

    /// Free pairs below nodeEnd.
    var pairFreeList: Int32

    /// Number of proxies created.
    public internal(set) var proxyCount: Int32

    /// The allocated proxy space.
    var proxyCapacity: Int32

    /// Proxy free list.
    var proxyFreeList: Int32

    /// Array of nodes for rebuild.
    var swapNodes: [TreeNode] = []

    /// Leaf indices for rebuild.
    var leafIndices: [Int32] = []

    /// Leaves for the rebuild. May represent a proxy or a retained subtree.
    var leafNodes: [TreeNode] = []

    /// Leaf bounding box centers for rebuild.
    var leafCenters: [Vec3] = []

    /// Bins for sorting during rebuild.
    var binIndices: [Int32] = []

    /// Allocated space for rebuilding.
    var rebuildCapacity: Int32 = 0

    /// DFS stack reused by rebuilds.
    var rebuildStack: [Int32] = []

    /// Stack reused while copying retained subtrees during a rebuild.
    var copyStack: [(oldPair: Int32, newIndex: Int32)] = []

    /// Rebuild orders the nodes so the children follow parents.
    var dfsOrdered: Bool = true

    // MARK: Lifecycle

    /// Constructing the tree initializes the node pool.
    public init(proxyCapacity: Int32) {
        let capacity = maxInt(proxyCapacity, 16)

        version = B3.dynamicTreeVersion

        // A tree of n proxies has 2n - 1 nodes plus the empty node beside the root
        nodeCapacity = 2 * capacity
        nodes = Array(repeating: TreeNode(), count: Int(nodeCapacity))
        parents = Array(repeating: 0, count: Int(nodeCapacity))
        pairFreeList = B3.nullIndex

        // The root and the empty node always exist
        nodes[Int(rootNode)] = makeEmptyNode()
        nodes[Int(rootNode) + 1] = makeEmptyNode()
        parents[Int(rootNode)] = B3.nullIndex
        parents[Int(rootNode) + 1] = B3.nullIndex

        nodeEnd = 2
        dfsOrdered = true

        self.proxyCapacity = capacity
        proxyCount = 0
        proxies = Array(repeating: TreeProxy(), count: Int(self.proxyCapacity))

        // Build a linked list for the free list.
        for i in 0..<Int(self.proxyCapacity) - 1 {
            proxies[i].node = B3.nullIndex
            proxies[i].next = Int32(i + 1)
        }

        proxies[Int(self.proxyCapacity) - 1].node = B3.nullIndex
        proxies[Int(self.proxyCapacity) - 1].next = B3.nullIndex
        proxyFreeList = 0
    }

    deinit {}

    // MARK: Pool management

    /// Allocate a sibling pair, growing the node pool if needed.
    private func allocateSiblingPair() -> Int32 {
        // Allocate from free list.
        if pairFreeList != B3.nullIndex {
            let pair = pairFreeList
            pairFreeList = parents[Int(pair)]
            return pair
        }

        // Extend the bump allocator if needed.
        if nodeEnd + 2 > nodeCapacity {
            let oldCapacity = nodeCapacity
            var newCapacity = oldCapacity + (oldCapacity >> 1)
            newCapacity += newCapacity & 1
            growNodeCapacity(to: newCapacity)
            nodeCapacity = newCapacity

            // The spare has to match, the rebuild allocates it again.
            swapNodes = []
        }

        let pair = nodeEnd
        nodeEnd += 2
        return pair
    }

    private func growNodeCapacity(to newCapacity: Int32) {
        let extra = Int(newCapacity) - nodes.count
        guard extra > 0 else {
            return
        }
        nodes.append(contentsOf: repeatElement(TreeNode(), count: extra))
        parents.append(contentsOf: repeatElement(0, count: extra))
    }

    private func freePair(_ pair: Int32) {
        b3Assert((pair & 1) == 0 && 2 <= pair && pair < nodeEnd)
        nodes[Int(pair)] = makeEmptyNode()
        nodes[Int(pair) + 1] = makeEmptyNode()
        parents[Int(pair)] = pairFreeList
        parents[Int(pair) + 1] = B3.nullIndex
        pairFreeList = pair
    }

    /// Allocate a proxy from the pool. Grow the pool if necessary.
    private func allocateProxy() -> Int32 {
        // Expand the pool as needed.
        if proxyFreeList == B3.nullIndex {
            b3Assert(proxyCount == proxyCapacity)

            // The free list is empty. Rebuild a bigger pool.
            let oldCapacity = proxyCapacity
            proxyCapacity += maxInt(oldCapacity >> 1, 1)
            proxies.append(contentsOf: repeatElement(TreeProxy(), count: Int(proxyCapacity - oldCapacity)))

            // Build a linked list for the free list.
            for i in Int(oldCapacity)..<Int(proxyCapacity) - 1 {
                proxies[i].node = B3.nullIndex
                proxies[i].next = Int32(i + 1)
            }

            proxies[Int(proxyCapacity) - 1].node = B3.nullIndex
            proxies[Int(proxyCapacity) - 1].next = B3.nullIndex
            proxyFreeList = oldCapacity
        }

        // Peel a proxy off the free list.
        let proxyIndex = proxyFreeList
        proxyFreeList = proxies[Int(proxyIndex)].next
        proxies[Int(proxyIndex)] = TreeProxy()
        proxies[Int(proxyIndex)].node = B3.nullIndex
        proxies[Int(proxyIndex)].next = B3.nullIndex
        proxyCount += 1
        return proxyIndex
    }

    /// Return a proxy to the pool.
    private func freeProxy(_ proxyId: Int32) {
        b3Assert(0 <= proxyId && proxyId < proxyCapacity)
        b3Assert(0 < proxyCount)
        proxies[Int(proxyId)].node = B3.nullIndex
        proxies[Int(proxyId)].next = proxyFreeList
        proxyFreeList = proxyId
        proxyCount -= 1
    }

    @inline(__always)
    private func getNodeHeight(_ node: TreeNode) -> Int32 {
        isLeaf(node) ? 0 : node.height
    }

    /// The internal node above a children pair.
    @inline(__always)
    private func makeInternalNode(_ pair: Int32) -> TreeNode {
        makeInternalNode(nodes, pair)
    }

    /// The internal node above a children pair, reading children from the given array.
    @inline(__always)
    private func makeInternalNode(_ nodes: [TreeNode], _ pair: Int32) -> TreeNode {
        let c1 = nodes[Int(pair)]
        let c2 = nodes[Int(pair) + 1]

        var node = TreeNode()
        node.aabb = aabbUnion(c1.aabb, c2.aabb)
        node.flagIndex = UInt32(pair) | ((c1.flagIndex | c2.flagIndex) & movedNodeFlag)
        node.height = 1 + maxInt(getNodeHeight(c1), getNodeHeight(c2))
        return node
    }

    @inline(__always)
    private func makeLeafNode(_ aabb: AABB, _ proxyId: Int32, _ userData: UInt64, _ moved: Bool) -> TreeNode {
        var node = TreeNode()
        node.aabb = aabb
        node.flagIndex = UInt32(proxyId) | leafNodeFlag | (moved ? movedNodeFlag : 0)
        node.shapeIndex = Int32(truncatingIfNeeded: userData)
        return node
    }

    /// A node landed at a new index, so tell what hangs below it.
    @inline(__always)
    private func linkChildren(_ nodeIndex: Int32) {
        let node = nodes[Int(nodeIndex)]
        if isLeaf(node) {
            proxies[Int(proxyIdOf(node))].node = nodeIndex
        } else {
            let pair = leftChild(node)
            parents[Int(pair)] = nodeIndex
            parents[Int(pair) + 1] = nodeIndex
        }
    }

    /// The sweep refit visits indices from high to low, so it needs every child above its parent.
    @inline(__always)
    private func isNodeOrdered(_ nodeIndex: Int32) -> Bool {
        let node = nodes[Int(nodeIndex)]
        return isLeaf(node) || nodeIndex < leftChild(node)
    }

    // MARK: Sibling selection

    /// Greedy algorithm for sibling selection using the SAH.
    private func findBestSibling(_ boxD: AABB) -> Int32 {
        let centerD = aabbCenter(boxD)
        let areaD = perimeter(boxD)

        var nodeIndex = rootNode

        if isLeaf(nodes[Int(nodeIndex)]) {
            return nodeIndex
        }

        let rootBox = nodes[Int(nodeIndex)].aabb

        // Area of current node
        var areaBase = perimeter(rootBox)

        // Area of inflated node
        var directCost = perimeter(aabbUnion(rootBox, boxD))
        var inheritedCost: Float = 0

        var bestSibling = nodeIndex
        var bestCost = directCost

        // Descend the tree, following a single greedy path.
        while true {
            let child1 = leftChild(nodes[Int(nodeIndex)])
            let child2 = child1 + 1

            // Cost of creating a new parent for this node and the new leaf
            let cost = directCost + inheritedCost

            // Sometimes there are multiple identical costs within tolerance.
            // This breaks the ties using the centroid distance.
            if cost < bestCost {
                bestSibling = nodeIndex
                bestCost = cost
            }

            // Inheritance cost seen by children
            inheritedCost += directCost - areaBase

            let leaf1 = isLeaf(nodes[Int(child1)])
            let leaf2 = isLeaf(nodes[Int(child2)])

            // Cost of descending into child 1
            var lowerCost1 = Float.greatestFiniteMagnitude
            let box1 = nodes[Int(child1)].aabb
            let directCost1 = perimeter(aabbUnion(box1, boxD))
            var area1: Float = 0
            if leaf1 {
                // Child 1 is a leaf
                // Cost of creating new node and increasing area of node P
                let cost1 = directCost1 + inheritedCost
                if cost1 < bestCost {
                    bestSibling = child1
                    bestCost = cost1
                }
            } else {
                // Child 1 is an internal node
                area1 = perimeter(box1)

                // Lower bound cost of inserting under child 1.
                lowerCost1 = inheritedCost + directCost1 + minFloat(areaD - area1, 0)
            }

            // Cost of descending into child 2
            var lowerCost2 = Float.greatestFiniteMagnitude
            let box2 = nodes[Int(child2)].aabb
            let directCost2 = perimeter(aabbUnion(box2, boxD))
            var area2: Float = 0
            if leaf2 {
                let cost2 = directCost2 + inheritedCost
                if cost2 < bestCost {
                    bestSibling = child2
                    bestCost = cost2
                }
            } else {
                area2 = perimeter(box2)
                lowerCost2 = inheritedCost + directCost2 + minFloat(areaD - area2, 0)
            }

            if leaf1 && leaf2 {
                break
            }

            // Can the cost possibly be decreased?
            if bestCost <= lowerCost1 && bestCost <= lowerCost2 {
                break
            }

            if lowerCost1 == lowerCost2 && leaf1 == false {
                b3Assert(lowerCost1 < Float.greatestFiniteMagnitude)
                b3Assert(lowerCost2 < Float.greatestFiniteMagnitude)

                // No clear choice based on lower bound surface area. This can happen when both
                // children fully contain D. Fall back to node distance.
                let d1 = aabbCenter(box1) - centerD
                let d2 = aabbCenter(box2) - centerD
                lowerCost1 = lengthSquared(d1)
                lowerCost2 = lengthSquared(d2)
            }

            // Descend
            if lowerCost1 < lowerCost2 && leaf1 == false {
                nodeIndex = child1
                areaBase = area1
                directCost = directCost1
            } else {
                nodeIndex = child2
                areaBase = area2
                directCost = directCost2
            }
        }

        return bestSibling
    }

    /// Swap a child of A with a grandchild under the other child.
    private func swapTreeNodes(_ iDown: Int32, _ iUp: Int32) {
        nodes.swapAt(Int(iDown), Int(iUp))
        linkChildren(iDown)
        linkChildren(iUp)

        if isNodeOrdered(iDown) == false || isNodeOrdered(iUp) == false {
            dfsOrdered = false
        }

        // The sibling of the node that went down holds a different subtree now
        let iC = iDown ^ 1
        nodes[Int(iC)] = makeInternalNode(leftChild(nodes[Int(iC)]))
    }

    /// Perform a left or right rotation if node A is imbalanced.
    private func rotateNodes(_ iA: Int32) {
        let nodeA = nodes[Int(iA)]
        b3Assert(isLeaf(nodeA) == false)

        let iB = leftChild(nodeA)
        let iC = iB + 1
        let nodeB = nodes[Int(iB)]
        let nodeC = nodes[Int(iC)]

        let leafB = isLeaf(nodeB)
        let leafC = isLeaf(nodeC)
        if leafB && leafC {
            return
        }

        var bestDown = B3.nullIndex
        var bestUp = B3.nullIndex
        var bestDelta: Float = 0

        if leafC == false {
            // Swap B with F or G
            let iF = leftChild(nodeC)
            let iG = iF + 1
            let areaC = perimeter(nodeC.aabb)

            // B <-> F then C (B G)
            let deltaBF = perimeter(aabbUnion(nodeB.aabb, nodes[Int(iG)].aabb)) - areaC
            if deltaBF < bestDelta {
                bestDown = iB
                bestUp = iF
                bestDelta = deltaBF
            }

            // B <-> G then C (F B)
            let deltaBG = perimeter(aabbUnion(nodeB.aabb, nodes[Int(iF)].aabb)) - areaC
            if deltaBG < bestDelta {
                bestDown = iB
                bestUp = iG
                bestDelta = deltaBG
            }
        }

        if leafB == false {
            // Swap C with D or E
            let iD = leftChild(nodeB)
            let iE = iD + 1
            let areaB = perimeter(nodeB.aabb)

            // C <-> D then B (C E)
            let deltaCD = perimeter(aabbUnion(nodeC.aabb, nodes[Int(iE)].aabb)) - areaB
            if deltaCD < bestDelta {
                bestDown = iC
                bestUp = iD
                bestDelta = deltaCD
            }

            // C <-> E then B (D C)
            let deltaCE = perimeter(aabbUnion(nodeC.aabb, nodes[Int(iD)].aabb)) - areaB
            if deltaCE < bestDelta {
                bestDown = iC
                bestUp = iE
                bestDelta = deltaCE
            }
        }

        if bestDown != B3.nullIndex {
            swapTreeNodes(bestDown, bestUp)
        }
    }

    // MARK: Insert and remove

    private func insertLeaf(_ aabb: AABB, _ proxyId: Int32, _ moved: Bool, _ shouldRotate: Bool) {
        let proxy = proxies[Int(proxyId)]
        let leaf = makeLeafNode(aabb, proxyId, proxy.userData, moved)

        if isEmptyNode(nodes[Int(rootNode)]) {
            nodes[Int(rootNode)] = leaf
            proxies[Int(proxyId)].node = rootNode
            return
        }

        let sibling = findBestSibling(aabb)

        // The sibling's position becomes the new parent. The sibling moves down into a new pair
        // beside the leaf, which puts it below its own children when it is internal.
        let pair = allocateSiblingPair()

        nodes[Int(pair)] = nodes[Int(sibling)]
        nodes[Int(pair) + 1] = leaf
        parents[Int(pair)] = sibling
        parents[Int(pair) + 1] = sibling
        linkChildren(pair)
        proxies[Int(proxyId)].node = pair + 1

        nodes[Int(sibling)] = makeInternalNode(pair)

        if isNodeOrdered(sibling) == false || isNodeOrdered(pair) == false {
            dfsOrdered = false
        }

        // Walk back up the tree refitting ancestors, the root included.
        var index = sibling
        while index != B3.nullIndex {
            if shouldRotate {
                rotateNodes(index)
            }

            nodes[Int(index)] = makeInternalNode(leftChild(nodes[Int(index)]))
            index = parents[Int(index)]
        }
    }

    private func removeLeaf(_ proxyId: Int32) {
        let leaf = proxies[Int(proxyId)].node
        b3Assert(0 <= leaf && leaf < nodeEnd)
        b3Assert(isLeaf(nodes[Int(leaf)]) && proxyIdOf(nodes[Int(leaf)]) == proxyId)

        if leaf == rootNode {
            nodes[Int(rootNode)] = makeEmptyNode()
            return
        }

        // The sibling takes the parent's position and the pair is freed. This keeps the order,
        // the sibling only moves down.
        let parent = parents[Int(leaf)]
        nodes[Int(parent)] = nodes[Int(leaf ^ 1)]
        linkChildren(parent)
        freePair(leaf & ~1)

        // Update ancestors.
        var index = parents[Int(parent)]
        while index != B3.nullIndex {
            nodes[Int(index)] = makeInternalNode(leftChild(nodes[Int(index)]))
            index = parents[Int(index)]
        }
    }

    /// Create a proxy in the tree as a leaf node.
    public func createProxy(_ aabb: AABB, categoryBits: UInt64, userData: UInt64) -> Int32 {
        b3Validate(isValidAABB(aabb))

        let proxyId = allocateProxy()

        proxies[Int(proxyId)].categoryBits = categoryBits
        proxies[Int(proxyId)].userData = userData

        insertLeaf(aabb, proxyId, false, true)

        return proxyId
    }

    /// Internal proxy creation with the moved flag control.
    func createProxyInternal(_ aabb: AABB, categoryBits: UInt64, userData: UInt64, markMoved: Bool) -> Int32 {
        b3Validate(isValidAABB(aabb))

        let proxyId = allocateProxy()

        proxies[Int(proxyId)].categoryBits = categoryBits
        proxies[Int(proxyId)].userData = userData

        insertLeaf(aabb, proxyId, markMoved, true)

        return proxyId
    }

    /// Destroy a proxy. The proxy id must be valid.
    public func destroyProxy(_ proxyId: Int32) {
        b3Assert(0 <= proxyId && proxyId < proxyCapacity)

        removeLeaf(proxyId)
        freeProxy(proxyId)
    }

    /// Move a proxy to a new AABB.
    func moveProxyInternal(_ proxyId: Int32, _ aabb: AABB, _ markMoved: Bool) {
        b3Validate(isValidAABB(aabb))
        b3Validate(aabb.upperBound.x - aabb.lowerBound.x < B3.huge)
        b3Validate(aabb.upperBound.y - aabb.lowerBound.y < B3.huge)
        b3Validate(aabb.upperBound.z - aabb.lowerBound.z < B3.huge)
        b3Assert(0 <= proxyId && proxyId < proxyCapacity)

        removeLeaf(proxyId)
        insertLeaf(aabb, proxyId, markMoved, false)
    }

    /// Move a proxy to a new AABB.
    public func moveProxy(_ proxyId: Int32, _ aabb: AABB) {
        moveProxyInternal(proxyId, aabb, false)
    }

    /// Enlarge a proxy and enlarge ancestors as necessary.
    public func enlargeProxy(_ proxyId: Int32, _ aabb: AABB) {
        b3Validate(isValidAABB(aabb))
        b3Validate(aabb.upperBound.x - aabb.lowerBound.x < B3.huge)
        b3Validate(aabb.upperBound.y - aabb.lowerBound.y < B3.huge)
        b3Validate(aabb.upperBound.z - aabb.lowerBound.z < B3.huge)
        b3Assert(0 <= proxyId && proxyId < proxyCapacity)

        var index = proxies[Int(proxyId)].node
        b3Assert(isLeaf(nodes[Int(index)]))

        // Caller must ensure this
        b3Validate(aabbContains(nodes[Int(index)].aabb, aabb) == false)

        nodes[Int(index)].aabb = aabb
        nodes[Int(index)].flagIndex |= movedNodeFlag

        index = parents[Int(index)]
        while index != B3.nullIndex {
            let changed = enlargeAABB(&nodes[Int(index)].aabb, aabb)

            // This is marked to ensure the root is marked in this loop or the one below.
            nodes[Int(index)].flagIndex |= movedNodeFlag

            index = parents[Int(index)]

            if changed == false {
                break
            }
        }

        // Mark all the way up to the root.
        while index != B3.nullIndex {
            if nodes[Int(index)].flagIndex & movedNodeFlag != 0 {
                // Early out because this ancestor was previously ascended and marked as moved.
                break
            }

            nodes[Int(index)].flagIndex |= movedNodeFlag
            index = parents[Int(index)]
        }
    }

    /// Modify the category bits on a proxy.
    public func setCategoryBits(_ proxyId: Int32, _ categoryBits: UInt64) {
        b3Assert(0 <= proxyId && proxyId < proxyCapacity)
        proxies[Int(proxyId)].categoryBits = categoryBits
    }

    /// Get the category bits on a proxy.
    public func getCategoryBits(_ proxyId: Int32) -> UInt64 {
        b3Assert(0 <= proxyId && proxyId < proxyCapacity)
        return proxies[Int(proxyId)].categoryBits
    }

    /// Get the user data on a proxy.
    public func getUserData(_ proxyId: Int32) -> UInt64 {
        proxies[Int(proxyId)].userData
    }

    /// Get the AABB of a proxy.
    public func getAABB(_ proxyId: Int32) -> AABB {
        nodes[Int(proxies[Int(proxyId)].node)].aabb
    }

    /// Get the height of the binary tree.
    public var height: Int32 {
        getNodeHeight(nodes[Int(rootNode)])
    }

    /// Get the ratio of the sum of the node areas to the root area.
    public var areaRatio: Float {
        if proxyCount == 0 {
            return 0
        }

        let rootArea = perimeter(nodes[Int(rootNode)].aabb)
        if rootArea <= 0 {
            return 0
        }

        // Free nodes and the empty node are leaf tagged.
        var internalArea: Float = 0
        let end = nodeEnd
        var i: Int32 = 2
        while i < end {
            if isLeaf(nodes[Int(i)]) == false {
                internalArea += perimeter(nodes[Int(i)].aabb)
            }
            i += 1
        }

        return internalArea / rootArea
    }

    /// Get the bounding box that contains the entire tree.
    public var rootBounds: AABB {
        if proxyCount == 0 {
            return AABB(Vec3(), Vec3())
        }

        return nodes[Int(rootNode)].aabb
    }

    /// Get the number of bytes used by this tree.
    public var byteCount: Int32 {
        var size = 256  // approximate C struct size
        size += Int(nodeCapacity) * MemoryLayout<TreeNode>.stride
        size += Int(nodeCapacity) * MemoryLayout<Int32>.stride
        size += Int(proxyCapacity) * MemoryLayout<TreeProxy>.stride
        size += swapNodes.isEmpty ? 0 : Int(nodeCapacity) * MemoryLayout<TreeNode>.stride
        size += Int(rebuildCapacity) * MemoryLayout<Int32>.stride
        size += Int(rebuildCapacity) * MemoryLayout<TreeNode>.stride
        size += Int(rebuildCapacity) * MemoryLayout<Vec3>.stride
        size += Int(rebuildCapacity) * MemoryLayout<Int32>.stride
        return Int32(size)
    }

    // MARK: Queries

    /// Query an AABB for overlapping proxies. The callback is called for each proxy that overlaps
    /// the supplied AABB. Return false to stop the query.
    public func query(
        _ aabb: AABB,
        maskBits: UInt64 = .max,
        requireAllBits: Bool = false,
        _ callback: (Int32, UInt64) -> Bool
    ) -> TreeStats {
        withView { view in view.query(aabb, maskBits: maskBits, requireAllBits: requireAllBits, callback) }
    }

    /// Query an AABB for the closest object.
    public func queryClosest(
        _ point: Vec3,
        maskBits: UInt64 = .max,
        requireAllBits: Bool = false,
        minDistanceSqr: Float = .greatestFiniteMagnitude,
        _ callback: (Float, Int32, UInt64) -> Float
    ) -> (stats: TreeStats, minDistanceSqr: Float) {
        withView { view in view.queryClosest(point, maskBits: maskBits, requireAllBits: requireAllBits, minDistanceSqr: minDistanceSqr, callback) }
    }

    /// Ray cast against the proxies in the tree.
    public func rayCast(
        _ input: RayCastInput,
        maskBits: UInt64 = .max,
        requireAllBits: Bool = false,
        _ callback: (inout RayCastInput, Int32, UInt64) -> Float
    ) -> TreeStats {
        withView { view in view.rayCast(input, maskBits: maskBits, requireAllBits: requireAllBits, callback) }
    }

    /// Sweep an AABB through the tree.
    public func boxCast(
        _ input: BoxCastInput,
        maskBits: UInt64 = .max,
        requireAllBits: Bool = false,
        _ callback: (inout BoxCastInput, Int32, UInt64) -> Float
    ) -> TreeStats {
        withView { view in view.boxCast(input, maskBits: maskBits, requireAllBits: requireAllBits, callback) }
    }

    private func withView<R>(_ body: (TreeView) -> R) -> R {
        nodes.withUnsafeBufferPointer { nodes in
            proxies.withUnsafeBufferPointer { proxies in
                body(TreeView(nodes: nodes, proxies: proxies))
            }
        }
    }

    // MARK: Validation

    /// Validate the tree structure. For testing.
    public func validate() {
        b3Assert(2 <= nodeEnd && nodeEnd <= nodeCapacity)
        b3Assert((nodeEnd & 1) == 0)
        b3Assert(parents[Int(rootNode)] == B3.nullIndex)
        b3Assert(isEmptyNode(nodes[Int(rootNode) + 1]))

        var freePairCount = 0
        var pair = pairFreeList
        while pair != B3.nullIndex {
            b3Assert((pair & 1) == 0 && 2 <= pair && pair < nodeEnd)
            b3Assert(isEmptyNode(nodes[Int(pair)]))
            b3Assert(isEmptyNode(nodes[Int(pair) + 1]))
            pair = parents[Int(pair)]
            freePairCount += 1
            b3Assert(2 * freePairCount < nodeEnd)
        }

        // Validate proxy free list.
        var freeProxyCount = 0
        var freeIndex = proxyFreeList
        while freeIndex != B3.nullIndex {
            b3Assert(0 <= freeIndex && freeIndex < proxyCapacity)
            b3Assert(proxies[Int(freeIndex)].node == B3.nullIndex)
            freeIndex = proxies[Int(freeIndex)].next
            freeProxyCount += 1
        }
        b3Assert(Int(proxyCount) + Int(freeProxyCount) == Int(proxyCapacity))

        b3Assert(nodeEnd == 2 * maxInt(proxyCount, 1) + 2 * Int32(freePairCount))

        if proxyCount == 0 {
            b3Assert(isEmptyNode(nodes[Int(rootNode)]))
            return
        }

        var leafCount = 0
        _ = validateSubtree(rootNode, &leafCount)
        b3Assert(leafCount == Int(proxyCount))
    }

    private func validateSubtree(_ nodeIndex: Int32, _ leafCount: inout Int) -> Int32 {
        b3Assert(0 <= nodeIndex && nodeIndex < nodeEnd)
        let node = nodes[Int(nodeIndex)]
        b3Assert(isEmptyNode(node) == false)
        b3Assert(isValidAABB(node.aabb))

        if isLeaf(node) {
            let proxyId = proxyIdOf(node)
            b3Assert(0 <= proxyId && proxyId < proxyCapacity)
            b3Assert(proxies[Int(proxyId)].node == nodeIndex)
            b3Assert(Int32(truncatingIfNeeded: proxies[Int(proxyId)].userData) == node.shapeIndex)
            leafCount += 1
            return 0
        }

        let pair = leftChild(node)
        b3Assert((pair & 1) == 0 && 2 <= pair && pair < nodeEnd)
        b3Assert(parents[Int(pair)] == nodeIndex)
        b3Assert(parents[Int(pair) + 1] == nodeIndex)
        b3Assert(dfsOrdered == false || nodeIndex < pair)

        let c1 = nodes[Int(pair)]
        let c2 = nodes[Int(pair) + 1]
        b3Assert(aabbContains(node.aabb, c1.aabb))
        b3Assert(aabbContains(node.aabb, c2.aabb))
        b3Assert(isNodeMoved(node) == (isNodeMoved(c1) || isNodeMoved(c2)))

        // A bad tree can stack overflow, but that is validation on its own.
        let height1 = validateSubtree(pair, &leafCount)
        let height2 = validateSubtree(pair + 1, &leafCount)
        let height = 1 + maxInt(height1, height2)
        b3Assert(node.height == height)
        return height
    }

    /// Validate this tree has no moved nodes. For testing.
    public func validateNoMoved() {
        let end = nodeEnd
        var i: Int32 = 0
        while i < end {
            b3Assert(isNodeMoved(nodes[Int(i)]) == false)
            i += 1
        }
    }

    // MARK: Rebuild

    /// Split indices by the median along the longest axis of the centroid bounds.
    private func partitionMid(_ indices: inout [Int32], _ centers: inout [Vec3], _ start: Int, _ count: Int) -> Int {
        // Handle trivial case
        if count <= 2 {
            return count / 2
        }

        var lowerBound = centers[start]
        var upperBound = centers[start]

        for i in 1..<count {
            lowerBound = minVec3(lowerBound, centers[start + i])
            upperBound = maxVec3(upperBound, centers[start + i])
        }

        let d = upperBound - lowerBound
        let c = 0.5 * (lowerBound + upperBound)

        // Partition longest axis using the Hoare partition scheme
        var i1 = 0
        var i2 = count

        @inline(__always)
        func swapAt(_ a: Int, _ b: Int) {
            indices.swapAt(start + a, start + b)
            centers.swapAt(start + a, start + b)
        }

        if d.x >= d.y && d.x >= d.z {
            let pivot = c.x

            while i1 < i2 {
                while i1 < i2 && centers[start + i1].x < pivot {
                    i1 += 1
                }

                while i1 < i2 && centers[start + i2 - 1].x >= pivot {
                    i2 -= 1
                }

                if i1 < i2 {
                    swapAt(i1, i2 - 1)
                    i1 += 1
                    i2 -= 1
                }
            }
        } else if d.y >= d.z {
            let pivot = c.y

            while i1 < i2 {
                while i1 < i2 && centers[start + i1].y < pivot {
                    i1 += 1
                }

                while i1 < i2 && centers[start + i2 - 1].y >= pivot {
                    i2 -= 1
                }

                if i1 < i2 {
                    swapAt(i1, i2 - 1)
                    i1 += 1
                    i2 -= 1
                }
            }
        } else {
            let pivot = c.z

            while i1 < i2 {
                while i1 < i2 && centers[start + i1].z < pivot {
                    i1 += 1
                }

                while i1 < i2 && centers[start + i2 - 1].z >= pivot {
                    i2 -= 1
                }

                if i1 < i2 {
                    swapAt(i1, i2 - 1)
                    i1 += 1
                    i2 -= 1
                }
            }
        }
        b3Assert(i1 == i2)

        if i1 > 0 && i1 < count {
            return i1
        }

        return count / 2
    }

    @inline(__always)
    private func bumpPair(_ parent: Int32) -> Int32 {
        let pair = nodeEnd
        b3Assert(pair + 2 <= nodeCapacity)
        nodeEnd += 2
        parents[Int(pair)] = parent
        parents[Int(pair) + 1] = parent
        return pair
    }

    @inline(__always)
    private static func setLeftChild(_ node: inout TreeNode, _ pair: Int32) {
        node.flagIndex = (node.flagIndex & ~nodeIndexMask) | UInt32(pair)
    }

    private func copySubtree(_ node: TreeNode, _ newIndex: Int32) {
        if isLeaf(node) {
            swapNodes[Int(newIndex)] = node
            proxies[Int(proxyIdOf(node))].node = newIndex
            return
        }

        if copyStack.count < treeStackSize {
            copyStack = Array(repeating: (0, 0), count: treeStackSize)
        }
        var stackCount = 0

        var oldPair = leftChild(node)
        var newPair = bumpPair(newIndex)
        var rootNodeCopy = node
        Self.setLeftChild(&rootNodeCopy, newPair)

        // Copy the subtree root. This can be a left or right sibling (even or odd).
        swapNodes[Int(newIndex)] = rootNodeCopy

        while true {
            let pairBase = Int(newPair)

            // Copy the siblings.
            swapNodes[pairBase] = nodes[Int(oldPair)]
            swapNodes[pairBase + 1] = nodes[Int(oldPair) + 1]

            if isLeaf(swapNodes[pairBase + 1]) {
                // Hook up proxy on right sibling.
                let proxyId = proxyIdOf(swapNodes[pairBase + 1])
                proxies[Int(proxyId)].node = newPair + 1
            } else {
                // Push the left child of the right sibling.
                if stackCount < treeStackSize {
                    let leftChildIndex = leftChild(swapNodes[pairBase + 1])
                    copyStack[stackCount] = (oldPair: leftChildIndex, newIndex: newPair + 1)
                    stackCount += 1
                } else {
                    b3Assert(stackCount < treeStackSize)
                }
            }

            var leftNode = swapNodes[pairBase]
            if isLeaf(leftNode) == false {
                // Descend into left subtree.
                let leftIndex = newPair
                oldPair = leftChild(leftNode)
                newPair = bumpPair(leftIndex)
                Self.setLeftChild(&leftNode, newPair)
                swapNodes[Int(leftIndex)] = leftNode
                continue
            }

            // Hook up proxy on left sibling.
            proxies[Int(proxyIdOf(leftNode))].node = newPair

            if stackCount == 0 {
                break
            }

            // Descend a right subtree that was previously pushed.
            stackCount -= 1
            let item = copyStack[stackCount]
            oldPair = item.oldPair
            newPair = bumpPair(item.newIndex)
            var rightNode = swapNodes[Int(item.newIndex)]
            Self.setLeftChild(&rightNode, newPair)
            swapNodes[Int(item.newIndex)] = rightNode
        }
    }

    private func placeLeaf(_ leaf: TreeNode, _ newIndex: Int32) {
        if isLeaf(leaf) {
            swapNodes[Int(newIndex)] = leaf
            proxies[Int(proxyIdOf(leaf))].node = newIndex
        } else {
            copySubtree(leaf, newIndex)
        }
    }

    private func buildTree(_ leafCount: Int32) {
        let leaves = leafNodes

        // Bump allocation into the swap nodes.
        nodeEnd = 2
        swapNodes[Int(rootNode) + 1] = makeEmptyNode()
        parents[Int(rootNode)] = B3.nullIndex
        parents[Int(rootNode) + 1] = B3.nullIndex

        if leafCount == 1 {
            placeLeaf(leaves[Int(leafIndices[0])], rootNode)
            return
        }

        var stack = [(nodeIndex: Int32, pair: Int32, childCount: Int32, startIndex: Int, endIndex: Int, splitIndex: Int)](
            repeating: (0, 0, 0, 0, 0, 0),
            count: treeStackSize
        )
        var top = 0

        stack[0].nodeIndex = rootNode
        stack[0].pair = bumpPair(rootNode)
        stack[0].childCount = -1
        stack[0].startIndex = 0
        stack[0].endIndex = Int(leafCount)
        stack[0].splitIndex = stack[0].startIndex + partitionMid(&leafIndices, &leafCenters, 0, Int(leafCount))

        while true {
            stack[top].childCount += 1

            if stack[top].childCount == 2 {
                // Both children written, so the parent node can be finalized.
                swapNodes[Int(stack[top].nodeIndex)] = makeInternalNode(swapNodes, stack[top].pair)
                if top == 0 {
                    break
                }

                top -= 1
                continue
            }

            let slot = stack[top].childCount
            let startIndex = slot == 0 ? stack[top].startIndex : stack[top].splitIndex
            let endIndex = slot == 0 ? stack[top].splitIndex : stack[top].endIndex
            let count = endIndex - startIndex
            b3Assert(count > 0)

            let nodeIndex = stack[top].pair + slot
            if count == 1 {
                placeLeaf(leaves[Int(leafIndices[startIndex])], nodeIndex)
                continue
            }

            b3Assert(top < treeStackSize - 1)

            top += 1
            stack[top].nodeIndex = nodeIndex
            stack[top].pair = bumpPair(nodeIndex)
            stack[top].childCount = -1
            stack[top].startIndex = startIndex
            stack[top].endIndex = endIndex
            stack[top].splitIndex = startIndex + partitionMid(&leafIndices, &leafCenters, startIndex, count)
        }
    }

    /// Rebuild the stale parts of the tree. The entire tree is put into DFS order.
    @discardableResult
    public func rebuild(fullBuild: Bool) -> Int32 {
        let count = proxyCount
        if count == 0 {
            return 0
        }

        // An unordered tree is rebuilt even when nothing moved, the sweep refit needs the order
        let root = nodes[Int(rootNode)]
        if fullBuild == false && isNodeMoved(root) == false && dfsOrdered {
            return 0
        }

        if swapNodes.isEmpty {
            swapNodes = Array(repeating: TreeNode(), count: Int(nodeCapacity))
        }

        if count > rebuildCapacity {
            let newCapacity = count + count / 2
            leafIndices = Array(repeating: 0, count: Int(newCapacity))
            leafNodes = Array(repeating: TreeNode(), count: Int(newCapacity))
            leafCenters = Array(repeating: Vec3(), count: Int(newCapacity))
            rebuildCapacity = newCapacity
        }

        // Gather build leaves. A marked internal node is descended and abandoned in the old array.
        // Everything else is a build leaf, kept subtrees included.
        var leafCount: Int32 = 0
        var stack = [Int32](repeating: 0, count: treeStackSize)
        var stackCount = 0

        if isLeaf(root) == false && (fullBuild || isNodeMoved(root)) {
            stack[stackCount] = leftChild(root)
            stackCount += 1
        } else {
            var node = root
            if fullBuild == false {
                node.flagIndex &= ~movedNodeFlag
            }
            leafIndices[0] = 0
            leafNodes[0] = node
            leafCenters[0] = aabbCenter(node.aabb)
            leafCount = 1
        }

        while stackCount > 0 {
            stackCount -= 1
            let pair = stack[stackCount]
            for i in 0..<2 {
                var node = nodes[Int(pair) + i]
                if isLeaf(node) == false && (fullBuild || isNodeMoved(node)) {
                    b3Assert(stackCount < treeStackSize)
                    stack[stackCount] = leftChild(node)
                    stackCount += 1
                    continue
                }

                if fullBuild == false {
                    node.flagIndex &= ~movedNodeFlag
                }
                leafIndices[Int(leafCount)] = leafCount
                leafNodes[Int(leafCount)] = node
                leafCenters[Int(leafCount)] = aabbCenter(node.aabb)
                leafCount += 1
            }
        }

        b3Assert(0 < leafCount && leafCount <= count)

        buildTree(leafCount)

        // The spare is now the tree and the old array is the spare.
        swap(&nodes, &swapNodes)
        pairFreeList = B3.nullIndex
        dfsOrdered = true

        #if DEBUG
        validate()
        if fullBuild == false {
            validateNoMoved()
        }
        #endif

        return leafCount
    }

    // MARK: Moved tracking

    /// Set the moved flag on the ancestors of the proxy. Serial use case.
    func markProxyMovedSerial(_ proxyId: Int32) {
        b3Validate(0 <= proxyId && proxyId < proxyCapacity)

        var index = proxies[Int(proxyId)].node
        b3Validate(0 <= index && index < nodeEnd)
        b3Validate(isLeaf(nodes[Int(index)]))

        while index != B3.nullIndex {
            nodes[Int(index)].flagIndex |= movedNodeFlag
            index = parents[Int(index)]
        }
    }

    /// Update a proxy AABB and flag the ancestors as moved.
    func markProxyMoved(_ proxyId: Int32, _ aabb: AABB) {
        b3Validate(0 <= proxyId && proxyId < proxyCapacity)

        var index = proxies[Int(proxyId)].node
        b3Validate(0 <= index && index < nodeEnd)

        b3Validate(isLeaf(nodes[Int(index)]))
        b3Validate(aabbContains(nodes[Int(index)].aabb, aabb) == false)

        // This is not raced since it is the leaf.
        nodes[Int(index)].aabb = aabb
        nodes[Int(index)].flagIndex |= movedNodeFlag

        index = parents[Int(index)]
        while index != B3.nullIndex {
            // Read first to avoid the fetch-or if possible.
            if nodes[Int(index)].flagIndex & movedNodeFlag != 0 {
                break
            }

            let previousFlags = nodes[Int(index)].flagIndex
            nodes[Int(index)].flagIndex |= movedNodeFlag
            if previousFlags & movedNodeFlag != 0 {
                // Ancestor already visited.
                break
            }

            index = parents[Int(index)]
        }
    }

    /// Clear the moved flags from the entire tree.
    func clearMoved() {
        if isNodeMoved(nodes[Int(rootNode)]) == false {
            return
        }

        nodes[Int(rootNode)].flagIndex &= ~movedNodeFlag
        if isLeaf(nodes[Int(rootNode)]) {
            return
        }

        var stack = [Int32](repeating: 0, count: treeStackSize)
        var stackCount = 0
        stack[stackCount] = leftChild(nodes[Int(rootNode)])
        stackCount += 1

        while stackCount > 0 {
            stackCount -= 1
            let pair = stack[stackCount]
            for i in 0..<2 {
                let nodeIndex = Int(pair) + i
                if nodes[nodeIndex].flagIndex & movedNodeFlag != 0 {
                    nodes[nodeIndex].flagIndex &= ~movedNodeFlag
                    if isLeaf(nodes[nodeIndex]) == false {
                        if stackCount < treeStackSize {
                            stack[stackCount] = leftChild(nodes[nodeIndex])
                            stackCount += 1
                        } else {
                            b3Assert(stackCount < treeStackSize)
                        }
                    }
                }
            }
        }
    }

    /// Gather the proxies with a moved flag set.
    func gatherMovedProxies(_ proxyIds: inout [Int32]) -> Int32 {
        let root = nodes[Int(rootNode)]
        if isNodeMoved(root) == false {
            return 0
        }

        if isLeaf(root) {
            proxyIds[0] = proxyIdOf(root)
            return 1
        }

        var count: Int32 = 0
        var stack = [Int32](repeating: 0, count: treeStackSize)
        var stackCount = 0
        stack[stackCount] = leftChild(root)
        stackCount += 1

        while stackCount > 0 {
            stackCount -= 1
            let pair = stack[stackCount]
            for i in 0..<2 {
                let node = nodes[Int(pair) + i]
                if isNodeMoved(node) == false {
                    continue
                }

                if isLeaf(node) {
                    proxyIds[Int(count)] = proxyIdOf(node)
                    count += 1
                } else {
                    if stackCount < treeStackSize {
                        stack[stackCount] = leftChild(node)
                        stackCount += 1
                    } else {
                        b3Assert(stackCount < treeStackSize)
                    }
                }
            }
        }

        return count
    }

    /// Slow refit for unit tests.
    private func refitSubtree(_ rootIndex: Int32) {
        let root = nodes[Int(rootIndex)]
        if isLeaf(root) || isNodeMoved(root) == false {
            return
        }

        var stack = [Int32](repeating: 0, count: treeStackSize)
        var stackCount = 0
        stack[stackCount] = ~rootIndex
        stackCount += 1
        stack[stackCount] = leftChild(root)
        stackCount += 1
        stack[stackCount] = leftChild(root) + 1
        stackCount += 1

        while stackCount > 0 {
            stackCount -= 1
            let item = stack[stackCount]
            if item < 0 {
                let nodeIndex = ~item
                let pair = leftChild(nodes[Int(nodeIndex)])
                nodes[Int(nodeIndex)].aabb = aabbUnion(nodes[Int(pair)].aabb, nodes[Int(pair) + 1].aabb)
                continue
            }

            if isLeaf(nodes[Int(item)]) || isNodeMoved(nodes[Int(item)]) == false {
                continue
            }

            if stackCount + 3 <= treeStackSize {
                let pair = leftChild(nodes[Int(item)])
                stack[stackCount] = ~item
                stackCount += 1
                stack[stackCount] = pair
                stackCount += 1
                stack[stackCount] = pair + 1
                stackCount += 1
            } else {
                b3Assert(stackCount + 3 <= treeStackSize)
            }
        }
    }

    /// Refit the tree using the moved flags. This requires DFS order.
    func refit() {
        if isNodeMoved(nodes[Int(rootNode)]) == false {
            return
        }

        if dfsOrdered == false {
            // This only happens in unit tests.
            refitSubtree(rootNode)
            return
        }

        var pair = nodeEnd - 2
        while pair >= 0 {
            let base = Int(pair)
            let flags1 = nodes[base].flagIndex
            let flags2 = nodes[base + 1].flagIndex

            // Did either move?
            if ((flags1 | flags2) & movedNodeFlag) == 0 {
                pair -= 2
                continue
            }

            // Is the node internal and moved?
            let refit1 = (flags1 & (leafNodeFlag | movedNodeFlag)) == movedNodeFlag
            let refit2 = (flags2 & (leafNodeFlag | movedNodeFlag)) == movedNodeFlag

            // If the node needs a refit then get the child index. Otherwise use a valid dummy index.
            let children1 = refit1 ? Int32(flags1 & nodeIndexMask) : pair
            let children2 = refit2 ? Int32(flags2 & nodeIndexMask) : pair

            // Conditionally store the union.
            if refit1 {
                nodes[base].aabb = aabbUnion(nodes[Int(children1)].aabb, nodes[Int(children1) + 1].aabb)
            }
            if refit2 {
                nodes[base + 1].aabb = aabbUnion(
                    nodes[Int(children2)].aabb, nodes[Int(children2) + 1].aabb
                )
            }

            pair -= 2
        }
    }

    // MARK: State

    var hasMoved: Bool {
        isNodeMoved(nodes[Int(rootNode)])
    }

    var needsRebuild: Bool {
        isNodeMoved(nodes[Int(rootNode)]) || dfsOrdered == false
    }

    // MARK: File IO

    /// Save this tree to a file for debugging.
    public func save(to fileName: String) {
        var data = Data()

        var header = TreeFileHeader()
        header.version = version
        header.nodeEnd = nodeEnd
        header.nodeCapacity = nodeEnd
        header.proxyCount = proxyCount
        header.proxyCapacity = proxyCapacity
        header.pairFreeList = pairFreeList
        header.proxyFreeList = proxyFreeList
        header.dfsOrdered = dfsOrdered ? 1 : 0

        let headerSize = MemoryLayout<TreeFileHeader>.size
        let headerStride = MemoryLayout<TreeFileHeader>.stride
        withUnsafeBytes(of: &header) { data.append(contentsOf: $0) }
        if headerStride > headerSize {
            data.append(contentsOf: [UInt8](repeating: 0, count: headerStride - headerSize))
        }

        nodes.withUnsafeBufferPointer { buffer in
            data.append(contentsOf: UnsafeRawBufferPointer(start: buffer.baseAddress, count: Int(nodeEnd) * MemoryLayout<TreeNode>.stride))
        }
        parents.withUnsafeBufferPointer { buffer in
            data.append(contentsOf: UnsafeRawBufferPointer(start: buffer.baseAddress, count: Int(nodeEnd) * MemoryLayout<Int32>.stride))
        }
        proxies.withUnsafeBufferPointer { buffer in
            data.append(contentsOf: UnsafeRawBufferPointer(
                start: buffer.baseAddress, count: Int(proxyCapacity) * MemoryLayout<TreeProxy>.stride
            ))
        }

        try? data.write(to: URL(fileURLWithPath: fileName))
    }

    /// Load a tree from a file for debugging.
    public static func load(from fileName: String, scale: Float) -> DynamicTree? {
        guard let data = try? Data(contentsOf: URL(fileURLWithPath: fileName)) else {
            return nil
        }

        return data.withUnsafeBytes { raw -> DynamicTree? in
            let stride = MemoryLayout<TreeFileHeader>.stride
            guard raw.count >= stride else {
                return nil
            }

            let header = raw.load(fromByteOffset: 0, as: TreeFileHeader.self)
            guard header.version == B3.dynamicTreeVersion else {
                return nil
            }

            let nodeEnd = header.nodeEnd
            let nodeCapacity = header.nodeCapacity
            let proxyCapacity = header.proxyCapacity

            let ok = nodeCapacity >= 2 && proxyCapacity >= 1 && (nodeEnd & 1) == 0 && nodeEnd >= 2
                && nodeEnd <= nodeCapacity
            guard ok else {
                return nil
            }

            let nodeBytes = Int(nodeCapacity) * MemoryLayout<TreeNode>.stride
            let parentBytes = Int(nodeCapacity) * MemoryLayout<Int32>.stride
            let proxyBytes = Int(proxyCapacity) * MemoryLayout<TreeProxy>.stride
            guard raw.count >= stride + nodeBytes + parentBytes + proxyBytes else {
                return nil
            }

            let tree = DynamicTree(proxyCapacity: proxyCapacity)
            tree.nodes = Array(repeating: TreeNode(), count: Int(nodeCapacity))
            tree.parents = Array(repeating: 0, count: Int(nodeCapacity))
            tree.proxies = Array(repeating: TreeProxy(), count: Int(proxyCapacity))

            let nodeOffset = stride
            let parentOffset = nodeOffset + nodeBytes
            let proxyOffset = parentOffset + parentBytes

            for i in 0..<Int(nodeCapacity) {
                tree.nodes[i] = raw.load(fromByteOffset: nodeOffset + i * MemoryLayout<TreeNode>.stride, as: TreeNode.self)
            }
            for i in 0..<Int(nodeCapacity) {
                tree.parents[i] = raw.load(
                    fromByteOffset: parentOffset + i * MemoryLayout<Int32>.stride, as: Int32.self
                )
            }
            for i in 0..<Int(proxyCapacity) {
                tree.proxies[i] = raw.load(
                    fromByteOffset: proxyOffset + i * MemoryLayout<TreeProxy>.stride, as: TreeProxy.self
                )
            }

            tree.nodeEnd = nodeEnd
            tree.nodeCapacity = nodeCapacity
            tree.proxyCapacity = proxyCapacity
            tree.proxyCount = header.proxyCount
            tree.pairFreeList = header.pairFreeList
            tree.proxyFreeList = header.proxyFreeList
            tree.dfsOrdered = header.dfsOrdered != 0
            tree.rebuildCapacity = 0

            // An empty node has infinite bounds, so the scale has to skip it.
            for i in 0..<Int(nodeEnd) {
                if isEmptyNode(tree.nodes[i]) {
                    continue
                }

                tree.nodes[i].aabb.lowerBound = scale * tree.nodes[i].aabb.lowerBound
                tree.nodes[i].aabb.upperBound = scale * tree.nodes[i].aabb.upperBound
            }

            return tree
        }
    }
}

/// On-disk header for `DynamicTree.save` / `DynamicTree.load`.
struct TreeFileHeader: BitwiseCopyable {
    var version: UInt64 = 0
    var nodeEnd: Int32 = 0
    var nodeCapacity: Int32 = 0
    var proxyCount: Int32 = 0
    var proxyCapacity: Int32 = 0
    var pairFreeList: Int32 = 0
    var proxyFreeList: Int32 = 0
    var dfsOrdered: Int32 = 0

    init() {}
}
