extension MeshNode {
    var isLeaf: Bool { data & 3 == 3 }
    var childOffset: Int { Int(data >> 2) }
    var leafTriangleCount: Int { Int(data >> 2) }
}

struct MeshTreeBuild {
    var nodes: [MeshNode] = []
    var height = 0

    init(primitives: inout [MeshPrimitive], useMedianSplit: Bool) {
        precondition(!primitives.isEmpty)
        nodes.reserveCapacity(2 * primitives.count - 1)
        var treeHeight = 0
        _ = build(&primitives, range: primitives.indices, useMedianSplit: useMedianSplit, height: &treeHeight)
        height = treeHeight
    }

    private mutating func build(
        _ primitives: inout [MeshPrimitive], range: Range<Int>, useMedianSplit: Bool, height: inout Int
    ) -> Int {
        if range.count > 4 {
            var split = useMedianSplit
                ? splitMeshMedian(&primitives, range: range)
                : splitMeshBinnedSAH(&primitives, range: range)
            if split.axis < 0 {
                if range.count > 8 {
                    split = splitMeshHalf(primitives, range: range)
                } else {
                    // C leaves the caller's zero height unchanged on this path.
                    return storeLeaf(primitives, range: range)
                }
            }
            let index = nodes.count
            nodes.append(MeshNode())
            var leftHeight = 0
            var rightHeight = 0
            let middle = range.lowerBound + split.index
            let left = build(&primitives, range: range.lowerBound..<middle,
                             useMedianSplit: useMedianSplit, height: &leftHeight)
            let right = build(&primitives, range: middle..<range.upperBound,
                              useMedianSplit: useMedianSplit, height: &rightHeight)
            height = max(leftHeight, rightHeight) + 1
            b3Assert(left - index == 1 && right - index > 1)
            let bounds = aabbUnion(split.leftBounds, split.rightBounds)
            nodes[index].data = UInt32(split.axis) | (UInt32(right - index) << 2)
            nodes[index].lowerBound = bounds.lowerBound
            nodes[index].upperBound = bounds.upperBound
            nodes[index].triangleOffset = 0
            return index
        }
        let index = storeLeaf(primitives, range: range)
        height = 1
        return index
    }

    private mutating func storeLeaf(_ primitives: [MeshPrimitive], range: Range<Int>) -> Int {
        var bounds = bounds3Empty
        for i in range { bounds = aabbUnion(bounds, primitives[i].bounds) }
        var node = MeshNode()
        node.lowerBound = bounds.lowerBound
        node.upperBound = bounds.upperBound
        node.data = 3 | UInt32(range.count) << 2
        node.triangleOffset = UInt32(range.lowerBound)
        let index = nodes.count
        nodes.append(node)
        return index
    }
}
