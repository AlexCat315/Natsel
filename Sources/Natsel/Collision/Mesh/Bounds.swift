struct MeshQueryBounds {
    let lower: Vec3
    let upper: Vec3
    var center: Vec3 { 0.5 * (lower + upper) }
    var extent: Vec3 { upper - center }

    init(bounds: AABB, scale: Vec3) {
        let inverse = Vec3(1 / scale.x, 1 / scale.y, 1 / scale.z)
        let a = inverse * bounds.lowerBound
        let b = inverse * bounds.upperBound
        lower = minVec3(a, b)
        upper = maxVec3(a, b)
    }
}

/// Visits candidate triangles in the C query's left-first depth-first order.
/// The callback performs the triangle SAT test, preserving its caller's vertex order.
func visitMeshCandidates(_ data: TriangleMesh, _ bounds: MeshQueryBounds,
                         _ callback: (Int, MeshTriangle) -> Bool) {
    let nodes = data.nodes
    let triangles = data.triangles
    // 128 entries covers any balanced BVH depth; keep it under the 1 KB threshold so Swift's
    // temporary allocation stays on the stack.
    withUnsafeTemporaryAllocation(of: Int.self, capacity: 128) { stack in
        var stackCount = 0
        stack[stackCount] = 0
        stackCount += 1
        while stackCount > 0 {
            stackCount -= 1
            let index = stack[stackCount]
            let node = nodes[index]
            if !boundsOverlap(node.lowerBound, node.upperBound, bounds.lower, bounds.upper) { continue }
            if node.isLeaf {
                let offset = Int(node.triangleOffset)
                for triangleIndex in offset..<(offset + node.leafTriangleCount) {
                    if !callback(triangleIndex, triangles[triangleIndex]) { return }
                }
            } else {
                if stackCount + 2 <= 128 {
                    stack[stackCount] = index + node.childOffset
                    stack[stackCount + 1] = index + 1
                    stackCount += 2
                }
            }
        }
    }
}
