/// A refittable feature BVH. Swept faces and edges appear once even when their motion spans
/// many spatial cells. Reuses the mesh builder and keeps the original feature indices.
struct SoftContactTree {
    private var nodes: [MeshNode] = []
    private var features: [Int32] = []
    private var escape: [Int] = []
    private var maximumFeature: [Int32] = []
    private var boxes: [AABB] = []
    private var referenceNodes: [AABB] = []

    mutating func refit(_ bounds: [AABB], referenceBounds: () -> [AABB] = { [] }) {
        let rebuild = bounds.count != boxes.count || nodes.isEmpty
        boxes = bounds
        guard !bounds.isEmpty else {
            nodes.removeAll(); features.removeAll(); escape.removeAll(); maximumFeature.removeAll(); referenceNodes.removeAll()
            return
        }
        if rebuild {
            var primitives = bounds.enumerated().map {
                MeshPrimitive(bounds: $0.element, center: aabbCenter($0.element), triangleIndex: $0.offset)
            }
            nodes = MeshTreeBuild(primitives: &primitives, useMedianSplit: true).nodes
            features = primitives.map { Int32($0.triangleIndex) }
            escape = [Int](repeating: 0, count: nodes.count)
            maximumFeature = [Int32](repeating: -1, count: nodes.count)
            for index in nodes.indices.reversed() {
                let node = nodes[index]
                escape[index] = node.isLeaf ? index + 1 : escape[index + node.childOffset]
                if node.isLeaf {
                    let start = Int(node.triangleOffset)
                    for slot in start..<(start + node.leafTriangleCount) { maximumFeature[index] = max(maximumFeature[index], features[slot]) }
                } else {
                    maximumFeature[index] = max(maximumFeature[index + 1], maximumFeature[index + node.childOffset])
                }
            }
            let reference = referenceBounds()
            referenceNodes.removeAll(keepingCapacity: true)
            if reference.count == bounds.count {
                referenceNodes = [AABB](repeating: bounds3Empty, count: nodes.count)
                for index in nodes.indices.reversed() {
                    let node = nodes[index]
                    if node.isLeaf {
                        let start = Int(node.triangleOffset)
                        var box = bounds3Empty
                        for slot in start..<(start + node.leafTriangleCount) {
                            box = aabbUnion(box, reference[Int(features[slot])])
                        }
                        referenceNodes[index] = box
                    } else {
                        referenceNodes[index] = aabbUnion(referenceNodes[index + 1], referenceNodes[index + node.childOffset])
                    }
                }
            }
            return
        }
        let features = features
        nodes.withUnsafeMutableBufferPointer { buffer in
            for index in buffer.indices.reversed() {
                let node = buffer[index]
                var box = bounds3Empty
                if node.isLeaf {
                    let start = Int(node.triangleOffset)
                    for slot in start..<(start + node.leafTriangleCount) {
                        box = aabbUnion(box, bounds[Int(features[slot])])
                    }
                } else {
                    let left = buffer[index + 1], right = buffer[index + node.childOffset]
                    box = aabbUnion(AABB(left.lowerBound, left.upperBound), AABB(right.lowerBound, right.upperBound))
                }
                buffer[index].lowerBound = box.lowerBound
                buffer[index].upperBound = box.upperBound
            }
        }
    }

    @inline(__always)
    func visit(_ query: AABB, afterFeature: Int32 = -1, referenceExclusion: (center: Vec3, radiusSquared: Float)? = nil, _ body: (Int32) -> Void) {
        var index = 0
        while index < nodes.count {
            let node = nodes[index]
            if maximumFeature[index] <= afterFeature {
                index = escape[index]
                continue
            }
            if !aabbOverlaps(query, AABB(node.lowerBound, node.upperBound)) {
                index = escape[index]
                continue
            }
            if !node.isLeaf, let exclusion = referenceExclusion, referenceNodes.count == nodes.count {
                let reference = referenceNodes[index]
                let farthest = maxVec3(absVec3(reference.lowerBound - exclusion.center), absVec3(reference.upperBound - exclusion.center))
                // Every point in this rest-space node is already excluded. Keep a roundoff margin
                // at the exclusion boundary; folded surfaces with distant rest positions survive.
                if lengthSquared(farthest) < exclusion.radiusSquared * (1 - 16 * Float.ulpOfOne) {
                    index = escape[index]
                    continue
                }
            }
            if node.isLeaf {
                let start = Int(node.triangleOffset)
                for slot in start..<(start + node.leafTriangleCount) {
                    let feature = features[slot]
                    if feature > afterFeature && aabbOverlaps(query, boxes[Int(feature)]) { body(feature) }
                }
            }
            index += 1
        }
    }
}
