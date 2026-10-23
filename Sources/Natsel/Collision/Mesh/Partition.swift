struct MeshPrimitive {
    var bounds: AABB
    var center: Vec3
    var triangleIndex: Int
}

struct MeshSplit {
    var leftBounds = bounds3Empty
    var rightBounds = bounds3Empty
    var axis = -1
    var index = -1
}

private struct MeshBucket {
    var count = 0
    var bounds = bounds3Empty
}

func splitMeshBinnedSAH(_ primitives: inout [MeshPrimitive], range: Range<Int>) -> MeshSplit {
    var split = MeshSplit()
    let first = primitives[range.lowerBound].center
    var bounds = AABB(first, first)
    for i in range.dropFirst() { bounds = aabbAddPoint(bounds, primitives[i].center) }
    var bestBucket = -1
    var bestCost = Float.greatestFiniteMagnitude
    for axis in 0..<3 {
        if getByIndex(aabbExtents(bounds), axis) < B3.linearSlop { continue }
        var buckets = [MeshBucket](repeating: MeshBucket(), count: 8)
        let factor = Float(8) * (1 - Float.ulpOfOne)
            / (getByIndex(bounds.upperBound, axis) - getByIndex(bounds.lowerBound, axis))
        for i in range {
            let index = Int(factor * (getByIndex(primitives[i].center, axis) - getByIndex(bounds.lowerBound, axis)))
            b3Assert(0 <= index && index < 8)
            buckets[index].count += 1
            buckets[index].bounds = aabbUnion(buckets[index].bounds, primitives[i].bounds)
        }
        for i in 0..<7 {
            var leftCount = 0
            var leftBounds = bounds3Empty
            for k in 0...i {
                leftCount += buckets[k].count
                leftBounds = aabbUnion(leftBounds, buckets[k].bounds)
            }
            var rightCount = 0
            var rightBounds = bounds3Empty
            for k in (i + 1)..<8 {
                rightCount += buckets[k].count
                rightBounds = aabbUnion(rightBounds, buckets[k].bounds)
            }
            if leftCount > 0 && rightCount > 0 {
                let cost = Float(leftCount) * aabbArea(leftBounds) + Float(rightCount) * aabbArea(rightBounds)
                if cost < bestCost {
                    bestBucket = i
                    bestCost = cost
                    split.axis = axis
                    split.index = leftCount
                    split.leftBounds = leftBounds
                    split.rightBounds = rightBounds
                }
            }
        }
    }
    if bestBucket >= 0 {
        let axis = split.axis
        let factor = Float(8) * (1 - Float.ulpOfOne)
            / (getByIndex(bounds.upperBound, axis) - getByIndex(bounds.lowerBound, axis))
        var splitIndex = range.lowerBound
        for i in range {
            let index = Int(factor * (getByIndex(primitives[i].center, axis) - getByIndex(bounds.lowerBound, axis)))
            if index <= bestBucket {
                primitives.swapAt(i, splitIndex)
                splitIndex += 1
            }
        }
        b3Assert(splitIndex - range.lowerBound == split.index)
    }
    return split
}

func splitMeshHalf(_ primitives: [MeshPrimitive], range: Range<Int>) -> MeshSplit {
    let index = range.count / 2
    var split = meshSplitBounds(primitives, range: range, index: index)
    split.axis = Int(majorAxis(aabbExtents(aabbUnion(split.leftBounds, split.rightBounds))))
    return split
}

func splitMeshMedian(_ primitives: inout [MeshPrimitive], range: Range<Int>) -> MeshSplit {
    b3Assert(range.count > 2)
    var lower = primitives[range.lowerBound].center
    var upper = lower
    for i in range.dropFirst() {
        lower = minVec3(lower, primitives[i].center)
        upper = maxVec3(upper, primitives[i].center)
    }
    let d = upper - lower
    let c = 0.5 * (lower + upper)
    let axis = d.x >= d.y && d.x >= d.z ? 0 : (d.y >= d.z ? 1 : 2)
    let pivot = getByIndex(c, axis)
    var i1 = range.lowerBound
    var i2 = range.upperBound
    while i1 < i2 {
        while i1 < i2 && getByIndex(primitives[i1].center, axis) < pivot { i1 += 1 }
        while i1 < i2 && getByIndex(primitives[i2 - 1].center, axis) >= pivot { i2 -= 1 }
        if i1 < i2 {
            primitives.swapAt(i1, i2 - 1)
            i1 += 1
            i2 -= 1
        }
    }
    b3Assert(i1 == i2)
    var index = i1 - range.lowerBound
    b3Assert(0 <= index && index < range.count)
    if index == 0 || index == range.count - 1 { index = range.count / 2 }
    var split = meshSplitBounds(primitives, range: range, index: index)
    split.axis = axis
    return split
}

private func meshSplitBounds(_ primitives: [MeshPrimitive], range: Range<Int>, index: Int) -> MeshSplit {
    var split = MeshSplit()
    split.index = index
    for i in range.lowerBound..<(range.lowerBound + index) {
        split.leftBounds = aabbUnion(split.leftBounds, primitives[i].bounds)
    }
    for i in (range.lowerBound + index)..<range.upperBound {
        split.rightBounds = aabbUnion(split.rightBounds, primitives[i].bounds)
    }
    return split
}
