struct BoundsPair: Equatable {
    let a: Int
    let b: Int
}

struct BoundsPairResult {
    var pairs: [BoundsPair] = []
    var candidateCount = 0
}

/// Sweep along the widest world axis, then reject on all three double precision intervals.
/// Output is in input-index order, independent of sort ties and world-coordinate magnitude.
func overlappingBoundsPairs(_ bounds: [WorldAABB]) -> BoundsPairResult {
    guard bounds.count > 1 else { return BoundsPairResult() }
    var lower = bounds[0].lowerBound
    var upper = bounds[0].upperBound
    for box in bounds.dropFirst() {
        lower = Pos(min(lower.x, box.lowerBound.x), min(lower.y, box.lowerBound.y), min(lower.z, box.lowerBound.z))
        upper = Pos(max(upper.x, box.upperBound.x), max(upper.y, box.upperBound.y), max(upper.z, box.upperBound.z))
    }
    let span = [upper.x - lower.x, upper.y - lower.y, upper.z - lower.z]
    let axis = span[1] > span[0] ? (span[2] > span[1] ? 2 : 1) : (span[2] > span[0] ? 2 : 0)
    func component(_ p: Pos) -> Double { axis == 0 ? p.x : axis == 1 ? p.y : p.z }
    let order = bounds.indices.sorted {
        let a = component(bounds[$0].lowerBound)
        let b = component(bounds[$1].lowerBound)
        return a == b ? $0 < $1 : a < b
    }
    var result = BoundsPairResult()
    for k in order.indices {
        let a = order[k]
        var l = k + 1
        while l < order.count && component(bounds[order[l]].lowerBound) <= component(bounds[a].upperBound) {
            let b = order[l]
            result.candidateCount += 1
            if bounds[a].overlaps(bounds[b]) { result.pairs.append(BoundsPair(a: min(a, b), b: max(a, b))) }
            l += 1
        }
    }
    result.pairs.sort { $0.a == $1.a ? $0.b < $1.b : $0.a < $1.a }
    return result
}
