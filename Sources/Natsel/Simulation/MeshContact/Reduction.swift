struct MeshPatchPoint {
    var p: Vec2
    var separation: Float
    var originalIndex: Int
}

private func betterMeshCullCandidate(_ score: Float, _ separation: Float,
                                     _ bestScore: Float, _ bestSeparation: Float,
                                     _ scoreTolerance: Float, _ separationTolerance: Float) -> Bool {
    if score > bestScore + scoreTolerance { return true }
    if score < bestScore - scoreTolerance { return false }
    return separation < bestSeparation - separationTolerance
}

/// Four-point reduction, including tie tolerances and swap removal order.
private func cullMeshPoints(
    _ points: UnsafeMutableBufferPointer<MeshPatchPoint>,
    _ selected: inout InlineArray<4, MeshPatchPoint>
) -> Int {
    if points.count <= 1 {
        if points.count == 1 { selected[0] = points[0] }
        return points.count
    }
    let tolerance: Float = 0.25 * B3.linearSlop
    let toleranceSquared = tolerance * tolerance
    let separationTolerance = B3.linearSlop
    var count = points.count
    var bestScore: Float = 0
    var bestSeparation = Float.greatestFiniteMagnitude
    var bestIndex1 = -1
    var bestIndex2 = -1
    for i in 0..<count {
        let p1 = points[i].p
        for j in (i + 1)..<count {
            let score = distanceSquared2(p1, points[j].p)
            let separation = points[i].separation + points[j].separation
            if betterMeshCullCandidate(score, separation, bestScore, bestSeparation, toleranceSquared, separationTolerance) {
                bestIndex1 = i
                bestIndex2 = j
                bestScore = score
                bestSeparation = separation
            }
        }
    }
    if bestScore < toleranceSquared {
        var deepest = 0
        for i in 1..<count {
            if points[i].separation < points[deepest].separation { deepest = i }
        }
        selected[0] = points[deepest]
        return 1
    }
    selected[0] = points[bestIndex1]
    selected[1] = points[bestIndex2]
    points[bestIndex2] = points[count - 1]
    points[bestIndex1] = points[count - 2]
    count -= 2
    if count == 0 { return 2 }
    let a = selected[0].p
    var b = selected[1].p
    var ba = Vec2(b.x - a.x, b.y - a.y)
    bestScore = 0
    bestSeparation = .greatestFiniteMagnitude
    var bestIndex = -1
    var bestSignedArea: Float = 0
    for i in 0..<count {
        let p = points[i].p
        let signedArea = cross2(ba, Vec2(p.x - a.x, p.y - a.y))
        let score = absFloat(signedArea)
        if betterMeshCullCandidate(score, points[i].separation, bestScore, bestSeparation, toleranceSquared, separationTolerance) {
            bestSignedArea = signedArea
            bestScore = score
            bestSeparation = points[i].separation
            bestIndex = i
        }
    }
    if bestIndex == -1 { return 2 }
    selected[2] = points[bestIndex]
    if count == 1 { return 3 }
    points[bestIndex] = points[count - 1]
    count -= 1
    var c = selected[2].p
    if bestSignedArea < 0 {
        swap(&b, &c)
        ba = Vec2(b.x - a.x, b.y - a.y)
    }
    let cb = Vec2(c.x - b.x, c.y - b.y)
    let ac = Vec2(a.x - c.x, a.y - c.y)
    bestScore = 0
    bestSeparation = .greatestFiniteMagnitude
    bestIndex = -1
    for i in 0..<count {
        let p = points[i].p
        let u1 = cross2(Vec2(p.x - a.x, p.y - a.y), ba)
        let u2 = cross2(Vec2(p.x - b.x, p.y - b.y), cb)
        let u3 = cross2(Vec2(p.x - c.x, p.y - c.y), ac)
        let score = maxFloat(u1, maxFloat(u2, u3))
        if betterMeshCullCandidate(score, points[i].separation, bestScore, bestSeparation, toleranceSquared, separationTolerance) {
            bestScore = score
            bestSeparation = points[i].separation
            bestIndex = i
        }
    }
    if bestIndex != -1 {
        selected[3] = points[bestIndex]
        return 4
    }
    return 3
}

/// Standalone/reference-test entry point. Production projects into the worker's reusable buffer.
func cullMeshPoints(_ source: [MeshPatchPoint]) -> [MeshPatchPoint] {
    var points = source
    var selected = InlineArray<4, MeshPatchPoint>(
        repeating: MeshPatchPoint(p: Vec2(), separation: 0, originalIndex: 0))
    let count = points.withUnsafeMutableBufferPointer { cullMeshPoints($0, &selected) }
    var output: [MeshPatchPoint] = []
    output.reserveCapacity(count)
    for i in 0..<count { output.append(selected[i]) }
    return output
}

/// Reduce in place, preserving each cluster's allocation for the next contact. The projected
/// scratch has no shared slices, so neither projection nor swap removal needs an Array copy.
func reduceMeshCluster(
    _ points: inout [LocalManifoldPoint], normal: Vec3, projected: inout [MeshPatchPoint]
) {
    let count = points.count
    if count <= 1 { return }
    let u = perp(normal)
    let v = cross(normal, u)
    let origin = points[0].point
    if projected.count < count {
        projected.append(contentsOf: repeatElement(
            MeshPatchPoint(p: Vec2(), separation: 0, originalIndex: 0), count: count - projected.count))
    }
    var selected = InlineArray<4, MeshPatchPoint>(
        repeating: MeshPatchPoint(p: Vec2(), separation: 0, originalIndex: 0))
    let selectedCount = projected.withUnsafeMutableBufferPointer { scratch in
        for i in 0..<count {
            let point = points[i]
            let d = point.point - origin
            scratch[i] = MeshPatchPoint(
                p: Vec2(dot(d, u), dot(d, v)), separation: point.separation, originalIndex: i)
        }
        return cullMeshPoints(UnsafeMutableBufferPointer(start: scratch.baseAddress, count: count), &selected)
    }
    // Fetch all chosen points before overwriting the source prefix: selected indices may alias it.
    var output = InlineArray<4, LocalManifoldPoint>(repeating: LocalManifoldPoint())
    for i in 0..<selectedCount { output[i] = points[selected[i].originalIndex] }
    for i in 0..<selectedCount { points[i] = output[i] }
    points.removeLast(count - selectedCount)
}
