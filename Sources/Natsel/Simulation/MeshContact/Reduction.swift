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

/// mesh_contact.c's four-point reduction, including tie tolerances and swap removal order.
func cullMeshPoints(_ source: [MeshPatchPoint]) -> [MeshPatchPoint] {
    if source.count <= 1 { return source }
    var points = source
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
        return [points[deepest]]
    }
    var final = [points[bestIndex1], points[bestIndex2]]
    points[bestIndex2] = points[count - 1]
    points[bestIndex1] = points[count - 2]
    count -= 2
    if count == 0 { return final }
    let a = final[0].p
    var b = final[1].p
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
    if bestIndex == -1 { return final }
    final.append(points[bestIndex])
    if count == 1 { return final }
    points[bestIndex] = points[count - 1]
    count -= 1
    var c = final[2].p
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
    if bestIndex != -1 { final.append(points[bestIndex]) }
    return final
}

func reduceMeshCluster(_ points: [LocalManifoldPoint], normal: Vec3) -> [LocalManifoldPoint] {
    if points.count <= 1 { return points }
    let u = perp(normal)
    let v = cross(normal, u)
    let origin = points[0].point
    var projected: [MeshPatchPoint] = []
    for (i, point) in points.enumerated() {
        let d = point.point - origin
        projected.append(MeshPatchPoint(p: Vec2(dot(d, u), dot(d, v)), separation: point.separation, originalIndex: i))
    }
    return cullMeshPoints(projected).map { points[$0.originalIndex] }
}
