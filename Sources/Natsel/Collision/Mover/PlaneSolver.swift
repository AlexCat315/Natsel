public func solvePlanes(_ targetDelta: Vec3, _ planes: inout [CollisionPlane]) -> PlaneSolverResult {
    for i in planes.indices { planes[i].push = 0 }
    var delta = targetDelta
    var iteration: Int32 = 0
    while iteration < 20 {
        var totalPush: Float = 0
        for i in planes.indices {
            let separation = planeSeparation(planes[i].plane, delta) + B3.linearSlop
            var push = -separation
            let accumulatedPush = planes[i].push
            planes[i].push = clampFloat(planes[i].push + push, 0, planes[i].pushLimit)
            push = planes[i].push - accumulatedPush
            delta = mulAdd(delta, push, planes[i].plane.normal)
            totalPush += abs(push)
        }
        if totalPush < B3.linearSlop { break }
        iteration += 1
    }
    var result = PlaneSolverResult()
    result.delta = delta
    result.iterationCount = iteration
    return result
}

public func clipVector(_ vector: Vec3, _ planes: [CollisionPlane]) -> Vec3 {
    var result = vector
    for plane in planes {
        if plane.push == 0 || !plane.clipVelocity { continue }
        result = mulSub(result, minFloat(0, dot(result, plane.plane.normal)), plane.plane.normal)
    }
    return result
}
