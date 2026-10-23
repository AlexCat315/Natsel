func collideMoverAndHull(_ result: inout PlaneResult, _ shape: Hull, _ mover: Capsule) -> Int32 {
    let points = [mover.center1, mover.center2]
    let output = points.withUnsafeBufferPointer { buffer in
        var input = DistanceInput()
        input.proxyA = ShapeProxy(points: shape.points, count: shape.vertexCount, radius: 0)
        input.proxyB = ShapeProxy(points: buffer.baseAddress, count: 2, radius: mover.radius)
        input.transform = .identity
        input.useRadii = false
        var cache = SimplexCache()
        return shapeDistance(input, &cache)
    }
    if output.distance == 0 { return 0 }
    if output.distance <= mover.radius {
        result = PlaneResult()
        result.plane = Plane(output.normal, mover.radius - output.distance)
        result.point = output.pointA
        return 1
    }
    return 0
}
