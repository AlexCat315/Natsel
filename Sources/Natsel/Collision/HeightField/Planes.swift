func collideMoverAndHeightField(_ field: HeightField, _ mover: Capsule, capacity: Int) -> [PlaneResult] {
    guard capacity > 0 else { return [] }
    let center = lerp(mover.center1, mover.center2, 0.5)
    let r = Vec3(mover.radius, mover.radius, mover.radius)
    let bounds = AABB(minVec3(mover.center1, mover.center2) - r, maxVec3(mover.center1, mover.center2) + r)
    let boundsCenter = 0.5 * (bounds.lowerBound + bounds.upperBound)
    let boundsExtent = bounds.upperBound - boundsCenter
    let region = HeightFieldRegion(field, bounds: bounds)
    let moverPoints = [mover.center1, mover.center2]
    return moverPoints.withUnsafeBufferPointer { buffer in
        var input = DistanceInput()
        input.proxyB = ShapeProxy(points: buffer.baseAddress, count: 2, radius: 0)
        input.transform = .identity
        input.useRadii = false
        var cache = SimplexCache()
        var planes: [PlaneResult] = []
        for row in region.rows {
            for column in region.columns {
                let cell = row * (field.columnCount - 1) + column
                let material = field.materialIndices[cell]
                if material == 0xff { continue }
                var v = field.corners(row: row, column: column)
                if field.header.clockwise != 0 {
                    let tmp = v[1]; v[1] = v[2]; v[2] = tmp
                }
                for second in [false, true] {
                    let sat = second ? [v[2], v[3], v[1]] : [v[0], v[2], v[1]]
                    if !boundsTriangleOverlap(boundsCenter, boundsExtent, sat[0], sat[1], sat[2]) { continue }
                    let points = second ? [v[3], v[1], v[2]] : [v[0], v[2], v[1]]
                    if signedVolume(points[0], points[1], points[2], center) < 0 { continue }
                    let out = points.withUnsafeBufferPointer { triangle in
                        input.proxyA = ShapeProxy(points: triangle.baseAddress, count: 3, radius: 0)
                        cache.count = 0
                        return shapeDistance(input, &cache)
                    }
                    if out.distance != 0 && out.distance <= mover.radius {
                        var result = PlaneResult()
                        result.plane = Plane(out.normal, mover.radius - out.distance)
                        result.point = out.pointA
                        result.triangleIndex = Int32(2 * cell + (second ? 1 : 0))
                        result.materialIndex = Int32(material)
                        planes.append(result)
                        if planes.count == capacity { return planes }
                    }
                }
            }
        }
        return planes
    }
}
