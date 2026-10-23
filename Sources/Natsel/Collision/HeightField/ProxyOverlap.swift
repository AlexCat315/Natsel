func overlapHeightField(_ field: HeightField, _ transform: Transform, _ proxy: ShapeProxy) -> Bool {
    var localPoints = [Vec3](repeating: .zero, count: Int(B3.maxShapeCastPoints))
    return localPoints.withUnsafeMutableBufferPointer { buffer in
        let local = makeLocalProxy(proxy, transform, buffer.baseAddress!)
        let bounds = computeProxyAABB(local)
        let center = 0.5 * (bounds.lowerBound + bounds.upperBound)
        let extent = bounds.upperBound - center
        let region = HeightFieldRegion(field, bounds: bounds)
        var input = DistanceInput()
        input.proxyB = local
        input.transform = .identity
        input.useRadii = true
        var cache = SimplexCache()
        for row in region.rows {
            for column in region.columns {
                let cell = row * (field.columnCount - 1) + column
                if field.materialIndices[cell] == 0xff { continue }
                let v = field.corners(row: row, column: column)
                for second in [false, true] {
                    let sat = second ? [v[2], v[3], v[1]] : [v[0], v[2], v[1]]
                    if !boundsTriangleOverlap(center, extent, sat[0], sat[1], sat[2]) { continue }
                    let points = second ? [v[3], v[1], v[2]] : [v[0], v[2], v[1]]
                    let touching = points.withUnsafeBufferPointer { triangle in
                        input.proxyA = ShapeProxy(points: triangle.baseAddress, count: 3, radius: 0)
                        cache.count = 0
                        return shapeDistance(input, &cache).distance < 0.1 * B3.linearSlop
                    }
                    if touching { return true }
                }
            }
        }
        return false
    }
}
