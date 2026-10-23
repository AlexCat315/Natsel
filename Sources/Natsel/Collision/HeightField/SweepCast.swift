private struct HeightGridAxis {
    var head: Int
    var tail: Int
    let end: Int
    let step: Int
    let deltaAlpha: Float
    var nextFraction: Float

    init(start: Float, end: Float, tail: Float, absoluteDelta: Float, scale: Float) {
        head = Int((start / scale).rounded(.down))
        self.end = Int((end / scale).rounded(.down))
        self.tail = Int((tail / scale).rounded(.down))
        if head < self.end {
            deltaAlpha = scale / absoluteDelta
            nextFraction = (scale * Float(head + 1) - start) / absoluteDelta
            step = 1
        } else if self.end < head {
            deltaAlpha = scale / absoluteDelta
            nextFraction = (start - scale * Float(head)) / absoluteDelta
            step = -1
        } else {
            deltaAlpha = 0
            nextFraction = .greatestFiniteMagnitude
            step = 0
        }
    }
}

func rayCastHeightField(_ field: HeightField, _ input: RayCastInput) -> CastOutput {
    withUnsafePointer(to: input.origin) { point in
        var shapeInput = ShapeCastInput()
        shapeInput.proxy = ShapeProxy(points: point, count: 1, radius: 0)
        shapeInput.translation = input.translation
        shapeInput.maxFraction = input.maxFraction
        return shapeCastHeightField(field, shapeInput)
    }
}

func shapeCastHeightField(_ field: HeightField, _ input: ShapeCastInput) -> CastOutput {
    let bounds = makeAABB(input.proxy.points!, input.proxy.count, input.proxy.radius)
    let translation = input.translation
    let scale = field.scale
    let start = aabbCenter(bounds)
    let delta = input.maxFraction * translation
    let end = start + delta
    var result = CastOutput()
    let extent = aabbExtents(bounds)
    let margin = Vec3(B3.maxAABBMargin, B3.maxAABBMargin, B3.maxAABBMargin)
    let combined = AABB((field.bounds.lowerBound - extent) - margin, (field.bounds.upperBound + extent) + margin)
    var minimum: Float = 0
    var maximum: Float = 0
    if !rayCastAABB(combined, start, end, &minimum, &maximum) { return result }
    var clampedStart = mulAdd(start, minimum, delta)
    let clampedDelta = (maximum - minimum) * delta
    var clampedEnd = clampedStart + clampedDelta
    let centerStart = clampedStart
    let centerEnd = clampedEnd
    let signX: Float = translation.x >= 0 ? 1 : -1
    let signZ: Float = translation.z >= 0 ? 1 : -1
    if translation.x >= 0 { clampedStart.x += extent.x } else { clampedStart.x -= extent.x }
    if translation.z >= 0 { clampedStart.z += extent.z } else { clampedStart.z -= extent.z }
    clampedEnd = clampedStart + clampedDelta
    let absolute = absVec3(clampedDelta)
    var x = HeightGridAxis(start: clampedStart.x, end: clampedEnd.x,
                           tail: clampedStart.x - 2 * signX * extent.x, absoluteDelta: absolute.x, scale: scale.x)
    var z = HeightGridAxis(start: clampedStart.z, end: clampedEnd.z,
                           tail: clampedStart.z - 2 * signZ * extent.z, absoluteDelta: absolute.z, scale: scale.z)
    var bestFraction = input.maxFraction
    let gridScale = input.maxFraction * (maximum - minimum)
    let gridOffset = input.maxFraction * minimum
    var pair = ShapeCastPairInput()
    pair.proxyB = input.proxy
    pair.transform = .identity
    pair.translationB = translation
    pair.canEncroach = input.canEncroach
    let castBounds = AABB(minVec3(centerStart, centerEnd) - extent, maxVec3(centerStart, centerEnd) + extent)
    while true {
        let row1 = max(0, min(z.tail, z.head))
        let row2 = min(field.rowCount - 2, max(z.tail, z.head))
        let column1 = max(0, min(x.tail, x.head))
        let column2 = min(field.columnCount - 2, max(x.tail, x.head))
        if row1 <= row2 && column1 <= column2 {
            for row in row1...row2 {
                for column in column1...column2 {
                    let cell = row * (field.columnCount - 1) + column
                    let material = field.materialIndices[cell]
                    if material == 0xff { continue }
                    var v = field.corners(row: row, column: column)
                    if field.header.clockwise != 0 {
                        let tmp = v[1]; v[1] = v[2]; v[2] = tmp
                    }
                    if !aabbOverlaps(castBounds, heightCellBounds(v)) { continue }
                    for second in [false, true] {
                        let index = Int32(2 * cell + (second ? 1 : 0))
                        let origin = second ? v[2] : v[0]
                        let b = second ? v[3] : v[2]
                        let c = v[1]
                        if input.proxy.count == 1 && input.proxy.radius == 0 {
                            let alpha = second
                                ? intersectMeshRayTriangle(start, translation, v[3], v[1], v[2])
                                : intersectMeshRayTriangle(start, translation, v[0], v[2], v[1])
                            if alpha < bestFraction {
                                result.point = mulAdd(start, alpha, translation)
                                result.normal = normalize(cross(b - origin, c - origin))
                                result.fraction = alpha
                                result.triangleIndex = index
                                result.materialIndex = Int32(material)
                                result.hit = true
                                bestFraction = alpha
                            }
                        } else if signedVolume(origin, b, c, start) >= 0 {
                            let triangle = [Vec3.zero, b - origin, c - origin]
                            let output = triangle.withUnsafeBufferPointer { points in
                                pair.proxyA = ShapeProxy(points: points.baseAddress, count: 3, radius: 0)
                                pair.maxFraction = bestFraction
                                pair.transform.p = -origin
                                return shapeCast(pair)
                            }
                            if output.hit {
                                bestFraction = output.fraction
                                result = output
                                result.point = result.point + origin
                                result.triangleIndex = index
                                result.materialIndex = Int32(material)
                            }
                        }
                    }
                }
            }
        }
        let sentinel = Float.greatestFiniteMagnitude
        let fractionX = x.nextFraction == sentinel ? sentinel : gridOffset + x.nextFraction * gridScale
        let fractionZ = z.nextFraction == sentinel ? sentinel : gridOffset + z.nextFraction * gridScale
        if fractionX > bestFraction && fractionZ > bestFraction { break }
        if x.nextFraction <= z.nextFraction {
            if x.head == x.end { break }
            x.head += x.step
            x.tail = x.head
            if extent.z == 0 {
                z.tail = z.head
            } else {
                let intercept = clampedStart.z + x.nextFraction * clampedDelta.z
                z.tail = Int(((intercept - 2 * signZ * extent.z) / scale.z).rounded(.down))
            }
            x.nextFraction += x.deltaAlpha
        } else {
            if z.head == z.end { break }
            z.head += z.step
            z.tail = z.head
            if extent.x == 0 {
                x.tail = x.head
            } else {
                let intercept = clampedStart.x + z.nextFraction * clampedDelta.x
                x.tail = Int(((intercept - 2 * signX * extent.x) / scale.x).rounded(.down))
            }
            z.nextFraction += z.deltaAlpha
        }
    }
    return result
}
