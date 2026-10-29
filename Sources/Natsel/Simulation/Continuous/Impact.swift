/// Triangle-query TOI, including its centroid fallback and static-surface assumption.
func triangleSurfaceTimeOfImpact(_ surface: TriangleSurface, shapeB: Shape, sweepA: Sweep, sweepB: Sweep,
                      maxFraction: Float, isSensor: Bool) -> TOIOutput {
    let centroid = getShapeCentroid(shapeB)
    let transformA = Transform(sweepA.c1 - rotateVector(sweepA.q1, sweepA.localCenter), sweepA.q1)
    let minExtent = computeShapeMinExtent(shapeB, centroid)
    let fallbackRadius = maxFloat(0.5 * minExtent, B3.linearSlop)
    let bounds = computeSweptShapeAABB(shapeB, sweepB, maxFraction)
    let localBounds = aabbTransform(invertTransform(transformA), bounds)
    return triangleSurfaceImpact(surface, shapeB: shapeB, sweepA: sweepA, sweepB: sweepB,
                                 maxFraction: maxFraction, isSensor: isSensor, transformA: transformA,
                                 localBounds: localBounds, fallbackRadius: fallbackRadius)
}

func triangleSurfaceImpact(_ surface: TriangleSurface, shapeB: Shape, sweepA: Sweep, sweepB: Sweep,
                           maxFraction: Float, isSensor: Bool, transformA: Transform,
                           localBounds: AABB, fallbackRadius: Float) -> TOIOutput {
    let centroid = getShapeCentroid(shapeB)
    let transformB1 = Transform(sweepB.c1 - rotateVector(sweepB.q1, sweepB.localCenter), sweepB.q1)
    let transformB2 = Transform(sweepB.c2 - rotateVector(sweepB.q2, sweepB.localCenter), sweepB.q2)
    let center1 = invTransformPoint(transformA, transformPoint(transformB1, centroid))
    let center2 = invTransformPoint(transformA, transformPoint(transformB2, centroid))
    let buffer = makeShapeProxy(shapeB)
    return buffer.withProxy { proxy in
        var input = TOIInput()
        input.proxyB = proxy
        input.sweepA = sweepA
        input.sweepB = sweepB
        input.maxFraction = maxFraction
        var best = TOIOutput()
        surface.query(localBounds) { a, b, c, _ in
            let normal = normalize(cross(b - a, c - a))
            let offset1 = dot(normal, center1 - a)
            let offset2 = dot(normal, center2 - a)
            if offset1 < 0 { return true }
            if !isSensor && offset1 - offset2 < fallbackRadius && offset2 > fallbackRadius { return true }
            let triangle: InlineArray<3, Vec3> = [a, b, c]
            withInlineReadBuffer(triangle) { points in
                input.proxyA = ShapeProxy(points: points.baseAddress, count: 3, radius: 0)
                var output = timeOfImpact(input)
                if 0 < output.fraction && output.fraction < input.maxFraction {
                    best = output
                    input.maxFraction = output.fraction
                } else if output.fraction == 0 {
                    withUnsafePointer(to: centroid) { point in
                        var fallback = input
                        fallback.proxyB = ShapeProxy(points: point, count: 1, radius: fallbackRadius + B3.linearSlop)
                        output = timeOfImpact(fallback)
                    }
                    if 0 < output.fraction && output.fraction < input.maxFraction {
                        best = output
                        input.maxFraction = output.fraction
                        best.usedFallback = true
                    }
                }
            }
            return true
        }
        return best
    }
}
