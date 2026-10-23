func compoundTimeOfImpact(_ compound: Compound, shapeB: Shape, sweepA: Sweep, sweepB: Sweep,
                          maxFraction: Float) -> TOIOutput {
    let transform = Transform(sweepA.c1, sweepA.q1)
    let centroid = getShapeCentroid(shapeB)
    let fallbackRadius = maxFloat(0.75 * computeShapeMinExtent(shapeB, centroid), B3.speculativeDistance)
    let bounds = computeSweptShapeAABB(shapeB, sweepB, maxFraction)
    let localBounds = aabbTransform(invertTransform(transform), bounds)
    let buffer = makeShapeProxy(shapeB)
    return buffer.withProxy { proxy in
        var input = TOIInput()
        input.proxyB = proxy
        input.sweepB = sweepB
        input.maxFraction = maxFraction
        var best = TOIOutput()
        queryCompound(compound, bounds: localBounds) { index in
            let child = compound.child(at: index)
            let childTransform = mulTransforms(transform, child.transform)
            var sweep = Sweep()
            sweep.localCenter = .zero
            sweep.c1 = childTransform.p
            sweep.c2 = childTransform.p
            sweep.q1 = childTransform.q
            sweep.q2 = childTransform.q
            input.sweepA = sweep
            let output: TOIOutput
            switch child.geometry {
            case .mesh(let mesh):
                output = triangleSurfaceImpact(TriangleSurface(.mesh(mesh))!, shapeB: shapeB,
                                                sweepA: sweep, sweepB: sweepB, maxFraction: input.maxFraction,
                                                isSensor: false, transformA: childTransform,
                                                localBounds: aabbTransform(invertTransform(child.transform), localBounds),
                                                fallbackRadius: fallbackRadius)
            case .capsule(let capsule):
                let points = [capsule.center1, capsule.center2]
                output = points.withUnsafeBufferPointer { points in
                    input.proxyA = ShapeProxy(points: points.baseAddress, count: 2, radius: capsule.radius)
                    return timeOfImpact(input)
                }
            case .hull(let hull):
                input.proxyA = ShapeProxy(points: hull.points, count: hull.vertexCount, radius: 0)
                output = timeOfImpact(input)
            case .sphere(let sphere):
                output = withUnsafePointer(to: sphere.center) { point in
                    input.proxyA = ShapeProxy(points: point, count: 1, radius: sphere.radius)
                    return timeOfImpact(input)
                }
            }
            if 0 < output.fraction && output.fraction < input.maxFraction {
                best = output
                input.maxFraction = output.fraction
            }
            input.proxyA = ShapeProxy()
            return true
        }
        return best
    }
}
