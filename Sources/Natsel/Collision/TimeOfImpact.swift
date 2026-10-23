// Time of impact using swept separating axes and a root finder.

/// The kind of feature the separation function is based on.
enum SeparationType: UInt8, Sendable {
    case unknown = 0
    case vertices
    case edges
    case faceA
    case faceB
}

/// A separation function evaluated over two sweeps.
struct SeparationFunction {
    var proxyA = ShapeProxy()
    var proxyB = ShapeProxy()
    var sweepA = Sweep()
    var sweepB = Sweep()

    // These are associated with different bodies depending on the separation function type.
    // It could be two local vectors/points on the same body.
    var witness1 = Vec3()
    var witness2 = Vec3()

    var type = SeparationType.unknown

    init() {}
}

/// Count the number of distinct indices.
func uniqueCount(_ vertexCount: Int32, _ vertices: inout [Int32]) -> Int32 {
    b3Assert(1 <= vertexCount && vertexCount <= 3)

    switch vertexCount {
    case 1:
        return 1

    case 2:
        return vertices[0] != vertices[1] ? 2 : 1

    case 3:
        if vertices[0] != vertices[1] && vertices[0] != vertices[2] && vertices[1] != vertices[2] {
            // All different
            return 3
        }

        if vertices[0] == vertices[1] && vertices[0] == vertices[2] && vertices[1] == vertices[2] {
            // All equal
            return 1
        }

        return 2

    default:
        b3Assert(false)
        return 0
    }
}

/// Check if the cross product of two edges switches direction.
func checkFastEdges(_ xfA: Transform, _ localEdgeA: Vec3, _ xfB: Transform, _ localEdgeB: Vec3, _ axis0: Vec3) -> Bool {
    // By taking the local witness axes we make sure that we
    // get the correct orientations (e.g. if one axis was flipped)!
    let edgeA = rotateVector(xfA.q, localEdgeA)
    let edgeB = rotateVector(xfB.q, localEdgeB)
    let axis = cross(edgeA, edgeB)
    return dot(axis, axis0) < 0
}

/// Build a separation function from a GJK simplex cache.
func makeSeparationFunction(
    _ cache: SimplexCache,
    _ proxyA: ShapeProxy,
    _ sweepA: Sweep,
    _ proxyB: ShapeProxy,
    _ sweepB: Sweep,
    _ worldNormal: Vec3,
    _ t1: Float
) -> SeparationFunction {
    b3Assert(1 <= cache.count && cache.count <= 3)
    b3Validate(isNormalized(worldNormal))

    var fcn = SeparationFunction()
    fcn.proxyA = proxyA
    fcn.proxyB = proxyB
    fcn.sweepA = sweepA
    fcn.sweepB = sweepB
    fcn.type = .unknown

    var indexA = [Int32(cache.indexA[0]), Int32(cache.indexA[1]), Int32(cache.indexA[2])]
    var indexB = [Int32(cache.indexB[0]), Int32(cache.indexB[1]), Int32(cache.indexB[2])]

    let uniqueCountA = uniqueCount(Int32(cache.count), &indexA)
    let uniqueCountB = uniqueCount(Int32(cache.count), &indexB)

    let xfA1 = getSweepTransform(sweepA, t1)
    let xfB1 = getSweepTransform(sweepB, t1)

    let qA = xfA1.q
    let qB = xfB1.q

    // Minimize round-off
    let deltaP = xfB1.p - xfA1.p

    let pointsA = proxyA.points!
    let pointsB = proxyB.points!

    switch cache.count {
    case 1:
        // Witness is the world space direction
        fcn.type = .vertices
        fcn.witness1 = worldNormal

    case 2:
        if uniqueCountA == 2 && uniqueCountB == 2 {
            // Edge/Edge
            let vA1 = pointsA[Int(indexA[0])]
            let localEdgeA = normalize(pointsA[Int(indexA[1])] - vA1)
            let edgeA = rotateVector(qA, localEdgeA)

            let vB1 = pointsB[Int(indexB[0])]
            var localEdgeB = normalize(pointsB[Int(indexB[1])] - vB1)
            let edgeB = rotateVector(qB, localEdgeB)

            var axis = cross(edgeA, edgeB)
            let lengthSquared = lengthSquared(axis)

            // Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
            let toleranceSquared: Float = 0.05 * 0.05
            if lengthSquared < toleranceSquared {
                // The axis is not safe to normalize so we use a world axis instead!
                fcn.type = .vertices
                fcn.witness1 = worldNormal
            } else {
                let delta = (rotateVector(qB, vB1) - rotateVector(qA, vA1)) + deltaP
                if dot(delta, axis) < 0 {
                    // Make axis point from A to B
                    axis = -axis
                    localEdgeB = -localEdgeB
                }

                // Check for possible sign flip in edge/edge cross product
                let xfA2 = getFinalSweepTransform(sweepA)
                let xfB2 = getFinalSweepTransform(sweepB)
                let fastEdges = checkFastEdges(xfA2, localEdgeA, xfB2, localEdgeB, axis)
                if fastEdges {
                    // Not safe to use local edges, fall back to initial world space axis instead
                    fcn.type = .vertices
                    fcn.witness1 = normalize(axis)
                } else {
                    // Edge cross product is safe. This converges faster than a fixed axis.
                    fcn.type = .edges
                    fcn.witness1 = localEdgeA
                    fcn.witness2 = localEdgeB
                }
            }
        } else {
            b3Validate(isNormalized(worldNormal))

            // Vertex versus edge, use world axis witness
            fcn.type = .vertices
            fcn.witness1 = worldNormal
        }

    case 3:
        if uniqueCountA == 3 {
            let vA1 = pointsA[Int(indexA[0])]
            let vA2 = pointsA[Int(indexA[1])]
            let vA3 = pointsA[Int(indexA[2])]
            var localAxisA = normalize(cross(vA2 - vA1, vA3 - vA1))
            let axisA = rotateVector(qA, localAxisA)

            let localPointA = (1.0 / 3.0) * ((vA1 + vA2) + vA3)
            let localPointB = pointsB[Int(indexB[0])]
            let delta = (rotateVector(qB, localPointB) - rotateVector(qA, localPointA)) + deltaP

            if dot(delta, axisA) < 0 {
                // Make axis point from A to B
                localAxisA = -localAxisA
            }

            // Witness is the local plane of faceA
            fcn.type = .faceA
            fcn.witness1 = localAxisA
            fcn.witness2 = localPointA
        } else if uniqueCountB == 3 {
            let vB1 = pointsB[Int(indexB[0])]
            let vB2 = pointsB[Int(indexB[1])]
            let vB3 = pointsB[Int(indexB[2])]
            var localAxisB = normalize(cross(vB2 - vB1, vB3 - vB1))
            let axisB = rotateVector(qB, localAxisB)

            let localPointA = pointsA[Int(indexA[0])]
            let localPointB = (1.0 / 3.0) * ((vB1 + vB2) + vB3)
            let delta = (rotateVector(qA, localPointA) - rotateVector(qB, localPointB)) - deltaP

            if dot(delta, axisB) < 0 {
                // Make axis point from B to A
                localAxisB = -localAxisB
            }

            // Witness is the local plane of faceB
            fcn.type = .faceB
            fcn.witness1 = localAxisB
            fcn.witness2 = localPointB
        } else {
            b3Assert(uniqueCountA == 2 && uniqueCountB == 2)

            if indexA[0] == indexA[1] {
                // Make first two indices are unique
                indexA[1] = indexA[2]
                b3Assert(indexA[0] != indexA[1])
            }

            let vA1 = pointsA[Int(indexA[0])]
            let vA2 = pointsA[Int(indexA[1])]
            let localEdgeA = normalize(vA2 - vA1)
            let edgeA = rotateVector(qA, localEdgeA)

            if indexB[0] == indexB[1] {
                // Make first two indices are unique
                indexB[1] = indexB[2]
                b3Assert(indexB[0] != indexB[1])
            }

            let vB1 = pointsB[Int(indexB[0])]
            let vB2 = pointsB[Int(indexB[1])]
            var localEdgeB = normalize(vB2 - vB1)
            let edgeB = rotateVector(qB, localEdgeB)

            var axis = cross(edgeA, edgeB)
            let lengthSquared = lengthSquared(axis)

            // Skip near parallel edges: |e1 x e1| = sin(alpha) * |e1| * |e2|
            let toleranceSquared: Float = 0.005 * 0.005
            if lengthSquared < toleranceSquared {
                // The axis is not safe to normalize so we use a world axis instead!
                fcn.type = .vertices
                fcn.witness1 = worldNormal
            } else {
                let delta = (rotateVector(qB, vB1) - rotateVector(qA, vA1)) + deltaP
                if dot(delta, axis) < 0 {
                    // Make axis point from A to B
                    axis = -axis
                    localEdgeB = -localEdgeB
                }

                // Check for possible sign flip in edge/edge cross product
                let xfA2 = getFinalSweepTransform(sweepA)
                let xfB2 = getFinalSweepTransform(sweepB)
                let fastEdges = checkFastEdges(xfA2, localEdgeA, xfB2, localEdgeB, axis)
                if fastEdges {
                    // Not safe to use local edges, fall back to initial world space axis instead
                    fcn.type = .vertices
                    fcn.witness1 = normalize(axis)
                } else {
                    // Edge cross product is safe. This converges faster than a fixed axis.
                    fcn.type = .edges
                    fcn.witness1 = localEdgeA
                    fcn.witness2 = localEdgeB
                }
            }
        }

    default:
        b3Assert(false)
    }

    return fcn
}

/// Find the minimum separation along the separating axis at time `t`.
func findMinSeparation(_ fcn: SeparationFunction, _ indexA: inout Int32, _ indexB: inout Int32, _ t: Float) -> Float {
    let xfA = getSweepTransform(fcn.sweepA, t)
    let xfB = getSweepTransform(fcn.sweepB, t)

    let pointsA = fcn.proxyA.points!
    let pointsB = fcn.proxyB.points!

    switch fcn.type {
    case .vertices:
        let axis = fcn.witness1

        let localAxisA = invRotateVector(xfA.q, axis)
        let localAxisB = invRotateVector(xfB.q, -axis)

        indexA = getPointSupport(pointsA, fcn.proxyA.count, localAxisA)
        indexB = getPointSupport(pointsB, fcn.proxyB.count, localAxisB)

        let deltaP = xfB.p - xfA.p
        let localPointA = pointsA[Int(indexA)]
        let localPointB = pointsB[Int(indexB)]
        let delta = (rotateVector(xfB.q, localPointB) - rotateVector(xfA.q, localPointA)) + deltaP
        return dot(delta, axis)

    case .edges:
        let edgeA = rotateVector(xfA.q, fcn.witness1)
        let edgeB = rotateVector(xfB.q, fcn.witness2)
        var axis = cross(edgeA, edgeB)
        b3Assert(axis.x != 0 || axis.y != 0 || axis.z != 0)
        axis = normalize(axis)

        let axisA = invRotateVector(xfA.q, axis)
        indexA = getPointSupport(pointsA, fcn.proxyA.count, axisA)

        let axisB = invRotateVector(xfB.q, axis)
        indexB = getPointSupport(pointsB, fcn.proxyB.count, -axisB)

        let deltaP = xfB.p - xfA.p
        let localPointA = pointsA[Int(indexA)]
        let localPointB = pointsB[Int(indexB)]
        let delta = (rotateVector(xfB.q, localPointB) - rotateVector(xfA.q, localPointA)) + deltaP

        return dot(delta, axis)

    case .faceA:
        let normal = rotateVector(xfA.q, fcn.witness1)
        indexA = -1
        let pointA = transformPoint(xfA, fcn.witness2)

        let axisB = invRotateVector(xfB.q, normal)
        indexB = getPointSupport(pointsB, fcn.proxyB.count, -axisB)
        let pointB = transformPoint(xfB, pointsB[Int(indexB)])

        return dot(pointB - pointA, normal)

    case .faceB:
        let normal = rotateVector(xfB.q, fcn.witness1)

        let axisA = invRotateVector(xfA.q, normal)
        indexA = getPointSupport(pointsA, fcn.proxyA.count, -axisA)
        let pointA = transformPoint(xfA, pointsA[Int(indexA)])

        indexB = -1
        let pointB = transformPoint(xfB, fcn.witness2)

        return dot(pointA - pointB, normal)

    default:
        b3Assert(false)
        return 0
    }
}

/// Evaluate the separation at time `beta` for the given witness indices.
func evaluateSeparation(_ fcn: SeparationFunction, _ index1: Int32, _ index2: Int32, _ beta: Float) -> Float {
    let transform1 = getSweepTransform(fcn.sweepA, beta)
    let transform2 = getSweepTransform(fcn.sweepB, beta)

    let pointsA = fcn.proxyA.points!
    let pointsB = fcn.proxyB.points!

    switch fcn.type {
    case .vertices:
        let axis = fcn.witness1

        let point1 = transformPoint(transform1, pointsA[Int(index1)])
        let point2 = transformPoint(transform2, pointsB[Int(index2)])

        return dot(point2 - point1, axis)

    case .edges:
        let edge1 = rotateVector(transform1.q, fcn.witness1)
        let edge2 = rotateVector(transform2.q, fcn.witness2)
        var axis = cross(edge1, edge2)
        axis = normalize(axis)

        let point1 = transformPoint(transform1, pointsA[Int(index1)])
        let point2 = transformPoint(transform2, pointsB[Int(index2)])

        return dot(point2 - point1, axis)

    case .faceA:
        let axis = rotateVector(transform1.q, fcn.witness1)

        let point1 = transformPoint(transform1, fcn.witness2)
        let point2 = transformPoint(transform2, pointsB[Int(index2)])

        return dot(point2 - point1, axis)

    case .faceB:
        let axis = rotateVector(transform2.q, fcn.witness1)

        let point1 = transformPoint(transform1, pointsA[Int(index1)])
        let point2 = transformPoint(transform2, fcn.witness2)

        return dot(point1 - point2, axis)

    default:
        b3Assert(false)
        return 0
    }
}

/// Fall back to a fixed world axis when the edge/edge root finder fails.
func forceFixedAxis(_ fcn: inout SeparationFunction, _ beta: Float) {
    b3Assert(fcn.type == .edges)

    let transform1 = getSweepTransform(fcn.sweepA, beta)
    let transform2 = getSweepTransform(fcn.sweepB, beta)

    let edge1 = rotateVector(transform1.q, fcn.witness1)
    let edge2 = rotateVector(transform2.q, fcn.witness2)
    var axis = cross(edge1, edge2)
    axis = normalize(axis)

    fcn.type = .vertices
    fcn.witness1 = axis
    fcn.witness2 = Vec3()
}

/// Compute the upper bound on time before two shapes penetrate.
func timeOfImpact(_ input: TOIInput) -> TOIOutput {
    var output = TOIOutput()

    // Set these to invalid values so they can be validated on exit
    output.state = .unknown
    output.fraction = -1

    var sweepA = input.sweepA
    var sweepB = input.sweepB

    // Shift to origin
    let origin = sweepA.c1
    sweepA.c1 = Vec3()
    sweepA.c2 = sweepA.c2 - origin
    sweepB.c1 = sweepB.c1 - origin
    sweepB.c2 = sweepB.c2 - origin

    let proxyA = input.proxyA
    let proxyB = input.proxyB

    let maxPushBackIterations = proxyA.count + proxyB.count
    let tMax = input.maxFraction

    // Setup target distance and tolerance
    let linearSlop = B3.linearSlop
    let totalRadius = proxyA.radius + proxyB.radius
    let target = maxFloat(linearSlop, totalRadius - linearSlop)
    let tolerance = 0.25 * linearSlop
    b3Assert(target > tolerance)

    var t1: Float = 0
    let maxIterations = 25
    var distanceIterations: Int32 = 0

    // Prepare input for distance query.
    var cache = SimplexCache()
    var distanceInput = DistanceInput()
    distanceInput.proxyA = proxyA
    distanceInput.proxyB = proxyB
    distanceInput.useRadii = false

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    while true {
        // Get the distance between shapes. We can also use the results to get a separating axis.
        let xfA = getSweepTransform(sweepA, t1)
        let xfB = getSweepTransform(sweepB, t1)
        distanceInput.transform = invMulTransforms(xfA, xfB)
        let distanceOutput = shapeDistance(distanceInput, &cache)
        output.distance = distanceOutput.distance

        // The distance query runs in frame A, project the witness data back to the shifted world
        let worldNormal = rotateVector(xfA.q, distanceOutput.normal)
        let worldPointA = transformPoint(xfA, distanceOutput.pointA)
        let worldPointB = transformPoint(xfA, distanceOutput.pointB)

        output.distanceIterations += 1
        distanceIterations += 1

        // If the shapes are overlapped, we give up on continuous collision.
        if distanceOutput.distance <= 0 {
            output.state = .overlapped
            output.fraction = 0

            // Averaged hit point
            let pA = mulAdd(worldPointA, proxyA.radius, worldNormal)
            let pB = mulAdd(worldPointB, -proxyB.radius, worldNormal)
            output.point = lerp(pA, pB, 0.5)
            output.point = output.point + origin
            output.normal = worldNormal

            break
        }

        if distanceOutput.distance <= target + tolerance {
            // Success!
            output.state = .hit

            // Averaged hit point
            let pA = mulAdd(worldPointA, proxyA.radius, worldNormal)
            let pB = mulAdd(worldPointB, -proxyB.radius, worldNormal)
            output.point = lerp(pA, pB, 0.5)
            output.point = output.point + origin
            output.normal = worldNormal
            output.fraction = t1
            break
        }

        if distanceIterations == maxIterations {
            // Progress too slow. This can happen when a capsule rotates around a
            // triangle vertex.
            output.state = .failed
            output.fraction = t1

            // Averaged hit point
            let pA = mulAdd(worldPointA, input.proxyA.radius, worldNormal)
            let pB = mulAdd(worldPointB, -input.proxyB.radius, worldNormal)
            output.point = lerp(pA, pB, 0.5)
            output.point = output.point + origin
            output.normal = worldNormal
            break
        }

        // Initialize the separating axis.
        let function = makeSeparationFunction(cache, proxyA, sweepA, proxyB, sweepB, worldNormal, t1)
        var fcn = function

        // Compute the TOI on the separating axis. We do this by successively resolving the deepest
        // point.
        var done = false
        var t2 = tMax
        var pushBackIterations: Int32 = 0

        while true {
            var indexA: Int32 = 0
            var indexB: Int32 = 0
            var s2 = findMinSeparation(fcn, &indexA, &indexB, t2)

            // Is the final configuration separated?
            if s2 - target > tolerance {
                // Success!
                output.state = .separated
                output.fraction = input.maxFraction
                done = true
                break
            }

            // Has the separation reached tolerance?
            if s2 >= target - tolerance {
                // Advance the sweeps
                t1 = t2
                break
            }

            // Compute the initial separation of the witness points
            var s1 = evaluateSeparation(fcn, indexA, indexB, t1)

            // Check for overlap. This might happen if the root finder runs out of iterations.
            if s1 < target - tolerance {
                // Failed!
                b3Validate(false)
                output.state = .failed
                output.fraction = t1
                done = true
                break
            }

            // Has the separation reached tolerance?
            if s1 <= target + tolerance {
                // Success! t1 should hold the TOI (could be 0)
                output.state = .hit
                output.fraction = t1
                done = true
                break
            }

            // Compute 1D root of: f(x) - target = 0
            var rootIterationCount: Int32 = 0
            let maxRootIterations: Int32 = 50
            var a1 = t1
            var a2 = t2
            while true {
                // Use a mix of false position and bisection.
                let t: Float
                if rootIterationCount & 1 != 0 {
                    // False position to improve convergence.
                    t = a1 + (target - s1) * (a2 - a1) / (s2 - s1)
                } else {
                    // Bisection to guarantee progress.
                    t = 0.5 * (a1 + a2)
                }

                output.rootIterations += 1
                rootIterationCount += 1

                let s = evaluateSeparation(fcn, indexA, indexB, t)

                // Has the separation reached tolerance?
                if absFloat(s - target) <= tolerance {
                    // t2 holds a tentative value for t1
                    t2 = t
                    break
                }

                // Ensure we continue to bracket the root.
                if s > target {
                    a1 = t
                    s1 = s
                } else {
                    a2 = t
                    s2 = s
                }

                if rootIterationCount == maxRootIterations {
                    b3Validate(false)
                    break
                }
            }

            // Restart the inner loop if we have a failing edge case.
            if rootIterationCount == maxRootIterations - 1 && fcn.type == .edges {
                b3Validate(false)

                rootIterationCount = 0
                t2 = input.maxFraction
                forceFixedAxis(&fcn, t1)
                b3Assert(fcn.type != .edges)
            }

            output.pushBackIterations += 1
            pushBackIterations += 1

            if pushBackIterations == maxPushBackIterations {
                break
            }
        }

        if done {
            // Averaged hit point
            let pA = mulAdd(worldPointA, input.proxyA.radius, worldNormal)
            let pB = mulAdd(worldPointB, -input.proxyB.radius, worldNormal)
            output.point = lerp(pA, pB, 0.5)
            output.point = output.point + origin
            output.normal = worldNormal
            break
        }
    }

    // It is expected that the state and fraction are set before reaching this
    b3Assert(output.state != .unknown)
    b3Assert(output.fraction >= 0)

    return output
}
