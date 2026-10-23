// Sphere and capsule collision routines from convex_manifold.c.

// MARK: - Sphere contacts

/// Sphere versus sphere.
func collideSpheres(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ sphereA: Sphere,
    _ sphereB: Sphere,
    _ transformBtoA: Transform
) {
    if capacity == 0 {
        return
    }

    // Work in shapeB coordinates
    let center1 = sphereA.center
    let center2 = transformPoint(transformBtoA, sphereB.center)

    let totalRadius = sphereA.radius + sphereB.radius
    let maxDistance = totalRadius + B3.speculativeDistance
    let offset = center2 - center1
    let distanceSq = lengthSquared(offset)

    if distanceSq > maxDistance * maxDistance {
        // We found a separating axis
        return
    }

    var normal = Vec3(0, 1, 0)
    let distance = distanceSq.squareRoot()
    if distance * distance > 1000 * Float.leastNormalMagnitude {
        normal = (1 / distance) * offset
    }

    // Contact at the midpoint
    // 0.5 * ( ((c1 + rA*n) + c2) - rB*n )
    let point = 0.5 * (mulSub(mulAdd(center1, sphereA.radius, normal) + center2, sphereB.radius, normal))

    // Manifold in frame B
    manifold.normal = normal
    manifold.pointCount = 1

    let pt = manifold.points!
    pt[0].point = point
    pt[0].separation = distance - totalRadius
    pt[0].pair = featurePairSingle
}

/// Capsule versus sphere.
func collideCapsuleAndSphere(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ capsuleA: Capsule,
    _ sphereB: Sphere,
    _ transformBtoA: Transform
) {
    manifold.pointCount = 0

    if capacity < 1 {
        return
    }

    // Work in shape B coordinates
    let center = transformPoint(transformBtoA, sphereB.center)
    let center1 = capsuleA.center1
    let center2 = capsuleA.center2

    let totalRadius = sphereB.radius + capsuleA.radius
    let maxDistance = totalRadius + B3.speculativeDistance

    let closestPoint = pointToSegmentDistance(center1, center2, center)
    let offset = center - closestPoint
    let distanceSq = lengthSquared(offset)

    if distanceSq > maxDistance * maxDistance {
        // We found a separating axis
        return
    }

    var normal = Vec3(0, 1, 0)
    let distance = distanceSq.squareRoot()
    if distance * distance > 1000 * Float.leastNormalMagnitude {
        normal = (1 / distance) * offset
    }

    // Contact at the midpoint
    // 0.5 * (((center - sB*n) + closestPoint) + cA*n)
    let point = 0.5 * (mulAdd(mulSub(center, sphereB.radius, normal) + closestPoint, capsuleA.radius, normal))

    // Manifold in frame B
    manifold.normal = normal
    manifold.pointCount = 1

    let pt = manifold.points!
    pt[0].point = point
    pt[0].separation = distance - totalRadius
    pt[0].pair = featurePairSingle
}

/// Hull versus sphere.
func collideHullAndSphere(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ hullA: Hull,
    _ sphereB: Sphere,
    _ transformBtoA: Transform,
    _ cache: inout SimplexCache
) {
    manifold.pointCount = 0

    if capacity == 0 {
        return
    }

    let center = transformPoint(transformBtoA, sphereB.center)

    let speculativeDistance = B3.speculativeDistance

    // Work in shapeA coordinates

    var distanceOutput = DistanceOutput()
    withUnsafePointer(to: center) { centerPtr in
        var distanceInput = DistanceInput()
        distanceInput.proxyA = ShapeProxy(points: hullA.points, count: hullA.vertexCount, radius: 0)
        distanceInput.proxyB = ShapeProxy(points: centerPtr, count: 1, radius: 0)
        distanceInput.transform = Transform.identity
        distanceInput.useRadii = false

        distanceOutput = shapeDistance(distanceInput, &cache)
    }

    let radiusA: Float = 0
    let radiusB = sphereB.radius
    let radius = radiusA + radiusB

    if distanceOutput.distance > radius + speculativeDistance {
        // We found a separating axis
        cache = SimplexCache()
        return
    }

    let pt = manifold.points!

    if distanceOutput.distance > 100 * Float.ulpOfOne {
        // Shallow penetration
        let normal = normalize(distanceOutput.pointB - distanceOutput.pointA)

        // cA is the projection of the sphere center onto the hull (pointA if radiusA == 0).
        let cA = mulAdd(center, radiusA - dot(center - distanceOutput.pointA, normal), normal)

        // cB is the deepest point on the sphere with respect to the reference face
        let cB = mulSub(center, radiusB, normal)

        let point = lerp(cA, cB, 0.5)

        // Manifold in frame A
        manifold.normal = normal
        manifold.pointCount = 1

        pt[0].point = point
        pt[0].separation = distanceOutput.distance - radius
        pt[0].pair = featurePairSingle
    } else {
        // Deep penetration
        var bestIndex: Int32 = -1
        var bestDistance = -Float.greatestFiniteMagnitude
        let planes = hullA.planes!

        for index in 0..<Int(hullA.faceCount) {
            let plane = planes[index]

            let distance = planeSeparation(plane, center)
            if distance > bestDistance {
                bestIndex = Int32(index)
                bestDistance = distance
            }
        }
        b3Assert(bestIndex >= 0)

        let normal = planes[Int(bestIndex)].normal

        // cA is the projection of the sphere center onto the hull
        let cA = mulAdd(center, radiusA - dot(center - distanceOutput.pointA, normal), normal)

        // cB is the deepest point on the sphere with respect to the reference face
        let cB = mulSub(center, radiusB, normal)

        let point = lerp(cA, cB, 0.5)

        // Manifold in frame A
        manifold.normal = normal
        manifold.pointCount = 1

        pt[0].point = point
        pt[0].separation = bestDistance - radius
        pt[0].pair = featurePairSingle
    }
}

// MARK: - Capsule contacts

/// Capsule versus capsule.
func collideCapsules(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ capsuleA: Capsule,
    _ capsuleB: Capsule,
    _ transformBtoA: Transform
) {
    manifold.pointCount = 0

    if capacity < 2 {
        return
    }

    // Work in shapeA coordinates
    let centerA1 = capsuleA.center1
    let centerA2 = capsuleA.center2
    let centerB1 = transformPoint(transformBtoA, capsuleB.center1)
    let centerB2 = transformPoint(transformBtoA, capsuleB.center2)

    let radius = capsuleA.radius + capsuleB.radius
    let maxDistance = radius + B3.speculativeDistance

    let result = segmentDistance(centerA1, centerA2, centerB1, centerB2)
    let offset = result.point2 - result.point1
    let distanceSquared = lengthSquared(offset)
    let linearSlop = B3.linearSlop
    let minDistance = 0.01 * linearSlop

    if distanceSquared > maxDistance * maxDistance || distanceSquared < minDistance * minDistance {
        // We found a separating axis
        return
    }

    var lengthA: Float = 0
    let segmentA = centerA2 - centerA1
    let edgeA = getLengthAndNormalize(&lengthA, segmentA)
    if lengthA < B3.minCapsuleLength {
        return
    }

    var lengthB: Float = 0
    let segmentB = centerB2 - centerB1
    let edgeB = getLengthAndNormalize(&lengthB, segmentB)
    if lengthB < B3.minCapsuleLength {
        return
    }

    // Parallel edges: |eA x eB| = sin(alpha)
    let alphaTol: Float = 0.05
    let alphaTolSqr = alphaTol * alphaTol
    let axis = cross(edgeA, edgeB)

    // Try to create two contact points if the capsules are nearly parallel
    if lengthSquared(axis) < alphaTolSqr {
        // Clip segment B against side planes of segment A

        // Side planes of A
        var planesA = [Plane](repeating: Plane(), count: 2)
        planesA[0].normal = -edgeA
        planesA[0].offset = -dot(edgeA, capsuleA.center1)
        planesA[1].normal = edgeA
        planesA[1].offset = dot(edgeA, capsuleA.center2)

        // Clip points for B
        var verticesB = [ClipVertex](repeating: ClipVertex(), count: 2)
        verticesB[0].position = centerB1
        verticesB[0].separation = 0
        verticesB[0].pair = makeFeaturePair(.shapeA, 0, .shapeA, 0)
        verticesB[1].position = centerB2
        verticesB[1].separation = 0
        verticesB[1].pair = makeFeaturePair(.shapeA, 1, .shapeA, 1)

        var pointCount = verticesB.withUnsafeMutableBufferPointer {
            clipSegment($0.baseAddress!, planesA[0])
        }
        if pointCount == 2 {
            pointCount = verticesB.withUnsafeMutableBufferPointer {
                clipSegment($0.baseAddress!, planesA[1])
            }
        }

        if pointCount == 2 {
            // Closest points on A to the clipped points on B.
            let closestPoint1 = pointToSegmentDistance(centerA1, centerA2, verticesB[0].position)
            let closestPoint2 = pointToSegmentDistance(centerA1, centerA2, verticesB[1].position)

            let distance1 = distance(closestPoint1, verticesB[0].position)
            let distance2 = distance(closestPoint2, verticesB[1].position)
            if distance1 <= radius && distance2 <= radius {
                if distance1 < minDistance || distance2 < minDistance {
                    // Avoid divide by zero
                    return
                }

                let normal1 = (1 / distance1) * (verticesB[0].position - closestPoint1)
                let normal2 = (1 / distance2) * (verticesB[1].position - closestPoint2)
                let normal = normalize(normal1 + normal2)
                let radiusA = capsuleA.radius
                let radiusB = capsuleB.radius

                // Contact is at the midpoint: 0.5 * (((vB.pos + rA*nK) + cP) - rB*n)
                let point1 = 0.5 * mulSub(
                    mulAdd(verticesB[0].position, radiusA, normal1) + closestPoint1,
                    radiusB,
                    normal
                )
                let point2 = 0.5 * mulSub(
                    mulAdd(verticesB[1].position, radiusA, normal2) + closestPoint2,
                    radiusB,
                    normal
                )

                // Manifold in frame A
                manifold.normal = normal
                manifold.pointCount = 2

                let pt = manifold.points!
                pt[0].point = point1
                pt[0].separation = distance1 - radius
                pt[0].pair = verticesB[0].pair

                pt[1].point = point2
                pt[1].separation = distance2 - radius
                pt[1].pair = verticesB[1].pair

                return
            }
        }
    }

    var distance: Float = 0
    let normal = getLengthAndNormalize(&distance, offset)
    // Contact at the midpoint 0.5 * (((p1 + rA*n) + p2) - rB*n)
    let point = 0.5 * mulSub(
        mulAdd(result.point1, capsuleA.radius, normal) + result.point2,
        capsuleB.radius,
        normal
    )

    // Manifold in frame A
    manifold.normal = normal
    manifold.pointCount = 1

    let pt = manifold.points!
    pt[0].point = point
    pt[0].separation = distance - radius
    pt[0].pair = featurePairSingle
}

// MARK: - Hull and capsule

/// Query the face direction between a hull and a capsule.
func queryFaceDirectionHullAndCapsule(_ hull: Hull, _ capsule: Capsule, _ capsuleTransform: Transform) -> SeparatingAxis {
    var maxFaceIndex: Int32 = -1
    var maxVertexIndex: Int32 = -1
    var maxFaceSeparation = -Float.greatestFiniteMagnitude
    let planes = hull.planes!

    let capsulePoints = [
        transformPoint(capsuleTransform, capsule.center1),
        transformPoint(capsuleTransform, capsule.center2),
    ]

    for faceIndex in 0..<Int(hull.faceCount) {
        let plane = planes[faceIndex]

        let vertexIndex = capsulePoints.withUnsafeBufferPointer {
            getPointSupport($0.baseAddress!, 2, -plane.normal)
        }
        let support = capsulePoints[Int(vertexIndex)]
        let separation = planeSeparation(plane, support)
        if separation > maxFaceSeparation {
            maxVertexIndex = vertexIndex
            maxFaceIndex = Int32(faceIndex)
            maxFaceSeparation = separation
        }
    }

    var axis = SeparatingAxis()
    axis.normal = planes[Int(maxFaceIndex)].normal
    axis.separation = maxFaceSeparation
    axis.indexA = maxFaceIndex
    axis.indexB = maxVertexIndex
    axis.type = .faceAxisA
    return axis
}

/// Query the edge direction between a hull and a capsule.
func queryEdgeDirectionHullAndCapsule(_ hull: Hull, _ capsule: Capsule, _ capsuleTransform: Transform) -> SeparatingAxis {
    // Find axis of minimum penetration
    var maxNormal = Vec3()
    var maxSeparation = -Float.greatestFiniteMagnitude
    var maxIndexA: Int32 = B3.nullIndex
    var maxIndexB: Int32 = B3.nullIndex

    // We perform all computations in local space of the hull
    let pA = transformPoint(capsuleTransform, capsule.center1)
    let qA = transformPoint(capsuleTransform, capsule.center2)
    let eA = qA - pA

    let edges = hull.edges!
    let points = hull.points!
    let planes = hull.planes!
    let squaredTolerance = parallelEdgeTolerance * parallelEdgeTolerance

    var index = 0
    while index < Int(hull.halfEdgeCount) {
        let edge = edges + index
        let twin = edges + index + 1
        b3Assert(Int(edge.pointee.twin) == index + 1 && Int(twin.pointee.twin) == index)

        let qB = points[Int(twin.pointee.origin)]
        let uB = planes[Int(edge.pointee.face)].normal
        let vB = planes[Int(twin.pointee.face)].normal

        // An isolated edge (e.g. like in a capsule) defines a circle through the origin on the
        // Gauss map. So testing for overlap between this circle and the arc AB simplifies to a
        // plane test.
        let cba = dot(uB, eA)
        let dba = dot(vB, eA)

        if cba * dba < 0 {
            // Avoid nearly parallel edges that may lead to invalid separation values at the noise
            // floor.
            if maxFloat(cba * cba, dba * dba) < squaredTolerance * lengthSquared(eA) {
                index += 2
                continue
            }

            // The intersection of the arcs on the Gauss map is the edge pair axis. Cast the arc of
            // hull B (from uB to vB) against the plane containing the arc of hull A:
            // dot(uB + t * (vB - uB), eA) == 0
            // then t = cba / (cba - dba)
            let t = cba / (cba - dba)
            var axis = lerp(uB, vB, t)
            b3Validate(lengthSquared(axis) > 1000 * Float.leastNormalMagnitude)
            axis = normalize(axis)
            let separation = dot(axis, qA - qB)

            if separation > maxSeparation {
                // Note: We don't exit early if we find a separating axis here since we want to
                // find the best one for caching and account for the convex radius later.
                maxNormal = axis
                maxSeparation = separation
                maxIndexA = 0
                maxIndexB = Int32(index)
            }
        }

        index += 2
    }

    // Save result
    var axis = SeparatingAxis()
    axis.normal = maxNormal
    axis.separation = maxSeparation
    axis.indexA = maxIndexA
    axis.indexB = maxIndexB
    axis.type = .edgePairAxis
    return axis
}

/// Build a two point contact between a hull face and a capsule.
func buildHullFaceAndCapsuleContact(
    _ manifold: inout LocalManifold,
    _ hullA: Hull,
    _ capsuleB: Capsule,
    _ transformBtoA: Transform,
    _ query: SeparatingAxis
) -> Bool {
    // Work in shapeA coordinates
    let planes = hullA.planes!

    // Clip the capsule edge against the side planes of the reference face
    let refFace = query.indexA
    let refPlane = planes[Int(refFace)]

    var segmentB = [ClipVertex](repeating: ClipVertex(), count: 2)
    segmentB[0].position = transformPoint(transformBtoA, capsuleB.center1)
    segmentB[0].separation = 0
    segmentB[0].pair = makeFeaturePair(.shapeA, 0, .shapeA, 0)
    segmentB[1].position = transformPoint(transformBtoA, capsuleB.center2)
    segmentB[1].separation = 0
    segmentB[1].pair = makeFeaturePair(.shapeA, 1, .shapeA, 1)

    let pointCount = segmentB.withUnsafeMutableBufferPointer {
        clipSegmentToHullFace($0.baseAddress!, hullA, refFace)
    }
    if pointCount < 2 {
        return false
    }

    let distance1 = planeSeparation(refPlane, segmentB[0].position)
    let distance2 = planeSeparation(refPlane, segmentB[1].position)
    let speculativeDistance = B3.speculativeDistance

    if distance1 <= speculativeDistance || distance2 <= speculativeDistance {
        let normal = refPlane.normal
        let point1 = mulSub(segmentB[0].position, 0.5 * (distance1 + capsuleB.radius), normal)
        let point2 = mulSub(segmentB[1].position, 0.5 * (distance2 + capsuleB.radius), normal)

        // Manifold in frame A
        manifold.normal = normal
        manifold.pointCount = 2

        let pt = manifold.points!
        pt[0].point = point1
        pt[0].separation = distance1 - capsuleB.radius
        pt[0].pair = segmentB[0].pair

        pt[1].point = point2
        pt[1].separation = distance2 - capsuleB.radius
        pt[1].pair = segmentB[1].pair

        return true
    }

    return false
}

/// Build a single point edge contact between a hull edge and a capsule.
func buildHullAndCapsuleEdgeContact(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ hullA: Hull,
    _ capsuleB: Capsule,
    _ transformBtoA: Transform,
    _ query: SeparatingAxis
) -> Bool {
    if capacity < 1 {
        return false
    }

    // Work in shapeA coordinates

    let pc = transformPoint(transformBtoA, capsuleB.center1)
    let qc = transformPoint(transformBtoA, capsuleB.center2)
    let ec = qc - pc

    let edges = hullA.edges!
    let points = hullA.points!

    let edge2 = edges + Int(query.indexB)
    let twin2 = edges + Int(edge2.pointee.twin)
    let ph = points[Int(edge2.pointee.origin)]
    let qh = points[Int(twin2.pointee.origin)]
    let eh = qh - ph

    let normal = query.normal

    let result = lineDistance(ph, eh, pc, ec)

    if isWithinSegments(result) == false {
        // closest point beyond end points
        return false
    }

    let point = 0.5 * (mulSub(result.point1, capsuleB.radius, normal) + result.point2)
    let separation = dot(normal, result.point2 - result.point1)
    b3Validate(absFloat(separation - query.separation) < B3.linearSlop)

    // Manifold in frame A
    manifold.normal = normal
    manifold.pointCount = 1

    let pt = manifold.points!
    pt[0].point = point
    pt[0].separation = separation - capsuleB.radius
    pt[0].pair = makeFeaturePair(.shapeA, query.indexA, .shapeB, query.indexB)
    return true
}

/// Hull versus capsule.
func collideHullAndCapsule(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ hullA: Hull,
    _ capsuleB: Capsule,
    _ transformBtoA: Transform,
    _ cache: inout SimplexCache
) {
    manifold.pointCount = 0

    if capacity < 2 {
        return
    }

    // Work in shapeA coordinates
    let capsulePoints = [capsuleB.center1, capsuleB.center2]

    var distanceOutput = DistanceOutput()
    capsulePoints.withUnsafeBufferPointer { points in
        var distanceInput = DistanceInput()
        distanceInput.proxyA = ShapeProxy(points: hullA.points, count: hullA.vertexCount, radius: 0)
        distanceInput.proxyB = ShapeProxy(points: points.baseAddress, count: 2, radius: 0)
        distanceInput.transform = transformBtoA
        distanceInput.useRadii = false

        distanceOutput = shapeDistance(distanceInput, &cache)
    }

    let speculativeDistance = B3.speculativeDistance

    if distanceOutput.distance > capsuleB.radius + speculativeDistance {
        // We found a separating axis
        cache = SimplexCache()
        return
    }

    let pt = manifold.points!

    if distanceOutput.distance > 100 * Float.ulpOfOne {
        let planes = hullA.planes!

        // Shallow penetration
        let delta = distanceOutput.normal
        let refFace = findHullSupportFace(hullA, delta)
        let refPlane = planes[Int(refFace)]

        // Try to create two contact points if closest points difference is nearly parallel to face
        // normal
        let kTolerance: Float = 0.998
        if absFloat(dot(refPlane.normal, delta)) > kTolerance {
            // Clip capsule segment against side planes of reference face
            var verticesB = [ClipVertex](repeating: ClipVertex(), count: 2)
            verticesB[0].position = transformPoint(transformBtoA, capsuleB.center1)
            verticesB[0].separation = 0
            verticesB[0].pair = makeFeaturePair(.shapeA, 0, .shapeA, 0)
            verticesB[1].position = transformPoint(transformBtoA, capsuleB.center2)
            verticesB[1].separation = 0
            verticesB[1].pair = makeFeaturePair(.shapeA, 1, .shapeA, 1)

            let pointCount = verticesB.withUnsafeMutableBufferPointer {
                clipSegmentToHullFace($0.baseAddress!, hullA, refFace)
            }

            if pointCount == 2 {
                let distance1 = planeSeparation(refPlane, verticesB[0].position)
                let distance2 = planeSeparation(refPlane, verticesB[1].position)
                if distance1 <= capsuleB.radius + speculativeDistance
                    || distance2 <= capsuleB.radius + speculativeDistance
                {
                    let normal = refPlane.normal
                    let point1 = mulSub(verticesB[0].position, 0.5 * (capsuleB.radius + distance1), normal)
                    let point2 = mulSub(verticesB[1].position, 0.5 * (capsuleB.radius + distance2), normal)

                    // Manifold in frame A
                    manifold.normal = normal
                    manifold.pointCount = 2

                    pt[0].point = point1
                    pt[0].separation = distance1 - capsuleB.radius
                    pt[0].pair = verticesB[0].pair

                    pt[1].point = point2
                    pt[1].separation = distance2 - capsuleB.radius
                    pt[1].pair = verticesB[1].pair

                    return
                }
            }
        }

        // Create contact from closest points
        let point = 0.5 * (mulSub(distanceOutput.pointA, capsuleB.radius, delta) + distanceOutput.pointB)

        // Manifold in frame A
        manifold.normal = delta
        manifold.pointCount = 1

        pt[0].point = point
        pt[0].separation = distanceOutput.distance - capsuleB.radius
        pt[0].pair = featurePairSingle
        return
    }

    // Deep penetration

    let faceQuery = queryFaceDirectionHullAndCapsule(hullA, capsuleB, transformBtoA)
    if faceQuery.separation > capsuleB.radius {
        // We found a separating axis
        return
    }

    let edgeQuery = queryEdgeDirectionHullAndCapsule(hullA, capsuleB, transformBtoA)
    if edgeQuery.separation > capsuleB.radius {
        // We found a separating axis
        return
    }

    // Create face contact
    var faceSeparation = faceQuery.separation - capsuleB.radius
    _ = buildHullFaceAndCapsuleContact(&manifold, hullA, capsuleB, transformBtoA, faceQuery)
    b3Validate(manifold.pointCount == 0 || manifold.pointCount == 2)
    if manifold.pointCount == 2 {
        // This becomes the clipped separation.
        faceSeparation = minFloat(pt[0].separation, pt[1].separation)
    }

    // Is there a valid edge-edge axis?
    if edgeQuery.indexA == B3.nullIndex {
        return
    }

    // Face contact can be empty if it does not realize the axis of minimum penetration.
    // Create edge contact if face contact fails or edge contact is significantly better!
    let linearSlop = B3.linearSlop
    let edgeSeparation = edgeQuery.separation - capsuleB.radius
    if manifold.pointCount == 0 || edgeSeparation > faceSeparation + linearSlop {
        // Edge contact
        _ = buildHullAndCapsuleEdgeContact(&manifold, capacity, hullA, capsuleB, transformBtoA, edgeQuery)
    }
}
