// Triangle versus sphere, capsule, and hull collision (triangle_manifold.c).

/// Triangle data in the local space of the other shape.
struct TriangleData {
    var v1 = Vec3()
    var v2 = Vec3()
    var v3 = Vec3()
    var e1 = Vec3()
    var e2 = Vec3()
    var e3 = Vec3()
    var plane = Plane()
    var flags: Int32 = 0

    init() {}
}

/// Indexed by the 3-bit vertex mask.
private let triangleFeatures: [TriangleFeature] = [
    .featureNone,  // 000 (unreachable)
    .featureVertex1,  // 001
    .featureVertex2,  // 010
    .featureEdge1,  // 011 v1,v2
    .featureVertex3,  // 100
    .featureEdge3,  // 101 v1,v3
    .featureEdge2,  // 110 v2,v3
    .featureTriangleFace,  // 111
]

/// Map the GJK simplex cache to the triangle feature that generated it.
func getTriangleFeature(_ cache: SimplexCache) -> TriangleFeature {
    let count = Int(cache.count)
    b3Assert(0 < count && count < 4)

    // Bit i set means triangle vertex i participates in the simplex.
    var mask = 0
    for i in 0..<count {
        b3Assert(cache.indexA[i] < 3)
        mask |= 1 << Int(cache.indexA[i])
    }

    return triangleFeatures[mask]
}

// MARK: - Triangle and sphere

/// Triangle versus sphere.
func collideTriangleAndSphere(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ triangleA: UnsafePointer<Vec3>,
    _ sphereB: Sphere
) {
    manifold.pointCount = 0

    if capacity == 0 {
        return
    }

    let center = sphereB.center
    let v1 = triangleA[0]
    let v2 = triangleA[1]
    let v3 = triangleA[2]
    let plane = makePlaneFromPoints(v1, v2, v3)

    let offset = planeSeparation(plane, center)
    if offset < 0 {
        // Cull back side collision
        return
    }

    let radius = sphereB.radius

    // Closest point on triangle to sphere center
    let closest = closestPointOnTriangle(v1, v2, v3, center)

    // Test separating axis
    let squaredDistance = distanceSquared(closest.point, center)
    let speculativeDistance = B3.speculativeDistance
    let maxDistance = radius + speculativeDistance
    if squaredDistance > maxDistance * maxDistance {
        return
    }

    let distance = squaredDistance.squareRoot()

    // Normal points from triangle to sphere
    var normal = Vec3()
    if distance * distance > 1000 * Float.leastNormalMagnitude {
        normal = (1 / distance) * (center - closest.point)
    } else {
        normal = normalize(cross(v2 - v1, v3 - v1))
    }

    // contact point mid-way
    // p = 0.5 * (c + q) - 0.5 * r * n
    let contactPoint = 0.5 * ((center - radius * normal) + closest.point)

    manifold.normal = normal
    manifold.pointCount = 1
    manifold.feature = closest.feature
    manifold.squaredDistance = squaredDistance

    let mp = manifold.points!
    mp[0].point = contactPoint
    mp[0].separation = distance - radius
    mp[0].pair = featurePairSingle
}

// MARK: - Triangle and capsule

/// Clip a segment against the three side planes of a triangle face.
func clipSegmentToTriangleFace(
    _ segment: UnsafeMutablePointer<ClipVertex>,
    _ points: UnsafePointer<Vec3>,
    _ plane: Plane
) -> Bool {
    var vertex1 = points[2]
    for i in 0..<3 {
        let vertex2 = points[i]
        let tangent = normalize(vertex2 - vertex1)
        let binormal = cross(tangent, plane.normal)

        let clipPlane = makePlaneFromNormalAndPoint(binormal, vertex1)

        var vertexCount: Int32 = 0
        let p1 = segment[0]
        let p2 = segment[1]

        let distance1 = planeSeparation(clipPlane, p1.position)
        let distance2 = planeSeparation(clipPlane, p2.position)

        // If the points are behind the plane
        if distance1 <= 0 {
            segment[Int(vertexCount)] = p1
            vertexCount += 1
        }
        if distance2 <= 0 {
            segment[Int(vertexCount)] = p2
            vertexCount += 1
        }

        // If the points are on different sides of the plane
        if (distance1 > 0) != (distance2 > 0) {
            // Find intersection point of edge and plane
            let t = distance1 / (distance1 - distance2)
            segment[Int(vertexCount)].position = lerp(p1.position, p2.position, t)
            segment[Int(vertexCount)].pair = distance1 > 0 ? p1.pair : p2.pair
            vertexCount += 1
        }

        if vertexCount != 2 {
            return false
        }

        vertex1 = vertex2
    }

    return true
}

/// Triangle face direction versus a capsule.
func queryTriangleFaceAndCapsule(_ plane: Plane, _ capsule: Capsule) -> SeparatingAxis {
    let separation1 = planeSeparation(plane, capsule.center1)
    let separation2 = planeSeparation(plane, capsule.center2)

    var axis = SeparatingAxis()
    axis.normal = plane.normal

    if separation1 < separation2 {
        axis.separation = separation1
        axis.indexA = 0
        axis.indexB = 0
    } else {
        axis.separation = separation2
        axis.indexA = 0
        axis.indexB = 1
    }

    return axis
}

/// Triangle edges versus a capsule edge.
func queryTriangleAndCapsuleEdges(
    _ vertices: UnsafePointer<Vec3>,
    _ plane: Plane,
    _ capsule: Capsule
) -> SeparatingAxis {
    // Work in the local space of the capsule
    let p1 = capsule.center1
    let p2 = capsule.center2
    let capsuleEdge = p2 - p1

    // Find axis of minimum penetration
    var maxNormal = Vec3()
    var maxSeparation = -Float.greatestFiniteMagnitude
    var maxIndex1: Int32 = B3.nullIndex
    var maxIndex2: Int32 = B3.nullIndex
    let squaredTolerance: Float = 0.005 * 0.005

    var edgeIndex: Int32 = 2
    let a = dot(capsuleEdge, plane.normal)
    var v1 = vertices[2]
    for index in 0..<3 {
        let v2 = vertices[index]
        let triangleEdge = v2 - v1
        let sideNormal = normalize(cross(triangleEdge, plane.normal))

        // Pretend the triangle edge embeds a zero area face with a side normal. This
        // provides a way to find an edge-edge normal that points outward from
        // the triangle.
        let b = dot(capsuleEdge, sideNormal)

        // Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
        if a * a + b * b < squaredTolerance * lengthSquared(capsuleEdge) {
            v1 = v2
            edgeIndex = Int32(index)
            continue
        }

        // Similar to hull vs hull (b3QueryEdgeDirections)
        var axis: Vec3
        if a * b <= 0 {
            let t = b / (b - a)
            axis = lerp(sideNormal, plane.normal, t)
        } else {
            let t = b / (a + b)
            axis = lerp(sideNormal, -plane.normal, t)
        }

        b3Validate(lengthSquared(axis) > 1000 * Float.leastNormalMagnitude)
        axis = normalize(axis)
        let separation = dot(axis, p1 - v1)

        if separation > maxSeparation {
            // Note: We don't exit early if we find a separating axis here since we want to
            // find the best one for caching and account for the convex radius later.
            maxNormal = axis
            maxSeparation = separation
            maxIndex1 = edgeIndex
            maxIndex2 = 0
        }

        v1 = v2
        edgeIndex = Int32(index)
    }

    // Save result
    var result = SeparatingAxis()
    result.normal = maxNormal
    result.separation = maxSeparation
    result.indexA = maxIndex1
    result.indexB = maxIndex2
    result.type = .invalidAxis
    return result
}

/// Build the face contact between a triangle and a capsule.
func buildTriangleAndCapsuleFaceContact(
    _ manifold: inout LocalManifold,
    _ triangle: UnsafePointer<Vec3>,
    _ plane: Plane,
    _ capsule: Capsule
) {
    b3Assert(manifold.pointCount == 0)

    var segment: InlineArray<2, ClipVertex> = [ClipVertex(), ClipVertex()]
    segment[0].position = capsule.center1
    segment[0].separation = 0
    segment[0].pair = makeFeaturePair(.shapeA, 0, .shapeA, 0)
    segment[1].position = capsule.center2
    segment[1].separation = 0
    segment[1].pair = makeFeaturePair(.shapeA, 1, .shapeA, 1)

    let havePoints = withInlineBuffer(&segment) {
        clipSegmentToTriangleFace($0.baseAddress!, triangle, plane)
    }
    if havePoints == false {
        return
    }

    let radius = capsule.radius
    let distance1 = planeSeparation(plane, segment[0].position)
    let distance2 = planeSeparation(plane, segment[1].position)

    let speculativeDistance = B3.speculativeDistance
    if distance1 > speculativeDistance + radius && distance2 > speculativeDistance + radius {
        return
    }

    // Average points. Half-way between capsule bottom and triangle plane.
    let point1 = mulSub(segment[0].position, 0.5 * (distance1 + capsule.radius), plane.normal)
    let point2 = mulSub(segment[1].position, 0.5 * (distance2 + capsule.radius), plane.normal)

    manifold.normal = plane.normal
    manifold.feature = .featureTriangleFace
    manifold.pointCount = 2

    let pt = manifold.points!
    pt[0].point = point1
    pt[0].separation = distance1 - capsule.radius
    pt[0].pair = segment[0].pair

    pt[1].point = point2
    pt[1].separation = distance2 - capsule.radius
    pt[1].pair = segment[1].pair
}

/// Build the edge contact between a triangle and a capsule.
func buildTriangleAndCapsuleEdgeContact(
    _ manifold: inout LocalManifold,
    _ triangle: UnsafePointer<Vec3>,
    _ plane: Plane,
    _ capsule: Capsule,
    _ query: SeparatingAxis
) {
    b3Assert(0 <= query.indexA && query.indexA < 3)

    let p1 = capsule.center1
    let p2 = capsule.center2
    let capsuleEdge = p2 - p1

    let v1 = triangle[Int(query.indexA)]
    let v2 = triangle[(Int(query.indexA) + 1) % 3]
    let triangleEdge = v2 - v1

    let sideNormal = normalize(cross(triangleEdge, plane.normal))

    // Pretend the triangle edge embeds a zero area face with a side normal. This
    // provides a way to find an edge-edge normal that points outward from
    // the triangle.
    let a = dot(capsuleEdge, plane.normal)
    let b = dot(capsuleEdge, sideNormal)

    // Is the capsule edge parallel to the triangle edge? If so, face contact can handle it.
    let squaredTolerance: Float = 0.005 * 0.005
    if a * a + b * b < squaredTolerance * lengthSquared(capsuleEdge) {
        return
    }

    // Similar to hull vs hull (b3QueryEdgeDirections)
    let normal = query.normal
    let result = lineDistance(v1, triangleEdge, p1, capsuleEdge)

    if result.fraction1 < 0 || 1 < result.fraction1 || result.fraction2 < 0 || 1 < result.fraction2 {
        // closest point beyond end points
        return
    }

    let point = lerp(result.point1, mulSub(result.point2, capsule.radius, normal), 0.5)
    let separation = dot(normal, p1 - v1)

    manifold.normal = normal
    manifold.pointCount = 1

    manifold.feature = query.indexA == 0 ? .featureEdge1 : (query.indexA == 1 ? .featureEdge2 : .featureEdge3)

    let pt = manifold.points!
    pt[0].point = point
    pt[0].separation = separation - capsule.radius
    pt[0].pair = makeFeaturePair(.shapeA, query.indexA, .shapeB, query.indexB)
}

/// Triangle versus capsule.
func collideTriangleAndCapsule(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ triangleA: UnsafePointer<Vec3>,
    _ capsuleB: Capsule,
    _ cache: inout SimplexCache
) {
    manifold.pointCount = 0

    if capacity < 2 {
        return
    }

    let v1 = triangleA[0]
    let v2 = triangleA[1]
    let v3 = triangleA[2]
    let plane = makePlaneFromPoints(v1, v2, v3)
    let capsuleCenter = lerp(capsuleB.center1, capsuleB.center2, 0.5)

    let offset = planeSeparation(plane, capsuleCenter)
    if offset < 0 {
        // Cull back side collision
        return
    }

    let capsulePoints: InlineArray<2, Vec3> = [capsuleB.center1, capsuleB.center2]

    var distanceOutput = DistanceOutput()
    withInlineReadBuffer(capsulePoints) { points in
        var distanceInput = DistanceInput()
        distanceInput.proxyA = ShapeProxy(points: triangleA, count: 3, radius: 0)
        distanceInput.proxyB = ShapeProxy(points: points.baseAddress, count: 2, radius: 0)
        distanceInput.transform = Transform.identity
        distanceInput.useRadii = false

        distanceOutput = shapeDistance(distanceInput, &cache)
    }

    let speculativeDistance = B3.speculativeDistance
    let radius = capsuleB.radius
    if distanceOutput.distance > radius + speculativeDistance {
        // Shapes are separated, persist the cache
        return
    }

    let pt = manifold.points!

    if distanceOutput.distance > 100 * Float.ulpOfOne {
        // Shallow penetration
        let delta = normalize(distanceOutput.pointB - distanceOutput.pointA)

        // Try to create two contact points if closest points difference is nearly parallel to
        // face normal
        let kTolerance: Float = 0.2
        let cosAngle = absFloat(dot(plane.normal, delta))
        if cosAngle > kTolerance {
            // Clip capsule segment against side planes of reference face
            var segment: InlineArray<2, ClipVertex> = [ClipVertex(), ClipVertex()]
            segment[0].position = capsuleB.center1
            segment[0].separation = 0
            segment[0].pair = makeFeaturePair(.shapeA, 0, .shapeA, 0)
            segment[1].position = capsuleB.center2
            segment[1].separation = 0
            segment[1].pair = makeFeaturePair(.shapeA, 1, .shapeA, 1)

            let havePoints = withInlineBuffer(&segment) {
                clipSegmentToTriangleFace($0.baseAddress!, triangleA, plane)
            }

            if havePoints {
                let distance1 = planeSeparation(plane, segment[0].position)
                let distance2 = planeSeparation(plane, segment[1].position)

                let normal = plane.normal
                let point1 = mulSub(segment[0].position, 0.5 * (radius + distance1), normal)
                let point2 = mulSub(segment[1].position, 0.5 * (radius + distance2), normal)

                // Normal points from triangle to capsule
                manifold.normal = normal
                manifold.feature = .featureTriangleFace
                manifold.pointCount = 2

                pt[0].point = point1
                pt[0].separation = distance1 - radius
                pt[0].pair = segment[0].pair

                pt[1].point = point2
                pt[1].separation = distance2 - radius
                pt[1].pair = segment[1].pair

                return
            }
        }

        // Create contact from closest points.
        let point = lerp(distanceOutput.pointA, mulSub(distanceOutput.pointB, radius, delta), 0.5)

        // Normal points from triangle to capsule.
        manifold.normal = delta
        manifold.pointCount = 1
        manifold.feature = getTriangleFeature(cache)

        pt[0].point = point
        pt[0].separation = distanceOutput.distance - radius
        pt[0].pair = featurePairSingle

        return
    }

    // Deep penetration

    let faceQuery = queryTriangleFaceAndCapsule(plane, capsuleB)
    if faceQuery.separation > radius {
        // Shapes are separated. Should be impossible for a reasonable capsule radius.
        return
    }

    let edgeQuery = queryTriangleAndCapsuleEdges(triangleA, plane, capsuleB)
    if edgeQuery.separation > radius {
        // Shapes are separated. Should be impossible for a reasonable capsule radius.
        return
    }

    // Create face contact
    var faceSeparation = faceQuery.separation - radius
    buildTriangleAndCapsuleFaceContact(&manifold, triangleA, plane, capsuleB)
    b3Validate(manifold.pointCount == 0 || manifold.pointCount == 2)
    if manifold.pointCount == 2 {
        // This becomes the clipped separation.
        faceSeparation = minFloat(pt[0].separation, pt[1].separation)
    }

    if edgeQuery.indexA == B3.nullIndex {
        // No valid edge contact.
        return
    }

    // Face contact can be empty if it does not realize the axis of minimum penetration.
    // Create edge contact if face contact fails or edge contact is significantly better!
    let linearSlop = B3.linearSlop
    let edgeSeparation = edgeQuery.separation - radius
    if manifold.pointCount == 0 || edgeSeparation > faceSeparation + linearSlop {
        // Edge contact
        buildTriangleAndCapsuleEdgeContact(&manifold, triangleA, plane, capsuleB, edgeQuery)
    }
}

// MARK: - Triangle and hull

/// The support vertex of a triangle.
func getTriangleSupport(_ points: UnsafePointer<Vec3>, _ direction: Vec3) -> Int32 {
    var index: Int32 = 0
    var distance = dot(points[0], direction)

    var d = dot(points[1], direction)
    if d > distance {
        distance = d
        index = 1
    }

    d = dot(points[2], direction)
    if d > distance {
        return 2
    }

    return index
}

/// Triangle face direction versus a hull.
func queryTriangleFace(_ triangle: TriangleData, _ hull: Hull) -> SeparatingAxis {
    let hullPoints = hull.points!
    let plane = triangle.plane
    let normal = -plane.normal
    let vertexIndex = findHullSupportVertex(hull, normal)
    let support = hullPoints[Int(vertexIndex)]
    let separation = planeSeparation(plane, support)

    var axis = SeparatingAxis()
    axis.normal = plane.normal
    axis.separation = separation
    axis.indexA = 0
    axis.indexB = vertexIndex
    axis.type = .faceAxisA
    return axis
}

/// Hull face direction versus a triangle.
func queryHullFace(_ triangle: TriangleData, _ hull: Hull) -> SeparatingAxis {
    let hullPlanes = hull.planes!
    let faceCount = hull.faceCount

    let trianglePoints: InlineArray<3, Vec3> = [triangle.v1, triangle.v2, triangle.v3]

    var maxNormal = Vec3()
    var maxFaceSeparation = -Float.infinity
    var maxFaceIndex: Int32 = B3.nullIndex
    var maxVertexIndex: Int32 = B3.nullIndex

    for faceIndex in 0..<Int(faceCount) {
        let plane = hullPlanes[faceIndex]

        let vertexIndex = withInlineReadBuffer(trianglePoints) {
            getTriangleSupport($0.baseAddress!, -plane.normal)
        }
        let support = trianglePoints[Int(vertexIndex)]
        let separation = planeSeparation(plane, support)
        if separation > maxFaceSeparation {
            maxNormal = plane.normal
            maxFaceSeparation = separation
            maxFaceIndex = Int32(faceIndex)
            maxVertexIndex = vertexIndex
        }
    }

    // Normal points from triangle to hull
    var axis = SeparatingAxis()
    axis.normal = -maxNormal
    axis.separation = maxFaceSeparation
    axis.indexA = maxVertexIndex
    axis.indexB = maxFaceIndex
    axis.type = .faceAxisB
    return axis
}

/// Triangle edge versus hull edge.
func queryTriangleAndHullEdges(_ triangle: TriangleData, _ hull: Hull) -> SeparatingAxis {
    var result = SeparatingAxis()
    result.normal = Vec3()
    result.separation = -Float.infinity
    result.indexA = B3.nullIndex
    result.indexB = B3.nullIndex
    result.type = .edgePairAxis

    let trianglePoints: InlineArray<3, Vec3> = [triangle.v1, triangle.v2, triangle.v3]
    let triangleEdges: InlineArray<3, Vec3> = [triangle.e1, triangle.e2, triangle.e3]

    let triNormal = triangle.plane.normal

    let hullEdges = hull.edges!
    let hullPoints = hull.points!
    let hullPlanes = hull.planes!
    let edgeCount = hull.halfEdgeCount
    let squaredTolerance: Float = 0.005 * 0.005

    var i = 0
    while i < Int(edgeCount) {
        let edge = hullEdges + i
        let twin = hullEdges + i + 1
        b3Assert(Int(edge.pointee.twin) == i + 1 && Int(twin.pointee.twin) == i)

        let hullPoint = hullPoints[Int(edge.pointee.origin)]
        let hullEdge = hullPoints[Int(twin.pointee.origin)] - hullPoint

        let hullNormal1 = hullPlanes[Int(edge.pointee.face)].normal
        let hullNormal2 = hullPlanes[Int(twin.pointee.face)].normal

        for j in 0..<3 {
            let triEdge = triangleEdges[j]

            let cab = dot(hullNormal1, triEdge)
            let dab = dot(hullNormal2, triEdge)
            let bcd = dot(triNormal, hullEdge)
            if cab * dab >= 0 || cab * bcd <= 0 {
                continue
            }

            // Avoid nearly parallel edges that may lead to invalid separation values at the noise
            // floor.
            if maxFloat(cab * cab, dab * dab) < squaredTolerance * lengthSquared(triEdge) {
                continue
            }

            // Similar to hull vs hull (b3QueryEdgeDirections)
            // dot(hullNormal1 + t * (hullNormal2 - hullNormal1), triEdge) = 0
            // Normal points out of hull by construction.
            let t = cab / (cab - dab)
            var axis = lerp(hullNormal1, hullNormal2, t)
            b3Validate(lengthSquared(axis) > 1000 * Float.leastNormalMagnitude)
            axis = normalize(axis)
            let separation = dot(axis, trianglePoints[j] - hullPoint)

            if separation > result.separation {
                // Note: We don't exit early if we find a separating axis here since we want to
                // find the best one for caching.
                // Flip normal to point from triangle to hull.
                result.normal = -axis
                result.separation = separation
                result.indexA = Int32(j)
                result.indexB = Int32(i)
            }
        }

        i += 2
    }

    return result
}

/// Clip the triangle face against a hull face.
func collideHullFace(
    _ manifold: inout LocalManifold,
    _ pointCapacity: Int32,
    _ triangle: TriangleData,
    _ hull: Hull,
    _ query: SeparatingAxis,
    _ cache: inout SATCache,
    _ enableSpeculative: Bool
) -> Float {
    b3Validate(query.type == .faceAxisB)
    b3Validate(0 <= query.indexA && query.indexA < 3)
    b3Validate(0 <= query.indexB && query.indexB < hull.faceCount)

    manifold.pointCount = 0

    let hullFaces = hull.faces!
    let hullEdges = hull.edges!
    let hullPlanes = hull.planes!
    let hullPoints = hull.points!

    // Reference hull face
    let refPlane = hullPlanes[Int(query.indexB)]

    // Build clip polygon from triangle face (the incident face)
    var buffer1 = InlineArray<64, ClipVertex>(repeating: ClipVertex())
    var buffer2 = InlineArray<64, ClipVertex>(repeating: ClipVertex())
    return withInlineBuffer(&buffer1) { buffer1 in
        return withInlineBuffer(&buffer2) { buffer2 in

            let v1 = triangle.v1
            let v2 = triangle.v2
            let v3 = triangle.v3
            buffer1[0].position = v1
            buffer1[0].separation = planeSeparation(refPlane, v1)
            buffer1[0].pair = makeFeaturePair(.shapeB, 2, .shapeB, 0)
            buffer1[1].position = v2
            buffer1[1].separation = planeSeparation(refPlane, v2)
            buffer1[1].pair = makeFeaturePair(.shapeB, 0, .shapeB, 1)
            buffer1[2].position = v3
            buffer1[2].separation = planeSeparation(refPlane, v3)
            buffer1[2].pair = makeFeaturePair(.shapeB, 1, .shapeB, 2)

            var pointCount: Int32 = 3

            // Clip triangle face against side planes of reference face
            var useBuffer1AsInput = true
            var useBuffer1AsOutput = false

            let face = hullFaces + Int(query.indexB)
            var edgeIndex = Int32(face.pointee.edge)

            repeat {
                let edge = hullEdges + Int(edgeIndex)
                let nextEdgeIndex = Int32(edge.pointee.next)
                let next = hullEdges + Int(nextEdgeIndex)
                let vertex1 = hullPoints[Int(edge.pointee.origin)]
                let vertex2 = hullPoints[Int(next.pointee.origin)]
                let tangent = normalize(vertex2 - vertex1)
                let binormal = cross(tangent, refPlane.normal)

                let clipPlane = makePlaneFromNormalAndPoint(binormal, vertex1)

                if useBuffer1AsInput {
                    useBuffer1AsOutput = true
                    pointCount = clipPolygon(buffer2.baseAddress!, buffer1.baseAddress!, pointCount, clipPlane, edgeIndex, refPlane)
                } else {
                    useBuffer1AsOutput = false
                    pointCount = clipPolygon(buffer1.baseAddress!, buffer2.baseAddress!, pointCount, clipPlane, edgeIndex, refPlane)
                }
                b3Assert(pointCount <= maxClipPoints)

                if pointCount < 3 {
                    // Using a stale cache
                    cache = SATCache()
                    return query.separation
                }

                useBuffer1AsInput = !useBuffer1AsInput
                edgeIndex = nextEdgeIndex
            } while edgeIndex != Int32(face.pointee.edge)

            pointCount = minInt(pointCount, pointCapacity)
            var minSeparation = Float.greatestFiniteMagnitude
            var finalPointCount: Int32 = 0

            let input = useBuffer1AsOutput ? buffer2 : buffer1

            for i in 0..<Int(pointCount) {
                let clipPoint = input[i]
                minSeparation = minFloat(minSeparation, clipPoint.separation)

                if enableSpeculative == false && clipPoint.separation > 0 {
                    continue
                }

                // Move point onto hull face for improved culling
                let point = mulSub(clipPoint.position, clipPoint.separation, refPlane.normal)

                let pt = manifold.points!
                pt[Int(finalPointCount)].point = point
                pt[Int(finalPointCount)].separation = clipPoint.separation
                pt[Int(finalPointCount)].pair = flipPair(clipPoint.pair)

                finalPointCount += 1
            }

            let speculativeDistance = enableSpeculative ? B3.speculativeDistance : 0
            if minSeparation > speculativeDistance {
                // This can occur with a stale SAT cache
                manifold.pointCount = 0
                cache = SATCache()
                return minSeparation
            }

            manifold.pointCount = finalPointCount
            manifold.normal = -refPlane.normal
            manifold.feature = .featureHullFace

            // Save cache
            cache.separation = minSeparation
            cache.type = .faceAxisB
            cache.indexA = UInt8(truncatingIfNeeded: query.indexA)
            cache.indexB = UInt8(truncatingIfNeeded: query.indexB)
            return minSeparation
        }
    }
}

/// Clip the incident hull face against the triangle face.
func collideTriangleFace(
    _ manifold: inout LocalManifold,
    _ pointCapacity: Int32,
    _ triangle: TriangleData,
    _ hull: Hull,
    _ query: SeparatingAxis,
    _ cache: inout SATCache,
    _ enableSpeculative: Bool
) -> Float {
    b3Validate(query.type == .faceAxisA)
    b3Validate(query.indexA == 0)
    b3Validate(0 <= query.indexB && query.indexB < hull.vertexCount)
    b3Validate(manifold.pointCount == 0)

    let hullFaces = hull.faces!
    let hullEdges = hull.edges!
    let hullPoints = hull.points!

    // Find incident face
    let refPlane = triangle.plane

    let incFace = findIncidentFace(hull, refPlane.normal, query.indexB)

    // Build clip polygon from incident face
    let bigCapacity = Int(2 * maxClipPoints)
    var buffer1 = InlineArray<128, ClipVertex>(repeating: ClipVertex())
    var buffer2 = InlineArray<128, ClipVertex>(repeating: ClipVertex())
    return withInlineBuffer(&buffer1) { buffer1 in
        return withInlineBuffer(&buffer2) { buffer2 in
            var pointCount: Int32 = 0
            let face = hullFaces + Int(incFace)
            var hullEdgeIndex = Int32(face.pointee.edge)

            repeat {
                let edge = hullEdges + Int(hullEdgeIndex)

                let nextEdgeIndex = Int32(edge.pointee.next)
                let next = hullEdges + Int(nextEdgeIndex)

                let hullPoint = hullPoints[Int(next.pointee.origin)]
                buffer1[Int(pointCount)].position = hullPoint
                buffer1[Int(pointCount)].separation = planeSeparation(refPlane, hullPoint)
                buffer1[Int(pointCount)].pair = makeFeaturePair(.shapeB, hullEdgeIndex, .shapeB, nextEdgeIndex)

                pointCount += 1
                hullEdgeIndex = nextEdgeIndex
            } while hullEdgeIndex != Int32(face.pointee.edge) && pointCount < Int32(bigCapacity)

            b3Assert(pointCount >= 3)

            // Clip incident face against side planes of reference face (triangle)
            var useBuffer1AsInput = true
            var useBuffer1AsOutput = false

            let trianglePoints: InlineArray<3, Vec3> = [triangle.v1, triangle.v2, triangle.v3]
            let triangleEdges: InlineArray<3, Vec3> = [triangle.e1, triangle.e2, triangle.e3]

            for i in 0..<3 where pointCount > 0 {
                var sideNormal = cross(triangleEdges[i], refPlane.normal)
                sideNormal = normalize(sideNormal)

                let clipPlane = makePlaneFromNormalAndPoint(sideNormal, trianglePoints[i])

                if useBuffer1AsInput {
                    useBuffer1AsOutput = true
                    pointCount = clipPolygon(
                                buffer2.baseAddress!, buffer1.baseAddress!, pointCount, clipPlane, Int32(i), refPlane
                            )
                } else {
                    useBuffer1AsOutput = false
                    pointCount = clipPolygon(
                                buffer1.baseAddress!, buffer2.baseAddress!, pointCount, clipPlane, Int32(i), refPlane
                            )
                }
                b3Assert(pointCount <= Int32(bigCapacity))

                useBuffer1AsInput = !useBuffer1AsInput
            }

            if pointCount == 0 {
                // Triangle face clipped away. Invalidate cache.
                cache = SATCache()
                return Float.greatestFiniteMagnitude
            }

            pointCount = minInt(pointCount, pointCapacity)

            var minSeparation = Float.greatestFiniteMagnitude

            var finalPointCount: Int32 = 0
            let input = useBuffer1AsOutput ? buffer2 : buffer1
            let pt = manifold.points!

            for i in 0..<Int(pointCount) {
                let clipPoint = input[i]
                minSeparation = minFloat(minSeparation, clipPoint.separation)

                if enableSpeculative == false && clipPoint.separation > 0 {
                    continue
                }

                // Move point onto triangle surface for improved culling
                let point = clipPoint.position

                pt[Int(finalPointCount)].point = point
                pt[Int(finalPointCount)].separation = clipPoint.separation
                pt[Int(finalPointCount)].pair = clipPoint.pair

                finalPointCount += 1
            }

            let speculativeDistance = enableSpeculative ? B3.speculativeDistance : 0
            if minSeparation >= speculativeDistance {
                // This can happen if the objects move apart while re-using a cached axis
                cache = SATCache()
                return minSeparation
            }

            manifold.pointCount = finalPointCount
            manifold.normal = refPlane.normal
            manifold.feature = .featureTriangleFace

            // Save cache
            cache.separation = minSeparation
            cache.type = .faceAxisA
            cache.indexA = UInt8(truncatingIfNeeded: query.indexA)
            cache.indexB = UInt8(truncatingIfNeeded: query.indexB)
            return minSeparation
        }
    }
}

/// Build the edge contact between a triangle edge and a hull edge.
func collideTriangleAndHullEdges(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ trianglePoint: Vec3,
    _ triangleEdge: Vec3,
    _ hull: Hull,
    _ query: SeparatingAxis,
    _ cache: inout SATCache
) {
    b3Validate(query.type == .edgePairAxis)
    b3Validate(0 <= query.indexA && query.indexA < 3)
    b3Validate(0 <= query.indexB && query.indexB < hull.halfEdgeCount)
    b3Validate(query.separation <= 2 * B3.speculativeDistance)

    let pA = trianglePoint
    let eA = triangleEdge

    let edgesB = hull.edges!
    let pointsB = hull.points!
    let edgeB = edgesB + Int(query.indexB)
    let twinB = edgesB + Int(edgeB.pointee.twin)
    let pB = pointsB[Int(edgeB.pointee.origin)]
    let qB = pointsB[Int(twinB.pointee.origin)]
    let eB = qB - pB

    // Get the closest points between the infinite edge lines
    let result = lineDistance(pA, eA, pB, eB)

    // Is one of the closest points outside of the associated edge segment?
    if capacity == 0 || result.fraction1 < 0 || 1 < result.fraction1 || result.fraction2 < 0
        || 1 < result.fraction2
    {
        // Invalid edge pair, no points generated
        b3Assert(manifold.pointCount == 0)
        cache = SATCache()
        return
    }

    let separation = dot(query.normal, pB - pA)
    let point = 0.5 * (result.point1 + result.point2)

    let pt = manifold.points!
    pt[0].point = point
    pt[0].separation = separation
    pt[0].pair = makeFeaturePair(.shapeA, query.indexA, .shapeB, query.indexB)

    // Save cache
    cache.separation = separation
    cache.type = .edgePairAxis
    cache.indexA = UInt8(truncatingIfNeeded: query.indexA)
    cache.indexB = UInt8(truncatingIfNeeded: query.indexB)

    manifold.normal = query.normal
    manifold.pointCount = 1

    manifold.feature = query.indexA == 0 ? .featureEdge1 : (query.indexA == 1 ? .featureEdge2 : .featureEdge3)
}

/// Triangle versus hull. The triangle is in the local space of the hull.
func collideTriangleAndHull(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ v1: Vec3,
    _ v2: Vec3,
    _ v3: Vec3,
    _ triangleFlags: Int32,
    _ hullB: Hull,
    _ cache: inout SATCache,
    _ enableSpeculative: Bool
) {
    manifold.pointCount = 0
    manifold.feature = .featureNone

    if capacity < 4 {
        return
    }

    let trianglePlane = makePlaneFromPoints(v1, v2, v3)
    let linearSlop = B3.linearSlop

    let offset = planeSeparation(trianglePlane, hullB.center)
    if cache.type == .backsideAxis {
        // Use hysteresis to avoid jitter on wavy meshes
        if absFloat(cache.separation - offset) < linearSlop {
            return
        }

        cache.type = .invalidAxis
    }

    if offset < -linearSlop {
        // Cull back side collision. Cache offset to add hysteresis.
        cache.type = .backsideAxis
        cache.separation = offset
        return
    }

    let trianglePoints: InlineArray<3, Vec3> = [v1, v2, v3]
    let triangleEdges: InlineArray<3, Vec3> = [v2 - v1, v3 - v2, v1 - v3]

    var triangle = TriangleData()
    triangle.v1 = v1
    triangle.v2 = v2
    triangle.v3 = v3
    triangle.e1 = triangleEdges[0]
    triangle.e2 = triangleEdges[1]
    triangle.e3 = triangleEdges[2]
    triangle.plane = trianglePlane
    triangle.flags = triangleFlags

    let edges = hullB.edges!
    let hullPlanes = hullB.planes!
    let hullPoints = hullB.points!

    let speculativeDistance = enableSpeculative ? B3.speculativeDistance : 0
    cache.hit = 1

    // Attempt to use the cache to speed up collision
    switch cache.type {
    case .faceAxisA:
        b3Assert(cache.indexA == 0)

        let vertexIndex = findHullSupportVertex(hullB, -trianglePlane.normal)
        let support = hullPoints[Int(vertexIndex)]
        let separation = planeSeparation(trianglePlane, support)
        if separation > speculativeDistance {
            // Cache hit, shapes are separated
            return
        }

        var faceQuery = SeparatingAxis()
        faceQuery.normal = trianglePlane.normal
        faceQuery.separation = separation
        faceQuery.indexA = Int32(cache.indexA)
        faceQuery.indexB = vertexIndex
        faceQuery.type = .faceAxisA

        // Read cache but don't modify it
        var localCache = cache
        let clippedSeparation = collideTriangleFace(
            &manifold, capacity, triangle, hullB, faceQuery, &localCache, enableSpeculative
        )

        if manifold.pointCount > 0 && absFloat(cache.separation - clippedSeparation) < linearSlop {
            // Cache hit, contact points generated
            return
        }

        // Invalidate cache and fall through
        manifold.pointCount = 0
        cache = SATCache()

    case .faceAxisB:
        b3Assert(Int32(cache.indexB) < hullB.faceCount)

        let plane = hullPlanes[Int(cache.indexB)]

        // Get triangle support point
        var vertexIndex: Int32 = 0
        var distance = -dot(v1, plane.normal)
        for i in 1..<3 {
            let d = -dot(trianglePoints[i], plane.normal)
            if d > distance {
                distance = d
                vertexIndex = Int32(i)
            }
        }

        let support = trianglePoints[Int(vertexIndex)]

        // Separation of triangle support point with hull plane
        let separation = planeSeparation(plane, support)
        if separation > speculativeDistance {
            // Cache hit, shapes are separated
            return
        }

        // Deep overlap may lead to an invalid cache
        let isDeep = separation < -2 * linearSlop

        // Don't persist deep cache or allow separation to change too much
        if isDeep == false {
            // Try to rebuild contact from last features
            var faceQuery = SeparatingAxis()
            faceQuery.normal = -plane.normal
            faceQuery.separation = separation
            faceQuery.indexA = vertexIndex
            faceQuery.indexB = Int32(cache.indexB)
            faceQuery.type = .faceAxisB

            // Read cache but don't modify it
            var localCache = cache
            let clippedSeparation = collideHullFace(
                &manifold, capacity, triangle, hullB, faceQuery, &localCache, enableSpeculative
            )

            // Cache reuse is only successful if it creates contact points and the clipped
            // separation didn't change much.
            if manifold.pointCount > 0 && absFloat(cache.separation - clippedSeparation) < linearSlop {
                // Cache hit, contact points generated
                return
            }
        }

        // Invalidate cache and fall through
        manifold.pointCount = 0
        cache = SATCache()

    case .edgePairAxis:
        b3Assert(cache.indexA < 3)
        let indexA = Int32(cache.indexA)

        let triPoint = trianglePoints[Int(indexA)]
        let triEdge = triangleEdges[Int(indexA)]

        b3Assert(Int32(cache.indexB) < hullB.halfEdgeCount - 1)
        let indexB = Int32(cache.indexB)

        let edge2 = edges + Int(indexB)
        let twin2 = edges + Int(indexB) + 1
        b3Assert(edge2.pointee.twin == indexB + 1 && twin2.pointee.twin == indexB)

        let hullPoint = hullPoints[Int(edge2.pointee.origin)]
        let hullEdge = hullPoints[Int(twin2.pointee.origin)] - hullPoint
        let hullNormal1 = hullPlanes[Int(edge2.pointee.face)].normal
        let hullNormal2 = hullPlanes[Int(twin2.pointee.face)].normal

        // Confirm the edge pair is still a Minkowski face.
        // See "Collision Detection of Convex Polyhedra Based on Duality Transformation"
        // Simplified for triangle versus hull.
        let cab = dot(hullNormal1, triEdge)
        let dab = dot(hullNormal2, triEdge)
        let bcd = dot(trianglePlane.normal, hullEdge)

        if cab * dab < 0 && cab * bcd > 0 {
            let squaredTolerance: Float = 0.005 * 0.005

            // Avoid nearly parallel edges that may lead to invalid separation values at the noise
            // floor.
            if maxFloat(cab * cab, dab * dab) >= squaredTolerance * lengthSquared(triEdge) {
                // Similar to hull vs hull (b3QueryEdgeDirections)
                let t = cab / (cab - dab)
                var axis = lerp(hullNormal1, hullNormal2, t)
                b3Validate(lengthSquared(axis) > 1000 * Float.leastNormalMagnitude)
                axis = normalize(axis)
                let separation = dot(axis, triPoint - hullPoint)
                if separation > speculativeDistance {
                    // Cache hit, shapes are separated
                    return
                }

                if absFloat(cache.separation - separation) < linearSlop {
                    // Try to rebuild contact from last features
                    // Flip normal to point from triangle to hull
                    var edgeQuery = SeparatingAxis()
                    edgeQuery.normal = -axis
                    edgeQuery.indexA = indexA
                    edgeQuery.indexB = indexB
                    edgeQuery.separation = separation
                    edgeQuery.type = .edgePairAxis

                    // Read cache but don't modify it
                    var localCache = cache
                    collideTriangleAndHullEdges(
                        &manifold, capacity, triPoint, triEdge, hullB, edgeQuery, &localCache
                    )

                    if manifold.pointCount > 0 {
                        // Cache hit, contact point generated
                        return
                    }
                }
            }
        }

        // Invalidate cache and fall through
        cache = SATCache()

    case .manualFaceAxisA:
        let query = queryTriangleFace(triangle, hullB)
        _ = collideTriangleFace(
            &manifold, capacity, triangle, hullB, query, &cache, enableSpeculative
        )
        return

    case .manualFaceAxisB:
        let query = queryHullFace(triangle, hullB)
        _ = collideHullFace(&manifold, capacity, triangle, hullB, query, &cache, enableSpeculative)
        return

    case .manualEdgePairAxis:
        let query = queryTriangleAndHullEdges(triangle, hullB)
        if query.indexA != B3.nullIndex {
            let trianglePoint = trianglePoints[Int(query.indexA)]
            let triangleEdge = triangleEdges[Int(query.indexA)]
            collideTriangleAndHullEdges(
                &manifold, capacity, trianglePoint, triangleEdge, hullB, query, &cache
            )
        }
        return

    default:
        b3Assert(cache.type == .invalidAxis)
        break
    }

    // Cache miss
    cache.hit = 0

    // Find axis of minimum penetration
    let faceQueryA = queryTriangleFace(triangle, hullB)
    if faceQueryA.separation > speculativeDistance {
        // Separating axis found
        cache.separation = faceQueryA.separation
        cache.type = .faceAxisA
        cache.indexA = UInt8(truncatingIfNeeded: faceQueryA.indexA)
        cache.indexB = UInt8(truncatingIfNeeded: faceQueryA.indexB)
        return
    }

    let faceQueryB = queryHullFace(triangle, hullB)
    if faceQueryB.separation > speculativeDistance {
        // Separating axis found
        cache.separation = faceQueryB.separation
        cache.type = .faceAxisB
        cache.indexA = UInt8(truncatingIfNeeded: faceQueryB.indexA)
        cache.indexB = UInt8(truncatingIfNeeded: faceQueryB.indexB)
        return
    }

    let edgeQuery = queryTriangleAndHullEdges(triangle, hullB)
    if edgeQuery.separation > speculativeDistance {
        // Separating axis found
        cache.separation = edgeQuery.separation
        cache.type = .edgePairAxis
        cache.indexA = UInt8(truncatingIfNeeded: edgeQuery.indexA)
        cache.indexB = UInt8(truncatingIfNeeded: edgeQuery.indexB)
        return
    }

    let clipSeparation: Float

    // Don't admit a hull face significantly opposed to the triangle face.
    // Need a tolerance to avoid ghost collisions.
    let pushingDown = dot(faceQueryB.normal, trianglePlane.normal) < -0.25
    if faceQueryB.separation >= faceQueryA.separation && pushingDown == false {
        clipSeparation = collideHullFace(
            &manifold, capacity, triangle, hullB, faceQueryB, &cache, enableSpeculative
        )
    } else {
        clipSeparation = collideTriangleFace(
            &manifold, capacity, triangle, hullB, faceQueryA, &cache, enableSpeculative
        )
    }

    // Does an edge axis exist?
    if edgeQuery.indexA != B3.nullIndex {
        // When axes are aligned the edge separation can be garbage.
        // If a face axis has positive separation there may be no points.
        let maxFaceSeparation = maxFloat(faceQueryA.separation, faceQueryB.separation)

        if (manifold.pointCount == 0 && edgeQuery.separation > maxFaceSeparation)
            || (manifold.pointCount == 1 && edgeQuery.separation > clipSeparation + linearSlop)
        {
            b3Assert(0 <= edgeQuery.indexA && edgeQuery.indexA < 3)
            let trianglePoint = trianglePoints[Int(edgeQuery.indexA)]
            let triangleEdge = triangleEdges[Int(edgeQuery.indexA)]
            manifold.pointCount = 0
            collideTriangleAndHullEdges(
                &manifold, capacity, trianglePoint, triangleEdge, hullB, edgeQuery, &cache
            )
        }
    }

    // Using the speculative distance means that sometimes there are no valid contact points from
    // SAT. In this fall back to GJK. This is important to prevent tunneling in rare cases.
    if manifold.pointCount == 0 {
        let triangleB: InlineArray<3, Vec3> = [v1, v2, v3]

        var distanceOutput = DistanceOutput()
        withInlineReadBuffer(triangleB) { points in
            var input = DistanceInput()
            input.proxyA = ShapeProxy(points: points.baseAddress, count: 3, radius: 0)
            input.proxyB = ShapeProxy(points: hullPoints, count: hullB.vertexCount, radius: 0)
            input.transform = Transform.identity
            input.useRadii = false

            var simplexCache = SimplexCache()
            distanceOutput = shapeDistance(input, &simplexCache)

            if distanceOutput.distance > 0 {
                b3Assert(0 < simplexCache.count && simplexCache.count <= 3)

                manifold.pointCount = 1
                manifold.feature = getTriangleFeature(simplexCache)
                manifold.normal = distanceOutput.normal
                manifold.points![0].point = distanceOutput.pointB
                manifold.points![0].separation = distanceOutput.distance

                // This feature pair is not accurate but maybe it doesn't matter
                manifold.points![0].pair = featurePairSingle
            }
        }

        // No way to cache this scenario
        cache = SATCache()
    }
}
