// MARK: - Mass and bounds

/// Compute the mass properties of a capsule.
func computeCapsuleMass(_ shape: Capsule, _ density: Float) -> MassData {
    let c1 = shape.center1
    let c2 = shape.center2
    let r = shape.radius

    // Cylinder
    let cylinderHeight = distance(c1, c2)
    let cylinderVolume = B3.pi * r * r * cylinderHeight
    let cylinderMass = cylinderVolume * density

    // Sphere
    let sphereVolume = (4.0 / 3.0) * B3.pi * r * r * r
    let sphereMass = sphereVolume * density

    // Local accumulated inertia
    var inertia = addMM(cylinderInertia(cylinderMass, r, cylinderHeight), sphereInertia(sphereMass, r))

    let steiner_ = 0.125 * sphereMass * (3.0 * r + 2.0 * cylinderHeight) * cylinderHeight
    inertia.cx.x += steiner_
    inertia.cz.z += steiner_

    // Align capsule axis with chosen up-axis
    var rotation = Matrix3.identity
    if cylinderHeight * cylinderHeight > 1000 * Float.leastNormalMagnitude {
        let direction = normalize(c2 - c1)
        let q = computeQuatBetweenUnitVectors(Vec3.axisY, direction)
        rotation = makeMatrixFromQuat(q)
    }

    let mass = sphereMass + cylinderMass
    let center = 0.5 * (c1 + c2)

    var out = MassData()
    out.mass = mass
    out.center = center

    // Rotate the central inertia into the shape frame
    out.inertia = mulMM(rotation, mulMM(inertia, transpose(rotation)))

    return out
}

/// Compute the AABB of a capsule.
func computeCapsuleAABB(_ shape: Capsule, _ transform: Transform) -> AABB {
    let r = shape.radius

    let center1 = transformPoint(transform, shape.center1)
    let center2 = transformPoint(transform, shape.center2)
    let extent = Vec3(r, r, r)

    return AABB(minVec3(center1, center2) - extent, maxVec3(center1, center2) + extent)
}

/// Compute the swept AABB of a capsule.
func computeSweptCapsuleAABB(_ shape: Capsule, _ xf1: Transform, _ xf2: Transform) -> AABB {
    let r = Vec3(shape.radius, shape.radius, shape.radius)
    let a = transformPoint(xf1, shape.center1)
    let b = transformPoint(xf1, shape.center2)
    let c = transformPoint(xf2, shape.center1)
    let d = transformPoint(xf2, shape.center2)

    return AABB(
        minVec3(minVec3(a, b), minVec3(c, d)) - r,
        maxVec3(maxVec3(a, b), maxVec3(c, d)) + r
    )
}

/// Do a capsule and a point cloud proxy overlap?
func overlapCapsule(_ shape: Capsule, _ shapeTransform: Transform, _ proxy: ShapeProxy) -> Bool {
    let points = [shape.center1, shape.center2]

    var input = DistanceInput()
    return points.withUnsafeBufferPointer { points in
        input.proxyA = ShapeProxy(points: points.baseAddress, count: 2, radius: shape.radius)
        input.proxyB = proxy
        input.transform = invMulTransforms(shapeTransform, Transform.identity)
        input.useRadii = true
        var cache = SimplexCache()
        let output = shapeDistance(input, &cache)
        return output.distance < B3.overlapSlop
    }
}

// MARK: - Ray cast

/// Ray cast a capsule.
///
/// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
/// http://www.codercorner.com/blog/?p=321
func rayCastCapsule(_ shape: Capsule, _ input: RayCastInput) -> CastOutput {
    b3Assert(isValidRay(input))

    let c1 = shape.center1
    let c2 = shape.center2
    let r = shape.radius

    // Initialize result structure
    var output = CastOutput()

    let d = c2 - c1

    // Fall back to sphere if the capsule is short
    let tol = 0.01 * B3.linearSlop
    let axisLengthSquared = lengthSquared(d)
    if axisLengthSquared < tol * tol {
        let sphereCenter = 0.5 * (shape.center1 + shape.center2)
        let sphere = Sphere(center: sphereCenter, radius: shape.radius)
        return rayCastSphere(sphere, input)
    }

    // Vector from first center to ray origin.
    let s = input.origin - c1

    // Capsule axis
    let length = axisLengthSquared.squareRoot()
    let axis = (1 / length) * d

    // Project ray origin onto capsule axis.
    let u = dot(s, axis)

    // Closest point on infinite capsule axis, relative to c1.
    let c = u * axis

    // Vector from closest point to ray origin
    let sc = s - c

    // Squared distance from ray origin to capsule axis
    let sc2 = lengthSquared(sc)

    // Is the ray origin within the infinite cylinder along the capsule axis?
    if sc2 < r * r {
        // Clamped barycentric coordinate of ray origin projected onto capsule axis.
        let uClamped = clampFloat(u, 0, length)

        // The closest point on the bounded capsule segment, relative to c1.
        let cp = uClamped * axis

        // Vector from ray origin to closest point on segment.
        let scp = s - cp

        // Squared distance of ray origin from capsule segment.
        let scp2 = lengthSquared(scp)

        // Is the ray origin within the capsule?
        if scp2 < r * r {
            output.hit = true
            output.point = input.origin
            return output
        }

        // The ray can hit an endcap.
        let sphere = Sphere(center: c1 + cp, radius: r)

        return rayCastSphere(sphere, input)
    }

    // Ray axis. A zero length ray reaching here starts outside the capsule, so it misses.
    // Same zero length convention as rayCastSphere.
    let dr = input.translation
    var rayLength: Float = 0
    let rayAxis = getLengthAndNormalize(&rayLength, dr)
    if rayLength == 0 {
        return output
    }

    // Barycentric coordinate of ray end point.
    let v = u + input.maxFraction * dot(dr, axis)

    // Early out: does the projected ray fall outside the capsule?
    if (u < -r && v < -r) || (length + r < u && length + r < v) {
        return output
    }

    // Compute the closest point between the ray segment and the capsule segment.
    // See Real-Time Collision Detection, section 5.1.9
    let a1 = axis
    let a2 = rayAxis
    let a12 = dot(a1, a2)

    // Ray distance to the near intersection with the infinite cylinder. Length units.
    let tr: Float

    let det = 1 - a12 * a12
    if det < Float.ulpOfOne {
        // Solve the 2D problem of ray versus circle starting at the ray origin, where the circle is
        // the axial view of the infinite capsule cylinder.

        // Instead of a cross product, subtract the parallel part to get a perpendicular vector.
        let perp = mulSub(a2, a12, a1)
        let perp2 = lengthSquared(perp)

        // Project to origin to infinite capsule axis vector onto the perpendicular vector.
        let beta = dot(sc, perp)

        // Setup quadratic root finder.
        let gamma = sc2 - r * r

        // Discriminant
        let disc = beta * beta - perp2 * gamma

        // Casting away from the axis, or the perpendicular gap never closes to the radius.
        if beta >= 0 || disc < 0 {
            return output
        }

        // Quadratic near root. Expressed in an alternate form to avoid the (-beta - sqrt)
        // cancellation as the ray nears parallel.
        tr = gamma / (-beta + disc.squareRoot())
    } else {
        // Ray and capsule axes are not parallel.

        // Closest points between the infinite ray and the infinite capsule axis.
        let invDet = 1 / det
        let sa1 = u
        let sa2 = dot(s, a2)

        let t1 = (sa1 - a12 * sa2) * invDet
        let t2 = (a12 * sa1 - sa2) * invDet

        // Closest points
        let p1 = t1 * a1
        let p2 = mulAdd(s, t2, a2)

        // Vector from closest point on infinite capsule to infinite ray.
        let g = p2 - p1

        let g2 = lengthSquared(g)
        if g2 > r * r {
            // Early out: closest point on infinite ray is outside infinite cylinder.
            return output
        }

        // Intersect the infinite ray with the infinite cylinder.
        let h = ((r * r - g2) * invDet).squareRoot()

        tr = t2 - h
    }

    // Outside ray?
    if tr < 0 || input.maxFraction * rayLength < tr {
        return output
    }

    // The corresponding distance on the capsule axis. Length units.
    let tc = u + tr * a12

    // Outside c1 end?
    if tc < 0 {
        // Ray cast sphere 1.
        let sphere = Sphere(center: c1, radius: r)
        return rayCastSphere(sphere, input)
    }

    // Outside c2 end?
    if length < tc {
        // Ray cast sphere 2.
        let sphere = Sphere(center: c2, radius: r)
        return rayCastSphere(sphere, input)
    }

    // Hit point on capsule side, relative to c1.
    let p = mulAdd(s, tr, rayAxis)

    // Hit normal.
    var normal = mulSub(p, tc, axis)
    normal = normalize(normal)

    output.point = c1 + p
    output.normal = normal
    output.fraction = clampFloat(tr / rayLength, 0, input.maxFraction)
    output.hit = true
    return output
}

/// Shape cast a capsule against a point cloud proxy.
func shapeCastCapsule(_ capsule: Capsule, _ input: ShapeCastInput) -> CastOutput {
    let points = [capsule.center1, capsule.center2]

    var pairInput = ShapeCastPairInput()
    return points.withUnsafeBufferPointer { points in
        pairInput.proxyA = ShapeProxy(points: points.baseAddress, count: 2, radius: capsule.radius)
        pairInput.proxyB = input.proxy
        pairInput.transform = Transform.identity
        pairInput.translationB = input.translation
        pairInput.maxFraction = input.maxFraction
        pairInput.canEncroach = input.canEncroach
        return shapeCast(pairInput)
    }
}

/// Collide a mover capsule with a capsule shape.
func collideMoverAndCapsule(_ result: inout PlaneResult, _ shape: Capsule, _ mover: Capsule) -> Int32 {
    let totalRadius = mover.radius + shape.radius

    let approach = segmentDistance(shape.center1, shape.center2, mover.center1, mover.center2)

    // The normal points from the shape toward the mover.
    var distance: Float = 0
    var normal = getLengthAndNormalize(&distance, approach.point2 - approach.point1)

    if distance > totalRadius {
        return 0
    }

    let linearSlop = B3.linearSlop
    if distance < linearSlop {
        // Deep overlap: the core segments intersect. Pick an arbitrary direction perpendicular
        // to the capsule axis.
        var moverLength: Float = 0
        let moverAxis = getLengthAndNormalize(&moverLength, mover.center2 - mover.center1)
        normal = moverLength > linearSlop ? perp(moverAxis) : Vec3.axisY
        distance = 0
    }

    let plane = Plane(normal, totalRadius - distance)
    result.plane = plane
    result.point = approach.point1
    result.triangleIndex = 0
    result.childIndex = 0
    result.materialIndex = 0
    return 1
}
