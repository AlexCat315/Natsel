// Dirk Gregorius contributed portions of this code.

// MARK: - Mass and bounds

/// Compute the mass properties of a sphere.
func computeSphereMass(_ shape: Sphere, _ density: Float) -> MassData {
    let center = shape.center
    let radius = shape.radius

    let volume = 4.0 / 3.0 * B3.pi * radius * radius * radius
    let mass = volume * density
    let ixx = 0.4 * mass * radius * radius

    var out = MassData()
    out.mass = mass
    out.center = center

    // Inertia about the center of mass
    out.inertia = makeDiagonalMatrix(ixx, ixx, ixx)
    return out
}

/// Compute the AABB of a sphere.
func computeSphereAABB(_ shape: Sphere, _ transform: Transform) -> AABB {
    let center = transformPoint(transform, shape.center)
    let radius = shape.radius
    let extent = Vec3(radius, radius, radius)
    return AABB(center - extent, center + extent)
}

/// Compute the swept AABB of a sphere.
func computeSweptSphereAABB(_ shape: Sphere, _ xf1: Transform, _ xf2: Transform) -> AABB {
    let r = Vec3(shape.radius, shape.radius, shape.radius)
    let center1 = transformPoint(xf1, shape.center)
    let center2 = transformPoint(xf2, shape.center)
    return AABB(minVec3(center1, center2) - r, maxVec3(center1, center2) + r)
}

/// Do a sphere and a point cloud proxy overlap?
func overlapSphere(_ shape: Sphere, _ shapeTransform: Transform, _ proxy: ShapeProxy) -> Bool {
    var input = DistanceInput()
    return withUnsafePointer(to: shape.center) { center in
        input.proxyA = ShapeProxy(points: center, count: 1, radius: shape.radius)
        input.proxyB = proxy
        input.transform = invMulTransforms(shapeTransform, Transform.identity)
        input.useRadii = true
        var cache = SimplexCache()
        let output = shapeDistance(input, &cache)
        return output.distance < B3.overlapSlop
    }
}

// MARK: - Ray casts

/// Ray cast a sphere.
///
/// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
/// http://www.codercorner.com/blog/?p=321
func rayCastSphere(_ shape: Sphere, _ input: RayCastInput) -> CastOutput {
    b3Assert(isValidRay(input))
    var output = CastOutput()

    let p = shape.center

    // Shift ray so sphere center is the origin
    let s = input.origin - p

    let r = shape.radius
    let rr = r * r

    var length: Float = 0
    let d = getLengthAndNormalize(&length, input.translation)
    if length == 0 {
        // zero length ray

        if lengthSquared(s) < rr {
            // initial overlap
            output.point = input.origin
            output.hit = true
        }

        return output
    }

    // Find closest point on ray to origin

    // solve: dot(s + t * d, d) = 0
    let t = -dot(s, d)

    // c is the closest point on the line to the origin
    let c = mulAdd(s, t, d)

    let cc = dot(c, c)

    if cc > rr {
        // closest point is outside the sphere
        return output
    }

    // Pythagoras
    let h = (rr - cc).squareRoot()

    let fraction = t - h

    if fraction < 0 || input.maxFraction * length < fraction {
        // intersection is point outside the range of the ray segment

        if lengthSquared(s) < rr {
            // initial overlap
            output.point = input.origin
            output.hit = true
        }

        return output
    }

    let hitPoint = mulAdd(s, fraction, d)

    output.fraction = fraction / length

    if output.fraction > input.maxFraction {
        b3Log("sphere input fraction = \(input.maxFraction), output fraction = \(output.fraction)")
        output.fraction = input.maxFraction
    }

    output.normal = normalize(hitPoint)
    output.point = mulAdd(p, shape.radius, output.normal)
    output.hit = true

    return output
}

/// Ray cast a hollow sphere. This will do interior hits.
///
/// Precision Improvements for Ray / Sphere Intersection - Ray Tracing Gems 2019
/// http://www.codercorner.com/blog/?p=321
func rayCastHollowSphere(_ sphere: Sphere, _ input: RayCastInput) -> CastOutput {
    let p = sphere.center

    var output = CastOutput()

    // Shift ray so sphere center is the origin
    let s = input.origin - p
    let d = normalize(input.translation)

    // Find closest point on ray to origin

    // solve: dot(s + t * d, d) = 0
    let t = -dot(s, d)

    // c is the closest point on the line to the origin
    let c = mulAdd(s, t, d)

    let cc = dot(c, c)
    let r = sphere.radius
    let rr = r * r

    if cc > rr {
        // closest point is outside the sphere
        return output
    }

    // Pythagoras
    let h = (rr - cc).squareRoot()

    var fraction = t - h

    if fraction < 0 {
        fraction = t + h
    }

    if fraction < 0 {
        // behind the ray
        return output
    }

    if fraction > input.maxFraction {
        return output
    }

    let hitPoint = mulAdd(s, fraction, d)

    output.fraction = fraction
    output.normal = normalize(hitPoint)
    output.point = mulAdd(p, sphere.radius, output.normal)
    output.hit = true

    return output
}

/// Shape cast a sphere against a point cloud proxy.
func shapeCastSphere(_ sphere: Sphere, _ input: ShapeCastInput) -> CastOutput {
    var pairInput = ShapeCastPairInput()
    return withUnsafePointer(to: sphere.center) { center in
        pairInput.proxyA = ShapeProxy(points: center, count: 1, radius: sphere.radius)
        pairInput.proxyB = input.proxy
        pairInput.transform = Transform.identity
        pairInput.translationB = input.translation
        pairInput.maxFraction = input.maxFraction
        pairInput.canEncroach = input.canEncroach
        return shapeCast(pairInput)
    }
}

/// Collide a mover capsule with a sphere.
func collideMoverAndSphere(_ result: inout PlaneResult, _ shape: Sphere, _ mover: Capsule) -> Int32 {
    let totalRadius = mover.radius + shape.radius
    let closest = pointToSegmentDistance(mover.center1, mover.center2, shape.center)

    // The normal points from the sphere toward the mover.
    var distance: Float = 0
    var normal = getLengthAndNormalize(&distance, closest - shape.center)

    if distance > totalRadius {
        return 0
    }

    let linearSlop = B3.linearSlop
    if distance < linearSlop {
        // Deep overlap: the mover axis passes through the sphere center, so no
        // direction is preferred. Push perpendicular to the mover axis.
        var length: Float = 0
        let axis = getLengthAndNormalize(&length, mover.center2 - mover.center1)
        normal = length > linearSlop ? perp(axis) : Vec3.axisY
        distance = 0
    }

    let plane = Plane(normal, totalRadius - distance)
    result.plane = plane
    result.point = shape.center
    result.triangleIndex = 0
    result.childIndex = 0
    result.materialIndex = 0
    return 1
}
