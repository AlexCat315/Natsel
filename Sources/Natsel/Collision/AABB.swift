// MARK: - AABB ray cast

/// Ray cast an AABB. This is a custom function used by height fields.
///
/// Similar to Real-time Collision Detection, p179.
func rayCastAABB(_ a: AABB, _ p1: Vec3, _ p2: Vec3, _ minFraction: inout Float, _ maxFraction: inout Float) -> Bool {
    // Ray direction and length
    let d = p2 - p1
    let rayLength = length(d)

    // Handle degenerate ray
    if rayLength < Float.ulpOfOne {
        // Check if point is inside AABB
        if p1.x >= a.lowerBound.x && p1.x <= a.upperBound.x && p1.y >= a.lowerBound.y && p1.y <= a.upperBound.y
            && p1.z >= a.lowerBound.z && p1.z <= a.upperBound.z
        {
            minFraction = 0
            maxFraction = 0
            return true
        }

        return false
    }

    let rayDir = (1 / rayLength) * d

    // Slab method for ray-AABB intersection
    var tMin: Float = 0
    var tMax = rayLength

    for axis in 0..<3 {
        let rayComponent = getByIndex(rayDir, axis)
        let rayStart = getByIndex(p1, axis)
        let boxMin = getByIndex(a.lowerBound, axis)
        let boxMax = getByIndex(a.upperBound, axis)

        if absFloat(rayComponent) < Float.ulpOfOne {
            // Ray is parallel to slab, check if ray origin is within slab
            if rayStart < boxMin || rayStart > boxMax {
                return false
            }
        } else {
            // Compute intersection distances
            var t1 = (boxMin - rayStart) / rayComponent
            var t2 = (boxMax - rayStart) / rayComponent

            // Ensure t1 <= t2
            if t1 > t2 {
                let temp = t1
                t1 = t2
                t2 = temp
            }

            // Update intersection interval
            tMin = maxFloat(tMin, t1)
            tMax = minFloat(tMax, t2)

            // Check for no intersection
            if tMin > tMax {
                return false
            }
        }
    }

    // Check if intersection is behind ray start
    if tMax < 0 {
        return false
    }

    // Convert distances to fractions
    minFraction = clampFloat(tMin / rayLength, 0, 1)
    maxFraction = clampFloat(tMax / rayLength, 0, 1)

    return true
}

func boundsOverlap(_ min1: Vec3, _ max1: Vec3, _ min2: Vec3, _ max2: Vec3) -> Bool {
    let separation = maxVec3(min2 - max1, min1 - max2)
    return separation.x <= 0 && separation.y <= 0 && separation.z <= 0
}

/// Scalar triangle/AABB separating-axis tests.
func boundsTriangleOverlap(_ center: Vec3, _ extent: Vec3, _ a: Vec3, _ b: Vec3, _ c: Vec3) -> Bool {
    let v1 = a - center
    let v2 = b - center
    let v3 = c - center
    let triangleMin = minVec3(v1, minVec3(v2, v3))
    let triangleMax = maxVec3(v1, maxVec3(v2, v3))
    let face = maxVec3(triangleMin - extent, -(triangleMax + extent))
    if face.x > 0 || face.y > 0 || face.z > 0 { return false }
    let e1 = v2 - v1
    let e2 = v3 - v2
    let e3 = v1 - v3
    let normal = cross(e1, e2)
    let triangleSeparation = abs(dot(normal, v1)) - dot(absVec3(normal), extent)
    if triangleSeparation > 0 { return false }
    let separation1 = absVec3(cross(e1, v1 + v3)) - absVec3(cross(e1, e3))
        - 2 * modifiedCross(absVec3(e1), extent)
    if separation1.x > 0 || separation1.y > 0 || separation1.z > 0 { return false }
    let separation2 = absVec3(cross(e2, v1 + v2)) - absVec3(cross(e2, e1))
        - 2 * modifiedCross(absVec3(e2), extent)
    if separation2.x > 0 || separation2.y > 0 || separation2.z > 0 { return false }
    let separation3 = absVec3(cross(e3, v2 + v3)) - absVec3(cross(e3, e2))
        - 2 * modifiedCross(absVec3(e3), extent)
    if separation3.x > 0 || separation3.y > 0 || separation3.z > 0 { return false }
    return true
}

