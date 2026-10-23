// Dirk Gregorius contributed portions of this code.

let maxSimplexVertices: Int32 = 4
let maxGJKIterations: Int32 = 32

// MARK: - Support

/// Get the support vertex of a proxy along an axis. The first vertex is moved to the origin for
/// improved precision, so `axis` is expected in the proxy's frame.
func getProxySupport(_ proxy: ShapeProxy, _ axis: Vec3) -> Int32 {
    let count = proxy.count
    let points = proxy.points

    b3Assert(count > 0)
    b3Assert(points != nil)

    // We move the first vertex into the origin for improved precision.
    // This is necessary since we don't have shape transforms and
    // vertices can potentially be far away from the origin (large).
    let origin = points![0]
    var maxIndex: Int32 = 0
    var maxProjection: Float = 0

    var index: Int32 = 1
    while index < count {
        // We subtract the first vertex since we are shifting into the origin.
        let projection = dot(axis, points![Int(index)] - origin)
        if projection > maxProjection {
            maxIndex = index
            maxProjection = projection
        }
        index += 1
    }

    return maxIndex
}

/// Get the support vertex of a point cloud along an axis, with the first point shifted to the origin.
func getPointSupport(_ points: UnsafePointer<Vec3>, _ count: Int32, _ axis: Vec3) -> Int32 {
    b3Assert(count > 0)

    let origin = points[0]
    var maxIndex: Int32 = 0
    var maxProjection: Float = 0

    var index: Int32 = 1
    while index < count {
        let projection = dot(axis, points[Int(index)] - origin)
        if projection > maxProjection {
            maxIndex = index
            maxProjection = projection
        }
        index += 1
    }

    return maxIndex
}

// MARK: - Barycentric coordinates

/// Barycentric coordinates of the closest point on the segment `a-b` to the origin. The last
/// element is the divisor.
func barycentricCoordsEdge(_ a: Vec3, _ b: Vec3) -> (Float, Float, Float) {
    let ab = b - a

    let divisor = dot(ab, ab)

    let u = dot(b, ab)
    let v = -dot(a, ab)
    return (u, v, divisor)
}

/// Barycentric coordinates of the closest point on the triangle `a-b-c` to the origin.
func barycentricCoordsTri(_ a: Vec3, _ b: Vec3, _ c: Vec3) -> (Float, Float, Float, Float) {
    let ab = b - a
    let ac = c - a

    let bXC = cross(b, c)
    let cXA = cross(c, a)
    let aXB = cross(a, b)

    let abXAc = cross(ab, ac)

    let divisor = dot(abXAc, abXAc)

    let u = dot(bXC, abXAc)
    let v = dot(cXA, abXAc)
    let w = dot(aXB, abXAc)
    return (u, v, w, divisor)
}

/// Barycentric coordinates of the closest point on the tetrahedron `a-b-c-d` to the origin.
func barycentricCoordsTet(_ a: Vec3, _ b: Vec3, _ c: Vec3, _ d: Vec3) -> (Float, Float, Float, Float, Float) {
    let ab = b - a
    let ac = c - a
    let ad = d - a

    // Last element is divisor (forced to be positive)
    let divisor = scalarTripleProduct(ab, ac, ad)

    let sign: Float = divisor < 0 ? -1 : 1
    let u = sign * scalarTripleProduct(b, c, d)
    let v = sign * scalarTripleProduct(a, d, c)
    let w = sign * scalarTripleProduct(a, b, d)
    let x = sign * scalarTripleProduct(a, c, b)
    return (u, v, w, x, sign * divisor)
}

// MARK: - Simplex helpers

/// The metric of a simplex: length, area, or volume.
func getMetric(_ simplex: Simplex) -> Float {
    let count = simplex.count
    b3Assert(1 <= count && count <= 4)

    let vertices = simplex.vertices

    switch count {
    case 1:
        return 0

    case 2:
        let a = vertices[0].w
        let b = vertices[1].w
        return distance(a, b)

    case 3:
        let a = vertices[0].w
        let b = vertices[1].w
        let c = vertices[2].w
        return length(cross(b - a, c - a)) / 2

    case 4:
        let a = vertices[0].w
        let b = vertices[1].w
        let c = vertices[2].w
        let d = vertices[3].w
        return scalarTripleProduct(b - a, c - a, d - a) / 6

    default:
        b3Assert(false)
        return 0
    }
}

/// Store the simplex in the cache for warm starting.
func writeCache(_ cache: inout SimplexCache, _ simplex: Simplex) {
    let count = simplex.count
    cache.metric = getMetric(simplex)
    cache.count = UInt16(count)
    for index in 0..<Int(count) {
        cache.indexA[index] = UInt8(truncatingIfNeeded: simplex.vertices[index].indexA)
        cache.indexB[index] = UInt8(truncatingIfNeeded: simplex.vertices[index].indexB)
    }
}

/// Solve the closest point on a 2-simplex. Returns false if there was no progress.
func solveSimplex2(_ simplex: inout Simplex) -> Bool {
    b3Assert(simplex.count == 2)

    let a = simplex.vertices[0].w
    let b = simplex.vertices[1].w
    let ab = b - a

    // Last element is divisor
    let divisor = dot(ab, ab)

    let u = dot(b, ab)
    let v = -dot(a, ab)

    // V( A )
    if v <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0].a = 1

        return true
    }

    // V( B )
    if u <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0] = simplex.vertices[1]
        simplex.vertices[0].a = 1

        return true
    }

    // Edge region
    if divisor <= 0 {
        return false
    }

    // VR( AB )
    let denominator = 1 / divisor
    simplex.vertices[0].a = denominator * u
    simplex.vertices[1].a = denominator * v

    return true
}

/// Solve the closest point on a 3-simplex. Returns false if there was no progress.
func solveSimplex3(_ simplex: inout Simplex) -> Bool {
    b3Assert(simplex.count == 3)

    // Get simplex (be aware of aliasing here!)
    let v1 = simplex.vertices[0]
    let v2 = simplex.vertices[1]
    let v3 = simplex.vertices[2]

    // Vertex regions
    let wAB = barycentricCoordsEdge(v1.w, v2.w)
    let wBC = barycentricCoordsEdge(v2.w, v3.w)
    let wCA = barycentricCoordsEdge(v3.w, v1.w)

    // VR( A )
    if wAB.1 <= 0 && wCA.0 <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0] = v1
        simplex.vertices[0].a = 1

        return true
    }

    // VR( B )
    if wBC.1 <= 0 && wAB.0 <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0] = v2
        simplex.vertices[0].a = 1

        return true
    }

    // VR( C )
    if wCA.1 <= 0 && wBC.0 <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0] = v3
        simplex.vertices[0].a = 1

        return true
    }

    // Edge regions
    let wABC = barycentricCoordsTri(v1.w, v2.w, v3.w)

    // VR( AB )
    if wABC.2 <= 0 && wAB.0 > 0 && wAB.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = v1
        simplex.vertices[1] = v2

        // Normalize
        let divisor = wAB.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wAB.0 / divisor
        simplex.vertices[1].a = wAB.1 / divisor

        return true
    }

    // VR( BC )
    if wABC.0 <= 0 && wBC.0 > 0 && wBC.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = v2
        simplex.vertices[1] = v3

        // Normalize
        let divisor = wBC.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wBC.0 / divisor
        simplex.vertices[1].a = wBC.1 / divisor

        return true
    }

    // VR( CA )
    if wABC.1 <= 0 && wCA.0 > 0 && wCA.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = v3
        simplex.vertices[1] = v1

        // Normalize
        let divisor = wCA.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wCA.0 / divisor
        simplex.vertices[1].a = wCA.1 / divisor

        return true
    }

    // Face region
    let divisor = wABC.3
    if divisor <= 0 {
        return false
    }

    // VR( ABC )
    simplex.vertices[0].a = wABC.0 / divisor
    simplex.vertices[1].a = wABC.1 / divisor
    simplex.vertices[2].a = wABC.2 / divisor

    return true
}

/// Solve the closest point on a 4-simplex. Returns false if there was no progress.
func solveSimplex4(_ simplex: inout Simplex) -> Bool {
    b3Assert(simplex.count == 4)

    // Get simplex (be aware of aliasing here!)
    let vertexA = simplex.vertices[0]
    let vertexB = simplex.vertices[1]
    let vertexC = simplex.vertices[2]
    let vertexD = simplex.vertices[3]

    // Vertex region
    let wAB = barycentricCoordsEdge(vertexA.w, vertexB.w)
    let wAC = barycentricCoordsEdge(vertexA.w, vertexC.w)
    let wAD = barycentricCoordsEdge(vertexA.w, vertexD.w)
    let wBC = barycentricCoordsEdge(vertexB.w, vertexC.w)
    let wCD = barycentricCoordsEdge(vertexC.w, vertexD.w)
    let wDB = barycentricCoordsEdge(vertexD.w, vertexB.w)

    // VR( A )
    if wAB.1 <= 0 && wAC.1 <= 0 && wAD.1 <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0] = vertexA
        simplex.vertices[0].a = 1

        return true
    }

    // VR( B )
    if wAB.0 <= 0 && wDB.0 <= 0 && wBC.1 <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0] = vertexB
        simplex.vertices[0].a = 1

        return true
    }

    // VR( C )
    if wAC.0 <= 0 && wBC.0 <= 0 && wCD.1 <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0] = vertexC
        simplex.vertices[0].a = 1

        return true
    }

    // VR( D )
    if wAD.0 <= 0 && wCD.0 <= 0 && wDB.1 <= 0 {
        // Reduce simplex
        simplex.count = 1
        simplex.vertices[0] = vertexD
        simplex.vertices[0].a = 1

        return true
    }

    // Edge region
    let wACB = barycentricCoordsTri(vertexA.w, vertexC.w, vertexB.w)
    let wABD = barycentricCoordsTri(vertexA.w, vertexB.w, vertexD.w)
    let wADC = barycentricCoordsTri(vertexA.w, vertexD.w, vertexC.w)
    let wBCD = barycentricCoordsTri(vertexB.w, vertexC.w, vertexD.w)

    // VR( AB )
    if wABD.2 <= 0 && wACB.1 <= 0 && wAB.0 > 0 && wAB.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = vertexA
        simplex.vertices[1] = vertexB

        // Normalize
        let divisor = wAB.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wAB.0 / divisor
        simplex.vertices[1].a = wAB.1 / divisor

        return true
    }

    // VR( AC )
    if wACB.2 <= 0 && wADC.1 <= 0 && wAC.0 > 0 && wAC.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = vertexA
        simplex.vertices[1] = vertexC

        // Normalize
        let divisor = wAC.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wAC.0 / divisor
        simplex.vertices[1].a = wAC.1 / divisor

        return true
    }

    // VR( AD )
    if wADC.2 <= 0 && wABD.1 <= 0 && wAD.0 > 0 && wAD.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = vertexA
        simplex.vertices[1] = vertexD

        // Normalize
        let divisor = wAD.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wAD.0 / divisor
        simplex.vertices[1].a = wAD.1 / divisor

        return true
    }

    // VR( BC )
    if wACB.0 <= 0 && wBCD.2 <= 0 && wBC.0 > 0 && wBC.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = vertexB
        simplex.vertices[1] = vertexC

        // Normalize
        let divisor = wBC.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wBC.0 / divisor
        simplex.vertices[1].a = wBC.1 / divisor

        return true
    }

    // VR( CD )
    if wADC.0 <= 0 && wBCD.0 <= 0 && wCD.0 > 0 && wCD.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = vertexC
        simplex.vertices[1] = vertexD

        // Normalize
        let divisor = wCD.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wCD.0 / divisor
        simplex.vertices[1].a = wCD.1 / divisor

        return true
    }

    // VR( DB )
    if wABD.0 <= 0 && wBCD.1 <= 0 && wDB.0 > 0 && wDB.1 > 0 {
        // Reduce simplex
        simplex.count = 2
        simplex.vertices[0] = vertexD
        simplex.vertices[1] = vertexB

        // Normalize
        let divisor = wDB.2
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wDB.0 / divisor
        simplex.vertices[1].a = wDB.1 / divisor

        return true
    }

    // Face regions
    let wABCD = barycentricCoordsTet(vertexA.w, vertexB.w, vertexC.w, vertexD.w)

    // VR( ACB )
    if wABCD.3 < 0 && wACB.0 > 0 && wACB.1 > 0 && wACB.2 > 0 {
        // Reduce simplex
        simplex.count = 3
        simplex.vertices[0] = vertexA
        simplex.vertices[1] = vertexC
        simplex.vertices[2] = vertexB

        // Normalize
        let divisor = wACB.3
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wACB.0 / divisor
        simplex.vertices[1].a = wACB.1 / divisor
        simplex.vertices[2].a = wACB.2 / divisor

        return true
    }

    // VR( ABD )
    if wABCD.2 < 0 && wABD.0 > 0 && wABD.1 > 0 && wABD.2 > 0 {
        // Reduce simplex
        simplex.count = 3
        simplex.vertices[0] = vertexA
        simplex.vertices[1] = vertexB
        simplex.vertices[2] = vertexD

        // Normalize
        let divisor = wABD.3
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wABD.0 / divisor
        simplex.vertices[1].a = wABD.1 / divisor
        simplex.vertices[2].a = wABD.2 / divisor

        return true
    }

    // VR( ADC )
    if wABCD.1 < 0 && wADC.0 > 0 && wADC.1 > 0 && wADC.2 > 0 {
        // Reduce simplex
        simplex.count = 3
        simplex.vertices[0] = vertexA
        simplex.vertices[1] = vertexD
        simplex.vertices[2] = vertexC

        // Normalize
        let divisor = wADC.3
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wADC.0 / divisor
        simplex.vertices[1].a = wADC.1 / divisor
        simplex.vertices[2].a = wADC.2 / divisor

        return true
    }

    // VR( BCD )
    if wABCD.0 < 0 && wBCD.0 > 0 && wBCD.1 > 0 && wBCD.2 > 0 {
        // Reduce simplex
        simplex.count = 3
        simplex.vertices[0] = vertexB
        simplex.vertices[1] = vertexC
        simplex.vertices[2] = vertexD

        // Normalize
        let divisor = wBCD.3
        if divisor <= 0 {
            return false
        }

        simplex.vertices[0].a = wBCD.0 / divisor
        simplex.vertices[1].a = wBCD.1 / divisor
        simplex.vertices[2].a = wBCD.2 / divisor

        return true
    }

    // *** Inside tetrahedron ***
    let divisor = wABCD.4
    if divisor <= 0 {
        return false
    }

    // VR( ABCD )
    simplex.vertices[0].a = wABCD.0 / divisor
    simplex.vertices[1].a = wABCD.1 / divisor
    simplex.vertices[2].a = wABCD.2 / divisor
    simplex.vertices[3].a = wABCD.3 / divisor

    return true
}

/// The witness points on each shape, in frame A.
func computeWitnessPoints(_ simplex: Simplex, _ vertexA: inout Vec3, _ vertexB: inout Vec3) {
    let vs = simplex.vertices
    let count = simplex.count
    b3Assert(1 <= count && count <= 4)

    switch count {
    case 1:
        vertexA = vs[0].wA
        vertexB = vs[0].wB

    case 2:
        vertexA = blend2(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA)
        vertexB = blend2(vs[0].a, vs[0].wB, vs[1].a, vs[1].wB)

    case 3:
        vertexA = blend3(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA, vs[2].a, vs[2].wA)
        vertexB = blend3(vs[0].a, vs[0].wB, vs[1].a, vs[1].wB, vs[2].a, vs[2].wB)

    case 4:
        // Force identical points and *zero* distance
        let sum = blend2(vs[0].a, vs[0].wA, vs[1].a, vs[1].wA)
            + blend2(vs[2].a, vs[2].wA, vs[3].a, vs[3].wA)
        vertexA = sum
        vertexB = sum

    default:
        b3Assert(false)
    }
}

/// The closest point on the current simplex.
func closestPointOnSimplex(_ simplex: Simplex) -> Vec3 {
    let vs = simplex.vertices

    switch simplex.count {
    case 1:
        return vs[0].w

    case 2:
        return blend2(vs[0].a, vs[0].w, vs[1].a, vs[1].w)

    case 3:
        return blend3(vs[0].a, vs[0].w, vs[1].a, vs[1].w, vs[2].a, vs[2].w)

    case 4:
        return blend2(vs[0].a, vs[0].w, vs[1].a, vs[1].w) + blend2(vs[2].a, vs[2].w, vs[3].a, vs[3].w)

    default:
        b3Assert(false)
        return Vec3()
    }
}

// MARK: - GJK distance

/// Compute the closest points between two convex shapes represented as point clouds. The query
/// runs in frame A and returns witness points and the normal in frame A.
func shapeDistance(
    _ input: DistanceInput,
    _ cache: inout SimplexCache,
    _ simplexes: UnsafeMutablePointer<Simplex>? = nil,
    _ simplexCapacity: Int32 = 0
) -> DistanceOutput {
    // The query runs in frame A using the relative pose of B in A.
    let xf = input.transform

    // Use matrices for faster math
    let m = makeMatrixFromQuat(xf.q)
    let mt = transpose(m)

    let proxyA = input.proxyA
    let proxyB = input.proxyB

    // Compute initial simplex from cache
    b3Assert(cache.count <= maxSimplexVertices)

    var simplex = Simplex()

    simplex.count = Int32(cache.count)
    for i in 0..<Int(cache.count) {
        let index1 = Int32(cache.indexA[i])
        let index2 = Int32(cache.indexB[i])

        b3Assert(0 <= index1 && index1 < proxyA.count)
        b3Assert(0 <= index2 && index2 < proxyB.count)

        let vertex1 = proxyA.points![Int(index1)]
        let vertex2 = mulMV(m, proxyB.points![Int(index2)]) + xf.p

        simplex.vertices[i].indexA = index1
        simplex.vertices[i].indexB = index2
        simplex.vertices[i].wA = vertex1
        simplex.vertices[i].wB = vertex2
        simplex.vertices[i].w = vertex2 - vertex1
        simplex.vertices[i].a = 0
    }

    // Compute the new simplex metric, if it is substantially
    // different than the old metric flush the simplex.
    if simplex.count > 0 {
        let metric1 = cache.metric
        let metric2 = getMetric(simplex)

        // todo the tetrahedron metric can be negative
        if 2 * metric1 < metric2 || metric2 < 0.5 * metric1 || metric2 < Float.ulpOfOne {
            // Flush the simplex
            simplex.count = 0
        }
    }

    // If the cache is invalid or empty
    if simplex.count == 0 {
        let vertex1 = proxyA.points![0]
        let vertex2 = mulMV(m, proxyB.points![0]) + xf.p

        simplex.count = 1
        simplex.vertices[0].indexA = 0
        simplex.vertices[0].indexB = 0
        simplex.vertices[0].wA = vertex1
        simplex.vertices[0].wB = vertex2
        simplex.vertices[0].w = vertex2 - vertex1
        simplex.vertices[0].a = 0
    }

    var backup = Simplex()

    var simplexIndex = 0
    if simplexes != nil && simplexIndex < Int(simplexCapacity) {
        simplexes![simplexIndex] = simplex
        simplexIndex += 1
    }

    var distanceOutput = DistanceOutput()

    // Keep track of squared distance
    var distanceSq = Float.greatestFiniteMagnitude

    var normal = Vec3()

    // Run GJK
    var iteration: Int32 = 0
    while iteration < maxGJKIterations {
        // Solve simplex
        var solved = false
        switch simplex.count {
        case 1:
            simplex.vertices[0].a = 1
            solved = true

        case 2:
            solved = solveSimplex2(&simplex)

        case 3:
            solved = solveSimplex3(&simplex)

        case 4:
            solved = solveSimplex4(&simplex)

        default:
            b3Assert(false)
        }

        if solved == false {
            // No progress - reconstruct last simplex
            b3Assert(backup.count != 0)
            simplex = backup
            break
        }

        if simplexes != nil && simplexIndex < Int(simplexCapacity) {
            simplexes![simplexIndex] = simplex
            simplexIndex += 1
            distanceOutput.iterations = iteration
            distanceOutput.simplexCount = Int32(simplexIndex)
        }

        if simplex.count == maxSimplexVertices {
            // Overlap
            var localPointA = Vec3()
            var localPointB = Vec3()
            computeWitnessPoints(simplex, &localPointA, &localPointB)
            distanceOutput.pointA = localPointA
            distanceOutput.pointB = localPointB
            distanceOutput.normal = Vec3()
            distanceOutput.distance = 0
            distanceOutput.iterations = iteration
            distanceOutput.simplexCount = Int32(simplexIndex)
            return distanceOutput
        }

        // Assure distance progression
        let oldDistanceSq = distanceSq

        // Compute closest point
        let closestPoint = closestPointOnSimplex(simplex)

        distanceSq = dot(closestPoint, closestPoint)

        if distanceSq >= oldDistanceSq {
            // No progress - reconstruct last simplex
            b3Assert(backup.count != 0)
            simplex = backup
            break
        }

        // Build new tentative support point
        var searchDirection = Vec3()

        switch simplex.count {
        case 1:
            // v = -A
            searchDirection = -simplex.vertices[0].w

        case 2:
            // v = (AB x AO) x AB
            let a = simplex.vertices[0].w
            let b = simplex.vertices[1].w

            let ab = b - a

            searchDirection = cross(cross(ab, -a), ab)

        case 3:
            // v = AB x AC or v = AC x AB
            let a = simplex.vertices[0].w
            let b = simplex.vertices[1].w
            let c = simplex.vertices[2].w

            let ab = b - a
            let ac = c - a

            let n = cross(ab, ac)

            searchDirection = dot(n, a) < 0 ? n : -n

        default:
            b3Assert(false)
        }

        if lengthSquared(searchDirection) < 1000 * Float.leastNormalMagnitude {
            // The origin is probably contained by a line segment or triangle.
            // Thus the shapes are overlapped.
            var localPointA = Vec3()
            var localPointB = Vec3()
            computeWitnessPoints(simplex, &localPointA, &localPointB)
            distanceOutput.pointA = localPointA
            distanceOutput.pointB = localPointB
            distanceOutput.normal = Vec3()
            distanceOutput.distance = 0
            distanceOutput.iterations = iteration
            distanceOutput.simplexCount = Int32(simplexIndex)
            b3Validate(distance(localPointA, localPointB) < Float.ulpOfOne)
            return distanceOutput
        }

        normal = -searchDirection

        // Get new support points
        let searchDirection1 = searchDirection
        let indexA = getProxySupport(input.proxyA, -searchDirection1)
        let supportA = input.proxyA.points![Int(indexA)]
        let searchDirection2 = mulMV(mt, searchDirection)
        let indexB = getProxySupport(input.proxyB, searchDirection2)
        let supportB = mulMV(m, input.proxyB.points![Int(indexB)]) + xf.p

        // Save current simplex and add new vertex - this can fail if we detect cycling
        backup = simplex

        // Check for duplicate support points. This is the main termination criteria.
        var duplicate = false
        for i in 0..<Int(simplex.count) {
            if simplex.vertices[i].indexA == indexA && simplex.vertices[i].indexB == indexB {
                duplicate = true
                break
            }
        }

        if duplicate {
            break
        }

        simplex.vertices[Int(simplex.count)].indexA = indexA
        simplex.vertices[Int(simplex.count)].indexB = indexB
        simplex.vertices[Int(simplex.count)].wA = supportA
        simplex.vertices[Int(simplex.count)].wB = supportB
        simplex.vertices[Int(simplex.count)].w = supportB - supportA
        simplex.count += 1

        iteration += 1
    }

    // Build witness points.
    var localPointA = Vec3()
    var localPointB = Vec3()
    computeWitnessPoints(simplex, &localPointA, &localPointB)

    distanceOutput.pointA = localPointA
    distanceOutput.pointB = localPointB
    distanceOutput.iterations = iteration
    distanceOutput.simplexCount = Int32(simplexIndex)

    normal = normalize(normal)
    if isNormalized(normal) == false {
        // Treat as overlap
        distanceOutput.distance = 0
        distanceOutput.normal = Vec3()
        return distanceOutput
    }

    // Results stay in frame A
    distanceOutput.distance = distance(localPointA, localPointB)
    distanceOutput.normal = normal

    // Apply radii if requested
    if input.useRadii {
        let rA = input.proxyA.radius
        let rB = input.proxyB.radius
        distanceOutput.distance = maxFloat(0, distanceOutput.distance - rA - rB)

        // Keep closest points on perimeter even if overlapped, this way the points move smoothly.
        distanceOutput.pointA = distanceOutput.pointA + rA * normal
        distanceOutput.pointB = distanceOutput.pointB - rB * normal
    }

    // Simplex is useful, cache it.
    writeCache(&cache, simplex)

    return distanceOutput
}

// MARK: - Shape cast

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point,
/// normal, and translation fraction. The query runs in frame A. Initially touching shapes are a
/// miss.
func shapeCast(_ input: ShapeCastPairInput) -> CastOutput {
    // Compute tolerance
    let linearSlop = B3.linearSlop
    let totalRadius = input.proxyA.radius + input.proxyB.radius
    var target = maxFloat(linearSlop, totalRadius - linearSlop)
    let tolerance = 0.25 * linearSlop

    b3Assert(target > tolerance)

    // Prepare input for distance query
    var cache = SimplexCache()

    var alpha: Float = 0

    var distanceInput = DistanceInput()
    distanceInput.proxyA = input.proxyA
    distanceInput.proxyB = input.proxyB
    distanceInput.useRadii = false

    // The whole cast runs in frame A. Advance the relative pose of B in float each iteration,
    // which keeps the math near the local origin and avoids re-relativizing world poses.
    distanceInput.transform = input.transform

    let delta2 = input.translationB
    var distanceOutput = DistanceOutput()
    var output = CastOutput()
    output.triangleIndex = B3.nullIndex

    var iteration = 0
    let maxIterations = 20

    while iteration < maxIterations {
        output.iterations += 1

        distanceOutput = shapeDistance(distanceInput, &cache)

        if distanceOutput.distance < target + tolerance {
            if iteration == 0 {
                if input.canEncroach && distanceOutput.distance > 2.0 * linearSlop {
                    target = distanceOutput.distance - linearSlop
                } else if input.canEncroach && distanceOutput.distance > 0 {
                    // This is for box movers and can't normally be reached by capsule
                    // movers or regular shape casts.

                    // Near-target start without overlap: advance by first order while
                    // retaining a rest separation. Sweeps that cannot consume the free
                    // gap (receding, tangential, grazing) miss.
                    let retained = 0.5 * linearSlop
                    let needed = distanceOutput.distance - retained
                    let approach = -dot(delta2, distanceOutput.normal)
                    if approach <= maxFloat(needed, 0) {
                        // Moving away, no hit.
                        return output
                    }

                    let fraction = needed > 0 ? needed / approach : 0
                    if fraction >= input.maxFraction {
                        // Still no hit because the max fraction is small.
                        return output
                    }

                    output.fraction = fraction
                    output.point = mulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal)
                    output.normal = distanceOutput.normal
                    output.hit = true
                    return output
                } else {
                    // Initial overlap
                    output.hit = true

                    // Compute a common point
                    let c1 = mulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal)
                    let c2 = mulAdd(distanceOutput.pointB, -input.proxyB.radius, distanceOutput.normal)
                    output.point = lerp(c1, c2, 0.5)
                    return output
                }
            } else {
                // Logging for bad input data
                if distanceOutput.distance > 0 && isNormalized(distanceOutput.normal) == false {
                    for i in 0..<Int(input.proxyA.count) {
                        let p = input.proxyA.points![i]
                        b3Log("pointA[\(i)] = {\(p.x), \(p.y), \(p.z)}")
                    }
                    b3Log("radiusA = \(input.proxyA.radius)")

                    for i in 0..<Int(input.proxyB.count) {
                        let p = input.proxyB.points![i]
                        b3Log("pointB[\(i)] = {\(p.x), \(p.y), \(p.z)}")
                    }
                    b3Log("radiusB = \(input.proxyB.radius)")

                    let xf = input.transform
                    b3Log("transform = {{\(xf.p.x), \(xf.p.y), \(xf.p.z)}, {{\(xf.q.v.x), \(xf.q.v.y), \(xf.q.v.z)}, \(xf.q.s)}")

                    let t = input.translationB
                    b3Log("t = {\(t.x), \(t.y), \(t.z)}")

                    b3Log("maxFraction = \(input.maxFraction), canEncroach = \(input.canEncroach)")

                    // Numerical problem. Likely extreme input.
                    return output
                }

                // Hitting this assert implies that the algorithm brought the shapes too close.
                // b3Assert(distanceOutput.distance > 0 && isNormalized(distanceOutput.normal))

                output.fraction = alpha
                output.point = mulAdd(distanceOutput.pointA, input.proxyA.radius, distanceOutput.normal)
                output.normal = distanceOutput.normal
                output.hit = true
                return output
            }
        }

        b3Assert(distanceOutput.distance > 0)
        b3Assert(isNormalized(distanceOutput.normal))

        // Check if shapes are approaching each other
        let denominator = dot(delta2, distanceOutput.normal)
        if denominator >= 0 {
            // Miss
            return output
        }

        // Advance sweep
        alpha += (target - distanceOutput.distance) / denominator
        if alpha >= input.maxFraction {
            // Success!
            return output
        }

        distanceInput.transform.p = mulAdd(input.transform.p, alpha, delta2)

        iteration += 1
    }

    // Failure!
    return output
}

/// Evaluate the transform sweep at a specific time.
func getSweepTransform(_ sweep: Sweep, _ time: Float) -> Transform {
    var transform = Transform()
    transform.q = nlerp(sweep.q1, sweep.q2, time)
    transform.p = lerp(sweep.c1, sweep.c2, time) - rotateVector(transform.q, sweep.localCenter)
    return transform
}

/// Evaluate the transform sweep at its final time.
func getFinalSweepTransform(_ sweep: Sweep) -> Transform {
    var transform = Transform()
    transform.q = sweep.q2
    transform.p = sweep.c2 - rotateVector(transform.q, sweep.localCenter)
    return transform
}
