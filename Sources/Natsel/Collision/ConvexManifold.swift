// Separating axis test and hull contact generation. The C SIMD implementation reduces edge
// candidates in ascending index order, so the scalar loops below produce the same winner. The
// wide support query is reproduced including the mantissa index embedding so near ties resolve
// to the same vertex.


/// Run `body` with a mutable buffer over an inline array's storage.
///
/// `InlineArray` has no `withUnsafeMutableBufferPointer`, but the collision kernels below take raw
/// pointers into these scratch vectors. The array is stack allocated, so this adds no work.
@inline(__always)
func withInlineBuffer<let N: Int, Element, R>(
    _ array: inout InlineArray<N, Element>,
    _ body: (UnsafeMutableBufferPointer<Element>) -> R
) -> R {
    withUnsafeMutablePointer(to: &array) { pointer in
        let raw = UnsafeMutableRawPointer(pointer)
        let typed = raw.assumingMemoryBound(to: Element.self)
        return body(UnsafeMutableBufferPointer(start: typed, count: N))
    }
}

/// Read-only form of `withInlineBuffer`.
@inline(__always)
func withInlineReadBuffer<let N: Int, Element, R>(
    _ array: InlineArray<N, Element>,
    _ body: (UnsafeBufferPointer<Element>) -> R
) -> R {
    var copy = array
    return withUnsafePointer(to: &copy) { pointer in
        let raw = UnsafeRawPointer(pointer)
        let typed = raw.assumingMemoryBound(to: Element.self)
        return body(UnsafeBufferPointer(start: typed, count: N))
    }
}

// MARK: - Constants

let maxHullEdges: Int32 = B3.maxHullEdges
let maxHullFaces: Int32 = B3.maxHullFaces
let maxHullVertices: Int32 = B3.maxHullVertices
let parallelEdgeTolerance: Float = B3.parallelEdgeTolerance
let parallelTol: Float = 1e-4
let edgeProbeMinTests = 10

/// The number of bits reserved for the vertex index in the wide support query.
let hullBitCount: Int32 = 7

// MARK: - Segment clipping

/// Clip a segment against a plane.
func clipSegment(_ segment: UnsafeMutablePointer<ClipVertex>, _ plane: Plane) -> Int32 {
    var vertexCount: Int32 = 0
    let vertex1 = segment[0]
    let vertex2 = segment[1]

    let distance1 = planeSeparation(plane, vertex1.position)
    let distance2 = planeSeparation(plane, vertex2.position)

    // If the points are behind the plane
    if distance1 <= 0 {
        segment[Int(vertexCount)] = vertex1
        vertexCount += 1
    }
    if distance2 <= 0 {
        segment[Int(vertexCount)] = vertex2
        vertexCount += 1
    }

    // If the points are on different sides of the plane
    if distance1 * distance2 < 0 {
        // Find intersection point of edge and plane
        let t = distance1 / (distance1 - distance2)
        segment[Int(vertexCount)].position = (1 - t) * vertex1.position + t * vertex2.position
        segment[Int(vertexCount)].pair = distance1 > 0 ? vertex1.pair : vertex2.pair
        vertexCount += 1
    }

    return vertexCount
}

/// Clip a segment against the side planes of a hull face.
func clipSegmentToHullFace(_ segment: UnsafeMutablePointer<ClipVertex>, _ hull: Hull, _ refFace: Int32) -> Int32 {
    let faces = hull.faces!
    let planes = hull.planes!
    let edges = hull.edges!
    let points = hull.points!

    let refPlane = planes[Int(refFace)]

    let face = faces + Int(refFace)

    var edgeIndex = Int32(face.pointee.edge)

    repeat {
        let edge = edges + Int(edgeIndex)
        let nextEdgeIndex = Int32(edge.pointee.next)
        let next = edges + Int(nextEdgeIndex)

        let vertex1 = points[Int(edge.pointee.origin)]
        let vertex2 = points[Int(next.pointee.origin)]
        let tangent = normalize(vertex2 - vertex1)
        let binormal = cross(tangent, refPlane.normal)

        let pointCount = clipSegment(segment, makePlaneFromNormalAndPoint(binormal, vertex1))
        if pointCount < 2 {
            return 0
        }

        edgeIndex = nextEdgeIndex
    } while edgeIndex != Int32(face.pointee.edge)

    return 2
}

// MARK: - Edge candidate test

/// Inscribed sphere edge test. https://box2d.org/posts/2026/09/inscribed-spheres/
func testEdgeCandidateSorted(_ a1: Float, _ a2: Float, _ c: Float, _ bound: Float) -> Bool {
    let hi = maxFloat(a1, a2)
    let lo = minFloat(a1, a2)
    let exterior = hi >= bound

    let u = lo - c * hi
    let maxIsInterior = u >= 0

    let s = 1 - c * c
    let boundTerm = (bound - hi) * (bound + hi)
    let lhs = u * u
    let rhs = boundTerm * s
    let maxBeatsBound = lhs >= rhs

    let nearlyParallel = s < parallelTol

    let interior = maxIsInterior && (maxBeatsBound || nearlyParallel)

    return exterior || interior
}

// MARK: - Wide helpers

/// Round a count up to the SIMD width (4 on ARM, matching the C build).
func roundUpToSIMDWidth(_ count: Int32) -> Int32 {
    (count + 4 - 1) & ~(4 - 1)
}

/// Dot all face normals with `d`, into `dots`.
func getFaceDots(_ hull: Hull, _ d: Vec3, _ dots: UnsafeMutablePointer<Float>) {
    let faceStride = Int(hullSoaStride(hull.faceCount))
    let nx = hull.soaNormals!
    let ny = nx + faceStride
    let nz = ny + faceStride

    let faceCount = Int(hull.faceCount)
    let dx = FloatW(repeating: d.x)
    let dy = FloatW(repeating: d.y)
    let dz = FloatW(repeating: d.z)
    var i = 0
    while i + 4 <= faceCount {
        let x = FloatW(UnsafeRawPointer(nx + i).loadUnaligned(as: SIMD4<Float>.self))
        let y = FloatW(UnsafeRawPointer(ny + i).loadUnaligned(as: SIMD4<Float>.self))
        let z = FloatW(UnsafeRawPointer(nz + i).loadUnaligned(as: SIMD4<Float>.self))
        let value = dx * x + dy * y + dz * z
        dots[i] = value[0]; dots[i + 1] = value[1]
        dots[i + 2] = value[2]; dots[i + 3] = value[3]
        i += 4
    }
    while i < faceCount {
        dots[i] = d.x * nx[i] + d.y * ny[i] + d.z * nz[i]
        i += 1
    }
}

/// dot(n, otherCenter) - offset for all faces of the hull.
func getFacePlaneSeparations(_ hull: Hull, _ otherCenter: Vec3, _ separations: UnsafeMutablePointer<Float>) {
    getFaceDots(hull, otherCenter, separations)

    let planes = hull.planes!
    let faceCount = Int(hull.faceCount)
    for i in 0..<faceCount {
        separations[i] -= planes[i].offset
    }
}

/// Re-test edge candidates against the plane separations of a probe point on the other hull.
func filterEdgeCandidates(
    _ hull: Hull,
    _ planeSeparations: UnsafePointer<Float>,
    _ bound: Float,
    _ edgeIndices: UnsafeMutablePointer<Int32>,
    _ count: Int32
) -> Int32 {
    let halfEdges = hull.edges!
    let cosines = hull.edgeCosines!
    var keptCount: Int32 = 0

    for k in 0..<Int(count) {
        let i = edgeIndices[k]
        let i1 = Int32(halfEdges[Int(i)].face)
        let i2 = Int32(halfEdges[Int(i) + 1].face)
        let c = cosines[Int(i) >> 1]
        edgeIndices[Int(keptCount)] = i
        if testEdgeCandidateSorted(planeSeparations[Int(i1)], planeSeparations[Int(i2)], c, bound) {
            keptCount += 1
        }
    }

    return keptCount
}

/// Four-wide support query, preserving the index bits embedded by the C kernel.
///
/// The C version minimizes `bias - dot(normal, vertex)` with the vertex index embedded in the low
/// mantissa bits and then performs a horizontal minimum. Because the values are positive, that is
/// a plain numeric minimum over all lanes, including the padded tail (which repeats vertex 0 with
/// a higher embedded index, so it never wins a tie). The exact support is recomputed afterwards
/// without the index bits.
func getSupportWide(
    _ normal: Vec3,
    _ vx: UnsafePointer<Float>,
    _ vy: UnsafePointer<Float>,
    _ vz: UnsafePointer<Float>,
    _ n: Int32,
    _ bias: Float
) -> (support: Float, vertexIndex: Int32) {
    var minValue = Float.infinity
    let mask: UInt32 = (1 << UInt32(hullBitCount)) - 1
    let clearMask = SIMD4<UInt32>(repeating: ~mask)
    let indexMask = SIMD4<UInt32>(repeating: mask)
    let nx = FloatW(repeating: normal.x)
    let ny = FloatW(repeating: normal.y)
    let nz = FloatW(repeating: normal.z)
    let bias4 = FloatW(repeating: bias)
    var minimum = FloatW(repeating: .infinity)
    var indices = SIMD4<UInt32>(0, 1, 2, 3)
    var i = 0
    while i + 4 <= Int(n) {
        let x = FloatW(UnsafeRawPointer(vx + i).loadUnaligned(as: SIMD4<Float>.self))
        let y = FloatW(UnsafeRawPointer(vy + i).loadUnaligned(as: SIMD4<Float>.self))
        let z = FloatW(UnsafeRawPointer(vz + i).loadUnaligned(as: SIMD4<Float>.self))
        let value = bias4 - (nx * x + ny * y + nz * z)
        let bits = unsafeBitCast(value.simd, to: SIMD4<UInt32>.self)
        let augmented = FloatW(unsafeBitCast((bits & clearMask) | (indices & indexMask), to: SIMD4<Float>.self))
        minimum = selectW(augmented .< minimum, augmented, minimum)
        indices = indices &+ SIMD4<UInt32>(repeating: 4)
        i += 4
    }
    for lane in 0..<4 where minimum[lane] < minValue { minValue = minimum[lane] }
    while i < Int(n) {
        let d = normal.x * vx[i] + normal.y * vy[i] + normal.z * vz[i]
        let value = bias - d
        let augmented = Float(bitPattern: (value.bitPattern & ~mask) | (UInt32(truncatingIfNeeded: i) & mask))
        if augmented < minValue { minValue = augmented }
        i += 1
    }

    // One horizontal min, the winning lane's value and index bits ride through.
    let vi = Int32(minValue.bitPattern & mask)

    // Exact support for the chosen vertex.
    let support = normal.x * vx[Int(vi)] + normal.y * vy[Int(vi)] + normal.z * vz[Int(vi)]

    return (support, vi)
}

/// Separation of a face plane against the support vertex of the other hull.
func getFaceSeparation(
    _ direction: Vec3,
    _ planeSeparation: Float,
    _ vx: UnsafePointer<Float>,
    _ vy: UnsafePointer<Float>,
    _ vz: UnsafePointer<Float>,
    _ n: Int32,
    _ center: Vec3,
    _ extents: Vec3,
    _ vertexIndex: inout Int32
) -> Float {
    let bias = dot(direction, center) + 1.0625 * dot(absVec3(direction), extents)
    let (support, vi) = getSupportWide(direction, vx, vy, vz, n, bias)
    vertexIndex = vi
    return planeSeparation - support
}

/// Transform a SoA point/normal stream by out = -(R*v (+t)).
func negativeTransformFromSoA(
    _ R: Matrix3,
    _ p: Vec3,
    _ inX: UnsafePointer<Float>,
    _ inY: UnsafePointer<Float>,
    _ inZ: UnsafePointer<Float>,
    _ n: Int32,
    _ outX: UnsafeMutablePointer<Float>,
    _ outY: UnsafeMutablePointer<Float>,
    _ outZ: UnsafeMutablePointer<Float>,
    _ isPoint: Bool
) {
    // row-column
    let r0 = Vec3(R.cx.x, R.cy.x, R.cz.x)
    let r1 = Vec3(R.cx.y, R.cy.y, R.cz.y)
    let r2 = Vec3(R.cx.z, R.cy.z, R.cz.z)

    var i = 0
    while i < Int(n) {
        let v = Vec3(inX[i], inY[i], inZ[i])

        // Rotate one vector per lane
        var out = Vec3(dot(r0, v), dot(r1, v), dot(r2, v))

        if isPoint {
            out = out + p
        }

        outX[i] = -out.x
        outY[i] = -out.y
        outZ[i] = -out.z

        i += 1
    }
}

// MARK: - Separating axis

/// Compute the best face and edge separating axes between two hulls, with B in A's frame.
/// Test/standalone entry point. The production narrow phase supplies a persistent slab because
/// Swift's temporary allocation falls back to the heap above 1 KB.
func computeSeparatingAxis(_ hullA: Hull, _ hullB: Hull, _ xfB: Transform, _ earlyReturn: Bool) -> AxisQuery {
    withUnsafeTemporaryAllocation(byteCount: satScratchByteCount, alignment: 16) { scratch in
        computeSeparatingAxis(hullA, hullB, xfB, earlyReturn, scratch.baseAddress!)
    }
}

func computeSeparatingAxis(
    _ hullA: Hull, _ hullB: Hull, _ xfB: Transform, _ earlyReturn: Bool,
    _ scratch: UnsafeMutableRawPointer
) -> AxisQuery {
    // Match C's uninitialized scratch: every consumed element is written first. One aligned
    // slab avoids clearing 25 maximum-sized arrays for each hull pair.
    precondition(B3.maxHullVertices <= 128 && B3.maxHullFaces <= 128 && B3.maxHullEdges <= 128)
    do {
        let dotA = scratch.advanced(by: 0).bindMemory(to: Float.self, capacity: 132)
        let dotB = scratch.advanced(by: 528).bindMemory(to: Float.self, capacity: 132)
        let planeDotA = scratch.advanced(by: 1056).bindMemory(to: Float.self, capacity: 132)
        let planeDotB = scratch.advanced(by: 1584).bindMemory(to: Float.self, capacity: 132)
        let edgeIndicesA = scratch.advanced(by: 2112).bindMemory(to: Int32.self, capacity: 132)
        let edgeIndicesB = scratch.advanced(by: 2640).bindMemory(to: Int32.self, capacity: 132)
        let bFNx = scratch.advanced(by: 3168).bindMemory(to: Float.self, capacity: 132)
        let bFNy = scratch.advanced(by: 3696).bindMemory(to: Float.self, capacity: 132)
        let bFNz = scratch.advanced(by: 4224).bindMemory(to: Float.self, capacity: 132)
        let bWx = scratch.advanced(by: 4752).bindMemory(to: Float.self, capacity: 132)
        let bWy = scratch.advanced(by: 5280).bindMemory(to: Float.self, capacity: 132)
        let bWz = scratch.advanced(by: 5808).bindMemory(to: Float.self, capacity: 132)
        let aN0x = scratch.advanced(by: 6336).bindMemory(to: Float.self, capacity: 132)
        let aN0y = scratch.advanced(by: 6864).bindMemory(to: Float.self, capacity: 132)
        let aN0z = scratch.advanced(by: 7392).bindMemory(to: Float.self, capacity: 132)
        let aN1x = scratch.advanced(by: 7920).bindMemory(to: Float.self, capacity: 132)
        let aN1y = scratch.advanced(by: 8448).bindMemory(to: Float.self, capacity: 132)
        let aN1z = scratch.advanced(by: 8976).bindMemory(to: Float.self, capacity: 132)
        let aDx = scratch.advanced(by: 9504).bindMemory(to: Float.self, capacity: 132)
        let aDy = scratch.advanced(by: 10032).bindMemory(to: Float.self, capacity: 132)
        let aDz = scratch.advanced(by: 10560).bindMemory(to: Float.self, capacity: 132)
        let aV0x = scratch.advanced(by: 11088).bindMemory(to: Float.self, capacity: 132)
        let aV0y = scratch.advanced(by: 11616).bindMemory(to: Float.self, capacity: 132)
        let aV0z = scratch.advanced(by: 12144).bindMemory(to: Float.self, capacity: 132)
        let aTol = scratch.advanced(by: 12672).bindMemory(to: Float.self, capacity: 132)
        let R = makeMatrixFromQuat(xfB.q)
        let invR = transpose(R)

        let speculativeDistance = B3.speculativeDistance

        var res = AxisQuery()
        res.faceA = SeparatingAxis()
        res.faceA.normal = Vec3()
        res.faceA.separation = -Float.infinity
        res.faceA.indexA = B3.nullIndex
        res.faceA.indexB = B3.nullIndex
        res.faceA.type = .faceAxisA

        res.faceB = SeparatingAxis()
        res.faceB.normal = Vec3()
        res.faceB.separation = -Float.infinity
        res.faceB.indexA = B3.nullIndex
        res.faceB.indexB = B3.nullIndex
        res.faceB.type = .faceAxisB

        res.edge = SeparatingAxis()
        res.edge.normal = Vec3()
        res.edge.separation = -Float.infinity
        res.edge.indexA = B3.nullIndex
        res.edge.indexB = B3.nullIndex
        res.edge.type = .edgePairAxis

        res.separatedFeature = .invalidAxis

        let faceCountA = hullA.faceCount
        let planesA = hullA.planes!

        let vertexStrideB = hullSoaStride(hullB.vertexCount)
        let vxB = hullB.soaVertices!
        let vyB = vxB + Int(vertexStrideB)
        let vzB = vyB + Int(vertexStrideB)
        let wideVertexCountB = roundUpToSIMDWidth(hullB.vertexCount)

        let cB = aabbCenter(hullB.aabb)
        let hB = aabbExtents(hullB.aabb)

        let deltaCenter = mulMV(R, hullB.center) + xfB.p - hullA.center
        let centerDistance = length(deltaCenter)
        let radius = hullA.innerRadius + hullB.innerRadius

        // Adjust the radius to ensure the best axis isn't skipped.
        let radiusBound = radius - (B3.linearSlop + 0.001 * (centerDistance + radius))

        // Stack scratch: this routine runs per candidate hull pair, so heap arrays here dominated the
        // collide cost. The sizes are fixed by the hull limits.

        getFaceDots(hullA, deltaCenter, dotA)

        // Find the face of hullA that most aligns with centerDelta.
        var seedIndexA: Int32 = 0
        var maxDotA = dotA[0]
        for i in 1..<Int(faceCountA) {
            if dotA[i] > maxDotA {
                maxDotA = dotA[i]
                seedIndexA = Int32(i)
            }
        }

        // Use the seed to get a lower bound on the separation for the faces of hullA.
        var floorA = -Float.infinity
        var seedSeparationA = -Float.infinity
        var seedVertexB: Int32 = 0
        if earlyReturn {
            let plane = planesA[Int(seedIndexA)]
            let direction = -mulMV(invR, plane.normal)
            let planeSep = dot(plane.normal, xfB.p) - plane.offset
            seedSeparationA = getFaceSeparation(
                direction, planeSep, vxB, vyB, vzB, wideVertexCountB, cB, hB, &seedVertexB
            )
            floorA = minFloat(seedSeparationA, speculativeDistance)
        }

        // Test A's face planes against B's vertices.
        for i in 0..<Int(faceCountA) {
            // The bound offset ensures the seed will be evaluated.
            if dotA[i] - radiusBound < maxFloat(floorA, res.faceA.separation) {
                continue
            }

            let plane = planesA[i]
            var vertexIndex = seedVertexB
            var separation = seedSeparationA

            // Avoid recomputing the seed face separation.
            if earlyReturn == false || i != Int(seedIndexA) {
                let direction = -mulMV(invR, plane.normal)
                let planeSep = dot(plane.normal, xfB.p) - plane.offset
                separation = getFaceSeparation(
                    direction, planeSep, vxB, vyB, vzB, wideVertexCountB, cB, hB, &vertexIndex
                )
            }

            if separation > res.faceA.separation {
                res.faceA.normal = plane.normal
                res.faceA.separation = separation
                res.faceA.indexA = Int32(i)
                res.faceA.indexB = vertexIndex
                if separation > speculativeDistance && earlyReturn {
                    res.separatedFeature = .faceAxisA
                    return res
                }
            }
        }

        b3Validate(res.faceA.indexA != B3.nullIndex)

        let faceCountB = hullB.faceCount
        let planesB = hullB.planes!

        let vertexStrideA = hullSoaStride(hullA.vertexCount)
        let vxA = hullA.soaVertices!
        let vyA = vxA + Int(vertexStrideA)
        let vzA = vyA + Int(vertexStrideA)
        let wideVertexCountA = roundUpToSIMDWidth(hullA.vertexCount)

        let cA = aabbCenter(hullA.aabb)
        let hA = aabbExtents(hullA.aabb)

        getFaceDots(hullB, -mulMV(invR, deltaCenter), dotB)

        // Find the face of hullB that most aligns with the vector pointing from centerB to centerA.
        var seedIndexB: Int32 = 0
        var maxDotB = dotB[0]
        for i in 1..<Int(faceCountB) {
            if dotB[i] > maxDotB {
                maxDotB = dotB[i]
                seedIndexB = Int32(i)
            }
        }

        // Get a lower bound on the separation for the faces of hullB.
        var floorB = -Float.infinity
        var seedSeparationB = -Float.infinity
        var seedVertexA: Int32 = 0
        if earlyReturn {
            let plane = planesB[Int(seedIndexB)]
            let direction = -mulMV(R, plane.normal)
            let planeSep = dot(direction, xfB.p) - plane.offset
            seedSeparationB = getFaceSeparation(
                direction, planeSep, vxA, vyA, vzA, wideVertexCountA, cA, hA, &seedVertexA
            )

            // Include the floor set by hull A faces.
            floorB = maxFloat(seedSeparationB, res.faceA.separation)
            floorB = minFloat(floorB, speculativeDistance)
        }

        // Test B's face planes against A's vertices.
        for i in 0..<Int(faceCountB) {
            if dotB[i] - radiusBound < maxFloat(floorB, res.faceB.separation) {
                continue
            }

            let plane = planesB[i]
            let direction = -mulMV(R, plane.normal)
            var vertexIndex = seedVertexA
            var separation = seedSeparationB

            // Avoid recomputing the seed face separation.
            if earlyReturn == false || i != Int(seedIndexB) {
                let planeSep = dot(direction, xfB.p) - plane.offset
                separation = getFaceSeparation(
                    direction, planeSep, vxA, vyA, vzA, wideVertexCountA, cA, hA, &vertexIndex
                )
            }

            if separation > res.faceB.separation {
                res.faceB.normal = direction
                res.faceB.separation = separation
                res.faceB.indexA = vertexIndex
                res.faceB.indexB = Int32(i)
                if separation > speculativeDistance && earlyReturn {
                    res.separatedFeature = .faceAxisB
                    return res
                }
            }
        }

        // Gather edges of A that can feasibly create a separating axis that beats the maximum face
        // separation.
        let halfEdgeCountA = hullA.halfEdgeCount
        let halfEdgesA = hullA.edges!
        var na: Int32 = 0

        let halfEdgeCountB = hullB.halfEdgeCount
        let halfEdgesB = hullB.edges!
        var nb: Int32 = 0

        let maxFaceSeparation = maxFloat(res.faceA.separation, res.faceB.separation)
        let boundSlack = radius - radiusBound
        let thresholdA = earlyReturn ? maxFaceSeparation + hullB.innerRadius - boundSlack : -Float.infinity
        let thresholdB = earlyReturn ? maxFaceSeparation + hullA.innerRadius - boundSlack : -Float.infinity

        let centerBinA = mulMV(R, hullB.center) + xfB.p
        let centerAinB = mulMV(invR, hullA.center - xfB.p)

        getFacePlaneSeparations(hullA, centerBinA, planeDotA)
        getFacePlaneSeparations(hullB, centerAinB, planeDotB)

        let cosinesA = hullA.edgeCosines!
        let cosinesB = hullB.edgeCosines!

        for i in stride(from: 0, to: Int(halfEdgeCountA), by: 2) {
            let i1 = Int32(halfEdgesA[i].face)
            let i2 = Int32(halfEdgesA[i + 1].face)
            let c = cosinesA[i >> 1]
            edgeIndicesA[Int(na)] = Int32(i)
            if testEdgeCandidateSorted(planeDotA[Int(i1)], planeDotA[Int(i2)], c, thresholdA) {
                na += 1
            }
        }

        for i in stride(from: 0, to: Int(halfEdgeCountB), by: 2) {
            let i1 = Int32(halfEdgesB[i].face)
            let i2 = Int32(halfEdgesB[i + 1].face)
            let c = cosinesB[i >> 1]
            edgeIndicesB[Int(nb)] = Int32(i)
            if testEdgeCandidateSorted(planeDotB[Int(i1)], planeDotB[Int(i2)], c, thresholdB) {
                nb += 1
            }
        }

        // Apply additional culling using the support points for the best face axes.
        if earlyReturn && nb * ((na + 3) >> 2) >= Int32(edgeProbeMinTests) {
            let probeBound = maxFaceSeparation - (0.1 * B3.linearSlop + 0.001 * absFloat(centerDistance + radius))

            let vertexB = res.faceA.indexB != B3.nullIndex ? res.faceA.indexB : seedVertexB
            let probeB = Vec3(vxB[Int(vertexB)], vyB[Int(vertexB)], vzB[Int(vertexB)])
            getFacePlaneSeparations(hullA, mulMV(R, probeB) + xfB.p, planeDotA)
            na = filterEdgeCandidates(hullA, planeDotA, probeBound, edgeIndicesA, na)

            let vertexA = res.faceB.indexA != B3.nullIndex ? res.faceB.indexA : seedVertexA
            let probeA = Vec3(vxA[Int(vertexA)], vyA[Int(vertexA)], vzA[Int(vertexA)])
            getFacePlaneSeparations(hullB, mulMV(invR, probeA - xfB.p), planeDotB)
            nb = filterEdgeCandidates(hullB, planeDotB, probeBound, edgeIndicesB, nb)
        }

        if na == 0 || nb == 0 {
            // No edge candidates found.
            return res
        }

        // B face normals in A space, negated. B vertices in A space, negated.

        let faceStrideB = hullSoaStride(faceCountB)
        let nxB = hullB.soaNormals!
        let nyB = nxB + Int(faceStrideB)
        let nzB = nyB + Int(faceStrideB)

        let wideFaceCountB = roundUpToSIMDWidth(faceCountB)
        negativeTransformFromSoA(
            R, xfB.p, nxB, nyB, nzB, wideFaceCountB,
            bFNx, bFNy, bFNz, false
        )
        negativeTransformFromSoA(
            R, xfB.p, vxB, vyB, vzB, wideVertexCountB,
            bWx, bWy, bWz, true
        )

        // Per A edge data. n0 and n1 are the two face normals, d the edge vector, v0 the first vertex.
        // tol is the parallel edge tolerance, scaled by the edge length.

        let squaredTol = parallelEdgeTolerance * parallelEdgeTolerance
        for k in 0..<Int(na) {
            let edge = halfEdgesA + Int(edgeIndicesA[k])
            let twin = edge + 1

            let A = planesA[Int(edge.pointee.face)].normal
            let B = planesA[Int(twin.pointee.face)].normal
            aN0x[k] = A.x
            aN0y[k] = A.y
            aN0z[k] = A.z
            aN1x[k] = B.x
            aN1y[k] = B.y
            aN1z[k] = B.z

            let v0 = Int(edge.pointee.origin)
            let v1 = Int(twin.pointee.origin)

            aDx[k] = vxA[v1] - vxA[v0]
            aDy[k] = vyA[v1] - vyA[v0]
            aDz[k] = vzA[v1] - vzA[v0]
            aV0x[k] = vxA[v0]
            aV0y[k] = vyA[v0]
            aV0z[k] = vzA[v0]

            aTol[k] = squaredTol * (aDx[k] * aDx[k] + aDy[k] * aDy[k] + aDz[k] * aDz[k])
        }

        let linearSlop = B3.linearSlop
        let EPS = -linearSlop * linearSlop

        // Scalar edge/edge test. This matches the C scalar fallback, which is required to agree with
        // the SIMD path for cross platform determinism.
        for j in 0..<Int(nb) {
            let edge = halfEdgesB + Int(edgeIndicesB[j])
            let twin = edge + 1
            let f0 = Int(edge.pointee.face)
            let f1 = Int(twin.pointee.face)
            let v0 = Int(edge.pointee.origin)
            let v1 = Int(twin.pointee.origin)

            let C = Vec3(bFNx[f0], bFNy[f0], bFNz[f0])
            let D = Vec3(bFNx[f1], bFNy[f1], bFNz[f1])
            let bv0 = Vec3(bWx[v0], bWy[v0], bWz[v0])
            let bv1 = Vec3(bWx[v1], bWy[v1], bWz[v1])
            let DC = bv1 - bv0

            for i in 0..<Int(na) {
                let d = Vec3(aDx[i], aDy[i], aDz[i])

                // CBA = C.dir, DBA = D.dir, where dir = B_x_A
                let CBA = dot(C, d)
                let DBA = dot(D, d)
                if CBA * DBA >= EPS {
                    continue
                }

                let n0 = Vec3(aN0x[i], aN0y[i], aN0z[i])
                let n1 = Vec3(aN1x[i], aN1y[i], aN1z[i])

                // ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
                let ADC = dot(n0, DC)
                let BDC = dot(n1, DC)
                if ADC * BDC >= EPS || CBA * BDC >= EPS {
                    continue
                }

                // Reject near parallel edges
                let maxCD = maxFloat(CBA * CBA, DBA * DBA)
                if maxCD <= aTol[i] {
                    continue
                }

                // t = -CBA / (DBA - CBA)
                let t = -CBA / (DBA - CBA)

                // normal = lerp(t, C, D) = C + (D-C)*t
                var n = mulAdd(C, t, D - C)
                let len2 = dot(n, n)

                let inv = 1 / len2.squareRoot()
                n = inv * n

                // separation = -dot(normal, av0 + bv0)
                let av0 = Vec3(aV0x[i], aV0y[i], aV0z[i])
                let separation = -dot(av0 + bv0, n)

                if separation > res.edge.separation {
                    res.edge.normal = n
                    res.edge.separation = separation

                    // Half edge index
                    res.edge.indexA = edgeIndicesA[i]
                    res.edge.indexB = edgeIndicesB[j]

                    if separation > speculativeDistance && earlyReturn {
                        res.separatedFeature = .edgePairAxis
                        return res
                    }
                }
            }
        }

        return res
    }
}

// MARK: - Manifold point reduction

/// Reduce the manifold points to a maximum of 4 points. This modifies the input point array to
/// improve performance.
func reduceManifoldPoints(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ points: UnsafeMutablePointer<LocalManifoldPoint>,
    _ count: Int32
) {
    if capacity < 4 {
        return
    }

    var count = count
    let dest = manifold.points!

    if count <= 4 {
        for i in 0..<Int(count) {
            dest[i] = points[i]
        }

        manifold.pointCount = count
        return
    }

    let normal = manifold.normal
    let speculativeDistance = B3.speculativeDistance
    let tolSqr = speculativeDistance * speculativeDistance

    // This bias is very important for contact point consistency across time steps.
    // It creates a pecking order to avoid flickering between candidates with similar scores.
    let bias: Float = 0.95

    // Step 1: find extreme point that is touching
    var bestIndex = B3.nullIndex
    var bestScore = -Float.greatestFiniteMagnitude

    let searchDirection = arbitraryPerp(normal)
    for index in 0..<Int(count) {
        let pt = points + index

        if pt.pointee.separation > speculativeDistance {
            continue
        }

        // The deeper the better
        let score = -pt.pointee.separation + dot(searchDirection, pt.pointee.point)
        if bias * score > bestScore {
            bestIndex = Int32(index)
            bestScore = score
        }
    }

    b3Validate(0 <= bestIndex && bestIndex < count)
    if bestIndex == B3.nullIndex {
        manifold.pointCount = 0
        return
    }

    dest[0] = points[Int(bestIndex)]
    manifold.pointCount = 1

    // Remove best point from array
    points[Int(bestIndex)] = points[Int(count - 1)]
    count -= 1

    let a = dest[0].point

    // Step 2: Find farthest point in 2D
    bestScore = 0
    bestIndex = B3.nullIndex

    for index in 0..<Int(count) {
        let p = points[index].point
        let d = p - a
        let v = mulSub(d, dot(d, normal), normal)
        let distanceSquared = lengthSquared(v)
        let separation = maxFloat(0, -points[index].separation)
        let score = distanceSquared + 4 * separation * separation
        if bias * score > bestScore {
            bestScore = score
            bestIndex = Int32(index)
        }
    }

    if bestScore < tolSqr {
        return
    }

    b3Assert(0 <= bestIndex && bestIndex < count)
    dest[1] = points[Int(bestIndex)]
    manifold.pointCount = 2

    // Remove best point from array
    points[Int(bestIndex)] = points[Int(count - 1)]
    count -= 1

    let b = dest[1].point

    // Step 3: Find the point with the maximum triangular area
    bestScore = tolSqr
    bestIndex = B3.nullIndex
    var bestSignedArea: Float = 0
    let ba = b - a
    for index in 0..<Int(count) {
        let p = points[index].point
        let signedArea = dot(normal, cross(ba, p - a))
        let score = absFloat(signedArea)
        if bias * score >= bestScore {
            bestScore = score
            bestIndex = Int32(index)
            bestSignedArea = signedArea
        }
    }

    if bestIndex == B3.nullIndex {
        return
    }

    b3Assert(bestIndex != B3.nullIndex)

    dest[2] = points[Int(bestIndex)]
    manifold.pointCount = 3
    points[Int(bestIndex)] = points[Int(count - 1)]
    count -= 1

    let c = dest[2].point

    // Step 4: get the point that adds the most area outside the current triangle
    bestScore = tolSqr
    bestIndex = B3.nullIndex
    let sign: Float = bestSignedArea < 0 ? -1 : 1
    for index in 0..<Int(count) {
        let p = points[index].point
        let u1 = sign * dot(normal, cross(p - a, ba))
        let u2 = sign * dot(normal, cross(p - b, c - b))
        let u3 = sign * dot(normal, cross(p - c, a - c))
        let score = maxFloat(u1, maxFloat(u2, u3))

        if bias * score > bestScore {
            bestScore = score
            bestIndex = Int32(index)
        }
    }

    if bestIndex != B3.nullIndex {
        dest[Int(manifold.pointCount)] = points[Int(bestIndex)]
        manifold.pointCount += 1
    }
}

// MARK: - Contact building

/// Build the incident face polygon in frame A.
func buildPolygon(
    _ out: UnsafeMutablePointer<ClipVertex>,
    _ transform: Transform,
    _ hull: Hull,
    _ incFace: Int32,
    _ refPlane: Plane
) -> Int32 {
    let faces = hull.faces!
    let edges = hull.edges!
    let points = hull.points!

    let face = faces + Int(incFace)
    var edgeIndex = Int32(face.pointee.edge)
    b3Assert(Int(edges[Int(edgeIndex)].face) == Int(incFace))

    var outCount: Int32 = 0

    let matrix = makeMatrixFromQuat(transform.q)

    repeat {
        let edge = edges + Int(edgeIndex)

        let nextEdgeIndex = Int32(edge.pointee.next)
        let next = edges + Int(nextEdgeIndex)

        var vertex = ClipVertex()
        vertex.position = mulMV(matrix, points[Int(next.pointee.origin)]) + transform.p
        vertex.separation = planeSeparation(refPlane, vertex.position)
        vertex.pair = makeFeaturePair(.shapeB, edgeIndex, .shapeB, nextEdgeIndex)

        out[Int(outCount)] = vertex
        outCount += 1

        edgeIndex = nextEdgeIndex
    } while edgeIndex != Int32(face.pointee.edge) && outCount < maxClipPoints

    b3Validate(validatePolygon(out, outCount))

    return outCount
}

/// Build a manifold for a face-A reference axis.
func buildFaceAContact(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ hullA: Hull,
    _ hullB: Hull,
    _ transformBtoA: Transform,
    _ query: SeparatingAxis,
    _ slab: UnsafeMutableRawPointer,
    _ cache: inout SATCache
) -> Bool {
    b3Assert(capacity > 0)
    b3Validate(query.type == .faceAxisA)
    b3Validate(0 <= query.indexA && query.indexA < hullA.faceCount)
    b3Validate(0 <= query.indexB && query.indexB < hullB.vertexCount)

    let facesA = hullA.faces!
    let edgesA = hullA.edges!
    let planesA = hullA.planes!
    let pointsA = hullA.points!

    // Reference face
    let refFace = query.indexA
    let refPlane = planesA[Int(refFace)]

    // Find incident face
    let refNormalInB = invRotateVector(transformBtoA.q, refPlane.normal)
    let incFace = findIncidentFace(hullB, refNormalInB, query.indexB)

    // Build the clip polygons and reduced points in one uninitialized temporary slab. Initializing
    // three 64-entry arrays here cleared several kilobytes per face contact, and selecting the
    // active buffer with a ternary copied an entire InlineArray.
    let buffer1 = slab.assumingMemoryBound(to: ClipVertex.self)
    let buffer2 = buffer1 + 64
    let points = UnsafeMutableRawPointer(buffer2 + 64)
        .assumingMemoryBound(to: LocalManifoldPoint.self)

            var pointCount = buildPolygon(buffer1, transformBtoA, hullB, incFace, refPlane)

            // Clip incident face against side planes of reference face
            let face = facesA + Int(refFace)
            var edgeIndex = Int32(face.pointee.edge)

            var useBuffer1AsInput = true

            repeat {
                let edge = edgesA + Int(edgeIndex)
                let nextEdgeIndex = Int32(edge.pointee.next)
                let next = edgesA + Int(nextEdgeIndex)
                let vertex1 = pointsA[Int(edge.pointee.origin)]
                let vertex2 = pointsA[Int(next.pointee.origin)]
                let tangent = normalize(vertex2 - vertex1)
                let binormal = cross(tangent, refPlane.normal)

                let clipPlane = makePlaneFromNormalAndPoint(binormal, vertex1)

                if useBuffer1AsInput {
                    pointCount = clipPolygon(buffer2, buffer1, pointCount, clipPlane, edgeIndex, refPlane)
                } else {
                    pointCount = clipPolygon(buffer1, buffer2, pointCount, clipPlane, edgeIndex, refPlane)
                }
                b3Assert(pointCount <= maxClipPoints)

                useBuffer1AsInput = !useBuffer1AsInput

                if pointCount < 3 {
                    cache = SATCache()
                    return false
                }

                edgeIndex = nextEdgeIndex
            } while edgeIndex != Int32(face.pointee.edge)

            pointCount = minInt(pointCount, maxClipPoints)

            var minSeparation = Float.greatestFiniteMagnitude

            manifold.normal = refPlane.normal

            let input = useBuffer1AsInput ? buffer1 : buffer2

            for i in 0..<Int(pointCount) {
                let clipPoint = input[i]
                var pt = LocalManifoldPoint()

                // Using the half-way point keeps the points in the same position when swapping reference
                // face from A to B.
                let point = mulSub(clipPoint.position, 0.5 * clipPoint.separation, refPlane.normal)

                pt.point = point
                pt.separation = clipPoint.separation
                pt.pair = clipPoint.pair

                minSeparation = minFloat(minSeparation, clipPoint.separation)

                points[i] = pt
            }

            if minSeparation >= B3.speculativeDistance {
                cache = SATCache()
                return false
            }

            reduceManifoldPoints(&manifold, capacity, points, pointCount)

            // Save cache
            cache.separation = minSeparation
            cache.type = .faceAxisA
            cache.indexA = UInt8(truncatingIfNeeded: query.indexA)
            cache.indexB = UInt8(truncatingIfNeeded: query.indexB)

            return true
}

/// Build a manifold for a face-B reference axis.
func buildFaceBContact(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ hullA: Hull,
    _ hullB: Hull,
    _ transformBtoA: Transform,
    _ query: SeparatingAxis,
    _ slab: UnsafeMutableRawPointer,
    _ cache: inout SATCache
) -> Bool {
    b3Validate(query.type == .faceAxisB)

    let transformAtoB = invertTransform(transformBtoA)

    var flippedQuery = SeparatingAxis()
    flippedQuery.normal = -query.normal
    flippedQuery.separation = query.separation
    flippedQuery.indexA = query.indexB
    flippedQuery.indexB = query.indexA
    flippedQuery.type = .faceAxisA

    let touching = buildFaceAContact(
        &manifold, capacity, hullB, hullA, transformAtoB, flippedQuery, slab, &cache
    )
    if touching == false {
        cache = SATCache()
        return false
    }

    // Results are in frame B, need to transform them into frame A
    let matrix = makeMatrixFromQuat(transformBtoA.q)

    // Transform and flip normal so it points from A to B, even though B has the reference face.
    manifold.normal = -mulMV(matrix, manifold.normal)

    // Transform points from frame B to frame A. Also flip the pairs to ensure correct matches.
    let dest = manifold.points!
    for i in 0..<Int(manifold.pointCount) {
        let pt = dest + i
        pt.pointee.point = mulMV(matrix, pt.pointee.point) + transformBtoA.p
        pt.pointee.pair = flipPair(pt.pointee.pair)
    }

    cache.type = .faceAxisB
    cache.indexA = UInt8(truncatingIfNeeded: query.indexA)
    cache.indexB = UInt8(truncatingIfNeeded: query.indexB)

    return true
}

/// Build a single point manifold for an edge pair axis.
func buildEdgeContact(
    _ manifold: inout LocalManifold,
    _ hullA: Hull,
    _ hullB: Hull,
    _ transformBtoA: Transform,
    _ query: SeparatingAxis,
    _ cache: inout SATCache
) -> Bool {
    b3Validate(query.type == .edgePairAxis)
    b3Validate(0 <= query.indexA && query.indexA < hullA.halfEdgeCount)
    b3Validate(0 <= query.indexB && query.indexB < hullB.halfEdgeCount)

    // Work in shapeA coordinates
    let edgesA = hullA.edges!
    let pointsA = hullA.points!

    let edgesB = hullB.edges!
    let pointsB = hullB.points!

    let edgeA = edgesA + Int(query.indexA)
    let twinA = edgesA + Int(edgeA.pointee.twin)
    let pA = pointsA[Int(edgeA.pointee.origin)]
    let qA = pointsA[Int(twinA.pointee.origin)]
    let eA = qA - pA

    let edgeB = edgesB + Int(query.indexB)
    let twinB = edgesB + Int(edgeB.pointee.twin)
    let pB = transformPoint(transformBtoA, pointsB[Int(edgeB.pointee.origin)])
    let qB = transformPoint(transformBtoA, pointsB[Int(twinB.pointee.origin)])
    let eB = qB - pB

    let normal = query.normal
    let result = lineDistance(pA, eA, pB, eB)

    if isWithinSegments(result) == false {
        cache = SATCache()
        return false
    }

    // This can slide off the end from caching
    let separation = dot(normal, result.point2 - result.point1)
    let point = 0.5 * (result.point1 + result.point2)

    // Result in frame A
    manifold.normal = normal
    manifold.pointCount = 1

    let pt = manifold.points!
    pt[0].point = point
    pt[0].separation = separation
    pt[0].pair = makeFeaturePair(.shapeA, query.indexA, .shapeB, query.indexB)

    // Save cache
    cache.separation = separation
    cache.type = .edgePairAxis
    cache.indexA = UInt8(truncatingIfNeeded: query.indexA)
    cache.indexB = UInt8(truncatingIfNeeded: query.indexB)

    return true
}

/// Is the segment distance result inside both segments?
@inline(__always)
func isWithinSegments(_ result: SegmentDistanceResult) -> Bool {
    (0 <= result.fraction1 && result.fraction1 <= 1) && (0 <= result.fraction2 && result.fraction2 <= 1)
}

// MARK: - Hull collision

@inline(__always)
func meshSATScratch(_ scratch: UnsafeMutablePointer<MeshContactScratch>) -> UnsafeMutableRawPointer {
    if scratch.pointee.satScratch.isEmpty {
        scratch.pointee.satScratch = Array(repeating: 0, count: satScratchByteCount)
    }
    return scratch.pointee.satScratch.withUnsafeMutableBytes { $0.baseAddress! }
}

@inline(__always)
func meshFaceScratch(_ scratch: UnsafeMutablePointer<MeshContactScratch>) -> UnsafeMutableRawPointer {
    if scratch.pointee.faceScratch.isEmpty {
        scratch.pointee.faceScratch = Array(repeating: 0, count: faceScratchByteCount)
    }
    return scratch.pointee.faceScratch.withUnsafeMutableBytes { $0.baseAddress! }
}

/// Test/standalone entry point.
func collideHulls(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ hullA: Hull,
    _ hullB: Hull,
    _ transformBtoA: Transform,
    _ cache: inout SATCache
) {
    var scratch = MeshContactScratch()
    withUnsafeMutablePointer(to: &scratch) { scratch in
        collideHulls(&manifold, capacity, hullA, hullB, transformBtoA, scratch, &cache)
    }
}

/// Compute the contact manifold between two hulls. The transform maps B into A's frame.
func collideHulls(
    _ manifold: inout LocalManifold,
    _ capacity: Int32,
    _ hullA: Hull,
    _ hullB: Hull,
    _ transformBtoA: Transform,
    _ scratch: UnsafeMutablePointer<MeshContactScratch>,
    _ cache: inout SATCache
) {
    manifold.pointCount = 0

    if capacity < 4 {
        return
    }

    let satSlab = meshSATScratch(scratch)
    let faceSlab = meshFaceScratch(scratch)

    // Work in shapeA coordinates
    let speculativeDistance = B3.speculativeDistance

    let linearSlop = B3.linearSlop
    let edgesA = hullA.edges!
    let planesA = hullA.planes!
    let pointsA = hullA.points!

    let edgesB = hullB.edges!
    let planesB = hullB.planes!
    let pointsB = hullB.points!

    cache.hit = 0

    // Attempt to use the cache to speed up collision
    switch cache.type {
    case .invalidAxis:
        break

    case .faceAxisA:
        b3Assert(Int32(cache.indexA) < hullA.faceCount)

        // Check for separation using cached face
        let plane = planesA[Int(cache.indexA)]
        let searchDirectionInB = -invRotateVector(transformBtoA.q, plane.normal)

        let vertexIndex = findHullSupportVertex(hullB, searchDirectionInB)
        let support = transformPoint(transformBtoA, pointsB[Int(vertexIndex)])
        let separation = planeSeparation(plane, support)

        if separation >= speculativeDistance {
            // Cache hit, shapes are separated
            cache.hit = 1
            return
        }

        // Attempt face contact using cached feature
        var faceQuery = SeparatingAxis()
        faceQuery.normal = plane.normal
        faceQuery.separation = 0
        faceQuery.indexA = Int32(cache.indexA)
        faceQuery.indexB = vertexIndex
        faceQuery.type = .faceAxisA

        var localCache = SATCache()
        let touching = buildFaceAContact(
            &manifold, capacity, hullA, hullB, transformBtoA, faceQuery, faceSlab, &localCache
        )
        if touching && absFloat(cache.separation - localCache.separation) < linearSlop {
            // Cache hit, contact points generated
            cache.hit = 1
            return
        }

    case .faceAxisB:
        b3Assert(Int32(cache.indexB) < hullB.faceCount)

        // Check for separation using cached face
        let plane = planesB[Int(cache.indexB)]
        let searchDirectionInA = -rotateVector(transformBtoA.q, plane.normal)

        let vertexIndex = findHullSupportVertex(hullA, searchDirectionInA)
        let support = invTransformPoint(transformBtoA, pointsA[Int(vertexIndex)])
        let separation = planeSeparation(plane, support)

        if separation >= speculativeDistance {
            // Cache hit, shapes are separated
            cache.hit = 1
            return
        }

        // Attempt face contact using cached feature
        var faceQuery = SeparatingAxis()
        faceQuery.normal = -plane.normal
        faceQuery.separation = 0
        faceQuery.indexA = vertexIndex
        faceQuery.indexB = Int32(cache.indexB)
        faceQuery.type = .faceAxisB

        var localCache = SATCache()
        let touching = buildFaceBContact(
            &manifold, capacity, hullA, hullB, transformBtoA, faceQuery, faceSlab, &localCache
        )
        if touching && absFloat(cache.separation - localCache.separation) < linearSlop {
            // Cache hit, contact points generated
            cache.hit = 1
            return
        }

    case .edgePairAxis:
        let indexA = Int32(cache.indexA)
        let edge1 = edgesA + Int(indexA)
        let twin1 = edgesA + Int(indexA) + 1
        b3Assert(edge1.pointee.twin == indexA + 1 && twin1.pointee.twin == indexA)

        let pA = pointsA[Int(edge1.pointee.origin)]
        let qA = pointsA[Int(twin1.pointee.origin)]
        let eA = qA - pA

        let uA = planesA[Int(edge1.pointee.face)].normal
        let vA = planesA[Int(twin1.pointee.face)].normal

        let indexB = Int32(cache.indexB)
        let edge2 = edgesB + Int(indexB)
        let twin2 = edgesB + Int(indexB) + 1
        b3Assert(edge2.pointee.twin == indexB + 1 && twin2.pointee.twin == indexB)

        let pB = transformPoint(transformBtoA, pointsB[Int(edge2.pointee.origin)])
        let qB = transformPoint(transformBtoA, pointsB[Int(twin2.pointee.origin)])
        let eB = qB - pB

        let uB = rotateVector(transformBtoA.q, planesB[Int(edge2.pointee.face)].normal)
        let vB = rotateVector(transformBtoA.q, planesB[Int(twin2.pointee.face)].normal)

        // flipping the signs of u2 and v2
        // cross(v2, u2) == cross(-v2, -u2)
        // so we still use -e2
        // but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
        let cba = dot(uB, eA)
        let dba = dot(vB, eA)
        let adc = -dot(uA, eB)
        let bdc = -dot(vA, eB)

        if cba * dba < 0 && adc * bdc < 0 && cba * bdc > 0 {
            // Avoid nearly parallel edges that may lead to invalid separation values at the noise
            // floor.
            let squaredTolerance = parallelEdgeTolerance * parallelEdgeTolerance
            if maxFloat(cba * cba, dba * dba) >= squaredTolerance * lengthSquared(eA) {
                let t = cba / (cba - dba)
                var axis = lerp(uB, vB, t)
                b3Validate(lengthSquared(axis) > 1000 * Float.leastNormalMagnitude)
                axis = normalize(axis)
                let separation = dot(axis, qA - qB)

                if separation > speculativeDistance {
                    // Cache hit, shapes are separated
                    cache.hit = 1
                    return
                }

                // Try to rebuild contact from last features
                var edgeQuery = SeparatingAxis()
                edgeQuery.normal = -axis
                edgeQuery.separation = 0
                edgeQuery.indexA = Int32(cache.indexA)
                edgeQuery.indexB = Int32(cache.indexB)
                edgeQuery.type = .edgePairAxis

                var localCache = SATCache()
                let touching = buildEdgeContact(
                    &manifold, hullA, hullB, transformBtoA, edgeQuery, &localCache
                )

                // This separation tolerance may have a big impact on performance in some
                // benchmarks.
                if touching && absFloat(cache.separation - localCache.separation) < linearSlop {
                    // Cache hit, contact point generated
                    cache.hit = 1
                    return
                }
            }
        }

    case .manualFaceAxisA:
        let axisQuery = computeSeparatingAxis(hullA, hullB, transformBtoA, false, satSlab)
        _ = buildFaceAContact(&manifold, capacity, hullA, hullB, transformBtoA, axisQuery.faceA, faceSlab, &cache)
        return

    case .manualFaceAxisB:
        let axisQuery = computeSeparatingAxis(hullA, hullB, transformBtoA, false, satSlab)
        _ = buildFaceBContact(&manifold, capacity, hullA, hullB, transformBtoA, axisQuery.faceB, faceSlab, &cache)
        return

    case .manualEdgePairAxis:
        let axisQuery = computeSeparatingAxis(hullA, hullB, transformBtoA, false, satSlab)
        let edgeQuery = axisQuery.edge
        if edgeQuery.indexA != B3.nullIndex {
            _ = buildEdgeContact(&manifold, hullA, hullB, transformBtoA, edgeQuery, &cache)
        }
        return

    default:
        b3Assert(false)
    }

    manifold.pointCount = 0
    cache = SATCache()

    let axisQuery = computeSeparatingAxis(hullA, hullB, transformBtoA, true, satSlab)

    if axisQuery.separatedFeature != .invalidAxis {
        // We found a separating axis
        cache.type = axisQuery.separatedFeature

        if axisQuery.separatedFeature == .faceAxisA {
            b3Validate(axisQuery.faceA.separation > speculativeDistance)
            cache.separation = axisQuery.faceA.separation
            cache.indexA = UInt8(truncatingIfNeeded: axisQuery.faceA.indexA)
            cache.indexB = UInt8(truncatingIfNeeded: axisQuery.faceA.indexB)
        } else if axisQuery.separatedFeature == .faceAxisB {
            b3Validate(axisQuery.faceB.separation > speculativeDistance)
            cache.separation = axisQuery.faceB.separation
            cache.indexA = UInt8(truncatingIfNeeded: axisQuery.faceB.indexA)
            cache.indexB = UInt8(truncatingIfNeeded: axisQuery.faceB.indexB)
        } else {
            b3Assert(axisQuery.separatedFeature == .edgePairAxis)
            b3Validate(axisQuery.edge.separation > speculativeDistance)
            cache.separation = axisQuery.edge.separation
            cache.indexA = UInt8(truncatingIfNeeded: axisQuery.edge.indexA)
            cache.indexB = UInt8(truncatingIfNeeded: axisQuery.edge.indexB)
        }
        return
    }

    b3Validate(
        axisQuery.faceA.separation <= speculativeDistance || axisQuery.faceB.separation <= speculativeDistance
            || axisQuery.edge.separation <= speculativeDistance
    )

    if axisQuery.faceA.separation > axisQuery.faceB.separation {
        let faceQuery = axisQuery.faceA
        b3Validate(0 <= faceQuery.indexA && faceQuery.indexA < hullA.faceCount)
        b3Validate(0 <= faceQuery.indexB && faceQuery.indexB < hullB.vertexCount)

        // Face contact A
        _ = buildFaceAContact(&manifold, capacity, hullA, hullB, transformBtoA, faceQuery, faceSlab, &cache)

        b3Validate(Int32(cache.indexA) < hullA.faceCount)
        b3Validate(Int32(cache.indexB) < hullB.vertexCount)
    } else {
        let faceQuery = axisQuery.faceB
        b3Validate(0 <= faceQuery.indexA && faceQuery.indexA < hullA.vertexCount)
        b3Validate(0 <= faceQuery.indexB && faceQuery.indexB < hullB.faceCount)

        // Face contact B
        _ = buildFaceBContact(&manifold, capacity, hullA, hullB, transformBtoA, faceQuery, faceSlab, &cache)

        b3Validate(Int32(cache.indexA) < hullA.vertexCount)
        b3Validate(Int32(cache.indexB) < hullB.faceCount)
    }

    let edgeQuery = axisQuery.edge

    if edgeQuery.indexA == B3.nullIndex {
        // There are no valid edge pairs (all edges parallel)
        return
    }

    let faceSeparation = maxFloat(axisQuery.faceA.separation, axisQuery.faceB.separation)
    let clipSeparation = cache.separation
    let edgeTol = linearSlop

    // Face contact can be empty if it is not the axis of maximum separation. It can also be empty
    // in narrow cases in the speculative region. If that case was important then a GJK fallback
    // would be used. So far it doesn't seem important.
    // Create edge contact if face contact fails or edge contact is significantly better.
    if (manifold.pointCount == 0 && edgeQuery.separation > faceSeparation)
        || edgeQuery.separation > clipSeparation + edgeTol
    {
        b3Assert(0 <= edgeQuery.indexA && edgeQuery.indexA < hullA.halfEdgeCount)
        b3Assert(0 <= edgeQuery.indexB && edgeQuery.indexB < hullB.halfEdgeCount)

        // Edge contact
        var edgePoint = LocalManifoldPoint()
        var edgeManifold = LocalManifold()
        edgeManifold.points = withUnsafeMutablePointer(to: &edgePoint) { $0 }

        var edgeCache = SATCache()
        _ = buildEdgeContact(&edgeManifold, hullA, hullB, transformBtoA, edgeQuery, &edgeCache)

        // It is possible with speculation to have vertex-vertex collision that is missed by SAT,
        // so edge contact yields no points. In that case perhaps the face contact has some points.
        if edgeManifold.pointCount == 1 {
            // Copy edge manifold out, being careful to preserve manifold point buffer.
            let points = manifold.points
            manifold = edgeManifold
            manifold.points = points
            manifold.points![0] = edgePoint
            cache = edgeCache
        }
    }
}
