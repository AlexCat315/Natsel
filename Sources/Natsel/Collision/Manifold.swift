// Dirk Gregorius contributed portions of this code.

/// The maximum number of clip points kept by the manifold clipping.
let maxClipPoints: Int32 = 64

/// A separating axis candidate.
struct SeparatingAxis: Sendable, Equatable, BitwiseCopyable {
    var normal = Vec3()
    var separation: Float = 0
    var indexA: Int32 = 0
    var indexB: Int32 = 0
    var type: SeparatingFeature = .invalidAxis

    init() {}
}

/// The result of the separating axis queries.
struct AxisQuery: Sendable, Equatable, BitwiseCopyable {
    var faceA = SeparatingAxis()
    var faceB = SeparatingAxis()
    var edge = SeparatingAxis()
    var separatedFeature: SeparatingFeature = .invalidAxis

    init() {}
}

/// A vertex used during clipping.
struct ClipVertex: Sendable, Equatable, BitwiseCopyable {
    var position = Vec3()
    var separation: Float = 0
    var pair = FeaturePair()

    init() {}
}

/// Which shape a feature belongs to.
enum FeatureOwner: UInt8, Sendable, BitwiseCopyable {
    case shapeA = 0
    case shapeB = 1
}

// MARK: - Feature helpers

/// The single point feature pair.
let featurePairSingle = FeaturePair()

/// Pack a feature pair into a 32-bit contact point id.
func makeFeatureId(_ pair: FeaturePair) -> UInt32 {
    (UInt32(pair.owner1) << 24) | (UInt32(pair.index1) << 16) | (UInt32(pair.owner2) << 8) | UInt32(pair.index2)
}

/// Make a feature pair from an incoming and an outgoing edge.
func makeFeaturePair(_ owner1: FeatureOwner, _ index1: Int32, _ owner2: FeatureOwner, _ index2: Int32) -> FeaturePair {
    b3Assert(0 <= index1 && index1 <= Int32(UInt8.max))
    b3Assert(0 <= index2 && index2 <= Int32(UInt8.max))

    var pair = FeaturePair()
    pair.index1 = UInt8(index1)
    pair.owner1 = owner1.rawValue
    pair.index2 = UInt8(index2)
    pair.owner2 = owner2.rawValue
    return pair
}

/// Flip a feature pair.
///
/// This logic seems wrong but it is designed so that choosing face A or B as the reference face
/// does not change the resulting feature pair. This way the contact impulses are persisted even
/// if there is reference face flip-flop. This is verified in the HullAndHull sample using the
/// SAT cache with a manual feature specified such as `manualFaceAxisA`.
func flipPair(_ pair: FeaturePair) -> FeaturePair {
    b3Assert(pair.owner1 == 0 || pair.owner1 == 1)
    b3Assert(pair.owner2 == 0 || pair.owner2 == 1)

    var out = FeaturePair()
    out.owner1 = 1 - pair.owner2
    out.owner2 = 1 - pair.owner1
    out.index1 = pair.index2
    out.index2 = pair.index1
    return out
}

/// Pick the separating axis with the largest separation.
func getBestAxis(_ query: AxisQuery) -> SeparatingAxis {
    b3Validate(query.faceA.type == .faceAxisA)
    b3Validate(query.edge.type == .edgePairAxis)
    b3Validate(query.faceB.type == .faceAxisB)

    if query.faceA.separation > query.faceB.separation {
        if query.edge.separation > query.faceA.separation {
            return query.edge
        }

        return query.faceA
    }

    if query.edge.separation > query.faceB.separation {
        return query.edge
    }

    return query.faceB
}

// MARK: - Incident face

/// Find the face on the other hull most anti-parallel to the reference normal, starting at the
/// supplied vertex.
///
/// This was extended to make the wedge shape get the correct incident face. Instead of looking
/// directly for the most anti-parallel face, we first find the closest vertex (passed in). Then we
/// look for all edges coming out of that vertex and look for the edge that is most perpendicular
/// to the reference normal. Then from that edge, we select the adjacent face that is most
/// anti-parallel to the reference normal.
func findIncidentFace(_ hull: Hull, _ refNormal: Vec3, _ vertexIndex: Int32) -> Int32 {
    b3Assert(0 <= vertexIndex && vertexIndex < hull.vertexCount)

    guard let vertices = hull.vertices, let edges = hull.edges, let planes = hull.planes, let points = hull.points
    else {
        return 0
    }

    var minEdgeIndex: Int32 = -1
    var minEdgeProjection = Float.greatestFiniteMagnitude

    let vertex = vertices + Int(vertexIndex)
    b3Assert(vertex.pointee.edge >= 0)

    var edgeIndex = Int32(vertex.pointee.edge)
    var edge = edges + Int(edgeIndex)
    let edgeOrigin = points[Int(edge.pointee.origin)]
    b3Assert(Int(edge.pointee.origin) == Int(vertexIndex))

    let startEdgeIndex = Int32(vertex.pointee.edge)

    repeat {
        let twin = edges + Int(edge.pointee.twin)
        let twinOrigin = points[Int(twin.pointee.origin)]

        let axis = normalize(twinOrigin - edgeOrigin)
        let edgeProjection = absFloat(dot(axis, refNormal))
        if edgeProjection < minEdgeProjection {
            minEdgeIndex = edgeIndex
            minEdgeProjection = edgeProjection
        }

        edgeIndex = Int32(twin.pointee.next)
        edge = edges + Int(edgeIndex)
        b3Assert(Int(edge.pointee.origin) == Int(vertexIndex))
    } while edgeIndex != startEdgeIndex

    b3Assert(minEdgeIndex >= 0)

    let minEdge = edges + Int(minEdgeIndex)
    let minFaceIndex1 = Int32(minEdge.pointee.face)
    let minPlane1 = planes[Int(minFaceIndex1)]

    let minTwin = edges + Int(minEdge.pointee.twin)
    let minFaceIndex2 = Int32(minTwin.pointee.face)
    let minPlane2 = planes[Int(minFaceIndex2)]

    return dot(minPlane1.normal, refNormal) < dot(minPlane2.normal, refNormal)
        ? minFaceIndex1 : minFaceIndex2
}

// MARK: - Clipping

/// Sutherland-Hodgman style clipping of a polygon against a single plane.
func clipPolygon(
    _ out: UnsafeMutablePointer<ClipVertex>,
    _ polygon: UnsafeMutablePointer<ClipVertex>,
    _ count: Int32,
    _ clipPlane: Plane,
    _ edge: Int32,
    _ refPlane: Plane
) -> Int32 {
    b3Assert(count >= 3)

    var vertex1 = polygon[Int(count - 1)]
    var distance1 = planeSeparation(clipPlane, vertex1.position)
    var outCount: Int32 = 0

    for index in 0..<Int(count) {
        let vertex2 = polygon[index]
        let distance2 = planeSeparation(clipPlane, vertex2.position)

        // Clip edge against plane
        if distance1 <= 0 && distance2 <= 0 {
            // Both vertices are behind the plane - keep vertex2
            out[Int(outCount)] = vertex2
            outCount += 1
        } else if distance1 <= 0 && distance2 > 0 {
            // Vertex1 is behind the plane, vertex2 is in front -> intersection point
            let fraction = distance1 / (distance1 - distance2)
            let position = mulAdd(vertex1.position, fraction, vertex2.position - vertex1.position)

            // Keep intersection point and adjust outgoing edge
            var vertex = ClipVertex()
            vertex.position = position
            vertex.separation = planeSeparation(refPlane, position)
            vertex.pair = vertex2.pair
            vertex.pair.owner2 = FeatureOwner.shapeA.rawValue
            vertex.pair.index2 = UInt8(edge)
            out[Int(outCount)] = vertex
            outCount += 1
        } else if distance2 <= 0 && distance1 > 0 {
            // Vertex1 is in front, vertex2 is behind the plane -> intersection point
            let fraction = distance1 / (distance1 - distance2)
            let position = mulAdd(vertex1.position, fraction, vertex2.position - vertex1.position)

            // Keep intersection point and adjust incoming edge
            var vertex = ClipVertex()
            vertex.position = position
            vertex.separation = planeSeparation(refPlane, position)
            vertex.pair = vertex1.pair
            vertex.pair.owner1 = FeatureOwner.shapeA.rawValue
            vertex.pair.index1 = UInt8(edge)
            out[Int(outCount)] = vertex
            outCount += 1

            // And also keep vertex2
            out[Int(outCount)] = vertex2
            outCount += 1
        }

        // Keep vertex2 as starting vertex for next edge
        vertex1 = vertex2
        distance1 = distance2
    }

    b3Validate(validatePolygon(out, outCount))

    return outCount
}

/// Verify that incoming and outgoing edges match. Empty polygons are valid because a manifold can
/// be fully clipped away when it is reconstructed from the cache.
func validatePolygon(_ polygon: UnsafeMutablePointer<ClipVertex>, _ count: Int32) -> Bool {
    if count == 0 {
        return true
    }

    var vertex1 = polygon[Int(count - 1)]
    for i in 0..<Int(count) {
        let vertex2 = polygon[i]

        if vertex1.pair.owner2 != vertex2.pair.owner1 {
            return false
        }

        if vertex1.pair.index2 != vertex2.pair.index1 {
            return false
        }

        vertex1 = vertex2
    }

    return true
}
