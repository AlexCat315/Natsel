#if canImport(Darwin)
internal import Darwin
#elseif canImport(Glibc)
internal import Glibc
#elseif canImport(Musl)
internal import Musl
#elseif canImport(Bionic)
internal import Bionic
#elseif os(Windows)
internal import ucrt
#endif

// MARK: - Hull storage

/// A convex hull.
///
/// A hull owns a single zeroed allocation holding the `HullData` header followed by the vertex,
/// point, edge, face, structure-of-arrays, and edge-cosine arrays. The header offsets address that
/// same allocation, mirroring the C layout where the arrays hang off the end of the struct. The
/// allocation is zeroed so padding can never leak into the content hash.
public final class Hull: @unchecked Sendable {
    let storage: GeometryStorage
    var base: UnsafeMutableRawPointer { storage.base }

    /// The total size of the geometry in bytes.
    public var byteCount: Int32 { storage.byteCount }

    init(takingOwnershipOf base: UnsafeMutableRawPointer, byteCount: Int32) {
        storage = GeometryStorage(takingOwnershipOf: base, byteCount: byteCount)
    }

    init(storage: GeometryStorage) { self.storage = storage }

    /// The serialized header. All offsets are relative to `base`.
    var header: UnsafeMutablePointer<HullData> {
        base.assumingMemoryBound(to: HullData.self)
    }

    // MARK: Read-only properties

    /// The number of vertices.
    public var vertexCount: Int32 { header.pointee.vertexCount }

    /// The number of half-edges.
    public var halfEdgeCount: Int32 { header.pointee.edgeCount }

    /// The number of faces.
    public var faceCount: Int32 { header.pointee.faceCount }

    /// The local axis-aligned bounding box.
    public var aabb: AABB { header.pointee.aabb }

    /// The surface area in squared length units.
    public var surfaceArea: Float { header.pointee.surfaceArea }

    /// The volume in cubic length units.
    public var volume: Float { header.pointee.volume }

    /// The radius of the largest sphere centered at the centroid.
    public var innerRadius: Float { header.pointee.innerRadius }

    /// The local centroid.
    public var center: Vec3 { header.pointee.center }

    /// The inertia tensor about the centroid.
    public var centralInertia: Matrix3 { header.pointee.centralInertia }

    /// The content hash. Zero while the hash is being computed.
    public internal(set) var hash: UInt64 {
        get { header.pointee.hash }
        set { header.pointee.hash = newValue }
    }

    /// The serialized version.
    public var version: UInt64 { header.pointee.version }

    // MARK: Array accessors

    var vertices: UnsafeMutablePointer<HullVertex>? {
        access(offset: header.pointee.vertexOffset)
    }

    var points: UnsafeMutablePointer<Vec3>? {
        access(offset: header.pointee.pointOffset)
    }

    var edges: UnsafeMutablePointer<HullHalfEdge>? {
        access(offset: header.pointee.edgeOffset)
    }

    var planes: UnsafeMutablePointer<Plane>? {
        access(offset: header.pointee.planeOffset)
    }

    var faces: UnsafeMutablePointer<HullFace>? {
        access(offset: header.pointee.faceOffset)
    }

    var soaVertices: UnsafeMutablePointer<Float>? {
        access(offset: header.pointee.soaVertexOffset)
    }

    var soaNormals: UnsafeMutablePointer<Float>? {
        access(offset: header.pointee.soaNormalOffset)
    }

    var edgeCosines: UnsafeMutablePointer<Float>? {
        access(offset: header.pointee.edgeCosineOffset)
    }

    private func access<T>(offset: Int32) -> UnsafeMutablePointer<T>? {
        guard offset != 0 else {
            return nil
        }
        return UnsafeMutableRawPointer(base).advanced(by: Int(offset)).assumingMemoryBound(to: T.self)
    }

    // MARK: Convenience

    /// A copy of the hull points.
    public func vertexPositions() -> [Vec3] {
        guard let points else {
            return []
        }
        return (0..<Int(vertexCount)).map { points[$0] }
    }
}

/// The stride of the structure-of-arrays arrays. Padded so the SIMD loops can run past the end.
func hullSoaStride(_ count: Int32) -> Int32 {
    (count + 8 - 1) & ~(8 - 1)
}

/// Byte offsets for a hull allocation. Mirrors the C sizing of `b3HullData` trailing arrays.
struct HullLayout {
    var byteCount = 0
    var vertexOffset = 0
    var pointOffset = 0
    var edgeOffset = 0
    var planeOffset = 0
    var faceOffset = 0
    var soaVertexOffset = 0
    var soaNormalOffset = 0
    var edgeCosineOffset = 0

    init(vertexCount: Int32, edgeCount: Int32, faceCount: Int32) {
        let soaVertexCount = hullSoaStride(vertexCount)
        let soaNormalCount = hullSoaStride(faceCount)

        var n = alignUp8(MemoryLayout<HullData>.stride)

        vertexOffset = n
        n += alignUp8(Int(vertexCount) * MemoryLayout<HullVertex>.stride)

        pointOffset = n
        n += alignUp8(Int(vertexCount) * MemoryLayout<Vec3>.stride)

        edgeOffset = n
        n += alignUp8(Int(edgeCount) * MemoryLayout<HullHalfEdge>.stride)

        planeOffset = n
        n += alignUp8(Int(faceCount) * MemoryLayout<Plane>.stride)

        faceOffset = n
        n += alignUp8(Int(faceCount) * MemoryLayout<HullFace>.stride)

        soaVertexOffset = n
        n += alignUp8(3 * Int(soaVertexCount) * MemoryLayout<Float>.stride)

        soaNormalOffset = n
        n += alignUp8(3 * Int(soaNormalCount) * MemoryLayout<Float>.stride)

        edgeCosineOffset = n
        n += alignUp8(Int(edgeCount / 2) * MemoryLayout<Float>.stride)

        byteCount = n
    }
}

extension Hull {
    /// Allocate a zeroed hull for the given counts and fill in the header offsets.
    static func allocate(vertexCount: Int32, edgeCount: Int32, faceCount: Int32) -> Hull? {
        let layout = HullLayout(vertexCount: vertexCount, edgeCount: edgeCount, faceCount: faceCount)
        guard let base = b3AllocZero(layout.byteCount) else {
            return nil
        }

        let hull = Hull(takingOwnershipOf: base, byteCount: Int32(layout.byteCount))
        let h = hull.header
        h.pointee.version = B3.hullVersion
        h.pointee.vertexOffset = Int32(layout.vertexOffset)
        h.pointee.pointOffset = Int32(layout.pointOffset)
        h.pointee.edgeOffset = Int32(layout.edgeOffset)
        h.pointee.planeOffset = Int32(layout.planeOffset)
        h.pointee.faceOffset = Int32(layout.faceOffset)
        h.pointee.soaVertexOffset = Int32(layout.soaVertexOffset)
        h.pointee.soaNormalOffset = Int32(layout.soaNormalOffset)
        h.pointee.edgeCosineOffset = Int32(layout.edgeCosineOffset)

        h.pointee.vertexCount = vertexCount
        h.pointee.edgeCount = edgeCount
        h.pointee.faceCount = faceCount
        h.pointee.byteCount = Int32(layout.byteCount)

        return hull
    }

    /// Wrap an existing serialized allocation.
    static func restore(takingOwnershipOf base: UnsafeMutableRawPointer, byteCount: Int32) -> Hull {
        Hull(takingOwnershipOf: base, byteCount: byteCount)
    }

    /// Clone the raw bytes of this hull into a new hull.
    func clone() -> Hull? {
        guard isFullyBuilt, let raw = b3Alloc(Int(byteCount)) else {
            return nil
        }
        memcpy(raw, base, Int(byteCount))
        return Hull(takingOwnershipOf: raw, byteCount: byteCount)
    }

    private var isFullyBuilt: Bool {
        header.pointee.version == B3.hullVersion
    }
}

// MARK: - Hull properties

/// Recompute the dot product of the two adjacent face normals for each full edge.
func updateHullEdgeCosines(_ hull: Hull) {
    guard let edges = hull.edges, let planes = hull.planes, let cosines = hull.edgeCosines else {
        return
    }

    let count = Int(hull.halfEdgeCount / 2)

    for i in 0..<count {
        cosines[i] = dot(planes[Int(edges[2 * i].face)].normal, planes[Int(edges[2 * i + 1].face)].normal)
    }
}

/// Recompute the local AABB from the hull points.
func updateHullBounds(_ hull: Hull) {
    guard let points = hull.points else {
        return
    }
    let vertexCount = hull.vertexCount

    b3Assert(vertexCount > 0)
    var bounds = AABB(points[0], points[0])

    var i: Int32 = 1
    while i < vertexCount {
        let p = points[Int(i)]
        bounds.lowerBound = minVec3(bounds.lowerBound, p)
        bounds.upperBound = maxVec3(bounds.upperBound, p)
        i += 1
    }

    hull.header.pointee.aabb = bounds
}

/// Compute the volume, centroid, surface area, inner radius, and inertia tensor.
/// M. Kallay - "Computing the Moment of Inertia of a Solid Defined by a Triangle Mesh"
func updateHullBulkProperties(_ hull: Hull) -> Bool {
    guard let points = hull.points, let faces = hull.faces, let edges = hull.edges, let planes = hull.planes else {
        return false
    }

    var area: Float = 0
    var volume: Float = 0
    var center = Vec3()

    // Use the first vertex to reduce round-off errors.
    let origin = points[0]

    var xx: Float = 0
    var xy: Float = 0
    var yy: Float = 0
    var xz: Float = 0
    var zz: Float = 0
    var yz: Float = 0

    let faceCount = Int(hull.faceCount)

    for faceIndex in 0..<faceCount {
        let face = faces + faceIndex
        let edge1 = edges + Int(face.pointee.edge)
        var edge2 = edges + Int(edge1.pointee.next)
        var edge3 = edges + Int(edge2.pointee.next)

        b3Assert(edge1 != edge3)
        b3Assert(edge1.pointee.origin < hull.vertexCount)

        let v1 = points[Int(edge1.pointee.origin)] - origin

        repeat {
            b3Assert(edge2.pointee.origin < hull.vertexCount)
            b3Assert(edge3.pointee.origin < hull.vertexCount)

            let v2 = points[Int(edge2.pointee.origin)] - origin
            let v3 = points[Int(edge3.pointee.origin)] - origin

            area += length(cross(v2 - v1, v3 - v1))

            let det = scalarTripleProduct(v1, v2, v3)

            volume += det

            let v4 = v1 + (v2 + v3)
            center = center + det * v4

            xx += det * (v1.x * v1.x + v2.x * v2.x + v3.x * v3.x + v4.x * v4.x)
            yy += det * (v1.y * v1.y + v2.y * v2.y + v3.y * v3.y + v4.y * v4.y)
            zz += det * (v1.z * v1.z + v2.z * v2.z + v3.z * v3.z + v4.z * v4.z)
            xy += det * (v1.x * v1.y + v2.x * v2.y + v3.x * v3.y + v4.x * v4.y)
            xz += det * (v1.x * v1.z + v2.x * v2.z + v3.x * v3.z + v4.x * v4.z)
            yz += det * (v1.y * v1.z + v2.y * v2.z + v3.y * v3.z + v4.y * v4.z)

            edge2 = edge3
            edge3 = edges + Int(edge3.pointee.next)
        } while edge1 != edge3
    }

    b3Validate(volume > 0)

    let localCenter = volume > 0 ? (0.25 / volume) * center : Vec3()
    center = localCenter + origin

    var radius = Float.greatestFiniteMagnitude
    for faceIndex in 0..<faceCount {
        let plane = planes[faceIndex]
        let distance = planeSeparation(plane, center)
        b3Validate(distance < 0)

        radius = minFloat(radius, -distance)
    }

    b3Validate(0 < radius && radius < Float.greatestFiniteMagnitude)

    var inertia = Matrix3()
    inertia.cx.x = yy + zz
    inertia.cy.x = -xy
    inertia.cz.x = -xz
    inertia.cx.y = -xy
    inertia.cy.y = xx + zz
    inertia.cz.y = -yz
    inertia.cx.z = -xz
    inertia.cy.z = -yz
    inertia.cz.z = xx + yy

    let mass = volume / 6.0

    var centralInertia = mulSM(1.0 / 120.0, inertia)
    centralInertia = subMM(centralInertia, steiner(mass, localCenter))

    hull.header.pointee.center = center
    hull.header.pointee.centralInertia = centralInertia
    hull.header.pointee.volume = mass
    hull.header.pointee.surfaceArea = 0.5 * area
    hull.header.pointee.innerRadius = radius

    if mass <= 0 {
        return false
    }

    if volume <= 0 {
        return false
    }

    if area <= 0 {
        return false
    }

    if radius <= 0 {
        return false
    }

    return true
}

// MARK: - Validation

/// Verify the hull topology and bulk properties.
func isValidHull(_ hull: Hull) -> Bool {
    guard hull.version == B3.hullVersion,
          let vertices = hull.vertices,
          let edges = hull.edges,
          let faces = hull.faces,
          let planes = hull.planes
    else {
        return false
    }

    let v = Int(hull.vertexCount)
    let e = Int(hull.halfEdgeCount / 2)
    let f = Int(hull.faceCount)

    // Euler characteristic for a closed manifold.
    if v - e + f != 2 {
        return false
    }

    for index in 0..<v {
        let vertex = vertices + index
        let edge = edges + Int(vertex.pointee.edge)

        if Int(edge.pointee.origin) != index {
            return false
        }
    }

    var index = 0
    while index < Int(hull.halfEdgeCount) {
        let edge = edges + index
        let twin = edges + index + 1

        if Int(edge.pointee.twin) != index + 1 {
            return false
        }

        if Int(twin.pointee.twin) != index {
            return false
        }

        index += 2
    }

    for faceIndex in 0..<f {
        let face = faces + faceIndex

        let baseEdgeIndex = Int(face.pointee.edge)
        var edge = edges + baseEdgeIndex

        let plane = planes[faceIndex]
        if planeSeparation(plane, hull.center) >= 0 {
            return false
        }

        var edgeIndex = baseEdgeIndex
        repeat {
            edge = edges + edgeIndex
            let next = edges + Int(edge.pointee.next)
            let twin = edges + Int(edge.pointee.twin)

            if Int(edge.pointee.face) != faceIndex {
                return false
            }

            if twin.pointee.twin != edgeIndex {
                return false
            }

            if next.pointee.origin != twin.pointee.origin {
                return false
            }

            edgeIndex = Int(edge.pointee.next)
        } while edgeIndex != baseEdgeIndex
    }

    if hull.volume <= 0 {
        return false
    }

    if hull.surfaceArea <= 0 {
        return false
    }

    if hull.innerRadius <= 0 {
        return false
    }

    return true
}

// MARK: - Support queries

/// Find the vertex that is farthest in the given direction.
///
/// The C version uses a 4-wide SIMD scan with a per-lane running maximum and then a final
/// lane-ordered tie break. That is equivalent to a forward scalar scan keeping the first maximum,
/// so this scalar loop reproduces the same vertex (needed for deterministic contact features).
func findHullSupportVertex(_ hull: Hull, _ direction: Vec3) -> Int32 {
    var bestIndex = B3.nullIndex
    var bestDot = -Float.greatestFiniteMagnitude

    let vertexCount = hull.vertexCount
    guard let vx = hull.soaVertices else {
        return B3.nullIndex
    }

    let soaVertexCount = Int(hullSoaStride(vertexCount))
    let vy = vx + soaVertexCount
    let vz = vy + soaVertexCount

    var i = 0
    while i < Int(vertexCount) {
        let d = direction.x * vx[i] + direction.y * vy[i] + direction.z * vz[i]
        if d > bestDot {
            bestDot = d
            bestIndex = Int32(i)
        }
        i += 1
    }

    b3Assert(0 <= bestIndex && bestIndex < vertexCount)
    return bestIndex
}

/// Find the face whose normal is most aligned with the given direction.
func findHullSupportFace(_ hull: Hull, _ direction: Vec3) -> Int32 {
    var bestIndex = B3.nullIndex
    var bestDot = -Float.greatestFiniteMagnitude

    let faceCount = hull.faceCount
    let planes = hull.planes!

    var index: Int32 = 0
    while index < faceCount {
        let d = dot(planes[Int(index)].normal, direction)
        if d > bestDot {
            bestDot = d
            bestIndex = index
        }
        index += 1
    }

    b3Assert(bestIndex >= 0)
    return bestIndex
}

// MARK: - Identity

/// The content hash of a hull.
func hashHullData(_ hull: Hull) -> UInt64 { hull.hash }

/// Compare two hulls by their serialized bytes.
func compareHullData(_ hull1: Hull, _ hull2: Hull) -> Bool {
    if hull1 === hull2 {
        return true
    }

    if hull1.byteCount != hull2.byteCount {
        return false
    }

    return memcmp(hull1.base, hull2.base, Int(hull1.byteCount)) == 0
}

/// Compute and store the content hash. The hash field must be zero first.
func computeHullHash(_ hull: Hull) {
    hull.hash = 0
    hull.hash = hash64NonZero(hull.base.assumingMemoryBound(to: UInt8.self), hull.byteCount)
}

// MARK: - Box hulls

/// The box hull topology template. The same vertex, edge, and face ordering as the C constant,
/// so box contacts produce the same feature ids.
private let boxVertexEdges: [UInt8] = [8, 1, 0, 9, 13, 3, 5, 11]

// Each entry is (next, twin, origin, face).
private let boxEdgeData: [(UInt8, UInt8, UInt8, UInt8)] = [
    (2, 1, 2, 0), (17, 0, 1, 5), (4, 3, 1, 0), (20, 2, 5, 3),
    (6, 5, 5, 0), (23, 4, 6, 4), (0, 7, 6, 0), (18, 6, 2, 2),
    (10, 9, 0, 1), (21, 8, 3, 5), (12, 11, 3, 1), (16, 10, 7, 2),
    (14, 13, 7, 1), (19, 12, 4, 4), (8, 15, 4, 1), (22, 14, 0, 3),
    (7, 17, 3, 2), (9, 16, 2, 5), (11, 19, 6, 2), (5, 18, 7, 4),
    (15, 21, 1, 3), (1, 20, 0, 5), (3, 23, 4, 3), (13, 22, 5, 4),
]

private let boxFaceEdges: [UInt8] = [0, 8, 16, 20, 19, 21]

/// Make a transformed box as a hull. Do not destroy a box hull; it is owned by ARC.
public func makeTransformedBoxHull(_ hx: Float, _ hy: Float, _ hz: Float, _ transform: Transform) -> Hull {
    b3Assert(isValidTransform(transform))

    let minH = 0.2 * B3.linearSlop
    let h = maxVec3(Vec3(minH, minH, minH), Vec3(hx, hy, hz))

    guard let hull = Hull.allocate(vertexCount: 8, edgeCount: 24, faceCount: 6) else {
        fatalError("failed to allocate box hull")
    }

    // Topology.
    let vertices = hull.vertices!
    for i in 0..<8 {
        vertices[i].edge = boxVertexEdges[i]
    }

    let edges = hull.edges!
    for i in 0..<24 {
        let e = boxEdgeData[i]
        edges[i].next = e.0
        edges[i].twin = e.1
        edges[i].origin = e.2
        edges[i].face = e.3
    }

    let faces = hull.faces!
    for i in 0..<6 {
        faces[i].edge = boxFaceEdges[i]
    }

    hull.header.pointee.aabb = aabbTransform(transform, AABB(-h, h))
    hull.header.pointee.surfaceArea = 8.0 * (h.x * h.y + h.x * h.z + h.y * h.z)
    hull.header.pointee.volume = 8.0 * h.x * h.y * h.z
    hull.header.pointee.innerRadius = minFloat(h.x, minFloat(h.y, h.z))
    hull.header.pointee.center = transform.p

    let boxInertia_ = boxInertia(hull.header.pointee.volume, -h, h)
    hull.header.pointee.centralInertia = rotateInertia(transform.q, boxInertia_)

    let lower = -h
    let upper = h

    let planes = hull.planes!
    planes[0] = transformPlane(transform, makePlaneFromNormalAndPoint(-Vec3.axisX, lower))
    planes[1] = transformPlane(transform, makePlaneFromNormalAndPoint(Vec3.axisX, upper))
    planes[2] = transformPlane(transform, makePlaneFromNormalAndPoint(-Vec3.axisY, lower))
    planes[3] = transformPlane(transform, makePlaneFromNormalAndPoint(Vec3.axisY, upper))
    planes[4] = transformPlane(transform, makePlaneFromNormalAndPoint(-Vec3.axisZ, lower))
    planes[5] = transformPlane(transform, makePlaneFromNormalAndPoint(Vec3.axisZ, upper))

    let points = hull.points!
    points[0] = transformPoint(transform, Vec3(h.x, h.y, h.z))
    points[1] = transformPoint(transform, Vec3(-h.x, h.y, h.z))
    points[2] = transformPoint(transform, Vec3(-h.x, -h.y, h.z))
    points[3] = transformPoint(transform, Vec3(h.x, -h.y, h.z))
    points[4] = transformPoint(transform, Vec3(h.x, h.y, -h.z))
    points[5] = transformPoint(transform, Vec3(-h.x, h.y, -h.z))
    points[6] = transformPoint(transform, Vec3(-h.x, -h.y, -h.z))
    points[7] = transformPoint(transform, Vec3(h.x, -h.y, -h.z))

    // Structure of arrays.
    let soaVertexCount = Int(hullSoaStride(8))
    let vx = hull.soaVertices!
    let vy = vx + soaVertexCount
    let vz = vy + soaVertexCount
    for i in 0..<8 {
        vx[i] = points[i].x
        vy[i] = points[i].y
        vz[i] = points[i].z
    }

    let soaNormalCount = Int(hullSoaStride(6))
    let nx = hull.soaNormals!
    let ny = nx + soaNormalCount
    let nz = ny + soaNormalCount
    for i in 0..<6 {
        nx[i] = planes[i].normal.x
        ny[i] = planes[i].normal.y
        nz[i] = planes[i].normal.z
    }
    for i in 6..<soaNormalCount {
        nx[i] = 0
        ny[i] = 0
        nz[i] = 0
    }

    // Edge cosines are all zero for an axis-aligned box: adjacent faces are perpendicular.
    computeHullHash(hull)
    return hull
}

/// Make a box as a hull.
public func makeBoxHull(_ hx: Float, _ hy: Float, _ hz: Float) -> Hull {
    makeTransformedBoxHull(hx, hy, hz, Transform.identity)
}

/// Make a cube as a hull.
public func makeCubeHull(_ halfWidth: Float) -> Hull {
    makeBoxHull(halfWidth, halfWidth, halfWidth)
}

/// Make an offset box as a hull.
public func makeOffsetBoxHull(_ hx: Float, _ hy: Float, _ hz: Float, _ offset: Vec3) -> Hull {
    makeTransformedBoxHull(hx, hy, hz, Transform(offset, Quat.identity))
}

/// Scale a box frame while keeping the rotation orthogonal.
func scaleBox(_ halfWidths: inout Vec3, _ transform: inout Transform, _ postScale: Vec3, _ minHalfWidth: Float) {
    b3Assert(isValidFloat(minHalfWidth) && minHalfWidth > 0)

    var q = transform.q

    if postScale.x < 0 || postScale.y < 0 || postScale.z < 0 {
        var m = makeMatrixFromQuat(q)
        m.cx.x *= postScale.x
        m.cy.x *= postScale.x
        m.cz.x *= postScale.x
        m.cx.y *= postScale.y
        m.cy.y *= postScale.y
        m.cz.y *= postScale.y
        m.cx.z *= postScale.z
        m.cy.z *= postScale.z
        m.cz.z *= postScale.z
        m.cx = normalize(m.cx)
        m.cy = normalize(m.cy)
        m.cz = normalize(m.cz)
        m.cx = postScale.x < 0 ? -m.cx : m.cx
        m.cy = postScale.y < 0 ? -m.cy : m.cy
        m.cz = postScale.z < 0 ? -m.cz : m.cz
        q = makeQuatFromMatrix(m)
    }

    let absScale = absVec3(postScale)

    let h = halfWidths
    let p1 = absScale * rotateVector(q, -h)
    let p2 = absScale * rotateVector(q, h)

    let localP1 = invRotateVector(q, p1)
    let localP2 = invRotateVector(q, p2)

    let lower = minVec3(localP1, localP2)
    let upper = maxVec3(localP1, localP2)

    let scaledHalfWidth = 0.5 * (upper - lower)

    let mLimit = Vec3(minHalfWidth, minHalfWidth, minHalfWidth)
    halfWidths = maxVec3(scaledHalfWidth, mLimit)
    transform.p = postScale * transform.p
    transform.q = q
}

/// Make a scaled box as a hull.
public func makeScaledBoxHull(_ halfWidths: Vec3, _ transform: Transform, _ postScale: Vec3) -> Hull {
    b3Assert(isValidTransform(transform))

    var h = halfWidths
    var xf = transform
    scaleBox(&h, &xf, postScale, 4.0 * B3.linearSlop)
    return makeTransformedBoxHull(h.x, h.y, h.z, xf)
}
