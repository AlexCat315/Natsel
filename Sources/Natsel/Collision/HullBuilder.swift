// Dirk Gregorius contributed portions of this code.
//
// Incremental (quickhull style) convex hull builder. This is a direct port of hull.c: the
// intrusive lists, bump-allocated pools with free lists, and every traversal order are kept so
// the emitted vertex/edge/face ordering matches the C library.

// MARK: - Nodes

private let markVisible: Int32 = 0
private let markDelete: Int32 = 1

/// Final hull is index-encoded with UInt8, so counts are capped at 256.
private let hullMaxCount: Int32 = 256

/// Intrusive doubly linked list node. Must be first in the owning struct.
struct QHListNode {
    var prev: UnsafeMutablePointer<QHListNode>?
    var next: UnsafeMutablePointer<QHListNode>?
}

struct QHVertex {
    // Intrusive list link. Must be first so (QHVertex*)node is valid.
    var link = QHListNode()

    var conflictFace: UnsafeMutablePointer<QHFace>?
    var position = Vec3()

    // Index in the finalized hull, stamped during emit.
    var finalIndex: Int32 = B3.nullIndex
    var reachable = false
}

struct QHHalfEdge {
    // Edge ring (CCW) around the owning face. Not an external list.
    var prev: UnsafeMutablePointer<QHHalfEdge>?
    var next: UnsafeMutablePointer<QHHalfEdge>?

    var origin: UnsafeMutablePointer<QHVertex>?
    var face: UnsafeMutablePointer<QHFace>?
    var twin: UnsafeMutablePointer<QHHalfEdge>?

    // Index in the finalized hull, stamped during emit.
    var finalIndex: Int32 = B3.nullIndex
}

struct QHFace {
    // Intrusive list link. Must be first so (QHFace*)node is valid.
    var link = QHListNode()

    var edge: UnsafeMutablePointer<QHHalfEdge>?

    var mark: Int32 = markVisible
    var area: Float = 0
    var plane = Plane()
    var centroid = Vec3()
    var maxConflictDistance: Float = 0

    // Sentinel head for this face's conflict list of QHVertex.
    var conflictListHead = QHVertex()

    // Cached farthest conflict vertex (above builder.minOutside).
    var maxConflict: UnsafeMutablePointer<QHVertex>?

    // Index in the finalized hull, stamped during emit.
    var finalIndex: Int32 = B3.nullIndex
    var flipped = false
}

/// One frame of the iterative horizon DFS.
struct HorizonFrame {
    var face: UnsafeMutablePointer<QHFace>?
    var startEdge: UnsafeMutablePointer<QHHalfEdge>?  // ring termination sentinel
    var edge: UnsafeMutablePointer<QHHalfEdge>?  // next edge to process
    var started = false
}

// MARK: - List operations

@inline(__always)
private func listInit(_ head: UnsafeMutablePointer<QHListNode>) {
    head.pointee.prev = head
    head.pointee.next = head
}

@inline(__always)
private func listEmpty(_ head: UnsafeMutablePointer<QHListNode>) -> Bool {
    head.pointee.next == head
}

@inline(__always)
private func listContains(_ node: UnsafeMutablePointer<QHListNode>) -> Bool {
    node.pointee.prev != nil && node.pointee.next != nil
}

/// Insert `node` before `where`.
@inline(__always)
private func listInsert(_ node: UnsafeMutablePointer<QHListNode>, _ where_: UnsafeMutablePointer<QHListNode>) {
    node.pointee.prev = where_.pointee.prev
    node.pointee.next = where_

    node.pointee.prev!.pointee.next = node
    where_.pointee.prev = node
}

@inline(__always)
private func listRemove(_ node: UnsafeMutablePointer<QHListNode>) {
    node.pointee.prev!.pointee.next = node.pointee.next
    node.pointee.next!.pointee.prev = node.pointee.prev

    node.pointee.prev = nil
    node.pointee.next = nil
}

@inline(__always)
private func listPushBack(_ head: UnsafeMutablePointer<QHListNode>, _ node: UnsafeMutablePointer<QHListNode>) {
    listInsert(node, head.pointee.prev!)
}

@inline(__always)
private func conflictHead(_ face: UnsafeMutablePointer<QHFace>) -> UnsafeMutablePointer<QHListNode> {
    withUnsafeMutablePointer(to: &face.pointee.conflictListHead.link) { $0 }
}

@inline(__always)
private func asVertex(_ node: UnsafeMutablePointer<QHListNode>) -> UnsafeMutablePointer<QHVertex> {
    UnsafeMutableRawPointer(node).assumingMemoryBound(to: QHVertex.self)
}

@inline(__always)
private func asFace(_ node: UnsafeMutablePointer<QHListNode>) -> UnsafeMutablePointer<QHFace> {
    UnsafeMutableRawPointer(node).assumingMemoryBound(to: QHFace.self)
}

// MARK: - Work sizes

/// All working memory for one hull build, carved from a single allocation.
struct HullWorkSizes {
    var N: Int32 = 0
    var M: Int32 = 0
    var vertexCapacity: Int32 = 0
    var edgeCapacity: Int32 = 0
    var faceCapacity: Int32 = 0
    var horizonCapacity: Int32 = 0
    var coneCapacity: Int32 = 0
    var mergedFacesCapacity: Int32 = 0
    var horizonStackCapacity: Int32 = 0
    var totalBytes = 0

    var offsetVertex = 0
    var offsetEdge = 0
    var offsetFace = 0
    var offsetHorizon = 0
    var offsetCone = 0
    var offsetMergedFaces = 0
    var offsetHorizonStack = 0
    var offsetShiftedPoints = 0

    init() {}

    init(pointCount: Int32, clampedMaxCount: Int32) {
        N = pointCount
        M = clampedMaxCount

        // Vertices: 4 initial hull vertices + at most one per remaining input point. No free list.
        vertexCapacity = pointCount + 4

        // Edges and faces use free-list recycling; capacity is proportional to live hull size.
        edgeCapacity = maxInt(48, 24 * M - 48)

        faceCapacity = maxInt(16, 5 * M - 10)

        horizonCapacity = maxInt(6, 3 * M - 6)
        coneCapacity = horizonCapacity
        mergedFacesCapacity = maxInt(4, 2 * M - 4)

        horizonStackCapacity = maxInt(4, 2 * M - 4)

        var offset = 0

        offsetVertex = offset
        offset = alignUp8(offset + Int(vertexCapacity) * MemoryLayout<QHVertex>.stride)

        offsetEdge = offset
        offset = alignUp8(offset + Int(edgeCapacity) * MemoryLayout<QHHalfEdge>.stride)

        offsetFace = offset
        offset = alignUp8(offset + Int(faceCapacity) * MemoryLayout<QHFace>.stride)

        offsetHorizon = offset
        offset = alignUp8(offset + Int(horizonCapacity) * MemoryLayout<UnsafeMutableRawPointer>.stride)

        offsetCone = offset
        offset = alignUp8(offset + Int(coneCapacity) * MemoryLayout<UnsafeMutableRawPointer>.stride)

        offsetMergedFaces = offset
        offset = alignUp8(offset + Int(mergedFacesCapacity) * MemoryLayout<UnsafeMutableRawPointer>.stride)

        offsetHorizonStack = offset
        offset = alignUp8(offset + Int(horizonStackCapacity) * MemoryLayout<HorizonFrame>.stride)

        offsetShiftedPoints = offset
        offset += Int(pointCount) * MemoryLayout<Vec3>.stride

        totalBytes = offset
    }
}

// MARK: - Builder

struct HullBuilder {
    var tolerance: Float = 0
    var minRadius: Float = 0
    var minOutside: Float = 0

    var interiorPoint = Vec3()

    // List sentinels. Only the link is meaningful.
    var orphanedHead: UnsafeMutablePointer<QHListNode>
    var vertexHead: UnsafeMutablePointer<QHListNode>
    var faceHead: UnsafeMutablePointer<QHListNode>

    // Bump-allocated pools with free lists for faces and edges.
    var vertexBase: UnsafeMutablePointer<QHVertex>
    var vertexCapacity: Int32 = 0
    var vertexCount: Int32 = 0

    var edgeBase: UnsafeMutablePointer<QHHalfEdge>
    var edgeCapacity: Int32 = 0
    var edgeCount: Int32 = 0

    // LIFO free list. Built with edge.next.
    var edgeFreeHead: UnsafeMutablePointer<QHHalfEdge>?

    var faceBase: UnsafeMutablePointer<QHFace>
    var faceCapacity: Int32 = 0
    var faceCount: Int32 = 0

    // LIFO free list. Built with face.link.next.
    var faceFreeHead: UnsafeMutablePointer<QHFace>?

    // Reusable scratch buffers.
    var horizon: [UnsafeMutablePointer<QHHalfEdge>] = []
    var cone: [UnsafeMutablePointer<QHFace>] = []
    var mergedFaces: [UnsafeMutablePointer<QHFace>] = []
    var horizonStack: [HorizonFrame] = []

    // Final counts of the constructed hull.
    var finalVertexCount: Int32 = 0
    var finalHalfEdgeCount: Int32 = 0
    var finalFaceCount: Int32 = 0

    init(memory: UnsafeMutableRawPointer, sizes: HullWorkSizes) {
        // Three list sentinels live on the heap so their addresses never move.
        orphanedHead = UnsafeMutablePointer<QHListNode>.allocate(capacity: 1)
        vertexHead = UnsafeMutablePointer<QHListNode>.allocate(capacity: 1)
        faceHead = UnsafeMutablePointer<QHListNode>.allocate(capacity: 1)
        listInit(orphanedHead)
        listInit(vertexHead)
        listInit(faceHead)

        vertexBase = memory.advanced(by: sizes.offsetVertex).assumingMemoryBound(to: QHVertex.self)
        vertexCapacity = sizes.vertexCapacity

        edgeBase = memory.advanced(by: sizes.offsetEdge).assumingMemoryBound(to: QHHalfEdge.self)
        edgeCapacity = sizes.edgeCapacity

        faceBase = memory.advanced(by: sizes.offsetFace).assumingMemoryBound(to: QHFace.self)
        faceCapacity = sizes.faceCapacity

        horizon = []
        horizon.reserveCapacity(Int(sizes.horizonCapacity))
        cone = []
        cone.reserveCapacity(Int(sizes.coneCapacity))
        mergedFaces = []
        mergedFaces.reserveCapacity(Int(sizes.mergedFacesCapacity))
        horizonStack = []
        horizonStack.reserveCapacity(Int(sizes.horizonStackCapacity))
    }

    func destroySentinels() {
        orphanedHead.deallocate()
        vertexHead.deallocate()
        faceHead.deallocate()
    }

    // MARK: Pool management

    mutating func newVertex(_ position: Vec3) -> UnsafeMutablePointer<QHVertex> {
        b3Assert(vertexCount < vertexCapacity)
        let vertex = vertexBase + Int(vertexCount)
        vertexCount += 1

        vertex.pointee.link.prev = nil
        vertex.pointee.link.next = nil
        vertex.pointee.conflictFace = nil
        vertex.pointee.position = position
        vertex.pointee.finalIndex = B3.nullIndex
        vertex.pointee.reachable = false

        return vertex
    }

    mutating func newEdge() -> UnsafeMutablePointer<QHHalfEdge> {
        let edge: UnsafeMutablePointer<QHHalfEdge>
        if let free = edgeFreeHead {
            edge = free
            edgeFreeHead = free.pointee.next
        } else {
            b3Assert(edgeCount < edgeCapacity)
            edge = edgeBase + Int(edgeCount)
            edgeCount += 1
        }
        // All other fields are written by newFace immediately after.
        edge.pointee.finalIndex = B3.nullIndex
        return edge
    }

    mutating func retireEdge(_ edge: UnsafeMutablePointer<QHHalfEdge>) {
        edge.pointee.next = edgeFreeHead
        edgeFreeHead = edge
    }

    mutating func newFace(
        _ v1: UnsafeMutablePointer<QHVertex>,
        _ v2: UnsafeMutablePointer<QHVertex>,
        _ v3: UnsafeMutablePointer<QHVertex>
    ) -> UnsafeMutablePointer<QHFace> {
        let face: UnsafeMutablePointer<QHFace>
        if let free = faceFreeHead {
            face = free
            // link.next was used as free-list pointer; recover next head before we clobber.
            faceFreeHead = free.pointee.link.next.map {
                UnsafeMutableRawPointer($0).assumingMemoryBound(to: QHFace.self)
            }
        } else {
            b3Assert(faceCount < faceCapacity)
            face = faceBase + Int(faceCount)
            faceCount += 1
        }

        // link.prev: nil on retired faces; nil here so pushBack's assert holds for fresh slots.
        // link.next: was the free-list pointer on reused slots, now stale; pushBack overwrites.
        face.pointee.link.prev = nil
        face.pointee.link.next = nil
        face.pointee.maxConflict = nil
        face.pointee.maxConflictDistance = 0
        face.pointee.finalIndex = B3.nullIndex

        let edge1 = newEdge()
        let edge2 = newEdge()
        let edge3 = newEdge()

        let p1 = v1.pointee.position
        let p2 = v2.pointee.position
        let p3 = v3.pointee.position

        var plane = Plane()
        plane.normal = cross(p2 - p1, p3 - p1)
        var length: Float = 0
        plane.normal = getLengthAndNormalize(&length, plane.normal)
        plane.offset = dot(plane.normal, p1)

        let area = 0.5 * length

        face.pointee.edge = edge1
        face.pointee.mark = markVisible
        face.pointee.area = area
        face.pointee.centroid = (1.0 / 3.0) * (v1.pointee.position + (v2.pointee.position + v3.pointee.position))
        face.pointee.plane = plane
        face.pointee.flipped = planeSeparation(plane, interiorPoint) > 0
        listInit(conflictHead(face))

        edge1.pointee.prev = edge3
        edge1.pointee.next = edge2
        edge1.pointee.origin = v1
        edge1.pointee.face = face
        edge1.pointee.twin = nil

        edge2.pointee.prev = edge1
        edge2.pointee.next = edge3
        edge2.pointee.origin = v2
        edge2.pointee.face = face
        edge2.pointee.twin = nil

        edge3.pointee.prev = edge2
        edge3.pointee.next = edge1
        edge3.pointee.origin = v3
        edge3.pointee.face = face
        edge3.pointee.twin = nil

        return face
    }

    /// Remove face and add to free list.
    mutating func retireFace(_ face: UnsafeMutablePointer<QHFace>) {
        // Sometimes a cone face gets merged and never added to the list.
        if listContains(&face.pointee.link) {
            listRemove(&face.pointee.link)
        }

        face.pointee.edge = nil
        b3Validate(face.pointee.link.prev == nil)
        face.pointee.link.next = faceFreeHead.map {
            UnsafeMutableRawPointer($0).assumingMemoryBound(to: QHListNode.self)
        }
        faceFreeHead = face
    }
}

// MARK: - Geometry helpers

private func buildBounds(_ vertices: UnsafePointer<Vec3>, _ vertexCount: Int32) -> AABB {
    var bounds = bounds3Empty
    for i in 0..<Int(vertexCount) {
        bounds.lowerBound = minVec3(bounds.lowerBound, vertices[i])
        bounds.upperBound = maxVec3(bounds.upperBound, vertices[i])
    }
    return bounds
}

private func findFarthestPointsAlongCardinalAxes(
    _ index1Out: inout Int32,
    _ index2Out: inout Int32,
    _ tolerance: Float,
    _ vertexCount: Int32,
    _ vertexBase: UnsafePointer<Vec3>
) {
    index1Out = B3.nullIndex
    index2Out = B3.nullIndex

    let v0 = vertexBase[0]
    var minPt = [v0, v0, v0]
    var maxPt = [v0, v0, v0]

    var minIndex: [Int32] = [0, 0, 0]
    var maxIndex: [Int32] = [0, 0, 0]

    for i in 1..<Int(vertexCount) {
        let v = vertexBase[i]

        if v.x < minPt[0].x {
            minPt[0] = v
            minIndex[0] = Int32(i)
        } else if v.x > maxPt[0].x {
            maxPt[0] = v
            maxIndex[0] = Int32(i)
        }

        if v.y < minPt[1].y {
            minPt[1] = v
            minIndex[1] = Int32(i)
        } else if v.y > maxPt[1].y {
            maxPt[1] = v
            maxIndex[1] = Int32(i)
        }

        if v.z < minPt[2].z {
            minPt[2] = v
            minIndex[2] = Int32(i)
        } else if v.z > maxPt[2].z {
            maxPt[2] = v
            maxIndex[2] = Int32(i)
        }
    }

    let distance = Vec3(
        maxPt[0].x - minPt[0].x,
        maxPt[1].y - minPt[1].y,
        maxPt[2].z - minPt[2].z
    )

    let maxElement = Int(maxElementIndex(distance))

    if getByIndex(distance, maxElement) > 2.0 * tolerance {
        index1Out = minIndex[maxElement]
        index2Out = maxIndex[maxElement]
    }
}

private func findFarthestPointFromLine(
    _ index1: Int32,
    _ index2: Int32,
    _ tolerance: Float,
    _ vertexCount: Int32,
    _ vertexBase: UnsafePointer<Vec3>
) -> Int32 {
    let a = vertexBase[Int(index1)]
    let b = vertexBase[Int(index2)]

    // |ap x ab|^2 / |ab|^2 is the squared perpendicular distance from p to the line.
    // Compares against (2 * tolerance)^2
    let ab = b - a
    let abLengthSqr = dot(ab, ab)
    b3Assert(abLengthSqr > 0)

    let invAbLengthSqr = 1 / abLengthSqr
    var maxDistanceSqr = 4.0 * tolerance * tolerance
    var maxIndex = B3.nullIndex

    for i in 0..<Int(vertexCount) {
        if Int32(i) == index1 || Int32(i) == index2 {
            continue
        }

        let ap = vertexBase[i] - a
        let c = cross(ap, ab)
        let distanceSqr = dot(c, c) * invAbLengthSqr
        if distanceSqr > maxDistanceSqr {
            maxDistanceSqr = distanceSqr
            maxIndex = Int32(i)
        }
    }

    return maxIndex
}

private func findFarthestPointFromPlane(
    _ index1: Int32,
    _ index2: Int32,
    _ index3: Int32,
    _ tolerance: Float,
    _ vertexCount: Int32,
    _ vertexBase: UnsafePointer<Vec3>
) -> Int32 {
    let a = vertexBase[Int(index1)]
    let b = vertexBase[Int(index2)]
    let c = vertexBase[Int(index3)]

    let plane = makePlaneFromPoints(a, b, c)

    var maxDistance = 2.0 * tolerance
    var maxIndex = B3.nullIndex

    for i in 0..<Int(vertexCount) {
        if Int32(i) == index1 || Int32(i) == index2 || Int32(i) == index3 {
            continue
        }

        let distance = absFloat(planeSeparation(plane, vertexBase[i]))
        if distance > maxDistance {
            maxDistance = distance
            maxIndex = Int32(i)
        }
    }

    return maxIndex
}

private func isEdgeConvex(_ edge: UnsafeMutablePointer<QHHalfEdge>, _ tolerance: Float) -> Bool {
    let distance = planeSeparation(edge.pointee.face!.pointee.plane, edge.pointee.twin!.pointee.face!.pointee.centroid)
    return distance < -tolerance
}

private func isEdgeConcave(_ edge: UnsafeMutablePointer<QHHalfEdge>, _ tolerance: Float) -> Bool {
    let distance = planeSeparation(edge.pointee.face!.pointee.plane, edge.pointee.twin!.pointee.face!.pointee.centroid)
    return distance > tolerance
}

private func vertexCountOfFace(_ face: UnsafeMutablePointer<QHFace>) -> Int32 {
    var count: Int32 = 0
    var edge = face.pointee.edge!
    repeat {
        count += 1
        edge = edge.pointee.next!
    } while edge != face.pointee.edge

    return count
}

private func linkFace(_ face: UnsafeMutablePointer<QHFace>, _ index: Int32, _ twin: UnsafeMutablePointer<QHHalfEdge>) {
    b3Assert(face != twin.pointee.face)

    var edge = face.pointee.edge!
    var i = index
    while i > 0 {
        b3Assert(edge.pointee.face == face)
        edge = edge.pointee.next!
        i -= 1
    }

    b3Assert(edge != twin)
    edge.pointee.twin = twin
    twin.pointee.twin = edge
}

private func linkFaces(
    _ face1: UnsafeMutablePointer<QHFace>,
    _ index1: Int32,
    _ face2: UnsafeMutablePointer<QHFace>,
    _ index2: Int32
) {
    b3Assert(face1 != face2)

    var edge1 = face1.pointee.edge!
    var i1 = index1
    while i1 > 0 {
        edge1 = edge1.pointee.next!
        i1 -= 1
    }

    var edge2 = face2.pointee.edge!
    var i2 = index2
    while i2 > 0 {
        edge2 = edge2.pointee.next!
        i2 -= 1
    }

    b3Assert(edge1 != edge2)
    edge1.pointee.twin = edge2
    edge2.pointee.twin = edge1
}

private func newellPlane(_ face: UnsafeMutablePointer<QHFace>) {
    var count: Float = 0
    var centroid = Vec3()
    var normal = Vec3()

    var edge = face.pointee.edge!
    b3Assert(edge.pointee.face == face)

    // Use the first vertex as the origin to reduce round-off
    let origin = edge.pointee.origin!.pointee.position

    repeat {
        let twin = edge.pointee.twin!
        b3Assert(twin.pointee.twin == edge)

        let v1 = edge.pointee.origin!.pointee.position - origin
        let v2 = twin.pointee.origin!.pointee.position - origin

        count += 1
        centroid = centroid + v1
        normal.x += (v1.y - v2.y) * (v1.z + v2.z)
        normal.y += (v1.z - v2.z) * (v1.x + v2.x)
        normal.z += (v1.x - v2.x) * (v1.y + v2.y)

        edge = edge.pointee.next!
    } while edge != face.pointee.edge

    b3Assert(count > 0)
    centroid = (1 / count) * centroid
    centroid = centroid + origin

    let length = length(normal)
    b3Validate(length > 0)
    normal = (1 / length) * normal

    face.pointee.centroid = centroid
    face.pointee.plane = makePlaneFromNormalAndPoint(normal, centroid)
    face.pointee.area = 0.5 * length
}

private func computeTolerance(_ b: inout HullBuilder, _ pointCount: Int32, _ points: UnsafePointer<Vec3>) {
    let bounds = buildBounds(points, pointCount)
    let maxAbs = maxVec3(absVec3(bounds.lowerBound), absVec3(bounds.upperBound))

    let maxSum = maxAbs.x + maxAbs.y + maxAbs.z
    let maxCoord = maxFloat(maxAbs.x, maxFloat(maxAbs.y, maxAbs.z))
    let maxDistance = minFloat(B3.sqrt3 * maxCoord, maxSum)

    let tolerance = (3.0 * maxDistance * 1.01 + maxCoord) * Float.ulpOfOne

    b.tolerance = tolerance
    b.minRadius = 4.0 * b.tolerance
    b.minOutside = 2.0 * b.minRadius
    b3Assert(b.minRadius < b.minOutside + 3.0 * Float.ulpOfOne)
}

private func buildInitialHull(_ b: inout HullBuilder, _ pointCount: Int32, _ points: UnsafePointer<Vec3>) -> Bool {
    var index1: Int32 = 0
    var index2: Int32 = 0
    findFarthestPointsAlongCardinalAxes(&index1, &index2, b.tolerance, pointCount, points)
    if index1 < 0 || index2 < 0 {
        return false
    }

    let index3 = findFarthestPointFromLine(index1, index2, b.tolerance, pointCount, points)
    if index3 < 0 {
        return false
    }

    let index4 = findFarthestPointFromPlane(index1, index2, index3, b.tolerance, pointCount, points)
    if index4 < 0 {
        return false
    }

    let v1 = points[Int(index1)] - points[Int(index4)]
    let v2 = points[Int(index2)] - points[Int(index4)]
    let v3 = points[Int(index3)] - points[Int(index4)]

    var idx2 = index2
    var idx3 = index3
    if scalarTripleProduct(v1, v2, v3) < 0 {
        let temp = idx2
        idx2 = idx3
        idx3 = temp
    }

    b.interiorPoint = Vec3()
    b.interiorPoint = b.interiorPoint + points[Int(index1)]
    b.interiorPoint = b.interiorPoint + points[Int(idx2)]
    b.interiorPoint = b.interiorPoint + points[Int(idx3)]
    b.interiorPoint = b.interiorPoint + points[Int(index4)]
    b.interiorPoint = 0.25 * b.interiorPoint

    let vertex1 = b.newVertex(points[Int(index1)])
    listPushBack(b.vertexHead, &vertex1.pointee.link)
    let vertex2 = b.newVertex(points[Int(idx2)])
    listPushBack(b.vertexHead, &vertex2.pointee.link)
    let vertex3 = b.newVertex(points[Int(idx3)])
    listPushBack(b.vertexHead, &vertex3.pointee.link)
    let vertex4 = b.newVertex(points[Int(index4)])
    listPushBack(b.vertexHead, &vertex4.pointee.link)

    let face1 = b.newFace(vertex1, vertex2, vertex3)
    listPushBack(b.faceHead, &face1.pointee.link)
    let face2 = b.newFace(vertex4, vertex2, vertex1)
    listPushBack(b.faceHead, &face2.pointee.link)
    let face3 = b.newFace(vertex4, vertex3, vertex2)
    listPushBack(b.faceHead, &face3.pointee.link)
    let face4 = b.newFace(vertex4, vertex1, vertex3)
    listPushBack(b.faceHead, &face4.pointee.link)

    linkFaces(face1, 0, face2, 1)
    linkFaces(face1, 1, face3, 1)
    linkFaces(face1, 2, face4, 1)

    linkFaces(face2, 0, face3, 2)
    linkFaces(face3, 0, face4, 2)
    linkFaces(face4, 0, face2, 2)

    for index in 0..<Int(pointCount) {
        if Int32(index) == index1 || Int32(index) == idx2 || Int32(index) == idx3
            || Int32(index) == index4
        {
            continue
        }

        let point = points[index]

        var maxDistance = b.minOutside
        var maxFace: UnsafeMutablePointer<QHFace>?

        var node = b.faceHead.pointee.next!
        while node != b.faceHead {
            let face = asFace(node)
            let distance = planeSeparation(face.pointee.plane, point)
            if distance > maxDistance {
                maxDistance = distance
                maxFace = face
            }
            node = node.pointee.next!
        }

        if let maxFace {
            let vertex = b.newVertex(point)
            vertex.pointee.conflictFace = maxFace
            listPushBack(conflictHead(maxFace), &vertex.pointee.link)
            if maxDistance > maxFace.pointee.maxConflictDistance {
                maxFace.pointee.maxConflictDistance = maxDistance
                maxFace.pointee.maxConflict = vertex
            }
        }
    }

    return true
}

/// Recompute the farthest-conflict cache after a face's plane changes.
private func recacheConflicts(_ face: UnsafeMutablePointer<QHFace>, _ minOutside: Float) {
    var maxVertex: UnsafeMutablePointer<QHVertex>?
    var maxDistance = minOutside

    var node = face.pointee.conflictListHead.link.next!
    let head = conflictHead(face)
    while node != head {
        let vertex = asVertex(node)
        let distance = planeSeparation(face.pointee.plane, vertex.pointee.position)
        if distance > maxDistance {
            maxDistance = distance
            maxVertex = vertex
        }
        node = node.pointee.next!
    }

    face.pointee.maxConflict = maxVertex
    face.pointee.maxConflictDistance = maxDistance
}

private func nextConflictVertex(_ b: inout HullBuilder) -> UnsafeMutablePointer<QHVertex>? {
    var maxVertex: UnsafeMutablePointer<QHVertex>?
    var maxDistance = b.minOutside

    var faceNode = b.faceHead.pointee.next!
    while faceNode != b.faceHead {
        let face = asFace(faceNode)
        if let conflict = face.pointee.maxConflict, face.pointee.maxConflictDistance > maxDistance {
            maxDistance = face.pointee.maxConflictDistance
            maxVertex = conflict
        }
        faceNode = faceNode.pointee.next!
    }

    return maxVertex
}

/// Move every conflict vertex of `face` onto the orphaned list and clear their conflictFace.
private func drainConflictList(_ b: inout HullBuilder, _ face: UnsafeMutablePointer<QHFace>) {
    var node = face.pointee.conflictListHead.link.next!
    let head = conflictHead(face)
    while node != head {
        let orphan = asVertex(node)
        node = node.pointee.next!

        orphan.pointee.conflictFace = nil
        listRemove(&orphan.pointee.link)
        listPushBack(b.orphanedHead, &orphan.pointee.link)
    }
    b3Assert(listEmpty(conflictHead(face)))
}

/// Mark a face for deletion, drain its conflict list, and populate a fresh DFS frame for it.
private func enterHorizonFace(
    _ b: inout HullBuilder,
    _ face: UnsafeMutablePointer<QHFace>,
    _ entryEdge: UnsafeMutablePointer<QHHalfEdge>?,
    _ frameOut: inout HorizonFrame
) {
    face.pointee.mark = markDelete
    drainConflictList(&b, face)

    frameOut.face = face
    frameOut.started = false
    if let entryEdge {
        frameOut.startEdge = entryEdge
        frameOut.edge = entryEdge.pointee.next
    } else {
        frameOut.startEdge = face.pointee.edge
        frameOut.edge = face.pointee.edge
    }
}

private func buildHorizon(_ b: inout HullBuilder, _ apex: UnsafeMutablePointer<QHVertex>, _ seed: UnsafeMutablePointer<QHFace>) {
    b.horizonStack.removeAll(keepingCapacity: true)

    var frame = HorizonFrame()
    enterHorizonFace(&b, seed, nil, &frame)
    b.horizonStack.append(frame)

    while b.horizonStack.isEmpty == false {
        let top = b.horizonStack.count - 1

        if b.horizonStack[top].started && b.horizonStack[top].edge == b.horizonStack[top].startEdge {
            b.horizonStack.removeLast()
            continue
        }
        b.horizonStack[top].started = true

        let edge = b.horizonStack[top].edge!
        let twin = edge.pointee.twin!
        b.horizonStack[top].edge = edge.pointee.next

        if twin.pointee.face!.pointee.mark != markVisible {
            continue
        }

        let distance = planeSeparation(twin.pointee.face!.pointee.plane, apex.pointee.position)
        if distance > b.minRadius {
            var newFrame = HorizonFrame()
            enterHorizonFace(&b, twin.pointee.face!, twin, &newFrame)
            b.horizonStack.append(newFrame)
        } else {
            b.horizon.append(edge)
        }
    }
}

private func buildCone(_ b: inout HullBuilder, _ apex: UnsafeMutablePointer<QHVertex>) {
    for i in 0..<b.horizon.count {
        let edge = b.horizon[i]
        b3Assert(edge.pointee.twin!.pointee.twin == edge)

        let face = b.newFace(apex, edge.pointee.origin!, edge.pointee.twin!.pointee.origin!)
        b.cone.append(face)

        linkFace(face, 1, edge.pointee.twin!)
    }

    var face1 = b.cone[b.cone.count - 1]
    for i in 0..<b.cone.count {
        let face2 = b.cone[i]
        linkFaces(face1, 2, face2, 0)
        face1 = face2
    }
}

/// Retire half-edges in the half-open ring range [begin, end).
private func destroyEdges(_ b: inout HullBuilder, _ begin: UnsafeMutablePointer<QHHalfEdge>, _ end: UnsafeMutablePointer<QHHalfEdge>) {
    var edge = begin
    while edge != end {
        let next = edge.pointee.next!
        b.retireEdge(edge)
        edge = next
    }
}

private func connectEdges(_ b: inout HullBuilder, _ prev: UnsafeMutablePointer<QHHalfEdge>, _ next: UnsafeMutablePointer<QHHalfEdge>) {
    b3Assert(prev != next)
    b3Assert(prev.pointee.face == next.pointee.face)

    // If both shared neighbors are the same face, prev and next together would orphan that face.
    if prev.pointee.twin!.pointee.face == next.pointee.twin!.pointee.face {
        // next is redundant.
        if next.pointee.face!.pointee.edge == next {
            next.pointee.face!.pointee.edge = prev
        }

        let twin: UnsafeMutablePointer<QHHalfEdge>
        if vertexCountOfFace(prev.pointee.twin!.pointee.face!) == 3 {
            // Capture all 3 half-edges of the dead triangle before the rewire overwrites prev.twin.
            let deadEdge0 = prev.pointee.twin!
            let deadEdge1 = next.pointee.twin!
            let deadEdge2 = next.pointee.twin!.pointee.prev!

            twin = deadEdge2.pointee.twin!
            b3Assert(twin.pointee.face!.pointee.mark != markDelete)

            let opposingFace = prev.pointee.twin!.pointee.face!
            opposingFace.pointee.mark = markDelete
            b.mergedFaces.append(opposingFace)

            prev.pointee.next = next.pointee.next
            prev.pointee.next!.pointee.prev = prev

            prev.pointee.twin = twin
            twin.pointee.twin = prev

            // Drop the redundant vertex (slot abandoned in the bump allocator).
            listRemove(&next.pointee.origin!.pointee.link)

            // Retire the 3 half-edges of the dead triangle now that the rewire is complete.
            b.retireEdge(deadEdge0)
            b.retireEdge(deadEdge1)
            b.retireEdge(deadEdge2)
        } else {
            twin = next.pointee.twin!

            if twin.pointee.face!.pointee.edge == prev.pointee.twin {
                twin.pointee.face!.pointee.edge = twin
            }

            twin.pointee.next = prev.pointee.twin!.pointee.next
            twin.pointee.next!.pointee.prev = twin
            // prev.twin slot is retired to the edge free list.
            b.retireEdge(prev.pointee.twin!)

            prev.pointee.next = next.pointee.next
            prev.pointee.next!.pointee.prev = prev

            prev.pointee.twin = twin
            twin.pointee.twin = prev

            // Drop the redundant vertex (slot abandoned in the bump allocator).
            listRemove(&next.pointee.origin!.pointee.link)
        }

        // Twin.face changed shape; recompute its plane and refresh its cached max conflict.
        newellPlane(twin.pointee.face!)
        recacheConflicts(twin.pointee.face!, b.minOutside)
    } else {
        prev.pointee.next = next
        next.pointee.prev = prev
    }
}

private func absorbFaces(_ b: inout HullBuilder, _ face: UnsafeMutablePointer<QHFace>) {
    for i in 0..<b.mergedFaces.count {
        b3Assert(b.mergedFaces[i].pointee.mark == markDelete)
        let head = conflictHead(b.mergedFaces[i])

        var node = head.pointee.next!
        while node != head {
            let vertex = asVertex(node)
            node = node.pointee.next!

            listRemove(&vertex.pointee.link)

            let distance = planeSeparation(face.pointee.plane, vertex.pointee.position)
            if distance > b.minOutside {
                listPushBack(conflictHead(face), &vertex.pointee.link)
                vertex.pointee.conflictFace = face
                if distance > face.pointee.maxConflictDistance {
                    face.pointee.maxConflictDistance = distance
                    face.pointee.maxConflict = vertex
                }
            } else {
                listPushBack(b.orphanedHead, &vertex.pointee.link)
                vertex.pointee.conflictFace = nil
            }
        }

        b3Assert(listEmpty(head))

        // Conflict list is now drained. Retire this face to the free list.
        b.retireFace(b.mergedFaces[i])
    }
}

private func connectFaces(_ b: inout HullBuilder, _ edge: UnsafeMutablePointer<QHHalfEdge>) {
    let face = edge.pointee.face!

    let twin = edge.pointee.twin!

    var edgePrev = edge.pointee.prev!
    var edgeNext = edge.pointee.next!
    var twinPrev = twin.pointee.prev!
    var twinNext = twin.pointee.next!

    while edgePrev.pointee.twin!.pointee.face == twin.pointee.face {
        b3Assert(edgePrev.pointee.twin == twinNext)
        b3Assert(twinNext.pointee.twin == edgePrev)

        edgePrev = edgePrev.pointee.prev!
        twinNext = twinNext.pointee.next!
    }
    b3Assert(edgePrev.pointee.face != twinNext.pointee.face)

    while edgeNext.pointee.twin!.pointee.face == twin.pointee.face {
        b3Assert(edgeNext.pointee.twin == twinPrev)
        b3Assert(twinPrev.pointee.twin == edgeNext)

        edgeNext = edgeNext.pointee.next!
        twinPrev = twinPrev.pointee.prev!
    }
    b3Assert(edgeNext.pointee.face != twinPrev.pointee.face)

    face.pointee.edge = edgePrev

    // Discard opposing face. mergedFaces is single-buffered: connectFaces does not nest.
    b.mergedFaces.removeAll(keepingCapacity: true)
    b.mergedFaces.append(twin.pointee.face!)
    twin.pointee.face!.pointee.mark = markDelete
    twin.pointee.face!.pointee.edge = nil

    var absorbed = twinNext
    while absorbed != twinPrev.pointee.next {
        absorbed.pointee.face = face
        absorbed = absorbed.pointee.next!
    }

    destroyEdges(&b, edgePrev.pointee.next!, edgeNext)
    destroyEdges(&b, twinPrev.pointee.next!, twinNext)

    connectEdges(&b, edgePrev, twinNext)
    connectEdges(&b, twinPrev, edgeNext)

    newellPlane(face)
    // Existing conflicts now have stale distances under the new plane; absorbFaces will then
    // add more incrementally and update the cache as it goes.
    recacheConflicts(face, b.minOutside)

    absorbFaces(&b, face)
}

private func mergeConcave(_ b: inout HullBuilder, _ face: UnsafeMutablePointer<QHFace>) -> Bool {
    var edge = face.pointee.edge!

    repeat {
        let twin = edge.pointee.twin!

        if isEdgeConcave(edge, b.minRadius) || isEdgeConcave(twin, b.minRadius) {
            connectFaces(&b, edge)
            return true
        }

        edge = edge.pointee.next!
    } while edge != face.pointee.edge

    return false
}

private func mergeCoplanar(_ b: inout HullBuilder, _ face: UnsafeMutablePointer<QHFace>) -> Bool {
    var edge = face.pointee.edge!

    repeat {
        let twin = edge.pointee.twin!

        if !isEdgeConvex(edge, b.minRadius) || !isEdgeConvex(twin, b.minRadius) {
            connectFaces(&b, edge)
            return true
        }

        edge = edge.pointee.next!
    } while edge != face.pointee.edge

    return false
}

private func mergeFaces(_ b: inout HullBuilder) {
    for i in 0..<b.cone.count {
        let face = b.cone[i]
        if face.pointee.mark == markVisible && face.pointee.flipped {
            face.pointee.flipped = false

            var bestArea: Float = 0
            var bestEdge: UnsafeMutablePointer<QHHalfEdge>?

            var edge = face.pointee.edge!
            repeat {
                let twin = edge.pointee.twin!
                let area = twin.pointee.face!.pointee.area
                if area > bestArea {
                    bestArea = area
                    bestEdge = edge
                }

                edge = edge.pointee.next!
            } while edge != face.pointee.edge

            b3Assert(bestEdge != nil)
            connectFaces(&b, bestEdge!)
        }
    }

    for i in 0..<b.cone.count {
        let face = b.cone[i]
        if face.pointee.mark == markVisible {
            while mergeConcave(&b, face) {}
        }
    }

    for i in 0..<b.cone.count {
        let face = b.cone[i]
        if face.pointee.mark == markVisible {
            while mergeCoplanar(&b, face) {}
        }
    }
}

private func resolveVertices(_ b: inout HullBuilder) {
    var node = b.orphanedHead.pointee.next!
    while node != b.orphanedHead {
        let vertex = asVertex(node)
        node = node.pointee.next!
        listRemove(&vertex.pointee.link)

        var maxDistance = b.minOutside
        var maxFace: UnsafeMutablePointer<QHFace>?

        for i in 0..<b.cone.count {
            if b.cone[i].pointee.mark == markVisible {
                let distance = planeSeparation(b.cone[i].pointee.plane, vertex.pointee.position)
                if distance > maxDistance {
                    maxDistance = distance
                    maxFace = b.cone[i]
                }
            }
        }

        if let maxFace {
            b3Assert(maxFace.pointee.mark == markVisible)
            listPushBack(conflictHead(maxFace), &vertex.pointee.link)
            vertex.pointee.conflictFace = maxFace
            if maxDistance > maxFace.pointee.maxConflictDistance {
                maxFace.pointee.maxConflictDistance = maxDistance
                maxFace.pointee.maxConflict = vertex
            }
        }
        // Otherwise: vertex is interior to the hull. Its slot in the bump pool is abandoned.
    }

    b3Assert(listEmpty(b.orphanedHead))
}

private func resolveFaces(_ b: inout HullBuilder) {
    // Splice deleted faces out of the face list. Faces already retired by absorbFaces are no
    // longer on faceList, so guard with listContains before removing.
    var node = b.faceHead.pointee.next!
    while node != b.faceHead {
        let face = asFace(node)
        node = node.pointee.next!

        if face.pointee.mark == markDelete && listContains(&face.pointee.link) {
            b3Assert(listEmpty(conflictHead(face)))

            // Each half-edge is owned by exactly one face, so ring walks over the dead region
            // retire every interior edge exactly once. Merge deleted faces are already off the
            // face list.
            let start = face.pointee.edge!
            var edge = start
            repeat {
                let next = edge.pointee.next!
                b.retireEdge(edge)
                edge = next
            } while edge != start

            b.retireFace(face)
        }
    }

    for i in 0..<b.cone.count {
        let face = b.cone[i]
        if face.pointee.mark == markDelete {
            continue
        }
        listPushBack(b.faceHead, &face.pointee.link)
    }
}

private func addVertexToHull(_ b: inout HullBuilder, _ vertex: UnsafeMutablePointer<QHVertex>) {
    let face = vertex.pointee.conflictFace!
    vertex.pointee.conflictFace = nil
    listRemove(&vertex.pointee.link)
    listPushBack(b.vertexHead, &vertex.pointee.link)

    b.horizon.removeAll(keepingCapacity: true)
    buildHorizon(&b, vertex, face)
    b3Assert(b.horizon.count >= 3)

    b.cone.removeAll(keepingCapacity: true)
    buildCone(&b, vertex)
    b3Assert(b.cone.count >= 3)

    mergeFaces(&b)
    resolveVertices(&b)
    resolveFaces(&b)
}

private func cleanHull(_ b: inout HullBuilder, _ origin: Vec3) {
    var faceCount: Int32 = 0
    var halfEdgeCount: Int32 = 0

    var faceNode = b.faceHead.pointee.next!
    while faceNode != b.faceHead {
        let face = asFace(faceNode)
        var edge = face.pointee.edge!

        repeat {
            edge.pointee.origin!.pointee.reachable = true
            edge = edge.pointee.next!
            halfEdgeCount += 1
        } while edge != face.pointee.edge

        face.pointee.plane.offset += dot(face.pointee.plane.normal, origin)
        face.pointee.centroid = face.pointee.centroid + origin
        faceCount += 1

        faceNode = faceNode.pointee.next!
    }

    var vertexCount: Int32 = 0
    var node = b.vertexHead.pointee.next!
    while node != b.vertexHead {
        let vertex = asVertex(node)
        node = node.pointee.next!

        if !vertex.pointee.reachable {
            listRemove(&vertex.pointee.link)
        } else {
            vertex.pointee.position = vertex.pointee.position + origin
            vertexCount += 1
        }
    }

    b.interiorPoint = b.interiorPoint + origin

    b.finalVertexCount = vertexCount
    b.finalHalfEdgeCount = halfEdgeCount
    b.finalFaceCount = faceCount
}

private func hasHull(_ b: inout HullBuilder) -> Bool {
    let v = b.finalVertexCount
    let e = b.finalHalfEdgeCount / 2
    let f = b.finalFaceCount
    return v - e + f == 2 && f >= 4
}

/// Build the entire hull. Returns true iff the result satisfies Euler's identity.
private func constructHull(
    _ b: inout HullBuilder,
    _ points: UnsafePointer<Vec3>,
    _ pointCount: Int32,
    _ maxVertexCount: Int32,
    _ origin: Vec3,
    _ shiftedPoints: UnsafeMutablePointer<Vec3>
) -> Bool {
    if pointCount < 4 {
        return false
    }

    for i in 0..<Int(pointCount) {
        shiftedPoints[i] = points[i] - origin
    }

    computeTolerance(&b, pointCount, shiftedPoints)
    let haveInitialHull = buildInitialHull(&b, pointCount, shiftedPoints)
    if haveInitialHull == false {
        return false
    }

    var budget = clampInt(maxVertexCount - 4, 0, hullMaxCount - 4)

    var vertex = nextConflictVertex(&b)
    while vertex != nil && budget > 0 {
        addVertexToHull(&b, vertex!)
        vertex = nextConflictVertex(&b)
        budget -= 1
    }

    cleanHull(&b, origin)

    return hasHull(&b)
}

// MARK: - Hull creation

/// Create a convex hull from a point cloud.
public func createHull(_ points: [Vec3], maxVertexCount: Int32) -> Hull? {
    guard points.count >= 4 else {
        return nil
    }

    let pointCount = Int32(points.count)
    let origin = points[0]
    let clampedMaxCount = clampInt(maxVertexCount, 4, B3.maxHullVertices)

    // Single allocation for all working memory.
    let sizes = HullWorkSizes(pointCount: pointCount, clampedMaxCount: clampedMaxCount)
    guard let work = b3Alloc(sizes.totalBytes) else {
        return nil
    }

    return points.withUnsafeBufferPointer { pointsBuffer -> Hull? in
        var builder = HullBuilder(memory: work, sizes: sizes)

        let shiftedPoints = work.advanced(by: sizes.offsetShiftedPoints).assumingMemoryBound(to: Vec3.self)

        let ok = constructHull(
            &builder, pointsBuffer.baseAddress!, pointCount, clampedMaxCount, origin, shiftedPoints
        )

        defer {
            builder.destroySentinels()
            b3Free(work, sizes.totalBytes)
        }

        guard ok else {
            return nil
        }

        if builder.finalVertexCount > B3.maxHullVertices {
            b3Log("hull final vertex count of \(builder.finalVertexCount) exceeds limit of \(B3.maxHullVertices)")
            return nil
        }

        if builder.finalFaceCount > B3.maxHullFaces {
            b3Log("hull final face count of \(builder.finalFaceCount) exceeds limit of \(B3.maxHullFaces)")
            return nil
        }

        let maxHalfEdgeCount = 2 * B3.maxHullEdges
        if builder.finalHalfEdgeCount > maxHalfEdgeCount {
            b3Log(
                "hull final half edge count of \(builder.finalHalfEdgeCount) exceeds limit of \(maxHalfEdgeCount)"
            )
            return nil
        }

        return emitHull(&builder)
    }
}

/// Walk the builder lists into the final hull arrays, stamping indices.
private func emitHull(_ b: inout HullBuilder) -> Hull? {
    // Walk lists into temp arrays, stamping finalIndex on each node so the resolution pass below
    // is O(E + F) instead of O(E^2 + F^2).
    var tempVertices: [UnsafeMutablePointer<QHVertex>] = []
    tempVertices.reserveCapacity(Int(hullMaxCount))
    var vertexCount: Int32 = 0
    var node = b.vertexHead.pointee.next!
    while node != b.vertexHead {
        let vertex = asVertex(node)
        vertex.pointee.finalIndex = vertexCount
        tempVertices.append(vertex)
        vertexCount += 1
        node = node.pointee.next!
    }

    // Collect edges in twin-paired order (i, i+1) by stamping each pair as we discover it.
    var tempFaces: [UnsafeMutablePointer<QHFace>] = []
    tempFaces.reserveCapacity(Int(hullMaxCount))
    var tempEdges: [UnsafeMutablePointer<QHHalfEdge>] = []
    tempEdges.reserveCapacity(Int(hullMaxCount))
    var faceCount: Int32 = 0
    var edgeCount: Int32 = 0

    var faceNode = b.faceHead.pointee.next!
    while faceNode != b.faceHead {
        let face = asFace(faceNode)
        face.pointee.finalIndex = faceCount
        tempFaces.append(face)
        faceCount += 1

        var edge = face.pointee.edge!
        repeat {
            if edge.pointee.finalIndex < 0 {
                edge.pointee.finalIndex = edgeCount
                tempEdges.append(edge)
                edgeCount += 1
                edge.pointee.twin!.pointee.finalIndex = edgeCount
                tempEdges.append(edge.pointee.twin!)
                edgeCount += 1
            }
            edge = edge.pointee.next!
        } while edge != face.pointee.edge

        faceNode = faceNode.pointee.next!
    }

    let soaVertexCount = Int(hullSoaStride(vertexCount))
    let soaNormalCount = Int(hullSoaStride(faceCount))

    guard let hull = Hull.allocate(vertexCount: vertexCount, edgeCount: edgeCount, faceCount: faceCount) else {
        return nil
    }

    let vertices = hull.vertices!
    let edges = hull.edges!
    let faces = hull.faces!
    let finalPoints = hull.points!
    let planes = hull.planes!
    let soaVertices = hull.soaVertices!
    let soaNormals = hull.soaNormals!

    let vx = soaVertices
    let vy = vx + soaVertexCount
    let vz = vy + soaVertexCount

    for index in 0..<Int(vertexCount) {
        vertices[index].edge = 0
        let p = tempVertices[index].pointee.position
        finalPoints[index] = p

        vx[index] = p.x
        vy[index] = p.y
        vz[index] = p.z
    }

    for index in Int(vertexCount)..<soaVertexCount {
        vx[index] = vx[0]
        vy[index] = vy[0]
        vz[index] = vz[0]
    }

    for index in 0..<Int(edgeCount) {
        let edge = tempEdges[index]
        b3Assert(0 <= edge.pointee.next!.pointee.finalIndex && edge.pointee.next!.pointee.finalIndex <= Int32(UInt8.max))
        b3Assert(0 <= edge.pointee.twin!.pointee.finalIndex && edge.pointee.twin!.pointee.finalIndex <= Int32(UInt8.max))
        b3Assert(0 <= edge.pointee.face!.pointee.finalIndex && edge.pointee.face!.pointee.finalIndex <= Int32(UInt8.max))
        b3Assert(0 <= edge.pointee.origin!.pointee.finalIndex && edge.pointee.origin!.pointee.finalIndex <= Int32(UInt8.max))

        edges[index].next = UInt8(truncatingIfNeeded: edge.pointee.next!.pointee.finalIndex)
        edges[index].twin = UInt8(truncatingIfNeeded: edge.pointee.twin!.pointee.finalIndex)
        edges[index].face = UInt8(truncatingIfNeeded: edge.pointee.face!.pointee.finalIndex)
        edges[index].origin = UInt8(truncatingIfNeeded: edge.pointee.origin!.pointee.finalIndex)

        vertices[Int(edge.pointee.origin!.pointee.finalIndex)].edge = UInt8(truncatingIfNeeded: index)
    }

    let nx = soaNormals
    let ny = nx + soaNormalCount
    let nz = ny + soaNormalCount

    for index in 0..<Int(faceCount) {
        let face = tempFaces[index]
        b3Assert(0 <= face.pointee.edge!.pointee.finalIndex && face.pointee.edge!.pointee.finalIndex <= Int32(UInt8.max))

        faces[index].edge = UInt8(truncatingIfNeeded: face.pointee.edge!.pointee.finalIndex)
        planes[index] = face.pointee.plane

        let n = face.pointee.plane.normal
        nx[index] = n.x
        ny[index] = n.y
        nz[index] = n.z
    }

    for index in Int(faceCount)..<soaNormalCount {
        nx[index] = 0
        ny[index] = 0
        nz[index] = 0
    }

    // All builder pointers are dead from here on.
    updateHullEdgeCosines(hull)

    updateHullBounds(hull)
    let success = updateHullBulkProperties(hull)
    if success == false {
        return nil
    }

    if isValidHull(hull) == false {
        return nil
    }

    computeHullHash(hull)
    return hull
}

/// Create a tessellated cylinder as a hull.
public func createCylinder(_ height: Float, _ radius: Float, _ yOffset: Float, _ sides: Int32) -> Hull? {
    b3Assert(height > 0)
    b3Assert(radius > 0)
    b3Assert(3 <= sides && sides <= 32)

    let pointCount = Int(2 * sides)
    var points = [Vec3](repeating: Vec3(), count: pointCount)

    var alpha: Float = 0
    let deltaAlpha = 2.0 * B3.pi / Float(sides)

    for index in 0..<Int(sides) {
        let cs = computeCosSin(alpha)

        points[2 * index + 0] = Vec3(radius * cs.cosine, yOffset, radius * cs.sine)
        points[2 * index + 1] = Vec3(radius * cs.cosine, yOffset + height, radius * cs.sine)

        alpha += deltaAlpha
    }

    let hull = createHull(points, maxVertexCount: Int32(pointCount))
    b3Assert(hull == nil || hull!.vertexCount == Int32(pointCount))
    b3Assert(hull == nil || hull!.halfEdgeCount == 6 * sides)
    b3Assert(hull == nil || hull!.faceCount == sides + 2)

    return hull
}

/// Create a tessellated cone as a hull.
public func createCone(_ height: Float, _ radius1: Float, _ radius2: Float, _ slices: Int32) -> Hull? {
    b3Assert(height > 0)
    b3Assert(radius1 > 0)
    b3Assert(radius2 > 0)
    b3Assert(4 <= slices && slices <= 32)

    let pointCount = Int(2 * slices)
    var points = [Vec3](repeating: Vec3(), count: pointCount)

    var alpha: Float = 0
    let deltaAlpha = 2.0 * B3.pi / Float(slices)

    for index in 0..<Int(slices) {
        let cs = computeCosSin(alpha)

        points[2 * index + 0] = Vec3(radius1 * cs.cosine, 0, radius1 * cs.sine)
        points[2 * index + 1] = Vec3(radius2 * cs.cosine, height, radius2 * cs.sine)

        alpha += deltaAlpha
    }

    let hull = createHull(points, maxVertexCount: Int32(pointCount))
    b3Assert(hull == nil || hull!.vertexCount == Int32(pointCount))
    b3Assert(hull == nil || hull!.halfEdgeCount == 6 * slices)
    b3Assert(hull == nil || hull!.faceCount == slices + 2)

    return hull
}

/// Create a rock shaped hull from a Fibonacci lattice.
public func createRock(_ radius: Float) -> Hull? {
    let pointCount = 10

    // Golden ratio
    let phi = (1.0 + Float(5).squareRoot()) / 2.0

    // Fibonacci lattice
    var points = [Vec3](repeating: Vec3(), count: pointCount)

    // Azimuthal angle
    let theta = 2.0 * B3.pi / phi

    var cs = CosSin(cosine: 1, sine: 0)
    let deltaCS = computeCosSin(theta)

    for i in 0..<pointCount {
        // Z coordinate
        let z = 1 - (2.0 * Float(i) + 1.0) / Float(pointCount)
        // Radius in xy-plane
        let radiusXY = (1 - z * z).squareRoot()

        points[i].x = radius * radiusXY * cs.cosine
        points[i].y = radius * radiusXY * cs.sine
        points[i].z = radius * z

        let cs0 = cs
        cs.cosine = deltaCS.cosine * cs0.cosine - deltaCS.sine * cs0.sine
        cs.sine = deltaCS.sine * cs0.cosine + deltaCS.cosine * cs0.sine
    }

    return createHull(points, maxVertexCount: Int32(pointCount))
}

private func nextComplexHullRandom(_ state: inout UInt32) -> UInt32 {
    state = state &* 2_147_001_325 &+ 715_136_305
    return state
}

private func complexHullRandomFloat(_ state: inout UInt32) -> Float {
    Float(nextComplexHullRandom(&state) & 0xffff) / 65535.0 - 0.5
}

private func complexHullRandomDirection(_ state: inout UInt32) -> Vec3 {
    var point = Vec3()
    var lengthSquared: Float = 0
    repeat {
        point.x = complexHullRandomFloat(&state)
        point.y = complexHullRandomFloat(&state)
        point.z = complexHullRandomFloat(&state)
        lengthSquared = dot(point, point)
    } while lengthSquared > 0.25

    return normalize(point)
}

/// Hull of 32 pseudo random points on a sphere. From the convex pile hull in PEEL.
public func createComplexHull(_ radius: Float) -> Hull? {
    let pointCount = 32

    var points = [Vec3](repeating: Vec3(), count: pointCount)
    var state: UInt32 = 42
    for i in 0..<pointCount {
        points[i] = radius * complexHullRandomDirection(&state)
    }

    return createHull(points, maxVertexCount: Int32(pointCount))
}

// MARK: - Clone and transform

/// Clone and transform a hull. Supports non-uniform and mirroring scale.
public func cloneAndTransformHull(_ original: Hull, _ transform: Transform, _ scale: Vec3) -> Hull? {
    guard isValidHull(original) else {
        return nil
    }

    guard let hull = original.clone() else {
        return nil
    }

    let safeScale = Natsel.safeScale(scale)

    let edges = hull.edges!
    let faces = hull.faces!
    let faceCount = Int(hull.faceCount)
    let vertexCount = Int(hull.vertexCount)
    let points = hull.points!

    if safeScale.x * safeScale.y * safeScale.z < 0 {
        // Reflected: reverse edge winding for each face.
        for i in 0..<faceCount {
            let face = faces + i

            let startEdgeIndex = Int32(face.pointee.edge)
            var currentEdgeIndex = startEdgeIndex
            var prevEdgeIndex: Int32 = B3.nullIndex

            repeat {
                let edge = edges + Int(currentEdgeIndex)

                if Int32(edge.pointee.next) == startEdgeIndex {
                    prevEdgeIndex = currentEdgeIndex
                    break
                }

                currentEdgeIndex = Int32(edge.pointee.next)
            } while currentEdgeIndex != startEdgeIndex

            b3Assert(prevEdgeIndex != B3.nullIndex)

            currentEdgeIndex = startEdgeIndex

            repeat {
                let edge = edges + Int(currentEdgeIndex)
                let nextIndex = Int32(edge.pointee.next)
                edge.pointee.next = UInt8(truncatingIfNeeded: prevEdgeIndex)

                if currentEdgeIndex < Int32(edge.pointee.twin) {
                    let twin = edges + Int(edge.pointee.twin)
                    swap(&edge.pointee.origin, &twin.pointee.origin)
                }

                prevEdgeIndex = currentEdgeIndex
                currentEdgeIndex = nextIndex
            } while currentEdgeIndex != startEdgeIndex
        }

        let vertices = hull.vertices!
        for i in 0..<vertexCount {
            let vertex = vertices + i
            let edge = edges + Int(vertex.pointee.edge)
            vertex.pointee.edge = edge.pointee.twin
        }
    }

    let matrix = makeMatrixFromQuat(transform.q)

    let soaVertexCount = Int(hullSoaStride(Int32(vertexCount)))
    let vx = hull.soaVertices!
    let vy = vx + soaVertexCount
    let vz = vy + soaVertexCount

    for i in 0..<vertexCount {
        let p = mulMV(matrix, safeScale * points[i]) + transform.p
        points[i] = p
        vx[i] = p.x
        vy[i] = p.y
        vz[i] = p.z
    }

    for i in vertexCount..<soaVertexCount {
        vx[i] = vx[0]
        vy[i] = vy[0]
        vz[i] = vz[0]
    }

    let planes = hull.planes!
    let soaNormalCount = Int(hullSoaStride(Int32(faceCount)))
    let nx = hull.soaNormals!
    let ny = nx + soaNormalCount
    let nz = ny + soaNormalCount

    for i in 0..<faceCount {
        var count: Float = 0
        var centroid = Vec3()
        var normal = Vec3()

        let face = faces + i
        let startEdgeIndex = Int32(face.pointee.edge)
        var currentEdgeIndex = startEdgeIndex

        let startEdge = edges + Int(currentEdgeIndex)
        b3Assert(Int(startEdge.pointee.face) == i)
        b3Assert(Int(startEdge.pointee.origin) < vertexCount)

        let origin = points[Int(startEdge.pointee.origin)]

        repeat {
            let edge = edges + Int(currentEdgeIndex)
            let twin = edges + Int(edge.pointee.twin)
            b3Assert(Int(twin.pointee.twin) == Int(currentEdgeIndex))

            let v1 = points[Int(edge.pointee.origin)] - origin
            let v2 = points[Int(twin.pointee.origin)] - origin

            count += 1
            centroid = centroid + v1
            normal.x += (v1.y - v2.y) * (v1.z + v2.z)
            normal.y += (v1.z - v2.z) * (v1.x + v2.x)
            normal.z += (v1.x - v2.x) * (v1.y + v2.y)

            currentEdgeIndex = Int32(edge.pointee.next)
        } while currentEdgeIndex != startEdgeIndex

        b3Assert(count > 0)
        centroid = (1 / count) * centroid
        centroid = centroid + origin

        let area = length(normal)
        b3Assert(area > 0)
        normal = (1 / area) * normal

        planes[i] = makePlaneFromNormalAndPoint(normal, centroid)

        nx[i] = normal.x
        ny[i] = normal.y
        nz[i] = normal.z
    }

    for i in faceCount..<soaNormalCount {
        nx[i] = 0
        ny[i] = 0
        nz[i] = 0
    }

    updateHullEdgeCosines(hull)

    updateHullBounds(hull)
    let success = updateHullBulkProperties(hull)
    if success == false {
        return nil
    }

    // Must ensure the hash is 0 so it doesn't contribute to itself.
    computeHullHash(hull)

    b3Validate(isValidHull(hull))

    return hull
}
