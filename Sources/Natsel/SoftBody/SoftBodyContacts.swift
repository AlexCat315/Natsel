internal import Dispatch

// Soft body self collision and soft body vs soft body contact.
//
// Once per step, contacts are found between vertices and triangles (or vertices and vertices for
// bodies without faces, such as ropes) that are within the collision thickness plus the distance
// both sides can travel in the step. A uniform grid of triangle bounds finds the candidates. Each
// substep projects the contacts with XPBD: a vertex keeps `thickness` from the triangle on the side
// it started on, with the correction shared by the vertex and the triangle's vertices through the
// barycentric weights, and friction removes relative tangential motion up to mu times the
// correction. Self contacts skip a vertex's own triangles and those of its edge neighbors, and pairs
// already closer than the thickness in the rest pose.
//
// Soft bodies that may touch are simulated together, substep by substep, on one thread, so the
// result does not depend on the worker count.

/// A vertex against a triangle, or against another vertex when the triangle slots repeat it.
struct SoftContact: BitwiseCopyable {
    /// Group local body slots and vertex indices.
    var vertexBody: Int32 = 0
    var vertex: Int32 = 0
    var triangleBody: Int32 = 0
    var a: Int32 = 0
    var b: Int32 = 0
    var c: Int32 = 0
    /// Side of the triangle the vertex started on (+1 or -1); zero marks a vertex-vertex contact.
    var side: Float = 0
    /// Face index in the triangle body, for the contact history.
    var face: Int32 = 0
    var thickness: Float = 0
    var friction: Float = 0
}

/// Positions and inverse masses of one body during the contact projection.
struct SoftContactBody {
    var x: UnsafeMutableBufferPointer<Vec3>
    var x0: UnsafeMutableBufferPointer<Vec3>
    var w: UnsafeBufferPointer<Float>
}

/// Closest point on a triangle with barycentric coordinates (Ericson, Real-Time Collision
/// Detection 5.1.5). `interior` is true when the point projects inside the face.
@inline(__always)
func closestTriangleBarycentric(_ a: Vec3, _ b: Vec3, _ c: Vec3, _ p: Vec3) -> (u: Float, v: Float, w: Float, interior: Bool) {
    let ab = b - a
    let ac = c - a
    let ap = p - a
    let d1 = dot(ab, ap)
    let d2 = dot(ac, ap)
    if d1 <= 0 && d2 <= 0 { return (1, 0, 0, false) }
    let bp = p - b
    let d3 = dot(ab, bp)
    let d4 = dot(ac, bp)
    if d3 >= 0 && d4 <= d3 { return (0, 1, 0, false) }
    let vc = d1 * d4 - d3 * d2
    if vc <= 0 && d1 >= 0 && d3 <= 0 {
        let v = d1 / (d1 - d3)
        return (1 - v, v, 0, false)
    }
    let cp = p - c
    let d5 = dot(ab, cp)
    let d6 = dot(ac, cp)
    if d6 >= 0 && d5 <= d6 { return (0, 0, 1, false) }
    let vb = d5 * d2 - d1 * d6
    if vb <= 0 && d2 >= 0 && d6 <= 0 {
        let w = d2 / (d2 - d6)
        return (1 - w, 0, w, false)
    }
    let va = d3 * d6 - d5 * d4
    if va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return (0, 1 - w, w, false)
    }
    let denominator = 1 / (va + vb + vc)
    let v = vb * denominator
    let w = vc * denominator
    return (1 - v - w, v, w, true)
}

/// Exact cell keys with feature chains. Query order is independent of dictionary bucket order.
struct CellGrid {
    var cellSize: Float
    var entries: [(key: Int64, item: Int32)] = []
    private var heads: [Int64: Int] = [:]
    private var next: [Int] = []
    private var lastItem = Int32.min
    private var orderedItems = true

    init(cellSize: Float) { self.cellSize = cellSize }

    @inline(__always)
    func cell(_ p: Vec3) -> (Int64, Int64, Int64) {
        let inv = 1 / cellSize
        return (Int64((p.x * inv).rounded(.down)), Int64((p.y * inv).rounded(.down)), Int64((p.z * inv).rounded(.down)))
    }

    @inline(__always)
    static func key(_ x: Int64, _ y: Int64, _ z: Int64) -> Int64 {
        let mask: Int64 = (1 << 21) - 1
        return ((x & mask) << 42) | ((y & mask) << 21) | (z & mask)
    }

    mutating func insert(_ box: AABB, _ item: Int32) {
        if item < lastItem { orderedItems = false }
        lastItem = item
        let lower = cell(box.lowerBound)
        let upper = cell(box.upperBound)
        for x in lower.0...upper.0 {
            for y in lower.1...upper.1 {
                for z in lower.2...upper.2 { entries.append((Self.key(x, y, z), item)) }
            }
        }
    }

    mutating func finish() {
        // Features are normally inserted in ascending index order. A stable per-cell chain then
        // needs no global cell sort, and lookups avoid a binary search for every visited cell.
        if !orderedItems { entries.sort { $0.item < $1.item } }
        orderedItems = true
        lastItem = entries.last?.item ?? .min
        heads.removeAll(keepingCapacity: true)
        heads.reserveCapacity(entries.count / 4)
        next = [Int](repeating: -1, count: entries.count)
        for index in entries.indices.reversed() {
            next[index] = heads.updateValue(index, forKey: entries[index].key) ?? -1
        }
    }

    /// Items in every cell the box overlaps. An item spanning several of those cells is visited
    /// once per cell; callers deduplicate.
    func visit(_ box: AABB, _ body: (Int32) -> Void) {
        let lower = cell(box.lowerBound)
        let upper = cell(box.upperBound)
        for x in lower.0...upper.0 {
            for y in lower.1...upper.1 {
                for z in lower.2...upper.2 { visitKey(Self.key(x, y, z), body) }
            }
        }
    }

    /// Items in the cell holding `p`.
    func visit(_ p: Vec3, _ body: (Int32) -> Void) {
        let c = cell(p)
        visitKey(Self.key(c.0, c.1, c.2), body)
    }

    private func visitKey(_ key: Int64, _ body: (Int32) -> Void) {
        var index = heads[key] ?? -1
        while index >= 0 {
            body(entries[index].item)
            index = next[index]
        }
    }
}

extension SoftBodyRecord {
    /// Collision thickness for self and soft-soft contact.
    var contactThickness: Float {
        if collisionThickness > 0 { return collisionThickness }
        return max(2 * vertexRadius, 0.25 * averageEdgeLength)
    }

    /// Edge neighbors in compressed rows, built on first use.
    mutating func buildNeighbors() {
        guard neighborStarts.isEmpty else { return }
        let n = vertexCount
        var lists = [[UInt32]](repeating: [], count: n)
        for edge in settings.edges {
            lists[Int(edge.vertex.x)].append(edge.vertex.y)
            lists[Int(edge.vertex.y)].append(edge.vertex.x)
        }
        neighborStarts.reserveCapacity(n + 1)
        for list in lists {
            neighborStarts.append(Int32(neighbors.count))
            neighbors.append(contentsOf: list.sorted())
        }
        neighborStarts.append(Int32(neighbors.count))
    }

    /// For self contact: the faces each vertex ignores and the edges each edge ignores, found once
    /// in the rest pose. A vertex ignores its own faces, its edge neighbors' faces, and faces closer
    /// than the exclusion distance at rest (the surface's own neighborhood, not a fold); an edge
    /// likewise. Detection then rejects these candidates with a binary search.
    mutating func buildSelfExclusions() {
        let exclusion = max(contactThickness, 2 * averageEdgeLength)
        guard exclusion != selfExclusionDistance || excludedFaceStarts.isEmpty else { return }
        selfExclusionDistance = exclusion
        buildNeighbors()
        buildMeshEdges()
        let rest = settings.vertices.map { $0.position }
        let faces = settings.faces
        let pad = Vec3(exclusion, exclusion, exclusion)
        let cell = max(1.5 * averageEdgeLength, exclusion, 1.0e-3)

        excludedFaceStarts.removeAll()
        excludedFaces.removeAll()
        if faces.isEmpty == false {
            // Unpadded features in the grid, padded queries: each feature sits in few cells.
            var grid = CellGrid(cellSize: cell)
            for (f, face) in faces.enumerated() {
                let a = rest[Int(face.vertex.x)]
                let b = rest[Int(face.vertex.y)]
                let c = rest[Int(face.vertex.z)]
                grid.insert(AABB(minVec3(a, minVec3(b, c)), maxVec3(a, maxVec3(b, c))), Int32(f))
            }
            grid.finish()
            var list: [Int32] = []
            var stamp = [Int32](repeating: -1, count: faces.count)
            for v in 0..<vertexCount {
                list.removeAll(keepingCapacity: true)
                let r = rest[v]
                let vertex = Int32(v)
                grid.visit(AABB(r - pad, r + pad)) { f in
                    if stamp[Int(f)] == vertex { return }
                    stamp[Int(f)] = vertex
                    let face = faces[Int(f)]
                    let ia = Int32(face.vertex.x)
                    let ib = Int32(face.vertex.y)
                    let ic = Int32(face.vertex.z)
                    var excluded = areNeighbors(vertex, ia) || areNeighbors(vertex, ib) || areNeighbors(vertex, ic)
                    if excluded == false {
                        let ra = rest[Int(ia)]
                        let rb = rest[Int(ib)]
                        let rc = rest[Int(ic)]
                        let bary = closestTriangleBarycentric(ra, rb, rc, r)
                        excluded = lengthSquared(r - (bary.u * ra + bary.v * rb + bary.w * rc)) < exclusion * exclusion
                    }
                    if excluded { list.append(f) }
                }
                excludedFaceStarts.append(Int32(excludedFaces.count))
                excludedFaces.append(contentsOf: list.sorted())
            }
            excludedFaceStarts.append(Int32(excludedFaces.count))
        }

        excludedEdgeStarts.removeAll()
        excludedEdges.removeAll()
        var grid = CellGrid(cellSize: cell)
        for (e, edge) in meshEdges.enumerated() {
            let a = rest[Int(edge.x)]
            let b = rest[Int(edge.y)]
            grid.insert(AABB(minVec3(a, b), maxVec3(a, b)), Int32(e))
        }
        grid.finish()
        var list: [Int32] = []
        var stamp = [Int32](repeating: -1, count: meshEdges.count)
        for (e, edge) in meshEdges.enumerated() {
            list.removeAll(keepingCapacity: true)
            let i0 = Int32(edge.x)
            let i1 = Int32(edge.y)
            let a = rest[Int(i0)]
            let b = rest[Int(i1)]
            let box = AABB(minVec3(a, b) - pad, maxVec3(a, b) + pad)
            grid.visit(box) { f in
                if stamp[Int(f)] == Int32(e) { return }
                stamp[Int(f)] = Int32(e)
                let other = meshEdges[Int(f)]
                let j0 = Int32(other.x)
                let j1 = Int32(other.y)
                let c = rest[Int(j0)]
                let d = rest[Int(j1)]
                guard aabbOverlaps(box, AABB(minVec3(c, d), maxVec3(c, d))) else { return }
                var excluded = areNeighbors(i0, j0) || areNeighbors(i0, j1) || areNeighbors(i1, j0) || areNeighbors(i1, j1)
                if excluded == false {
                    let result = segmentDistance(a, b, c, d)
                    excluded = lengthSquared(result.point1 - result.point2) < exclusion * exclusion
                }
                if excluded { list.append(f) }
            }
            excludedEdgeStarts.append(Int32(excludedEdges.count))
            excludedEdges.append(contentsOf: list.sorted())
        }
        excludedEdgeStarts.append(Int32(excludedEdges.count))
    }

    @inline(__always)
    private static func contains(_ list: [Int32], _ start: Int32, _ end: Int32, _ value: Int32) -> Bool {
        var low = Int(start)
        var high = Int(end)
        while low < high {
            let mid = (low + high) / 2
            if list[mid] < value { low = mid + 1 } else { high = mid }
        }
        return low < Int(end) && list[low] == value
    }

    @inline(__always)
    func excludesFace(_ vertex: Int32, _ face: Int32) -> Bool {
        Self.contains(excludedFaces, excludedFaceStarts[Int(vertex)], excludedFaceStarts[Int(vertex) + 1], face)
    }

    @inline(__always)
    func excludesEdge(_ edge: Int32, _ other: Int32) -> Bool {
        Self.contains(excludedEdges, excludedEdgeStarts[Int(edge)], excludedEdgeStarts[Int(edge) + 1], other)
    }

    @inline(__always)
    func areNeighbors(_ i: Int32, _ j: Int32) -> Bool {
        if i == j { return true }
        let start = Int(neighborStarts[Int(i)])
        let end = Int(neighborStarts[Int(i) + 1])
        for k in start..<end where neighbors[k] == UInt32(j) { return true }
        return false
    }
}

@inline(__always)
private func aabbContainsPoint(_ box: AABB, _ p: Vec3) -> Bool {
    p.x >= box.lowerBound.x && p.y >= box.lowerBound.y && p.z >= box.lowerBound.z
        && p.x <= box.upperBound.x && p.y <= box.upperBound.y && p.z <= box.upperBound.z
}

/// Most triangles kept per vertex and body: the closest ones. A vertex resting on a surface is
/// near several triangles; the nearest few already hold it, and more only cost iterations.
private let maxTrianglesPerVertex = 4

extension SoftBodyRecord {
    /// Vertices on the surface: all of them for bodies without faces.
    /// Also decides whether the faces form a closed, consistently wound surface: then every
    /// directed edge has its reverse in another face.
    mutating func buildSurfaceVertices() {
        guard surfaceVertices.isEmpty else { return }
        if settings.faces.isEmpty {
            surfaceVertices = (0..<Int32(vertexCount)).map { $0 }
            return
        }
        var used = [Bool](repeating: false, count: vertexCount)
        var directed = Set<UInt64>()
        var duplicate = false
        for face in settings.faces {
            used[Int(face.vertex.x)] = true
            used[Int(face.vertex.y)] = true
            used[Int(face.vertex.z)] = true
            for (a, b) in [(face.vertex.x, face.vertex.y), (face.vertex.y, face.vertex.z), (face.vertex.z, face.vertex.x)] {
                if directed.insert(UInt64(a) << 32 | UInt64(b)).inserted == false { duplicate = true }
            }
        }
        surfaceVertices = (0..<Int32(vertexCount)).filter { used[Int($0)] }
        closedSurface = duplicate == false && directed.allSatisfy { directed.contains(($0 & 0xffff_ffff) << 32 | $0 >> 32) }
    }
}

/// Per body data shared by every pair during detection.
private struct DetectionBody {
    /// How far each vertex can move in the step.
    var vertexMargins: [Float] = []
    /// Where each vertex can be during the step: from its position along its velocity, padded by
    /// what gravity and velocity changes can add.
    var vertexBoxes: [AABB] = []
    /// Bounds of each face padded by its vertices' margins and the largest thickness into it.
    var faceBoxes: [AABB] = []
    var faceMargins: [Float] = []
    var grid: CellGrid?
    var faceTree: SoftContactTree?
    /// +1 or -1 for a closed surface wound outward or inward, 0 for an open one.
    var closedSide: Float = 0
    /// Edge bounds padded like the faces, and their grid.
    var edgeBoxes: [AABB] = []
    var edgeMargins: [Float] = []
    var edgeTree: SoftContactTree?
}

extension World {
    /// Find vertex-triangle and vertex-vertex contacts within a group of soft bodies.
    ///
    /// A vertex and a feature can meet when their distance is below the thickness plus how far
    /// each can move: its own fastest speed and what gravity adds over the step (a tighter bound
    /// from relative velocities misses contacts when one body stops on the ground mid step). Each
    /// feature is padded by its own motion in the grid and each vertex queries the cells its own
    /// motion box covers, so slow vertices test few candidates.
    func detectSoftContacts(
        _ records: UnsafeMutablePointer<SoftBodyRecord>, _ group: [Int], _ dt: Float, _ contacts: inout [SoftContact],
        _ edgeContacts: inout [SoftEdgeContact], parallel: Bool = false, detectionWorkers: Int = 1
    ) {
        contacts.removeAll(keepingCapacity: true)
        edgeContacts.removeAll(keepingCapacity: true)

        struct Pair {
            var slotP: Int
            var slotT: Int
            var thickness: Float
        }
        var pairs: [Pair] = []
        var bodyMargins = [Float](repeating: 0, count: group.count)
        for (slot, index) in group.enumerated() {
            let gain = length(gravity) * abs(records[index].gravityScale) * dt
            bodyMargins[slot] = (records[index].maxSpeed + gain) * dt
        }
        var targetThickness = [Float](repeating: 0, count: group.count)
        for (slotP, indexP) in group.enumerated() {
            if records[indexP].enableSelfCollision {
                records[indexP].buildNeighbors()
                if records[indexP].enableEdgeCollision || records[indexP].settings.faces.isEmpty == false {
                    records[indexP].buildSelfExclusions()
                }
            }
            records[indexP].buildSurfaceVertices()
            for (slotT, indexT) in group.enumerated() {
                let p = records + indexP
                let t = records + indexT
                let pFaces = p.pointee.settings.faces.isEmpty == false
                let tFaces = t.pointee.settings.faces.isEmpty == false
                if slotP == slotT {
                    guard p.pointee.enableSelfCollision else { continue }
                } else {
                    guard p.pointee.collideWithSoftBodies, t.pointee.collideWithSoftBodies,
                        shouldShapesCollide(p.pointee.filter, t.pointee.filter)
                    else { continue }
                    // Vertex-vertex pairs between face-less bodies are found once.
                    if pFaces == false && tFaces == false && slotT < slotP { continue }
                    // Faces of P are tested against vertices of T in the mirrored pass.
                    if tFaces == false && pFaces { continue }
                }
                let thickness = max(p.pointee.contactThickness, t.pointee.contactThickness)
                let reach = thickness + bodyMargins[slotP] + bodyMargins[slotT]
                guard aabbOverlaps(aabbInflate(p.pointee.bounds, reach), t.pointee.bounds) else { continue }
                pairs.append(Pair(slotP: slotP, slotT: slotT, thickness: thickness))
                targetThickness[slotT] = max(targetThickness[slotT], thickness)
            }
        }
        // Edge-edge pairs, unordered: an edge passing between two vertices of the other surface
        // touches no vertex or face there.
        var edgePairs: [Pair] = []
        for (slotP, indexP) in group.enumerated() {
            for slotT in slotP..<group.count {
                let p = records + indexP
                let t = records + group[slotT]
                if slotP == slotT {
                    guard p.pointee.enableSelfCollision else { continue }
                } else {
                    guard p.pointee.collideWithSoftBodies, t.pointee.collideWithSoftBodies,
                        shouldShapesCollide(p.pointee.filter, t.pointee.filter)
                    else { continue }
                }
                guard p.pointee.enableEdgeCollision, t.pointee.enableEdgeCollision else { continue }
                p.pointee.buildMeshEdges()
                t.pointee.buildMeshEdges()
                guard p.pointee.meshEdges.isEmpty == false, t.pointee.meshEdges.isEmpty == false else { continue }
                let thickness = max(p.pointee.contactThickness, t.pointee.contactThickness)
                let reach = thickness + bodyMargins[slotP] + bodyMargins[slotT]
                guard aabbOverlaps(aabbInflate(p.pointee.bounds, reach), t.pointee.bounds) else { continue }
                edgePairs.append(Pair(slotP: slotP, slotT: slotT, thickness: thickness))
            }
        }
        guard pairs.isEmpty == false || edgePairs.isEmpty == false else {
            // Nothing in reach: forget the old sides.
            for index in group {
                records[index].faceSides.removeAll(keepingCapacity: true)
                }
            return
        }

        // Margins of every body in a pair, and a grid per target.
        var data = [DetectionBody](repeating: DetectionBody(), count: group.count)
        var needed = [Bool](repeating: false, count: group.count)
        for pair in pairs + edgePairs {
            needed[pair.slotP] = true
            needed[pair.slotT] = true
        }
        for (slot, index) in group.enumerated() where needed[slot] {
            let record = records + index
            let gain = length(gravity) * abs(record.pointee.gravityScale) * dt * dt
            data[slot].vertexMargins = record.pointee.velocities.map { length($0) * dt + gain }
            let x = record.pointee.positions
            data[slot].vertexBoxes = record.pointee.velocities.enumerated().map { i, v in
                let end = mulAdd(x[i], dt, v)
                // A quarter of the step's travel in any direction covers turns and pushes.
                let slack = gain + 0.25 * length(v) * dt
                return aabbInflate(AABB(minVec3(x[i], end), maxVec3(x[i], end)), slack)
            }
        }
        for pair in pairs where data[pair.slotT].grid == nil && data[pair.slotT].faceTree == nil {
            let slot = pair.slotT
            let t = records + group[slot]
            let x = t.pointee.positions
            let margins = data[slot].vertexMargins
            let thickness = targetThickness[slot]
            if t.pointee.settings.faces.isEmpty {
                var grid = CellGrid(cellSize: max(t.pointee.averageEdgeLength, thickness, 1.0e-3))
                let sweeps = data[slot].vertexBoxes
                _ = margins
                for j in 0..<x.count { grid.insert(aabbInflate(sweeps[j], thickness), Int32(j)) }
                grid.finish()
                data[slot].grid = grid
            } else {
                let faces = t.pointee.settings.faces
                var boxes = [AABB](repeating: AABB(), count: faces.count)
                var faceMargins = [Float](repeating: 0, count: faces.count)
                for (f, face) in faces.enumerated() {
                    let ia = Int(face.vertex.x)
                    let ib = Int(face.vertex.y)
                    let ic = Int(face.vertex.z)
                    let margin = max(margins[ia], max(margins[ib], margins[ic]))
                    let sweeps = data[slot].vertexBoxes
                    faceMargins[f] = margin
                    boxes[f] = aabbInflate(aabbUnion(sweeps[ia], aabbUnion(sweeps[ib], sweeps[ic])), thickness)
                }
                let rest = t.pointee.settings.vertices
                t.pointee.contactFaceTree.refit(boxes) {
                    faces.map { face in
                        let a = rest[Int(face.vertex.x)].position, b = rest[Int(face.vertex.y)].position, c = rest[Int(face.vertex.z)].position
                        return AABB(minVec3(a, minVec3(b, c)), maxVec3(a, maxVec3(b, c)))
                    }
                }
                data[slot].faceTree = t.pointee.contactFaceTree
                data[slot].faceBoxes = boxes
                data[slot].faceMargins = faceMargins
                if t.pointee.closedSurface {
                    let origin = x[0]
                    var sixVolume: Float = 0
                    for face in faces {
                        sixVolume += dot(x[Int(face.vertex.x)] - origin,
                                         cross(x[Int(face.vertex.y)] - origin, x[Int(face.vertex.z)] - origin))
                    }
                    data[slot].closedSide = sixVolume >= 0 ? 1 : -1
                }
            }
        }

        // Edge grids of the edge pair targets.
        for pair in edgePairs where data[pair.slotT].edgeTree == nil {
            let slot = pair.slotT
            let t = records + group[slot]
            let x = t.pointee.positions
            let margins = data[slot].vertexMargins
            let thickness = edgePairs.filter { $0.slotT == slot }.reduce(Float(0)) { max($0, $1.thickness) }
            var boxes = [AABB](repeating: AABB(), count: t.pointee.meshEdges.count)
            var edgeMargins = [Float](repeating: 0, count: t.pointee.meshEdges.count)
            for (e, edge) in t.pointee.meshEdges.enumerated() {
                let margin = max(margins[Int(edge.x)], margins[Int(edge.y)])
                edgeMargins[e] = margin
                boxes[e] = aabbInflate(aabbUnion(data[slot].vertexBoxes[Int(edge.x)], data[slot].vertexBoxes[Int(edge.y)]),
                                       thickness)
            }
            let rest = t.pointee.settings.vertices
            let edges = t.pointee.meshEdges
            t.pointee.contactEdgeTree.refit(boxes) {
                edges.map { edge in
                    let a = rest[Int(edge.x)].position, b = rest[Int(edge.y)].position
                    return AABB(minVec3(a, b), maxVec3(a, b))
                }
            }
            data[slot].edgeTree = t.pointee.contactEdgeTree
            data[slot].edgeBoxes = boxes
            data[slot].edgeMargins = edgeMargins
        }

        // Each pair fills its own list; the lists are joined in pair order, so the result does not
        // depend on the threads.
        let shared = data
        nonisolated(unsafe) let records = records
        var lists = [[SoftContact]](repeating: [], count: pairs.count)
        func detect(_ k: Int, _ list: inout [SoftContact], sourceRange: Range<Int>? = nil) {
            let pair = pairs[k]
            let p = records + group[pair.slotP]
            let t = records + group[pair.slotT]
            let friction = (p.pointee.friction * t.pointee.friction).squareRoot()
            if t.pointee.settings.faces.isEmpty {
                findVertexVertexContacts(p, t, Int32(pair.slotP), Int32(pair.slotT), pair.thickness, friction,
                                         pair.slotP == pair.slotT, shared[pair.slotP], shared[pair.slotT], &list, sourceRange: sourceRange)
            } else {
                findVertexTriangleContacts(p, t, Int32(pair.slotP), Int32(pair.slotT), pair.thickness, friction,
                                           pair.slotP == pair.slotT, shared[pair.slotP], shared[pair.slotT], &list, sourceRange: sourceRange)
            }
        }
        if parallel && pairs.count > 1 {
            lists.withUnsafeMutableBufferPointer { buffer in
                nonisolated(unsafe) let buffer = buffer
                DispatchQueue.concurrentPerform(iterations: buffer.count) { k in detect(k, &buffer[k]) }
            }
        } else if pairs.count == 1 && detectionWorkers > 1 {
            let count = records[group[pairs[0].slotP]].surfaceVertices.count
            let chunks = min(detectionWorkers, max(1, count / 256))
            var parts = [[SoftContact]](repeating: [], count: chunks)
            parts.withUnsafeMutableBufferPointer { buffer in
                nonisolated(unsafe) let buffer = buffer
                DispatchQueue.concurrentPerform(iterations: chunks) { k in
                    detect(0, &buffer[k], sourceRange: (count * k / chunks)..<(count * (k + 1) / chunks))
                }
            }
            lists[0] = parts.flatMap { $0 }
        } else {
            for k in 0..<pairs.count { detect(k, &lists[k]) }
        }
        for list in lists { contacts.append(contentsOf: list) }

        var edgeLists = [[SoftEdgeContact]](repeating: [], count: edgePairs.count)
        func detectEdges(_ k: Int, _ list: inout [SoftEdgeContact], sourceRange: Range<Int>? = nil) {
            let pair = edgePairs[k]
            let p = records + group[pair.slotP]
            let t = records + group[pair.slotT]
            let friction = (p.pointee.friction * t.pointee.friction).squareRoot()
            findEdgeEdgeContacts(p, t, Int32(pair.slotP), Int32(pair.slotT), pair.thickness, friction,
                                 pair.slotP == pair.slotT, shared[pair.slotP], shared[pair.slotT], &list, sourceRange: sourceRange)
        }
        if parallel && edgePairs.count > 1 {
            edgeLists.withUnsafeMutableBufferPointer { buffer in
                nonisolated(unsafe) let buffer = buffer
                DispatchQueue.concurrentPerform(iterations: buffer.count) { k in detectEdges(k, &buffer[k]) }
            }
        } else if edgePairs.count == 1 && detectionWorkers > 1 {
            let count = records[group[edgePairs[0].slotP]].meshEdges.count
            let chunks = min(detectionWorkers, max(1, count / 256))
            var parts = [[SoftEdgeContact]](repeating: [], count: chunks)
            parts.withUnsafeMutableBufferPointer { buffer in
                nonisolated(unsafe) let buffer = buffer
                DispatchQueue.concurrentPerform(iterations: chunks) { k in
                    detectEdges(0, &buffer[k], sourceRange: (count * k / chunks)..<(count * (k + 1) / chunks))
                }
            }
            edgeLists[0] = parts.flatMap { $0 }
        } else {
            for k in 0..<edgePairs.count { detectEdges(k, &edgeLists[k]) }
        }
        for list in edgeLists { edgeContacts.append(contentsOf: list) }

        // Remember the sides for the next step.
        for index in group {
            records[index].faceSides.removeAll(keepingCapacity: true)
        }
        for contact in contacts where contact.side != 0 {
            let p = records + group[Int(contact.vertexBody)]
            let t = records + group[Int(contact.triangleBody)]
            guard t.pointee.closedSurface == false || t.pointee.settings.faces.isEmpty else { continue }
            p.pointee.faceSides[softContactKey(t.pointee.id, contact.vertex, contact.face)] = contact.side
        }

    }
}

private func findVertexTriangleContacts(
    _ p: UnsafeMutablePointer<SoftBodyRecord>, _ t: UnsafeMutablePointer<SoftBodyRecord>, _ slotP: Int32,
    _ slotT: Int32, _ thickness: Float, _ friction: Float, _ selfPair: Bool, _ pData: DetectionBody,
    _ tData: DetectionBody, _ contacts: inout [SoftContact], sourceRange: Range<Int>? = nil
) {
    guard let tree = tData.faceTree else { return }
    let faces = t.pointee.settings.faces
    let tx = t.pointee.positions
    let tw = t.pointee.invMasses
    let px = p.pointee.positions
    let pw = p.pointee.invMasses
    let excludedStarts = p.pointee.excludedFaceStarts
    let excludedFaces = p.pointee.excludedFaces
    let rest = p.pointee.settings.vertices
    let exclusion = max(thickness, 2 * p.pointee.averageEdgeLength)
    let faceMargins = tData.faceMargins
    let vertexMargins = pData.vertexMargins
    let targetBounds = aabbInflate(t.pointee.bounds, thickness + (faceMargins.max() ?? 0))

    // Faces already tested for the current vertex.
    var stamp = [Int32](repeating: -1, count: faces.count)
    // The closest few triangles of the current vertex: (distance squared, face, side).
    var best: [(Float, Int32, Float)] = []
    best.reserveCapacity(maxTrianglesPerVertex + 1)

    let vertices = p.pointee.surfaceVertices
    for source in sourceRange ?? vertices.indices {
        let vertex32 = vertices[source]
        let vertex = Int(vertex32)
        let x = px[vertex]
        let margin = vertexMargins[vertex]
        let query = pData.vertexBoxes[vertex]
        guard aabbOverlaps(query, targetBounds) else { continue }
        best.removeAll(keepingCapacity: true)
        // Pre-stamp the rest neighborhood once instead of binary-searching its exclusion row
        // for each spatial candidate. The same stamp also deduplicates features across cells.
        if selfPair {
            for offset in Int(excludedStarts[vertex])..<Int(excludedStarts[vertex + 1]) {
                stamp[Int(excludedFaces[offset])] = vertex32
            }
        }
        tree.visit(query, referenceExclusion: selfPair ? (rest[vertex].position, exclusion * exclusion) : nil) { f in
            if stamp[Int(f)] == vertex32 { return }
            let face = faces[Int(f)]
            guard !face.isDegenerate else { return }
            let ia = Int32(face.vertex.x)
            let ib = Int32(face.vertex.y)
            let ic = Int32(face.vertex.z)
            // Static against static never moves.
            if pw[vertex] == 0 && tw[Int(ia)] == 0 && tw[Int(ib)] == 0 && tw[Int(ic)] == 0 { return }
            let a = tx[Int(ia)]
            let b = tx[Int(ib)]
            let c = tx[Int(ic)]
            let bary = closestTriangleBarycentric(a, b, c, x)
            let d = x - (bary.u * a + bary.v * b + bary.w * c)
            let distanceSquared = lengthSquared(d)
            let reach = thickness + margin + faceMargins[Int(f)]
            guard distanceSquared < reach * reach else { return }
            if best.count == maxTrianglesPerVertex {
                let worst = best[maxTrianglesPerVertex - 1]
                guard distanceSquared < worst.0 || (distanceSquared == worst.0 && f < worst.1) else { return }
            }
            // The outside of a closed surface is always the valid side. On an open surface the side
            // is the one the vertex is on now; a vertex can legitimately pass round a free border to
            // the other side. Only when it rests too close to tell does a pair in contact last step
            // keep its side, since a pair squeezed slightly inside the thickness can read the wrong
            // one and be pushed through.
            var side = tData.closedSide
            if side == 0 {
                let normal = cross(b - a, c - a)
                let gap = dot(d, normal) / max(length(normal), 1.0e-12)
                side = gap >= 0 ? 1 : -1
                if abs(gap) < ambiguousGap * thickness,
                    let remembered = p.pointee.faceSides[softContactKey(t.pointee.id, vertex32, f)]
                {
                    side = remembered
                }
            }
            // Insert sorted by distance, then face index.
            var k = best.count
            best.append((distanceSquared, f, side))
            while k > 0 && (best[k - 1].0 > distanceSquared || (best[k - 1].0 == distanceSquared && best[k - 1].1 > f)) {
                best.swapAt(k, k - 1)
                k -= 1
            }
            if best.count > maxTrianglesPerVertex { best.removeLast() }
        }
        for (_, f, side) in best {
            let face = faces[Int(f)]
            var contact = SoftContact()
            contact.vertexBody = slotP
            contact.vertex = vertex32
            contact.triangleBody = slotT
            contact.a = Int32(face.vertex.x)
            contact.b = Int32(face.vertex.y)
            contact.c = Int32(face.vertex.z)
            contact.side = side
            contact.face = f
            contact.thickness = thickness
            contact.friction = friction
            contacts.append(contact)
        }
    }
}

private func findVertexVertexContacts(
    _ p: UnsafeMutablePointer<SoftBodyRecord>, _ t: UnsafeMutablePointer<SoftBodyRecord>, _ slotP: Int32,
    _ slotT: Int32, _ thickness: Float, _ friction: Float, _ selfPair: Bool, _ pData: DetectionBody,
    _ tData: DetectionBody, _ contacts: inout [SoftContact], sourceRange: Range<Int>? = nil
) {
    guard let grid = tData.grid else { return }
    let px = p.pointee.positions
    let pw = p.pointee.invMasses
    let tx = t.pointee.positions
    let tw = t.pointee.invMasses
    let rest = p.pointee.settings.vertices
    let restExclusion = max(thickness, 2 * p.pointee.averageEdgeLength)
    var stamp = [Int32](repeating: -1, count: tx.count)
    for i in sourceRange ?? px.indices {
        let x = px[i]
        let margin = pData.vertexMargins[i]
        let query = pData.vertexBoxes[i]
        grid.visit(query) { j in
            if stamp[Int(j)] == Int32(i) { return }
            stamp[Int(j)] = Int32(i)
            if selfPair {
                if j <= Int32(i) || p.pointee.areNeighbors(Int32(i), j) { return }
                if lengthSquared(rest[i].position - rest[Int(j)].position) < restExclusion * restExclusion { return }
            }
            if pw[i] == 0 && tw[Int(j)] == 0 { return }
            let reach = thickness + margin + tData.vertexMargins[Int(j)]
            guard lengthSquared(x - tx[Int(j)]) < reach * reach else { return }
            var contact = SoftContact()
            contact.vertexBody = slotP
            contact.vertex = Int32(i)
            contact.triangleBody = slotT
            contact.a = j
            contact.b = j
            contact.c = j
            contact.side = 0
            contact.thickness = thickness
            contact.friction = friction
            contacts.append(contact)
        }
    }
}

/// One Gauss-Seidel pass over the contacts. A contact stops approach motion at the thickness, but
/// a vertex that starts the substep inside the thickness is pushed out by at most `maxRecovery`:
/// projecting it out at once would turn the push into velocity and pump energy into piles.
func solveSoftContacts(
    _ contacts: UnsafeBufferPointer<SoftContact>, _ bodies: UnsafeBufferPointer<SoftContactBody>, maxRecovery: Float
) {
    @inline(__always)
    func target(_ thickness: Float, _ startSeparation: Float) -> Float {
        minFloat(thickness, startSeparation + maxRecovery)
    }

    for contact in contacts {
        let pb = bodies[Int(contact.vertexBody)]
        let tb = bodies[Int(contact.triangleBody)]
        let i = Int(contact.vertex)
        let wp = pb.w[i]

        if contact.side == 0 {
            // Vertex-vertex.
            let j = Int(contact.a)
            let wq = tb.w[j]
            let total = wp + wq
            guard total > 0 else { continue }
            let d = pb.x[i] - tb.x[j]
            let len = length(d)
            let c = len - target(contact.thickness, length(pb.x0[i] - tb.x0[j]))
            guard c < 0, len > 1.0e-9 else { continue }
            let n = (1 / len) * d
            var correction = (-c / total) * n
            // Friction against the relative tangential motion of this substep.
            let motion = (pb.x[i] - pb.x0[i]) - (tb.x[j] - tb.x0[j])
            let tangent = motion - dot(motion, n) * n
            let tangentLength = length(tangent)
            if tangentLength > 1.0e-9 {
                correction = correction - (minFloat(contact.friction * (-c) / tangentLength, 1) / total) * tangent
            }
            pb.x[i] = mulAdd(pb.x[i], wp, correction)
            tb.x[j] = mulSub(tb.x[j], wq, correction)
            continue
        }

        let ia = Int(contact.a)
        let ib = Int(contact.b)
        let ic = Int(contact.c)
        let a = tb.x[ia]
        let b = tb.x[ib]
        let c = tb.x[ic]
        let x = pb.x[i]
        let bary = closestTriangleBarycentric(a, b, c, x)
        let q = bary.u * a + bary.v * b + bary.w * c
        var n: Vec3
        var separation: Float
        if bary.interior {
            var len: Float = 0
            n = getLengthAndNormalize(&len, cross(b - a, c - a))
            guard len > 1.0e-12 else { continue }
            n = contact.side * n
            separation = dot(x - q, n)
        } else {
            // Beside the face: keep the distance to the closest edge or corner.
            let d = x - q
            separation = length(d)
            guard separation > 1.0e-9 else { continue }
            n = (1 / separation) * d
        }
        let start = pb.x0[i] - (bary.u * tb.x0[ia] + bary.v * tb.x0[ib] + bary.w * tb.x0[ic])
        let violation = separation - target(contact.thickness, dot(start, n))
        guard violation < 0 else { continue }

        let wa = tb.w[ia] * bary.u
        let wb = tb.w[ib] * bary.v
        let wc = tb.w[ic] * bary.w
        let total = wp + wa * bary.u + wb * bary.v + wc * bary.w
        guard total > 0 else { continue }
        var correction = (-violation / total) * n

        let motion = (x - pb.x0[i])
            - (bary.u * (a - tb.x0[ia]) + bary.v * (b - tb.x0[ib]) + bary.w * (c - tb.x0[ic]))
        let tangent = motion - dot(motion, n) * n
        let tangentLength = length(tangent)
        if tangentLength > 1.0e-9 {
            correction = correction - (minFloat(contact.friction * (-violation) / tangentLength, 1) / total) * tangent
        }

        pb.x[i] = mulAdd(x, wp, correction)
        tb.x[ia] = mulSub(tb.x[ia], wa, correction)
        tb.x[ib] = mulSub(tb.x[ib], wb, correction)
        tb.x[ic] = mulSub(tb.x[ic], wc, correction)
    }
}

// MARK: - Edge-edge

/// Two edges kept apart: the closest points of the segments stay `thickness` apart on the side
/// they started on, along the normal of the plane both edges span.
struct SoftEdgeContact: BitwiseCopyable {
    var bodyP: Int32 = 0
    var p0: Int32 = 0
    var p1: Int32 = 0
    var bodyT: Int32 = 0
    var t0: Int32 = 0
    var t1: Int32 = 0
    var side: Float = 1
    var thickness: Float = 0
    var friction: Float = 0
    /// Both bodies have faces: when the closest points reach an edge end the vertex-face contacts
    /// own the configuration and this contact rests.
    var interiorOnly = false
}

/// Closer than this fraction of the thickness along its normal, a pair's side cannot be read.
private let ambiguousGap: Float = 0.5

/// Key of a pair in a body's contact history: the other body's id and both feature indices.
@inline(__always)
func softContactKey(_ otherBody: Int32, _ feature: Int32, _ otherFeature: Int32) -> UInt64 {
    (UInt64(UInt16(truncatingIfNeeded: otherBody)) << 48) | (UInt64(UInt32(bitPattern: feature) & 0xff_ffff) << 24)
        | UInt64(UInt32(bitPattern: otherFeature) & 0xff_ffff)
}

/// Most edges kept per edge and body.
private let maxEdgesPerEdge = 4

/// Edges this close to parallel have no stable common normal; vertex contacts cover them.
private let parallelSine: Float = 0.05

private func findEdgeEdgeContacts(
    _ p: UnsafeMutablePointer<SoftBodyRecord>, _ t: UnsafeMutablePointer<SoftBodyRecord>, _ slotP: Int32,
    _ slotT: Int32, _ thickness: Float, _ friction: Float, _ selfPair: Bool, _ pData: DetectionBody,
    _ tData: DetectionBody, _ contacts: inout [SoftEdgeContact], sourceRange: Range<Int>? = nil
) {
    guard let tree = tData.edgeTree else { return }
    let pEdges = p.pointee.meshEdges
    let tEdges = t.pointee.meshEdges
    let px = p.pointee.positions
    let tx = t.pointee.positions
    let pw = p.pointee.invMasses
    let tw = t.pointee.invMasses
    let excludedStarts = p.pointee.excludedEdgeStarts
    let excludedEdges = p.pointee.excludedEdges
    let rest = p.pointee.settings.vertices
    let exclusion = max(thickness, 2 * p.pointee.averageEdgeLength)
    // Both with faces: an edge's end points are covered by the vertex-face contacts.
    let interiorOnly = p.pointee.settings.faces.isEmpty == false && t.pointee.settings.faces.isEmpty == false
    let edgeMargins = tData.edgeMargins
    var stamp = [Int32](repeating: -1, count: tEdges.count)
    let limit = maxEdgesPerEdge
    var best: [(Float, Int32, Float)] = []
    best.reserveCapacity(limit + 1)

    for e in sourceRange ?? pEdges.indices {
        let edge = pEdges[e]
        let i0 = Int(edge.x)
        let i1 = Int(edge.y)
        let a0 = px[i0]
        let a1 = px[i1]
        let axisA = a1 - a0
        let axisLengthSquaredA = lengthSquared(axisA)
        let margin = max(pData.vertexMargins[i0], pData.vertexMargins[i1])
        let query = aabbUnion(pData.vertexBoxes[i0], pData.vertexBoxes[i1])
        best.removeAll(keepingCapacity: true)
        if selfPair {
            for offset in Int(excludedStarts[e])..<Int(excludedStarts[e + 1]) {
                let other = excludedEdges[offset]
                if Int(other) > e { stamp[Int(other)] = Int32(e) }
            }
        }
        tree.visit(query, afterFeature: selfPair ? Int32(e) : -1,
            referenceExclusion: selfPair ? (rest[i0].position, exclusion * exclusion) : nil) { f in
            if stamp[Int(f)] == Int32(e) { return }
            let other = tEdges[Int(f)]
            let j0 = Int(other.x)
            let j1 = Int(other.y)
            if pw[i0] == 0 && pw[i1] == 0 && tw[j0] == 0 && tw[j1] == 0 { return }
            let b0 = tx[j0]
            let b1 = tx[j1]
            // Parallel, coplanar and out-of-reach pairs need no closest-segment solve. Use the
            // unnormalized common normal so these rejections also avoid normalization/sqrt.
            let axisB = b1 - b0
            let normal = cross(axisA, axisB)
            let normalSquared = lengthSquared(normal)
            guard normalSquared > parallelSine * parallelSine * max(axisLengthSquaredA * lengthSquared(axisB), 1.0e-18) else { return }
            let signedGap = dot(a0 - b0, normal)
            let gapSquared = signedGap * signedGap
            let reach = thickness + margin + edgeMargins[Int(f)]
            let unreadableGap = ambiguousGap * thickness
            guard gapSquared < reach * reach * normalSquared,
                gapSquared >= unreadableGap * unreadableGap * normalSquared else { return }
            let result = segmentDistance(a0, a1, b0, b1)
            if interiorOnly {
                let s = result.fraction1
                let u = result.fraction2
                if s <= 0.02 || s >= 0.98 || u <= 0.02 || u >= 0.98 { return }
            }
            let d = result.point1 - result.point2
            let distanceSquared = lengthSquared(d)
            guard distanceSquared < reach * reach else { return }
            if best.count == limit {
                let worst = best[limit - 1]
                guard distanceSquared < worst.0 || (distanceSquared == worst.0 && f < worst.1) else { return }
            }
            // Edges closer than half the thickness along their common normal have no readable side,
            // and a wrong side pushes them through each other: leave such pairs to the vertex-face
            // contacts this step. (A remembered side does not help: the common normal flips as edges
            // turn past parallel, which folds do all the time.)
            let side: Float = signedGap >= 0 ? 1 : -1
            var k = best.count
            best.append((distanceSquared, f, side))
            while k > 0 && (best[k - 1].0 > distanceSquared || (best[k - 1].0 == distanceSquared && best[k - 1].1 > f)) {
                best.swapAt(k, k - 1)
                k -= 1
            }
            if best.count > limit { best.removeLast() }
        }
        for (_, f, side) in best {
            let other = tEdges[Int(f)]
            contacts.append(SoftEdgeContact(bodyP: slotP, p0: Int32(i0), p1: Int32(i1), bodyT: slotT,
                                            t0: Int32(other.x), t1: Int32(other.y), side: side,
                                            thickness: thickness, friction: friction, interiorOnly: interiorOnly))
        }
    }
}

/// One Gauss-Seidel pass over the edge contacts, with the same recovery limit as the vertex
/// contacts.
func solveSoftEdgeContacts(
    _ contacts: UnsafeBufferPointer<SoftEdgeContact>, _ bodies: UnsafeBufferPointer<SoftContactBody>, maxRecovery: Float
) {
    for contact in contacts {
        let pb = bodies[Int(contact.bodyP)]
        let tb = bodies[Int(contact.bodyT)]
        let i0 = Int(contact.p0)
        let i1 = Int(contact.p1)
        let j0 = Int(contact.t0)
        let j1 = Int(contact.t1)
        let a0 = pb.x[i0]
        let a1 = pb.x[i1]
        let b0 = tb.x[j0]
        let b1 = tb.x[j1]
        let result = segmentDistance(a0, a1, b0, b1)
        let s = result.fraction1
        let u = result.fraction2
        let d = result.point1 - result.point2

        // Crossing configuration: both closest points inside their edges and the edges not near
        // parallel. Then the side is the one remembered along the common normal. Otherwise the
        // common normal says nothing about the gap: fall back to the plain distance, or leave the
        // pair to the vertex-face contacts.
        let crossing = cross(a1 - a0, b1 - b0)
        let sine2 = lengthSquared(crossing) / max(lengthSquared(a1 - a0) * lengthSquared(b1 - b0), 1.0e-18)
        let interior = s > 0 && s < 1 && u > 0 && u < 1 && sine2 > parallelSine * parallelSine
        var n: Vec3
        var separation: Float
        if interior {
            n = contact.side * normalize(crossing)
            separation = dot(d, n)
        } else {
            if contact.interiorOnly { continue }
            separation = length(d)
            guard separation > 1.0e-9 else { continue }
            n = (1 / separation) * d
        }

        // Where the same points were at the start of the substep.
        let start = ((1 - s) * pb.x0[i0] + s * pb.x0[i1]) - ((1 - u) * tb.x0[j0] + u * tb.x0[j1])
        let target = minFloat(contact.thickness, dot(start, n) + maxRecovery)
        let violation = separation - target
        guard violation < 0 else { continue }

        let wa0 = pb.w[i0] * (1 - s)
        let wa1 = pb.w[i1] * s
        let wb0 = tb.w[j0] * (1 - u)
        let wb1 = tb.w[j1] * u
        let total = wa0 * (1 - s) + wa1 * s + wb0 * (1 - u) + wb1 * u
        guard total > 0 else { continue }
        var correction = (-violation / total) * n

        let motion = (result.point1 - result.point2) - start
        let tangent = motion - dot(motion, n) * n
        let tangentLength = length(tangent)
        if tangentLength > 1.0e-9 {
            correction = correction - (minFloat(contact.friction * (-violation) / tangentLength, 1) / total) * tangent
        }

        pb.x[i0] = mulAdd(pb.x[i0], wa0, correction)
        pb.x[i1] = mulAdd(pb.x[i1], wa1, correction)
        tb.x[j0] = mulSub(tb.x[j0], wb0, correction)
        tb.x[j1] = mulSub(tb.x[j1], wb1, correction)
    }
}

