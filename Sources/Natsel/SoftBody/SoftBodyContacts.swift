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

/// Sorted (cell, item) pairs: a uniform grid without hashing collisions.
struct CellGrid {
    var cellSize: Float
    var entries: [(key: Int64, item: Int32)] = []

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
        let lower = cell(box.lowerBound)
        let upper = cell(box.upperBound)
        for x in lower.0...upper.0 {
            for y in lower.1...upper.1 {
                for z in lower.2...upper.2 { entries.append((Self.key(x, y, z), item)) }
            }
        }
    }

    mutating func finish() {
        entries.sort { $0.key != $1.key ? $0.key < $1.key : $0.item < $1.item }
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
        var low = 0
        var high = entries.count
        while low < high {
            let mid = (low + high) / 2
            if entries[mid].key < key { low = mid + 1 } else { high = mid }
        }
        while low < entries.count && entries[low].key == key {
            body(entries[low].item)
            low += 1
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
    /// Bounds of each face padded by its vertices' margins and the largest thickness into it.
    var faceBoxes: [AABB] = []
    var faceMargins: [Float] = []
    var grid: CellGrid?
    /// +1 or -1 for a closed surface wound outward or inward, 0 for an open one.
    var closedSide: Float = 0
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
        parallel: Bool = false
    ) {
        contacts.removeAll(keepingCapacity: true)

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
            if records[indexP].enableSelfCollision { records[indexP].buildNeighbors() }
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
        guard pairs.isEmpty == false else { return }

        // Margins of every body in a pair, and a grid per target.
        var data = [DetectionBody](repeating: DetectionBody(), count: group.count)
        var needed = [Bool](repeating: false, count: group.count)
        for pair in pairs {
            needed[pair.slotP] = true
            needed[pair.slotT] = true
        }
        for (slot, index) in group.enumerated() where needed[slot] {
            let record = records + index
            let gain = length(gravity) * abs(record.pointee.gravityScale) * dt * dt
            data[slot].vertexMargins = record.pointee.velocities.map { length($0) * dt + gain }
        }
        for pair in pairs where data[pair.slotT].grid == nil {
            let slot = pair.slotT
            let t = records + group[slot]
            let x = t.pointee.positions
            let margins = data[slot].vertexMargins
            let thickness = targetThickness[slot]
            if t.pointee.settings.faces.isEmpty {
                var grid = CellGrid(cellSize: max(t.pointee.averageEdgeLength, thickness, 1.0e-3))
                for (j, p) in x.enumerated() {
                    let pad = margins[j] + thickness
                    grid.insert(AABB(p - Vec3(pad, pad, pad), p + Vec3(pad, pad, pad)), Int32(j))
                }
                grid.finish()
                data[slot].grid = grid
            } else {
                let faces = t.pointee.settings.faces
                var grid = CellGrid(cellSize: max(1.5 * t.pointee.averageEdgeLength, 2 * thickness, 1.0e-3))
                var boxes = [AABB](repeating: AABB(), count: faces.count)
                var faceMargins = [Float](repeating: 0, count: faces.count)
                for (f, face) in faces.enumerated() {
                    let ia = Int(face.vertex.x)
                    let ib = Int(face.vertex.y)
                    let ic = Int(face.vertex.z)
                    let margin = max(margins[ia], max(margins[ib], margins[ic]))
                    let pad = Vec3(margin + thickness, margin + thickness, margin + thickness)
                    faceMargins[f] = margin
                    boxes[f] = AABB(minVec3(x[ia], minVec3(x[ib], x[ic])) - pad, maxVec3(x[ia], maxVec3(x[ib], x[ic])) + pad)
                    if face.isDegenerate == false { grid.insert(boxes[f], Int32(f)) }
                }
                grid.finish()
                data[slot].grid = grid
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

        // Each pair fills its own list; the lists are joined in pair order, so the result does not
        // depend on the threads.
        let shared = data
        nonisolated(unsafe) let records = records
        var lists = [[SoftContact]](repeating: [], count: pairs.count)
        func detect(_ k: Int, _ list: inout [SoftContact]) {
            let pair = pairs[k]
            let p = records + group[pair.slotP]
            let t = records + group[pair.slotT]
            let friction = (p.pointee.friction * t.pointee.friction).squareRoot()
            if t.pointee.settings.faces.isEmpty {
                findVertexVertexContacts(p, t, Int32(pair.slotP), Int32(pair.slotT), pair.thickness, friction,
                                         pair.slotP == pair.slotT, shared[pair.slotP], shared[pair.slotT], &list)
            } else {
                findVertexTriangleContacts(p, t, Int32(pair.slotP), Int32(pair.slotT), pair.thickness, friction,
                                           pair.slotP == pair.slotT, shared[pair.slotP], shared[pair.slotT], &list)
            }
        }
        if parallel && pairs.count > 1 {
            lists.withUnsafeMutableBufferPointer { buffer in
                nonisolated(unsafe) let buffer = buffer
                DispatchQueue.concurrentPerform(iterations: buffer.count) { k in detect(k, &buffer[k]) }
            }
        } else {
            for k in 0..<pairs.count { detect(k, &lists[k]) }
        }
        for list in lists { contacts.append(contentsOf: list) }
    }
}

private func findVertexTriangleContacts(
    _ p: UnsafeMutablePointer<SoftBodyRecord>, _ t: UnsafeMutablePointer<SoftBodyRecord>, _ slotP: Int32,
    _ slotT: Int32, _ thickness: Float, _ friction: Float, _ selfPair: Bool, _ pData: DetectionBody,
    _ tData: DetectionBody, _ contacts: inout [SoftContact]
) {
    guard let grid = tData.grid else { return }
    let faces = t.pointee.settings.faces
    let tx = t.pointee.positions
    let tw = t.pointee.invMasses
    let px = p.pointee.positions
    let pw = p.pointee.invMasses
    let rest = t.pointee.settings.vertices
    let boxes = tData.faceBoxes
    let faceMargins = tData.faceMargins
    let vertexMargins = pData.vertexMargins
    // Self pairs this close in the rest pose are the surface's own neighborhood, not a fold.
    let restExclusion = max(thickness, 2 * p.pointee.averageEdgeLength)
    let targetBounds = aabbInflate(t.pointee.bounds, thickness + (faceMargins.max() ?? 0))

    // Faces already tested for the current vertex.
    var stamp = [Int32](repeating: -1, count: faces.count)
    // The closest few triangles of the current vertex: (distance squared, face, side).
    var best: [(Float, Int32, Float)] = []
    best.reserveCapacity(maxTrianglesPerVertex + 1)

    for vertex32 in p.pointee.surfaceVertices {
        let vertex = Int(vertex32)
        let x = px[vertex]
        let margin = vertexMargins[vertex]
        let query = AABB(x - Vec3(margin, margin, margin), x + Vec3(margin, margin, margin))
        guard aabbOverlaps(query, targetBounds) else { continue }
        best.removeAll(keepingCapacity: true)
        grid.visit(query) { f in
            if stamp[Int(f)] == vertex32 { return }
            stamp[Int(f)] = vertex32
            guard aabbOverlaps(query, boxes[Int(f)]) else { return }
            let face = faces[Int(f)]
            let ia = Int32(face.vertex.x)
            let ib = Int32(face.vertex.y)
            let ic = Int32(face.vertex.z)
            if selfPair {
                if vertex32 == ia || vertex32 == ib || vertex32 == ic { return }
                if p.pointee.areNeighbors(vertex32, ia) || p.pointee.areNeighbors(vertex32, ib)
                    || p.pointee.areNeighbors(vertex32, ic)
                {
                    return
                }
            }
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
            if selfPair {
                let r = rest[vertex].position
                let ra = rest[Int(ia)].position
                let rb = rest[Int(ib)].position
                let rc = rest[Int(ic)].position
                let rbary = closestTriangleBarycentric(ra, rb, rc, r)
                let rq = rbary.u * ra + rbary.v * rb + rbary.w * rc
                if lengthSquared(r - rq) < restExclusion * restExclusion { return }
            }
            // The outside of a closed surface is always the valid side; for open surfaces the side
            // the vertex is on now.
            let side: Float = tData.closedSide != 0 ? tData.closedSide : (dot(d, cross(b - a, c - a)) >= 0 ? 1 : -1)
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
            contact.thickness = thickness
            contact.friction = friction
            contacts.append(contact)
        }
    }
}

private func findVertexVertexContacts(
    _ p: UnsafeMutablePointer<SoftBodyRecord>, _ t: UnsafeMutablePointer<SoftBodyRecord>, _ slotP: Int32,
    _ slotT: Int32, _ thickness: Float, _ friction: Float, _ selfPair: Bool, _ pData: DetectionBody,
    _ tData: DetectionBody, _ contacts: inout [SoftContact]
) {
    guard let grid = tData.grid else { return }
    let px = p.pointee.positions
    let pw = p.pointee.invMasses
    let tx = t.pointee.positions
    let tw = t.pointee.invMasses
    let rest = p.pointee.settings.vertices
    let restExclusion = max(thickness, 2 * p.pointee.averageEdgeLength)
    var stamp = [Int32](repeating: -1, count: tx.count)
    for (i, x) in px.enumerated() {
        let margin = pData.vertexMargins[i]
        let query = AABB(x - Vec3(margin, margin, margin), x + Vec3(margin, margin, margin))
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
