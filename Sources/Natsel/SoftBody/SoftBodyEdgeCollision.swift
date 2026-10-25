// Edges of soft bodies against rigid shapes. Vertex planes alone let a thin feature (a rod, a
// blade, a hull edge, a mesh edge) slip between two vertices: neither vertex comes near it while the
// edge between them passes through. Once per step every surface edge looks for the closest feature
// of the nearby shapes; when that point lies strictly inside the edge (an end point is already the
// vertex's job), the edge gets a contact plane through the feature. Each substep keeps the edge point
// on the positive side of its plane, sharing the correction between the two vertices by their
// weights and exchanging the impulse with dynamic bodies.

/// An edge point held on the positive side of a plane fixed to a rigid shape.
struct SoftEdgePlane: BitwiseCopyable {
    var v0: Int32 = 0
    var v1: Int32 = 0
    /// Position of the contact point along the edge, from v0 to v1.
    var s: Float = 0
    var normal = Vec3()
    var offset: Float = 0
    var collider: Int32 = 0
}

/// Closest feature of a shape to a segment, in the shape frame.
struct SegmentHit {
    /// Fraction along the segment.
    var s: Float = 0
    var normal = Vec3()
    var point = Vec3()
    /// Distance from the surface; negative inside a rounded feature.
    var distance: Float = .greatestFiniteMagnitude
}

/// Edge contacts only where vertices cannot see the feature.
private let interiorFraction: Float = 0.02

/// Vertex normals closer than about 8 degrees mean a flat surface under the edge.
private let flatCosine: Float = 0.99

extension SoftBodyRecord {
    /// Surface edges: the edges of the faces, or the constraint edges of bodies without faces.
    mutating func buildMeshEdges() {
        guard meshEdges.isEmpty else { return }
        if settings.faces.isEmpty {
            meshEdges = settings.edges.map { $0.vertex }
            return
        }
        var keys = Set<UInt64>()
        var edges: [SIMD2<UInt32>] = []
        for face in settings.faces where face.isDegenerate == false {
            for (a, b) in [(face.vertex.x, face.vertex.y), (face.vertex.y, face.vertex.z), (face.vertex.z, face.vertex.x)] {
                let lo = min(a, b)
                let hi = max(a, b)
                if keys.insert(UInt64(lo) << 32 | UInt64(hi)).inserted { edges.append(SIMD2(lo, hi)) }
            }
        }
        meshEdges = edges.sorted { $0.x != $1.x ? $0.x < $1.x : $0.y < $1.y }
    }

    /// Keep edge points above their rigid planes. Runs before the vertex collisions of a substep, so
    /// the velocities derived there include these corrections.
    func applyEdgeCollisions(
        _ x: UnsafeMutableBufferPointer<Vec3>, _ x0: UnsafeMutableBufferPointer<Vec3>, _ w: UnsafeBufferPointer<Float>,
        _ contact: UnsafeMutableBufferPointer<Bool>, _ colliders: UnsafeMutableBufferPointer<SoftBodyCollider>,
        _ h: Float, last: Bool
    ) {
        guard edgeContacts.isEmpty == false else { return }
        let invH = 1 / h
        // Dynamic bodies have moved by their tracked velocities; planes move with them.
        for i in 0..<colliders.count where colliders[i].isDynamic {
            colliders[i].displacement = mulAdd(colliders[i].displacement, h, colliders[i].linearVelocity)
            colliders[i].rotation = mulAdd(colliders[i].rotation, h, colliders[i].angularVelocity)
        }
        edgeContacts.withUnsafeBufferPointer { planes in
            for plane in planes {
                let i0 = Int(plane.v0)
                let i1 = Int(plane.v1)
                let s = plane.s
                let a0 = 1 - s
                let point = a0 * x[i0] + s * x[i1]
                let ci = Int(plane.collider)
                let r = point - subPos(colliders[ci].center, origin)
                let shift = colliders[ci].displacement + cross(colliders[ci].rotation, r)
                let penetration = plane.offset + dot(plane.normal, shift) - dot(plane.normal, point)
                guard penetration > 0 else { continue }
                let wEdge = a0 * a0 * w[i0] + s * s * w[i1]
                guard wEdge > 0 else { continue }

                var wBody: Float = 0
                if colliders[ci].isDynamic {
                    let rn = cross(r, plane.normal)
                    wBody = colliders[ci].invMass + dot(rn, mulMV(colliders[ci].invInertia, rn))
                }
                let lambda = penetration / (wEdge + wBody)
                var correction = lambda * plane.normal

                // Friction against the edge point's motion relative to the body over the substep.
                let start = a0 * x0[i0] + s * x0[i1]
                let motion = (point - start) - h * colliders[ci].pointVelocity(r)
                let tangent = motion - dot(motion, plane.normal) * plane.normal
                let tangentLength = length(tangent)
                if tangentLength > 1.0e-9 {
                    let k = minFloat(colliders[ci].friction * penetration / tangentLength, 1)
                    correction = correction - (k / (wEdge + wBody)) * tangent
                }

                x[i0] = mulAdd(x[i0], a0 * w[i0], correction)
                x[i1] = mulAdd(x[i1], s * w[i1], correction)
                if colliders[ci].isDynamic {
                    // The edge's momentum change, applied to the body with the opposite sign.
                    let impulse = invH * correction
                    let dv = colliders[ci].invMass * impulse
                    let dw = mulMV(colliders[ci].invInertia, cross(r, impulse))
                    colliders[ci].linearVelocity = colliders[ci].linearVelocity - dv
                    colliders[ci].angularVelocity = colliders[ci].angularVelocity - dw
                    // The body moves away within this substep too, so later edges see it moved.
                    colliders[ci].displacement = mulSub(colliders[ci].displacement, h, dv)
                    colliders[ci].rotation = mulSub(colliders[ci].rotation, h, dw)
                    colliders[ci].updated = true
                }
                if last {
                    contact[i0] = true
                    contact[i1] = true
                    colliders[ci].contactCount += 1
                }
            }
        }
    }
}

extension World {
    /// Closest feature of `geometry` (placed by `childTransform` in the shape frame) to the segment
    /// (a, b), both in the shape frame. Only hits closer than `reach` count.
    func closestSegmentFeature(
        _ geometry: ShapeGeometry, _ childTransform: Transform, _ a: Vec3, _ b: Vec3, _ localBounds: AABB,
        _ reach: Float, _ best: inout SegmentHit
    ) {
        switch geometry {
        case .sphere(let sphere):
            let center = transformPoint(childTransform, sphere.center)
            let q = pointToSegmentDistance(a, b, center)
            consider(q, center, sphere.radius, a, b, reach, &best)
        case .capsule(let capsule):
            let c1 = transformPoint(childTransform, capsule.center1)
            let c2 = transformPoint(childTransform, capsule.center2)
            let result = segmentDistance(a, b, c1, c2)
            consider(result.point1, result.point2, capsule.radius, a, b, reach, &best)
        case .hull(let hull):
            guard let points = hull.points else { return }
            // Segment in the hull frame.
            var segment = (invTransformPoint(childTransform, a), invTransformPoint(childTransform, b))
            segmentAgainstPoints(points, hull.vertexCount, &segment, childTransform, a, b, reach, &best)
        case .mesh(let mesh):
            let bounds = aabbTransformInverse(childTransform, localBounds)
            queryMesh(mesh, bounds: bounds) { p, q, r, _ in
                self.segmentAgainstTriangle(transformPoint(childTransform, p), transformPoint(childTransform, q),
                                            transformPoint(childTransform, r), a, b, reach, &best)
                return true
            }
        case .heightField(let field):
            queryHeightField(field, bounds: localBounds) { p, q, r, _ in
                self.segmentAgainstTriangle(p, q, r, a, b, reach, &best)
                return true
            }
        case .compound(let compound):
            var children: [Int] = []
            queryCompound(compound, bounds: localBounds) { child in
                children.append(child)
                return true
            }
            for c in children {
                let child = compound.child(at: c)
                let xf = mulTransforms(childTransform, child.transform)
                switch child.geometry {
                case .sphere(let s): closestSegmentFeature(.sphere(s), xf, a, b, localBounds, reach, &best)
                case .capsule(let s): closestSegmentFeature(.capsule(s), xf, a, b, localBounds, reach, &best)
                case .hull(let h): closestSegmentFeature(.hull(h), xf, a, b, localBounds, reach, &best)
                case .mesh(let m): closestSegmentFeature(.mesh(m), xf, a, b, localBounds, reach, &best)
                }
            }
        }
    }

    /// A rounded feature: `onSegment` and `onFeature` are the closest points on the edge and on the
    /// feature's core, `radius` the rounding.
    @inline(__always)
    private func consider(
        _ onSegment: Vec3, _ onFeature: Vec3, _ radius: Float, _ a: Vec3, _ b: Vec3, _ reach: Float, _ best: inout SegmentHit
    ) {
        let d = onSegment - onFeature
        let len = length(d)
        let distance = len - radius
        // Inside the rounding the direction from the core still says which way is out; only an edge
        // through the core itself has none.
        guard len > 1.0e-6, distance < reach, distance < best.distance else { return }
        let n = (1 / len) * d
        best = SegmentHit(s: segmentFraction(a, b, onSegment), normal: n, point: mulAdd(onFeature, radius, n),
                          distance: distance)
    }

    private func segmentAgainstTriangle(
        _ p: Vec3, _ q: Vec3, _ r: Vec3, _ a: Vec3, _ b: Vec3, _ reach: Float, _ best: inout SegmentHit
    ) {
        var triangle = (p, q, r)
        withUnsafePointer(to: &triangle) { pointer in
            pointer.withMemoryRebound(to: Vec3.self, capacity: 3) { points in
                var segment = (a, b)
                segmentAgainstPoints(points, 3, &segment, .identity, a, b, reach, &best)
            }
        }
    }

    /// GJK between a point cloud (in the frame placed by `transform`) and a segment given in that
    /// same frame; the hit is stored in the shape frame.
    private func segmentAgainstPoints(
        _ points: UnsafePointer<Vec3>, _ count: Int32, _ segment: inout (Vec3, Vec3), _ transform: Transform,
        _ a: Vec3, _ b: Vec3, _ reach: Float, _ best: inout SegmentHit
    ) {
        let output = withUnsafePointer(to: &segment) { pointer in
            pointer.withMemoryRebound(to: Vec3.self, capacity: 2) { segmentPoints in
                var input = DistanceInput()
                input.proxyA = ShapeProxy(points: points, count: count, radius: 0)
                input.proxyB = ShapeProxy(points: segmentPoints, count: 2, radius: 0)
                input.transform = .identity
                input.useRadii = false
                var cache = SimplexCache()
                return shapeDistance(input, &cache)
            }
        }
        // Zero distance: the edge is already through the feature; the vertices own that case.
        guard output.distance > 1.0e-6, output.distance < reach, output.distance < best.distance else { return }
        let onSegment = transformPoint(transform, output.pointB)
        best = SegmentHit(s: segmentFraction(a, b, onSegment), normal: rotateVector(transform.q, output.normal),
                          point: transformPoint(transform, output.pointA), distance: output.distance)
    }

    /// Detect edge planes against one shape. Called from the rigid detection for each candidate.
    func detectEdgePlanes(
        _ record: inout SoftBodyRecord, _ geometry: ShapeGeometry, _ transform: Transform, _ shapeBounds: AABB,
        _ edgeBounds: [AABB], _ reach: Float, _ colliderIndex: Int32, _ bestDistance: inout [Float],
        _ bestPlane: inout [SoftEdgePlane?]
    ) -> Bool {
        var used = false
        let radius = record.vertexRadius
        for (e, edge) in record.meshEdges.enumerated() where aabbOverlaps(edgeBounds[e], shapeBounds) {
            let i0 = Int(edge.x)
            let i1 = Int(edge.y)
            guard record.invMasses[i0] > 0 || record.invMasses[i1] > 0 else { continue }
            // Both ends already on this shape with nearly the same normal: the surface between them
            // is flat at this scale and nothing fits between the vertices.
            if record.colliderIndices[i0] == colliderIndex && record.colliderIndices[i1] == colliderIndex
                && dot(record.planeNormals[i0], record.planeNormals[i1]) > flatCosine
            {
                continue
            }
            let a = invTransformPoint(transform, record.positions[i0])
            let b = invTransformPoint(transform, record.positions[i1])
            let localBounds = aabbTransformInverse(transform, edgeBounds[e])
            var hit = SegmentHit()
            closestSegmentFeature(geometry, .identity, a, b, localBounds, reach + radius, &hit)
            let separation = hit.distance - radius
            guard hit.distance < .greatestFiniteMagnitude, hit.s > interiorFraction, hit.s < 1 - interiorFraction,
                separation < bestDistance[e]
            else { continue }
            bestDistance[e] = separation
            let n = rotateVector(transform.q, hit.normal)
            let point = transformPoint(transform, hit.point)
            bestPlane[e] = SoftEdgePlane(v0: Int32(i0), v1: Int32(i1), s: hit.s, normal: n,
                                         offset: dot(n, point) + radius, collider: colliderIndex)
            used = true
        }
        return used
    }
}

@inline(__always)
private func segmentFraction(_ a: Vec3, _ b: Vec3, _ p: Vec3) -> Float {
    let d = b - a
    let lengthSquared = dot(d, d)
    return lengthSquared > 0 ? clampFloat(dot(p - a, d) / lengthSquared, 0, 1) : 0
}
