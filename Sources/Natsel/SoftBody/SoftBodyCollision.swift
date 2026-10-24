// Soft body vs rigid shape collision detection. Once per step every vertex picks the closest
// surface plane among the rigid shapes near it. The XPBD iterations then keep the vertex on the
// positive side of that plane and exchange impulses with the rigid body.
//
// Vertices are processed in spatial patches. A patch is tested against a shape's bounds before any
// vertex is, and triangle geometry is fetched once per patch.

/// Closest surface feature of a shape to a point, in the shape's local frame.
private struct SurfaceHit {
    var normal = Vec3()
    /// Signed distance from the surface along `normal`; negative inside.
    var distance: Float = .greatestFiniteMagnitude
    var point = Vec3()
}

/// Per patch scratch reused across shapes.
private struct PatchScratch {
    var local: [Vec3] = []
    var triangles: [(Vec3, Vec3, Vec3)] = []
    /// Candidate features found for the patch, in the shape frame.
    var hits: [(vertex: Int, hit: SurfaceHit)] = []

    mutating func consider(_ vertex: Int, _ hit: SurfaceHit) {
        hits.append((vertex, hit))
    }
}

extension World {
    /// Collect nearby rigid shapes and choose a collision plane for each vertex.
    func detectSoftBodyCollisions(_ record: inout SoftBodyRecord, _ dt: Float) {
        let count = record.vertexCount
        record.colliders.removeAll(keepingCapacity: true)
        record.colliderIndices.withUnsafeMutableBufferPointer { $0.update(repeating: B3.nullIndex) }
        record.hasContact.withUnsafeMutableBufferPointer { $0.update(repeating: false) }
        record.edgeContacts.removeAll(keepingCapacity: true)
        guard record.collideWithRigidBodies, count > 0 else { return }

        // Speculative margin: a vertex can move at most this far in one step.
        var maxSpeedSquared: Float = 0
        record.velocities.withUnsafeBufferPointer { v in
            for i in 0..<v.count { maxSpeedSquared = maxFloat(maxSpeedSquared, lengthSquared(v[i])) }
        }
        let margin = maxSpeedSquared.squareRoot() * dt + B3.speculativeDistance + record.vertexRadius
        let queryBounds = aabbInflate(record.bounds, margin)

        // Candidate shapes in a deterministic order.
        var candidates: [Int32] = []
        for tree in broadPhase.trees {
            _ = tree.query(queryBounds, maskBits: record.filter.maskBits, requireAllBits: false) { _, data in
                let shapeIndex = Int32(truncatingIfNeeded: Int64(bitPattern: data))
                let shape = self.shapes[Int(shapeIndex)]
                if shape.sensorIndex != B3.nullIndex { return true }
                if shouldShapesCollide(record.filter, shape.filter) == false { return true }
                if self.bodies[Int(shape.bodyId)].setIndex == SetType.disabledSet.rawValue { return true }
                candidates.append(shapeIndex)
                return true
            }
        }
        guard candidates.isEmpty == false else { return }
        candidates.sort()

        // Patch bounds once per step.
        let patches = record.settings.patches
        var patchBounds = [AABB](repeating: AABB(), count: patches.count)
        record.positions.withUnsafeBufferPointer { p in
            for (k, range) in patches.enumerated() {
                var lower = p[range.lowerBound]
                var upper = lower
                for i in range {
                    lower = minVec3(lower, p[i])
                    upper = maxVec3(upper, p[i])
                }
                patchBounds[k] = aabbInflate(AABB(lower, upper), margin)
            }
        }

        var best = [Float](repeating: margin, count: count)
        var scratch = PatchScratch()
        let radius = record.vertexRadius

        // Edge bounds for the edge planes.
        if record.enableEdgeCollision { record.buildMeshEdges() }
        let edgeCount = record.enableEdgeCollision ? record.meshEdges.count : 0
        var edgeBounds = [AABB](repeating: AABB(), count: edgeCount)
        for e in 0..<edgeCount {
            let edge = record.meshEdges[e]
            let p0 = record.positions[Int(edge.x)]
            let p1 = record.positions[Int(edge.y)]
            edgeBounds[e] = aabbInflate(AABB(minVec3(p0, p1), maxVec3(p0, p1)), margin)
        }
        var edgeBest = [Float](repeating: margin, count: edgeCount)
        var edgePlanes = [SoftEdgePlane?](repeating: nil, count: edgeCount)

        for shapeIndex in candidates {
            let shape = shapes[Int(shapeIndex)]
            let bodyIndex = shape.bodyId
            let body = bodies[Int(bodyIndex)]
            let transform = bodyTransformQuick(bodyIndex)
            let shapeBounds = aabbInflate(shape.aabb, margin)

            var collider = SoftBodyCollider()
            collider.shapeIndex = shapeIndex
            collider.bodyIndex = bodyIndex
            let material = shape.material(0)
            collider.friction = mixFriction(record.friction, 0, material.friction, material.userMaterialId)
            collider.restitution = mixRestitution(record.restitution, 0, material.restitution, material.userMaterialId)
            if body.setIndex == SetType.awakeSet.rawValue {
                let set = Int(SetType.awakeSet.rawValue)
                let sim = solverSets[set].bodySims[Int(body.localIndex)]
                let state = solverSets[set].bodyStates[Int(body.localIndex)]
                collider.awakeIndex = body.localIndex
                collider.isDynamic = body.type == .dynamicBody && record.twoWayCoupling
                collider.invMass = collider.isDynamic ? sim.invMass : 0
                collider.invInertia = collider.isDynamic ? sim.invInertiaWorld : Matrix3()
                collider.center = sim.center
                collider.linearVelocity = state.linearVelocity
                collider.angularVelocity = state.angularVelocity
            } else {
                collider.center = transform.p
                // A sleeping dynamic body is treated as static until something wakes it.
                collider.isDynamic = false
            }
            collider.originalLinearVelocity = collider.linearVelocity
            collider.originalAngularVelocity = collider.angularVelocity

            let colliderIndex = Int32(record.colliders.count)
            var used = false

            for (k, range) in patches.enumerated() where aabbOverlaps(patchBounds[k], shapeBounds) {
                // Patch vertices in the shape frame.
                scratch.local.removeAll(keepingCapacity: true)
                scratch.hits.removeAll(keepingCapacity: true)
                for i in range { scratch.local.append(invTransformPoint(transform, record.positions[i])) }
                let localBounds = aabbTransformInverse(transform, patchBounds[k])
                collideGeometry(shape.geometry, .identity, range, localBounds, margin + radius, &scratch)

                for (vertex, hit) in scratch.hits {
                    let separation = hit.distance - radius
                    guard separation < best[vertex] else { continue }
                    best[vertex] = separation
                    let n = rotateVector(transform.q, hit.normal)
                    let point = transformPoint(transform, hit.point)
                    record.planeNormals[vertex] = n
                    record.planeOffsets[vertex] = dot(n, point) + radius
                    record.colliderIndices[vertex] = colliderIndex
                    used = true
                }
            }

            if edgeCount > 0 && detectEdgePlanes(&record, shape.geometry, transform, shapeBounds, edgeBounds, margin,
                                                 colliderIndex, &edgeBest, &edgePlanes)
            {
                used = true
            }

            if used { record.colliders.append(collider) }
        }
        for plane in edgePlanes {
            if let plane { record.edgeContacts.append(plane) }
        }
    }

    /// Dispatch on geometry. `childTransform` places the geometry in the shape frame.
    private func collideGeometry(
        _ geometry: ShapeGeometry, _ childTransform: Transform, _ range: Range<Int>, _ localBounds: AABB,
        _ reach: Float, _ scratch: inout PatchScratch
    ) {
        switch geometry {
        case .sphere(let sphere):
            collideSphere(sphere, childTransform, range, reach, &scratch)
        case .capsule(let capsule):
            collideCapsule(capsule, childTransform, range, reach, &scratch)
        case .hull(let hull):
            collideHull(hull, childTransform, range, reach, &scratch)
        case .mesh(let mesh):
            scratch.triangles.removeAll(keepingCapacity: true)
            let bounds = aabbTransformInverse(childTransform, localBounds)
            queryMesh(mesh, bounds: bounds) { a, b, c, _ in
                scratch.triangles.append((transformPoint(childTransform, a), transformPoint(childTransform, b),
                                          transformPoint(childTransform, c)))
                return true
            }
            collideTriangles(range, reach, &scratch)
        case .heightField(let field):
            scratch.triangles.removeAll(keepingCapacity: true)
            queryHeightField(field, bounds: localBounds) { a, b, c, _ in
                scratch.triangles.append((a, b, c))
                return true
            }
            collideTriangles(range, reach, &scratch)
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
                case .sphere(let s): collideSphere(s, xf, range, reach, &scratch)
                case .capsule(let s): collideCapsule(s, xf, range, reach, &scratch)
                case .hull(let h): collideHull(h, xf, range, reach, &scratch)
                case .mesh(let m):
                    collideGeometry(.mesh(m), xf, range, localBounds, reach, &scratch)
                }
            }
        }
    }

    private func collideSphere(
        _ sphere: Sphere, _ xf: Transform, _ range: Range<Int>, _ reach: Float, _ scratch: inout PatchScratch
    ) {
        let center = transformPoint(xf, sphere.center)
        for (k, vertex) in range.enumerated() {
            let d = scratch.local[k] - center
            let len = length(d)
            let distance = len - sphere.radius
            guard distance < reach else { continue }
            let n = len > 1.0e-6 ? (1 / len) * d : Vec3(0, 1, 0)
            scratch.consider(vertex, SurfaceHit(normal: n, distance: distance, point: mulAdd(center, sphere.radius, n)))
        }
    }

    private func collideCapsule(
        _ capsule: Capsule, _ xf: Transform, _ range: Range<Int>, _ reach: Float, _ scratch: inout PatchScratch
    ) {
        let a = transformPoint(xf, capsule.center1)
        let b = transformPoint(xf, capsule.center2)
        for (k, vertex) in range.enumerated() {
            let p = scratch.local[k]
            let q = pointToSegmentDistance(a, b, p)
            let d = p - q
            let len = length(d)
            let distance = len - capsule.radius
            guard distance < reach else { continue }
            let n = len > 1.0e-6 ? (1 / len) * d : perp(normalize(b - a))
            scratch.consider(vertex, SurfaceHit(normal: n, distance: distance, point: mulAdd(q, capsule.radius, n)))
        }
    }

    private func collideHull(
        _ hull: Hull, _ xf: Transform, _ range: Range<Int>, _ reach: Float, _ scratch: inout PatchScratch
    ) {
        guard let planes = hull.planes, let points = hull.points else { return }
        let faceCount = Int(hull.faceCount)
        var cache = SimplexCache()
        for (k, vertex) in range.enumerated() {
            // Hull frame.
            let p = invTransformPoint(xf, scratch.local[k])

            // The largest face separation bounds the distance from below.
            var maxSeparation: Float = -.greatestFiniteMagnitude
            var bestFace = 0
            for f in 0..<faceCount {
                let s = dot(planes[f].normal, p) - planes[f].offset
                if s > maxSeparation {
                    maxSeparation = s
                    bestFace = f
                }
            }
            guard maxSeparation < reach else { continue }

            var hit = SurfaceHit()
            if maxSeparation <= 0 {
                // Inside: push out through the closest face.
                let n = planes[bestFace].normal
                hit = SurfaceHit(normal: n, distance: maxSeparation, point: mulSub(p, maxSeparation, n))
            } else {
                // Outside: the exact closest point may be on an edge or vertex.
                var point = p
                let output = withUnsafePointer(to: &point) { pointer in
                    var input = DistanceInput()
                    input.proxyA = ShapeProxy(points: points, count: hull.vertexCount, radius: 0)
                    input.proxyB = ShapeProxy(points: pointer, count: 1, radius: 0)
                    input.transform = .identity
                    input.useRadii = false
                    cache.count = 0
                    return shapeDistance(input, &cache)
                }
                guard output.distance < reach else { continue }
                let n = output.distance > 1.0e-6 ? output.normal : planes[bestFace].normal
                hit = SurfaceHit(normal: n, distance: output.distance, point: output.pointA)
            }

            hit.normal = rotateVector(xf.q, hit.normal)
            hit.point = transformPoint(xf, hit.point)
            scratch.consider(vertex, hit)
        }
    }

    /// One sided triangles: vertices behind a triangle count as penetrating only when they are
    /// within reach of its face.
    private func collideTriangles(
        _ range: Range<Int>, _ reach: Float, _ scratch: inout PatchScratch
    ) {
        guard scratch.triangles.isEmpty == false else { return }
        let reachSquared = reach * reach
        for (a, b, c) in scratch.triangles {
            let normalLength = length(cross(b - a, c - a))
            guard normalLength > 1.0e-12 else { continue }
            let faceNormal = (1 / normalLength) * cross(b - a, c - a)
            let lower = minVec3(a, minVec3(b, c)) - Vec3(reach, reach, reach)
            let upper = maxVec3(a, maxVec3(b, c)) + Vec3(reach, reach, reach)

            for (k, vertex) in range.enumerated() {
                let p = scratch.local[k]
                if p.x < lower.x || p.y < lower.y || p.z < lower.z || p.x > upper.x || p.y > upper.y || p.z > upper.z {
                    continue
                }
                let side = dot(p - a, faceNormal)
                guard side > -reach else { continue }
                let closest = closestPointOnTriangle(a, b, c, p)
                let d = p - closest.point
                let distanceSquared = lengthSquared(d)
                if side >= 0 {
                    guard distanceSquared < reachSquared else { continue }
                    let len = distanceSquared.squareRoot()
                    let n = len > 1.0e-6 ? (1 / len) * d : faceNormal
                    scratch.consider(vertex, SurfaceHit(normal: n, distance: len, point: closest.point))
                } else if closest.feature == .featureTriangleFace {
                    // Behind the face interior: penetration along the face normal.
                    scratch.consider(vertex, SurfaceHit(normal: faceNormal, distance: side, point: mulSub(p, side, faceNormal)))
                }
            }
        }
    }
}

/// Bounds of a world box in a local frame.
@inline(__always)
func aabbTransformInverse(_ transform: Transform, _ box: AABB) -> AABB {
    aabbTransform(invertTransform(transform), box)
}
