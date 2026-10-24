// Buoyancy and fluid drag. The submerged part of each shape is computed exactly for spheres,
// hulls, and closed meshes (clipped against the fluid surface) and by quadrature for capsules.

/// Submerged volume of a body below a fluid surface.
public struct SubmergedVolume: Sendable, Equatable {
    /// Volume of all shapes.
    public var totalVolume: Float = 0
    /// Volume below the surface.
    public var submergedVolume: Float = 0
    /// Centroid of the submerged volume, in world space.
    public var centerOfBuoyancy = Pos()

    public init() {}
}

/// Accumulates volume and first moment.
private struct VolumeSum {
    var total: Float = 0
    var submerged: Float = 0
    var moment = Vec3()

    mutating func add(total t: Float, submerged s: Float, centroid: Vec3) {
        total += t
        submerged += s
        moment = mulAdd(moment, s, centroid)
    }
}

extension World {
    /// Volume of the body below the plane `dot(surfaceNormal, x) = dot(surfaceNormal, surfacePosition)`.
    /// The normal points out of the fluid.
    public func bodySubmergedVolume(_ bodyId: BodyID, surfacePosition: Pos, surfaceNormal: Vec3) -> SubmergedVolume {
        var result = SubmergedVolume()
        guard isValidBody(bodyId), lengthSquared(surfaceNormal) > 0 else { return result }
        let record = body(bodyId)
        let transform = bodyTransformQuick(bodyIndex(bodyId))
        let n = normalize(surfaceNormal)
        // Work relative to the body origin to keep precision far from the world origin.
        let origin = transform.p
        let plane = Plane(n, dot(n, subPos(surfacePosition, origin)))
        let local = Transform(Vec3(), transform.q)

        var sum = VolumeSum()
        var shapeId = record.headShapeId
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            if shape.sensorIndex == B3.nullIndex {
                accumulateSubmerged(shape.geometry, local, plane, &sum)
            }
            shapeId = shape.nextShapeId
        }

        result.totalVolume = sum.total
        result.submergedVolume = sum.submerged
        result.centerOfBuoyancy = sum.submerged > 0
            ? offsetPos(origin, (1 / sum.submerged) * sum.moment) : transform.p
        return result
    }

    /// Apply buoyancy and drag for one time step (Jolt's `Body::ApplyBuoyancyImpulse`).
    /// - buoyancy: fluid density relative to the body's average density. 1 is neutral, more floats.
    /// - linearDrag: drag coefficient (about 0.5 for a sphere).
    /// - angularDrag: fraction of angular velocity removed per second when fully submerged.
    /// - fluidVelocity: velocity of the fluid, for currents.
    /// Returns false when the body is not in the fluid.
    @discardableResult
    public func applyBuoyancyImpulse(
        _ bodyId: BodyID, surfacePosition: Pos, surfaceNormal: Vec3, buoyancy: Float,
        linearDrag: Float = 0.5, angularDrag: Float = 0.01, fluidVelocity: Vec3 = Vec3(), timeStep: Float
    ) -> Bool {
        guard isValidBody(bodyId), timeStep > 0 else { return false }
        let record = body(bodyId)
        guard record.type == .dynamicBody, record.setIndex != SetType.disabledSet.rawValue else { return false }

        let volume = bodySubmergedVolume(bodyId, surfacePosition: surfacePosition, surfaceNormal: surfaceNormal)
        guard volume.submergedVolume > 1.0e-9 * lengthUnitsPerMeter, volume.totalVolume > 0 else { return false }

        let sim = bodySim(record)
        guard sim.invMass > 0 else { return false }
        let mass = 1 / sim.invMass
        let fluidDensity = buoyancy * mass / volume.totalVolume
        let fraction = minFloat(1, volume.submergedVolume / volume.totalVolume)

        var linearVelocity = Vec3()
        var angularVelocity = Vec3()
        if let state = bodyState(record) {
            linearVelocity = state.linearVelocity
            angularVelocity = state.angularVelocity
        }

        // Archimedes: the displaced fluid's weight, acting at the center of buoyancy.
        var impulse = (-fluidDensity * volume.submergedVolume * timeStep) * gravity

        // Quadratic drag on the velocity of the center of buoyancy relative to the fluid.
        let r = subPos(volume.centerOfBuoyancy, sim.center)
        let relative = linearVelocity + cross(angularVelocity, r) - fluidVelocity
        let speed = length(relative)
        if speed > 1.0e-6 {
            // Frontal area of the local bounds seen along the flow.
            let bounds = bodyLocalBounds(record)
            let size = bounds.upperBound - bounds.lowerBound
            let localFlow = invRotateVector(sim.transform.q, relative)
            let area = (absFloat(localFlow.x) * size.y * size.z + absFloat(localFlow.y) * size.x * size.z
                + absFloat(localFlow.z) * size.x * size.y) / speed
            var drag = (-0.5 * fluidDensity * linearDrag * area * fraction * timeStep * speed) * relative
            // Drag can stop the body relative to the fluid but not reverse it.
            let change = sim.invMass * length(drag)
            if change > speed { drag = (speed / change) * drag }
            impulse += drag
        }

        applyBodyLinearImpulse(bodyId, impulse, volume.centerOfBuoyancy, true)

        // Angular drag removes a fraction of the spin, bounded so it never reverses it.
        let damping = minFloat(1, angularDrag * fraction * timeStep)
        if damping > 0 && lengthSquared(angularVelocity) > 0 {
            let inertia = invertMatrix(sim.invInertiaWorld)
            if isValidMatrix3(inertia) {
                applyBodyAngularImpulse(bodyId, mulMV(inertia, -damping * angularVelocity), true)
            }
        }
        return true
    }

    /// Apply buoyancy to every dynamic body overlapping a box of fluid whose surface is the box top
    /// (along `surfaceNormal`). Returns the number of bodies affected.
    @discardableResult
    public func applyBuoyancy(
        in volume: AABB, surfaceNormal: Vec3 = Vec3(0, 1, 0), buoyancy: Float, linearDrag: Float = 0.5,
        angularDrag: Float = 0.01, fluidVelocity: Vec3 = Vec3(), timeStep: Float,
        filter: QueryFilter = .defaultFilter
    ) -> Int {
        let n = normalize(surfaceNormal)
        let corner = Vec3(n.x >= 0 ? volume.upperBound.x : volume.lowerBound.x,
                          n.y >= 0 ? volume.upperBound.y : volume.lowerBound.y,
                          n.z >= 0 ? volume.upperBound.z : volume.lowerBound.z)
        var seen = Set<Int32>()
        let tree = broadPhase.trees[Int(BodyType.dynamicBody.rawValue)]
        _ = tree.query(volume, maskBits: filter.maskBits, requireAllBits: false) { _, data in
            let shape = self.shapes[Int(data)]
            if shouldQueryCollide(shape.filter, filter) { seen.insert(shape.bodyId) }
            return true
        }
        var count = 0
        for bodyIndex in seen.sorted() {
            let id = makeBodyID(bodyIndex)
            if applyBuoyancyImpulse(id, surfacePosition: corner, surfaceNormal: n, buoyancy: buoyancy,
                                    linearDrag: linearDrag, angularDrag: angularDrag, fluidVelocity: fluidVelocity,
                                    timeStep: timeStep)
            {
                count += 1
            }
        }
        return count
    }

    private func bodyLocalBounds(_ record: Body) -> AABB {
        var bounds = AABB(Vec3(.greatestFiniteMagnitude, .greatestFiniteMagnitude, .greatestFiniteMagnitude),
                          Vec3(-.greatestFiniteMagnitude, -.greatestFiniteMagnitude, -.greatestFiniteMagnitude))
        var shapeId = record.headShapeId
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            bounds = aabbUnion(bounds, computeShapeAABB(shape, .identity))
            shapeId = shape.nextShapeId
        }
        return bounds
    }

    // MARK: Geometry

    private func accumulateSubmerged(_ geometry: ShapeGeometry, _ xf: Transform, _ plane: Plane, _ sum: inout VolumeSum) {
        switch geometry {
        case .sphere(let sphere):
            submergedSphere(transformPoint(xf, sphere.center), sphere.radius, plane, &sum)
        case .capsule(let capsule):
            submergedCapsule(transformPoint(xf, capsule.center1), transformPoint(xf, capsule.center2), capsule.radius, plane, &sum)
        case .hull(let hull):
            submergedHull(hull, xf, plane, &sum)
        case .mesh(let mesh):
            submergedMesh(mesh, xf, plane, &sum)
        case .compound(let compound):
            for i in 0..<compound.childCount {
                let child = compound.child(at: i)
                let childXF = mulTransforms(xf, child.transform)
                switch child.geometry {
                case .sphere(let s): accumulateSubmerged(.sphere(s), childXF, plane, &sum)
                case .capsule(let c): accumulateSubmerged(.capsule(c), childXF, plane, &sum)
                case .hull(let h): accumulateSubmerged(.hull(h), childXF, plane, &sum)
                case .mesh(let m): accumulateSubmerged(.mesh(m), childXF, plane, &sum)
                }
            }
        case .heightField:
            // Height fields are open surfaces without volume.
            break
        }
    }
}

// MARK: - Sphere

/// Spherical cap below the plane.
private func submergedSphere(_ center: Vec3, _ radius: Float, _ plane: Plane, _ sum: inout VolumeSum) {
    let total = 4 / 3 * B3.pi * radius * radius * radius
    let height = clampFloat(radius - (dot(plane.normal, center) - plane.offset), 0, 2 * radius)
    guard height > 0 else {
        sum.add(total: total, submerged: 0, centroid: center)
        return
    }
    let volume = B3.pi * height * height * (3 * radius - height) / 3
    // Centroid of a cap of height h, measured from the sphere center.
    let depth = 3 * (2 * radius - height) * (2 * radius - height) / (4 * (3 * radius - height))
    sum.add(total: total, submerged: volume, centroid: mulSub(center, depth, plane.normal))
}

// MARK: - Capsule

/// Circular segment beyond a chord at signed distance `d` from the center of a disk of radius `r`:
/// area and centroid distance from the center.
@inline(__always)
private func circularSegment(_ r: Float, _ d: Float) -> (area: Float, centroid: Float) {
    if d >= r { return (0, 0) }
    if d <= -r { return (B3.pi * r * r, 0) }
    let half = (r * r - d * d).squareRoot()
    let angle = atan2Approx(half, d)
    let area = r * r * angle - d * half
    guard area > 0 else { return (0, 0) }
    return (area, 2 * half * half * half / (3 * area))
}

/// Integrate the disks perpendicular to the axis with Simpson's rule.
private func submergedCapsule(_ p1: Vec3, _ p2: Vec3, _ radius: Float, _ plane: Plane, _ sum: inout VolumeSum) {
    var axisLength: Float = 0
    let axis = getLengthAndNormalize(&axisLength, p2 - p1)
    let total = B3.pi * radius * radius * (axisLength + 4 / 3 * radius)
    let center = 0.5 * (p1 + p2)

    // Quick outs against the bounding sphere segment.
    let s1 = dot(plane.normal, p1) - plane.offset
    let s2 = dot(plane.normal, p2) - plane.offset
    if min(s1, s2) >= radius {
        sum.add(total: total, submerged: 0, centroid: center)
        return
    }
    if max(s1, s2) <= -radius {
        sum.add(total: total, submerged: total, centroid: center)
        return
    }
    if axisLength < 1.0e-6 {
        submergedSphere(center, radius, plane, &sum)
        return
    }

    // In-disk direction of steepest descent into the fluid.
    let m = plane.normal - dot(plane.normal, axis) * axis
    let mLength = length(m)
    let half = 0.5 * axisLength + radius

    // Submerged disk area and centroid at axial position t.
    func slice(_ t: Float) -> (area: Float, centroid: Vec3) {
        let beyond = absFloat(t) - 0.5 * axisLength
        let r = beyond > 0 ? maxFloat(0, radius * radius - beyond * beyond).squareRoot() : radius
        let c = mulAdd(center, t, axis)
        let s = dot(plane.normal, c) - plane.offset
        if mLength < 1.0e-6 {
            return (s < 0 ? B3.pi * r * r : 0, c)
        }
        let segment = circularSegment(r, s / mLength)
        return (segment.area, mulSub(c, segment.centroid / mLength, m))
    }

    // The integrand is smooth between the cylinder/cap joints and the points where the disks
    // start or stop touching the surface. Integrate each smooth piece with Gauss-Legendre.
    var breaks: [Float] = [-half, -0.5 * axisLength, 0.5 * axisLength, half]
    let slope = dot(plane.normal, axis)
    if absFloat(slope) > 1.0e-6 {
        let s0 = dot(plane.normal, center) - plane.offset
        for edge in [-radius * mLength, 0, radius * mLength] {
            let t = (edge - s0) / slope
            if t > -half && t < half { breaks.append(t) }
        }
    }
    breaks.sort()

    let nodes: [Float] = [-0.906179846, -0.538469310, 0, 0.538469310, 0.906179846]
    let weights: [Float] = [0.236926885, 0.478628670, 0.568888889, 0.478628670, 0.236926885]
    var volume: Float = 0
    var moment = Vec3()
    for k in 0..<(breaks.count - 1) {
        let a = breaks[k]
        let b = breaks[k + 1]
        guard b - a > 1.0e-9 else { continue }
        let pieces = 4
        let width = (b - a) / Float(pieces)
        for p in 0..<pieces {
            let mid = a + (Float(p) + 0.5) * width
            for g in 0..<5 {
                let t = mid + 0.5 * width * nodes[g]
                let value = slice(t)
                let w = 0.5 * width * weights[g] * value.area
                volume += w
                moment = mulAdd(moment, w, value.centroid)
            }
        }
    }
    sum.add(total: total, submerged: volume, centroid: volume > 0 ? (1 / volume) * moment : center)
}

// MARK: - Polyhedra

/// Clip a convex polygon to the part below the plane and add the tetrahedra it forms with a point
/// on the plane. The cap that closes the clipped solid lies in the plane, so it adds nothing.
private func addClippedPolygon(_ polygon: [Vec3], _ plane: Plane, _ apex: Vec3, _ volume: inout Float, _ moment: inout Vec3) {
    var clipped: [Vec3] = []
    clipped.reserveCapacity(polygon.count + 2)
    let count = polygon.count
    for i in 0..<count {
        let a = polygon[i]
        let b = polygon[(i + 1) % count]
        let sa = dot(plane.normal, a) - plane.offset
        let sb = dot(plane.normal, b) - plane.offset
        if sa <= 0 { clipped.append(a) }
        if (sa < 0) != (sb < 0) && sa != sb {
            let t = sa / (sa - sb)
            clipped.append(mulAdd(a, t, b - a))
        }
    }
    guard clipped.count >= 3 else { return }
    let p0 = clipped[0]
    for i in 1..<(clipped.count - 1) {
        let p1 = clipped[i]
        let p2 = clipped[i + 1]
        let v = dot(p0 - apex, cross(p1 - apex, p2 - apex)) / 6
        volume += v
        moment = mulAdd(moment, v * 0.25, apex + p0 + p1 + p2)
    }
}

private func submergedHull(_ hull: Hull, _ xf: Transform, _ plane: Plane, _ sum: inout VolumeSum) {
    guard let points = hull.points, let edges = hull.edges, let faces = hull.faces else { return }
    let total = hull.volume
    let center = transformPoint(xf, hull.center)

    // Whole hull on one side?
    var lowest: Float = .greatestFiniteMagnitude
    var highest: Float = -.greatestFiniteMagnitude
    var world = [Vec3](repeating: Vec3(), count: Int(hull.vertexCount))
    for i in 0..<Int(hull.vertexCount) {
        world[i] = transformPoint(xf, points[i])
        let s = dot(plane.normal, world[i]) - plane.offset
        lowest = minFloat(lowest, s)
        highest = maxFloat(highest, s)
    }
    if lowest >= 0 {
        sum.add(total: total, submerged: 0, centroid: center)
        return
    }
    if highest <= 0 {
        sum.add(total: total, submerged: total, centroid: center)
        return
    }

    let apex = mulSub(center, dot(plane.normal, center) - plane.offset, plane.normal)
    var volume: Float = 0
    var moment = Vec3()
    var polygon: [Vec3] = []
    for f in 0..<Int(hull.faceCount) {
        polygon.removeAll(keepingCapacity: true)
        let start = Int(faces[f].edge)
        var e = start
        repeat {
            polygon.append(world[Int(edges[e].origin)])
            e = Int(edges[e].next)
        } while e != start && polygon.count <= Int(hull.vertexCount)
        addClippedPolygon(polygon, plane, apex, &volume, &moment)
    }
    sum.add(total: total, submerged: maxFloat(0, volume), centroid: volume > 0 ? (1 / volume) * moment : center)
}

/// Closed meshes only: an open mesh has no inside.
private func submergedMesh(_ mesh: Mesh, _ xf: Transform, _ plane: Plane, _ sum: inout VolumeSum) {
    guard let data = mesh.data else { return }
    let bounds = data.bounds
    let reference = transformPoint(xf, mesh.scale * aabbCenter(bounds))
    let apex = mulSub(reference, dot(plane.normal, reference) - plane.offset, plane.normal)
    var total: Float = 0
    var volume: Float = 0
    var moment = Vec3()
    var fullMoment = Vec3()
    queryMesh(mesh, bounds: AABB(mesh.scale * bounds.lowerBound - Vec3.one, mesh.scale * bounds.upperBound + Vec3.one).normalized) { a, b, c, _ in
        let wa = transformPoint(xf, a)
        let wb = transformPoint(xf, b)
        let wc = transformPoint(xf, c)
        let v = dot(wa - reference, cross(wb - reference, wc - reference)) / 6
        total += v
        fullMoment = mulAdd(fullMoment, v * 0.25, reference + wa + wb + wc)
        addClippedPolygon([wa, wb, wc], plane, apex, &volume, &moment)
        return true
    }
    guard total > 0 else { return }
    sum.add(total: total, submerged: maxFloat(0, volume), centroid: volume > 0 ? (1 / volume) * moment : (1 / total) * fullMoment)
}

private extension AABB {
    /// Order the corners after a scale that may be negative.
    var normalized: AABB { AABB(minVec3(lowerBound, upperBound), maxVec3(lowerBound, upperBound)) }
}
