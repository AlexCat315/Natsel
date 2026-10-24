internal import Dispatch

// Soft body simulation with small step XPBD. Each world step is split into `iterationCount`
// substeps; every substep integrates, projects the constraints once, resolves collisions, and
// derives velocities from the position change.
//
// Soft bodies run after the rigid bodies are finalized. Collision planes therefore sit at the rigid
// bodies' end-of-step poses, and the impulses exchanged with dynamic bodies change their velocities
// for the next step. Each soft body is independent, so they are simulated in parallel when the
// world has workers; rigid body velocity changes are merged afterwards in soft body order.

extension World {
    func stepSoftBodies(_ dt: Float) {
        guard softBodyIdPool.idCount > 0, dt > 0 else { return }

        wakeTouchedSoftBodies()
        wakeSoftBodiesTouchedBySoftBodies(dt)

        var active: [Int] = []
        for i in softBodies.indices where softBodies[i].id != B3.nullIndex && softBodies[i].awake {
            active.append(i)
        }
        guard active.isEmpty == false else { return }

        let groups = softContactGroups(active, dt)
        let gravity = gravity
        let workers = Int(maxInt(1, minInt(workerCount, 8)))
        let parallelWork = active.reduce(0) { $0 + softBodies[$1].vertexCount }

        // Lone bodies run in parallel; bodies in contact run together, each substep's per body work
        // spread over the workers.
        let singles = groups.filter { $0.count == 1 }
        let clusters = groups.filter { $0.count > 1 }
        softBodies.withUnsafeMutableBufferPointer { buffer in
            nonisolated(unsafe) let records = buffer.baseAddress!
            nonisolated(unsafe) let world = self
            if workers > 1 && singles.count > 1 && parallelWork >= 512 {
                singles.withUnsafeBufferPointer { list in
                    nonisolated(unsafe) let groups = list
                    DispatchQueue.concurrentPerform(iterations: groups.count) { k in
                        world.simulateSoftBodyGroup(records, groups[k], dt, gravity, parallel: false)
                    }
                }
            } else {
                for group in singles { simulateSoftBodyGroup(records, group, dt, gravity, parallel: false) }
            }
            for group in clusters {
                let work = group.reduce(0) { $0 + records[$1].vertexCount }
                simulateSoftBodyGroup(records, group, dt, gravity, parallel: workers > 1 && work >= 512)
            }
        }

        // Merge rigid body velocity changes in a fixed order.
        for i in active {
            applySoftBodyImpulses(i)
            softBodies[i].updateSleep(dt)
        }
    }

    /// Simulate soft bodies that may touch each other substep by substep, projecting their contacts
    /// between the constraints and the rigid body collisions. Runs on any thread; touches only the
    /// group's records.
    func simulateSoftBodyGroup(
        _ records: UnsafeMutablePointer<SoftBodyRecord>, _ group: [Int], _ dt: Float, _ gravity: Vec3, parallel: Bool
    ) {
        nonisolated(unsafe) let records = records
        nonisolated(unsafe) let world = self
        /// Run per body work, on the workers when allowed. Bodies only touch their own records.
        func forEachBody(_ body: @escaping (Int, Int) -> Void) {
            if parallel {
                group.withUnsafeBufferPointer { list in
                    nonisolated(unsafe) let list = list
                    DispatchQueue.concurrentPerform(iterations: list.count) { k in body(k, list[k]) }
                }
            } else {
                for (k, index) in group.enumerated() { body(k, index) }
            }
        }

        forEachBody { _, index in world.detectSoftBodyCollisions(&records[index], dt) }

        var contacts: [SoftContact] = []
        var edgeContacts: [SoftEdgeContact] = []
        let needsContacts = group.count > 1 || records[group[0]].enableSelfCollision
        if needsContacts {
            detectSoftContacts(records, group, dt, &contacts, &edgeContacts, parallel: parallel)
        } else {
            records[group[0]].faceSides.removeAll(keepingCapacity: true)
        }

        if contacts.isEmpty && edgeContacts.isEmpty {
            forEachBody { _, index in
                records[index].simulate(dt, gravity)
                records[index].finishStep()
            }
            return
        }

        // Bodies in contact substep together, at the largest iteration count among them.
        let iterations = group.reduce(Int32(1)) { max($0, records[$1].iterationCount) }
        var substepList: [SoftBodySubstep] = []
        for index in group { substepList.append(records[index].beginStep(dt, gravity, iterations)) }
        let substeps = substepList

        // Arrays keep their storage for the whole step.
        var views: [SoftContactBody] = []
        for index in group {
            let record = records + index
            views.append(SoftContactBody(
                x: record.pointee.positions.withUnsafeMutableBufferPointer { $0 },
                x0: record.pointee.previousPositions.withUnsafeMutableBufferPointer { $0 },
                w: record.pointee.invMasses.withUnsafeBufferPointer { $0 }))
        }

        let count = substeps[0].count
        contacts.withUnsafeBufferPointer { contactList in
        edgeContacts.withUnsafeBufferPointer { edgeList in
            views.withUnsafeBufferPointer { bodyList in
                for iteration in 0..<count {
                    let last = iteration == count - 1
                    forEachBody { k, index in records[index].solveSubstep(substeps[k]) }
                    solveSoftContacts(contactList, bodyList, maxRecovery: contactSpeed * substeps[0].h)
                    solveSoftEdgeContacts(edgeList, bodyList, maxRecovery: contactSpeed * substeps[0].h)
                    forEachBody { k, index in records[index].collideSubstep(substeps[k], last: last) }
                }
            }
        }
        }
        forEachBody { _, index in records[index].finishStep() }
    }

    /// Connected components of soft bodies whose swept bounds overlap, in index order.
    private func softContactGroups(_ active: [Int], _ dt: Float) -> [[Int]] {
        var parent = Array(0..<active.count)
        func find(_ i: Int) -> Int {
            var i = i
            while parent[i] != i {
                parent[i] = parent[parent[i]]
                i = parent[i]
            }
            return i
        }
        let boxes = active.map { i -> AABB in
            let record = softBodies[i]
            return aabbInflate(record.bounds, record.maxSpeed * dt + record.contactThickness)
        }
        for a in 0..<active.count where softBodies[active[a]].collideWithSoftBodies {
            for b in (a + 1)..<active.count where softBodies[active[b]].collideWithSoftBodies {
                guard aabbOverlaps(boxes[a], boxes[b]),
                    shouldShapesCollide(softBodies[active[a]].filter, softBodies[active[b]].filter)
                else { continue }
                let ra = find(a)
                let rb = find(b)
                if ra != rb { parent[max(ra, rb)] = min(ra, rb) }
            }
        }
        var groups: [[Int]] = []
        var groupOfRoot = [Int: Int]()
        for k in 0..<active.count {
            let root = find(k)
            if let g = groupOfRoot[root] {
                groups[g].append(active[k])
            } else {
                groupOfRoot[root] = groups.count
                groups.append([active[k]])
            }
        }
        return groups
    }

    /// A sleeping soft body wakes when a moving soft body reaches it.
    private func wakeSoftBodiesTouchedBySoftBodies(_ dt: Float) {
        for i in softBodies.indices
        where softBodies[i].id != B3.nullIndex && softBodies[i].awake == false && softBodies[i].collideWithSoftBodies {
            let bounds = aabbInflate(softBodies[i].bounds, softBodies[i].contactThickness)
            for j in softBodies.indices
            where j != i && softBodies[j].id != B3.nullIndex && softBodies[j].awake && softBodies[j].collideWithSoftBodies {
                let mover = softBodies[j]
                guard mover.maxSpeed > mover.sleepThreshold,
                    aabbOverlaps(bounds, aabbInflate(mover.bounds, mover.maxSpeed * dt)),
                    shouldShapesCollide(softBodies[i].filter, mover.filter)
                else { continue }
                wakeSoftBody(i)
                break
            }
        }
    }

    /// Sleeping soft bodies wake when a moving rigid body enters their bounds.
    private func wakeTouchedSoftBodies() {
        for i in softBodies.indices where softBodies[i].id != B3.nullIndex && softBodies[i].awake == false {
            let bounds = aabbInflate(softBodies[i].bounds, B3.speculativeDistance)
            var wake = false
            for tree in broadPhase.trees[1...] {
                _ = tree.query(bounds, maskBits: softBodies[i].filter.maskBits, requireAllBits: false) { _, data in
                    let shape = self.shapes[Int(data)]
                    let body = self.bodies[Int(shape.bodyId)]
                    if body.setIndex == SetType.awakeSet.rawValue && body.sleepVelocity > body.sleepThreshold
                        && shouldShapesCollide(self.softBodies[i].filter, shape.filter)
                    {
                        wake = true
                        return false
                    }
                    return true
                }
                if wake { break }
            }
            if wake { wakeSoftBody(i) }
        }
    }

    /// Apply the velocity changes accumulated on dynamic colliders to the rigid bodies.
    private func applySoftBodyImpulses(_ index: Int) {
        let awake = Int(SetType.awakeSet.rawValue)
        for collider in softBodies[index].colliders {
            let body = bodies[Int(collider.bodyIndex)]
            if collider.updated && collider.awakeIndex != B3.nullIndex && body.setIndex == SetType.awakeSet.rawValue {
                let local = Int(body.localIndex)
                var state = solverSets[awake].bodyStates[local]
                state.linearVelocity += collider.linearVelocity - collider.originalLinearVelocity
                state.angularVelocity += collider.angularVelocity - collider.originalAngularVelocity
                solverSets[awake].bodyStates[local] = state
            } else if collider.contactCount > 0 && body.type == .dynamicBody
                && body.setIndex >= SetType.firstSleepingSet.rawValue
                && softBodies[index].maxSpeed > body.sleepThreshold && softBodies[index].twoWayCoupling
            {
                // A moving soft body landing on a sleeping body wakes it; it responds next step.
                wakeBody(collider.bodyIndex)
            }
        }
    }
}

// MARK: - XPBD

/// Constants of one substep.
struct SoftBodySubstep {
    var count = 1
    var h: Float = 0
    var invH2: Float = 0
    var damping: Float = 1
    var maxSpeedSquared: Float = 0
    var g = Vec3()
    var restitutionThreshold: Float = 0
}

extension SoftBodyRecord {
    /// Largest vertex speed after the last step.
    var maxSpeed: Float {
        var m: Float = 0
        velocities.withUnsafeBufferPointer { v in
            for i in 0..<v.count { m = maxFloat(m, lengthSquared(v[i])) }
        }
        return m.squareRoot()
    }

    mutating func simulate(_ dt: Float, _ gravity: Vec3) {
        let substep = beginStep(dt, gravity, iterationCount)
        for iteration in 0..<substep.count {
            solveSubstep(substep)
            collideSubstep(substep, last: iteration == substep.count - 1)
        }
    }

    /// Per step constants and the skinning update. `iterations` may exceed the body's own count
    /// when it substeps together with other soft bodies.
    mutating func beginStep(_ dt: Float, _ gravity: Vec3, _ iterations: Int32) -> SoftBodySubstep {
        // Kinematic skinned vertices follow the skin.
        if hasSkinPose && settings.skinned.isEmpty == false {
            updateSkin()
            for (k, s) in settings.skinned.enumerated() where invMasses[Int(s.vertex)] == 0 {
                positions[Int(s.vertex)] = skinnedPositions[k]
            }
        }
        for i in colliders.indices {
            colliders[i].contactCount = 0
            colliders[i].displacement = Vec3()
            colliders[i].rotation = Vec3()
        }

        var substep = SoftBodySubstep()
        substep.count = Int(max(1, iterations))
        substep.h = dt / Float(substep.count)
        substep.invH2 = 1 / (substep.h * substep.h)
        substep.damping = maxFloat(0, 1 - linearDamping * substep.h)
        substep.maxSpeedSquared = maxLinearVelocity * maxLinearVelocity
        substep.g = gravityScale * gravity
        substep.restitutionThreshold = -2 * length(gravity) * substep.h
        return substep
    }

    /// Pressure, integration, and the constraints of one substep.
    mutating func solveSubstep(_ substep: SoftBodySubstep) {
        let settings = self.settings
        let h = substep.h
        let damping = substep.damping
        let maxSpeedSquared = substep.maxSpeedSquared
        let g = substep.g

        // Move the mutable state out so the buffers are uniquely referenced and the helpers below
        // can read the rest of the record.
        var xs: [Vec3] = []
        var x0s: [Vec3] = []
        var vs: [Vec3] = []
        swap(&xs, &positions)
        swap(&x0s, &previousPositions)
        swap(&vs, &velocities)
        defer {
            swap(&xs, &positions)
            swap(&x0s, &previousPositions)
            swap(&vs, &velocities)
        }

        xs.withUnsafeMutableBufferPointer { x in
        x0s.withUnsafeMutableBufferPointer { x0 in
        vs.withUnsafeMutableBufferPointer { v in
        invMasses.withUnsafeBufferPointer { w in
            let n = x.count
            if pressure > 0 && settings.faces.isEmpty == false {
                applyPressure(x, v, w, h)
            }

            // Integrate.
            for i in 0..<n {
                x0[i] = x[i]
                guard w[i] > 0 else { continue }
                var vi = damping * mulAdd(v[i], h, g)
                let s2 = lengthSquared(vi)
                if s2 > maxSpeedSquared { vi = (maxLinearVelocity / s2.squareRoot()) * vi }
                v[i] = vi
                x[i] = mulAdd(x[i], h, vi)
            }

            solveBendBatches(x, w, substep.invH2)
            solveVolumeBatches(x, w, substep.invH2)
            applySkin(settings.skinned, x, w)
            solveEdgeBatches(x, w, substep.invH2)
            applyLongRangeAttachments(settings.longRangeAttachments, x, w)
        }
        }
        }
        }
    }

    /// Rigid body collisions of one substep, and velocities from the position change.
    mutating func collideSubstep(_ substep: SoftBodySubstep, last: Bool) {
        var xs: [Vec3] = []
        var x0s: [Vec3] = []
        var vs: [Vec3] = []
        var contacts: [Bool] = []
        var colliderState: [SoftBodyCollider] = []
        swap(&xs, &positions)
        swap(&x0s, &previousPositions)
        swap(&vs, &velocities)
        swap(&contacts, &hasContact)
        swap(&colliderState, &colliders)
        defer {
            swap(&xs, &positions)
            swap(&x0s, &previousPositions)
            swap(&vs, &velocities)
            swap(&contacts, &hasContact)
            swap(&colliderState, &colliders)
        }

        xs.withUnsafeMutableBufferPointer { x in
        x0s.withUnsafeMutableBufferPointer { x0 in
        vs.withUnsafeMutableBufferPointer { v in
        contacts.withUnsafeMutableBufferPointer { contact in
        colliderState.withUnsafeMutableBufferPointer { cs in
        invMasses.withUnsafeBufferPointer { w in
            applyEdgeCollisions(x, x0, w, contact, cs, substep.h, last: last)
            applyCollisions(x, x0, v, w, contact, cs, substep.h, substep.restitutionThreshold, last: last)
        }
        }
        }
        }
        }
        }
    }

    /// Called after `simulate`; bounds use the final positions.
    mutating func finishStep() {
        updateBounds()
    }

    // MARK: Constraints

    private func applyLongRangeAttachments(
        _ attachments: [SoftBodyLRA], _ x: UnsafeMutableBufferPointer<Vec3>, _ w: UnsafeBufferPointer<Float>
    ) {
        guard attachments.isEmpty == false else { return }
        attachments.withUnsafeBufferPointer { attachments in
            for a in attachments where w[Int(a.vertex)] > 0 {
                let anchor = x[Int(a.anchor)]
                let d = x[Int(a.vertex)] - anchor
                let len2 = lengthSquared(d)
                if len2 > a.maxDistance * a.maxDistance {
                    x[Int(a.vertex)] = mulAdd(anchor, a.maxDistance / len2.squareRoot(), d)
                }
            }
        }
    }

    private func applySkin(
        _ skinned: [SoftBodySkinned], _ x: UnsafeMutableBufferPointer<Vec3>, _ w: UnsafeBufferPointer<Float>
    ) {
        guard hasSkinPose, skinned.isEmpty == false else { return }
        for (k, s) in skinned.enumerated() where w[Int(s.vertex)] > 0 {
            let v = Int(s.vertex)
            let target = skinnedPositions[k]
            var p = x[v]

            // Stay within the max distance of the skin.
            let d = p - target
            let len2 = lengthSquared(d)
            if len2 > s.maxDistance * s.maxDistance {
                p = len2 > 0 ? mulAdd(target, s.maxDistance / len2.squareRoot(), d) : target
            }

            // Stay outside the back stop sphere.
            if s.backStopRadius > 0 {
                let center = mulSub(target, s.backStopDistance + s.backStopRadius, skinnedNormals[k])
                let e = p - center
                let len = length(e)
                if len < s.backStopRadius {
                    p = len > 1.0e-9 ? mulAdd(center, s.backStopRadius / len, e) : mulAdd(center, s.backStopRadius, skinnedNormals[k])
                }
            }
            x[v] = p
        }
    }

    /// Inflate a closed surface: p = pressure / V on each face, shared by its three vertices.
    private func applyPressure(
        _ x: UnsafeMutableBufferPointer<Vec3>, _ v: UnsafeMutableBufferPointer<Vec3>, _ w: UnsafeBufferPointer<Float>,
        _ h: Float
    ) {
        let faces = settings.faces
        let origin = x[0]
        var sixVolume: Float = 0
        for f in faces {
            let a = x[Int(f.vertex.x)] - origin
            let b = x[Int(f.vertex.y)] - origin
            let c = x[Int(f.vertex.z)] - origin
            sixVolume += dot(a, cross(b, c))
        }
        guard sixVolume > 1.0e-9 else { return }
        // Face force = p * area normal = p * cross / 2, split over three vertices.
        let coefficient = pressure / (sixVolume / 6) * h / 6
        for f in faces {
            let i0 = Int(f.vertex.x)
            let i1 = Int(f.vertex.y)
            let i2 = Int(f.vertex.z)
            let impulse = coefficient * cross(x[i1] - x[i0], x[i2] - x[i0])
            v[i0] = mulAdd(v[i0], w[i0], impulse)
            v[i1] = mulAdd(v[i1], w[i1], impulse)
            v[i2] = mulAdd(v[i2], w[i2], impulse)
        }
    }

    // MARK: Collision

    /// Project colliding vertices out of their planes, then update velocities with friction,
    /// restitution, and the impulse exchange with dynamic bodies (Müller et al., Detailed Rigid
    /// Body Simulation with Extended Position Based Dynamics, section 3.6).
    private func applyCollisions(
        _ x: UnsafeMutableBufferPointer<Vec3>, _ x0: UnsafeMutableBufferPointer<Vec3>,
        _ v: UnsafeMutableBufferPointer<Vec3>, _ w: UnsafeBufferPointer<Float>,
        _ contact: UnsafeMutableBufferPointer<Bool>, _ colliders: UnsafeMutableBufferPointer<SoftBodyCollider>,
        _ h: Float, _ restitutionThreshold: Float, last: Bool
    ) {
        let invH = 1 / h
        let n = x.count
        let hasColliders = colliders.isEmpty == false
        planeNormals.withUnsafeBufferPointer { normals in
        planeOffsets.withUnsafeBufferPointer { offsets in
        colliderIndices.withUnsafeBufferPointer { indices in
            for i in 0..<n {
                guard w[i] > 0 else { continue }
                let previousVelocity = v[i]
                var vi = invH * (x[i] - x0[i])

                let ci = indices[i]
                if hasColliders && ci >= 0 {
                    let normal = normals[i]
                    let penetration = offsets[i] - dot(normal, x[i])
                    if penetration > 0 {
                        x[i] = mulAdd(x[i], penetration, normal)
                        if last { contact[i] = true }

                        let collider = Int(ci)
                        let r = x[i] - colliders[collider].center
                        let bodyVelocity = colliders[collider].pointVelocity(r)
                        let relative = vi - bodyVelocity
                        let vn = dot(normal, relative)
                        let normalVelocity = vn * normal
                        let tangentVelocity = relative - normalVelocity
                        let tangentLength = length(tangentVelocity)

                        // Friction bounded by the normal correction: |dv_t| <= mu * d / h.
                        var dv = Vec3()
                        if tangentLength > 0 {
                            dv = minFloat(colliders[collider].friction * penetration / (tangentLength * h), 1) * tangentVelocity
                        }
                        // Remove the approach velocity, then bounce off fast impacts.
                        dv += normalVelocity
                        let previousNormal = dot(previousVelocity - bodyVelocity, normal)
                        if previousNormal < restitutionThreshold {
                            dv = mulAdd(dv, colliders[collider].restitution * previousNormal, normal)
                        }

                        if colliders[collider].isDynamic {
                            let rn = cross(r, normal)
                            let w2 = colliders[collider].invMass + dot(rn, mulMV(colliders[collider].invInertia, rn))
                            let total = w[i] + w2
                            if total > 0 {
                                let p = (1 / total) * dv
                                vi = mulSub(vi, w[i], p)
                                colliders[collider].linearVelocity = mulAdd(colliders[collider].linearVelocity, colliders[collider].invMass, p)
                                colliders[collider].angularVelocity = colliders[collider].angularVelocity
                                    + mulMV(colliders[collider].invInertia, cross(r, p))
                                colliders[collider].updated = true
                            }
                        } else {
                            vi = vi - dv
                        }
                        if last { colliders[collider].contactCount += 1 }
                    }
                }
                v[i] = vi
            }
        }
        }
        }
    }

    // MARK: Sleep

    mutating func updateSleep(_ dt: Float) {
        guard enableSleep else {
            sleepTime = 0
            return
        }
        if maxSpeed < sleepThreshold {
            sleepTime += dt
            if sleepTime >= B3.timeToSleep {
                awake = false
                velocities.withUnsafeMutableBufferPointer { $0.update(repeating: Vec3()) }
            }
        } else {
            sleepTime = 0
        }
    }
}

// MARK: - Dihedral gradients

struct DihedralGradients {
    var angle: Float = 0
    var g0 = Vec3()
    var g1 = Vec3()
    var g2 = Vec3()
    var g3 = Vec3()
    var valid = false
}

/// Angle and its gradients for the dihedral between triangles (x0, x1, x2) and (x1, x0, x3)
/// (Bridson et al. 2003, Simulation of Clothing with Folds and Wrinkles).
@inline(__always)
func dihedralGradients(_ x0: Vec3, _ x1: Vec3, _ x2: Vec3, _ x3: Vec3) -> DihedralGradients {
    var result = DihedralGradients()
    let e = x1 - x0
    let len = length(e)
    let n1 = cross(e, x2 - x0)
    let n2 = cross(x3 - x0, e)
    let n1Squared = lengthSquared(n1)
    let n2Squared = lengthSquared(n2)
    guard len > 1.0e-9, n1Squared > 1.0e-18, n2Squared > 1.0e-18 else { return result }

    result.angle = atan2Approx(dot(cross(n1, n2), e) / len, dot(n1, n2))

    let invLen = 1 / len
    let m1 = (1 / n1Squared) * n1
    let m2 = (1 / n2Squared) * n2
    // Gradients of the angle; they sum to zero.
    result.g2 = -len * m1
    result.g3 = -len * m2
    result.g0 = -(dot(x2 - x1, e) * invLen * m1 + dot(x3 - x1, e) * invLen * m2)
    result.g1 = dot(x2 - x0, e) * invLen * m1 + dot(x3 - x0, e) * invLen * m2
    result.valid = true
    return result
}
