

// Convex and mesh contact solver (contact_solver.c and contact_solver_wide.inl).
//
// The C solver batches four independent constraints per SIMD group. Graph coloring guarantees the
// bodies in a color are disjoint, so the lanes never interact. This port evaluates the same
// per-lane arithmetic in the same color/contact order. WideContactSolver.swift packs disjoint
// contacts into four SIMD lanes; this file also supplies the sequential overflow and fallback paths.

// MARK: - Symmetric 3x3

/// Symmetric 3x3 matrix in the same six-component form used by the C wide solver.
struct SymMatrix3: BitwiseCopyable {
    var cxx: Float = 0
    var cxy: Float = 0
    var cxz: Float = 0
    var cyy: Float = 0
    var cyz: Float = 0
    var czz: Float = 0

    init() {}

    init(cxx: Float, cxy: Float, cxz: Float, cyy: Float, cyz: Float, czz: Float) {
        self.cxx = cxx
        self.cxy = cxy
        self.cxz = cxz
        self.cyy = cyy
        self.cyz = cyz
        self.czz = czz
    }

    init(_ m: Matrix3) {
        // The C wide solver gathers the upper triangle of the matrix.
        cxx = m.cx.x
        cxy = m.cx.y
        cxz = m.cx.z
        cyy = m.cy.y
        cyz = m.cy.z
        czz = m.cz.z
    }

    /// m * a. The association order matches `b3MulMVW`.
    @inline(__always)
    func mulMV(_ a: Vec3) -> Vec3 {
        Vec3(
            cxx * a.x + (cxy * a.y + cxz * a.z),
            cxy * a.x + (cyy * a.y + cyz * a.z),
            cxz * a.x + (cyz * a.y + czz * a.z)
        )
    }
}

@inline(__always)
func addSym(_ a: SymMatrix3, _ b: SymMatrix3) -> SymMatrix3 {
    SymMatrix3(
        cxx: a.cxx + b.cxx, cxy: a.cxy + b.cxy, cxz: a.cxz + b.cxz,
        cyy: a.cyy + b.cyy, cyz: a.cyz + b.cyz, czz: a.czz + b.czz
    )
}

/// Invert a symmetric matrix using the same formula as the C wide solver.
func invertSym(_ m: SymMatrix3) -> SymMatrix3 {
    let cxx = m.cyy * m.czz - m.cyz * m.cyz
    let cxy = m.cxz * m.cyz - m.cxy * m.czz
    let cxz = m.cxy * m.cyz - m.cxz * m.cyy
    let cyy = m.cxx * m.czz - m.cxz * m.cxz
    let cyz = m.cxy * m.cxz - m.cxx * m.cyz
    let czz = m.cxx * m.cyy - m.cxy * m.cxy

    let det = m.cxx * cxx + (m.cxy * cxy + m.cxz * cxz)
    let valid = absFloat(det) > 1000 * Float.leastNormalMagnitude
    let invDet: Float = valid ? 1 / det : 0

    return SymMatrix3(
        cxx: invDet * cxx, cxy: invDet * cxy, cxz: invDet * cxz,
        cyy: invDet * cyy, cyz: invDet * cyz, czz: invDet * czz
    )
}

// MARK: - Constraints

struct ConstraintPoint: BitwiseCopyable {
    var rA = Vec3()
    var rB = Vec3()
    var baseSeparation: Float = 0
    var relativeVelocity: Float = 0
    var normalImpulse: Float = 0
    var totalNormalImpulse: Float = 0
    var restitutionImpulse: Float = 0
    var normalMass: Float = 0
    var leverArm: Float = 0

    init() {}
}

struct ManifoldConstraint {
    var points = InlineArray<4, ConstraintPoint>(repeating: ConstraintPoint())
    var pointCount: Int32 = 0
    var normal = Vec3()
    var tangent1 = Vec3()
    var tangent2 = Vec3()

    /// Friction centers
    var centerA = Vec3()
    var centerB = Vec3()
    var twistMass: Float = 0
    var twistImpulse: Float = 0
    var tangentMass = Matrix2()
    var frictionImpulse = Vec2()
    var rollingImpulse = Vec3()
    var tangentVelocity1: Float = 0
    var tangentVelocity2: Float = 0

    init() {}
}

struct ContactConstraint {
    var contactId: Int32 = B3.nullIndex
    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var invMassA: Float = 0
    var invMassB: Float = 0
    var invIA = SymMatrix3()
    var invIB = SymMatrix3()
    var rollingMass = SymMatrix3()
    var softness = Softness()
    var friction: Float = 0
    var restitution: Float = 0
    var rollingResistance: Float = 0
    var isStatic = false
    /// Index of this constraint's first manifold in the solver's flat manifold pool. Storing an
    /// index instead of an array keeps the constraint a plain value type: copying it no longer
    /// retains a heap buffer, which dominated the solver profile.
    var manifoldStart: Int32 = 0
    /// Number of manifolds this constraint owns in the flat pool.
    var manifoldCount: Int32 = 0

    init() {}
}

// MARK: - Solver

/// Inverse rotate used by the C wide solver. This is a different formulation from the scalar
/// `invRotateVector` and is not bit-equivalent to it.
@inline(__always)
func invRotateVectorWide(_ q: Quat, _ a: Vec3) -> Vec3 {
    var t = cross(q.v, a)
    t = Vec3(t.x + t.x, t.y + t.y, t.z + t.z)
    let u = cross(q.v, t)
    return Vec3((a.x - q.s * t.x) + u.x, (a.y - q.s * t.y) + u.y, (a.z - q.s * t.z) + u.z)
}

extension World {
    /// The solver constraint data for each graph color, rebuilt each step.
    struct SolverContactData {
        var constraints: [[ContactConstraint]] = Array(repeating: [], count: Int(B3.graphColorCount))

        /// Flat pool of manifold constraints. Each `ContactConstraint` owns a contiguous run
        /// starting at its `manifoldStart`.
        var manifoldPool: [ManifoldConstraint] = []

        /// Number of convex constraints at the front of each color's constraint list. The remaining
        /// constraints are mesh contacts.
        var convexCounts: [Int] = Array(repeating: 0, count: Int(B3.graphColorCount))

        // Packed once per step and reused by every substep. Overflow remains sequential.
        var wideConstraints: [[ContactConstraintW]] = Array(repeating: [], count: Int(B3.graphColorCount))
        var wideManifolds: [ManifoldConstraintW] = []
        var wideConvexCounts: [Int] = Array(repeating: 0, count: Int(B3.graphColorCount))

        /// The index range of the convex or mesh constraints for a color.
        func range(_ colorIndex: Int32, _ mesh: Bool) -> Range<Int> {
            let count = constraints[Int(colorIndex)].count
            let convex = convexCounts[Int(colorIndex)]
            return mesh ? convex..<count : 0..<convex
        }
    }

    /// Read one awake body state.
    ///
    /// The element is addressed through one array rather than through `solverSets[...]`, because
    /// subscripting the set copies the whole `SolverSet` and retains all four of its arrays.
    @inline(__always)
    func solverState(_ index: Int32) -> BodyState {
        guard index >= 0 else { return BodyState.identity }
        return solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(index)]
    }

    @inline(__always)
    func storeSolverState(_ index: Int32, _ state: BodyState) {
        guard index >= 0, (state.flags & BodyFlags.dynamicFlag.rawValue) != 0 else { return }
        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(index)] = state
    }

    /// Read a state from a borrowed state array. `index < 0` is a static body, which contributes
    /// no velocity, so identity is the correct stand-in.
    @inline(__always)
    static func stateAt(_ states: UnsafeMutableBufferPointer<BodyState>, _ index: Int32) -> BodyState {
        index >= 0 ? states[Int(index)] : BodyState.identity
    }

    /// Write a state back into a borrowed state array. Static bodies are never written.
    @inline(__always)
    static func setStateAt(
        _ states: UnsafeMutableBufferPointer<BodyState>, _ index: Int32, _ state: BodyState
    ) {
        if index >= 0 && (state.flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            states[Int(index)] = state
        }
    }

    /// Lend the awake set's body state array for one solver phase.
    ///
    /// Indexing `solverSets[...]` copies the whole `SolverSet`, which retains four arrays; doing that
    /// twice per constraint (read plus store) was the dominant solver cost. The closure receives the
    /// states in place, and nothing inside a contact phase may resize the awake set.
    @inline(__always)
    func withSolverStates<R>(_ body: (UnsafeMutableBufferPointer<BodyState>) -> R) -> R {
        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates.withUnsafeMutableBufferPointer { buffer in
            body(buffer)
        }
    }

    /// Convex contact ids for a color, in solver order.
    func solverConvexContactIds(_ colorIndex: Int32) -> [Int32] {
        constraintGraph.colors[Int(colorIndex)].convexContacts
    }

    /// Mesh contact ids for a color, in solver order. The C solver sorts these by manifold count so
    /// contacts with more manifolds are grouped into the same SIMD lanes.
    func solverMeshContactIds(_ colorIndex: Int32) -> [Int32] {
        let color = constraintGraph.colors[Int(colorIndex)]
        if colorIndex == overflowIndex {
            return color.contacts.map { $0.contactId }
        }
        let maximum = color.contacts.map { Int($0.manifoldCount) }.max() ?? 0
        var buckets = [[Int32]](repeating: [], count: maximum + 1)
        for spec in color.contacts { buckets[Int(spec.manifoldCount)].append(spec.contactId) }
        return buckets.reversed().flatMap { $0 }
    }

    /// All contact ids for a color in solver order: convex first, then mesh.
    func solverContactIds(_ colorIndex: Int32) -> [Int32] {
        solverConvexContactIds(colorIndex) + solverMeshContactIds(colorIndex)
    }

    /// Prepare the constraint data for one color.
    func prepareContacts(_ colorIndex: Int32, _ softness: Softness, _ staticSoftness: Softness, _ context: StepContext) {
        let contactIds = solverContactIds(colorIndex)
        guard contactIds.isEmpty == false else {
            return
        }

        let warmStartScale: Float = enableWarmStarting ? 1 : 0
        let invTau = 1 / B3.speculativeDistance
        let minFrictionWeight = B3.minFrictionWeight

        var constraints: [ContactConstraint] = []
        constraints.reserveCapacity(contactIds.count)

        for contactId in contactIds {
            let contact = contacts[Int(contactId)]
            let tangentVelocity = contact.tangentVelocity

            var constraint = ContactConstraint()
            constraint.contactId = contactId

            let indexA = decodeAwakeIndex(contact.encodedBodySimA)
            let indexB = decodeAwakeIndex(contact.encodedBodySimB)
            constraint.indexA = indexA
            constraint.indexB = indexB

            let simA = indexA >= 0 ? solverSets[Int(SetType.awakeSet.rawValue)].bodySims[Int(indexA)] : BodySim()
            let simB = indexB >= 0 ? solverSets[Int(SetType.awakeSet.rawValue)].bodySims[Int(indexB)] : BodySim()

            constraint.invMassA = simA.invMass
            constraint.invMassB = simB.invMass
            constraint.invIA = SymMatrix3(simA.invInertiaWorld)
            constraint.invIB = SymMatrix3(simB.invInertiaWorld)
            constraint.friction = contact.friction
            constraint.restitution = contact.restitution
            constraint.rollingResistance = contact.rollingResistance
            constraint.isStatic = (contact.flags & ContactFlags.staticFlag.rawValue) != 0
            constraint.softness = constraint.isStatic ? staticSoftness : softness

            let invIAB = addSym(constraint.invIA, constraint.invIB)
            if constraint.rollingResistance > 0 {
                constraint.rollingMass = invertSym(invIAB)
            } else {
                constraint.rollingMass = SymMatrix3()
            }

            for manifold in contact.manifolds {

                var manifoldConstraint = ManifoldConstraint()
                let pointCount = manifold.pointCount
                manifoldConstraint.pointCount = pointCount
                manifoldConstraint.normal = manifold.normal
                manifoldConstraint.tangent1 = perp(manifold.normal)
                manifoldConstraint.tangent2 = cross(manifoldConstraint.tangent1, manifold.normal)

                manifoldConstraint.tangentVelocity1 = dot(tangentVelocity, manifoldConstraint.tangent1)
                manifoldConstraint.tangentVelocity2 = dot(tangentVelocity, manifoldConstraint.tangent2)

                manifoldConstraint.twistImpulse = warmStartScale * manifold.twistImpulse
                if constraint.rollingResistance > 0 {
                    manifoldConstraint.rollingImpulse = warmStartScale * manifold.rollingImpulse
                } else {
                    manifoldConstraint.rollingImpulse = Vec3()
                }

                manifoldConstraint.frictionImpulse = Vec2(
                    warmStartScale * dot(manifold.frictionImpulse, manifoldConstraint.tangent1),
                    warmStartScale * dot(manifold.frictionImpulse, manifoldConstraint.tangent2)
                )

                let normal = manifoldConstraint.normal
                let tangent1 = manifoldConstraint.tangent1
                let tangent2 = manifoldConstraint.tangent2

                var centerA = Vec3()
                var centerB = Vec3()
                var totalWeight: Float = 0

                for pointIndex in 0..<Int(pointCount) {
                    let mp = manifold.points[pointIndex]
                    var cp = ConstraintPoint()

                    cp.rA = mp.anchorA
                    cp.rB = mp.anchorB
                    cp.baseSeparation = mp.separation - dot(cp.rB - cp.rA, normal)
                    cp.normalImpulse = warmStartScale * mp.normalImpulse
                    cp.totalNormalImpulse = 0
                    cp.relativeVelocity = 0
                    cp.restitutionImpulse = 0

                    let weight = clampFloat(2.0 - mp.separation * invTau, minFrictionWeight, 1)
                    centerA = centerA + weight * cp.rA
                    centerB = centerB + weight * cp.rB
                    totalWeight += weight

                    let rnA = cross(cp.rA, normal)
                    let rnB = cross(cp.rB, normal)

                    var kNormal = constraint.invMassA + constraint.invMassB
                    kNormal += dot(rnA, constraint.invIA.mulMV(rnA))
                    kNormal += dot(rnB, constraint.invIB.mulMV(rnB))
                    cp.normalMass = kNormal > 0 ? 1 / kNormal : 0

                    manifoldConstraint.points[pointIndex] = cp
                }

                if totalWeight > 0 {
                    centerA = (1 / totalWeight) * centerA
                    centerB = (1 / totalWeight) * centerB
                    manifoldConstraint.centerA = centerA
                    manifoldConstraint.centerB = centerB
                } else {
                    manifoldConstraint.centerA = Vec3()
                    manifoldConstraint.centerB = Vec3()
                }

                // Lever arms use the friction center.
                for pointIndex in 0..<Int(pointCount) {
                    var cp = manifoldConstraint.points[pointIndex]
                    let d = cp.rA - manifoldConstraint.centerA
                    cp.leverArm = dot(d, d).squareRoot()
                    manifoldConstraint.points[pointIndex] = cp
                }

                // Compute effective mass for the tangential friction. The C wide solver evaluates this
                // once per manifold, not per point.
                do {
                    let rtA1 = cross(manifoldConstraint.centerA, tangent1)
                    let rtA2 = cross(manifoldConstraint.centerA, tangent2)
                    let rtB1 = cross(manifoldConstraint.centerB, tangent1)
                    let rtB2 = cross(manifoldConstraint.centerB, tangent2)

                    let iArtA1 = constraint.invIA.mulMV(rtA1)
                    let iArtA2 = constraint.invIA.mulMV(rtA2)
                    let iBrtB1 = constraint.invIB.mulMV(rtB1)
                    let iBrtB2 = constraint.invIB.mulMV(rtB2)

                    let mSum = constraint.invMassA + constraint.invMassB
                    let kxx = (mSum + dot(rtA1, iArtA1)) + dot(rtB1, iBrtB1)
                    let kyy = (mSum + dot(rtA2, iArtA2)) + dot(rtB2, iBrtB2)
                    let kxy = dot(rtA1, iArtA2) + dot(rtB1, iBrtB2)

                    let det = kxx * kyy - kxy * kxy
                    let valid = absFloat(det) > 1000 * Float.leastNormalMagnitude && totalWeight > 0
                    let invDet: Float = valid ? 1 / det : 0

                    var tangentMass = Matrix2()
                    tangentMass.cx = Vec2(invDet * kyy, -(invDet * kxy))
                    tangentMass.cy = Vec2(-(invDet * kxy), invDet * kxx)
                    manifoldConstraint.tangentMass = tangentMass
                }

                let kTwist = dot(normal, invIAB.mulMV(normal))
                manifoldConstraint.twistMass = kTwist > 0 ? 1 / kTwist : 0

                // Sample relative velocities if needed for restitution or hit events.
                let haveRestitution = contact.restitution > 0
                let hitEvents = (contact.flags & ContactFlags.simEnableHitEvent.rawValue) != 0
                if haveRestitution || hitEvents {
                    let stateA = solverState(indexA)
                    let stateB = solverState(indexB)

                    for pointIndex in 0..<Int(pointCount) {
                        var cp = manifoldConstraint.points[pointIndex]
                        let vrA = stateA.linearVelocity + cross(stateA.angularVelocity, cp.rA)
                        let vrB = stateB.linearVelocity + cross(stateB.angularVelocity, cp.rB)
                        cp.relativeVelocity = dot(normal, vrB - vrA)
                        manifoldConstraint.points[pointIndex] = cp
                    }
                }

                if haveRestitution {
                    solveAnyRestitution = true
                }

                if constraint.manifoldCount == 0 {
                    constraint.manifoldStart = Int32(solverContacts.manifoldPool.count)
                }
                solverContacts.manifoldPool.append(manifoldConstraint)
                constraint.manifoldCount += 1
            }
            constraints.append(constraint)
        }

        solverContacts.constraints[Int(colorIndex)] = constraints
        solverContacts.convexCounts[Int(colorIndex)] = solverConvexContactIds(colorIndex).count
    }

    /// Prepare all constraint data for this step.
    func prepareAllContacts(_ context: StepContext) {
        solverContacts.constraints = Array(repeating: [], count: Int(B3.graphColorCount))
        solverContacts.convexCounts = Array(repeating: 0, count: Int(B3.graphColorCount))
        solveAnyRestitution = false

        prepareAllJoints(context)

        let contactHertz = minFloat(contactHertz, 0.125 * context.inv_h)
        let contactSoftness = makeSoft(contactHertz, contactDampingRatio, context.h)
        let staticSoftness = makeSoft(2.0 * contactHertz, 0.5 * contactDampingRatio, context.h)

        // The flat manifold pool is rebuilt from scratch each step.
        solverContacts.manifoldPool.removeAll(keepingCapacity: true)

        // Overflow first, then colors in order.
        prepareContacts(overflowIndex, contactSoftness, staticSoftness, context)
        for i in 0..<Int(overflowIndex) {
            prepareContacts(Int32(i), contactSoftness, staticSoftness, context)
        }
        prepareWideContacts()
    }

    // MARK: Warm start

    func warmStartContacts(_ colorIndex: Int32, _ mesh: Bool) {
        if usesWideContacts(colorIndex, mesh) {
            warmStartContactsWide(colorIndex, mesh)
            return
        }
        let range = solverContacts.range(colorIndex, mesh)
        let list = solverContacts.constraints[Int(colorIndex)]
        withSolverStates { states in
            solverContacts.manifoldPool.withUnsafeMutableBufferPointer { pool in
                warmStartContactsBody(range, list, states, pool)
            }
        }
    }

    private func warmStartContactsBody(
        _ range: Range<Int>, _ list: [ContactConstraint],
        _ states: UnsafeMutableBufferPointer<BodyState>,
        _ pool: UnsafeMutableBufferPointer<ManifoldConstraint>
    ) {
        for constraintIndex in range {
            let c = list[constraintIndex]

            var linearImpulse = Vec3()
            var angularImpulseA = Vec3()
            var angularImpulseB = Vec3()

            for manifoldOffset in 0..<Int(c.manifoldCount) {
                let cm = pool.baseAddress! + Int(c.manifoldStart) + manifoldOffset
                var totalNormalImpulse: Float = 0
                var momentA = Vec3()
                var momentB = Vec3()

                for pointIndex in 0..<Int(cm.pointee.pointCount) {
                    let cp = cm.pointee.points[pointIndex]
                    totalNormalImpulse += cp.normalImpulse
                    momentA = mulAdd(momentA, cp.normalImpulse, cp.rA)
                    momentB = mulAdd(momentB, cp.normalImpulse, cp.rB)
                }

                let normal = cm.pointee.normal
                var frictionImpulse = cm.pointee.frictionImpulse.x * cm.pointee.tangent1
                frictionImpulse = mulAdd(frictionImpulse, cm.pointee.frictionImpulse.y, cm.pointee.tangent2)

                let linear = mulAdd(frictionImpulse, totalNormalImpulse, normal)

                let twistImpulse = cm.pointee.twistImpulse * normal
                var angularA = cross(momentA, normal) + cross(cm.pointee.centerA, frictionImpulse) + twistImpulse
                var angularB = cross(momentB, normal) + cross(cm.pointee.centerB, frictionImpulse) + twistImpulse

                if c.rollingResistance > 0 {
                    angularA = angularA + cm.pointee.rollingImpulse
                    angularB = angularB + cm.pointee.rollingImpulse
                }

                linearImpulse = linearImpulse + linear
                angularImpulseA = angularImpulseA + angularA
                angularImpulseB = angularImpulseB + angularB
            }

            var bA = World.stateAt(states, c.indexA)
            var bB = World.stateAt(states, c.indexB)

            bA.angularVelocity = bA.angularVelocity - c.invIA.mulMV(angularImpulseA)
            bA.linearVelocity = bA.linearVelocity - c.invMassA * linearImpulse
            bB.angularVelocity = bB.angularVelocity + c.invIB.mulMV(angularImpulseB)
            bB.linearVelocity = bB.linearVelocity + c.invMassB * linearImpulse

            World.setStateAt(states, c.indexA, bA)
            World.setStateAt(states, c.indexB, bB)
        }
    }

    // MARK: Push (non-penetration with soft bias)

    func pushContacts(_ colorIndex: Int32, _ context: StepContext, _ mesh: Bool) {
        if usesWideContacts(colorIndex, mesh) {
            pushContactsWide(colorIndex, context, mesh)
            return
        }
        let range = solverContacts.range(colorIndex, mesh)
        let list = solverContacts.constraints[Int(colorIndex)]
        withSolverStates { states in
            solverContacts.manifoldPool.withUnsafeMutableBufferPointer { pool in
                pushContactsBody(range, list, context, states, pool)
            }
        }
    }

    private func pushContactsBody(
        _ range: Range<Int>, _ list: [ContactConstraint], _ context: StepContext,
        _ states: UnsafeMutableBufferPointer<BodyState>,
        _ pool: UnsafeMutableBufferPointer<ManifoldConstraint>
    ) {
        let inv_h = context.inv_h
        let contactSpeed = -contactSpeed
        let dynamicBiasRate = context.contactSoftness.massScale * context.contactSoftness.biasRate
        let dynamicMassScale = context.contactSoftness.massScale
        let dynamicImpulseScale = context.contactSoftness.impulseScale
        let staticBiasRate = context.staticSoftness.massScale * context.staticSoftness.biasRate
        let staticMassScale = context.staticSoftness.massScale
        let staticImpulseScale = context.staticSoftness.impulseScale

        for constraintIndex in range {
            let c = list[constraintIndex]

            var bA = World.stateAt(states, c.indexA)
            var bB = World.stateAt(states, c.indexB)

            let isStatic = c.isStatic
            let biasRate = isStatic ? staticBiasRate : dynamicBiasRate
            let massScale = isStatic ? staticMassScale : dynamicMassScale
            let impulseScale = isStatic ? staticImpulseScale : dynamicImpulseScale

            let dp = bB.deltaPosition - bA.deltaPosition
            let invMass = c.invMassA + c.invMassB

            for manifoldOffset in 0..<Int(c.manifoldCount) {
                let manifoldIndex = Int(c.manifoldStart) + manifoldOffset
                let cm = pool.baseAddress! + manifoldIndex
                let normal = cm.pointee.normal
                let normalSeparation = dot(normal, dp)
                let normalA = invRotateVectorWide(bA.deltaRotation, normal)
                let normalB = invRotateVectorWide(bB.deltaRotation, normal)
                var linearNormalVelocity = dot(normal, bB.linearVelocity - bA.linearVelocity)

                var manifoldImpulse: Float = 0

                for pointIndex in 0..<Int(cm.pointee.pointCount) {
                    var cp = cm.pointee.points[pointIndex]

                    let s = (normalSeparation + (dot(normalB, cp.rB) - dot(normalA, cp.rA))) + cp.baseSeparation
                    let separated = s > 0

                    // Speculative bias - positive
                    let specBias = s * inv_h

                    // Overlap bias - negative
                    let overlapBias = maxFloat(biasRate * s, contactSpeed)
                    let velocityBias = separated ? specBias : overlapBias

                    let pointMassScale = separated ? 1 : massScale
                    let pointImpulseScale = separated ? 0 : impulseScale

                    let uA = cross(cp.rA, normal)
                    let uB = cross(cp.rB, normal)
                    let gA = c.invIA.mulMV(uA)
                    let gB = c.invIB.mulMV(uB)

                    let vn = linearNormalVelocity + (dot(uB, bB.angularVelocity) - dot(uA, bA.angularVelocity))

                    let negImpulse = cp.normalMass * (pointMassScale * vn + velocityBias)
                        + pointImpulseScale * cp.normalImpulse
                    let impulse = maxFloat(-negImpulse, -cp.normalImpulse)
                    cp.normalImpulse = cp.normalImpulse + impulse

                    cm.pointee.points[pointIndex] = cp

                    linearNormalVelocity += invMass * impulse
                    manifoldImpulse += impulse
                    bA.angularVelocity = bA.angularVelocity - impulse * gA
                    bB.angularVelocity = bB.angularVelocity + impulse * gB
                }

                let P = manifoldImpulse * normal
                bA.linearVelocity = bA.linearVelocity - c.invMassA * P
                bB.linearVelocity = bB.linearVelocity + c.invMassB * P

            }

            World.setStateAt(states, c.indexA, bA)
            World.setStateAt(states, c.indexB, bB)
        }
    }

    // MARK: Solve (relax)

    func solveContacts(_ colorIndex: Int32, _ context: StepContext, _ mesh: Bool) {
        if usesWideContacts(colorIndex, mesh) {
            solveContactsWide(colorIndex, context, mesh)
            return
        }
        let range = solverContacts.range(colorIndex, mesh)
        let list = solverContacts.constraints[Int(colorIndex)]
        withSolverStates { states in
            solverContacts.manifoldPool.withUnsafeMutableBufferPointer { pool in
                solveContactsBody(range, list, context, states, pool)
            }
        }
    }

    private func solveContactsBody(
        _ range: Range<Int>, _ list: [ContactConstraint], _ context: StepContext,
        _ states: UnsafeMutableBufferPointer<BodyState>,
        _ pool: UnsafeMutableBufferPointer<ManifoldConstraint>
    ) {
        let inv_h = context.inv_h
        let epsilon = Float.ulpOfOne

        for constraintIndex in range {
            let c = list[constraintIndex]

            var bA = World.stateAt(states, c.indexA)
            var bB = World.stateAt(states, c.indexB)

            let anyRolling = c.rollingResistance > 0
            let dp = bB.deltaPosition - bA.deltaPosition
            let invMass = c.invMassA + c.invMassB

            for manifoldOffset in 0..<Int(c.manifoldCount) {
                let manifoldIndex = Int(c.manifoldStart) + manifoldOffset
                let cm = pool.baseAddress! + manifoldIndex
                let normal = cm.pointee.normal
                let normalSeparation = dot(normal, dp)
                let normalA = invRotateVectorWide(bA.deltaRotation, normal)
                let normalB = invRotateVectorWide(bB.deltaRotation, normal)

                let dv = bB.linearVelocity - bA.linearVelocity
                var linearNormalVelocity = dot(normal, dv)
                let linearTangentVelocity1 = dot(cm.pointee.tangent1, dv) - cm.pointee.tangentVelocity1
                let linearTangentVelocity2 = dot(cm.pointee.tangent2, dv) - cm.pointee.tangentVelocity2
                var manifoldImpulse: Float = 0
                var totalNormalImpulse: Float = 0
                var totalTwistLimit: Float = 0

                // Normal constraints
                for pointIndex in 0..<Int(cm.pointee.pointCount) {
                    var cp = cm.pointee.points[pointIndex]

                    let s = (normalSeparation + (dot(normalB, cp.rB) - dot(normalA, cp.rA))) + cp.baseSeparation

                    // Speculative bias, positive if separated and zero if overlapped
                    let velocityBias = maxFloat(0, s * inv_h)

                    let uA = cross(cp.rA, normal)
                    let uB = cross(cp.rB, normal)
                    let gA = c.invIA.mulMV(uA)
                    let gB = c.invIB.mulMV(uB)

                    let vn = linearNormalVelocity + (dot(uB, bB.angularVelocity) - dot(uA, bA.angularVelocity))

                    let negImpulse = cp.normalMass * (vn + velocityBias)
                    let impulse = maxFloat(-negImpulse, -cp.normalImpulse)
                    let newImpulse = cp.normalImpulse + impulse
                    cp.normalImpulse = newImpulse
                    cp.totalNormalImpulse = cp.totalNormalImpulse + newImpulse
                    totalNormalImpulse += newImpulse
                    totalTwistLimit += cp.leverArm * newImpulse

                    linearNormalVelocity += invMass * impulse
                    manifoldImpulse += impulse
                    bA.angularVelocity = bA.angularVelocity - impulse * gA
                    bB.angularVelocity = bB.angularVelocity + impulse * gB

                    cm.pointee.points[pointIndex] = cp
                }

                // Rolling resistance. After normal constraints because it uses the normal impulse.
                if anyRolling {
                    var deltaImpulse = c.rollingMass.mulMV(bA.angularVelocity - bB.angularVelocity)
                    let oldImpulse = cm.pointee.rollingImpulse
                    cm.pointee.rollingImpulse = oldImpulse + deltaImpulse

                    let maxImpulse = c.rollingResistance * totalNormalImpulse
                    let lengthSquared = dot(cm.pointee.rollingImpulse, cm.pointee.rollingImpulse)

                    let mask = lengthSquared > epsilon + maxImpulse * maxImpulse

                    // No approximate rsqrt here to maintain cross-platform determinism.
                    let normalize = maxImpulse / (lengthSquared.squareRoot() + epsilon)
                    let scale = mask ? normalize : 1

                    var scaledImpulse = scale * cm.pointee.rollingImpulse
                    if c.rollingResistance == 0 {
                        scaledImpulse = Vec3()
                    }
                    cm.pointee.rollingImpulse = scaledImpulse

                    deltaImpulse = cm.pointee.rollingImpulse - oldImpulse

                    bA.angularVelocity = bA.angularVelocity - c.invIA.mulMV(deltaImpulse)
                    bB.angularVelocity = bB.angularVelocity + c.invIB.mulMV(deltaImpulse)
                }

                // Central twist friction.
                do {
                    let twistSpeed = dot(normal, bB.angularVelocity - bA.angularVelocity)
                    let maxLambda = c.friction * totalTwistLimit
                    let oldImpulse = cm.pointee.twistImpulse
                    var impulse = -(cm.pointee.twistMass * twistSpeed)
                    impulse = maxFloat(-maxLambda - oldImpulse, minFloat(impulse, maxLambda - oldImpulse))
                    cm.pointee.twistImpulse = oldImpulse + impulse

                    bA.angularVelocity = bA.angularVelocity - impulse * c.invIA.mulMV(normal)
                    bB.angularVelocity = bB.angularVelocity + impulse * c.invIB.mulMV(normal)
                }

                // Central friction.
                do {
                    let tangent1 = cm.pointee.tangent1
                    let tangent2 = cm.pointee.tangent2

                    let rA = cm.pointee.centerA
                    let rB = cm.pointee.centerB

                    let angularRelativeVelocity = cross(bB.angularVelocity, rB) - cross(bA.angularVelocity, rA)

                    let vt = Vec2(
                        linearTangentVelocity1 + dot(angularRelativeVelocity, tangent1),
                        linearTangentVelocity2 + dot(angularRelativeVelocity, tangent2)
                    )

                    var deltaImpulse = mulMV2(cm.pointee.tangentMass, vt)
                    deltaImpulse = Vec2(-deltaImpulse.x, -deltaImpulse.y)
                    var newImpulse = Vec2(cm.pointee.frictionImpulse.x + deltaImpulse.x, cm.pointee.frictionImpulse.y + deltaImpulse.y)

                    let maxImpulse = c.friction * totalNormalImpulse
                    let lengthSquared = newImpulse.x * newImpulse.x + newImpulse.y * newImpulse.y
                    let mask = lengthSquared > maxImpulse * maxImpulse
                    let normalize = maxImpulse / (lengthSquared.squareRoot() + epsilon)
                    let scale = mask ? normalize : 1
                    newImpulse = Vec2(scale * newImpulse.x, scale * newImpulse.y)

                    deltaImpulse = Vec2(
                        newImpulse.x - cm.pointee.frictionImpulse.x,
                        newImpulse.y - cm.pointee.frictionImpulse.y
                    )

                    cm.pointee.frictionImpulse = newImpulse

                    var P = deltaImpulse.x * tangent1
                    P = P + deltaImpulse.y * tangent2
                    bA.angularVelocity = bA.angularVelocity - c.invIA.mulMV(cross(rA, P))
                    bB.angularVelocity = bB.angularVelocity + c.invIB.mulMV(cross(rB, P))

                    P = P + manifoldImpulse * normal
                    bA.linearVelocity = bA.linearVelocity - c.invMassA * P
                    bB.linearVelocity = bB.linearVelocity + c.invMassB * P
                }

            }

            World.setStateAt(states, c.indexA, bA)
            World.setStateAt(states, c.indexB, bB)
        }
    }

    // MARK: Restitution

    func applyRestitution(_ colorIndex: Int32, _ context: StepContext, _ mesh: Bool) {
        if usesWideContacts(colorIndex, mesh) {
            applyRestitutionWide(colorIndex, context, mesh)
            return
        }
        let range = solverContacts.range(colorIndex, mesh)
        let list = solverContacts.constraints[Int(colorIndex)]
        withSolverStates { states in
            solverContacts.manifoldPool.withUnsafeMutableBufferPointer { pool in
                applyRestitutionBody(range, list, context, states, pool)
            }
        }
    }

    private func applyRestitutionBody(
        _ range: Range<Int>, _ list: [ContactConstraint], _ context: StepContext,
        _ states: UnsafeMutableBufferPointer<BodyState>,
        _ pool: UnsafeMutableBufferPointer<ManifoldConstraint>
    ) {
        let inv_h = context.inv_h
        let negRestitutionThreshold = -restitutionThreshold
        let propagate = enableRestitutionPropagation

        for constraintIndex in range {
            let c = list[constraintIndex]

            if propagate == false && c.restitution == 0 {
                continue
            }

            var bA = World.stateAt(states, c.indexA)
            var bB = World.stateAt(states, c.indexB)

            let restitutionMask = c.restitution > 0
            let dp = bB.deltaPosition - bA.deltaPosition

            for manifoldOffset in 0..<Int(c.manifoldCount) {
                let manifoldIndex = Int(c.manifoldStart) + manifoldOffset
                let cm = pool.baseAddress! + manifoldIndex
                let normal = cm.pointee.normal
                let normalSeparation = dot(normal, dp)
                let normalA = invRotateVectorWide(bA.deltaRotation, normal)
                let normalB = invRotateVectorWide(bB.deltaRotation, normal)

                for pointIndex in 0..<Int(cm.pointee.pointCount) {
                    var cp = cm.pointee.points[pointIndex]

                    let s = (normalSeparation + (dot(normalB, cp.rB) - dot(normalA, cp.rA))) + cp.baseSeparation

                    let normalMass = propagate ? cp.normalMass : (restitutionMask ? cp.normalMass : 0)

                    let compressionImpulse = cp.totalNormalImpulse - cp.restitutionImpulse
                    let armed = restitutionMask && cp.relativeVelocity < negRestitutionThreshold && compressionImpulse > 0

                    let specBias = maxFloat(0, s * inv_h)
                    let velocityBias = armed ? c.restitution * cp.relativeVelocity : specBias

                    let vrA = bA.linearVelocity + cross(bA.angularVelocity, cp.rA)
                    let vrB = bB.linearVelocity + cross(bB.angularVelocity, cp.rB)
                    let vn = dot(vrB - vrA, normal)

                    let negImpulse = normalMass * (vn + velocityBias)

                    let newImpulse = maxFloat(cp.normalImpulse - negImpulse, 0)
                    var impulse = newImpulse - cp.normalImpulse

                    let approachImpulse = minFloat(maxFloat(-normalMass * vn, 0), maxFloat(impulse, 0))
                    let allowance = c.restitution * (compressionImpulse + approachImpulse) - cp.restitutionImpulse
                    let maxImpulse = approachImpulse + maxFloat(allowance, 0)
                    impulse = armed ? minFloat(impulse, maxImpulse) : impulse

                    cp.normalImpulse = cp.normalImpulse + impulse
                    cp.restitutionImpulse = cp.restitutionImpulse + (impulse - approachImpulse)
                    cp.totalNormalImpulse = cp.totalNormalImpulse + impulse

                    let P = impulse * normal
                    bA.angularVelocity = bA.angularVelocity - c.invIA.mulMV(cross(cp.rA, P))
                    bA.linearVelocity = bA.linearVelocity - c.invMassA * P
                    bB.angularVelocity = bB.angularVelocity + c.invIB.mulMV(cross(cp.rB, P))
                    bB.linearVelocity = bB.linearVelocity + c.invMassB * P

                    cm.pointee.points[pointIndex] = cp
                }

            }

            World.setStateAt(states, c.indexA, bA)
            World.setStateAt(states, c.indexB, bB)
        }
    }

    // MARK: Store

    func storeImpulses(_ colorIndex: Int32, _ mesh: Bool) {
        if usesWideContacts(colorIndex, mesh) {
            storeImpulsesWide(colorIndex, mesh)
            return
        }
        let list = solverContacts.constraints[Int(colorIndex)]
        let range = solverContacts.range(colorIndex, mesh)
        guard !range.isEmpty else { return }
        solverContacts.manifoldPool.withUnsafeBufferPointer { pool in
            contacts.withUnsafeMutableBufferPointer { store in
                for constraintIndex in range {
                    let c = list[constraintIndex]
                    let id = Int(c.contactId)
                    guard store[id].manifoldCount > 0 else { continue }
                    let hitEvents = (store[id].flags & ContactFlags.simEnableHitEvent.rawValue) != 0
                    store[id].manifolds.withUnsafeMutableBufferPointer { manifolds in
                        for offset in 0..<Int(c.manifoldCount) {
                            let cm = pool.baseAddress! + Int(c.manifoldStart) + offset
                            let out = manifolds.baseAddress! + offset
                            let count = Int(minInt(out.pointee.pointCount, cm.pointee.pointCount))
                            for pi in 0..<count {
                                out.pointee.points[pi].normalImpulse = cm.pointee.points[pi].normalImpulse
                                out.pointee.points[pi].totalNormalImpulse = cm.pointee.points[pi].totalNormalImpulse
                                if hitEvents {
                                    out.pointee.points[pi].normalVelocity = cm.pointee.points[pi].relativeVelocity
                                }
                            }
                            out.pointee.twistImpulse = cm.pointee.twistImpulse
                            out.pointee.frictionImpulse = cm.pointee.frictionImpulse.x * cm.pointee.tangent1
                                + cm.pointee.frictionImpulse.y * cm.pointee.tangent2
                            out.pointee.rollingImpulse = cm.pointee.rollingImpulse
                        }
                    }
                }
            }
        }
    }

    /// Store impulses for the whole graph.
    func storeAllImpulses() {
        storeWideJointImpulses()
        for mesh in [false, true] {
            storeImpulses(overflowIndex, mesh)
            for i in 0..<Int(overflowIndex) {
                storeImpulses(Int32(i), mesh)
            }
        }
    }

    /// Warm start every active constraint in solver order.
    /// Warm start the overflow constraints, then each color in order. Within a color the joints
    /// are solved before the contacts, matching the C stage layout.
    func warmStartAll(_ context: StepContext) {
        warmStartJoints(overflowIndex, context)
        warmStartContacts(overflowIndex, false)
        warmStartContacts(overflowIndex, true)
        for i in 0..<Int(overflowIndex) {
            warmStartJoints(Int32(i), context)
            warmStartContacts(Int32(i), false)
            warmStartContacts(Int32(i), true)
        }
    }

    func pushAll(_ context: StepContext) {
        solveJoints(overflowIndex, context, true)
        pushContacts(overflowIndex, context, false)
        pushContacts(overflowIndex, context, true)
        for i in 0..<Int(overflowIndex) {
            solveJoints(Int32(i), context, true)
            pushContacts(Int32(i), context, false)
            pushContacts(Int32(i), context, true)
        }
    }

    func solveAll(_ context: StepContext) {
        solveJoints(overflowIndex, context, false)
        solveContacts(overflowIndex, context, false)
        solveContacts(overflowIndex, context, true)
        for i in 0..<Int(overflowIndex) {
            solveJoints(Int32(i), context, false)
            solveContacts(Int32(i), context, false)
            solveContacts(Int32(i), context, true)
        }
    }

    func applyRestitutionAll(_ context: StepContext) {
        applyRestitution(overflowIndex, context, false)
        applyRestitution(overflowIndex, context, true)
        for i in 0..<Int(overflowIndex) {
            applyRestitution(Int32(i), context, false)
            applyRestitution(Int32(i), context, true)
        }
    }
}
