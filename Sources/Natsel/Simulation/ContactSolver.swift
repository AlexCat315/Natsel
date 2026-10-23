

// Convex and mesh contact solver (contact_solver.c and contact_solver_wide.inl).
//
// The C solver batches four independent constraints per SIMD group. Graph coloring guarantees the
// bodies in a color are disjoint, so the lanes never interact. This port evaluates the same
// per-lane arithmetic in the same color/contact order, which is bit-equivalent to the 4-wide
// kernel while staying scalar.

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
    }

    // MARK: Warm start

    func warmStartContacts(_ colorIndex: Int32, _ mesh: Bool) {
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
        let list = solverContacts.constraints[Int(colorIndex)]
        let range = solverContacts.range(colorIndex, mesh)
        let pool = solverContacts.manifoldPool
        // Write through the element rather than through a `var contact = contacts[...]` copy: the
        // copy makes `manifolds` multiply referenced, so every point store would reallocate the
        // array (copy-on-write) instead of updating it in place.
        contacts.withUnsafeMutableBufferPointer { store in
            for constraintIndex in range {
                let c = list[constraintIndex]
                let contactId = Int(c.contactId)
                guard store[contactId].manifoldCount > 0 else {
                    continue
                }

                let hitEvents = (store[contactId].flags & ContactFlags.simEnableHitEvent.rawValue) != 0

                for manifoldOffset in 0..<Int(c.manifoldCount) {
                    let cm = pool[Int(c.manifoldStart) + manifoldOffset]
                    let pointCount = Int(minInt(store[contactId].manifolds[manifoldOffset].pointCount, cm.pointCount))

                    for pointIndex in 0..<pointCount {
                        let cp = cm.points[pointIndex]
                        store[contactId].manifolds[manifoldOffset].points[pointIndex].normalImpulse =
                            cp.normalImpulse
                        store[contactId].manifolds[manifoldOffset].points[pointIndex].totalNormalImpulse =
                            cp.totalNormalImpulse

                        // The hit-event pass reads the approach speed recorded at prepare time.
                        if hitEvents {
                            store[contactId].manifolds[manifoldOffset].points[pointIndex].normalVelocity =
                                cp.relativeVelocity
                        }
                    }

                    store[contactId].manifolds[manifoldOffset].twistImpulse = cm.twistImpulse
                    store[contactId].manifolds[manifoldOffset].frictionImpulse =
                        cm.frictionImpulse.x * cm.tangent1 + cm.frictionImpulse.y * cm.tangent2
                    store[contactId].manifolds[manifoldOffset].rollingImpulse = cm.rollingImpulse
                }
            }
        }
    }

    /// Store impulses for the whole graph.
    func storeAllImpulses() {
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
        pushConvex(overflowIndex, context)
        pushContacts(overflowIndex, context, true)
        for i in 0..<Int(overflowIndex) {
            solveJoints(Int32(i), context, true)
            pushConvex(Int32(i), context)
            pushContacts(Int32(i), context, true)
        }
    }

    /// Convex contacts go through the 4-lane path when it is enabled and there are enough of them.
    @inline(__always)
    func pushConvex(_ colorIndex: Int32, _ context: StepContext) {
        if enableWideSolver {
            pushContactsConvexWide(colorIndex, context)
        } else {
            pushContacts(colorIndex, context, false)
        }
    }

    /// Push the convex contacts of one color through the 4-lane path, falling back to the scalar
    /// loop when there are not enough contacts to fill a batch.
    func pushContactsConvexWide(_ colorIndex: Int32, _ context: StepContext) {
        guard solverContacts.range(colorIndex, false).count >= 4 else {
            pushContacts(colorIndex, context, false)
            return
        }
        pushContactsWide(colorIndex, context)
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

// MARK: - Wide contact solve (4 lanes)

extension World {
    /// Solve four convex contacts per pass, mirroring C's `b3PushContactsW`. Lane `i` runs the same
    /// scalar expressions as `pushContacts`, so the result is bit-identical; C verifies the same
    /// equivalence between its NEON path and the scalar fallback.
    ///
    /// The three arrays are copied into local `var`s, mutated through one flat pointer each, and
    /// published back once. Swift's exclusive-access rule forbids nesting `withUnsafeMutable...`
    /// over overlapping state, so the pointers are taken explicitly.
    func pushContactsWide(_ colorIndex: Int32, _ context: StepContext) {
        let range = solverContacts.range(colorIndex, false)
        guard range.count >= 4 else {
            pushContacts(colorIndex, context, false)
            return
        }

        let inv_h = FloatW(repeating: context.inv_h)
        let contactSpeedW = FloatW(repeating: -contactSpeed)
        let zeroW = FloatW(repeating: 0)
        let oneW = FloatW(repeating: 1)

        let dynBias = context.contactSoftness.massScale * context.contactSoftness.biasRate
        let dynMass = context.contactSoftness.massScale
        let dynImpulse = context.contactSoftness.impulseScale
        let statBias = context.staticSoftness.massScale * context.staticSoftness.biasRate
        let statMass = context.staticSoftness.massScale
        let statImpulse = context.staticSoftness.impulseScale

        let awakeSet = Int(SetType.awakeSet.rawValue)

        var colorConstraints = solverContacts.constraints[Int(colorIndex)]
        var poolArray = solverContacts.manifoldPool
        var stateArray = solverSets[awakeSet].bodyStates

        let constraintCount = colorConstraints.count
        let poolCount = poolArray.count
        let stateCount = stateArray.count

        colorConstraints.withUnsafeMutableBufferPointer { csBuf in
            poolArray.withUnsafeMutableBufferPointer { poolBuf in
                stateArray.withUnsafeMutableBufferPointer { stateBuf in
                    widePushBody(
                        csBuf, constraintCount, poolBuf, poolCount,
                        stateBuf, stateCount, range,
                        inv_h, contactSpeedW, zeroW, oneW,
                        dynBias, dynMass, dynImpulse,
                        statBias, statMass, statImpulse)
                }
            }
        }

        solverContacts.constraints[Int(colorIndex)] = colorConstraints
        solverContacts.manifoldPool = poolArray
        solverSets[awakeSet].bodyStates = stateArray
    }

    /// The 4-lane push loop, split out so the closure nesting stays shallow.
    private func widePushBody(
        _ cs: UnsafeMutableBufferPointer<ContactConstraint>, _ csCount: Int,
        _ pool: UnsafeMutableBufferPointer<ManifoldConstraint>, _ poolCount: Int,
        _ states: UnsafeMutableBufferPointer<BodyState>, _ stateCount: Int,
        _ range: Range<Int>,
        _ inv_h: FloatW, _ contactSpeedW: FloatW, _ zeroW: FloatW, _ oneW: FloatW,
        _ dynBias: Float, _ dynMass: Float, _ dynImpulse: Float,
        _ statBias: Float, _ statMass: Float, _ statImpulse: Float
    ) {
        _ = csCount
        _ = poolCount
        _ = stateCount

        var batch = range.lowerBound
        while batch < range.upperBound {
            let lanes = min(4, range.upperBound - batch)

            var invMassA = FloatW(repeating: 0)
            var invMassB = FloatW(repeating: 0)
            var invIA = SymMatrix3W()
            var invIB = SymMatrix3W()
            var softMask = FloatW(repeating: 0)
            var idxA = SIMD4<Int32>(repeating: 0)
            var idxB = SIMD4<Int32>(repeating: 0)
            var bA = BodyStateW()
            var bB = BodyStateW()

            for lane in 0..<lanes {
                let c = cs[batch + lane]
                idxA[lane] = c.indexA + 1
                idxB[lane] = c.indexB + 1
                invMassA[lane] = c.invMassA
                invMassB[lane] = c.invMassB
                invIA.xx[lane] = c.invIA.cxx
                invIA.yy[lane] = c.invIA.cyy
                invIA.zz[lane] = c.invIA.czz
                invIA.xy[lane] = c.invIA.cxy
                invIA.xz[lane] = c.invIA.cxz
                invIA.yz[lane] = c.invIA.cyz
                invIB.xx[lane] = c.invIB.cxx
                invIB.yy[lane] = c.invIB.cyy
                invIB.zz[lane] = c.invIB.czz
                invIB.xy[lane] = c.invIB.cxy
                invIB.xz[lane] = c.invIB.cxz
                invIB.yz[lane] = c.invIB.cyz
                softMask[lane] = c.indexA < 0 ? 1 : 0

                let sa = idxA[lane] == 0 ? BodyState.identity : states[Int(idxA[lane] - 1)]
                let sb = idxB[lane] == 0 ? BodyState.identity : states[Int(idxB[lane] - 1)]
                bA.v.x[lane] = sa.linearVelocity.x
                bA.v.y[lane] = sa.linearVelocity.y
                bA.v.z[lane] = sa.linearVelocity.z
                bA.w.x[lane] = sa.angularVelocity.x
                bA.w.y[lane] = sa.angularVelocity.y
                bA.w.z[lane] = sa.angularVelocity.z
                bA.dp.x[lane] = sa.deltaPosition.x
                bA.dp.y[lane] = sa.deltaPosition.y
                bA.dp.z[lane] = sa.deltaPosition.z
                bA.dq.v.x[lane] = sa.deltaRotation.v.x
                bA.dq.v.y[lane] = sa.deltaRotation.v.y
                bA.dq.v.z[lane] = sa.deltaRotation.v.z
                bA.dq.s[lane] = sa.deltaRotation.s

                bB.v.x[lane] = sb.linearVelocity.x
                bB.v.y[lane] = sb.linearVelocity.y
                bB.v.z[lane] = sb.linearVelocity.z
                bB.w.x[lane] = sb.angularVelocity.x
                bB.w.y[lane] = sb.angularVelocity.y
                bB.w.z[lane] = sb.angularVelocity.z
                bB.dp.x[lane] = sb.deltaPosition.x
                bB.dp.y[lane] = sb.deltaPosition.y
                bB.dp.z[lane] = sb.deltaPosition.z
                bB.dq.v.x[lane] = sb.deltaRotation.v.x
                bB.dq.v.y[lane] = sb.deltaRotation.v.y
                bB.dq.v.z[lane] = sb.deltaRotation.v.z
                bB.dq.s[lane] = sb.deltaRotation.s
            }

            let biasRate = selectW(softMask, FloatW(repeating: statBias), FloatW(repeating: dynBias))
            let massScale = selectW(softMask, FloatW(repeating: statMass), FloatW(repeating: dynMass))
            let impulseScale = selectW(
                softMask, FloatW(repeating: statImpulse), FloatW(repeating: dynImpulse))

            let dp = subW(bB.dp, bA.dp)
            let invMass = invMassA + invMassB

            var normalW = Vec3W()
            var pointCount = 0
            for lane in 0..<lanes {
                let cm = pool[Int(cs[batch + lane].manifoldStart)]
                normalW.x[lane] = cm.normal.x
                normalW.y[lane] = cm.normal.y
                normalW.z[lane] = cm.normal.z
                pointCount = max(pointCount, Int(cm.pointCount))
            }

            let normalSeparation = dotW(normalW, dp)
            let normalA = invRotateVecW(bA.dq, normalW)
            let normalB = invRotateVecW(bB.dq, normalW)
            var linearNormalVelocity = dotW(normalW, subW(bB.v, bA.v))
            var manifoldImpulse = zeroW

            for pointIndex in 0..<pointCount {
                var rA = Vec3W(), rB = Vec3W()
                var baseSep = FloatW(repeating: 0)
                var normalMass = FloatW(repeating: 0)
                var normalImpulse = FloatW(repeating: 0)

                for lane in 0..<lanes {
                    let base = Int(cs[batch + lane].manifoldStart)
                    guard pointIndex < Int(pool[base].pointCount) else { continue }
                    let cp = pool[base].points[pointIndex]
                    rA[lane] = cp.rA
                    rB[lane] = cp.rB
                    baseSep[lane] = cp.baseSeparation
                    normalMass[lane] = cp.normalMass
                    normalImpulse[lane] = cp.normalImpulse
                }

                let s = normalSeparation + (dotW(normalB, rB) - dotW(normalA, rA)) + baseSep
                let separated = greaterW(s, zeroW)
                let specBias = s * inv_h
                let overlapBias = maxW(biasRate * s, contactSpeedW)
                let velocityBias = selectW(separated, specBias, overlapBias)
                let pointMassScale = selectW(separated, oneW, massScale)
                let pointImpulseScale = selectW(separated, zeroW, impulseScale)

                let uA = crossW(rA, normalW)
                let uB = crossW(rB, normalW)
                let gA = mulMVW(invIA, uA)
                let gB = mulMVW(invIB, uB)

                let vn = linearNormalVelocity + (dotW(uB, bB.w) - dotW(uA, bA.w))
                let negImpulse =
                    normalMass * (pointMassScale * vn + velocityBias)
                    + pointImpulseScale * normalImpulse
                let impulse = maxW(-negImpulse, -normalImpulse)
                let newImpulse = normalImpulse + impulse

                linearNormalVelocity = linearNormalVelocity + invMass * impulse
                manifoldImpulse = manifoldImpulse + impulse
                bA.w = subW(bA.w, mulW(impulse, gA))
                bB.w = addW(bB.w, mulW(impulse, gB))

                for lane in 0..<lanes {
                    let base = Int(cs[batch + lane].manifoldStart)
                    guard pointIndex < Int(pool[base].pointCount) else { continue }
                    pool[base].points[pointIndex].normalImpulse = newImpulse[lane]
                }
            }

            let P = mulSVW(manifoldImpulse, normalW)
            bA.v = subW(bA.v, mulW(invMassA, P))
            bB.v = addW(bB.v, mulW(invMassB, P))

            for lane in 0..<lanes {
                if idxA[lane] != 0 {
                    let local = Int(idxA[lane] - 1)
                    if (states[local].flags & BodyFlags.dynamicFlag.rawValue) != 0 {
                        states[local].linearVelocity = bA.v[lane]
                        states[local].angularVelocity = bA.w[lane]
                    }
                }
                if idxB[lane] != 0 {
                    let local = Int(idxB[lane] - 1)
                    if (states[local].flags & BodyFlags.dynamicFlag.rawValue) != 0 {
                        states[local].linearVelocity = bB.v[lane]
                        states[local].angularVelocity = bB.w[lane]
                    }
                }
            }

            batch += lanes
        }
    }
}
