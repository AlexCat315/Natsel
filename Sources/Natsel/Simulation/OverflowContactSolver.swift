// The overflow color runs the scalar contact solver in sequential order.

struct OverflowContactConstraint {
    var contactId: Int32 = B3.nullIndex
    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var invMassA: Float = 0
    var invMassB: Float = 0
    var invIA = Matrix3()
    var invIB = Matrix3()
    var rollingMass = Matrix3()
    var softness = Softness()
    var friction: Float = 0
    var restitution: Float = 0
    var rollingResistance: Float = 0
    var manifoldStart = 0
    var manifoldCount = 0
}

extension World {
    func prepareOverflowContacts(_ softness: Softness, _ staticSoftness: Softness) {
        overflowContacts.removeAll(keepingCapacity: true)
        overflowManifolds.removeAll(keepingCapacity: true)
        let specs = constraintGraph.colors[Int(overflowIndex)].contacts
        guard !specs.isEmpty else { return }

        let warmStartScale: Float = enableWarmStarting ? 1 : 0
        let invTau = 1 / B3.speculativeDistance
        let staticFlag = ContactFlags.staticFlag.rawValue
        let hitFlag = ContactFlags.simEnableHitEvent.rawValue
        let awake = Int(SetType.awakeSet.rawValue)
        var anyRestitution = false

        solverSets[awake].bodySims.withUnsafeBufferPointer { sims in
            solverSets[awake].bodyStates.withUnsafeBufferPointer { states in
                contacts.withUnsafeBufferPointer { contactBuffer in
                    for spec in specs {
                        let contact = contactBuffer.baseAddress! + Int(spec.contactId)
                        let indexA = decodeAwakeIndex(contact.pointee.encodedBodySimA)
                        let indexB = decodeAwakeIndex(contact.pointee.encodedBodySimB)

                        var c = OverflowContactConstraint()
                        c.contactId = spec.contactId
                        c.indexA = indexA
                        c.indexB = indexB
                        if indexA >= 0 {
                            c.invMassA = sims[Int(indexA)].invMass
                            c.invIA = sims[Int(indexA)].invInertiaWorld
                        }
                        if indexB >= 0 {
                            c.invMassB = sims[Int(indexB)].invMass
                            c.invIB = sims[Int(indexB)].invInertiaWorld
                        }
                        let mA = c.invMassA
                        let mB = c.invMassB
                        let iA = c.invIA
                        let iB = c.invIB
                        c.rollingMass = invertMatrix(addMM(iA, iB))
                        c.softness = (contact.pointee.flags & staticFlag) != 0 ? staticSoftness : softness
                        c.friction = contact.pointee.friction
                        c.restitution = contact.pointee.restitution
                        c.rollingResistance = contact.pointee.rollingResistance

                        // Only sample contact point normal velocity if needed.
                        let haveRestitution = contact.pointee.restitution > 0
                        let hitEvents = (contact.pointee.flags & hitFlag) != 0
                        let sampleVelocity = haveRestitution || hitEvents
                        anyRestitution = anyRestitution || haveRestitution

                        var vA = Vec3(), wA = Vec3(), vB = Vec3(), wB = Vec3()
                        if sampleVelocity {
                            if indexA >= 0 {
                                vA = states[Int(indexA)].linearVelocity
                                wA = states[Int(indexA)].angularVelocity
                            }
                            if indexB >= 0 {
                                vB = states[Int(indexB)].linearVelocity
                                wB = states[Int(indexB)].angularVelocity
                            }
                        }

                        let manifoldCount = Int(contact.pointee.manifoldCount)
                        c.manifoldStart = overflowManifolds.count
                        c.manifoldCount = manifoldCount
                        if manifoldCount > 0 {
                            let manifolds = UnsafeMutablePointer(mutating: contact.pointee.manifolds.buffer.baseAddress!)
                            for manifoldIndex in 0..<manifoldCount {
                                let manifold = manifolds + manifoldIndex
                                var constraint = ManifoldConstraint()
                                let pointCount = Int(manifold.pointee.pointCount)
                                let normal = manifold.pointee.normal
                                let tangent1 = perp(normal)
                                let tangent2 = cross(tangent1, normal)

                                constraint.pointCount = Int32(pointCount)
                                constraint.normal = normal
                                constraint.tangent1 = tangent1
                                constraint.tangent2 = tangent2
                                constraint.tangentVelocity1 = dot(contact.pointee.tangentVelocity, tangent1)
                                constraint.tangentVelocity2 = dot(contact.pointee.tangentVelocity, tangent2)

                                var centerA = Vec3()
                                var centerB = Vec3()
                                var totalFrictionWeight: Float = 0

                                for pointIndex in 0..<pointCount {
                                    var cp = ConstraintPoint()
                                    let mp = manifold.pointee.points[pointIndex]
                                    cp.rA = mp.anchorA
                                    cp.rB = mp.anchorB

                                    let s = mp.separation
                                    cp.baseSeparation = s - dot(cp.rB - cp.rA, normal)
                                    cp.normalImpulse = warmStartScale * mp.normalImpulse
                                    cp.totalNormalImpulse = 0
                                    cp.restitutionImpulse = 0

                                    let rA = cp.rA
                                    let rB = cp.rB
                                    let rnA = cross(rA, normal)
                                    let rnB = cross(rB, normal)
                                    let kNormal = mA + mB + dot(rnA, mulMV(iA, rnA)) + dot(rnB, mulMV(iB, rnB))
                                    cp.normalMass = kNormal > 0 ? 1 / kNormal : 0

                                    if sampleVelocity {
                                        let vrA = vA + cross(wA, rA)
                                        let vrB = vB + cross(wB, rB)
                                        let vn = dot(normal, vrB - vrA)
                                        cp.relativeVelocity = vn
                                        manifold.pointee.points[pointIndex].normalVelocity = hitEvents ? vn : 0
                                    } else {
                                        cp.relativeVelocity = 0
                                        manifold.pointee.points[pointIndex].normalVelocity = 0
                                    }

                                    let weight = clampFloat(2 - s * invTau, B3.minFrictionWeight, 1)
                                    centerA = mulAdd(centerA, weight, rA)
                                    centerB = mulAdd(centerB, weight, rB)
                                    totalFrictionWeight += weight
                                    constraint.points[pointIndex] = cp
                                }

                                let invWeight = 1 / totalFrictionWeight
                                centerA = invWeight * centerA
                                centerB = invWeight * centerB
                                constraint.centerA = centerA
                                constraint.centerB = centerB

                                for pointIndex in 0..<pointCount {
                                    constraint.points[pointIndex].leverArm = distance(constraint.points[pointIndex].rA, centerA)
                                }

                                let rtA1 = cross(centerA, tangent1)
                                let rtA2 = cross(centerA, tangent2)
                                let rtB1 = cross(centerB, tangent1)
                                let rtB2 = cross(centerB, tangent2)

                                var k = Matrix2()
                                k.cx.x = mA + mB + dot(rtA1, mulMV(iA, rtA1)) + dot(rtB1, mulMV(iB, rtB1))
                                k.cy.y = mA + mB + dot(rtA2, mulMV(iA, rtA2)) + dot(rtB2, mulMV(iB, rtB2))
                                k.cx.y = dot(rtA1, mulMV(iA, rtA2)) + dot(rtB1, mulMV(iB, rtB2))
                                k.cy.x = k.cx.y
                                constraint.tangentMass = invert2(k)
                                constraint.frictionImpulse = Vec2(
                                    warmStartScale * dot(manifold.pointee.frictionImpulse, tangent1),
                                    warmStartScale * dot(manifold.pointee.frictionImpulse, tangent2))

                                let kTwist = dot(normal, mulMV(addMM(iA, iB), normal))
                                constraint.twistMass = kTwist > 0 ? 1 / kTwist : 0
                                constraint.twistImpulse = warmStartScale * manifold.pointee.twistImpulse
                                constraint.rollingImpulse = warmStartScale * manifold.pointee.rollingImpulse

                                overflowManifolds.append(constraint)
                            }
                        }
                        overflowContacts.append(c)
                    }
                }
            }
        }

        if anyRestitution {
            solveAnyRestitution = true
        }
    }

    @inline(__always)
    private func withOverflowContacts(
        _ body: (UnsafeMutablePointer<OverflowContactConstraint>, Int,
                 UnsafeMutablePointer<ManifoldConstraint>, UnsafeMutableBufferPointer<BodyState>) -> Void
    ) {
        guard !overflowContacts.isEmpty else { return }
        withSolverStates { states in
            overflowContacts.withUnsafeMutableBufferPointer { list in
                overflowManifolds.withUnsafeMutableBufferPointer { pool in
                    body(list.baseAddress!, list.count, pool.baseAddress ?? UnsafeMutablePointer(bitPattern: 16)!, states)
                }
            }
        }
    }

    func warmStartOverflowContacts() {
        withOverflowContacts { list, count, pool, states in
            let dynamicFlag = BodyFlags.dynamicFlag.rawValue
            for i in 0..<count {
                let c = list + i
                var linearImpulse = Vec3()
                var angularImpulseA = Vec3()
                var angularImpulseB = Vec3()

                for manifoldIndex in 0..<c.pointee.manifoldCount {
                    let constraint = pool + c.pointee.manifoldStart + manifoldIndex
                    let normal = constraint.pointee.normal
                    var totalNormalImpulse: Float = 0
                    var momentA = Vec3()
                    var momentB = Vec3()
                    for j in 0..<Int(constraint.pointee.pointCount) {
                        let cp = constraint.pointee.points[j]
                        totalNormalImpulse += cp.normalImpulse
                        momentA = mulAdd(momentA, cp.normalImpulse, cp.rA)
                        momentB = mulAdd(momentB, cp.normalImpulse, cp.rB)
                    }

                    var frictionImpulse = constraint.pointee.frictionImpulse.x * constraint.pointee.tangent1
                    frictionImpulse = mulAdd(frictionImpulse, constraint.pointee.frictionImpulse.y, constraint.pointee.tangent2)

                    linearImpulse = linearImpulse + mulAdd(frictionImpulse, totalNormalImpulse, normal)

                    let twistImpulse = constraint.pointee.twistImpulse * normal
                    let angularA = (cross(momentA, normal) + cross(constraint.pointee.centerA, frictionImpulse)) + twistImpulse
                    let angularB = (cross(momentB, normal) + cross(constraint.pointee.centerB, frictionImpulse)) + twistImpulse

                    angularImpulseA = angularImpulseA + (angularA + constraint.pointee.rollingImpulse)
                    angularImpulseB = angularImpulseB + (angularB + constraint.pointee.rollingImpulse)
                }

                let indexA = c.pointee.indexA
                let indexB = c.pointee.indexB
                if indexA >= 0 && (states[Int(indexA)].flags & dynamicFlag) != 0 {
                    states[Int(indexA)].linearVelocity = mulSub(states[Int(indexA)].linearVelocity, c.pointee.invMassA, linearImpulse)
                    states[Int(indexA)].angularVelocity =
                        states[Int(indexA)].angularVelocity - mulMV(c.pointee.invIA, angularImpulseA)
                }
                if indexB >= 0 && (states[Int(indexB)].flags & dynamicFlag) != 0 {
                    states[Int(indexB)].linearVelocity = mulAdd(states[Int(indexB)].linearVelocity, c.pointee.invMassB, linearImpulse)
                    states[Int(indexB)].angularVelocity =
                        states[Int(indexB)].angularVelocity + mulMV(c.pointee.invIB, angularImpulseB)
                }
            }
        }
    }

    func pushOverflowContacts(_ context: StepContext) {
        let inv_h = context.inv_h
        let contactSpeed = self.contactSpeed
        withOverflowContacts { list, count, pool, states in
            for i in 0..<count {
                let c = list + i
                let mA = c.pointee.invMassA
                let iA = c.pointee.invIA
                let mB = c.pointee.invMassB
                let iB = c.pointee.invIB
                let stateA = World.stateAt(states, c.pointee.indexA)
                let stateB = World.stateAt(states, c.pointee.indexB)
                var vA = stateA.linearVelocity
                var wA = stateA.angularVelocity
                let dqA = stateA.deltaRotation
                var vB = stateB.linearVelocity
                var wB = stateB.angularVelocity
                let dqB = stateB.deltaRotation
                let dp = stateB.deltaPosition - stateA.deltaPosition
                let softness = c.pointee.softness

                for j in 0..<c.pointee.manifoldCount {
                    let constraint = pool + c.pointee.manifoldStart + j
                    let normal = constraint.pointee.normal
                    for pointIndex in 0..<Int(constraint.pointee.pointCount) {
                        let rA = constraint.pointee.points[pointIndex].rA
                        let rB = constraint.pointee.points[pointIndex].rB

                        let ds = dp + (rotateVector(dqB, rB) - rotateVector(dqA, rA))
                        let s = dot(ds, normal) + constraint.pointee.points[pointIndex].baseSeparation

                        let velocityBias: Float
                        let massScale: Float
                        let impulseScale: Float
                        if s > 0 {
                            velocityBias = s * inv_h
                            massScale = 1
                            impulseScale = 0
                        } else {
                            velocityBias = maxFloat(softness.massScale * softness.biasRate * s, -contactSpeed)
                            massScale = softness.massScale
                            impulseScale = softness.impulseScale
                        }

                        let vrA = vA + cross(wA, rA)
                        let vrB = vB + cross(wB, rB)
                        let vn = dot(vrB - vrA, normal)

                        let normalImpulse = constraint.pointee.points[pointIndex].normalImpulse
                        var deltaImpulse = -constraint.pointee.points[pointIndex].normalMass * (massScale * vn + velocityBias)
                            - impulseScale * normalImpulse
                        let newImpulse = maxFloat(normalImpulse + deltaImpulse, 0)
                        deltaImpulse = newImpulse - normalImpulse
                        constraint.pointee.points[pointIndex].normalImpulse = newImpulse

                        let P = deltaImpulse * normal
                        vA = mulSub(vA, mA, P)
                        wA = wA - mulMV(iA, cross(rA, P))
                        vB = mulAdd(vB, mB, P)
                        wB = wB + mulMV(iB, cross(rB, P))
                    }
                }

                Self.storeOverflowVelocity(states, c.pointee.indexA, stateA.flags, vA, wA)
                Self.storeOverflowVelocity(states, c.pointee.indexB, stateB.flags, vB, wB)
            }
        }
    }

    func solveOverflowContacts(_ context: StepContext) {
        let inv_h = context.inv_h
        withOverflowContacts { list, count, pool, states in
            for i in 0..<count {
                let c = list + i
                let mA = c.pointee.invMassA
                let iA = c.pointee.invIA
                let mB = c.pointee.invMassB
                let iB = c.pointee.invIB
                let stateA = World.stateAt(states, c.pointee.indexA)
                let stateB = World.stateAt(states, c.pointee.indexB)
                var vA = stateA.linearVelocity
                var wA = stateA.angularVelocity
                let dqA = stateA.deltaRotation
                var vB = stateB.linearVelocity
                var wB = stateB.angularVelocity
                let dqB = stateB.deltaRotation
                let dp = stateB.deltaPosition - stateA.deltaPosition
                let friction = c.pointee.friction
                let rollingResistance = c.pointee.rollingResistance

                for j in 0..<c.pointee.manifoldCount {
                    let constraint = pool + c.pointee.manifoldStart + j
                    let normal = constraint.pointee.normal
                    var totalNormalImpulse: Float = 0
                    var totalTwistLimit: Float = 0

                    for pointIndex in 0..<Int(constraint.pointee.pointCount) {
                        let rA = constraint.pointee.points[pointIndex].rA
                        let rB = constraint.pointee.points[pointIndex].rB

                        let ds = dp + (rotateVector(dqB, rB) - rotateVector(dqA, rA))
                        let s = dot(ds, normal) + constraint.pointee.points[pointIndex].baseSeparation

                        // speculative bias, zero when overlapped
                        let velocityBias: Float = s > 0 ? s * inv_h : 0

                        let vrA = vA + cross(wA, rA)
                        let vrB = vB + cross(wB, rB)
                        let vn = dot(vrB - vrA, normal)

                        let normalImpulse = constraint.pointee.points[pointIndex].normalImpulse
                        var deltaImpulse = -constraint.pointee.points[pointIndex].normalMass * (vn + velocityBias)
                        let newImpulse = maxFloat(normalImpulse + deltaImpulse, 0)
                        deltaImpulse = newImpulse - normalImpulse
                        constraint.pointee.points[pointIndex].normalImpulse = newImpulse
                        constraint.pointee.points[pointIndex].totalNormalImpulse += newImpulse
                        totalNormalImpulse += newImpulse
                        totalTwistLimit += constraint.pointee.points[pointIndex].leverArm * newImpulse

                        let P = deltaImpulse * normal
                        vA = mulSub(vA, mA, P)
                        wA = wA - mulMV(iA, cross(rA, P))
                        vB = mulAdd(vB, mB, P)
                        wB = wB + mulMV(iB, cross(rB, P))
                    }

                    // Central twist friction
                    do {
                        let twistSpeed = dot(normal, wB - wA)
                        let maxImpulse = friction * totalTwistLimit
                        var deltaImpulse = -constraint.pointee.twistMass * twistSpeed
                        let oldImpulse = constraint.pointee.twistImpulse
                        constraint.pointee.twistImpulse = clampFloat(oldImpulse + deltaImpulse, -maxImpulse, maxImpulse)
                        deltaImpulse = constraint.pointee.twistImpulse - oldImpulse

                        wA = wA - mulMV(iA, deltaImpulse * normal)
                        wB = wB + mulMV(iB, deltaImpulse * normal)
                    }

                    // Rolling resistance
                    if rollingResistance > 0 {
                        var deltaImpulse = -mulMV(c.pointee.rollingMass, wB - wA)
                        let oldImpulse = constraint.pointee.rollingImpulse
                        constraint.pointee.rollingImpulse = oldImpulse + deltaImpulse

                        let maxImpulse = rollingResistance * totalNormalImpulse
                        let magSqr = dot(constraint.pointee.rollingImpulse, constraint.pointee.rollingImpulse)
                        if magSqr > maxImpulse * maxImpulse + Float.ulpOfOne {
                            constraint.pointee.rollingImpulse =
                                (maxImpulse / magSqr.squareRoot()) * constraint.pointee.rollingImpulse
                        }

                        deltaImpulse = constraint.pointee.rollingImpulse - oldImpulse
                        wA = wA - mulMV(iA, deltaImpulse)
                        wB = wB + mulMV(iB, deltaImpulse)
                    }

                    // Central friction
                    do {
                        let tangent1 = constraint.pointee.tangent1
                        let tangent2 = constraint.pointee.tangent2
                        let rA = constraint.pointee.centerA
                        let rB = constraint.pointee.centerB

                        let vrA = vA + cross(wA, rA)
                        let vrB = vB + cross(wB, rB)
                        let vr = vrB - vrA
                        let vt = Vec2(
                            dot(vr, tangent1) - constraint.pointee.tangentVelocity1,
                            dot(vr, tangent2) - constraint.pointee.tangentVelocity2)

                        let tm = mulMV2(constraint.pointee.tangentMass, vt)
                        var deltaImpulse = Vec2(-tm.x, -tm.y)
                        var newImpulse = Vec2(
                            constraint.pointee.frictionImpulse.x + deltaImpulse.x,
                            constraint.pointee.frictionImpulse.y + deltaImpulse.y)

                        let maxImpulse = friction * totalNormalImpulse
                        let lengthSquared = dot2(newImpulse, newImpulse)
                        if lengthSquared > maxImpulse * maxImpulse {
                            let scale = maxImpulse / lengthSquared.squareRoot()
                            newImpulse.x *= scale
                            newImpulse.y *= scale
                        }
                        deltaImpulse = sub2(newImpulse, constraint.pointee.frictionImpulse)
                        constraint.pointee.frictionImpulse = newImpulse

                        let P = blend2(deltaImpulse.x, tangent1, deltaImpulse.y, tangent2)
                        vA = mulSub(vA, mA, P)
                        wA = wA - mulMV(iA, cross(rA, P))
                        vB = mulAdd(vB, mB, P)
                        wB = wB + mulMV(iB, cross(rB, P))
                    }
                }

                Self.storeOverflowVelocity(states, c.pointee.indexA, stateA.flags, vA, wA)
                Self.storeOverflowVelocity(states, c.pointee.indexB, stateB.flags, vB, wB)
            }
        }
    }

    func applyOverflowRestitution(_ context: StepContext) {
        let threshold = restitutionThreshold
        let inv_h = context.inv_h
        let propagate = enableRestitutionPropagation
        withOverflowContacts { list, count, pool, states in
            for i in 0..<count {
                let c = list + i
                let restitution = c.pointee.restitution
                if propagate == false && restitution == 0 {
                    continue
                }

                let mA = c.pointee.invMassA
                let iA = c.pointee.invIA
                let mB = c.pointee.invMassB
                let iB = c.pointee.invIB
                let stateA = World.stateAt(states, c.pointee.indexA)
                let stateB = World.stateAt(states, c.pointee.indexB)
                var vA = stateA.linearVelocity
                var wA = stateA.angularVelocity
                let dqA = stateA.deltaRotation
                var vB = stateB.linearVelocity
                var wB = stateB.angularVelocity
                let dqB = stateB.deltaRotation
                let dp = stateB.deltaPosition - stateA.deltaPosition

                for j in 0..<c.pointee.manifoldCount {
                    let constraint = pool + c.pointee.manifoldStart + j
                    let normal = constraint.pointee.normal
                    for pointIndex in 0..<Int(constraint.pointee.pointCount) {
                        let cp = constraint.pointee.points[pointIndex]
                        let rA = cp.rA
                        let rB = cp.rB

                        // The total normal impulse is 0 for speculative points.
                        let compressionImpulse = cp.totalNormalImpulse - cp.restitutionImpulse
                        let armed = restitution > 0 && cp.relativeVelocity < -threshold && compressionImpulse > 0

                        let velocityBias: Float
                        if armed {
                            velocityBias = restitution * cp.relativeVelocity
                        } else {
                            let ds = dp + (rotateVector(dqB, rB) - rotateVector(dqA, rA))
                            let s = dot(ds, normal) + cp.baseSeparation
                            velocityBias = s > 0 ? s * inv_h : 0
                        }

                        let vrA = vA + cross(wA, rA)
                        let vrB = vB + cross(wB, rB)
                        let vn = dot(vrB - vrA, normal)

                        var impulse = -cp.normalMass * (vn + velocityBias)
                        let newImpulse = maxFloat(cp.normalImpulse + impulse, 0)
                        impulse = newImpulse - cp.normalImpulse

                        let approachImpulse = minFloat(maxFloat(-cp.normalMass * vn, 0), maxFloat(impulse, 0))

                        if armed {
                            // Poisson kinetic restitution guarantees no energy gain.
                            let allowance = restitution * (compressionImpulse + approachImpulse) - cp.restitutionImpulse
                            impulse = minFloat(impulse, approachImpulse + maxFloat(allowance, 0))
                        }

                        constraint.pointee.points[pointIndex].normalImpulse += impulse
                        constraint.pointee.points[pointIndex].restitutionImpulse += impulse - approachImpulse
                        constraint.pointee.points[pointIndex].totalNormalImpulse += impulse

                        let P = impulse * normal
                        vA = mulSub(vA, mA, P)
                        wA = wA - mulMV(iA, cross(rA, P))
                        vB = mulAdd(vB, mB, P)
                        wB = wB + mulMV(iB, cross(rB, P))
                    }
                }

                Self.storeOverflowVelocity(states, c.pointee.indexA, stateA.flags, vA, wA)
                Self.storeOverflowVelocity(states, c.pointee.indexB, stateB.flags, vB, wB)
            }
        }
    }

    func storeOverflowImpulses() {
        guard !overflowContacts.isEmpty else { return }
        overflowManifolds.withUnsafeBufferPointer { pool in
            contacts.withUnsafeBufferPointer { store in
                for c in overflowContacts where c.manifoldCount > 0 {
                    let contact = store.baseAddress! + Int(c.contactId)
                    let manifolds = UnsafeMutablePointer(mutating: contact.pointee.manifolds.buffer.baseAddress!)
                    for manifoldIndex in 0..<c.manifoldCount {
                        let manifold = manifolds + manifoldIndex
                        let constraint = pool[c.manifoldStart + manifoldIndex]
                        manifold.pointee.twistImpulse = constraint.twistImpulse
                        manifold.pointee.frictionImpulse = blend2(
                            constraint.frictionImpulse.x, constraint.tangent1,
                            constraint.frictionImpulse.y, constraint.tangent2)
                        manifold.pointee.rollingImpulse = constraint.rollingImpulse
                        for pointIndex in 0..<Int(constraint.pointCount) {
                            manifold.pointee.points[pointIndex].normalImpulse = constraint.points[pointIndex].normalImpulse
                            manifold.pointee.points[pointIndex].totalNormalImpulse =
                                constraint.points[pointIndex].totalNormalImpulse
                        }
                    }
                }
            }
        }
    }

    @inline(__always)
    static func storeOverflowVelocity(
        _ states: UnsafeMutableBufferPointer<BodyState>, _ index: Int32, _ flags: UInt32, _ v: Vec3, _ w: Vec3
    ) {
        if index >= 0 && (flags & BodyFlags.dynamicFlag.rawValue) != 0 {
            states[Int(index)].linearVelocity = v
            states[Int(index)].angularVelocity = w
        }
    }
}
