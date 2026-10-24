

internal // Solver integration, finalization, and a contact-free step (solver.c).

/// Constraint softness.
struct Softness: BitwiseCopyable {
    var biasRate: Float = 0
    var massScale: Float = 0
    var impulseScale: Float = 0

    init() {}
}

/// Make a softness from a Hertz frequency and damping ratio.
@inline(__always)
func makeSoft(_ hertz: Float, _ zeta: Float, _ h: Float) -> Softness {
    if hertz == 0 {
        return Softness()
    }

    let omega = 2.0 * B3.pi * hertz
    let a1 = 2.0 * zeta + h * omega
    let a2 = h * omega * a1
    let a3 = 1.0 / (1.0 + a2)

    var softness = Softness()
    softness.biasRate = omega / a1
    softness.massScale = a2 * a3
    softness.impulseScale = a3
    return softness
}

/// Per-thread task storage.
struct TaskContext {
    /// Broad-phase pairs.
    var pairKeys: [UInt64] = []

    /// Used to put islands to sleep.
    var awakeIslandBitSet = BitSet()

    /// Per worker split island candidate.
    var splitSleepTime: Float = 0
    var splitIslandId: Int32 = B3.nullIndex

    /// Continuous collision sensor hits.
    var sensorHits: [(sensorId: Int32, visitorId: Int32)] = []

    /// Number of contacts recycled this step.
    var recycledContactCount: Int32 = 0

    /// Per-worker SAT diagnostics, merged after the collide pass.
    var satCallCount: Int32 = 0
    var satCacheHitCount: Int32 = 0

    /// Contact ids whose state changed this pass.
    var contactStateBits = BitSet()

    /// Diagnostic iteration counts.
    var distanceIterations: Int32 = 0
    var pushBackIterations: Int32 = 0
    var rootIterations: Int32 = 0

    init() {}
}

extension World {
    // MARK: Velocity integration

    /// Integrate velocities, apply damping, and apply gyroscopic torque.
    func integrateVelocities(_ h: Float) {
        let (simsBase, simsCapacity) = solverSets[Int(SetType.awakeSet.rawValue)].bodySims
            .withUnsafeBufferPointer { ($0.baseAddress, $0.count) }
        let (statesBase, statesCapacity) = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates
            .withUnsafeMutableBufferPointer { ($0.baseAddress, $0.count) }
        guard let simsBase, let statesBase else { return }
        let sims = UnsafeBufferPointer(start: simsBase, count: simsCapacity)
        let states = UnsafeMutableBufferPointer(start: statesBase, count: statesCapacity)
        let count = sims.count

        for i in 0..<count {
            let sim = sims[i]
            var state = states[i]

            var v = state.linearVelocity
            var w = state.angularVelocity

            // Damping math: v2 = v1 * 1 / (1 + c * dt)
            let linearDamping = 1.0 / (1.0 + h * sim.linearDamping)
            let angularDamping = 1.0 / (1.0 + h * sim.angularDamping)

            // Gravity scale will be zero for kinematic bodies
            let gravityScale = sim.invMass > 0 ? sim.gravityScale : 0

            let linearVelocityDelta = blend2(h * sim.invMass, sim.force, h * gravityScale, gravity)
            v = mulAdd(linearVelocityDelta, linearDamping, v)

            let angularVelocityDelta = h * mulMV(sim.invInertiaWorld, sim.torque)
            w = mulAdd(angularVelocityDelta, angularDamping, w)

            // Gyroscopic torque by solving this nonlinear equation using Newton-Raphson.
            // I * (w2 - w1) + h * cross(w2, I * w2) = 0
            if (sim.flags & BodyFlags.enableGyroscopicTorque.rawValue) != 0 {
                // Get current rotation.
                let q0 = sim.transform.q
                let q = mulQuat(state.deltaRotation, q0)

                // Compute local angular velocity
                let omega1 = invRotateVector(q, w)
                var omega2 = omega1

                // Symmetric inertia tensor: 6 unique entries (column-major)
                let i00 = sim.inertia.cx.x
                let i01 = sim.inertia.cy.x
                let i02 = sim.inertia.cz.x
                let i11 = sim.inertia.cy.y
                let i12 = sim.inertia.cz.y
                let i22 = sim.inertia.cz.z

                for _ in 0..<Int(B3.gyroscopicIterations) {
                    let w1 = omega2.x
                    let w2 = omega2.y
                    let w3 = omega2.z

                    // Iw = I * omega2 (shared between residual and Jacobian)
                    let Iw1 = i00 * w1 + i01 * w2 + i02 * w3
                    let Iw2 = i01 * w1 + i11 * w2 + i12 * w3
                    let Iw3 = i02 * w1 + i12 * w2 + i22 * w3

                    // Residual: b = I * (omega2 - omega1) + h * cross(omega2, I * omega2)
                    let dw = omega2 - omega1
                    let b = Vec3(
                        i00 * dw.x + i01 * dw.y + i02 * dw.z + h * (w2 * Iw3 - w3 * Iw2),
                        i01 * dw.x + i11 * dw.y + i12 * dw.z + h * (w3 * Iw1 - w1 * Iw3),
                        i02 * dw.x + i12 * dw.y + i22 * dw.z + h * (w1 * Iw2 - w2 * Iw1)
                    )

                    // Jacobian J = I + h * (skew(omega2) * I - skew(I * omega2))
                    let J = Matrix3(
                        Vec3(
                            i00 + h * (w2 * i02 - w3 * i01),
                            i01 + h * (w3 * i00 - w1 * i02 - Iw3),
                            i02 + h * (w1 * i01 - w2 * i00 + Iw2)
                        ),
                        Vec3(
                            i01 + h * (w2 * i12 - w3 * i11 + Iw3),
                            i11 + h * (w3 * i01 - w1 * i12),
                            i12 + h * (w1 * i11 - w2 * i01 - Iw1)
                        ),
                        Vec3(
                            i02 + h * (w2 * i22 - w3 * i12 - Iw2),
                            i12 + h * (w3 * i02 - w1 * i22 + Iw1),
                            i22 + h * (w1 * i12 - w2 * i02)
                        )
                    )

                    omega2 = omega2 - solve3(J, b)
                }

                w = rotateVector(q, omega2)
            }

            state.linearVelocity = v
            state.angularVelocity = w
            states[i] = state
        }
    }

    // MARK: Position integration

    /// Integrate positions and apply motion locks.
    func integratePositions(_ h: Float, _ inv_dt: Float) {
        let (statesBase, statesCapacity) = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates
            .withUnsafeMutableBufferPointer { ($0.baseAddress, $0.count) }
        guard let statesBase else { return }
        let states = UnsafeMutableBufferPointer(start: statesBase, count: statesCapacity)
        let count = states.count

        let maxLinearSpeed = maxLinearSpeed
        let maxAngularSpeed = B3.maxRotation * inv_dt
        let maxLinearSpeedSquared = maxLinearSpeed * maxLinearSpeed
        let maxAngularSpeedSquared = maxAngularSpeed * maxAngularSpeed

        for i in 0..<count {
            var state = states[i]

            var v = state.linearVelocity
            var w = state.angularVelocity

            // Motion locks - these can be viewed as a constraint that comes last
            v.x = (state.flags & BodyFlags.lockLinearX.rawValue) != 0 ? 0 : v.x
            v.y = (state.flags & BodyFlags.lockLinearY.rawValue) != 0 ? 0 : v.y
            v.z = (state.flags & BodyFlags.lockLinearZ.rawValue) != 0 ? 0 : v.z
            w.x = (state.flags & BodyFlags.lockAngularX.rawValue) != 0 ? 0 : w.x
            w.y = (state.flags & BodyFlags.lockAngularY.rawValue) != 0 ? 0 : w.y
            w.z = (state.flags & BodyFlags.lockAngularZ.rawValue) != 0 ? 0 : w.z

            // Clamp to max linear speed
            if dot(v, v) > maxLinearSpeedSquared {
                let ratio = maxLinearSpeed / length(v)
                v = ratio * v
                state.flags |= BodyFlags.isSpeedCapped.rawValue
            }

            // Clamp to max angular speed
            if dot(w, w) > maxAngularSpeedSquared && (state.flags & BodyFlags.allowFastRotation.rawValue) == 0 {
                let ratio = maxAngularSpeed / length(w)
                w = ratio * w
                state.flags |= BodyFlags.isSpeedCapped.rawValue
            }

            state.linearVelocity = v
            state.angularVelocity = w
            state.deltaPosition = mulAdd(state.deltaPosition, h, v)
            state.deltaRotation = integrateRotation(state.deltaRotation, h * w)

            states[i] = state
        }
    }

    // MARK: Finalize bodies

    /// Update the body transforms from the state deltas.
    func finalizeBodies(_ context: inout StepContext) {
        var awakeSet = solverSets[Int(SetType.awakeSet.rawValue)]
        let count = awakeSet.bodySims.count

        let enableSleep = enableSleep
        let enableContinuous = enableContinuous
        let timeStep = context.dt
        let invTimeStep = context.inv_dt
        let worldId = worldId

        let speculativeScalar = B3.speculativeDistance

        bodyMoveEvents = Array(repeating: BodyMoveEvent(), count: count)

        for simIndex in 0..<count {
            var state = awakeSet.bodyStates[simIndex]
            var sim = awakeSet.bodySims[simIndex]

            let v = state.linearVelocity
            let w = state.angularVelocity
            let localOmega = invRotateVector(sim.transform.q, w)
            let localDeltaRotation = invRotateVector(sim.transform.q, state.deltaRotation.v)

            b3Assert(isValidVec3(v))
            b3Assert(isValidVec3(w))

            sim.center = offsetPos(sim.center, state.deltaPosition)
            sim.transform.q = normalizeQuat(mulQuat(state.deltaRotation, sim.transform.q))

            // Use the velocity of the farthest point on the body to account for rotation.
            let velocityArc = modifiedCross(absVec3(localOmega), sim.maxExtent)
            let maxVelocity = length(v) + length(velocityArc)

            // Sleep needs to observe position correction as well as true velocity.
            let rotationArc = modifiedCross(absVec3(localDeltaRotation), sim.maxExtent)
            let maxDeltaPosition = length(state.deltaPosition) + 2.0 * length(rotationArc)

            // Position correction is not as important for sleep as true velocity.
            let positionSleepFactor: Float = 0.5
            let sleepVelocity = maxFloat(maxVelocity, positionSleepFactor * invTimeStep * maxDeltaPosition)

            // reset state deltas
            state.deltaPosition = Vec3()
            state.deltaRotation = Quat.identity

            sim.transform.p = offsetPos(sim.center, -rotateVector(sim.transform.q, sim.localCenter))

            var body = bodies[Int(sim.bodyId)]
            body.bodyMoveIndex = Int32(simIndex)
            body.sleepVelocity = sleepVelocity

            var moveEvent = BodyMoveEvent()
            moveEvent.userData = body.userData
            moveEvent.transform = sim.transform
            moveEvent.bodyId = BodyID(index1: sim.bodyId + 1, world0: worldId, generation: body.generation)
            moveEvent.fellAsleep = false
            bodyMoveEvents[simIndex] = moveEvent

            // reset applied force and torque
            sim.force = Vec3()
            sim.torque = Vec3()

            b3Assert((body.flags & BodyFlags.dirtyMass.rawValue) == 0)

            // Clear the transient flags.
            body.flags &= ~BodyFlags.bodyTransientFlags.rawValue
            sim.flags &= ~(BodyFlags.isFast.rawValue | BodyFlags.bodyTransientFlags.rawValue)

            // The body state flag knows about speed capping.
            body.flags |= (state.flags & BodyFlags.isSpeedCapped.rawValue)
            state.flags &= ~BodyFlags.bodyTransientFlags.rawValue

            if enableSleep == false || (body.flags & BodyFlags.enableSleep.rawValue) == 0
                || sleepVelocity > body.sleepThreshold
            {
                // Body is not sleepy
                body.sleepTime = 0

                let safetyFactor = body.safetyFactor
                let maxMotion = maxFloat(maxDeltaPosition, maxVelocity * timeStep)
                if body.type == .dynamicBody && enableContinuous && maxMotion > safetyFactor * sim.minExtent {
                    // This flag is used for debug draw and contact recycling.
                    sim.flags |= BodyFlags.isFast.rawValue

                    // Flush this body so the continuous pass sees its current sim and state.
                    awakeSet.bodyStates[simIndex] = state
                    awakeSet.bodySims[simIndex] = sim
                    bodies[Int(sim.bodyId)] = body

                    // Store in fast array for the continuous collision stage
                    if (sim.flags & BodyFlags.isBullet.rawValue) != 0 {
                        context.bulletBodies.append(Int32(simIndex))
                    } else {
                        solveContinuous(Int32(simIndex), timeStep, &awakeSet)
                    }

                    // The continuous pass may have advanced the body and changed its state.
                    state = awakeSet.bodyStates[simIndex]
                    sim = awakeSet.bodySims[simIndex]
                    body = bodies[Int(sim.bodyId)]
                } else {
                    // Body is safe to advance
                    sim.center0 = sim.center
                    sim.rotation0 = sim.transform.q
                }
            } else {
                // Body is safe to advance and is falling asleep
                sim.center0 = sim.center
                sim.rotation0 = sim.transform.q
                body.sleepTime += timeStep
            }

            // Update world space inverse inertia tensor.
            let rotationMatrix = makeMatrixFromQuat(sim.transform.q)
            sim.invInertiaWorld = mulMM(mulMM(rotationMatrix, sim.invInertiaLocal), transpose(rotationMatrix))

            // Any single body in an island can keep it awake
            let island = islands[Int(body.islandId)]
            if body.sleepTime < B3.timeToSleep {
                // keep island awake
                let islandIndex = island.localIndex
                b3Assert(islandIndex >= 0 && Int(islandIndex) < awakeSet.islandSims.count)
                if islandIndex >= 0 && Int(islandIndex) < awakeSet.islandSims.count {
                    taskContexts[0].awakeIslandBitSet.setBitGrow(UInt32(islandIndex))
                }
            } else if island.constraintRemoveCount > 0 {
                var taskContext = taskContexts[0]
                if body.sleepTime > taskContext.splitSleepTime
                    || (body.sleepTime == taskContext.splitSleepTime && body.islandId > taskContext.splitIslandId)
                {
                    // pick the sleepiest candidate
                    taskContext.splitIslandId = body.islandId
                    taskContext.splitSleepTime = body.sleepTime
                    taskContexts[0] = taskContext
                }
            }

            bodies[Int(sim.bodyId)] = body

            // Update shapes AABBs
            // For fast non-bullet bodies the AABB has already been updated in the CCD pass.
            if (sim.flags & BodyFlags.isFast.rawValue) == 0 {
                let transform = sim.transform
                var shapeId = body.headShapeId
                // Mutate the shape in place. A `var shape = shapes[...]` copy plus a store back makes
                // the whole world shape array multiply referenced, so the next store copies the
                // entire array. With thousands of bodies that copy dominated the step.
                shapes.withUnsafeMutableBufferPointer { shapeStore in
                    while shapeId != B3.nullIndex {
                        let shapeIndex = Int(shapeId)
                        let nextShapeId = shapeStore[shapeIndex].nextShapeId

                        let aabb = computeFatShapeAABB(shapeStore[shapeIndex], transform, speculativeScalar)
                        shapeStore[shapeIndex].aabb = aabb

                        let fatAABB = fatAABBs[shapeIndex]
                        if aabbContains(fatAABB, aabb) == false {
                            let inflated = aabbInflate(aabb, shapeStore[shapeIndex].aabbMargin)
                            fatAABBs[shapeIndex] = inflated

                            // Mark the hierarchy as moved.
                            broadPhase.markProxyMoved(shapeStore[shapeIndex].proxyKey, inflated)
                        }

                        shapeId = nextShapeId
                    }
                }
            }

            awakeSet.bodyStates[simIndex] = state
            awakeSet.bodySims[simIndex] = sim
        }

        solverSets[Int(SetType.awakeSet.rawValue)] = awakeSet
    }

    // MARK: Step context

    struct StepContext {
        var dt: Float = 0
        var inv_dt: Float = 0
        var h: Float = 0
        var inv_h: Float = 0
        var subStepCount: Int32 = 0
        var maxLinearVelocity: Float = 0
        var contactSoftness = Softness()
        var staticSoftness = Softness()
        var enableWarmStarting = false

        /// Fast bullet bodies that need continuous collision handling.
        var bulletBodies: [Int32] = []
    }

    /// Advance the simulation. Contact and joint solving is ported separately; this performs
    /// broad-phase pair updates, integration, and body finalization.
    public func step(_ timeStep: Float, subStepCount: Int32 = 4) {
        b3Assert(timeStep >= 0)

        bodyMoveEvents.removeAll()
        sensorBeginEvents.removeAll()
        contactBeginEvents.removeAll()
        contactHitEvents.removeAll()

        updateBroadPhasePairs()
        var context = StepContext()
        context.dt = timeStep
        context.subStepCount = maxInt(1, subStepCount)

        if timeStep > 0 {
            context.inv_dt = 1 / timeStep
            context.h = timeStep / Float(context.subStepCount)
            context.inv_h = Float(context.subStepCount) * context.inv_dt
        } else {
            context.inv_dt = 0
            context.h = 0
            context.inv_h = 0
        }

        self.inv_h = context.inv_h
        self.inv_dt = context.inv_dt

        // Hertz values get reduced for large time steps
        let contactHertz = minFloat(self.contactHertz, 0.125 * context.inv_h)
        context.contactSoftness = makeSoft(contactHertz, contactDampingRatio, context.h)
        context.staticSoftness = makeSoft(2.0 * contactHertz, 0.5 * contactDampingRatio, context.h)

        context.maxLinearVelocity = maxLinearSpeed
        context.enableWarmStarting = enableWarmStarting

        // Narrow phase: update contacts
        taskContexts[0].recycledContactCount = 0
        if timeStep > 0 {
            #if DEBUG
            b3Assert(validateIslands() == nil)
            #endif
            collide(&context)
        }

        if timeStep > 0 {
            stepIndex += 1

            let awakeCount = solverSets[Int(SetType.awakeSet.rawValue)].bodySims.count
            if awakeCount == 0 {
                acceptSensorHits()
                overlapSensors()
                sensorEndEvents.publish()
                contactEndEvents.publish()
                return
            }

            context.bulletBodies.removeAll()

            // Finish a pending island split from the previous step.
            if splitIslandId != B3.nullIndex {
                splitIsland(splitIslandId)
                splitIslandId = B3.nullIndex
            }

            prepareAllContacts(context)

            for _ in 0..<Int(context.subStepCount) {
                integrateVelocities(context.h)

                warmStartAll(context)

                for _ in 0..<1 {
                    pushAll(context)
                }

                integratePositions(context.h, context.inv_dt)

                for _ in 0..<1 {
                    solveAll(context)
                }

            }

            if solveAnyRestitution && restitutionIterations > 0 {
                for _ in 0..<Int(restitutionIterations) {
                    applyRestitutionAll(context)
                }
            }

            storeAllImpulses()
            emitContactHitEvents()

            // Prepare the island bit set used in body finalization.
            let awakeIslandCount = solverSets[Int(SetType.awakeSet.rawValue)].islandSims.count
            for i in 0..<taskContexts.count {
                taskContexts[i].awakeIslandBitSet.setBitCountAndClear(UInt32(awakeIslandCount))
                taskContexts[i].splitIslandId = B3.nullIndex
                taskContexts[i].splitSleepTime = 0
            }

            emitJointEvents(context.inv_h)

            finalizeBodies(&context)

            // Refit the kinematic and dynamic trees after body finalization, matching b3Solve.
            broadPhase.trees[Int(BodyType.kinematicBody.rawValue)].refit()
            broadPhase.trees[Int(BodyType.dynamicBody.rawValue)].refit()

            // Fast bullet bodies: run the swept pass, then enlarge their broad-phase proxies.
            if context.bulletBodies.isEmpty == false {
                for index in context.bulletBodies {
                    solveContinuous(index, context.dt)
                }

                let dynamicTree = broadPhase.trees[Int(BodyType.dynamicBody.rawValue)]
                var awakeSet = solverSets[Int(SetType.awakeSet.rawValue)]
                for index in context.bulletBodies {
                    var sim = awakeSet.bodySims[Int(index)]
                    if (sim.flags & BodyFlags.enlargeBulletBounds.rawValue) == 0 {
                        continue
                    }

                    // Clear flag
                    sim.flags &= ~BodyFlags.enlargeBulletBounds.rawValue

                    let bulletBody = bodies[Int(sim.bodyId)]
                    var shapeId = bulletBody.headShapeId
                    while shapeId != B3.nullIndex {
                        let shape = shapes[Int(shapeId)]
                        let proxyId = proxyIdOfKey(shape.proxyKey)
                        let treeAABB = dynamicTree.getAABB(proxyId)
                        let shapeFatAABB = fatAABBs[Int(shapeId)]

                        if aabbContains(treeAABB, shapeFatAABB) == false {
                            dynamicTree.enlargeProxy(proxyId, shapeFatAABB)
                        }

                        shapeId = shape.nextShapeId
                    }

                    awakeSet.bodySims[Int(index)] = sim
                }
                solverSets[Int(SetType.awakeSet.rawValue)] = awakeSet
            }

            if enableSleep {
                processSleep()
            }

            acceptSensorHits()
            overlapSensors()
        }

        sensorEndEvents.publish()
        contactEndEvents.publish()
    }
}
