

// Continuous collision (solver.c CCD pass).

/// Mutable state shared by the swept queries of one fast body.
final class ContinuousContext {
    var world: World!
    var fastBodyId: Int32 = B3.nullIndex
    var fastShapeId: Int32 = B3.nullIndex
    var base = Pos()
    var sweep = Sweep()
    var fraction: Float = 1
    var impact: (surface: Int32, mover: Int32, output: TOIOutput)?

    var sensorHits: [(sensorId: Int32, visitorId: Int32, fraction: Float)] = []
    var distanceIterations: Int32 = 0
    var pushBackIterations: Int32 = 0
    var rootIterations: Int32 = 0
}

/// Offset an AABB by a position, rounding outward as in the C code.
@inline(__always)
func aabbOffset(_ localBox: AABB, _ origin: Pos) -> AABB {
    offsetAABB(localBox, origin)
}

extension World {
    /// Is this shape moving fast enough to need continuous collision?
    func isShapeFast(
        _ shape: borrowing Shape, _ centroid1: Vec3, _ centroid2: Vec3, _ rotationChord: Float,
        _ safetyFactor: Float
    ) -> Bool {
        let radius: Float
        switch shape.geometry {
        case .sphere(let sphere):
            radius = sphere.radius
        case .capsule(let capsule):
            radius = 0.5 * distance(capsule.center1, capsule.center2) + capsule.radius
        case .hull(let hull):
            let farthestPoint = farthestPointOnAABB(hull.aabb, shape.localCentroid)
            radius = distance(farthestPoint, shape.localCentroid)
        default:
            return true
        }

        let minExtent = computeShapeMinExtent(shape, shape.localCentroid)
        let maxMotion = distance(centroid1, centroid2) + rotationChord * radius
        return maxMotion > safetyFactor * minExtent
    }

    /// Time of impact between convex shapes or a mesh and a convex shape.
    func shapeTimeOfImpact(
        _ shapeA: borrowing Shape, _ shapeB: borrowing Shape, _ sweepA: Sweep, _ sweepB: Sweep, _ maxFraction: Float
    ) -> TOIOutput {

        if case .compound(let compound) = shapeA.geometry {
            return compoundTimeOfImpact(compound, shapeB: shapeB, sweepA: sweepA, sweepB: sweepB, maxFraction: maxFraction)
        }

        if let surface = TriangleSurface(shapeA.geometry) {
            return triangleSurfaceTimeOfImpact(surface, shapeB: shapeB, sweepA: sweepA, sweepB: sweepB,
                                    maxFraction: maxFraction, isSensor: shapeA.sensorIndex != B3.nullIndex)
        }

        b3Assert(shapeB.type != .compoundShape && shapeB.type != .meshShape && shapeB.type != .heightShape)

        let bufferA = makeShapeProxy(shapeA)
        let bufferB = makeShapeProxy(shapeB)

        return bufferA.withProxy { proxyA in
            bufferB.withProxy { proxyB in
                var input = TOIInput()
                input.proxyA = proxyA
                input.proxyB = proxyB
                input.sweepA = sweepA
                input.sweepB = sweepB
                input.maxFraction = maxFraction
                return timeOfImpact(input)
            }
        }
    }

    /// Process one proxy during a swept query.
    private func continuousQueryCallback(_ shapeId: Int32, _ context: ContinuousContext) -> Bool {
        shapes.withUnsafeBufferPointer { shapeBuffer in
            continuousQueryCallback(
                shapeId, context, shapeBuffer.baseAddress! + Int(context.fastShapeId), shapeBuffer.baseAddress! + Int(shapeId))
        }
    }

    /// The callback on borrowed shapes; copying a `Shape` retains its geometry and materials.
    private func continuousQueryCallback(
        _ shapeId: Int32, _ context: ContinuousContext, _ fastShapePointer: UnsafePointer<Shape>,
        _ shapePointer: UnsafePointer<Shape>
    ) -> Bool {
        let fastBodyId = context.fastBodyId
        let fastShapeIdValue = fastShapePointer.pointee.id

        // Skip same shape
        if shapeId == fastShapeIdValue {
            return true
        }

        let bodyId = shapePointer.pointee.bodyId

        // Skip same body
        if bodyId == fastShapePointer.pointee.bodyId {
            return true
        }

        // Skip sensors unless both shapes want sensor events
        let isSensor = shapePointer.pointee.sensorIndex != B3.nullIndex
        if isSensor
            && ((shapePointer.pointee.flags & ShapeFlags.enableSensorEvents.rawValue) == 0
                || (fastShapePointer.pointee.flags & ShapeFlags.enableSensorEvents.rawValue) == 0)
        {
            return true
        }

        // Skip filtered shapes
        if shouldShapesCollide(fastShapePointer.pointee.filter, shapePointer.pointee.filter) == false {
            return true
        }

        // Skip bullets
        if (bodies[Int(bodyId)].flags & BodyFlags.isBullet.rawValue) != 0 {
            return true
        }

        // Skip filtered bodies
        if shouldBodiesCollide(bodies[Int(fastBodyId)], bodies[Int(bodyId)]) == false {
            return true
        }

        // Time of impact versus shape. Supports all convex shape types.
        let setIndex = Int(bodies[Int(bodyId)].setIndex)
        let localIndex = Int(bodies[Int(bodyId)].localIndex)
        let sweepA = makeRelativeSweep(solverSets[setIndex].bodySims[localIndex], context.base)
        let output = shapeTimeOfImpact(
            shapePointer.pointee, fastShapePointer.pointee, sweepA, context.sweep, context.fraction)

        if isSensor {
            // Only accept a sensor hit that is sooner than the current solid hit.
            if output.fraction <= context.fraction {
                context.sensorHits.append((sensorId: shapeId, visitorId: fastShapeIdValue, fraction: output.fraction))
            }
        } else if 0 < output.fraction && output.fraction < context.fraction {
            context.fraction = output.fraction
            context.impact = (shapeId, fastShapeIdValue, output)
            context.distanceIterations = maxInt(
                context.distanceIterations, output.distanceIterations
            )
            context.pushBackIterations = maxInt(
                context.pushBackIterations, output.pushBackIterations
            )
            context.rootIterations = maxInt(context.rootIterations, output.rootIterations)
        }

        // Continue query
        return true
    }

    /// Continuous collision of a dynamic body against static bodies (and against dynamic bodies
    /// for bullets).
    func solveContinuous(_ bodySimIndex: Int32, _ dt: Float) {
        var awakeSet = solverSets[Int(SetType.awakeSet.rawValue)]
        solveContinuous(bodySimIndex, dt, &awakeSet)
        solverSets[Int(SetType.awakeSet.rawValue)] = awakeSet
    }

    /// Continuous pass against an awake set the caller already owns. The caller keeps the set so
    /// the per-body loop does not copy it in and out of `solverSets` for every fast body.
    func solveContinuous(_ bodySimIndex: Int32, _ dt: Float, _ awakeSet: inout SolverSet) {
        awakeSet.bodySims.withUnsafeMutableBufferPointer { sims in
            awakeSet.bodyStates.withUnsafeMutableBufferPointer { states in
                solveContinuous(bodySimIndex, dt, sims, states)
            }
        }
    }

    /// The finalize pass already owns these buffers. Borrow them once for the whole pass so
    /// each fast body does not repeat Array uniqueness checks when its pose is written back.
    func solveContinuous(
        _ bodySimIndex: Int32, _ dt: Float,
        _ sims: UnsafeMutableBufferPointer<BodySim>, _ states: UnsafeMutableBufferPointer<BodyState>
    ) {
        var fastBodySim = sims[Int(bodySimIndex)]
        b3Validate((fastBodySim.flags & BodyFlags.isFast.rawValue) != 0)

        // Re-center the sweep on the fast body so the TOI and the swept query stay in float precision
        let base = fastBodySim.center0

        let sweep = makeRelativeSweep(fastBodySim, base)

        var xf1 = Transform()
        xf1.q = sweep.q1
        xf1.p = (sweep.c1 - rotateVector(sweep.q1, sweep.localCenter))

        var xf2 = Transform()
        xf2.q = sweep.q2
        xf2.p = (sweep.c2 - rotateVector(sweep.q2, sweep.localCenter))

        let staticTree = broadPhase.trees[Int(BodyType.staticBody.rawValue)]
        let kinematicTree = broadPhase.trees[Int(BodyType.kinematicBody.rawValue)]
        let dynamicTree = broadPhase.trees[Int(BodyType.dynamicBody.rawValue)]
        let fastHeadShapeId = bodies[Int(fastBodySim.bodyId)].headShapeId

        let context = continuousContext
        context.world = self
        context.fastBodyId = fastBodySim.bodyId
        context.base = base
        context.sweep = sweep
        context.fraction = 1
        context.impact = nil
        context.sensorHits.removeAll(keepingCapacity: true)
        context.distanceIterations = 0
        context.pushBackIterations = 0
        context.rootIterations = 0

        let isBullet = (fastBodySim.flags & BodyFlags.isBullet.rawValue) != 0
        let safetyFactor = bodies[Int(fastBodySim.bodyId)].safetyFactor
        let rotationChord = 2 * length(invMulQuat(sweep.q1, sweep.q2).v)

        var shapeId = fastHeadShapeId
        while shapeId != B3.nullIndex {
            let fastIndex = Int(shapeId)
            shapeId = shapes[fastIndex].nextShapeId

            context.fastShapeId = Int32(fastIndex)
            let localCentroid = shapes[fastIndex].localCentroid
            let centroid1 = transformPoint(xf1, localCentroid)
            let centroid2 = transformPoint(xf2, localCentroid)

            let box1 = shapes[fastIndex].aabb
            // xf2 is relative to the base, so translate the box back to world space, rounding outward
            let box2 = aabbOffset(computeShapeAABB(shapes[fastIndex], xf2), base)

            // Store this to avoid double computation in the case there is no impact event
            shapes[fastIndex].aabb = box2

            // No continuous collision for meshes
            let shapeType = shapes[fastIndex].type
            if shapeType == .meshShape || shapeType == .heightShape {
                continue
            }

            // No continuous collision for sensors
            if shapes[fastIndex].sensorIndex != B3.nullIndex {
                continue
            }

            if isBullet == false {
                let isShapeFast = isShapeFast(
                    shapes[fastIndex], centroid1, centroid2, rotationChord, safetyFactor
                )
                if isShapeFast == false {
                    continue
                }
            }

            var sweptBox = aabbUnion(box1, box2)
            if enableDynamicContinuous && rotationChord > 0 {
                let radius = length(fastBodySim.maxExtent)
                let arcBounds = aabbInflate(AABB(minVec3(sweep.c1, sweep.c2), maxVec3(sweep.c1, sweep.c2)), radius)
                sweptBox = aabbUnion(sweptBox, aabbOffset(arcBounds, base))
            }
            _ = staticTree.query(sweptBox, maskBits: defaultMaskBits, requireAllBits: false) {
                proxyId, userData in
                self.continuousQueryCallback(Int32(Int64(bitPattern: userData)), context)
            }

            if isBullet {
                _ = kinematicTree.query(
                    sweptBox, maskBits: defaultMaskBits, requireAllBits: false
                ) { proxyId, userData in
                    self.continuousQueryCallback(Int32(Int64(bitPattern: userData)), context)
                }
                _ = dynamicTree.query(
                    sweptBox, maskBits: defaultMaskBits, requireAllBits: false
                ) { proxyId, userData in
                    self.continuousQueryCallback(Int32(Int64(bitPattern: userData)), context)
                }
            }
        }

        let speculativeScalar = B3.speculativeDistance

        if context.fraction < 1 {
            // Handle time of impact event. The sweep is relative to the base, so re-add the base
            // to return the advanced pose to world space.
            let q = nlerp(sweep.q1, sweep.q2, context.fraction)
            let c = lerp(sweep.c1, sweep.c2, context.fraction)
            let origin = (c - rotateVector(q, sweep.localCenter))

            // Advance body
            let transform = WorldTransform(offsetPos(base, origin), q)
            let center = offsetPos(base, c)
            fastBodySim.transform = transform
            fastBodySim.center = center
            fastBodySim.rotation0 = q
            fastBodySim.center0 = center
            fastBodySim.flags |= BodyFlags.hadTimeOfImpact.rawValue

            // Timeloss means there is a lost gravity contribution. Other forces and torques are
            // ignored for now.
            var fastBodyState = states[Int(bodySimIndex)]
            let v = fastBodyState.linearVelocity
            let timeLoss = (1 - context.fraction) * dt
            var dv = (-timeLoss * fastBodySim.gravityScale) * gravity
            dv.x = (fastBodyState.flags & BodyFlags.lockLinearX.rawValue) != 0 ? 0 : dv.x
            dv.y = (fastBodyState.flags & BodyFlags.lockLinearY.rawValue) != 0 ? 0 : dv.y
            dv.z = (fastBodyState.flags & BodyFlags.lockLinearZ.rawValue) != 0 ? 0 : dv.z
            fastBodyState.linearVelocity = v + dv
            states[Int(bodySimIndex)] = fastBodyState

            if let impact = context.impact {
                let a = shapes[Int(impact.surface)], b = shapes[Int(impact.mover)]
                let flags = a.flags | b.flags
                if flags & (ShapeFlags.enableContactEvents.rawValue | ShapeFlags.enableHitEvents.rawValue) != 0 {
                    let point = offsetPos(base, impact.output.point)
                    let other = bodies[Int(a.bodyId)]
                    let otherSim = other.setIndex == SetType.awakeSet.rawValue
                        ? sims[Int(other.localIndex)] : solverSets[Int(other.setIndex)].bodySims[Int(other.localIndex)]
                    let otherState = other.setIndex == SetType.awakeSet.rawValue ? states[Int(other.localIndex)] : BodyState()
                    let va = otherState.linearVelocity + cross(otherState.angularVelocity, subPos(point, otherSim.center))
                    let vb = fastBodyState.linearVelocity + cross(fastBodyState.angularVelocity, subPos(point, center))
                    var event = ContactHitEvent()
                    event.shapeIdA = makeShapeID(impact.surface)
                    event.shapeIdB = makeShapeID(impact.mover)
                    event.point = point
                    event.normal = impact.output.normal
                    event.approachSpeed = max(0, -dot(vb - va, event.normal))
                    event.userMaterialIdA = a.material(0).userMaterialId
                    event.userMaterialIdB = b.material(0).userMaterialId
                    continuousImpactEvents.append(event)
                    if event.approachSpeed > hitEventThreshold && flags & ShapeFlags.enableHitEvents.rawValue != 0 {
                        contactHitEvents.append(event)
                    }
                }
            }

            // The move event was written before CCD, so correct it with the impact pose
            if bodySimIndex < bodyMoveEvents.count {
                var event = bodyMoveEvents[Int(bodySimIndex)]
                event.transform = fastBodySim.transform
                bodyMoveEvents[Int(bodySimIndex)] = event
            }

            // Prepare AABBs for broad-phase.
            // Even though a body is fast, it may not move much. So the AABB may not need enlargement.
            shapeId = fastHeadShapeId
            while shapeId != B3.nullIndex {
                let index = Int(shapeId)

                // Must recompute aabb at the interpolated transform
                let aabb = computeFatShapeAABB(shapes[index], transform, speculativeScalar)
                shapes[index].aabb = aabb

                let fatAABB = fatAABBs[Int(shapeId)]
                if aabbContains(fatAABB, aabb) == false {
                    let inflated = aabbInflate(aabb, shapes[index].aabbMargin)
                    fatAABBs[Int(shapeId)] = inflated

                    // Regular bodies mark the hierarchy as moved using atomic operations.
                    // Bullets are handled separately at a later stage.
                    if isBullet == false {
                        broadPhase.markProxyMoved(shapes[index].proxyKey, inflated)
                    } else {
                        fastBodySim.flags |= BodyFlags.enlargeBulletBounds.rawValue
                    }
                }

                shapeId = shapes[index].nextShapeId
            }
        } else {
            // No time of impact event

            // Advance body
            fastBodySim.rotation0 = fastBodySim.transform.q
            fastBodySim.center0 = fastBodySim.center

            // Prepare AABBs for broad-phase
            shapeId = fastHeadShapeId
            while shapeId != B3.nullIndex {
                let index = Int(shapeId)

                // shape.aabb is still valid from above
                let fatAABB = fatAABBs[index]
                if aabbContains(fatAABB, shapes[index].aabb) == false {
                    let inflated = aabbInflate(shapes[index].aabb, shapes[index].aabbMargin)
                    fatAABBs[Int(shapeId)] = inflated

                    if isBullet == false {
                        broadPhase.markProxyMoved(shapes[index].proxyKey, inflated)
                    } else {
                        fastBodySim.flags |= BodyFlags.enlargeBulletBounds.rawValue
                    }
                }

                shapeId = shapes[index].nextShapeId
            }
        }

        // Push sensor hits on the task context for serial processing.
        for hit in context.sensorHits {
            // Skip any sensor hits that occurred after a solid hit
            if hit.fraction < context.fraction {
                taskContexts[0].sensorHits.append((sensorId: hit.sensorId, visitorId: hit.visitorId))
            }
        }

        taskContexts[0].distanceIterations = maxInt(taskContexts[0].distanceIterations, context.distanceIterations)
        taskContexts[0].pushBackIterations = maxInt(
            taskContexts[0].pushBackIterations, context.pushBackIterations
        )
        taskContexts[0].rootIterations = maxInt(taskContexts[0].rootIterations, context.rootIterations)

        sims[Int(bodySimIndex)] = fastBodySim
    }
}
