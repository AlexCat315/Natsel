

// Continuous collision (solver.c CCD pass).

/// Mutable state shared by the swept queries of one fast body.
private final class ContinuousContext {
    var world: World!
    var fastBodyId: Int32 = B3.nullIndex
    var fastShapeId: Int32 = B3.nullIndex
    var base = Pos()
    var sweep = Sweep()
    var fraction: Float = 1

    var sensorHits: [(sensorId: Int32, visitorId: Int32, fraction: Float)] = []
    var distanceIterations: Int32 = 0
    var pushBackIterations: Int32 = 0
    var rootIterations: Int32 = 0
}

/// Offset an AABB by a position, rounding outward as in the C code.
@inline(__always)
func aabbOffset(_ localBox: AABB, _ origin: Pos) -> AABB {
    AABB(
        Vec3(
            roundDownFloat(origin.x + localBox.lowerBound.x),
            roundDownFloat(origin.y + localBox.lowerBound.y),
            roundDownFloat(origin.z + localBox.lowerBound.z)
        ),
        Vec3(
            roundUpFloat(origin.x + localBox.upperBound.x),
            roundUpFloat(origin.y + localBox.upperBound.y),
            roundUpFloat(origin.z + localBox.upperBound.z)
        )
    )
}

extension World {
    /// Is this shape moving fast enough to need continuous collision?
    func isShapeFast(
        _ shape: Shape, _ centroid1: Vec3, _ centroid2: Vec3, _ rotationChord: Float,
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
        _ shapeA: Shape, _ shapeB: Shape, _ sweepA: Sweep, _ sweepB: Sweep, _ maxFraction: Float
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
        let fastShape = shapes[Int(context.fastShapeId)]
        let fastBodyId = context.fastBodyId

        // Skip same shape
        if shapeId == fastShape.id {
            return true
        }

        let shape = shapes[Int(shapeId)]

        // Skip same body
        if shape.bodyId == fastShape.bodyId {
            return true
        }

        // Skip sensors unless both shapes want sensor events
        let isSensor = shape.sensorIndex != B3.nullIndex
        if isSensor
            && ((shape.flags & ShapeFlags.enableSensorEvents.rawValue) == 0
                || (fastShape.flags & ShapeFlags.enableSensorEvents.rawValue) == 0)
        {
            return true
        }

        // Skip filtered shapes
        if shouldShapesCollide(fastShape.filter, shape.filter) == false {
            return true
        }

        let body = bodies[Int(shape.bodyId)]

        // Skip bullets
        if (body.flags & BodyFlags.isBullet.rawValue) != 0 {
            return true
        }

        // Skip filtered bodies
        let fastBody = bodies[Int(fastBodyId)]
        if shouldBodiesCollide(fastBody, body) == false {
            return true
        }

        let bodySim = jointBodySim(shape.bodyId)

        // Time of impact versus shape. Supports all convex shape types.
        let sweepA = makeRelativeSweep(bodySim, context.base)
        let output = shapeTimeOfImpact(shape, fastShape, sweepA, context.sweep, context.fraction)

        if isSensor {
            // Only accept a sensor hit that is sooner than the current solid hit.
            if output.fraction <= context.fraction && context.sensorHits.count < 8 {
                context.sensorHits.append((sensorId: shape.id, visitorId: fastShape.id, fraction: output.fraction))
            }
        } else if 0 < output.fraction && output.fraction < context.fraction {
            context.fraction = output.fraction
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
        var fastBodySim = awakeSet.bodySims[Int(bodySimIndex)]
        b3Validate((fastBodySim.flags & BodyFlags.isFast.rawValue) != 0)

        // Re-center the sweep on the fast body so the TOI and the swept query stay in float precision
        let base = fastBodySim.center0

        let sweep = makeRelativeSweep(fastBodySim, base)

        var xf1 = Transform()
        xf1.q = sweep.q1
        xf1.p = subPos(sweep.c1, rotateVector(sweep.q1, sweep.localCenter))

        var xf2 = Transform()
        xf2.q = sweep.q2
        xf2.p = subPos(sweep.c2, rotateVector(sweep.q2, sweep.localCenter))

        let staticTree = broadPhase.trees[Int(BodyType.staticBody.rawValue)]
        let kinematicTree = broadPhase.trees[Int(BodyType.kinematicBody.rawValue)]
        let dynamicTree = broadPhase.trees[Int(BodyType.dynamicBody.rawValue)]
        let fastBody = bodies[Int(fastBodySim.bodyId)]

        let context = ContinuousContext()
        context.world = self
        context.fastBodyId = fastBodySim.bodyId
        context.base = base
        context.sweep = sweep
        context.fraction = 1

        let isBullet = (fastBodySim.flags & BodyFlags.isBullet.rawValue) != 0
        let safetyFactor = fastBody.safetyFactor
        let rotationChord = 2 * length(invMulQuat(sweep.q1, sweep.q2).v)

        var shapeId = fastBody.headShapeId
        while shapeId != B3.nullIndex {
            var fastShape = shapes[Int(shapeId)]
            shapeId = fastShape.nextShapeId

            context.fastShapeId = fastShape.id
            let centroid1 = transformPoint(xf1, fastShape.localCentroid)
            let centroid2 = transformPoint(xf2, fastShape.localCentroid)

            let box1 = fastShape.aabb
            // xf2 is relative to the base, so translate the box back to world space, rounding outward
            let box2 = aabbOffset(computeShapeAABB(fastShape, xf2), base)

            // Store this to avoid double computation in the case there is no impact event
            fastShape.aabb = box2
            shapes[Int(fastShape.id)] = fastShape

            // No continuous collision for meshes
            if fastShape.type == .meshShape || fastShape.type == .heightShape {
                continue
            }

            // No continuous collision for sensors
            if fastShape.sensorIndex != B3.nullIndex {
                continue
            }

            if isBullet == false {
                let isShapeFast = isShapeFast(
                    fastShape, centroid1, centroid2, rotationChord, safetyFactor
                )
                if isShapeFast == false {
                    continue
                }
            }

            let sweptBox = aabbUnion(box1, box2)
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

        var taskContext = taskContexts[0]

        if context.fraction < 1 {
            // Handle time of impact event. The sweep is relative to the base, so re-add the base
            // to return the advanced pose to world space.
            let q = nlerp(sweep.q1, sweep.q2, context.fraction)
            let c = lerp(sweep.c1, sweep.c2, context.fraction)
            let origin = subPos(c, rotateVector(q, sweep.localCenter))

            // Advance body
            let transform = Transform(offsetPos(base, origin), q)
            let center = offsetPos(base, c)
            fastBodySim.transform = transform
            fastBodySim.center = center
            fastBodySim.rotation0 = q
            fastBodySim.center0 = center
            fastBodySim.flags |= BodyFlags.hadTimeOfImpact.rawValue

            // Timeloss means there is a lost gravity contribution. Other forces and torques are
            // ignored for now.
            var fastBodyState = awakeSet.bodyStates[Int(bodySimIndex)]
            let v = fastBodyState.linearVelocity
            let timeLoss = (1 - context.fraction) * dt
            var dv = (-timeLoss * fastBodySim.gravityScale) * gravity
            dv.x = (fastBodyState.flags & BodyFlags.lockLinearX.rawValue) != 0 ? 0 : dv.x
            dv.y = (fastBodyState.flags & BodyFlags.lockLinearY.rawValue) != 0 ? 0 : dv.y
            dv.z = (fastBodyState.flags & BodyFlags.lockLinearZ.rawValue) != 0 ? 0 : dv.z
            fastBodyState.linearVelocity = v + dv
            awakeSet.bodyStates[Int(bodySimIndex)] = fastBodyState

            // The move event was written before CCD, so correct it with the impact pose
            if bodySimIndex < bodyMoveEvents.count {
                var event = bodyMoveEvents[Int(bodySimIndex)]
                event.transform = fastBodySim.transform
                bodyMoveEvents[Int(bodySimIndex)] = event
            }

            // Prepare AABBs for broad-phase.
            // Even though a body is fast, it may not move much. So the AABB may not need enlargement.
            shapeId = fastBody.headShapeId
            while shapeId != B3.nullIndex {
                var shape = shapes[Int(shapeId)]

                // Must recompute aabb at the interpolated transform
                let aabb = computeFatShapeAABB(shape, transform, speculativeScalar)
                shape.aabb = aabb

                let fatAABB = fatAABBs[Int(shapeId)]
                if aabbContains(fatAABB, aabb) == false {
                    let inflated = aabbInflate(aabb, shape.aabbMargin)
                    fatAABBs[Int(shapeId)] = inflated

                    // Regular bodies mark the hierarchy as moved using atomic operations.
                    // Bullets are handled separately at a later stage.
                    if isBullet == false {
                        broadPhase.markProxyMoved(shape.proxyKey, inflated)
                    } else {
                        fastBodySim.flags |= BodyFlags.enlargeBulletBounds.rawValue
                    }
                }

                shapes[Int(shapeId)] = shape
                shapeId = shape.nextShapeId
            }
        } else {
            // No time of impact event

            // Advance body
            fastBodySim.rotation0 = fastBodySim.transform.q
            fastBodySim.center0 = fastBodySim.center

            // Prepare AABBs for broad-phase
            shapeId = fastBody.headShapeId
            while shapeId != B3.nullIndex {
                let shape = shapes[Int(shapeId)]

                // shape.aabb is still valid from above
                let fatAABB = fatAABBs[Int(shapeId)]
                if aabbContains(fatAABB, shape.aabb) == false {
                    let inflated = aabbInflate(shape.aabb, shape.aabbMargin)
                    fatAABBs[Int(shapeId)] = inflated

                    if isBullet == false {
                        broadPhase.markProxyMoved(shape.proxyKey, inflated)
                    } else {
                        fastBodySim.flags |= BodyFlags.enlargeBulletBounds.rawValue
                    }
                }

                shapeId = shape.nextShapeId
            }
        }

        // Push sensor hits on the task context for serial processing.
        for hit in context.sensorHits {
            // Skip any sensor hits that occurred after a solid hit
            if hit.fraction < context.fraction {
                taskContext.sensorHits.append((sensorId: hit.sensorId, visitorId: hit.visitorId))
            }
        }

        taskContext.distanceIterations = maxInt(taskContext.distanceIterations, context.distanceIterations)
        taskContext.pushBackIterations = maxInt(
            taskContext.pushBackIterations, context.pushBackIterations
        )
        taskContext.rootIterations = maxInt(taskContext.rootIterations, context.rootIterations)
        taskContexts[0] = taskContext

        awakeSet.bodySims[Int(bodySimIndex)] = fastBodySim
    }
}
