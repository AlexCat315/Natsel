/// Captured before finalization resets slow bodies' previous poses. Every pair uses the same
/// interval, regardless of body creation order or worker scheduling.
struct ContinuousStart {
    var center: Pos
    var rotation: Quat
}

struct DynamicContinuousImpulse {
    var body: Int32
    var point: Pos
    var impulse: Vec3
}

private struct ContinuousCandidate {
    var shape: Int32
    var body: Int32
    var sim: Int
    var start: ContinuousStart
    var end: ContinuousStart
    var localCenter: Vec3
    var bounds: WorldAABB
    var dynamic: Bool
    var fast: Bool
    var invMass: Float
    var invInertiaLocal: Matrix3

    func inverseInertia(at fraction: Float) -> Matrix3 {
        guard dynamic else { return Matrix3() }
        let rotation = makeMatrixFromQuat(nlerp(start.rotation, end.rotation, fraction))
        return mulMM(mulMM(rotation, invInertiaLocal), transpose(rotation))
    }

    func sweep(relativeTo base: Pos) -> Sweep {
        var result = Sweep()
        result.c1 = subPos(start.center, base)
        result.c2 = subPos(end.center, base)
        result.q1 = start.rotation
        result.q2 = end.rotation
        result.localCenter = localCenter
        return result
    }
}

extension World {
    /// Supplemental moving-pair CCD. Broad phase compares swept bounds in double precision;
    /// endpoint-only proxies would miss bodies crossing between their final positions.
    func solveDynamicContinuous(
        _ starts: [ContinuousStart], _ sims: UnsafeMutableBufferPointer<BodySim>,
        _ states: UnsafeMutableBufferPointer<BodyState>
    ) {
        dynamicContinuousImpulses.removeAll(keepingCapacity: true)
        dynamicContinuousActiveBodies.removeAll(keepingCapacity: true)
        guard sims.contains(where: { ($0.flags & BodyFlags.isFast.rawValue) != 0 }) else { return }
        var candidates: [ContinuousCandidate] = []
        for body in bodies where body.id != B3.nullIndex && body.type != .staticBody
            && body.setIndex != SetType.disabledSet.rawValue {
            let awake = body.setIndex == SetType.awakeSet.rawValue
            let index = Int(body.localIndex)
            let sim = awake ? sims[index] : solverSets[Int(body.setIndex)].bodySims[index]
            let start = awake ? starts[index] : ContinuousStart(center: sim.center, rotation: sim.transform.q)
            let end = ContinuousStart(center: sim.center, rotation: sim.transform.q)
            var shapeId = body.headShapeId
            while shapeId != B3.nullIndex {
                let shape = shapes[Int(shapeId)]
                defer { shapeId = shape.nextShapeId }
                guard isConvex(shape.type), shape.sensorIndex == B3.nullIndex else { continue }
                let xf1 = Transform(-rotateVector(start.rotation, sim.localCenter), start.rotation)
                let xf2 = Transform(subPos(end.center, start.center) - rotateVector(end.rotation, sim.localCenter), end.rotation)
                var bounds = aabbUnion(computeShapeAABB(shape, xf1), computeShapeAABB(shape, xf2))
                // The COM-centered bounding sphere contains the complete rotational arc.
                if lengthSquared(invMulQuat(start.rotation, end.rotation).v) > 1.0e-12 {
                    let box = computeShapeAABB(shape, Transform.identity)
                    let radius = length(maxVec3(absVec3(box.lowerBound - sim.localCenter), absVec3(box.upperBound - sim.localCenter)))
                    let delta = subPos(end.center, start.center)
                    bounds = aabbUnion(bounds, aabbInflate(AABB(minVec3(.zero, delta), maxVec3(.zero, delta)), radius))
                }
                candidates.append(ContinuousCandidate(shape: shapeId, body: body.id, sim: awake ? index : -1,
                    start: start, end: end, localCenter: sim.localCenter,
                    bounds: WorldAABB(localBounds: bounds, origin: start.center), dynamic: body.type == .dynamicBody,
                    fast: awake && (sim.flags & BodyFlags.isFast.rawValue) != 0,
                    invMass: body.type == .dynamicBody ? sim.invMass : 0, invInertiaLocal: sim.invInertiaLocal))
            }
        }
        var hits: [(a: Int, b: Int, output: TOIOutput)] = []
        for pair in overlappingBoundsPairs(candidates.map { $0.bounds }).pairs {
            let i = pair.a
            let bIndex = pair.b
            let a = candidates[i]
            let b = candidates[bIndex]
            guard a.body != b.body, (a.fast || b.fast), (a.dynamic || b.dynamic),
                a.bounds.overlaps(b.bounds), shouldBodiesCollide(bodies[Int(a.body)], bodies[Int(b.body)]) else { continue }
            let shapeA = shapes[Int(a.shape)]
            let shapeB = shapes[Int(b.shape)]
            guard shouldShapesCollide(shapeA.filter, shapeB.filter) else { continue }
            let idA = ShapeID(index1: a.shape + 1, world0: worldId, generation: shapeA.generation)
            let idB = ShapeID(index1: b.shape + 1, world0: worldId, generation: shapeB.generation)
            if ((shapeA.flags | shapeB.flags) & ShapeFlags.enableCustomFiltering.rawValue) != 0,
                let filter = customFilter, !filter(idA, idB) { continue }
            let result = shapeTimeOfImpact(shapeA, shapeB, a.sweep(relativeTo: a.start.center),
                                           b.sweep(relativeTo: a.start.center), 1)
            guard result.fraction > 0, result.fraction < 1 else { continue }
            if ((shapeA.flags | shapeB.flags) & ShapeFlags.enablePreSolveEvents.rawValue) != 0,
                let callback = preSolve,
                !callback(idA, idB, offsetPos(a.start.center, result.point), result.normal) { continue }
            hits.append((i, bIndex, result))
        }
        hits.sort {
            if $0.output.fraction != $1.output.fraction { return $0.output.fraction < $1.output.fraction }
            if $0.a != $1.a { return $0.a < $1.a }
            return $0.b < $1.b
        }
        var fractions = [Float](repeating: 1, count: sims.count)
        var carries = [Vec3](repeating: .zero, count: sims.count)
        var sleepingVelocities: [Int32: (linear: Vec3, angular: Vec3)] = [:]
        for hit in hits {
            let a = candidates[hit.a]
            let b = candidates[hit.b]
            let result = hit.output
            guard (a.sim < 0 || fractions[a.sim] >= result.fraction),
                (b.sim < 0 || fractions[b.sim] >= result.fraction) else { continue }
            // Resolve the impact velocity now. Waiting for next step's soft contact solver can let
            // fast pairs cross again while its position correction is still converging.
            let normal = result.normal
            let point = result.point
            let ca = result.fraction * subPos(a.end.center, a.start.center)
            let cb = subPos(b.start.center, a.start.center) + result.fraction * subPos(b.end.center, b.start.center)
            let ra = point - ca
            let rb = point - cb
            let ma = a.invMass
            let mb = b.invMass
            let ia = a.inverseInertia(at: result.fraction)
            let ib = b.inverseInertia(at: result.fraction)
            let sleepA = sleepingVelocities[a.body] ?? (.zero, .zero)
            let sleepB = sleepingVelocities[b.body] ?? (.zero, .zero)
            let va = a.sim >= 0 ? states[a.sim].linearVelocity + cross(states[a.sim].angularVelocity, ra)
                : sleepA.linear + cross(sleepA.angular, ra)
            let vb = b.sim >= 0 ? states[b.sim].linearVelocity + cross(states[b.sim].angularVelocity, rb)
                : sleepB.linear + cross(sleepB.angular, rb)
            let approach = dot(vb - va, normal)
            let ran = cross(ra, normal)
            let rbn = cross(rb, normal)
            let mass = ma + mb + dot(ran, mulMV(ia, ran)) + dot(rbn, mulMV(ib, rbn))
            if approach < 0 && mass > 0 {
                let materialA = shapes[Int(a.shape)].material(0)
                let materialB = shapes[Int(b.shape)].material(0)
                let bounce = -approach > restitutionThreshold
                    ? mixRestitution(materialA.restitution, materialA.userMaterialId, materialB.restitution, materialB.userMaterialId) : 0
                let impulse = (-(1 + bounce) * approach / mass) * normal
                if ma > 0 {
                    dynamicContinuousActiveBodies.append(a.body)
                    if a.sim >= 0 {
                        states[a.sim].linearVelocity -= ma * impulse
                        states[a.sim].angularVelocity -= mulMV(ia, cross(ra, impulse))
                    } else {
                        sleepingVelocities[a.body] = (sleepA.linear - ma * impulse, sleepA.angular - mulMV(ia, cross(ra, impulse)))
                        dynamicContinuousImpulses.append(DynamicContinuousImpulse(body: a.body,
                            point: offsetPos(a.start.center, result.point), impulse: -impulse))
                    }
                }
                if mb > 0 {
                    dynamicContinuousActiveBodies.append(b.body)
                    if b.sim >= 0 {
                        states[b.sim].linearVelocity += mb * impulse
                        states[b.sim].angularVelocity += mulMV(ib, cross(rb, impulse))
                    } else {
                        sleepingVelocities[b.body] = (sleepB.linear + mb * impulse, sleepB.angular + mulMV(ib, cross(rb, impulse)))
                        dynamicContinuousImpulses.append(DynamicContinuousImpulse(body: b.body,
                            point: offsetPos(a.start.center, result.point), impulse: impulse))
                    }
                }
                let shapeA = shapes[Int(a.shape)]
                let shapeB = shapes[Int(b.shape)]
                if -approach > hitEventThreshold && ((shapeA.flags | shapeB.flags) & ShapeFlags.enableHitEvents.rawValue) != 0 {
                    var event = ContactHitEvent()
                    event.shapeIdA = makeShapeID(a.shape)
                    event.shapeIdB = makeShapeID(b.shape)
                    event.point = offsetPos(a.start.center, result.point)
                    event.normal = normal
                    event.approachSpeed = -approach
                    event.userMaterialIdA = materialA.userMaterialId
                    event.userMaterialIdB = materialB.userMaterialId
                    contactHitEvents.append(event)
                }
            }
            for (mover, other, n) in [(a, b, normal), (b, a, -normal)]
            where mover.dynamic && mover.sim >= 0 && result.fraction < fractions[mover.sim] {
                fractions[mover.sim] = result.fraction
                carries[mover.sim] = other.dynamic ? .zero
                    : ((1 - result.fraction) * dot(subPos(other.end.center, other.start.center), n)) * n
            }
        }
        for i in sims.indices where fractions[i] < 1 {
            let t = fractions[i]
            var sim = sims[i]
            sim.center = offsetPos(starts[i].center, t * subPos(sim.center, starts[i].center) + carries[i])
            sim.transform.q = nlerp(starts[i].rotation, sim.transform.q, t)
            sim.transform.p = offsetPos(sim.center, -rotateVector(sim.transform.q, sim.localCenter))
            sim.center0 = starts[i].center
            sim.rotation0 = starts[i].rotation
            sim.flags |= BodyFlags.isFast.rawValue | BodyFlags.hadTimeOfImpact.rawValue
            sims[i] = sim
            bodyMoveEvents[i].transform = sim.transform
            finalizeFlags[i] = (sim.flags & BodyFlags.isBullet.rawValue) != 0 ? FinalizeFlag.bullet : FinalizeFlag.continuous
        }
    }
}
