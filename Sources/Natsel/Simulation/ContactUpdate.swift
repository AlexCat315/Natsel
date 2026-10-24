// Convex contact update (the narrow phase of contact.c).

extension World {
    /// Compute a new manifold for a convex shape pair, recycling impulses by feature id.
    /// Returns true if the shapes are touching.
    func computeConvexManifold(
        _ contactId: Int32,
        _ shapeA: UnsafePointer<Shape>,
        _ xfA: WorldTransform,
        _ shapeB: UnsafePointer<Shape>,
        _ xfB: WorldTransform,
        _ scratch: UnsafeMutablePointer<MeshContactScratch>,
        _ cs: inout ContactStore
    ) -> Bool {
        return cs.withContact(contactId) { contact in

            let typeA = shapeA.pointee.type
            let typeB = shapeB.pointee.type

            var pointBuffer = InlineArray<4, LocalManifoldPoint>(repeating: LocalManifoldPoint())
            var geomManifold = LocalManifold()
            let pointCapacity = B3.maxManifoldPoints

            let transformBtoA = invMulWorldTransforms(xfA, xfB)

            return withInlineBuffer(&pointBuffer) { points in
                geomManifold.points = points.baseAddress

                if typeA == .sphereShape {
                    b3Assert(typeB == .sphereShape)
                    collideSpheres(
                        &geomManifold, pointCapacity, shapeA.pointee.sphere, shapeB.pointee.sphere, transformBtoA
                    )
                } else if typeA == .capsuleShape {
                    if typeB == .sphereShape {
                        collideCapsuleAndSphere(
                            &geomManifold, pointCapacity, shapeA.pointee.capsule, shapeB.pointee.sphere, transformBtoA
                        )
                    } else {
                        b3Assert(typeB == .capsuleShape)
                        collideCapsules(
                            &geomManifold, pointCapacity, shapeA.pointee.capsule, shapeB.pointee.capsule, transformBtoA
                        )
                    }
                } else {
                    b3Assert(typeA == .hullShape)

                    if typeB == .sphereShape {
                        collideHullAndSphere(
                            &geomManifold, pointCapacity, shapeA.pointee.hull!, shapeB.pointee.sphere, transformBtoA,
                            &contact.simplexCache
                        )
                    } else if typeB == .capsuleShape {
                        collideHullAndCapsule(
                            &geomManifold, pointCapacity, shapeA.pointee.hull!, shapeB.pointee.capsule, transformBtoA,
                            &contact.simplexCache
                        )
                    } else {
                        b3Assert(typeB == .hullShape)
                        collideHulls(
                            &geomManifold, pointCapacity, shapeA.pointee.hull!, shapeB.pointee.hull!, transformBtoA,
                            scratch, &contact.satCache
                        )
                        scratch.pointee.satCalls += 1
                        scratch.pointee.satCacheHits += Int32(contact.satCache.hit)
                    }
                }

                if geomManifold.pointCount == 0 {
                    if contact.manifoldCount > 0 {
                        contact.manifolds.removeAll()
                        contact.manifoldCount = 0
                    }

                    return false
                }

                var oldPoints = InlineArray<4, ManifoldPoint>(repeating: ManifoldPoint())
                var oldCount: Int32 = 0

                if contact.manifoldCount == 0 {
                    contact.manifolds.replace(with: ContactManifolds(Manifold()))
                    contact.manifoldCount = 1
                } else {
                    oldCount = contact.manifolds[0].pointCount
                    for i in 0..<Int(oldCount) {
                        oldPoints[i] = contact.manifolds[0].points[i]
                    }
                }

                b3Assert(0 < geomManifold.pointCount && geomManifold.pointCount <= B3.maxManifoldPoints)

                var manifold = contact.manifolds[0]
                manifold.pointCount = geomManifold.pointCount

                let matrixA = makeMatrixFromQuat(xfA.q)
                manifold.normal = mulMV(matrixA, geomManifold.normal)

                // Store point data in contact
                for i in 0..<Int(geomManifold.pointCount) {
                    let source = geomManifold.points![i]
                    var target = ManifoldPoint()

                    // Contact points are computed in frame A
                    target.anchorA = mulMV(matrixA, source.point)
                    target.anchorB = target.anchorA + subPos(xfA.p, xfB.p)
                    target.separation = source.separation
                    target.featureId = makeFeatureId(source.pair)
                    target.triangleIndex = B3.nullIndex
                    target.normalVelocity = 0

                    manifold.points[i] = target
                }

                // Copy impulses from old points
                for i in 0..<Int(geomManifold.pointCount) {
                    var pt2 = manifold.points[i]
                    pt2.totalNormalImpulse = 0
                    pt2.persisted = false

                    for j in 0..<Int(oldCount) {
                        var pt1 = oldPoints[j]

                        if pt2.featureId == pt1.featureId {
                            pt2.normalImpulse = pt1.normalImpulse
                            pt2.persisted = true

                            // claimed
                            pt1.featureId = UInt32.max
                            oldPoints[j] = pt1

                            break
                        }
                    }

                    if pt2.persisted == false {
                        pt2.normalImpulse = 0
                    }

                    manifold.points[i] = pt2
                }

                contact.manifolds[0] = manifold

                return true
            }
        }
    }

    /// Update a convex contact and mix materials.
    func updateConvexContact(
        _ contactId: Int32,
        _ shapeA: UnsafePointer<Shape>,
        _ xfA: WorldTransform,
        _ shapeB: UnsafePointer<Shape>,
        _ xfB: WorldTransform,
        _ flip: Bool,
        _ scratch: UnsafeMutablePointer<MeshContactScratch>,
        _ cs: inout ContactStore
    ) -> Bool {
        // Compute new manifold
        let touching = computeConvexManifold(contactId, shapeA, xfA, shapeB, xfB, scratch, &cs)

        if touching == false {
            b3Assert(cs.get(contactId).manifoldCount == 0)
            return false
        }

        return cs.withContact(contactId) { contact in
            b3Assert(contact.manifoldCount == 1)

            if flip {
                // Not flipping the feature ids because they just need to match and flipping is
                // consistent.
                var manifold = contact.manifolds[0]
                manifold.normal = -manifold.normal
                let pointCount = manifold.pointCount
                for i in 0..<Int(pointCount) {
                    var mp = manifold.points[i]
                    swap(&mp.anchorA, &mp.anchorB)
                    manifold.points[i] = mp
                }
                contact.manifolds[0] = manifold
            }

            let materialA = shapeA.pointee.material(0)
            let materialB = shapeB.pointee.material(0)

            // Keep these updated in case the values on the shapes are modified
            contact.friction = mixFriction(
                materialA.friction, materialA.userMaterialId, materialB.friction, materialB.userMaterialId
            )
            contact.restitution = mixRestitution(
                materialA.restitution, materialA.userMaterialId, materialB.restitution,
                materialB.userMaterialId
            )

            if materialA.rollingResistance > 0 || materialB.rollingResistance > 0 {
                func radius(_ shape: UnsafePointer<Shape>) -> Float {
                    switch shape.pointee.geometry {
                    case .sphere(let sphere):
                        return sphere.radius
                    case .capsule(let capsule):
                        return capsule.radius
                    case .hull(let hull):
                        return 0.25 * hull.innerRadius
                    default:
                        return 0
                    }
                }

                let radiusA = radius(shapeA)
                let radiusB = radius(shapeB)
                let maxRadius = maxFloat(radiusA, radiusB)
                contact.rollingResistance =
                    maxFloat(materialA.rollingResistance, materialB.rollingResistance) * maxRadius
            } else {
                contact.rollingResistance = 0
            }

            let tangentVelocityA = rotateVector(xfA.q, materialA.tangentVelocity)
            let tangentVelocityB = rotateVector(xfB.q, materialB.tangentVelocity)
            contact.tangentVelocity = tangentVelocityA - tangentVelocityB

            if let preSolve, (contact.flags & ContactFlags.simEnablePreSolveEvents.rawValue) != 0 {
                let idA = ShapeID(
                    index1: shapeA.pointee.id + 1, world0: worldId, generation: shapeA.pointee.generation
                )
                let idB = ShapeID(
                    index1: shapeB.pointee.id + 1, world0: worldId, generation: shapeB.pointee.generation
                )

                // This call assumes thread safety
                let point = offsetPos(xfA.p, contact.manifolds[0].points[0].anchorA)
                let normal = contact.manifolds[0].normal
                if preSolve(idA, idB, point, normal) == false {
                    // Disable the contact
                    contact.manifolds.removeAll()
                    contact.manifoldCount = 0

                    // Invalidate the recycling cache.
                    contact.flags &= ~ContactFlags.relativeTransformValid.rawValue
                    return false
                }
            }

            if (shapeA.pointee.flags & ShapeFlags.enableHitEvents.rawValue) != 0
                || (shapeB.pointee.flags & ShapeFlags.enableHitEvents.rawValue) != 0
            {
                contact.flags |= ContactFlags.simEnableHitEvent.rawValue
            } else {
                contact.flags &= ~ContactFlags.simEnableHitEvent.rawValue
            }

            return true
        }
    }

    /// Compatibility wrapper for callers that do not own mesh scratch.
    @discardableResult
    func updateContact(
        _ contactId: Int32,
        _ localCenterA: Vec3,
        _ xfA: WorldTransform,
        _ localCenterB: Vec3,
        _ xfB: WorldTransform,
        _ isFast: Bool,
        _ cs: inout ContactStore
    ) -> Bool {
        withUnsafeMutablePointer(to: &meshContactScratch) { scratch in
            updateContact(
                contactId, localCenterA, xfA, localCenterB, xfB, isFast,
                scratch, &cs)
        }
    }

    /// Update the contact manifold and touching status.
    @discardableResult
    func updateContact(
        _ contactId: Int32,
        _ localCenterA: Vec3,
        _ xfA: WorldTransform,
        _ localCenterB: Vec3,
        _ xfB: WorldTransform,
        _ isFast: Bool,
        _ scratch: UnsafeMutablePointer<MeshContactScratch>,
        _ cs: inout ContactStore
    ) -> Bool {
        let contact = cs.get(contactId)
        let shapeIndexA = Int(contact.shapeIdA)
        let shapeIndexB = Int(contact.shapeIdB)

        return shapes.withUnsafeBufferPointer { shapeBuffer in
            let shapeA = shapeBuffer.baseAddress! + shapeIndexA
            let shapeB = shapeBuffer.baseAddress! + shapeIndexB

            b3Assert(shapeB.pointee.type != .compoundShape)

            let touching: Bool
            let typeA = shapeA.pointee.type

            if typeA == .compoundShape {
                touching = updateCompoundContact(
                    contactId, shapeA.pointee, xfA, shapeB.pointee, xfB, isFast, scratch, &cs)
            } else if typeA == .meshShape || typeA == .heightShape {
                touching = computeMeshManifolds(
                    contactId, shapeA: shapeA.pointee, transformA: xfA,
                    shapeB: shapeB.pointee, transformB: xfB, isFast: isFast, scratch, cs: &cs)
                let enableHit =
                    touching
                    && ((shapeA.pointee.flags | shapeB.pointee.flags)
                        & ShapeFlags.enableHitEvents.rawValue) != 0
                cs.withContact(contactId) { contact in
                    if enableHit {
                        contact.flags |= ContactFlags.simEnableHitEvent.rawValue
                    } else {
                        contact.flags &= ~ContactFlags.simEnableHitEvent.rawValue
                    }
                }
            } else {
                // Convex-vs-convex
                touching = updateConvexContact(contactId, shapeA, xfA, shapeB, xfB, false, scratch, &cs)
            }

            if touching {
                let centerA = rotateVector(xfA.q, localCenterA)
                let centerB = rotateVector(xfB.q, localCenterB)

                // Adjust anchors to be relative to center of mass, editing the stored contact in place.
                cs.withContact(contactId) { contact in
                    for i in 0..<contact.manifoldCount {
                        for j in 0..<contact.manifolds[Int(i)].pointCount {
                            contact.manifolds[Int(i)].points[Int(j)].anchorA =
                                contact.manifolds[Int(i)].points[Int(j)].anchorA - centerA
                            contact.manifolds[Int(i)].points[Int(j)].anchorB =
                                contact.manifolds[Int(i)].points[Int(j)].anchorB - centerB
                        }
                    }
                    contact.flags |= ContactFlags.simTouching.rawValue
                }
            } else {
                cs.withContact(contactId) { $0.flags &= ~ContactFlags.simTouching.rawValue }
            }

            return touching
        }
    }
}
