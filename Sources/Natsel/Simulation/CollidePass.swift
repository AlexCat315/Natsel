internal import Dispatch

/// Compact narrow-phase view of a body sim. Reading a full `BodySim` by value here copied several
/// hundred bytes twice per contact and showed up as the largest `memmove` source in the profile.
struct ContactBodySnapshot {
    var transform: WorldTransform
    var localCenter: Vec3
    var center: Pos
    var maxExtent: Vec3
    var flags: UInt32
}

// Narrow-phase collide pass (physics_world.c b3Collide).

extension World {
    /// Resolve the body sim referenced by an encoded contact body sim index.
    /// Resolve a cached contact body-sim encoding to the solver set and local index. Returning the
    /// location instead of the sim itself avoids copying the whole `BodySim` per contact, which the
    /// profile showed as the dominant cost in the collide pass.
    @inline(__always)
    func resolveContactBodySimIndex(_ encodedIndex: Int32, _ bodyId: Int32) -> (set: Int32, local: Int32) {
        if encodedIndex >= 0 {
            return (Int32(SetType.awakeSet.rawValue), encodedIndex)
        }

        if encodedIndex < B3.nullIndex {
            return (Int32(SetType.staticSet.rawValue), -encodedIndex - 2)
        }

        let body = bodies[Int(bodyId)]
        return (body.setIndex, body.localIndex)
    }

    /// Read the handful of body-sim fields the narrow phase needs without copying the whole struct.
    @inline(__always)
    func contactBodySnapshotAt(
        _ set: Int32, _ local: Int32,
        _ awakeSims: UnsafeBufferPointer<BodySim>, _ staticSims: UnsafeBufferPointer<BodySim>
    ) -> ContactBodySnapshot {
        let index = Int(local)
        let sims: UnsafeBufferPointer<BodySim>
        if set == Int32(SetType.awakeSet.rawValue) {
            sims = awakeSims
        } else if set == Int32(SetType.staticSet.rawValue) {
            sims = staticSims
        } else {
            return solverSets[Int(set)].bodySims.withUnsafeBufferPointer { buffer in
                let sim = buffer.baseAddress! + index
                return ContactBodySnapshot(
                    transform: sim.pointee.transform,
                    localCenter: sim.pointee.localCenter,
                    center: sim.pointee.center,
                    maxExtent: sim.pointee.maxExtent,
                    flags: sim.pointee.flags)
            }
        }
        let sim = sims.baseAddress! + index
        return ContactBodySnapshot(
            transform: sim.pointee.transform,
            localCenter: sim.pointee.localCenter,
            center: sim.pointee.center,
            maxExtent: sim.pointee.maxExtent,
            flags: sim.pointee.flags)
    }

    /// True if the encoded index refers to a static body.
    @inline(__always)
    func isStaticSimIndex(_ encodedIndex: Int32) -> Bool {
        encodedIndex < B3.nullIndex
    }

    /// Update awake contacts and apply state transitions.
    func collide(_ context: inout StepContext) {
        // Gather contacts from all graph colors, then the non-touching contacts. Reuse the list
        // across steps; rebuilding it allocated a new array every frame.
        contactIndexScratch.removeAll(keepingCapacity: true)
        for i in 0..<Int(B3.graphColorCount) {
            let color = constraintGraph.colors[i]
            contactIndexScratch.append(contentsOf: color.convexContacts)
            for spec in color.contacts {
                contactIndexScratch.append(spec.contactId)
            }
        }

        contactIndexScratch.append(
            contentsOf: solverSets[Int(SetType.awakeSet.rawValue)].contactIndices)

        guard contactIndexScratch.isEmpty == false else {
            return
        }
        let contactIndices = contactIndexScratch

        meshContactScratch.recycledContacts = 0
        meshContactScratch.satCalls = 0
        meshContactScratch.satCacheHits = 0

        let contactCapacity = UInt32(maxInt(Int32(contacts.count), 8))
        let workers = maxInt(1, minInt(workerCount, 8))

        // Each worker collides its own slice and writes only contacts inside that slice, so the
        // passes never touch the same element. C does the same through b3ParallelFor.
        if workers > 1, contactIndices.count >= Self.parallelCollideMinCount {
            var bits = [BitSet](repeating: BitSet(), count: Int(workers))
            for i in 0..<Int(workers) {
                bits[i].setBitCountAndClear(contactCapacity)
            }

            let chunk = (contactIndices.count + Int(workers) - 1) / Int(workers)
            var recycled = [Int32](repeating: 0, count: Int(workers))
            var workerSatCalls = [Int32](repeating: 0, count: Int(workers))
            var workerSatHits = [Int32](repeating: 0, count: Int(workers))

            let awakeSetIndex = Int(SetType.awakeSet.rawValue)
            let staticSetIndex = Int(SetType.staticSet.rawValue)
            solverSets[awakeSetIndex].bodySims.withUnsafeBufferPointer { awakeSims in
                solverSets[staticSetIndex].bodySims.withUnsafeBufferPointer { staticSims in
                    fatAABBs.withUnsafeBufferPointer { fat in
                        bits.withUnsafeMutableBufferPointer { bitBuffer in
                            recycled.withUnsafeMutableBufferPointer { recycledBuffer in
                              workerSatCalls.withUnsafeMutableBufferPointer { satCallsBuffer in
                               workerSatHits.withUnsafeMutableBufferPointer { satHitsBuffer in
                                contacts.withUnsafeMutableBufferPointer { contactBuffer in
                                    contactIndices.withUnsafeBufferPointer { indexBuffer in
                                        let bitBase = bitBuffer.baseAddress!
                                        let recycledBase = recycledBuffer.baseAddress!
                                        let satCallsBase = satCallsBuffer.baseAddress!
                                        let satHitsBase = satHitsBuffer.baseAddress!
                                        let contactBase = contactBuffer.baseAddress!
                                        let indexBase = indexBuffer.baseAddress!
                                        let indexCount = indexBuffer.count
                                        DispatchQueue.concurrentPerform(iterations: Int(workers)) { w in
                                            let start = w * chunk
                                            let end = min(indexCount, start + chunk)
                                            guard start < end else { return }
                                            var recycledCount: Int32 = 0
                                            var localStore = ContactStore(
                                                UnsafeMutableBufferPointer(
                                                    start: contactBase, count: contacts.count))
                                            var localScratch = MeshContactScratch()
                                            withUnsafeMutablePointer(to: &localScratch) { scratch in
                                                for i in start..<end {
                                                    collideOneContact(
                                                        indexBase[i], &bitBase[w], store: &localStore,
                                                        &recycledCount, scratch, awakeSims, staticSims, fat)
                                                }
                                            }
                                            recycledBase[w] = recycledCount
                                            satCallsBase[w] = localScratch.satCalls
                                            satHitsBase[w] = localScratch.satCacheHits
                                        }
                                    }
                                }
                               }
                              }
                            }
                        }
                    }
                }
            }

            var stateBits = BitSet()
            stateBits.setBitCountAndClear(contactCapacity)
            var totalRecycled: Int32 = 0
            for i in 0..<Int(workers) {
                stateBits.inPlaceUnion(bits[i])
                totalRecycled += recycled[i]
                meshContactScratch.satCalls += workerSatCalls[i]
                meshContactScratch.satCacheHits += workerSatHits[i]
            }
            taskContexts[0].recycledContactCount = totalRecycled
            satCallCount += meshContactScratch.satCalls
            satCacheHitCount += meshContactScratch.satCacheHits
            collideStatePass(stateBits)
            return
        }

        var stateBits = BitSet(bitCapacity: contactCapacity)
        stateBits.setBitCountAndClear(contactCapacity)

        // Borrow the body-sim and AABB buffers once for the whole pass. Indexing `solverSets[...]`
        // per contact copied the SolverSet and retained its five arrays twice per contact.
        let awakeSetIndex = Int(SetType.awakeSet.rawValue)
        let staticSetIndex = Int(SetType.staticSet.rawValue)
        solverSets[awakeSetIndex].bodySims.withUnsafeBufferPointer { awakeSims in
            solverSets[staticSetIndex].bodySims.withUnsafeBufferPointer { staticSims in
                fatAABBs.withUnsafeBufferPointer { fat in
                    collideEachContact(
                        contactIndices, &stateBits, awakeSims, staticSims, fat)
                }
            }
        }
        taskContexts[0].recycledContactCount = meshContactScratch.recycledContacts
        satCallCount += meshContactScratch.satCalls
        satCacheHitCount += meshContactScratch.satCacheHits
        collideStatePass(stateBits)
    }

    /// Below this contact count the dispatch cost dominates and the pass stays serial.
    private static let parallelCollideMinCount = 512

    /// Apply contact state transitions in contact id order. This phase is serial because it moves
    /// contacts between the graph and the awake set.
    private func collideStatePass(_ stateBits: BitSet) {
        // Process contact state changes in contact id order. Iterating set bits avoids scanning
        // the entire contact table (tens of thousands of entries) every step.
        stateBits.forEachSetBit { contactId in
            // Read the flag word first: most slots are untouched on a given step, and copying the
            // whole contact for them was pure overhead.
            let flags = contacts[Int(contactId)].flags
            guard flags != 0 else {
                return
            }
            var contact = contacts[Int(contactId)]

            if (flags & ContactFlags.simDisjoint.rawValue) != 0 {
                // Bounding boxes no longer overlap
                destroyContact(contactId, false)
            } else if (flags & ContactFlags.simStartedTouching.rawValue) != 0 {
                b3Assert(contact.islandId == B3.nullIndex)

                if (flags & ContactFlags.enableContactEvents.rawValue) != 0 {
                    let shapeA = shapes[Int(contact.shapeIdA)]
                    let shapeB = shapes[Int(contact.shapeIdB)]
                    var event = ContactBeginTouchEvent()
                    event.shapeIdA = ShapeID(
                        index1: shapeA.id + 1, world0: worldId, generation: shapeA.generation
                    )
                    event.shapeIdB = ShapeID(
                        index1: shapeB.id + 1, world0: worldId, generation: shapeB.generation
                    )
                    event.contactId = ContactID(
                        index1: contactId + 1, world0: worldId, generation: contact.generation
                    )
                    contactBeginEvents.append(event)
                }

                b3Assert(contact.manifoldCount > 0)
                b3Assert(contact.setIndex == SetType.awakeSet.rawValue)

                // Link first because this wakes colliding bodies and ensures the body sims are in
                // the correct place.
                contact.flags &= ~ContactFlags.simStartedTouching.rawValue
                contact.flags |= ContactFlags.touching.rawValue
                contacts[Int(contactId)] = contact
                linkContact(contactId)

                contact = contacts[Int(contactId)]

                // Make sure these didn't change
                b3Assert(contact.colorIndex == B3.nullIndex)

                let oldLocalIndex = contact.localIndex

                addContactToGraph(contactId)
                removeNonTouchingContact(Int32(SetType.awakeSet.rawValue), oldLocalIndex)
            } else if (flags & ContactFlags.simStoppedTouching.rawValue) != 0 {
                if (flags & ContactFlags.enableContactEvents.rawValue) != 0 {
                    let shapeA = shapes[Int(contact.shapeIdA)]
                    let shapeB = shapes[Int(contact.shapeIdB)]
                    var event = ContactEndTouchEvent()
                    event.shapeIdA = ShapeID(
                        index1: shapeA.id + 1, world0: worldId, generation: shapeA.generation
                    )
                    event.shapeIdB = ShapeID(
                        index1: shapeB.id + 1, world0: worldId, generation: shapeB.generation
                    )
                    event.contactId = ContactID(
                        index1: contactId + 1, world0: worldId, generation: contact.generation
                    )
                    contactEndEvents.pending.append(event)
                }

                contact.flags &= ~ContactFlags.simStoppedTouching.rawValue
                contact.flags &= ~ContactFlags.touching.rawValue
                contacts[Int(contactId)] = contact

                b3Assert(contact.manifoldCount == 0)

                // Cache these here for the remove below
                let colorIndex = contact.colorIndex
                let localIndex = contact.localIndex

                unlinkContact(contactId)

                contact = contacts[Int(contactId)]
                let bodyIdA = contact.edges[0].bodyId
                let bodyIdB = contact.edges[1].bodyId

                addNonTouchingContact(contactId)

                let isMeshContact = (contact.flags & ContactFlags.simMeshContact.rawValue) != 0
                removeContactFromGraph(bodyIdA, bodyIdB, colorIndex, localIndex, isMeshContact)
            }
        }
    }

    /// Update the awake contacts one by one, sharing one store across the whole pass.
    ///
    /// `ContactStore` holds a write reference to the world's contact array. Taking it per contact
    /// made the array multiply referenced for every contact, so each write copied the entire array;
    /// with tens of thousands of contacts that copy dominated the collide phase.
    private func collideEachContact(
        _ contactIds: [Int32], _ stateBits: inout BitSet,
        _ awakeSims: UnsafeBufferPointer<BodySim>, _ staticSims: UnsafeBufferPointer<BodySim>,
        _ fatAABBs: UnsafeBufferPointer<AABB>
    ) {
        var store = ContactStore(&contacts)
        withUnsafeMutablePointer(to: &meshContactScratch) { scratch in
            for contactId in contactIds {
                collideOneContact(
                    contactId, &stateBits, store: &store, nil, scratch,
                    awakeSims, staticSims, fatAABBs)
            }
        }
    }

    private func collideOneContact(
        _ contactId: Int32, _ stateBits: inout BitSet,
        store: inout ContactStore,
        _ recycledCount: UnsafeMutablePointer<Int32>?,
        _ scratch: UnsafeMutablePointer<MeshContactScratch>,
        _ awakeSims: UnsafeBufferPointer<BodySim>, _ staticSims: UnsafeBufferPointer<BodySim>,
        _ fatAABBs: UnsafeBufferPointer<AABB>
    ) {
        // Borrow the stored contact so recycling does not retain and then copy its manifolds.
        let update = store.withContact(contactId) { contact -> (ContactBodySnapshot, ContactBodySnapshot, Bool)? in
            b3Assert(contact.contactId == contactId)

            let shapeIdA = contact.shapeIdA
            let shapeIdB = contact.shapeIdB

            // Do proxies still overlap?
            let overlap = aabbOverlaps(fatAABBs[Int(shapeIdA)], fatAABBs[Int(shapeIdB)])
            if overlap == false {
                // This contact will be destroyed
                contact.flags |= ContactFlags.simDisjoint.rawValue
                contact.flags &= ~ContactFlags.simTouching.rawValue
                stateBits.setBit(UInt32(contactId))
                return nil
            }

            // Update contact respecting shape/body order (A,B)
            let encodedA = contact.encodedBodySimA
            let encodedB = contact.encodedBodySimB
            let locA = resolveContactBodySimIndex(encodedA, contact.edges[0].bodyId)
            let locB = resolveContactBodySimIndex(encodedB, contact.edges[1].bodyId)
            let bodySimA = contactBodySnapshotAt(locA.set, locA.local, awakeSims, staticSims)
            let bodySimB = contactBodySnapshotAt(locB.set, locB.local, awakeSims, staticSims)

            let wasTouching = (contact.flags & ContactFlags.simTouching.rawValue) != 0

            let transformA = bodySimA.transform
            let transformB = bodySimB.transform

            let isFast = ((bodySimA.flags | bodySimB.flags) & BodyFlags.isFast.rawValue) != 0

            let recycleDistance = contactRecycleDistance
            let speculativeDistance = B3.speculativeDistance
            let recycleDistanceNonTouching = minFloat(recycleDistance, speculativeDistance)
            let recycleTolerance = wasTouching ? recycleDistance : recycleDistanceNonTouching

            // Contact recycling optimization.
            if isFast == false && recycleDistance > 0
                && (contact.flags & ContactFlags.relativeTransformValid.rawValue) != 0
                && (contact.flags & ContactFlags.recycle.rawValue) != 0
            {
                // The scalar part of b3InvMulQuat is just the quaternion dot product.
                let angleA = dotQuat(transformA.q, contact.cachedRotationA)
                let angleB = dotQuat(transformB.q, contact.cachedRotationB)
                let angularDistance = minFloat(angleA * angleA, angleB * angleB)

                let xf = invMulWorldTransforms(transformA, transformB)
                let xfc = contact.cachedRelativePose
                let maxExtentA = isStaticSimIndex(encodedA) ? Vec3() : bodySimA.maxExtent
                let maxExtentB = isStaticSimIndex(encodedB) ? Vec3() : bodySimB.maxExtent
                let maxExtent = maxVec3(maxExtentA, maxExtentB)

                let distSquared = distanceSquared(xf.p, xfc.p)

                if angularDistance > B3.contactRecycleAngularDistance
                    && distSquared < recycleTolerance * recycleTolerance
                {
                    let distance = distSquared.squareRoot()
                    let slack = recycleTolerance - distance

                    let qr = invMulQuat(xfc.q, xf.q)
                    let arc = modifiedCross(absVec3(qr.v), maxExtent)

                    let arcSq = 4.0 * lengthSquared(arc)
                    if arcSq < slack * slack {
                        let dqA = mulQuat(transformA.q, conjugate(contact.cachedRotationA))
                        let dqB = mulQuat(transformB.q, conjugate(contact.cachedRotationB))
                        let matrixA = makeMatrixFromQuat(dqA)
                        let matrixB = makeMatrixFromQuat(dqB)

                        // Minimize round-off
                        let dc = subPos(bodySimB.center, bodySimA.center)

                        let manifoldCount = contact.manifoldCount
                        for manifoldIndex in 0..<Int(manifoldCount) {
                            var manifold = contact.manifolds[manifoldIndex]
                            let normal = manifold.normal
                            let pointCount = manifold.pointCount

                            for pointIndex in 0..<Int(pointCount) {
                                // Keep anchors but update separation, same as sub-stepping.
                                var mp = manifold.points[pointIndex]
                                let rA = mulMV(matrixA, mp.anchorA)
                                let rB = mulMV(matrixB, mp.anchorB)
                                let dp = dc + (rB - rA)
                                mp.separation = mp.baseSeparation + dot(dp, normal)
                                mp.normalVelocity = 0
                                mp.persisted = true
                                manifold.points[pointIndex] = mp
                            }

                            contact.manifolds[manifoldIndex] = manifold
                        }

                        // Contact is recycled. This also skips updating other aspects of the contact
                        // such as material parameters.
                        if let recycledCount {
                            recycledCount.pointee += 1
                        } else {
                            scratch.pointee.recycledContacts += 1
                        }
                        return nil
                    }
                }
            }

            // Caching for contact recycling.
            contact.cachedRotationA = transformA.q
            contact.cachedRotationB = transformB.q
            contact.cachedRelativePose = invMulWorldTransforms(transformA, transformB)
            contact.flags |= ContactFlags.relativeTransformValid.rawValue

            return (bodySimA, bodySimB, wasTouching)
        }
        guard let (bodySimA, bodySimB, wasTouching) = update else { return }

        // This updates solid contacts
        let touching = updateContact(
            contactId, bodySimA.localCenter, bodySimA.transform, bodySimB.localCenter, bodySimB.transform,
            ((bodySimA.flags | bodySimB.flags) & BodyFlags.isFast.rawValue) != 0, scratch, &store
        )

        store.withContact(contactId) { contact in

            if touching && wasTouching && contact.flags & ContactFlags.simMeshContact.rawValue != 0 {
                constraintGraph.colors[Int(contact.colorIndex)].contacts[Int(contact.localIndex)].manifoldCount =
                    UInt16(contact.manifoldCount)
            }

            if touching && wasTouching == false {
                contact.flags |= ContactFlags.simStartedTouching.rawValue
                stateBits.setBit(UInt32(contactId))
            } else if touching == false && wasTouching {
                contact.flags |= ContactFlags.simStoppedTouching.rawValue
                stateBits.setBit(UInt32(contactId))
            }

            if touching {
                let manifoldCount = contact.manifoldCount
                for manifoldIndex in 0..<Int(manifoldCount) {
                    var manifold = contact.manifolds[manifoldIndex]
                    let pointCount = manifold.pointCount
                    for pointIndex in 0..<Int(pointCount) {
                        // Cache separation
                        var mp = manifold.points[pointIndex]
                        mp.baseSeparation = mp.separation
                        manifold.points[pointIndex] = mp
                    }
                    contact.manifolds[manifoldIndex] = manifold
                }
            }
        }
    }

    /// Add a non-touching contact to the awake solver set.
    func addNonTouchingContact(_ contactId: Int32) {
        var contact = contacts[Int(contactId)]
        b3Assert(contact.setIndex == SetType.awakeSet.rawValue)
        let awakeSetIndex = Int(SetType.awakeSet.rawValue)
        contact.colorIndex = B3.nullIndex
        contact.localIndex = Int32(solverSets[awakeSetIndex].contactIndices.count)
        solverSets[awakeSetIndex].contactIndices.append(contact.contactId)
        contacts[Int(contactId)] = contact
    }

    /// Remove a non-touching contact from a solver set.
    func removeNonTouchingContact(_ setIndex: Int32, _ localIndex: Int32) {
        let movedIndex = removeSwapArray(
            &solverSets[Int(setIndex)].contactIndices, Int(localIndex))
        if movedIndex != B3.nullIndex {
            let movedContactIndex = solverSets[Int(setIndex)].contactIndices[Int(localIndex)]
            var movedContact = contacts[Int(movedContactIndex)]
            b3Assert(movedContact.setIndex == setIndex)
            b3Assert(movedContact.colorIndex == B3.nullIndex)
            b3Assert(movedContact.localIndex == movedIndex)
            movedContact.localIndex = localIndex
            contacts[Int(movedContactIndex)] = movedContact
        }
    }
}
