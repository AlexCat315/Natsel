

// Solver set transfers, waking, and merging (solver_set.c).

extension World {
    /// Wake a solver set. Does not merge islands.
    ///
    /// Contacts can be in several places:
    /// 1. non-touching contacts in the disabled set
    /// 2. non-touching contacts already in the awake set
    /// 3. touching contacts in the sleeping set
    /// This handles contact types 1 and 3.
    func wakeSolverSet(_ setIndex: Int32) {
        b3Assert(setIndex >= SetType.firstSleepingSet.rawValue)

        let set = solverSets[Int(setIndex)]
        var awakeSet = solverSets[Int(SetType.awakeSet.rawValue)]
        var disabledSet = solverSets[Int(SetType.disabledSet.rawValue)]

        let bodyCount = set.bodySims.count
        for i in 0..<bodyCount {
            let simSrc = set.bodySims[i]

            var body = bodies[Int(simSrc.bodyId)]
            b3Assert(body.setIndex == setIndex)
            body.setIndex = SetType.awakeSet.rawValue
            body.localIndex = Int32(awakeSet.bodySims.count)

            // Reset sleep timer
            body.sleepTime = 0
            bodies[Int(simSrc.bodyId)] = body

            awakeSet.bodySims.append(simSrc)

            var state = BodyState.identity
            state.flags = body.flags
            awakeSet.bodyStates.append(state)

            // Move non-touching contacts from disabled set to awake set.
            let encodedBodySimIndex = body.localIndex
            var contactKey = body.headContactKey
            while contactKey != B3.nullIndex {
                let edgeIndex = Int(contactKey & 1)
                let contactId = contactKey >> 1

                var contact = contacts[Int(contactId)]
                contactKey = contact.edges[edgeIndex].nextKey

                if contact.setIndex != SetType.disabledSet.rawValue {
                    b3Assert(
                        contact.setIndex == SetType.awakeSet.rawValue || contact.setIndex == setIndex
                    )

                    // If this contact is awake but not touching, fix up the body sim index here.
                    if contact.setIndex == SetType.awakeSet.rawValue {
                        b3Assert(contact.colorIndex == B3.nullIndex)
                        if edgeIndex == 0 {
                            contact.encodedBodySimA = encodedBodySimIndex
                        } else {
                            contact.encodedBodySimB = encodedBodySimIndex
                        }
                        contacts[Int(contactId)] = contact
                    }

                    continue
                }

                let localIndex = contact.localIndex

                b3Assert(0 <= localIndex && Int(localIndex) < disabledSet.contactIndices.count)
                b3Assert(disabledSet.contactIndices[Int(localIndex)] == contactId)

                b3Assert((contact.flags & ContactFlags.simTouching.rawValue) == 0 && contact.manifoldCount == 0)

                contact.setIndex = SetType.awakeSet.rawValue
                contact.localIndex = Int32(awakeSet.contactIndices.count)
                awakeSet.contactIndices.append(contactId)

                if edgeIndex == 0 {
                    contact.encodedBodySimA = encodedBodySimIndex
                } else {
                    contact.encodedBodySimB = encodedBodySimIndex
                }

                contacts[Int(contactId)] = contact

                let movedLocalIndex = removeSwapArray(&disabledSet.contactIndices, Int(localIndex))
                if movedLocalIndex != B3.nullIndex {
                    // fix moved element
                    let movedContactIndex = disabledSet.contactIndices[Int(localIndex)]
                    var movedContact = contacts[Int(movedContactIndex)]
                    b3Assert(movedContact.localIndex == movedLocalIndex)
                    movedContact.localIndex = localIndex
                    contacts[Int(movedContactIndex)] = movedContact
                }
            }
        }

        // Transfer touching contacts from sleeping set to constraint graph.
        let contactCount = set.contactIndices.count
        for i in 0..<contactCount {
            let contactId = set.contactIndices[i]
            var contact = contacts[Int(contactId)]
            b3Assert((contact.flags & ContactFlags.simTouching.rawValue) != 0)
            b3Assert(contact.setIndex == setIndex)
            addContactToGraph(contactId)
            contact = contacts[Int(contactId)]
            contact.setIndex = SetType.awakeSet.rawValue
            contacts[Int(contactId)] = contact
        }

        // Transfer joints from sleeping set to awake set.
        let jointCount = set.jointSims.count
        for i in 0..<jointCount {
            let jointSim = set.jointSims[i]
            var joint = joints[Int(jointSim.jointId)]
            b3Assert(joint.setIndex == setIndex)
            addJointToGraph(jointSim, jointSim.jointId)
            joint = joints[Int(jointSim.jointId)]
            joint.setIndex = SetType.awakeSet.rawValue
            joints[Int(jointSim.jointId)] = joint
        }

        // Transfer islands from sleeping set to awake set.
        let islandCount = set.islandSims.count
        for i in 0..<islandCount {
            let islandSrc = set.islandSims[i]
            var island = islands[Int(islandSrc.islandId)]
            island.setIndex = SetType.awakeSet.rawValue
            island.localIndex = Int32(awakeSet.islandSims.count)
            islands[Int(islandSrc.islandId)] = island
            awakeSet.islandSims.append(islandSrc)
        }

        solverSets[Int(SetType.awakeSet.rawValue)] = awakeSet
        solverSets[Int(SetType.disabledSet.rawValue)] = disabledSet

        // Destroy the sleeping set.
        destroySolverSet(setIndex)
    }

    // MARK: Merging

    /// Called when joints are created between sets. The sets can continue sleeping if both are
    /// asleep. Islands get merged when the set is woken.
    func mergeSolverSets(_ setId1: Int32, _ setId2: Int32) {
        b3Assert(setId1 >= SetType.firstSleepingSet.rawValue)
        b3Assert(setId2 >= SetType.firstSleepingSet.rawValue)

        var bigId = setId1
        var smallId = setId2
        if solverSets[Int(bigId)].bodySims.count < solverSets[Int(smallId)].bodySims.count {
            swap(&bigId, &smallId)
        }

        var set1 = solverSets[Int(bigId)]
        let set2 = solverSets[Int(smallId)]

        // transfer bodies
        for i in 0..<set2.bodySims.count {
            let simSrc = set2.bodySims[i]

            var body = bodies[Int(simSrc.bodyId)]
            b3Assert(body.setIndex == smallId)
            body.setIndex = bigId
            body.localIndex = Int32(set1.bodySims.count)
            bodies[Int(simSrc.bodyId)] = body

            set1.bodySims.append(simSrc)
        }

        // transfer contacts
        for contactId in set2.contactIndices {
            var contact = contacts[Int(contactId)]
            b3Assert(contact.setIndex == smallId)
            contact.setIndex = bigId
            contact.localIndex = Int32(set1.contactIndices.count)
            contacts[Int(contactId)] = contact
            set1.contactIndices.append(contactId)
        }

        // transfer joints
        for jointSim in set2.jointSims {
            var joint = joints[Int(jointSim.jointId)]
            b3Assert(joint.setIndex == smallId)
            joint.setIndex = bigId
            joint.localIndex = Int32(set1.jointSims.count)
            joints[Int(jointSim.jointId)] = joint

            set1.jointSims.append(jointSim)
        }

        // transfer islands
        for islandSrc in set2.islandSims {
            let islandId = islandSrc.islandId

            var island = islands[Int(islandId)]
            island.setIndex = bigId
            island.localIndex = Int32(set1.islandSims.count)
            islands[Int(islandId)] = island

            set1.islandSims.append(islandSrc)
        }

        solverSets[Int(bigId)] = set1

        // destroy the merged set
        destroySolverSet(smallId)
    }

    // MARK: Transfers

    /// Move a body between solver sets. The body must have no contacts.
    func transferBody(_ targetSetIndex: Int32, _ sourceSetIndex: Int32, _ bodyId: Int32) {
        b3Assert(bodies[Int(bodyId)].headContactKey == B3.nullIndex)

        if targetSetIndex == sourceSetIndex {
            return
        }

        var body = bodies[Int(bodyId)]
        var sourceSet = solverSets[Int(sourceSetIndex)]
        var targetSet = solverSets[Int(targetSetIndex)]

        let sourceIndex = body.localIndex
        var targetSim = sourceSet.bodySims[Int(sourceIndex)]

        let targetIndex = targetSet.bodySims.count
        targetSet.bodySims.append(targetSim)

        // Clear transient body flags
        body.flags &= ~BodyFlags.bodyTransientFlags.rawValue
        targetSim.flags &= ~(BodyFlags.isFast.rawValue | BodyFlags.bodyTransientFlags.rawValue)
        targetSet.bodySims[targetIndex] = targetSim

        // Remove body sim from the solver set that owns it
        let movedIndex = removeSwapArray(&sourceSet.bodySims, Int(sourceIndex))
        if movedIndex != B3.nullIndex {
            // Fix moved body index
            let movedId = sourceSet.bodySims[Int(sourceIndex)].bodyId
            var movedBody = bodies[Int(movedId)]
            b3Assert(movedBody.localIndex == movedIndex)
            movedBody.localIndex = sourceIndex
            bodies[Int(movedId)] = movedBody

            // The moved body's contacts cache its old awake index, so refresh them.
            if sourceSet.setIndex == SetType.awakeSet.rawValue {
                refreshBodyContactIndices(movedId)
            }
        }

        if sourceSet.setIndex == SetType.awakeSet.rawValue {
            removeSwapArray(&sourceSet.bodyStates, Int(sourceIndex))
        } else if targetSet.setIndex == SetType.awakeSet.rawValue {
            var state = BodyState.identity
            state.flags = body.flags
            targetSet.bodyStates.append(state)
        }

        body.setIndex = targetSet.setIndex
        body.localIndex = Int32(targetIndex)
        bodies[Int(bodyId)] = body

        solverSets[Int(sourceSetIndex)] = sourceSet
        solverSets[Int(targetSetIndex)] = targetSet
    }

    /// Move a joint between solver sets.
    func transferJoint(_ targetSetIndex: Int32, _ sourceSetIndex: Int32, _ jointId: Int32) {
        if targetSetIndex == sourceSetIndex {
            return
        }

        var joint = joints[Int(jointId)]
        var sourceSet = solverSets[Int(sourceSetIndex)]
        var targetSet = solverSets[Int(targetSetIndex)]

        let localIndex = joint.localIndex
        let colorIndex = joint.colorIndex

        // Retrieve source.
        let sourceSim: JointSim
        if sourceSet.setIndex == SetType.awakeSet.rawValue {
            b3Assert(0 <= colorIndex && colorIndex < B3.graphColorCount)
            sourceSim = constraintGraph.colors[Int(colorIndex)].jointSims[Int(localIndex)]
        } else {
            b3Assert(colorIndex == B3.nullIndex)
            sourceSim = sourceSet.jointSims[Int(localIndex)]
        }

        // Create target and copy. Fix joint.
        if targetSet.setIndex == SetType.awakeSet.rawValue {
            addJointToGraph(sourceSim, jointId)
            joint = joints[Int(jointId)]
            joint.setIndex = SetType.awakeSet.rawValue
            joints[Int(jointId)] = joint
        } else {
            joint.setIndex = targetSet.setIndex
            joint.localIndex = Int32(targetSet.jointSims.count)
            joint.colorIndex = B3.nullIndex
            joints[Int(jointId)] = joint

            targetSet.jointSims.append(sourceSim)
        }

        // Destroy source.
        if sourceSet.setIndex == SetType.awakeSet.rawValue {
            removeJointFromGraph(joint.edges[0].bodyId, joint.edges[1].bodyId, colorIndex, localIndex)
        } else {
            let movedIndex = removeSwapArray(&sourceSet.jointSims, Int(localIndex))
            if movedIndex != B3.nullIndex {
                // fix swapped element
                let movedId = sourceSet.jointSims[Int(localIndex)].jointId
                var movedJoint = joints[Int(movedId)]
                movedJoint.localIndex = localIndex
                joints[Int(movedId)] = movedJoint
            }
        }

        solverSets[Int(sourceSetIndex)] = sourceSet
        solverSets[Int(targetSetIndex)] = targetSet
    }
}
