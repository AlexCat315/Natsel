

// Island splitting and sleeping.

extension World {
    /// Refresh the encoded body sim index stored in this body's contacts.
    func refreshBodyContactIndices(_ bodyId: Int32) {
        let body = bodies[Int(bodyId)]
        let encodedBodySimIndex = encodeBodySimIndex(body)

        var contactKey = body.headContactKey
        while contactKey != B3.nullIndex {
            let edgeIndex = Int(contactKey & 1)
            let contactId = contactKey >> 1

            var contact = contacts[Int(contactId)]
            contactKey = contact.edges[edgeIndex].nextKey

            if edgeIndex == 0 {
                contact.encodedBodySimA = encodedBodySimIndex
            } else {
                contact.encodedBodySimB = encodedBodySimIndex
            }
            contacts[Int(contactId)] = contact
        }
    }

    // MARK: Union find

    @inline(__always)
    private func islandFindParent(_ parents: inout [Int32], _ node: Int32) -> Int32 {
        var node = node
        while parents[Int(node)] != node {
            let grandParent = parents[Int(parents[Int(node)])]
            parents[Int(node)] = grandParent
            node = grandParent
        }
        return node
    }

    @inline(__always)
    private func islandUnion(
        _ parents: inout [Int32], _ ranks: inout [Int32], _ node1: Int32, _ node2: Int32,
        _ contactCounts: inout [Int32], _ jointCounts: inout [Int32]
    ) {
        let root1 = islandFindParent(&parents, node1)
        let root2 = islandFindParent(&parents, node2)
        if root1 != root2 {
            if ranks[Int(root1)] < ranks[Int(root2)] {
                parents[Int(root1)] = root2
                contactCounts[Int(root2)] += contactCounts[Int(root1)]
                jointCounts[Int(root2)] += jointCounts[Int(root1)]
            } else if ranks[Int(root1)] > ranks[Int(root2)] {
                parents[Int(root2)] = root1
                contactCounts[Int(root1)] += contactCounts[Int(root2)]
                jointCounts[Int(root1)] += jointCounts[Int(root2)]
            } else {
                parents[Int(root2)] = root1
                ranks[Int(root1)] += 1
                contactCounts[Int(root1)] += contactCounts[Int(root2)]
                jointCounts[Int(root1)] += jointCounts[Int(root2)]
            }
        }
    }

    /// Split an island because some contacts and/or joints have been removed. Uses union find.
    func splitIsland(_ baseId: Int32) {
        let baseIsland = islands[Int(baseId)]
        b3Assert(baseIsland.constraintRemoveCount > 0)
        b3Assert(baseIsland.setIndex == SetType.awakeSet.rawValue)

        let baseBodyIds = baseIsland.bodies
        let baseContacts = baseIsland.contacts
        let baseJoints = baseIsland.joints

        let baseBodyCount = baseBodyIds.count
        let baseContactCount = baseContacts.count
        let baseJointCount = baseJoints.count

        var parents = [Int32](repeating: 0, count: baseBodyCount)
        var ranks = [Int32](repeating: 0, count: baseBodyCount)
        var contactCounts = [Int32](repeating: 0, count: baseBodyCount)
        var jointCounts = [Int32](repeating: 0, count: baseBodyCount)
        for i in 0..<baseBodyCount {
            parents[i] = Int32(i)
        }

        // Union over contacts, tracking per-component contact counts
        for i in 0..<baseContactCount {
            let bodyIdA = baseContacts[i].bodyIdA
            let bodyIdB = baseContacts[i].bodyIdB
            let islandIndexA = bodies[Int(bodyIdA)].islandIndex
            let islandIndexB = bodies[Int(bodyIdB)].islandIndex

            // Only connect non-static bodies that still belong to this island.
            let validA = islandIndexA >= 0 && Int(islandIndexA) < baseBodyCount
            let validB = islandIndexB >= 0 && Int(islandIndexB) < baseBodyCount
            if validA && validB {
                islandUnion(&parents, &ranks, islandIndexA, islandIndexB, &contactCounts, &jointCounts)
                let root = islandFindParent(&parents, islandIndexA)
                contactCounts[Int(root)] += 1
            } else if validA || validB {
                let islandIndex = validA ? islandIndexA : islandIndexB
                let root = islandFindParent(&parents, islandIndex)
                contactCounts[Int(root)] += 1
            }
        }

        // Union over joints, tracking per-component joint counts
        for i in 0..<baseJointCount {
            let bodyIdA = baseJoints[i].bodyIdA
            let bodyIdB = baseJoints[i].bodyIdB
            let islandIndexA = bodies[Int(bodyIdA)].islandIndex
            let islandIndexB = bodies[Int(bodyIdB)].islandIndex

            let validA = islandIndexA >= 0 && Int(islandIndexA) < baseBodyCount
            let validB = islandIndexB >= 0 && Int(islandIndexB) < baseBodyCount
            if validA && validB {
                islandUnion(&parents, &ranks, islandIndexA, islandIndexB, &contactCounts, &jointCounts)
                let root = islandFindParent(&parents, islandIndexA)
                jointCounts[Int(root)] += 1
            } else if validA || validB {
                let islandIndex = validA ? islandIndexA : islandIndexB
                let root = islandFindParent(&parents, islandIndex)
                jointCounts[Int(root)] += 1
            }
        }

        // Flatten all parent indices and count connected components.
        var componentCount = 0
        for i in 0..<baseBodyCount {
            parents[i] = islandFindParent(&parents, Int32(i))
            if parents[i] == Int32(i) {
                componentCount += 1
            }
        }

        // Island is still fully connected, no split needed.
        if componentCount == 1 {
            islands[Int(baseId)].constraintRemoveCount = 0
            return
        }

        // Detach the base island so destroying it won't free the arrays.
        var base = islands[Int(baseId)]
        base.bodies = []
        base.contacts = []
        base.joints = []
        islands[Int(baseId)] = base

        // Map from body index to new island index. Only set for root bodies.
        var rootMap = [Int32](repeating: B3.nullIndex, count: baseBodyCount)
        var componentBodyCounts = [Int32](repeating: 0, count: componentCount)
        var componentContactCounts = [Int32](repeating: 0, count: componentCount)
        var componentJointCounts = [Int32](repeating: 0, count: componentCount)
        var islandCount = 0

        for i in 0..<baseBodyCount {
            let rootIndex = parents[i]
            if rootMap[Int(rootIndex)] == B3.nullIndex {
                rootMap[Int(rootIndex)] = Int32(islandCount)
                componentBodyCounts[islandCount] = 0
                componentContactCounts[islandCount] = contactCounts[Int(rootIndex)]
                componentJointCounts[islandCount] = jointCounts[Int(rootIndex)]
                islandCount += 1
            }

            componentBodyCounts[Int(rootMap[Int(rootIndex)])] += 1
        }

        b3Assert(islandCount == componentCount)

        // Map from new island index to island id
        var islandIds = [Int32](repeating: B3.nullIndex, count: islandCount)

        // Create new islands.
        for i in 0..<islandCount {
            islandIds[i] = createIsland(SetType.awakeSet.rawValue)
        }

        // Assign bodies to new islands. Reserve each island's final capacity up front so the
        // rebuild does not repeatedly grow (and copy) the arrays.
        for i in 0..<islandCount {
            let newIslandId = islandIds[i]
            islands[Int(newIslandId)].bodies.reserveCapacity(Int(componentBodyCounts[i]))
        }
        for i in 0..<baseBodyCount {
            let bodyId = baseBodyIds[i]
            let root = islandFindParent(&parents, Int32(i))

            guard let newIslandId = islandIds[Int(rootMap[Int(root)])] as Int32? else {
                continue
            }
            var body = bodies[Int(bodyId)]
            body.islandId = newIslandId
            body.islandIndex = Int32(islands[Int(newIslandId)].bodies.count)
            islands[Int(newIslandId)].bodies.append(bodyId)
            bodies[Int(bodyId)] = body
        }

        // Assign contacts to the island of their bodies
        for i in 0..<baseContactCount {
            let link = baseContacts[i]
            var contact = contacts[Int(link.contactId)]

            // Static bodies don't have an island id.
            let islandIdA = bodies[Int(link.bodyIdA)].islandId
            let islandIdB = bodies[Int(link.bodyIdB)].islandId
            let targetIslandId = islandIdA != B3.nullIndex ? islandIdA : islandIdB

            contact.islandId = targetIslandId
            contact.islandIndex = Int32(islands[Int(targetIslandId)].contacts.count)
            islands[Int(targetIslandId)].contacts.append(link)
            contacts[Int(link.contactId)] = contact
        }

        // Assign joints to the island of their bodies
        for i in 0..<baseJointCount {
            let link = baseJoints[i]
            var joint = joints[Int(link.jointId)]

            // Static bodies don't have an island id.
            let islandIdA = bodies[Int(link.bodyIdA)].islandId
            let islandIdB = bodies[Int(link.bodyIdB)].islandId
            let targetIslandId = islandIdA != B3.nullIndex ? islandIdA : islandIdB

            joint.islandId = targetIslandId
            joint.islandIndex = Int32(islands[Int(targetIslandId)].joints.count)
            islands[Int(targetIslandId)].joints.append(link)
            joints[Int(link.jointId)] = joint
        }

        // Destroy the base island
        destroyIsland(baseId)
    }

    // MARK: Sleeping

    @inline(__always)
    func sleepBodySimIndex(_ encoded: Int32) -> Int32 {
        encoded >= 0 ? B3.nullIndex : encoded
    }

    /// Put an island to sleep. Creates a new sleeping solver set for it.
    func trySleepIsland(_ islandId: Int32) {
        var island = islands[Int(islandId)]
        b3Assert(island.setIndex == SetType.awakeSet.rawValue)

        // Cannot put an island to sleep while it has a pending split and more than one body.
        if island.constraintRemoveCount > 0 && island.bodies.count > 1 {
            return
        }

        let sleepSetId = solverSetIdPool.alloc()
        if Int(sleepSetId) == solverSets.count {
            solverSets.append(SolverSet())
        }

        var sleepSet = SolverSet()
        sleepSet.setIndex = sleepSetId

        let awakeSetIndex = Int(SetType.awakeSet.rawValue)
        let disabledSetIndex = Int(SetType.disabledSet.rawValue)

        b3Assert(
            0 <= island.localIndex
                && Int(island.localIndex) < solverSets[awakeSetIndex].islandSims.count)

        // move awake bodies to sleeping set
        // this shuffles around bodies in the awake set
        for i in 0..<island.bodies.count {
            let bodyId = island.bodies[i]
            var body = bodies[Int(bodyId)]
            b3Assert(body.setIndex == SetType.awakeSet.rawValue)
            b3Assert(body.islandId == islandId)
            b3Assert(body.islandIndex == Int32(i))

            // Update the body move event to indicate this body fell asleep.
            if body.bodyMoveIndex != B3.nullIndex && Int(body.bodyMoveIndex) < bodyMoveEvents.count {
                var moveEvent = bodyMoveEvents[Int(body.bodyMoveIndex)]
                moveEvent.fellAsleep = true
                bodyMoveEvents[Int(body.bodyMoveIndex)] = moveEvent
                body.bodyMoveIndex = B3.nullIndex
            }

            let awakeBodyIndex = body.localIndex
            let awakeSim = solverSets[awakeSetIndex].bodySims[Int(awakeBodyIndex)]

            // move body sim to sleep set
            let sleepBodyIndex = Int32(sleepSet.bodySims.count)
            sleepSet.bodySims.append(awakeSim)

            let movedIndex = removeSwapArray(
                &solverSets[awakeSetIndex].bodySims, Int(awakeBodyIndex))
            if movedIndex != B3.nullIndex {
                // fix local index on moved element
                let movedId = solverSets[awakeSetIndex].bodySims[Int(awakeBodyIndex)].bodyId
                var movedBody = bodies[Int(movedId)]
                b3Assert(movedBody.localIndex == movedIndex)
                movedBody.localIndex = awakeBodyIndex
                bodies[Int(movedId)] = movedBody

                if movedBody.islandId != islandId {
                    refreshBodyContactIndices(movedId)
                }
            }

            // destroy state, no need to clone
            removeSwapArray(&solverSets[awakeSetIndex].bodyStates, Int(awakeBodyIndex))

            body.setIndex = sleepSetId
            body.localIndex = sleepBodyIndex
            bodies[Int(bodyId)] = body

            // Move non-touching contacts to the disabled set.
            var contactKey = body.headContactKey
            while contactKey != B3.nullIndex {
                let contactId = contactKey >> 1
                let edgeIndex = Int(contactKey & 1)

                var contact = contacts[Int(contactId)]
                b3Assert(contact.setIndex == SetType.awakeSet.rawValue)

                contactKey = contact.edges[edgeIndex].nextKey

                if contact.colorIndex != B3.nullIndex {
                    // contact is touching and will be moved separately
                    b3Assert((contact.flags & ContactFlags.touching.rawValue) != 0)
                    continue
                }

                if edgeIndex == 0 {
                    contact.encodedBodySimA = B3.nullIndex
                } else {
                    contact.encodedBodySimB = B3.nullIndex
                }

                // the other body may still be awake
                let otherEdgeIndex = edgeIndex ^ 1
                let otherBodyId = contact.edges[otherEdgeIndex].bodyId
                if bodies[Int(otherBodyId)].setIndex == SetType.awakeSet.rawValue {
                    contacts[Int(contactId)] = contact
                    continue
                }

                let localIndex = contact.localIndex
                b3Assert(solverSets[awakeSetIndex].contactIndices[Int(localIndex)] == contactId)

                b3Assert(contact.manifoldCount == 0)
                b3Assert((contact.flags & ContactFlags.touching.rawValue) == 0)

                // Move the non-touching contact to the disabled set.
                contact.setIndex = SetType.disabledSet.rawValue
                contact.localIndex = Int32(solverSets[disabledSetIndex].contactIndices.count)
                solverSets[disabledSetIndex].contactIndices.append(contact.contactId)

                let movedLocalIndex = removeSwapArray(
                    &solverSets[awakeSetIndex].contactIndices, Int(localIndex))
                if movedLocalIndex != B3.nullIndex {
                    let movedContactIndex =
                        solverSets[awakeSetIndex].contactIndices[Int(localIndex)]
                    var movedContact = contacts[Int(movedContactIndex)]
                    b3Assert(movedContact.localIndex == movedLocalIndex)
                    movedContact.localIndex = localIndex
                    contacts[Int(movedContactIndex)] = movedContact
                }

                contacts[Int(contactId)] = contact
            }
        }

        // move touching contacts to sleeping set
        for i in 0..<island.contacts.count {
            let contactId = island.contacts[i].contactId
            var contact = contacts[Int(contactId)]
            b3Assert(contact.setIndex == SetType.awakeSet.rawValue)
            b3Assert(contact.islandId == islandId)
            b3Assert(contact.islandIndex == Int32(i))
            let colorIndex = contact.colorIndex
            b3Assert(0 <= colorIndex && colorIndex < B3.graphColorCount)

            // Remove bodies from graph coloring associated with this constraint
            if colorIndex != overflowIndex {
                // might clear a bit for a static body, but this has no effect
                constraintGraph.clearBit(colorIndex, contact.edges[0].bodyId)
                constraintGraph.clearBit(colorIndex, contact.edges[1].bodyId)
            }

            let sleepContactIndex = Int32(sleepSet.contactIndices.count)
            sleepSet.contactIndices.append(contactId)

            let localIndex = contact.localIndex
            if (contact.flags & ContactFlags.simMeshContact.rawValue) != 0 || colorIndex == overflowIndex {
                let movedLocalIndex = removeSwapArray(
                    &constraintGraph.colors[Int(colorIndex)].contacts, Int(localIndex)
                )
                if movedLocalIndex != B3.nullIndex {
                    let movedContactId = constraintGraph.colors[Int(colorIndex)].contacts[Int(localIndex)].contactId
                    var movedContact = contacts[Int(movedContactId)]
                    b3Assert(movedContact.localIndex == movedLocalIndex)
                    movedContact.localIndex = localIndex
                    contacts[Int(movedContactId)] = movedContact
                }
            } else {
                let movedLocalIndex = removeSwapArray(
                    &constraintGraph.colors[Int(colorIndex)].convexContacts, Int(localIndex)
                )
                if movedLocalIndex != B3.nullIndex {
                    let movedContactId = constraintGraph.colors[Int(colorIndex)].convexContacts[Int(localIndex)]
                    var movedContact = contacts[Int(movedContactId)]
                    b3Assert(movedContact.localIndex == movedLocalIndex)
                    movedContact.localIndex = localIndex
                    contacts[Int(movedContactId)] = movedContact
                }
            }

            contact.setIndex = sleepSetId
            contact.colorIndex = B3.nullIndex
            contact.localIndex = sleepContactIndex
            contact.encodedBodySimA = sleepBodySimIndex(contact.encodedBodySimA)
            contact.encodedBodySimB = sleepBodySimIndex(contact.encodedBodySimB)
            contacts[Int(contactId)] = contact
        }

        // move joints
        for i in 0..<island.joints.count {
            let jointId = island.joints[i].jointId
            var joint = joints[Int(jointId)]
            b3Assert(joint.setIndex == SetType.awakeSet.rawValue)
            b3Assert(joint.islandId == islandId)
            b3Assert(joint.islandIndex == Int32(i))
            let colorIndex = joint.colorIndex
            let localIndex = joint.localIndex

            b3Assert(0 <= colorIndex && colorIndex < B3.graphColorCount)

            // might clear a bit for a static body, but this has no effect
            if colorIndex != overflowIndex {
                constraintGraph.clearBit(colorIndex, joint.edges[0].bodyId)
                constraintGraph.clearBit(colorIndex, joint.edges[1].bodyId)
            }

            let sleepJointIndex = Int32(sleepSet.jointSims.count)
            sleepSet.jointSims.append(constraintGraph.colors[Int(colorIndex)].jointSims[Int(localIndex)])

            let movedIndex = removeSwapArray(
                &constraintGraph.colors[Int(colorIndex)].jointSims, Int(localIndex)
            )
            if movedIndex != B3.nullIndex {
                let movedId = constraintGraph.colors[Int(colorIndex)].jointSims[Int(localIndex)].jointId
                var movedJoint = joints[Int(movedId)]
                b3Assert(movedJoint.localIndex == movedIndex)
                movedJoint.localIndex = localIndex
                joints[Int(movedId)] = movedJoint
            }

            joint.setIndex = sleepSetId
            joint.colorIndex = B3.nullIndex
            joint.localIndex = sleepJointIndex
            joints[Int(jointId)] = joint
        }

        // move island struct
        do {
            b3Assert(island.setIndex == SetType.awakeSet.rawValue)

            let islandIndex = Int(island.localIndex)
            var islandSim = IslandSim()
            islandSim.islandId = islandId
            sleepSet.islandSims.append(islandSim)

            let movedIslandIndex = removeSwapArray(
                &solverSets[awakeSetIndex].islandSims, islandIndex)
            if movedIslandIndex != B3.nullIndex {
                let movedIslandId =
                    solverSets[awakeSetIndex].islandSims[islandIndex].islandId
                var movedIsland = islands[Int(movedIslandId)]
                b3Assert(movedIsland.localIndex == movedIslandIndex)
                movedIsland.localIndex = Int32(islandIndex)
                islands[Int(movedIslandId)] = movedIsland
            }

            island.setIndex = sleepSetId
            island.localIndex = 0
            islands[Int(islandId)] = island
        }

        if splitIslandId == islandId {
            splitIslandId = B3.nullIndex
        }

        solverSets[Int(sleepSetId)] = sleepSet
    }

    /// Collect the sleepiest island that needs splitting and put sleeping islands to sleep.
    func processSleep() {
        // Collect split island candidate for the next time step.
        b3Assert(splitIslandId == B3.nullIndex)
        var splitSleepTimer: Float = 0
        for i in 0..<taskContexts.count {
            let taskContext = taskContexts[i]
            if taskContext.splitIslandId != B3.nullIndex && taskContext.splitSleepTime >= splitSleepTimer {
                b3Assert(taskContext.splitSleepTime > 0)

                // Tie breaking for determinism. Largest island id wins.
                if taskContext.splitSleepTime == splitSleepTimer && taskContext.splitIslandId < splitIslandId {
                    continue
                }

                splitIslandId = taskContext.splitIslandId
                splitSleepTimer = taskContext.splitSleepTime
            }
        }

        // Need to process in reverse because this moves islands to sleeping solver sets.
        let awakeSet = solverSets[Int(SetType.awakeSet.rawValue)]
        let islandsCopy = awakeSet.islandSims
        let awakeIslandBitSet = taskContexts[0].awakeIslandBitSet
        for islandIndex in stride(from: islandsCopy.count - 1, through: 0, by: -1) {
            if awakeIslandBitSet.getBit(UInt32(islandIndex)) {
                // this island is still awake
                continue
            }

            trySleepIsland(islandsCopy[islandIndex].islandId)
        }

        taskContexts[0].awakeIslandBitSet = awakeIslandBitSet
    }
}
