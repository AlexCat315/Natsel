

// Island graph maintenance (island.c).

extension World {
    /// Merge two islands. Keeps the bigger island to reduce cache misses.
    func mergeIslands(_ islandIdA: Int32, _ islandIdB: Int32) -> Int32 {
        if islandIdA == islandIdB {
            return islandIdA
        }

        if islandIdA == B3.nullIndex {
            b3Assert(islandIdB != B3.nullIndex)
            return islandIdB
        }

        if islandIdB == B3.nullIndex {
            b3Assert(islandIdA != B3.nullIndex)
            return islandIdA
        }

        var bigId: Int32
        var smallId: Int32
        do {
            let islandA = islands[Int(islandIdA)]
            let islandB = islands[Int(islandIdB)]

            if islandA.bodies.count >= islandB.bodies.count {
                bigId = islandIdA
                smallId = islandIdB
            } else {
                bigId = islandIdB
                smallId = islandIdA
            }
        }

        var bigIsland = islands[Int(bigId)]
        let smallIsland = islands[Int(smallId)]
        let bigIslandId = bigId

        // Move bodies from smaller island to larger island
        for bodyId in smallIsland.bodies {
            var body = bodies[Int(bodyId)]
            b3Validate(body.islandId == smallIsland.islandId)
            body.islandId = bigIslandId
            body.islandIndex = Int32(bigIsland.bodies.count)
            bodies[Int(bodyId)] = body
            bigIsland.bodies.append(bodyId)
        }

        // Migrate contacts from smaller island to larger island
        if smallIsland.contacts.count > 0 {
            for link in smallIsland.contacts {
                var contact = contacts[Int(link.contactId)]
                contact.islandId = bigIslandId
                contact.islandIndex = Int32(bigIsland.contacts.count)
                contacts[Int(link.contactId)] = contact
                bigIsland.contacts.append(link)
            }
        }

        // Migrate joints from smaller island to larger island
        if smallIsland.joints.count > 0 {
            for link in smallIsland.joints {
                var joint = joints[Int(link.jointId)]
                joint.islandId = bigIslandId
                joint.islandIndex = Int32(bigIsland.joints.count)
                joints[Int(link.jointId)] = joint
                bigIsland.joints.append(link)
            }
        }

        // Track removed constraints
        bigIsland.constraintRemoveCount += smallIsland.constraintRemoveCount

        islands[Int(bigId)] = bigIsland

        destroyIsland(smallIsland.islandId)

        return bigIslandId
    }

    private func addContactToIsland(_ islandId: Int32, _ contactId: Int32) {
        var contact = contacts[Int(contactId)]
        b3Assert(contact.islandId == B3.nullIndex)
        b3Assert(contact.islandIndex == B3.nullIndex)

        var island = islands[Int(islandId)]

        contact.islandId = islandId
        contact.islandIndex = Int32(island.contacts.count)
        contacts[Int(contactId)] = contact

        var link = ContactLink()
        link.contactId = contact.contactId
        link.bodyIdA = contact.edges[0].bodyId
        link.bodyIdB = contact.edges[1].bodyId
        island.contacts.append(link)
        islands[Int(islandId)] = island
    }

    /// Link a touching contact into the island graph.
    func linkContact(_ contactId: Int32) {
        var contact = contacts[Int(contactId)]
        b3Assert((contact.flags & ContactFlags.simTouching.rawValue) != 0)

        let bodyIdA = contact.edges[0].bodyId
        let bodyIdB = contact.edges[1].bodyId

        var bodyA = bodies[Int(bodyIdA)]
        var bodyB = bodies[Int(bodyIdB)]

        b3Assert(bodyA.setIndex != SetType.disabledSet.rawValue && bodyB.setIndex != SetType.disabledSet.rawValue)
        b3Assert(bodyA.setIndex != SetType.staticSet.rawValue || bodyB.setIndex != SetType.staticSet.rawValue)

        // Wake bodyB if bodyA is awake and bodyB is sleeping
        if bodyA.setIndex == SetType.awakeSet.rawValue && bodyB.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeSolverSet(bodyB.setIndex)
        }

        // Wake bodyA if bodyB is awake and bodyA is sleeping
        if bodyB.setIndex == SetType.awakeSet.rawValue && bodyA.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeSolverSet(bodyA.setIndex)
        }

        bodyA = bodies[Int(bodyIdA)]
        bodyB = bodies[Int(bodyIdB)]

        let islandIdA = bodyA.islandId
        let islandIdB = bodyB.islandId

        // Static bodies have null island indices.
        b3Assert(bodyA.setIndex != SetType.staticSet.rawValue || islandIdA == B3.nullIndex)
        b3Assert(bodyB.setIndex != SetType.staticSet.rawValue || islandIdB == B3.nullIndex)
        b3Assert(islandIdA != B3.nullIndex || islandIdB != B3.nullIndex)

        // Merge islands. This will destroy one of the islands.
        let finalIslandId = mergeIslands(islandIdA, islandIdB)

        // Add contact to the island that survived
        addContactToIsland(finalIslandId, contactId)

        contact = contacts[Int(contactId)]
        b3Validate(contact.islandId == finalIslandId)
    }

    /// Unlink a contact that no longer has contact points or is being destroyed.
    func unlinkContact(_ contactId: Int32) {
        var contact = contacts[Int(contactId)]
        b3Assert(contact.islandId != B3.nullIndex)

        // remove from island
        let islandId = contact.islandId
        var island = islands[Int(islandId)]

        let removeIndex = Int(contact.islandIndex)
        b3Assert(0 <= removeIndex && removeIndex < island.contacts.count)
        b3Assert(island.contacts[removeIndex].contactId == contact.contactId)

        let movedIndex = removeSwapArray(&island.contacts, removeIndex)
        if movedIndex != B3.nullIndex {
            // Fix islandIndex on the contact that was swapped into removeIndex
            let movedLink = island.contacts[removeIndex]
            var movedContact = contacts[Int(movedLink.contactId)]
            b3Assert(movedContact.islandIndex == movedIndex)
            movedContact.islandIndex = Int32(removeIndex)
            contacts[Int(movedLink.contactId)] = movedContact
        }

        contact.islandId = B3.nullIndex
        contact.islandIndex = B3.nullIndex
        contacts[Int(contactId)] = contact

        island.constraintRemoveCount += 1
        islands[Int(islandId)] = island
    }
}

extension World {
    private func addJointToIsland(_ islandId: Int32, _ jointId: Int32) {
        var joint = joints[Int(jointId)]
        b3Assert(joint.islandId == B3.nullIndex)
        b3Assert(joint.islandIndex == B3.nullIndex)

        var island = islands[Int(islandId)]

        joint.islandId = islandId
        joint.islandIndex = Int32(island.joints.count)
        joints[Int(jointId)] = joint

        var link = JointLink()
        link.jointId = joint.jointId
        link.bodyIdA = joint.edges[0].bodyId
        link.bodyIdB = joint.edges[1].bodyId
        island.joints.append(link)
        islands[Int(islandId)] = island
    }

    /// Link a joint into the island graph.
    func linkJoint(_ jointId: Int32) {
        let joint = joints[Int(jointId)]
        var bodyA = bodies[Int(joint.edges[0].bodyId)]
        let bodyB = bodies[Int(joint.edges[1].bodyId)]

        b3Assert(bodyA.type == .dynamicBody || bodyB.type == .dynamicBody)

        if bodyA.setIndex == SetType.awakeSet.rawValue && bodyB.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeSolverSet(bodyB.setIndex)
        } else if bodyB.setIndex == SetType.awakeSet.rawValue
            && bodyA.setIndex >= SetType.firstSleepingSet.rawValue
        {
            wakeSolverSet(bodyA.setIndex)
        }

        bodyA = bodies[Int(joint.edges[0].bodyId)]
        let bodyB2 = bodies[Int(joint.edges[1].bodyId)]

        let islandIdA = bodyA.islandId
        let islandIdB = bodyB2.islandId

        b3Assert(islandIdA != B3.nullIndex || islandIdB != B3.nullIndex)

        // Merge islands. This will destroy one of the islands.
        let finalIslandId = mergeIslands(islandIdA, islandIdB)

        // Add joint to the island that survived
        addJointToIsland(finalIslandId, jointId)
    }

    /// Unlink a joint from its island.
    func unlinkJoint(_ jointId: Int32) {
        var joint = joints[Int(jointId)]
        if joint.islandId == B3.nullIndex {
            return
        }

        // remove from island
        let islandId = joint.islandId
        var island = islands[Int(islandId)]

        let removeIndex = Int(joint.islandIndex)
        b3Assert(0 <= removeIndex && removeIndex < island.joints.count)
        b3Assert(island.joints[removeIndex].jointId == joint.jointId)

        let movedIndex = removeSwapArray(&island.joints, removeIndex)
        if movedIndex != B3.nullIndex {
            // Fix islandIndex on the joint that was swapped into removeIndex
            let movedLink = island.joints[removeIndex]
            var movedJoint = joints[Int(movedLink.jointId)]
            b3Assert(movedJoint.islandIndex == movedIndex)
            movedJoint.islandIndex = Int32(removeIndex)
            joints[Int(movedLink.jointId)] = movedJoint
        }

        joint.islandId = B3.nullIndex
        joint.islandIndex = B3.nullIndex
        joints[Int(jointId)] = joint

        island.constraintRemoveCount += 1
        islands[Int(islandId)] = island
    }
}
