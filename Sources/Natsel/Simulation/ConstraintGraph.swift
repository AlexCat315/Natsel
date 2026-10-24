

// Constraint graph coloring (constraint_graph.c). Constraints involving two dynamic bodies get
// low color indices; constraints involving a static body build from the end so they have higher
// solver priority. Constraints that cannot be colored go to the overflow color.

/// This holds constraints that cannot fit the graph color limit.
let overflowIndex: Int32 = B3.graphColorCount - 1

/// Dynamic constraints cannot encroach on colors reserved for static constraints.
let dynamicColorCount: Int32 = B3.graphColorCount - 4

/// A contact spec for mesh and overflow contacts.
struct ContactSpec: BitwiseCopyable {
    var contactId: Int32 = B3.nullIndex
    var manifoldStart: Int32 = 0
    var manifoldCount: UInt16 = 0

    init() {}
}

/// One graph color: a body bitset and the constraints in this color.
struct GraphColor {
    /// This bitset is indexed by bodyId so it is over-sized to encompass static bodies.
    var bodySet = BitSet()

    // Cache friendly arrays
    var jointSims: [JointSim] = []
    var convexContacts: [Int32] = []
    var contacts: [ContactSpec] = []

    init() {}
}

/// The constraint graph.
final class ConstraintGraph {
    /// Including overflow at the end.
    var colors: [GraphColor] = []

    init(bodyCapacity: Int32) {
        colors = Array(repeating: GraphColor(), count: Int(B3.graphColorCount))

        let capacity = maxInt(bodyCapacity, 8)

        // Initialize graph color bit set. No bitset for the overflow color.
        for i in 0..<Int(overflowIndex) {
            colors[i].bodySet = BitSet(bitCapacity: UInt32(capacity))
            colors[i].bodySet.setBitCountAndClear(UInt32(capacity))
        }
    }

    func getBit(_ colorIndex: Int32, _ bodyId: Int32) -> Bool {
        colors[Int(colorIndex)].bodySet.getBit(UInt32(bodyId))
    }

    func setBitGrow(_ colorIndex: Int32, _ bodyId: Int32) {
        colors[Int(colorIndex)].bodySet.setBitGrow(UInt32(bodyId))
    }

    func clearBit(_ colorIndex: Int32, _ bodyId: Int32) {
        colors[Int(colorIndex)].bodySet.clearBit(UInt32(bodyId))
    }
}

extension World {
    /// Contacts are always created as non-touching. They get cloned into the constraint graph once
    /// they are found to be touching.
    func addContactToGraph(_ contactId: Int32) {
        var contact = contacts[Int(contactId)]
        b3Assert(contact.manifoldCount > 0)

        var colorIndex = overflowIndex

        let bodyIdA = contact.edges[0].bodyId
        let bodyIdB = contact.edges[1].bodyId
        let bodyA = bodies[Int(bodyIdA)]
        let bodyB = bodies[Int(bodyIdB)]

        let typeA = bodyA.type
        let typeB = bodyB.type
        b3Assert(typeA == .dynamicBody || typeB == .dynamicBody)

        if typeA == .dynamicBody && typeB == .dynamicBody {
            // Dynamic constraint colors cannot encroach on colors reserved for static constraints
            for i in 0..<Int(dynamicColorCount) {
                if constraintGraph.getBit(Int32(i), bodyIdA) || constraintGraph.getBit(Int32(i), bodyIdB) {
                    continue
                }

                constraintGraph.setBitGrow(Int32(i), bodyIdA)
                constraintGraph.setBitGrow(Int32(i), bodyIdB)
                colorIndex = Int32(i)
                break
            }
        } else if typeA == .dynamicBody {
            // Static constraint colors build from the end to get higher priority
            var i = overflowIndex - 1
            while i >= 1 {
                if constraintGraph.getBit(i, bodyIdA) == false {
                    constraintGraph.setBitGrow(i, bodyIdA)
                    colorIndex = i
                    break
                }
                i -= 1
            }
        } else if typeB == .dynamicBody {
            var i = overflowIndex - 1
            while i >= 1 {
                if constraintGraph.getBit(i, bodyIdB) == false {
                    constraintGraph.setBitGrow(i, bodyIdB)
                    colorIndex = i
                    break
                }
                i -= 1
            }
        }

        let isScalar =
            (contact.flags & ContactFlags.simMeshContact.rawValue) != 0 || colorIndex == overflowIndex

        contact.colorIndex = colorIndex
        contact.localIndex = isScalar
            ? Int32(constraintGraph.colors[Int(colorIndex)].contacts.count)
            : Int32(constraintGraph.colors[Int(colorIndex)].convexContacts.count)
        contact.encodedBodySimA = encodeBodySimIndex(bodyA)
        contact.encodedBodySimB = encodeBodySimIndex(bodyB)

        if isScalar {
            b3Assert(contact.manifoldCount < Int32(UInt16.max))
            var spec = ContactSpec()
            spec.contactId = contact.contactId
            spec.manifoldStart = 0
            spec.manifoldCount = UInt16(truncatingIfNeeded: contact.manifoldCount)
            constraintGraph.colors[Int(colorIndex)].contacts.append(spec)
        } else {
            constraintGraph.colors[Int(colorIndex)].convexContacts.append(contact.contactId)
        }

        contacts[Int(contactId)] = contact
    }

    /// Remove a contact from its graph color.
    func removeContactFromGraph(
        _ bodyIdA: Int32,
        _ bodyIdB: Int32,
        _ colorIndex: Int32,
        _ localIndex: Int32,
        _ meshContact: Bool
    ) {
        b3Assert(0 <= colorIndex && colorIndex < B3.graphColorCount)

        if colorIndex != overflowIndex {
            // This might clear a bit for a static body, but this has no effect
            constraintGraph.clearBit(colorIndex, bodyIdA)
            constraintGraph.clearBit(colorIndex, bodyIdB)
        }

        if meshContact || colorIndex == overflowIndex {
            let movedIndex = removeSwapArray(&constraintGraph.colors[Int(colorIndex)].contacts, Int(localIndex))
            if movedIndex != B3.nullIndex {
                // Fix index on swapped contact
                let movedContactId = constraintGraph.colors[Int(colorIndex)].contacts[Int(localIndex)].contactId
                var movedContact = contacts[Int(movedContactId)]
                b3Assert(movedContact.setIndex == SetType.awakeSet.rawValue)
                b3Assert(movedContact.colorIndex == colorIndex)
                b3Assert(movedContact.localIndex == movedIndex)
                movedContact.localIndex = localIndex
                contacts[Int(movedContactId)] = movedContact
            }
        } else {
            let movedIndex = removeSwapArray(
                &constraintGraph.colors[Int(colorIndex)].convexContacts, Int(localIndex)
            )
            if movedIndex != B3.nullIndex {
                // Fix index on swapped contact
                let movedContactId = constraintGraph.colors[Int(colorIndex)].convexContacts[Int(localIndex)]
                var movedContact = contacts[Int(movedContactId)]
                b3Assert(movedContact.setIndex == SetType.awakeSet.rawValue)
                b3Assert(movedContact.colorIndex == colorIndex)
                b3Assert(movedContact.localIndex == movedIndex)
                b3Assert((movedContact.flags & ContactFlags.simMeshContact.rawValue) == 0)
                movedContact.localIndex = localIndex
                contacts[Int(movedContactId)] = movedContact
            }
        }
    }

    // MARK: Joints

    /// Assign a graph color for a joint.
    func assignJointColor(_ bodyIdA: Int32, _ bodyIdB: Int32, _ typeA: BodyType, _ typeB: BodyType) -> Int32 {
        b3Assert(typeA == .dynamicBody || typeB == .dynamicBody)

        if typeA == .dynamicBody && typeB == .dynamicBody {
            for i in 0..<Int(dynamicColorCount) {
                if constraintGraph.getBit(Int32(i), bodyIdA) || constraintGraph.getBit(Int32(i), bodyIdB) {
                    continue
                }

                constraintGraph.setBitGrow(Int32(i), bodyIdA)
                constraintGraph.setBitGrow(Int32(i), bodyIdB)
                return Int32(i)
            }
        } else if typeA == .dynamicBody {
            var i = overflowIndex - 1
            while i >= 1 {
                if constraintGraph.getBit(i, bodyIdA) == false {
                    constraintGraph.setBitGrow(i, bodyIdA)
                    return i
                }
                i -= 1
            }
        } else if typeB == .dynamicBody {
            var i = overflowIndex - 1
            while i >= 1 {
                if constraintGraph.getBit(i, bodyIdB) == false {
                    constraintGraph.setBitGrow(i, bodyIdB)
                    return i
                }
                i -= 1
            }
        }

        return overflowIndex
    }

    /// Create a joint sim in the graph and return its index.
    @discardableResult
    func createJointInGraph(_ jointId: Int32) -> Int32 {
        var joint = joints[Int(jointId)]

        let bodyIdA = joint.edges[0].bodyId
        let bodyIdB = joint.edges[1].bodyId
        let bodyA = bodies[Int(bodyIdA)]
        let bodyB = bodies[Int(bodyIdB)]

        let colorIndex = assignJointColor(bodyIdA, bodyIdB, bodyA.type, bodyB.type)

        var jointSim = JointSim()
        jointSim.jointId = joint.jointId
        constraintGraph.colors[Int(colorIndex)].jointSims.append(jointSim)

        joint.colorIndex = colorIndex
        joint.localIndex = Int32(constraintGraph.colors[Int(colorIndex)].jointSims.count - 1)
        joints[Int(jointId)] = joint

        return joint.localIndex
    }

    /// Add a joint to the graph at the given sim.
    func addJointToGraph(_ jointSim: JointSim, _ jointId: Int32) {
        let index = createJointInGraph(jointId)
        let joint = joints[Int(jointId)]
        constraintGraph.colors[Int(joint.colorIndex)].jointSims[Int(index)] = jointSim
    }

    /// Remove a joint from its graph color.
    func removeJointFromGraph(
        _ bodyIdA: Int32,
        _ bodyIdB: Int32,
        _ colorIndex: Int32,
        _ localIndex: Int32
    ) {
        b3Assert(0 <= colorIndex && colorIndex < B3.graphColorCount)

        if colorIndex != overflowIndex {
            // May clear static bodies, no effect
            constraintGraph.clearBit(colorIndex, bodyIdA)
            constraintGraph.clearBit(colorIndex, bodyIdB)
        }

        let movedIndex = removeSwapArray(
            &constraintGraph.colors[Int(colorIndex)].jointSims, Int(localIndex)
        )
        if movedIndex != B3.nullIndex {
            // Fix moved joint
            let movedId = constraintGraph.colors[Int(colorIndex)].jointSims[Int(localIndex)].jointId
            var movedJoint = joints[Int(movedId)]
            b3Assert(movedJoint.setIndex == SetType.awakeSet.rawValue)
            b3Assert(movedJoint.colorIndex == colorIndex)
            b3Assert(movedJoint.localIndex == movedIndex)
            movedJoint.localIndex = localIndex
            joints[Int(movedId)] = movedJoint
        }
    }
}

extension World {
    /// Debug helper: verify every graph color's contact array matches the contacts' cached indices.
    func validateGraphIndices() -> String? {
        for colorIndex in 0..<Int(B3.graphColorCount) {
            let color = constraintGraph.colors[colorIndex]

            for (i, contactId) in color.convexContacts.enumerated() {
                let contact = contacts[Int(contactId)]
                if contact.contactId != contactId {
                    return "color \(colorIndex) convex[\(i)] id=\(contactId) slot=freed"
                }
                if contact.colorIndex != Int32(colorIndex) || contact.localIndex != Int32(i) {
                    return
                        "color \(colorIndex) convex[\(i)] id=\(contactId) has color=\(contact.colorIndex) local=\(contact.localIndex)"
                }
            }

            for (i, spec) in color.contacts.enumerated() {
                let contactId = spec.contactId
                let contact = contacts[Int(contactId)]
                if contact.contactId != contactId {
                    return "color \(colorIndex) scalar[\(i)] id=\(contactId) slot=freed"
                }
                if contact.colorIndex != Int32(colorIndex) || contact.localIndex != Int32(i) {
                    return
                        "color \(colorIndex) scalar[\(i)] id=\(contactId) has color=\(contact.colorIndex) local=\(contact.localIndex)"
                }
            }
        }
        return nil
    }
}

extension World {
    /// Debug helper: verify every contact agrees with the set and island that claim to own it.
    /// Returns the first violation or nil.
    func validateContactOwnership() -> String? {
        for i in 0..<contacts.count {
            let c = contacts[i]

            if c.contactId == B3.nullIndex {
                continue
            }

            if c.contactId != Int32(i) {
                return "contact[\(i)] has contactId \(c.contactId)"
            }

            // A live contact must sit in a real set.
            if c.setIndex < 0 || Int(c.setIndex) >= solverSets.count {
                return "contact \(i) has setIndex \(c.setIndex)"
            }

            // Touching awake contacts belong to the graph, not the set's non-touching list.
            if c.setIndex == SetType.awakeSet.rawValue && c.colorIndex != B3.nullIndex {
                if c.localIndex < 0 {
                    return "contact \(i) graph color \(c.colorIndex) has localIndex \(c.localIndex)"
                }
            } else {
                // Non-graph contacts are listed in their set's contactIndices at localIndex.
                let set = solverSets[Int(c.setIndex)]
                if c.localIndex < 0 || Int(c.localIndex) >= set.contactIndices.count {
                    return
                        "contact \(i) set \(c.setIndex) localIndex \(c.localIndex) beyond \(set.contactIndices.count)"
                }
                if set.contactIndices[Int(c.localIndex)] != Int32(i) {
                    return
                        "contact \(i) set \(c.setIndex) localIndex \(c.localIndex) holds \(set.contactIndices[Int(c.localIndex)])"
                }
            }

            // If it claims an island, the island must list it at islandIndex.
            if c.islandId != B3.nullIndex {
                if Int(c.islandId) >= islands.count {
                    return "contact \(i) islandId \(c.islandId) out of range"
                }
                let island = islands[Int(c.islandId)]
                if c.islandIndex < 0 || Int(c.islandIndex) >= island.contacts.count {
                    return
                        "contact \(i) island \(c.islandId) islandIndex \(c.islandIndex) beyond \(island.contacts.count)"
                }
                if island.contacts[Int(c.islandIndex)].contactId != Int32(i) {
                    return
                        "contact \(i) island \(c.islandId) islandIndex \(c.islandIndex) holds \(island.contacts[Int(c.islandIndex)].contactId)"
                }
            }
        }
        return nil
    }

    /// Debug helper: verify every solver set's contact list points at contacts that agree.
    func validateSetContacts() -> String? {
        for setIndex in 0..<solverSets.count {
            let set = solverSets[Int(setIndex)]
            for (k, contactId) in set.contactIndices.enumerated() {
                if contactId < 0 || Int(contactId) >= contacts.count {
                    return "set \(setIndex) slot \(k) holds \(contactId) out of range"
                }
                let c = contacts[Int(contactId)]
                if c.contactId == B3.nullIndex {
                    return "set \(setIndex) slot \(k) holds freed contact \(contactId)"
                }
                if c.setIndex != Int32(setIndex) || c.localIndex != Int32(k) {
                    return
                        "set \(setIndex) slot \(k) holds contact \(contactId) with set \(c.setIndex) local \(c.localIndex)"
                }
            }
        }
        return nil
    }
}

extension World {
    /// Debug helper: verify island membership matches the bodies and solver sets.
    func validateIslands() -> String? {
        for i in 0..<islands.count {
            let island = islands[i]
            if island.islandId == B3.nullIndex {
                continue
            }
            if island.setIndex < 0 || Int(island.setIndex) >= solverSets.count {
                return "island \(i) setIndex \(island.setIndex)"
            }
            let set = solverSets[Int(island.setIndex)]
            if island.localIndex < 0 || Int(island.localIndex) >= set.islandSims.count {
                return
                    "island \(i) set \(island.setIndex) localIndex \(island.localIndex) beyond \(set.islandSims.count)"
            }
            if set.islandSims[Int(island.localIndex)].islandId != Int32(i) {
                return
                    "island \(i) set \(island.setIndex) localIndex \(island.localIndex) holds \(set.islandSims[Int(island.localIndex)].islandId)"
            }
            for (k, bodyId) in island.bodies.enumerated() {
                let body = bodies[Int(bodyId)]
                if body.islandId != Int32(i) || body.islandIndex != Int32(k) {
                    return
                        "island \(i) body slot \(k)=\(bodyId) has island \(body.islandId) index \(body.islandIndex)"
                }
                if body.setIndex != island.setIndex {
                    return "island \(i) body \(bodyId) set \(body.setIndex) != \(island.setIndex)"
                }
            }
        }
        return nil
    }
}
