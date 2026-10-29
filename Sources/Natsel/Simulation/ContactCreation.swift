// Broad-phase pair generation and structural contact creation
//.

/// Stack entry for cross-tree traversal.
struct TraversalPair {
    var a: Int32 = 0
    var b: Int32 = 0
}

// MARK: - Contact registers

struct ContactRegister {
    var supported = false
    var primary = false
}

private let shapeTypeCount = Int(ShapeType.count)

private let contactRegisters: [[ContactRegister]] = {
    var registers = [[ContactRegister]](
        repeating: [ContactRegister](repeating: ContactRegister(), count: shapeTypeCount),
        count: shapeTypeCount
    )

    func addType(_ type1: ShapeType, _ type2: ShapeType) {
        registers[Int(type1.rawValue)][Int(type2.rawValue)].supported = true
        registers[Int(type1.rawValue)][Int(type2.rawValue)].primary = true

        if type1 != type2 {
            registers[Int(type2.rawValue)][Int(type1.rawValue)].supported = true
            registers[Int(type2.rawValue)][Int(type1.rawValue)].primary = false
        }
    }

    addType(.sphereShape, .sphereShape)
    addType(.capsuleShape, .sphereShape)
    addType(.capsuleShape, .capsuleShape)
    addType(.compoundShape, .sphereShape)
    addType(.compoundShape, .capsuleShape)
    addType(.compoundShape, .hullShape)
    addType(.hullShape, .sphereShape)
    addType(.hullShape, .capsuleShape)
    addType(.hullShape, .hullShape)
    addType(.meshShape, .sphereShape)
    addType(.meshShape, .capsuleShape)
    addType(.meshShape, .hullShape)
    addType(.heightShape, .sphereShape)
    addType(.heightShape, .capsuleShape)
    addType(.heightShape, .hullShape)

    return registers
}()

// MARK: - Contact bookkeeping

extension World {
    /// Do a body pair's joints allow collision?
    func shouldBodiesCollide(_ bodyA: borrowing Body, _ bodyB: borrowing Body) -> Bool {
        let typeA = bodyA.type
        let typeB = bodyB.type
        if typeA != .dynamicBody && typeB != .dynamicBody {
            return false
        }

        let jointKey: Int32
        let otherBodyId: Int32
        if bodyA.jointCount < bodyB.jointCount {
            jointKey = bodyA.headJointKey
            otherBodyId = bodyB.id
        } else {
            jointKey = bodyB.headJointKey
            otherBodyId = bodyA.id
        }

        var key = jointKey
        while key != B3.nullIndex {
            let jointId = key >> 1
            let edgeIndex = Int(key & 1)
            let otherEdgeIndex = edgeIndex ^ 1

            let joint = joints[Int(jointId)]
            if joint.collideConnected == false && joint.edges[otherEdgeIndex].bodyId == otherBodyId {
                return false
            }

            key = joint.edges[edgeIndex].nextKey
        }

        return true
    }

    /// The shape pair gauntlet.
    func shouldCreatePair(_ shapeIdA: Int32, _ shapeIdB: Int32) -> Bool {
        shapes.withUnsafeBufferPointer { shapeBuffer in
            shouldCreatePair(shapeIdA, shapeIdB, shapeBuffer.baseAddress! + Int(shapeIdA), shapeBuffer.baseAddress! + Int(shapeIdB))
        }
    }

    /// The gauntlet on borrowed shapes; copying a `Shape` retains its geometry and materials.
    func shouldCreatePair(
        _ shapeIdA: Int32, _ shapeIdB: Int32, _ shapeA: UnsafePointer<Shape>, _ shapeB: UnsafePointer<Shape>
    ) -> Bool {

        let bodyIdA = shapeA.pointee.bodyId
        let bodyIdB = shapeB.pointee.bodyId

        // Are the shapes on the same body?
        if bodyIdA == bodyIdB {
            return false
        }

        // Sensors are handled elsewhere
        if shapeA.pointee.sensorIndex != B3.nullIndex || shapeB.pointee.sensorIndex != B3.nullIndex {
            return false
        }

        if shouldShapesCollide(shapeA.pointee.filter, shapeB.pointee.filter) == false {
            return false
        }

        // Does a joint override collision?
        let bodyA = bodies[Int(bodyIdA)]
        let bodyB = bodies[Int(bodyIdB)]
        if shouldBodiesCollide(bodyA, bodyB) == false {
            return false
        }

        // Custom user filter
        if (shapeA.pointee.flags & ShapeFlags.enableCustomFiltering.rawValue) != 0
            || (shapeB.pointee.flags & ShapeFlags.enableCustomFiltering.rawValue) != 0
        {
            if let customFilter {
                let idA = ShapeID(
                    index1: shapeIdA + 1, world0: worldId, generation: shapeA.pointee.generation
                )
                let idB = ShapeID(
                    index1: shapeIdB + 1, world0: worldId, generation: shapeB.pointee.generation
                )
                if customFilter(idA, idB) == false {
                    return false
                }
            }
        }

        return true
    }

    /// Create the persistent contact for a shape pair. Returns the new contact id.
    @discardableResult
    func createContact(_ shapeIdA: Int32, _ shapeIdB: Int32, _ childIndex: Int32) -> Int32? {
        let typeA = shapes[Int(shapeIdA)].type
        let typeB = shapes[Int(shapeIdB)].type

        let register = contactRegisters[Int(typeA.rawValue)][Int(typeB.rawValue)]
        if register.supported == false {
            // For example, no mesh vs mesh collision
            return nil
        }

        if register.primary == false {
            // flip order
            return createContact(shapeIdB, shapeIdA, childIndex)
        }

        return shapes.withUnsafeBufferPointer { shapeBuffer in
            createContact(shapeBuffer.baseAddress! + Int(shapeIdA), shapeBuffer.baseAddress! + Int(shapeIdB), childIndex)
        }
    }

    /// Contact creation on borrowed shapes; copying a `Shape` retains its geometry and materials.
    private func createContact(
        _ shapeA: UnsafePointer<Shape>, _ shapeB: UnsafePointer<Shape>, _ childIndex: Int32
    ) -> Int32 {
        let typeA = shapeA.pointee.type
        let typeB = shapeB.pointee.type
        let bodyIdA = Int(shapeA.pointee.bodyId)
        let bodyIdB = Int(shapeB.pointee.bodyId)

        b3Assert(bodies[bodyIdA].setIndex != SetType.disabledSet.rawValue && bodies[bodyIdB].setIndex != SetType.disabledSet.rawValue)
        b3Assert(bodies[bodyIdA].setIndex != SetType.staticSet.rawValue || bodies[bodyIdB].setIndex != SetType.staticSet.rawValue)

        let setIndex: Int32
        if bodies[bodyIdA].setIndex == SetType.awakeSet.rawValue || bodies[bodyIdB].setIndex == SetType.awakeSet.rawValue {
            setIndex = SetType.awakeSet.rawValue
        } else {
            // sleeping and non-touching contacts live in the disabled set
            setIndex = SetType.disabledSet.rawValue
        }

        let setId = Int(setIndex)

        // Create contact key and contact
        let contactId = contactIdPool.alloc()
        if Int(contactId) == contacts.count {
            contacts.append(Contact())
        }

        var contact = Contact()
        contact.payload = contactPayloads.initialize(Int(contactId))
        contact.contactId = contactId
        contact.generation = contact.generation &+ 1
        contact.setIndex = setIndex
        contact.colorIndex = B3.nullIndex
        contact.localIndex = Int32(solverSets[setId].contactIndices.count)
        contact.islandId = B3.nullIndex
        contact.islandIndex = B3.nullIndex
        contact.shapeIdA = shapeA.pointee.id
        contact.shapeIdB = shapeB.pointee.id
        contact.childIndex = childIndex
        contact.encodedBodySimA = encodeBodySimIndex(bodies[bodyIdA])
        contact.encodedBodySimB = encodeBodySimIndex(bodies[bodyIdB])

        if shapeA.pointee.type == .meshShape || shapeA.pointee.type == .heightShape {
            contact.flags |= ContactFlags.simMeshContact.rawValue
        }

        if case .compound(let compound) = shapeA.pointee.geometry,
            case .mesh = compound.child(at: Int(childIndex)).geometry
        {
            contact.flags |= ContactFlags.simMeshContact.rawValue
        }

        if bodies[bodyIdA].type == .staticBody || bodies[bodyIdB].type == .staticBody {
            contact.flags |= ContactFlags.staticFlag.rawValue
        }

        b3Assert(shapeA.pointee.sensorIndex == B3.nullIndex && shapeB.pointee.sensorIndex == B3.nullIndex)

        if (shapeA.pointee.flags & ShapeFlags.enableContactEvents.rawValue) != 0
            || (shapeB.pointee.flags & ShapeFlags.enableContactEvents.rawValue) != 0
        {
            contact.flags |= ContactFlags.enableContactEvents.rawValue
        }

        if (shapeA.pointee.flags & ShapeFlags.enableSpeculative.rawValue) != 0
            && (shapeB.pointee.flags & ShapeFlags.enableSpeculative.rawValue) != 0
        {
            contact.flags |= ContactFlags.enableSpeculativePoints.rawValue
        }

        // Disabling speculative collision also disables recycling.
        if (bodies[bodyIdA].flags & BodyFlags.enableContactRecycling.rawValue) != 0
            && (bodies[bodyIdB].flags & BodyFlags.enableContactRecycling.rawValue) != 0
            && (contact.flags & ContactFlags.enableSpeculativePoints.rawValue) != 0
        {
            contact.flags |= ContactFlags.recycle.rawValue
        }

        // Connect to body A
        contact.edges[0].bodyId = shapeA.pointee.bodyId
        contact.edges[0].prevKey = B3.nullIndex
        contact.edges[0].nextKey = bodies[bodyIdA].headContactKey

        let keyA = (contactId << 1) | 0
        let headContactKeyA = bodies[bodyIdA].headContactKey
        if headContactKeyA != B3.nullIndex {
            contacts[Int(headContactKeyA >> 1)].edges[Int(headContactKeyA & 1)].prevKey = keyA
        }
        bodies[bodyIdA].headContactKey = keyA
        bodies[bodyIdA].contactCount += 1

        // Connect to body B
        contact.edges[1].bodyId = shapeB.pointee.bodyId
        contact.edges[1].prevKey = B3.nullIndex
        contact.edges[1].nextKey = bodies[bodyIdB].headContactKey

        let keyB = (contactId << 1) | 1
        let headContactKeyB = bodies[bodyIdB].headContactKey
        if headContactKeyB != B3.nullIndex {
            contacts[Int(headContactKeyB >> 1)].edges[Int(headContactKeyB & 1)].prevKey = keyB
        }
        bodies[bodyIdB].headContactKey = keyB
        bodies[bodyIdB].contactCount += 1

        // Add to pair set for fast lookup
        let pairKey = shapePairKey(shapeA.pointee.id, shapeB.pointee.id, childIndex)
        _ = broadPhase.pairSet.addKey(pairKey)

        // Contacts are created as non-touching.
        solverSets[setId].contactIndices.append(contactId)

        var radiusA: Float = 0
        if typeA == .sphereShape {
            radiusA = shapeA.pointee.sphere.radius
        } else if typeA == .capsuleShape {
            radiusA = shapeA.pointee.capsule.radius
        }

        var radiusB: Float = 0
        if typeB == .sphereShape {
            radiusB = shapeB.pointee.sphere.radius
        } else if typeB == .capsuleShape {
            radiusB = shapeB.pointee.capsule.radius
        }

        let maxRadius = maxFloat(radiusA, radiusB)

        // Assuming the rolling resistance doesn't change
        contact.rollingResistance =
            maxFloat(shapeA.pointee.material(0).rollingResistance, shapeB.pointee.material(0).rollingResistance) * maxRadius

        if (shapeA.pointee.flags & ShapeFlags.enablePreSolveEvents.rawValue) != 0
            || (shapeB.pointee.flags & ShapeFlags.enablePreSolveEvents.rawValue) != 0
        {
            contact.flags |= ContactFlags.simEnablePreSolveEvents.rawValue
        }

        contacts[Int(contactId)] = contact

        return contactId
    }

    /// Destroy a contact and unlink it from the bodies, set, and graph.
    func destroyContact(_ contactId: Int32, _ wakeBodies: Bool) {
        var contact = contacts[Int(contactId)]

        // A stale entry can outlive its contact, for example a sleep-set index that the wake
        // path already migrated. Ignore it rather than tearing the slot down twice.
        if contact.contactId == B3.nullIndex {
            return
        }

        // Remove pair from set
        let pairKey = shapePairKey(contact.shapeIdA, contact.shapeIdB, contact.childIndex)
        _ = broadPhase.pairSet.removeKey(pairKey)

        contact.manifolds.removeAll()
        contact.manifoldCount = 0

        let edgeA = contact.edges[0]
        let edgeB = contact.edges[1]

        let bodyIdA = edgeA.bodyId
        let bodyIdB = edgeB.bodyId
        var bodyA = bodies[Int(bodyIdA)]
        var bodyB = bodies[Int(bodyIdB)]

        let flags = contact.flags
        let touching = (flags & ContactFlags.touching.rawValue) != 0

        // End touch events are queued for publication.

        // Remove from body A
        if edgeA.prevKey != B3.nullIndex {
            var prevContact = contacts[Int(edgeA.prevKey >> 1)]
            prevContact.edges[Int(edgeA.prevKey & 1)].nextKey = edgeA.nextKey
            contacts[Int(edgeA.prevKey >> 1)] = prevContact
        }

        if edgeA.nextKey != B3.nullIndex {
            var nextContact = contacts[Int(edgeA.nextKey >> 1)]
            nextContact.edges[Int(edgeA.nextKey & 1)].prevKey = edgeA.prevKey
            contacts[Int(edgeA.nextKey >> 1)] = nextContact
        }

        let edgeKeyA = (contactId << 1) | 0
        if bodyA.headContactKey == edgeKeyA {
            bodyA.headContactKey = edgeA.nextKey
        }
        bodyA.contactCount -= 1

        // Remove from body B
        if edgeB.prevKey != B3.nullIndex {
            var prevContact = contacts[Int(edgeB.prevKey >> 1)]
            prevContact.edges[Int(edgeB.prevKey & 1)].nextKey = edgeB.nextKey
            contacts[Int(edgeB.prevKey >> 1)] = prevContact
        }

        if edgeB.nextKey != B3.nullIndex {
            var nextContact = contacts[Int(edgeB.nextKey >> 1)]
            nextContact.edges[Int(edgeB.nextKey & 1)].prevKey = edgeB.prevKey
            contacts[Int(edgeB.nextKey >> 1)] = nextContact
        }

        let edgeKeyB = (contactId << 1) | 1
        if bodyB.headContactKey == edgeKeyB {
            bodyB.headContactKey = edgeB.nextKey
        }
        bodyB.contactCount -= 1

        bodies[Int(bodyIdA)] = bodyA
        bodies[Int(bodyIdB)] = bodyB

        if (contact.flags & ContactFlags.simMeshContact.rawValue) != 0 {
            contact.meshContact = MeshContactState()
        }

        if contact.islandId != B3.nullIndex {
            unlinkContact(contactId)
            contact.islandId = B3.nullIndex
            contact.islandIndex = B3.nullIndex
        }

        // Remove contact from the array that owns it. Contacts in the awake set that are touching
        // live in the constraint graph and are handled by the solver.
        if contact.colorIndex == B3.nullIndex {
            b3Assert(
                contact.setIndex != SetType.awakeSet.rawValue
                    || (contact.flags & ContactFlags.touching.rawValue) == 0
            )
            let setIndex = Int(contact.setIndex)
            let localIndex = contact.localIndex
            let movedIndex = removeSwapArray(
                &solverSets[setIndex].contactIndices, Int(localIndex))
            if movedIndex != B3.nullIndex {
                let movedContactIndex = solverSets[setIndex].contactIndices[Int(localIndex)]
                var movedContact = contacts[Int(movedContactIndex)]
                movedContact.localIndex = localIndex
                contacts[Int(movedContactIndex)] = movedContact
            }
        } else {
            removeContactFromGraph(
                bodyIdA, bodyIdB, contact.colorIndex, contact.localIndex,
                (flags & ContactFlags.simMeshContact.rawValue) != 0)
        }
        // Free contact and id (preserve generation)
        contact.contactId = B3.nullIndex
        contact.setIndex = B3.nullIndex
        contact.colorIndex = B3.nullIndex
        contact.localIndex = B3.nullIndex
        contacts[Int(contactId)] = contact
        contactIdPool.free(contactId)

        if wakeBodies && touching {
            wakeBody(bodyIdA)
            wakeBody(bodyIdB)
        }
    }

    /// Destroy all contacts attached to a body.
    func destroyBodyContacts(_ bodyId: Int32, _ wakeBodies: Bool) {
        var edgeKey = bodies[Int(bodyId)].headContactKey
        while edgeKey != B3.nullIndex {
            let contactId = edgeKey >> 1
            let edgeIndex = Int(edgeKey & 1)

            let contact = contacts[Int(contactId)]
            edgeKey = contact.edges[edgeIndex].nextKey
            destroyContact(contactId, wakeBodies)
        }
    }

    // MARK: Pair generation

    struct NodePair {
        var a = TreeNode()
        var b = TreeNode()

        init() {}

        init(_ a: TreeNode, _ b: TreeNode) {
            self.a = a
            self.b = b
        }
    }

    /// Gather the sibling pairs with a moved node.
    func gatherMovedSiblings(_ tree: DynamicTree, _ pairIndices: inout [Int32]) -> Int32 {
        var count: Int32 = 0
        var pair: Int32 = 2
        while pair < tree.nodeEnd {
            let flags1 = tree.nodes[Int(pair)].flagIndex
            let flags2 = tree.nodes[Int(pair) + 1].flagIndex
            if ((flags1 | flags2) & movedNodeFlag) != 0 {
                pairIndices[Int(count)] = pair
                count += 1
            }
            pair += 2
        }

        return count
    }

    /// Did either move and if so do they overlap?
    @inline(__always)
    func testPair(_ a: TreeNode, _ b: TreeNode) -> Bool {
        if ((a.flagIndex | b.flagIndex) & movedNodeFlag) == 0 {
            return false
        }

        return aabbOverlaps(a.aabb, b.aabb)
    }

    /// Queue a candidate and run the gauntlet in batches so the pair-set and shape loads can be
    /// prefetched. The pair keys are sorted afterwards, so batching does not
    /// change the result.
    @inline(__always)
    func addCandidatePair(_ shapeIdA: Int32, _ shapeIdB: Int32, _ pairKeys: inout [UInt64]) {
        candidateBatch[candidateBatchCount] = SIMD2(minInt(shapeIdA, shapeIdB), maxInt(shapeIdA, shapeIdB))
        candidateBatchCount += 1
        if candidateBatchCount == candidateBatchSize {
            flushCandidatePairs(&pairKeys)
        }
    }

    func flushCandidatePairs(_ pairKeys: inout [UInt64]) {
        let count = candidateBatchCount
        candidateBatchCount = 0
        guard count > 0 else { return }
        var plain = InlineArray<32, SIMD2<Int32>>(repeating: .zero)
        var keys = InlineArray<32, UInt64>(repeating: 0)
        var hashes = InlineArray<32, UInt32>(repeating: 0)
        var plainCount = 0
        shapes.withUnsafeBufferPointer { shapeBuffer in
            let shapeBase = shapeBuffer.baseAddress!
            for i in 0..<count {
                let a = candidateBatch[i].x
                let b = candidateBatch[i].y
                let shapeA = shapeBase + Int(a)
                let shapeB = shapeBase + Int(b)
                // Metadata rejects same-body pairs before touching reference-counted geometry.
                if shapeA.pointee.bodyId == shapeB.pointee.bodyId { continue }
                if shapeA.pointee.type == .compoundShape, case .compound(let compound) = shapeA.pointee.geometry {
                    emitCompoundPairs(a, b, compound, &pairKeys)
                    continue
                }
                if shapeB.pointee.type == .compoundShape, case .compound(let compound) = shapeB.pointee.geometry {
                    emitCompoundPairs(b, a, compound, &pairKeys)
                    continue
                }
                plain[plainCount] = SIMD2(a, b)
                plainCount += 1
            }

            // Prefetch hash set entries.
            let pairSet = broadPhase.pairSet
            for i in 0..<plainCount {
                keys[i] = shapePairKey(plain[i].x, plain[i].y, 0)
                hashes[i] = keyHash(keys[i])
                b3Prefetch(UnsafeRawPointer(pairSet.items! + Int(hashes[i] & (pairSet.capacity - 1))))
            }

            // Cull existing pairs.
            var count2 = 0
            for i in 0..<plainCount where pairSet.containsHashedKey(keys[i], hashes[i]) == false {
                plain[count2] = plain[i]
                keys[count2] = keys[i]
                count2 += 1
            }

            // Prefetch shapes.
            for i in 0..<count2 {
                b3Prefetch(UnsafeRawPointer(shapeBase + Int(plain[i].x)))
                b3Prefetch(UnsafeRawPointer(shapeBase + Int(plain[i].y)))
            }

            for i in 0..<count2
            where shouldCreatePair(plain[i].x, plain[i].y, shapeBase + Int(plain[i].x), shapeBase + Int(plain[i].y)) {
                pairKeys.append(keys[i])
            }
        }
    }

    func collideProxyAndSubtree(
        _ proxy: TreeNode,
        _ nodes: UnsafeBufferPointer<TreeNode>,
        _ pairStart: Int32,
        _ pairKeys: inout [UInt64],
        _ stack: UnsafeMutablePointer<Int32>
    ) {
        let proxyMark = proxy.flagIndex & movedNodeFlag
        let shapeId = proxy.shapeIndex

        var stackCount = 0
        stack[stackCount] = pairStart
        stackCount += 1

        while stackCount > 0 {
            stackCount -= 1
            let pair = stack[stackCount]
            for i in 0..<2 {
                let node = nodes[Int(pair) + i]
                if ((node.flagIndex | proxyMark) & movedNodeFlag) == 0 {
                    continue
                }

                if aabbOverlaps(proxy.aabb, node.aabb) == false {
                    continue
                }

                if isLeaf(node) {
                    addCandidatePair(shapeId, node.shapeIndex, &pairKeys)
                } else {
                    if stackCount < treeStackSize {
                        stack[stackCount] = leftChild(node)
                        stackCount += 1
                    } else {
                        b3Assert(stackCount < treeStackSize)
                    }
                }
            }
        }
    }

    @inline(__always)
    func visitPair(
        _ arrayA: UnsafeBufferPointer<TreeNode>,
        _ arrayB: UnsafeBufferPointer<TreeNode>,
        _ nodeA: TreeNode,
        _ nodeB: TreeNode,
        _ stack: UnsafeMutablePointer<TraversalPair>,
        _ stackCount: inout Int,
        _ pairKeys: inout [UInt64],
        _ proxyStack: UnsafeMutablePointer<Int32>
    ) {
        if testPair(nodeA, nodeB) == false {
            return
        }

        let leafA = isLeaf(nodeA)
        let leafB = isLeaf(nodeB)
        if leafA && leafB {
            addCandidatePair(nodeA.shapeIndex, nodeB.shapeIndex, &pairKeys)
        } else if leafA {
            collideProxyAndSubtree(nodeA, arrayB, leftChild(nodeB), &pairKeys, proxyStack)
        } else if leafB {
            collideProxyAndSubtree(nodeB, arrayA, leftChild(nodeA), &pairKeys, proxyStack)
        } else {
            if stackCount < treeStackSize {
                stack[stackCount] = TraversalPair(a: leftChild(nodeA), b: leftChild(nodeB))
                stackCount += 1
            } else {
                b3Assert(stackCount < treeStackSize)
            }
        }
    }

    func collideCrossPairs(
        _ arrayA: UnsafeBufferPointer<TreeNode>,
        _ arrayB: UnsafeBufferPointer<TreeNode>,
        _ subtreeA: TreeNode,
        _ subtreeB: TreeNode,
        _ pairKeys: inout [UInt64]
    ) {
        var stackCount = 0

        // Seed the stack.
        visitPair(
            arrayA, arrayB, subtreeA, subtreeB, broadPhasePairStack, &stackCount, &pairKeys,
            broadPhaseProxyStack)

        while stackCount > 0 {
            stackCount -= 1
            let pair = broadPhasePairStack[stackCount]
            for i in 0..<2 {
                for j in 0..<2 {
                    visitPair(
                        arrayA, arrayB, arrayA[Int(pair.a) + i], arrayB[Int(pair.b) + j],
                        broadPhasePairStack, &stackCount, &pairKeys, broadPhaseProxyStack
                    )
                }
            }
        }
    }

    /// This does a serial cross-tree breadth first search until the queue is full.
    /// The queue and result arrays are caller-owned scratch so a step does not allocate them.
    func gatherCrossSeeds(
        _ treeA: DynamicTree, _ treeB: DynamicTree,
        _ queue: inout [NodePair], _ seeds: inout [NodePair]
    ) {
        let nodesA = treeA.nodes
        let nodesB = treeB.nodes

        let crossSeedCount = 64
        let queueCapacity = 2 * crossSeedCount
        if queue.count < queueCapacity {
            queue.append(contentsOf: repeatElement(NodePair(), count: queueCapacity - queue.count))
        }
        let mask = queueCapacity - 1
        var head = 0
        var tail = 0

        seeds.removeAll(keepingCapacity: true)
        seeds.reserveCapacity(crossSeedCount)

        let rootA = nodesA[Int(rootNode)]
        let rootB = nodesB[Int(rootNode)]
        if testPair(rootA, rootB) {
            queue[tail & mask] = NodePair(rootA, rootB)
            tail += 1
        }

        while head < tail && seeds.count + (tail - head) + 3 < crossSeedCount {
            let pair = queue[head & mask]
            head += 1

            if isLeaf(pair.a) || isLeaf(pair.b) {
                seeds.append(pair)
                continue
            }

            let a = Int(leftChild(pair.a))
            let b = Int(leftChild(pair.b))

            // Nodes have two children each, so four combinations.
            for i in 0..<2 {
                for j in 0..<2 {
                    if testPair(nodesA[a + i], nodesB[b + j]) {
                        queue[tail & mask] = NodePair(nodesA[a + i], nodesB[b + j])
                        tail += 1
                    }
                }
            }
        }

        while head < tail {
            seeds.append(queue[head & mask])
            head += 1
        }
    }

    /// Generate broad-phase pairs and create contacts for new pairs.
    func updateBroadPhasePairs() {
        let staticTree = broadPhase.trees[Int(BodyType.staticBody.rawValue)]
        let kinematicTree = broadPhase.trees[Int(BodyType.kinematicBody.rawValue)]
        let dynamicTree = broadPhase.trees[Int(BodyType.dynamicBody.rawValue)]

        var needUpdate = staticTree.hasMoved
        needUpdate = needUpdate || kinematicTree.needsRebuild
        needUpdate = needUpdate || dynamicTree.needsRebuild

        if needUpdate == false {
            return
        }

        pairKeyScratch.removeAll(keepingCapacity: true)
        if pairKeyScratch.capacity < 64 {
            pairKeyScratch.reserveCapacity(64)
        }

        // Get the sibling pairs of the dynamic body tree that have moved. Size the buffer from the
        // actual pair count so a rebuild that leaves an odd nodeEnd cannot overflow it.
        let pairCapacity = maxInt((dynamicTree.nodeEnd - 2) / 2 + 1, 1)
        if movedSiblingScratch.count < Int(pairCapacity) {
            movedSiblingScratch.append(
                contentsOf: repeatElement(0, count: Int(pairCapacity) - movedSiblingScratch.count))
        }
        let dynamicMoveCount = gatherMovedSiblings(dynamicTree, &movedSiblingScratch)

        // Get seeds for colliding against the static and kinematic trees.
        gatherCrossSeeds(dynamicTree, staticTree, &crossSeedQueue, &staticSeedScratch)
        gatherCrossSeeds(dynamicTree, kinematicTree, &crossSeedQueue, &kinematicSeedScratch)
        b3Assert(staticSeedScratch.count <= 64)
        b3Assert(kinematicSeedScratch.count <= 64)

        // End all read borrows before rebuilding the trees.
        dynamicTree.nodes.withUnsafeBufferPointer { dynamicNodes in
            staticTree.nodes.withUnsafeBufferPointer { staticNodes in
                kinematicTree.nodes.withUnsafeBufferPointer { kinematicNodes in

                    // Collide the dynamic body tree against the static and kinematic trees.
                    for seed in staticSeedScratch {
                        collideCrossPairs(dynamicNodes, staticNodes, seed.a, seed.b, &pairKeyScratch)
                    }

                    for seed in kinematicSeedScratch {
                        collideCrossPairs(dynamicNodes, kinematicNodes, seed.a, seed.b, &pairKeyScratch)
                    }

                    // Collide the dynamic body tree against itself.
                    for i in 0..<Int(dynamicMoveCount) {
                        let nodeIndex = movedSiblingScratch[i]
                        collideCrossPairs(
                            dynamicNodes, dynamicNodes, dynamicNodes[Int(nodeIndex)], dynamicNodes[Int(nodeIndex) + 1],
                            &pairKeyScratch
                        )
                    }

                }
            }
        }

        flushCandidatePairs(&pairKeyScratch)

        staticTree.clearMoved()

        // Update stale trees.
        dynamicTree.rebuild(fullBuild: false)
        kinematicTree.rebuild(fullBuild: false)

        // Pairs arrive in deterministic order but scrambled relative to body and shape order.
        // Sort them here.
        pairKeyScratch.sort()

        for i in 0..<pairKeyScratch.count {
            let key = pairKeyScratch[i]

            // The traversal is cross subtree and the seeds are disjoint, so a duplicate key means
            // a real bug. Skip rather than double add to the pair set.
            let duplicate = i > 0 && key == pairKeyScratch[i - 1]
            b3Validate(duplicate == false)
            if duplicate {
                continue
            }

            let shapeIdA = Int32((key >> UInt64(64 - B3.shapePower)) & UInt64(B3.maxShapes - 1))
            let shapeIdB = Int32((key >> UInt64(64 - 2 * B3.shapePower)) & UInt64(B3.maxShapes - 1))
            let childIndex = Int32(key & UInt64(B3.maxChildShapes - 1))

            createContact(shapeIdA, shapeIdB, childIndex)
        }
    }
}
