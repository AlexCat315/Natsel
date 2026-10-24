

// Joint model, creation, and destruction (joint.c).

/// Distance joint solver data.
struct DistanceJointSim: BitwiseCopyable {
    var length: Float = 0
    var hertz: Float = 0
    var dampingRatio: Float = 0
    var lowerSpringForce: Float = 0
    var upperSpringForce: Float = 0
    var minLength: Float = 0
    var maxLength: Float = 0

    var maxMotorForce: Float = 0
    var motorSpeed: Float = 0

    var impulse: Float = 0
    var lowerImpulse: Float = 0
    var upperImpulse: Float = 0
    var motorImpulse: Float = 0

    var indexA: Int32 = B3.nullIndex
    var indexB: Int32 = B3.nullIndex
    var anchorA = Vec3()
    var anchorB = Vec3()
    var deltaCenter = Vec3()
    var distanceSoftness = Softness()
    var axialMass: Float = 0

    var enableSpring = false
    var enableLimit = false
    var enableMotor = false

    init() {}
}

extension JointSim {
    var distanceJoint: DistanceJointSim {
        get { distance }
        set { distance = newValue }
    }
}

/// Result of creating a joint.
struct JointPair {
    var jointId: Int32 = B3.nullIndex
    var sim: JointSim = JointSim()
}

extension World {
    /// Create a filter joint. This disables collision between the attached bodies and keeps them
    /// in the same island, matching `b3CreateFilterJoint`.
    @discardableResult
    public func createFilterJoint(_ def: FilterJointDef) -> JointID {
        let pair = createJoint(def.base, .filterJoint)
        let jointId = JointID(
            index1: pair.jointId + 1, world0: worldId,
            generation: joints[Int(pair.jointId)].generation
        )
        return jointId
    }

    /// Destroy all contacts between two bodies.
    func destroyContactsBetweenBodies(_ bodyIdA: Int32, _ bodyIdB: Int32) {
        let bodyA = bodies[Int(bodyIdA)]
        let bodyB = bodies[Int(bodyIdB)]

        // use the smaller of the two contact lists
        var contactKey: Int32
        var otherBodyId: Int32
        if bodyA.contactCount < bodyB.contactCount {
            contactKey = bodyA.headContactKey
            otherBodyId = bodyIdB
        } else {
            contactKey = bodyB.headContactKey
            otherBodyId = bodyIdA
        }

        // no need to wake bodies when a joint removes collision between them
        let wakeBodies = false

        while contactKey != B3.nullIndex {
            let contactId = contactKey >> 1
            let edgeIndex = Int(contactKey & 1)

            let contact = contacts[Int(contactId)]
            contactKey = contact.edges[edgeIndex].nextKey

            let otherEdgeIndex = edgeIndex ^ 1
            if contact.edges[otherEdgeIndex].bodyId == otherBodyId {
                // Careful, this removes the contact from the current doubly linked list
                destroyContact(contactId, wakeBodies)
            }
        }
    }

    /// Create a joint and place it in the correct solver set and graph color.
    func createJoint(_ def: JointDef, _ type: JointType) -> JointPair {
        let bodyIdA = def.bodyIdA.index1 - 1
        let bodyIdB = def.bodyIdB.index1 - 1

        b3Assert(bodyIdA != bodyIdB)

        // If the joint prevents collisions, then destroy all contacts between attached bodies.
        if def.collideConnected == false {
            destroyContactsBetweenBodies(bodyIdA, bodyIdB)
        }

        var bodyA = bodies[Int(bodyIdA)]
        var bodyB = bodies[Int(bodyIdB)]

        let maxSetIndex = maxInt(bodyA.setIndex, bodyB.setIndex)

        // Create joint id and joint
        let jointId = jointIdPool.alloc()
        if Int(jointId) == joints.count {
            joints.append(Joint())
        }

        var joint = Joint()
        joint.jointId = jointId
        joint.userData = def.userData
        // A recycled slot keeps its revision so stale ids stay invalid (b3CreateJointInternal).
        joint.generation = joints[Int(jointId)].generation &+ 1
        joint.setIndex = B3.nullIndex
        joint.colorIndex = B3.nullIndex
        joint.localIndex = B3.nullIndex
        joint.islandId = B3.nullIndex
        joint.islandIndex = B3.nullIndex
        joint.drawScale = def.drawScale
        joint.type = type
        joint.collideConnected = def.collideConnected

        // Doubly linked list on bodyA
        joint.edges[0].bodyId = bodyIdA
        joint.edges[0].prevKey = B3.nullIndex
        joint.edges[0].nextKey = bodyA.headJointKey

        let keyA = (jointId << 1) | 0
        if bodyA.headJointKey != B3.nullIndex {
            let jointAId = bodyA.headJointKey >> 1
            var jointA = joints[Int(jointAId)]
            let edgeIndex = Int(bodyA.headJointKey & 1)
            jointA.edges[edgeIndex].prevKey = keyA
            joints[Int(jointAId)] = jointA
        }
        bodyA.headJointKey = keyA
        bodyA.jointCount += 1

        // Doubly linked list on bodyB
        joint.edges[1].bodyId = bodyIdB
        joint.edges[1].prevKey = B3.nullIndex
        joint.edges[1].nextKey = bodyB.headJointKey

        let keyB = (jointId << 1) | 1
        if bodyB.headJointKey != B3.nullIndex {
            let jointBId = bodyB.headJointKey >> 1
            var jointB = joints[Int(jointBId)]
            let edgeIndex = Int(bodyB.headJointKey & 1)
            jointB.edges[edgeIndex].prevKey = keyB
            joints[Int(jointBId)] = jointB
        }
        bodyB.headJointKey = keyB
        bodyB.jointCount += 1

        bodies[Int(bodyIdA)] = bodyA
        bodies[Int(bodyIdB)] = bodyB
        joints[Int(jointId)] = joint

        var sim = JointSim()
        var pair = JointPair(jointId: jointId, sim: sim)

        if bodyA.setIndex == SetType.disabledSet.rawValue || bodyB.setIndex == SetType.disabledSet.rawValue {
            // if either body is disabled, create in disabled set
            joint.setIndex = SetType.disabledSet.rawValue
            var set = solverSets[Int(SetType.disabledSet.rawValue)]
            joint.localIndex = Int32(set.jointSims.count)

            sim = JointSim()
            sim.jointId = jointId
            sim.bodyIdA = bodyIdA
            sim.bodyIdB = bodyIdB
            set.jointSims.append(sim)
            solverSets[Int(SetType.disabledSet.rawValue)] = set

            joints[Int(jointId)] = joint
            pair.sim = sim
        } else if bodyA.type != .dynamicBody && bodyB.type != .dynamicBody {
            // joint is not attached to a dynamic body
            joint.setIndex = SetType.staticSet.rawValue
            var set = solverSets[Int(SetType.staticSet.rawValue)]
            joint.localIndex = Int32(set.jointSims.count)

            sim = JointSim()
            sim.jointId = jointId
            sim.bodyIdA = bodyIdA
            sim.bodyIdB = bodyIdB
            set.jointSims.append(sim)
            solverSets[Int(SetType.staticSet.rawValue)] = set

            joints[Int(jointId)] = joint
            pair.sim = sim
        } else if bodyA.setIndex == SetType.awakeSet.rawValue || bodyB.setIndex == SetType.awakeSet.rawValue {
            // if either body is sleeping, wake it
            if maxSetIndex >= SetType.firstSleepingSet.rawValue {
                wakeSolverSet(maxSetIndex)
                bodyA = bodies[Int(bodyIdA)]
                bodyB = bodies[Int(bodyIdB)]
            }

            joint.setIndex = SetType.awakeSet.rawValue
            joints[Int(jointId)] = joint

            let index = createJointInGraph(jointId)
            joint = joints[Int(jointId)]
            let colorIndex = joint.colorIndex
            sim = constraintGraph.colors[Int(colorIndex)].jointSims[Int(index)]
            sim.jointId = jointId
            sim.bodyIdA = bodyIdA
            sim.bodyIdB = bodyIdB
            pair.sim = sim
        } else {
            // joint connected between sleeping and/or static bodies
            b3Assert(
                bodyA.setIndex >= SetType.firstSleepingSet.rawValue
                    || bodyB.setIndex >= SetType.firstSleepingSet.rawValue
            )
            b3Assert(
                bodyA.setIndex != SetType.staticSet.rawValue || bodyB.setIndex != SetType.staticSet.rawValue
            )

            // joint should go into the sleeping set (not static set)
            var setIndex = maxSetIndex

            joint.setIndex = setIndex
            var set = solverSets[Int(setIndex)]
            joint.localIndex = Int32(set.jointSims.count)

            sim = JointSim()
            sim.jointId = jointId
            sim.bodyIdA = bodyIdA
            sim.bodyIdB = bodyIdB
            set.jointSims.append(sim)
            solverSets[Int(setIndex)] = set

            // Publish the local index before any set merge can move this joint.
            joints[Int(jointId)] = joint

            if bodyA.setIndex != bodyB.setIndex
                && bodyA.setIndex >= SetType.firstSleepingSet.rawValue
                && bodyB.setIndex >= SetType.firstSleepingSet.rawValue
            {
                // merge sleeping sets
                mergeSolverSets(bodyA.setIndex, bodyB.setIndex)
                b3Assert(bodies[Int(bodyIdA)].setIndex == bodies[Int(bodyIdB)].setIndex)

                // fix potentially invalid set index
                setIndex = bodies[Int(bodyIdA)].setIndex
                joint = joints[Int(jointId)]
                joint.setIndex = setIndex
                joints[Int(jointId)] = joint

                // Careful! The joint sim was orphaned by the set merge.
                sim = solverSets[Int(setIndex)].jointSims[Int(joint.localIndex)]
                sim.jointId = jointId
                sim.bodyIdA = bodyIdA
                sim.bodyIdB = bodyIdB
            }

            b3Assert(joint.setIndex == setIndex)
            pair.sim = sim
        }

        sim.localFrameA = def.localFrameA
        sim.localFrameB = def.localFrameB
        sim.type = type
        sim.constraintHertz = def.constraintHertz
        sim.constraintDampingRatio = def.constraintDampingRatio
        sim.constraintSoftness = Softness()
        sim.constraintSoftness.biasRate = 0
        sim.constraintSoftness.massScale = 1
        sim.constraintSoftness.impulseScale = 0
        sim.forceThreshold = def.forceThreshold
        sim.torqueThreshold = def.torqueThreshold

        b3Assert(sim.jointId == jointId)
        b3Assert(sim.bodyIdA == bodyIdA)
        b3Assert(sim.bodyIdB == bodyIdB)

        writeJointSim(jointId, sim)

        joint = joints[Int(jointId)]
        if joint.setIndex > SetType.disabledSet.rawValue {
            // Add edge to island graph
            linkJoint(jointId)
        }

        pair.sim = sim
        pair.jointId = jointId
        return pair
    }

    /// Write the sim back to wherever it lives (graph color or solver set).
    func writeJointSim(_ jointId: Int32, _ sim: JointSim) {
        let joint = joints[Int(jointId)]
        if joint.setIndex == SetType.awakeSet.rawValue {
            constraintGraph.colors[Int(joint.colorIndex)].jointSims[Int(joint.localIndex)] = sim
        } else if joint.setIndex != B3.nullIndex {
            solverSets[Int(joint.setIndex)].jointSims[Int(joint.localIndex)] = sim
        }
    }

    /// Read the sim from wherever it lives.
    func readJointSim(_ jointId: Int32) -> JointSim {
        let joint = joints[Int(jointId)]
        if joint.setIndex == SetType.awakeSet.rawValue {
            return constraintGraph.colors[Int(joint.colorIndex)].jointSims[Int(joint.localIndex)]
        }
        if joint.setIndex != B3.nullIndex {
            return solverSets[Int(joint.setIndex)].jointSims[Int(joint.localIndex)]
        }
        return JointSim()
    }

    // MARK: Distance joint

    @discardableResult
    public func createDistanceJoint(_ def: DistanceJointDef) -> JointID {
        let pair = createJoint(def.base, .distanceJoint)

        var sim = pair.sim
        var joint = DistanceJointSim()
        joint.length = maxFloat(def.length, B3.linearSlop)
        joint.hertz = def.hertz
        joint.dampingRatio = def.dampingRatio
        joint.lowerSpringForce = def.lowerSpringForce
        joint.upperSpringForce = def.upperSpringForce
        joint.minLength = maxFloat(def.minLength, B3.linearSlop)
        joint.maxLength = maxFloat(def.minLength, def.maxLength)
        joint.maxMotorForce = def.maxMotorForce
        joint.motorSpeed = def.motorSpeed
        joint.enableSpring = def.enableSpring
        joint.enableLimit = def.enableLimit
        joint.enableMotor = def.enableMotor
        joint.impulse = 0
        joint.lowerImpulse = 0
        joint.upperImpulse = 0
        joint.motorImpulse = 0
        sim.distanceJoint = joint
        writeJointSim(pair.jointId, sim)

        let j = joints[Int(pair.jointId)]
        return JointID(index1: pair.jointId + 1, world0: worldId, generation: j.generation)
    }

    // MARK: Distance joint tuning

    private func updateDistanceJoint(_ jointId: JointID, _ body: (inout DistanceJointSim) -> Void) -> Bool {
        let index = jointId.index1 - 1
        guard index >= 0, Int(index) < joints.count else { return false }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.type == .distanceJoint else { return false }
        var sim = readJointSim(index)
        var distance = sim.distanceJoint
        body(&distance)
        sim.distanceJoint = distance
        writeJointSim(index, sim)
        return true
    }

    /// Set the rest length. Resets the axial impulses.
    public func setDistanceJointLength(_ jointId: JointID, _ length: Float) {
        _ = updateDistanceJoint(jointId) { joint in
            joint.length = clampFloat(length, B3.linearSlop, B3.huge)
            joint.impulse = 0
            joint.lowerImpulse = 0
            joint.upperImpulse = 0
        }
    }

    public func distanceJointLength(_ jointId: JointID) -> Float? {
        distanceJointSim(jointId)?.length
    }

    public func enableDistanceJointLimit(_ jointId: JointID, _ enable: Bool) {
        _ = updateDistanceJoint(jointId) { $0.enableLimit = enable }
    }

    public func isDistanceJointLimitEnabled(_ jointId: JointID) -> Bool {
        distanceJointSim(jointId)?.enableLimit ?? false
    }

    /// Set the length range. Resets the limit impulses.
    public func setDistanceJointLengthRange(_ jointId: JointID, _ minLength: Float, _ maxLength: Float) {
        _ = updateDistanceJoint(jointId) { joint in
            let lower = clampFloat(minLength, B3.linearSlop, B3.huge)
            let upper = clampFloat(maxLength, B3.linearSlop, B3.huge)
            joint.minLength = minFloat(lower, upper)
            joint.maxLength = maxFloat(lower, upper)
            joint.impulse = 0
            joint.lowerImpulse = 0
            joint.upperImpulse = 0
        }
    }

    public func distanceJointMinLength(_ jointId: JointID) -> Float? {
        distanceJointSim(jointId)?.minLength
    }

    public func distanceJointMaxLength(_ jointId: JointID) -> Float? {
        distanceJointSim(jointId)?.maxLength
    }

    /// Current distance between the joint anchors.
    public func distanceJointCurrentLength(_ jointId: JointID) -> Float? {
        let index = jointId.index1 - 1
        guard index >= 0, Int(index) < joints.count else { return nil }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.type == .distanceJoint else { return nil }
        let sim = readJointSim(index)

        let transformA = bodyTransformQuick(sim.bodyIdA)
        let transformB = bodyTransformQuick(sim.bodyIdB)
        let pA = transformPoint(transformA, sim.localFrameA.p)
        let pB = transformPoint(transformB, sim.localFrameB.p)
        return length(subPos(pB, pA))
    }

    public func enableDistanceJointSpring(_ jointId: JointID, _ enable: Bool) {
        _ = updateDistanceJoint(jointId) { $0.enableSpring = enable }
    }

    public func isDistanceJointSpringEnabled(_ jointId: JointID) -> Bool {
        distanceJointSim(jointId)?.enableSpring ?? false
    }

    public func setDistanceJointSpringForceRange(_ jointId: JointID, _ lowerForce: Float, _ upperForce: Float) {
        _ = updateDistanceJoint(jointId) { joint in
            joint.lowerSpringForce = lowerForce
            joint.upperSpringForce = upperForce
        }
    }

    public func distanceJointSpringForceRange(_ jointId: JointID) -> (lower: Float, upper: Float)? {
        guard let sim = distanceJointSim(jointId) else { return nil }
        return (sim.lowerSpringForce, sim.upperSpringForce)
    }

    public func setDistanceJointSpringHertz(_ jointId: JointID, _ hertz: Float) {
        _ = updateDistanceJoint(jointId) { $0.hertz = hertz }
    }

    public func setDistanceJointSpringDampingRatio(_ jointId: JointID, _ dampingRatio: Float) {
        _ = updateDistanceJoint(jointId) { $0.dampingRatio = dampingRatio }
    }

    public func distanceJointSpringHertz(_ jointId: JointID) -> Float? {
        distanceJointSim(jointId)?.hertz
    }

    public func distanceJointSpringDampingRatio(_ jointId: JointID) -> Float? {
        distanceJointSim(jointId)?.dampingRatio
    }

    public func enableDistanceJointMotor(_ jointId: JointID, _ enable: Bool) {
        _ = updateDistanceJoint(jointId) { joint in
            if enable != joint.enableMotor {
                joint.enableMotor = enable
                joint.motorImpulse = 0
            }
        }
    }

    public func isDistanceJointMotorEnabled(_ jointId: JointID) -> Bool {
        distanceJointSim(jointId)?.enableMotor ?? false
    }

    public func setDistanceJointMotorSpeed(_ jointId: JointID, _ motorSpeed: Float) {
        _ = updateDistanceJoint(jointId) { $0.motorSpeed = motorSpeed }
    }

    public func distanceJointMotorSpeed(_ jointId: JointID) -> Float? {
        distanceJointSim(jointId)?.motorSpeed
    }

    public func setDistanceJointMaxMotorForce(_ jointId: JointID, _ force: Float) {
        _ = updateDistanceJoint(jointId) { $0.maxMotorForce = force }
    }

    public func distanceJointMaxMotorForce(_ jointId: JointID) -> Float? {
        distanceJointSim(jointId)?.maxMotorForce
    }

    /// Motor force reported by the solver for the last step.
    public func distanceJointMotorForce(_ jointId: JointID) -> Float? {
        guard let sim = distanceJointSim(jointId) else { return nil }
        return inv_h * sim.motorImpulse
    }

    func distanceJointSim(_ jointId: JointID) -> DistanceJointSim? {
        let index = jointId.index1 - 1
        guard index >= 0 && Int(index) < joints.count else { return nil }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.type == .distanceJoint else { return nil }
        return readJointSim(index).distanceJoint
    }

    // MARK: Destruction

    /// Destroy a joint and remove it from its body lists, island, and solver set.
    public func destroyJoint(_ jointId: Int32, _ wakeAttached: Bool) {
        var joint = joints[Int(jointId)]
        if joint.type == .sixDOFJoint {
            releaseSixDOFData(readJointSim(jointId).sixDOF.slot)
        } else if joint.type == .pathJoint {
            releasePathData(readJointSim(jointId).path.slot)
        }

        let edgeA = joint.edges[0]
        let edgeB = joint.edges[1]

        let idA = edgeA.bodyId
        let idB = edgeB.bodyId
        var bodyA = bodies[Int(idA)]
        var bodyB = bodies[Int(idB)]

        // Remove from body A
        if edgeA.prevKey != B3.nullIndex {
            let prevJointId = edgeA.prevKey >> 1
            var prevJoint = joints[Int(prevJointId)]
            let edgeIndex = Int(edgeA.prevKey & 1)
            prevJoint.edges[edgeIndex].nextKey = edgeA.nextKey
            joints[Int(prevJointId)] = prevJoint
        }

        if edgeA.nextKey != B3.nullIndex {
            let nextJointId = edgeA.nextKey >> 1
            var nextJoint = joints[Int(nextJointId)]
            let edgeIndex = Int(edgeA.nextKey & 1)
            nextJoint.edges[edgeIndex].prevKey = edgeA.prevKey
            joints[Int(nextJointId)] = nextJoint
        }

        let edgeKeyA = (jointId << 1) | 0
        if bodyA.headJointKey == edgeKeyA {
            bodyA.headJointKey = edgeA.nextKey
        }
        bodyA.jointCount -= 1

        // Remove from body B
        if edgeB.prevKey != B3.nullIndex {
            let prevJointId = edgeB.prevKey >> 1
            var prevJoint = joints[Int(prevJointId)]
            let edgeIndex = Int(edgeB.prevKey & 1)
            prevJoint.edges[edgeIndex].nextKey = edgeB.nextKey
            joints[Int(prevJointId)] = prevJoint
        }

        if edgeB.nextKey != B3.nullIndex {
            let nextJointId = edgeB.nextKey >> 1
            var nextJoint = joints[Int(nextJointId)]
            let edgeIndex = Int(edgeB.nextKey & 1)
            nextJoint.edges[edgeIndex].prevKey = edgeB.prevKey
            joints[Int(nextJointId)] = nextJoint
        }

        let edgeKeyB = (jointId << 1) | 1
        if bodyB.headJointKey == edgeKeyB {
            bodyB.headJointKey = edgeB.nextKey
        }
        bodyB.jointCount -= 1

        bodies[Int(idA)] = bodyA
        bodies[Int(idB)] = bodyB

        if joint.islandId != B3.nullIndex {
            b3Assert(joint.setIndex > SetType.disabledSet.rawValue)
            unlinkJoint(jointId)
            joint = joints[Int(jointId)]
        }

        // Remove joint from solver set that owns it
        let setIndex = joint.setIndex
        let localIndex = joint.localIndex

        if setIndex == SetType.awakeSet.rawValue {
            removeJointFromGraph(joint.edges[0].bodyId, joint.edges[1].bodyId, joint.colorIndex, localIndex)
        } else if setIndex != B3.nullIndex {
            var set = solverSets[Int(setIndex)]
            let movedIndex = removeSwapArray(&set.jointSims, Int(localIndex))
            if movedIndex != B3.nullIndex {
                let movedId = set.jointSims[Int(localIndex)].jointId
                var movedJoint = joints[Int(movedId)]
                b3Assert(movedJoint.localIndex == movedIndex)
                movedJoint.localIndex = localIndex
                joints[Int(movedId)] = movedJoint
            }
            solverSets[Int(setIndex)] = set
        }

        // Free joint and id (preserve joint revision)
        joint.setIndex = B3.nullIndex
        joint.localIndex = B3.nullIndex
        joint.colorIndex = B3.nullIndex
        joint.jointId = B3.nullIndex
        joints[Int(jointId)] = joint
        jointIdPool.free(jointId)

        if wakeAttached {
            wakeBody(idA)
            wakeBody(idB)
        }
    }
}

// MARK: - Generic joint tuning

extension World {
    /// Read-modify-write the sim of a joint of a given type.
    private func mutateJointSim(
        _ jointId: JointID, _ type: JointType, _ body: (inout JointSim) -> Void
    ) -> Bool {
        let index = jointId.index1 - 1
        guard index >= 0, Int(index) < joints.count else { return false }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.type == type else { return false }
        var sim = readJointSim(index)
        body(&sim)
        writeJointSim(index, sim)
        return true
    }

    /// Read the sim of a joint of a given type.
    fileprivate func jointSim(_ jointId: JointID, _ type: JointType) -> JointSim? {
        let index = jointId.index1 - 1
        guard index >= 0, Int(index) < joints.count else { return nil }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.type == type else { return nil }
        return readJointSim(index)
    }

    /// The type of a joint, or nil if the id is invalid.
    public func jointType(_ jointId: JointID) -> JointType? {
        let index = jointId.index1 - 1
        guard jointId.world0 == worldId, index >= 0, Int(index) < joints.count else { return nil }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex, joint.generation == jointId.generation else { return nil }
        return joint.type
    }

    /// The two bodies attached to a joint.
    public func jointBodies(_ jointId: JointID) -> (a: BodyID, b: BodyID)? {
        let index = jointId.index1 - 1
        guard index >= 0, Int(index) < joints.count else { return nil }
        let joint = joints[Int(index)]
        guard joint.jointId != B3.nullIndex else { return nil }
        return (makeBodyID(joint.edges[0].bodyId), makeBodyID(joint.edges[1].bodyId))
    }

    /// True while the joint id refers to a live joint.
    public func jointIsValid(_ jointId: JointID) -> Bool {
        jointType(jointId) != nil
    }

    // MARK: Revolute

    @discardableResult
    public func setRevoluteJointLimits(_ jointId: JointID, _ lower: Float, _ upper: Float) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { sim in
            sim.revolute.lowerAngle = lower
            sim.revolute.upperAngle = upper
            sim.revolute.lowerImpulse = 0
            sim.revolute.upperImpulse = 0
        }
    }

    public func revoluteJointLimits(_ jointId: JointID) -> (lower: Float, upper: Float)? {
        guard let sim = jointSim(jointId, .revoluteJoint) else { return nil }
        return (sim.revolute.lowerAngle, sim.revolute.upperAngle)
    }

    @discardableResult
    public func enableRevoluteJointLimit(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { $0.revolute.enableLimit = enable }
    }

    public func isRevoluteJointLimitEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .revoluteJoint)?.revolute.enableLimit ?? false
    }

    @discardableResult
    public func enableRevoluteJointMotor(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { sim in
            if enable != sim.revolute.enableMotor {
                sim.revolute.enableMotor = enable
                sim.revolute.motorImpulse = 0
            }
        }
    }

    public func isRevoluteJointMotorEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .revoluteJoint)?.revolute.enableMotor ?? false
    }

    @discardableResult
    public func setRevoluteJointMotorSpeed(_ jointId: JointID, _ speed: Float) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { $0.revolute.motorSpeed = speed }
    }

    public func revoluteJointMotorSpeed(_ jointId: JointID) -> Float? {
        jointSim(jointId, .revoluteJoint)?.revolute.motorSpeed
    }

    @discardableResult
    public func setRevoluteJointMaxMotorTorque(_ jointId: JointID, _ torque: Float) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { $0.revolute.maxMotorTorque = torque }
    }

    public func revoluteJointMaxMotorTorque(_ jointId: JointID) -> Float? {
        jointSim(jointId, .revoluteJoint)?.revolute.maxMotorTorque
    }

    /// Motor torque reported by the solver for the last step.
    public func revoluteJointMotorTorque(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .revoluteJoint) else { return nil }
        return inv_h * sim.revolute.motorImpulse
    }

    /// Current hinge angle relative to the joint target.
    public func revoluteJointAngle(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .revoluteJoint) else { return nil }
        let quatA = mulQuat(bodyTransformQuick(sim.bodyIdA).q, sim.localFrameA.q)
        let quatB = mulQuat(bodyTransformQuick(sim.bodyIdB).q, sim.localFrameB.q)
        return getTwistAngle(invMulQuat(quatA, quatB))
    }

    @discardableResult
    public func enableRevoluteJointSpring(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { $0.revolute.enableSpring = enable }
    }

    public func isRevoluteJointSpringEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .revoluteJoint)?.revolute.enableSpring ?? false
    }

    @discardableResult
    public func setRevoluteJointSpringHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { $0.revolute.hertz = hertz }
    }

    @discardableResult
    public func setRevoluteJointSpringDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { $0.revolute.dampingRatio = ratio }
    }

    public func revoluteJointSpringHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .revoluteJoint)?.revolute.hertz
    }

    public func revoluteJointSpringDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .revoluteJoint)?.revolute.dampingRatio
    }

    // MARK: Prismatic

    @discardableResult
    public func setPrismaticJointLimits(_ jointId: JointID, _ lower: Float, _ upper: Float) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { sim in
            sim.prismatic.lowerTranslation = lower
            sim.prismatic.upperTranslation = upper
            sim.prismatic.lowerImpulse = 0
            sim.prismatic.upperImpulse = 0
        }
    }

    public func prismaticJointLimits(_ jointId: JointID) -> (lower: Float, upper: Float)? {
        guard let sim = jointSim(jointId, .prismaticJoint) else { return nil }
        return (sim.prismatic.lowerTranslation, sim.prismatic.upperTranslation)
    }

    @discardableResult
    public func enablePrismaticJointLimit(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { $0.prismatic.enableLimit = enable }
    }

    public func isPrismaticJointLimitEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .prismaticJoint)?.prismatic.enableLimit ?? false
    }

    @discardableResult
    public func enablePrismaticJointMotor(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { sim in
            if enable != sim.prismatic.enableMotor {
                sim.prismatic.enableMotor = enable
                sim.prismatic.motorImpulse = 0
            }
        }
    }

    public func isPrismaticJointMotorEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .prismaticJoint)?.prismatic.enableMotor ?? false
    }

    @discardableResult
    public func setPrismaticJointMotorSpeed(_ jointId: JointID, _ speed: Float) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { $0.prismatic.motorSpeed = speed }
    }

    public func prismaticJointMotorSpeed(_ jointId: JointID) -> Float? {
        jointSim(jointId, .prismaticJoint)?.prismatic.motorSpeed
    }

    @discardableResult
    public func setPrismaticJointMaxMotorForce(_ jointId: JointID, _ force: Float) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { $0.prismatic.maxMotorForce = force }
    }

    public func prismaticJointMaxMotorForce(_ jointId: JointID) -> Float? {
        jointSim(jointId, .prismaticJoint)?.prismatic.maxMotorForce
    }

    public func prismaticJointMotorForce(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .prismaticJoint) else { return nil }
        return inv_h * sim.prismatic.motorImpulse
    }

    @discardableResult
    public func enablePrismaticJointSpring(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { $0.prismatic.enableSpring = enable }
    }

    public func isPrismaticJointSpringEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .prismaticJoint)?.prismatic.enableSpring ?? false
    }

    @discardableResult
    public func setPrismaticJointSpringHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { $0.prismatic.hertz = hertz }
    }

    @discardableResult
    public func setPrismaticJointSpringDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { $0.prismatic.dampingRatio = ratio }
    }

    public func prismaticJointSpringHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .prismaticJoint)?.prismatic.hertz
    }

    public func prismaticJointSpringDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .prismaticJoint)?.prismatic.dampingRatio
    }

    @discardableResult
    public func setPrismaticJointTargetTranslation(_ jointId: JointID, _ value: Float) -> Bool {
        mutateJointSim(jointId, .prismaticJoint) { $0.prismatic.targetTranslation = value }
    }

    public func prismaticJointTargetTranslation(_ jointId: JointID) -> Float? {
        jointSim(jointId, .prismaticJoint)?.prismatic.targetTranslation
    }
}

// MARK: - Wheel, spherical, motor, weld, parallel tuning

extension World {
    // MARK: Wheel

    @discardableResult
    public func enableWheelJointSpinMotor(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.enableSpinMotor = enable }
    }

    public func isWheelJointSpinMotorEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .wheelJoint)?.wheel.enableSpinMotor ?? false
    }

    @discardableResult
    public func setWheelJointSpinSpeed(_ jointId: JointID, _ speed: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.spinSpeed = speed }
    }

    public func wheelJointSpinSpeed(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.spinSpeed
    }

    @discardableResult
    public func setWheelJointMaxSpinTorque(_ jointId: JointID, _ torque: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.maxSpinTorque = torque }
    }

    public func wheelJointMaxSpinTorque(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.maxSpinTorque
    }

    public func wheelJointMotorTorque(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        return inv_h * sim.wheel.spinImpulse
    }

    @discardableResult
    public func enableWheelJointSuspensionSpring(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.enableSuspensionSpring = enable }
    }

    @discardableResult
    public func setWheelJointSuspensionSpring(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { sim in
            sim.wheel.suspensionHertz = hertz
            sim.wheel.suspensionDampingRatio = dampingRatio
        }
    }

    public func wheelJointSuspensionSpring(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        return (sim.wheel.suspensionHertz, sim.wheel.suspensionDampingRatio)
    }

    @discardableResult
    public func setWheelJointSuspensionLimits(_ jointId: JointID, _ lower: Float, _ upper: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { sim in
            sim.wheel.lowerSuspensionLimit = lower
            sim.wheel.upperSuspensionLimit = upper
        }
    }

    @discardableResult
    public func enableWheelJointSuspensionLimit(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.enableSuspensionLimit = enable }
    }

    public func wheelJointSuspensionLimits(_ jointId: JointID) -> (lower: Float, upper: Float)? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        return (sim.wheel.lowerSuspensionLimit, sim.wheel.upperSuspensionLimit)
    }

    @discardableResult
    public func enableWheelJointSteering(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.enableSteering = enable }
    }

    public func isWheelJointSteeringEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .wheelJoint)?.wheel.enableSteering ?? false
    }

    @discardableResult
    public func setWheelJointTargetSteeringAngle(_ jointId: JointID, _ angle: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.targetSteeringAngle = angle }
    }

    public func wheelJointTargetSteeringAngle(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.targetSteeringAngle
    }

    @discardableResult
    public func setWheelJointMaxSteeringTorque(_ jointId: JointID, _ torque: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.maxSteeringTorque = torque }
    }

    public func wheelJointMaxSteeringTorque(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.maxSteeringTorque
    }

    @discardableResult
    public func setWheelJointSteeringSpring(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { sim in
            sim.wheel.steeringHertz = hertz
            sim.wheel.steeringDampingRatio = dampingRatio
        }
    }

    public func wheelJointSteeringSpring(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        return (sim.wheel.steeringHertz, sim.wheel.steeringDampingRatio)
    }

    @discardableResult
    public func setWheelJointSteeringLimits(_ jointId: JointID, _ lower: Float, _ upper: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { sim in
            sim.wheel.lowerSteeringLimit = lower
            sim.wheel.upperSteeringLimit = upper
        }
    }

    @discardableResult
    public func enableWheelJointSteeringLimit(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.enableSteeringLimit = enable }
    }

    public func wheelJointSteeringLimits(_ jointId: JointID) -> (lower: Float, upper: Float)? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        return (sim.wheel.lowerSteeringLimit, sim.wheel.upperSteeringLimit)
    }

    // MARK: Spherical

    @discardableResult
    public func setSphericalJointConeLimit(_ jointId: JointID, _ coneAngle: Float) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.coneAngle = coneAngle }
    }

    @discardableResult
    public func enableSphericalJointConeLimit(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.enableConeLimit = enable }
    }

    public func sphericalJointConeLimit(_ jointId: JointID) -> (enabled: Bool, coneAngle: Float)? {
        guard let sim = jointSim(jointId, .sphericalJoint) else { return nil }
        return (sim.spherical.enableConeLimit, sim.spherical.coneAngle)
    }

    @discardableResult
    public func setSphericalJointTwistLimits(_ jointId: JointID, _ lower: Float, _ upper: Float) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { sim in
            sim.spherical.lowerTwistAngle = lower
            sim.spherical.upperTwistAngle = upper
        }
    }

    @discardableResult
    public func enableSphericalJointTwistLimit(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.enableTwistLimit = enable }
    }

    public func sphericalJointTwistLimits(_ jointId: JointID) -> (lower: Float, upper: Float)? {
        guard let sim = jointSim(jointId, .sphericalJoint) else { return nil }
        return (sim.spherical.lowerTwistAngle, sim.spherical.upperTwistAngle)
    }

    @discardableResult
    public func enableSphericalJointMotor(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.enableMotor = enable }
    }

    @discardableResult
    public func setSphericalJointMotorVelocity(_ jointId: JointID, _ velocity: Vec3) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.motorVelocity = velocity }
    }

    public func sphericalJointMotorVelocity(_ jointId: JointID) -> Vec3? {
        jointSim(jointId, .sphericalJoint)?.spherical.motorVelocity
    }

    @discardableResult
    public func setSphericalJointMaxMotorTorque(_ jointId: JointID, _ torque: Float) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.maxMotorTorque = torque }
    }

    public func sphericalJointMaxMotorTorque(_ jointId: JointID) -> Float? {
        jointSim(jointId, .sphericalJoint)?.spherical.maxMotorTorque
    }

    @discardableResult
    public func enableSphericalJointSpring(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.enableSpring = enable }
    }

    @discardableResult
    public func setSphericalJointSpring(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { sim in
            sim.spherical.hertz = hertz
            sim.spherical.dampingRatio = dampingRatio
        }
    }

    public func sphericalJointSpring(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId, .sphericalJoint) else { return nil }
        return (sim.spherical.hertz, sim.spherical.dampingRatio)
    }

    @discardableResult
    public func setSphericalJointTargetRotation(_ jointId: JointID, _ rotation: Quat) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.targetRotation = rotation }
    }

    public func sphericalJointTargetRotation(_ jointId: JointID) -> Quat? {
        jointSim(jointId, .sphericalJoint)?.spherical.targetRotation
    }

    // MARK: Motor

    @discardableResult
    public func setMotorJointLinearVelocity(_ jointId: JointID, _ velocity: Vec3) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.linearVelocity = velocity }
    }

    public func motorJointLinearVelocity(_ jointId: JointID) -> Vec3? {
        jointSim(jointId, .motorJoint)?.motor.linearVelocity
    }

    @discardableResult
    public func setMotorJointAngularVelocity(_ jointId: JointID, _ velocity: Vec3) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.angularVelocity = velocity }
    }

    public func motorJointAngularVelocity(_ jointId: JointID) -> Vec3? {
        jointSim(jointId, .motorJoint)?.motor.angularVelocity
    }

    @discardableResult
    public func setMotorJointMaxVelocityForce(_ jointId: JointID, _ force: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.maxVelocityForce = force }
    }

    public func motorJointMaxVelocityForce(_ jointId: JointID) -> Float? {
        jointSim(jointId, .motorJoint)?.motor.maxVelocityForce
    }

    @discardableResult
    public func setMotorJointMaxVelocityTorque(_ jointId: JointID, _ torque: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.maxVelocityTorque = torque }
    }

    public func motorJointMaxVelocityTorque(_ jointId: JointID) -> Float? {
        jointSim(jointId, .motorJoint)?.motor.maxVelocityTorque
    }

    @discardableResult
    public func setMotorJointLinearSpring(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { sim in
            sim.motor.linearHertz = hertz
            sim.motor.linearDampingRatio = dampingRatio
        }
    }

    public func motorJointLinearSpring(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId, .motorJoint) else { return nil }
        return (sim.motor.linearHertz, sim.motor.linearDampingRatio)
    }

    @discardableResult
    public func setMotorJointAngularSpring(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { sim in
            sim.motor.angularHertz = hertz
            sim.motor.angularDampingRatio = dampingRatio
        }
    }

    public func motorJointAngularSpring(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId, .motorJoint) else { return nil }
        return (sim.motor.angularHertz, sim.motor.angularDampingRatio)
    }

    @discardableResult
    public func setMotorJointMaxSpringForce(_ jointId: JointID, _ force: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.maxSpringForce = force }
    }

    public func motorJointMaxSpringForce(_ jointId: JointID) -> Float? {
        jointSim(jointId, .motorJoint)?.motor.maxSpringForce
    }

    @discardableResult
    public func setMotorJointMaxSpringTorque(_ jointId: JointID, _ torque: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.maxSpringTorque = torque }
    }

    public func motorJointMaxSpringTorque(_ jointId: JointID) -> Float? {
        jointSim(jointId, .motorJoint)?.motor.maxSpringTorque
    }

    // MARK: Weld

    @discardableResult
    public func setWeldJointLinearSpring(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId, .weldJoint) { sim in
            sim.weld.linearHertz = hertz
            sim.weld.linearDampingRatio = dampingRatio
        }
    }

    public func weldJointLinearSpring(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId, .weldJoint) else { return nil }
        return (sim.weld.linearHertz, sim.weld.linearDampingRatio)
    }

    @discardableResult
    public func setWeldJointAngularSpring(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId, .weldJoint) { sim in
            sim.weld.angularHertz = hertz
            sim.weld.angularDampingRatio = dampingRatio
        }
    }

    public func weldJointAngularSpring(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId, .weldJoint) else { return nil }
        return (sim.weld.angularHertz, sim.weld.angularDampingRatio)
    }

    // MARK: Parallel

    @discardableResult
    public func setParallelJointSpring(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId, .parallelJoint) { sim in
            sim.parallel.hertz = hertz
            sim.parallel.dampingRatio = dampingRatio
        }
    }

    public func parallelJointSpring(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId, .parallelJoint) else { return nil }
        return (sim.parallel.hertz, sim.parallel.dampingRatio)
    }

    @discardableResult
    public func setParallelJointMaxTorque(_ jointId: JointID, _ torque: Float) -> Bool {
        mutateJointSim(jointId, .parallelJoint) { $0.parallel.maxTorque = torque }
    }

    public func parallelJointMaxTorque(_ jointId: JointID) -> Float? {
        jointSim(jointId, .parallelJoint)?.parallel.maxTorque
    }
}

// MARK: - Joint inspection

extension World {
    /// Read any joint's sim regardless of type.
    func jointSim(_ jointId: JointID) -> JointSim? {
        let index = jointId.index1 - 1
        guard index >= 0, Int(index) < joints.count else { return nil }
        guard joints[Int(index)].jointId != B3.nullIndex else { return nil }
        return readJointSim(index)
    }

    /// Mutate any joint's sim regardless of type.
    @discardableResult
    func mutateJointSim(_ jointId: JointID, _ body: (inout JointSim) -> Void) -> Bool {
        let index = jointId.index1 - 1
        guard index >= 0, Int(index) < joints.count else { return false }
        guard joints[Int(index)].jointId != B3.nullIndex else { return false }
        var sim = readJointSim(index)
        body(&sim)
        writeJointSim(index, sim)
        return true
    }

    /// The joint record for an id.
    func joint(_ jointId: JointID) -> Joint {
        joints[Int(jointId.index1 - 1)]
    }

    func jointIndex(_ jointId: JointID) -> Int32 {
        jointId.index1 - 1
    }

    /// Is the joint awake? True when at least one attached body is awake.
    public func jointIsAwake(_ jointId: JointID) -> Bool {
        let joint = self.joint(jointId)
        guard joint.setIndex != B3.nullIndex else { return false }
        return joint.setIndex == SetType.awakeSet.rawValue
    }

    /// Wake both bodies attached to the joint.
    public func wakeJointBodies(_ jointId: JointID) {
        let joint = self.joint(jointId)
        wakeBody(joint.edges[0].bodyId)
        wakeBody(joint.edges[1].bodyId)
    }

    /// Is collision allowed between the connected bodies?
    public func jointCollideConnected(_ jointId: JointID) -> Bool {
        self.joint(jointId).collideConnected
    }

    /// Allow or prevent collision between the connected bodies.
    public func setJointCollideConnected(_ jointId: JointID, _ shouldCollide: Bool) {
        let index = jointIndex(jointId)
        var record = joints[Int(index)]
        record.collideConnected = shouldCollide
        joints[Int(index)] = record
    }

    /// The joint's constraint tuning.
    public func jointConstraintTuning(_ jointId: JointID) -> (hertz: Float, dampingRatio: Float)? {
        guard let sim = jointSim(jointId) else { return nil }
        return (sim.constraintHertz, sim.constraintDampingRatio)
    }

    /// Set the joint's constraint tuning.
    @discardableResult
    public func setJointConstraintTuning(_ jointId: JointID, hertz: Float, dampingRatio: Float) -> Bool {
        mutateJointSim(jointId) { sim in
            sim.constraintHertz = hertz
            sim.constraintDampingRatio = dampingRatio
        }
    }

    /// The force threshold for joint events.
    public func jointForceThreshold(_ jointId: JointID) -> Float? {
        jointSim(jointId)?.forceThreshold
    }

    /// Set the force threshold for joint events.
    @discardableResult
    public func setJointForceThreshold(_ jointId: JointID, _ threshold: Float) -> Bool {
        mutateJointSim(jointId) { $0.forceThreshold = threshold }
    }

    /// The torque threshold for joint events.
    public func jointTorqueThreshold(_ jointId: JointID) -> Float? {
        jointSim(jointId)?.torqueThreshold
    }

    /// Set the torque threshold for joint events.
    @discardableResult
    public func setJointTorqueThreshold(_ jointId: JointID, _ threshold: Float) -> Bool {
        mutateJointSim(jointId) { $0.torqueThreshold = threshold }
    }

    /// The local frame on body A.
    public func jointLocalFrameA(_ jointId: JointID) -> Transform? {
        jointSim(jointId)?.localFrameA
    }

    /// The local frame on body B.
    public func jointLocalFrameB(_ jointId: JointID) -> Transform? {
        jointSim(jointId)?.localFrameB
    }

    /// Set the local frame on body A.
    @discardableResult
    public func setJointLocalFrameA(_ jointId: JointID, _ frame: Transform) -> Bool {
        mutateJointSim(jointId) { $0.localFrameA = frame }
    }

    /// Set the local frame on body B.
    @discardableResult
    public func setJointLocalFrameB(_ jointId: JointID, _ frame: Transform) -> Bool {
        mutateJointSim(jointId) { $0.localFrameB = frame }
    }

    /// A world point on the joint's body A.
    public func jointWorldAnchorA(_ jointId: JointID) -> Pos? {
        guard let sim = jointSim(jointId) else { return nil }
        return transformWorldPoint(bodyTransformQuick(sim.bodyIdA), sim.localFrameA.p)
    }

    /// The current linear separation error. Zero for a coincident anchor.
    public func jointLinearSeparation(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId) else { return nil }
        let xfA = bodyTransformQuick(sim.bodyIdA)
        let xfB = bodyTransformQuick(sim.bodyIdB)
        let pA = transformWorldPoint(xfA, sim.localFrameA.p)
        let pB = transformWorldPoint(xfB, sim.localFrameB.p)

        switch jointType(jointId) {
        case .parallelJoint:
            return 0
        default:
            return length(subPos(pB, pA))
        }
    }

    /// The current angular separation error in radians.
    public func jointAngularSeparation(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId) else { return nil }
        let xfA = bodyTransformQuick(sim.bodyIdA)
        let xfB = bodyTransformQuick(sim.bodyIdB)
        let qA = mulQuat(xfA.q, sim.localFrameA.q)
        let qB = mulQuat(xfB.q, sim.localFrameB.q)
        let rel = invMulQuat(qA, qB)

        // The angle of the relative rotation, clamped for numerical safety.
        return 2 * atan2Approx(length(rel.v), absFloat(rel.s))
    }

    /// The current constraint force on the joint, in newtons.
    public func jointConstraintForce(_ jointId: JointID) -> Vec3? {
        guard let sim = jointSim(jointId) else { return nil }
        return jointConstraintForce(sim)
    }

    /// The current constraint torque on the joint, in newton-meters.
    public func jointConstraintTorque(_ jointId: JointID) -> Vec3? {
        guard let sim = jointSim(jointId) else { return nil }
        return jointConstraintTorque(sim)
    }

    private func jointConstraintForce(_ sim: JointSim) -> Vec3 {
        switch sim.type {
        case .parallelJoint, .filterJoint:
            return Vec3()
        case .gearJoint, .rackAndPinionJoint, .pulleyJoint, .sixDOFJoint, .pathJoint:
            return couplingJointForce(sim)
        case .distanceJoint:
            let joint = sim.distance
            let xfA = bodyTransformQuick(sim.bodyIdA)
            let xfB = bodyTransformQuick(sim.bodyIdB)
            let pA = transformWorldPoint(xfA, sim.localFrameA.p)
            let pB = transformWorldPoint(xfB, sim.localFrameB.p)
            let axis = normalize(subPos(pB, pA))
            let force =
                (joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse)
                * inv_h
            return force * axis
        case .motorJoint:
            return inv_h * (sim.motor.linearVelocityImpulse + sim.motor.linearSpringImpulse)
        case .prismaticJoint:
            let joint = sim.prismatic
            let xfA = bodyTransformQuick(sim.bodyIdA)
            let impulse = Vec3(
                joint.perpImpulse.x, joint.perpImpulse.y,
                joint.motorImpulse + joint.lowerImpulse + joint.upperImpulse + joint.springImpulse
            )
            // Convert the impulse from joint space to world.
            var force = inv_h * impulse
            force = rotateVector(mulQuat(xfA.q, sim.localFrameA.q), force)
            return force
        case .revoluteJoint:
            return inv_h * sim.revolute.linearImpulse
        case .sphericalJoint:
            return inv_h * sim.sphericalJoint.linearImpulse
        case .weldJoint:
            return inv_h * sim.weld.linearImpulse
        case .wheelJoint:
            let joint = sim.wheel
            let xfA = bodyTransformQuick(sim.bodyIdA)
            let impulse = Vec3(
                joint.linearImpulse.x, joint.linearImpulse.y,
                joint.lowerSuspensionLimit + joint.upperSuspensionImpulse
                    + joint.suspensionSpringImpulse
            )
            var force = inv_h * impulse
            force = rotateVector(mulQuat(xfA.q, sim.localFrameA.q), force)
            return force
        }
    }

    private func jointConstraintTorque(_ sim: JointSim) -> Vec3 {
        switch sim.type {
        case .distanceJoint, .filterJoint:
            return Vec3()
        case .gearJoint, .rackAndPinionJoint, .pulleyJoint, .sixDOFJoint, .pathJoint:
            return couplingJointTorque(sim)
        case .parallelJoint:
            return inv_h * Vec3(0, 0, sim.parallel.maxTorque)
        case .motorJoint:
            return inv_h * (sim.motor.angularVelocityImpulse + sim.motor.angularSpringImpulse)
        case .prismaticJoint:
            let xfA = bodyTransformQuick(sim.bodyIdA)
            let torque = rotateVector(
                mulQuat(xfA.q, sim.localFrameA.q), inv_h * sim.prismatic.angularImpulse)
            return torque
        case .revoluteJoint:
            let joint = sim.revolute
            let xfA = bodyTransformQuick(sim.bodyIdA)
            let axis = rotateVector(xfA.q, rotateVector(sim.localFrameA.q, .axisZ))

            let relQ = invMulQuat(joint.frameA.q, joint.frameB.q)

            // These are needed for warm starting.
            let perpAxisX =
                0.5 * rotateVector(
                    joint.frameA.q,
                    relQ.s * Vec3.axisX + cross(relQ.v, .axisX))
            let perpAxisY =
                0.5 * rotateVector(
                    joint.frameA.q,
                    relQ.s * Vec3.axisY + cross(relQ.v, .axisY))

            let axialImpulse =
                joint.springImpulse + joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse
            var angularImpulse =
                joint.perpImpulse.x * perpAxisX + joint.perpImpulse.y * perpAxisY
            angularImpulse = mulAdd(angularImpulse, axialImpulse, joint.rotationAxisZ)
            let impulse = mulAdd(angularImpulse, axialImpulse, axis)
            return inv_h * impulse
        case .sphericalJoint:
            let joint = sim.sphericalJoint
            let xfA = bodyTransformQuick(sim.bodyIdA)
            let xfB = bodyTransformQuick(sim.bodyIdB)
            let qA = mulQuat(xfA.q, sim.localFrameA.q)
            let qB = mulQuat(xfB.q, sim.localFrameB.q)

            // Cone axis is the z-axis of body A.
            let coneAxis = rotateVector(qA, .axisZ)
            let twistAxis = rotateVector(qB, .axisZ)
            let swingAxis = normalize(cross(coneAxis, twistAxis))

            var impulse = joint.springImpulse + joint.motorImpulse
            impulse = mulAdd(
                impulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, twistAxis)
            impulse = mulAdd(impulse, joint.swingImpulse, swingAxis)
            return inv_h * impulse
        case .weldJoint:
            return inv_h * sim.weld.angularImpulse
        case .wheelJoint:
            let simA = solverSets[Int(bodies[Int(sim.bodyIdA)].setIndex)]
                .bodySims[Int(bodies[Int(sim.bodyIdA)].localIndex)]
            let qA = mulQuat(simA.transform.q, sim.localFrameA.q)
            let matrixA = makeMatrixFromQuat(qA)
            return (inv_h * sim.wheel.spinImpulse) * matrixA.cz
        }
    }
}

// MARK: - Per-type scalar accessors

extension World {
    // MARK: Wheel

    public func enableWheelJointSuspension(_ jointId: JointID, _ enable: Bool) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.enableSuspensionSpring = enable }
    }

    public func isWheelJointSuspensionEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .wheelJoint)?.wheel.enableSuspensionSpring ?? false
    }

    public func isWheelJointSuspensionLimitEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .wheelJoint)?.wheel.enableSuspensionLimit ?? false
    }

    public func isWheelJointSteeringLimitEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .wheelJoint)?.wheel.enableSteeringLimit ?? false
    }

    public func wheelJointSuspensionHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.suspensionHertz
    }

    @discardableResult
    public func setWheelJointSuspensionHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.suspensionHertz = hertz }
    }

    public func wheelJointSuspensionDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.suspensionDampingRatio
    }

    @discardableResult
    public func setWheelJointSuspensionDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.suspensionDampingRatio = ratio }
    }

    public func wheelJointLowerSuspensionLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.lowerSuspensionLimit
    }

    public func wheelJointUpperSuspensionLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.upperSuspensionLimit
    }

    public func wheelJointSpinMotorSpeed(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.spinSpeed
    }

    @discardableResult
    public func setWheelJointSpinMotorSpeed(_ jointId: JointID, _ speed: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.spinSpeed = speed }
    }

    /// The wheel's angular speed about its spin axis, in radians per second.
    public func wheelJointSpinSpeedValue(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        let simB = solverSets[Int(bodies[Int(sim.bodyIdB)].setIndex)]
            .bodySims[Int(bodies[Int(sim.bodyIdB)].localIndex)]
        let quatB = mulQuat(simB.transform.q, sim.localFrameB.q)
        let spinAxis = rotateVector(quatB, .axisZ)

        let wA = bodyAngularVelocityAt(sim.bodyIdA)
        let wB = bodyAngularVelocityAt(sim.bodyIdB)
        return dot(wB - wA, spinAxis)
    }

    /// The motor torque applied about the spin axis, in newton-meters.
    public func wheelJointSpinTorqueValue(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        return inv_h * sim.wheel.spinImpulse
    }

    /// The steering angle in radians.
    public func wheelJointSteeringAngle(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        let simA = solverSets[Int(bodies[Int(sim.bodyIdA)].setIndex)]
            .bodySims[Int(bodies[Int(sim.bodyIdA)].localIndex)]
        let simB = solverSets[Int(bodies[Int(sim.bodyIdB)].setIndex)]
            .bodySims[Int(bodies[Int(sim.bodyIdB)].localIndex)]

        let quatA = mulQuat(simA.transform.q, sim.localFrameA.q)
        let quatB = mulQuat(simB.transform.q, sim.localFrameB.q)
        let matrixA = makeMatrixFromQuat(quatA)
        let matrixB = makeMatrixFromQuat(quatB)

        // Twist around the x-axis.
        let cs = dot(matrixB.cz, matrixA.cz)
        let ss = -dot(matrixB.cz, matrixA.cy)
        return atan2Approx(ss, cs)
    }

    /// The steering torque in newton-meters.
    public func wheelJointSteeringTorque(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .wheelJoint) else { return nil }
        return inv_h * sim.wheel.steeringSpringImpulse
    }

    public func wheelJointSteeringHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.steeringHertz
    }

    @discardableResult
    public func setWheelJointSteeringHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.steeringHertz = hertz }
    }

    public func wheelJointSteeringDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.steeringDampingRatio
    }

    @discardableResult
    public func setWheelJointSteeringDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .wheelJoint) { $0.wheel.steeringDampingRatio = ratio }
    }

    public func wheelJointLowerSteeringLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.lowerSteeringLimit
    }

    public func wheelJointUpperSteeringLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .wheelJoint)?.wheel.upperSteeringLimit
    }
}

// MARK: - Remaining per-type accessors

extension World {
    // MARK: Spherical

    public func sphericalJointConeAngle(_ jointId: JointID) -> Float? {
        jointSim(jointId, .sphericalJoint)?.spherical.coneAngle
    }

    public func isSphericalJointConeLimitEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .sphericalJoint)?.spherical.enableConeLimit ?? false
    }

    public func isSphericalJointTwistLimitEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .sphericalJoint)?.spherical.enableTwistLimit ?? false
    }

    public func isSphericalJointMotorEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .sphericalJoint)?.spherical.enableMotor ?? false
    }

    public func isSphericalJointSpringEnabled(_ jointId: JointID) -> Bool {
        jointSim(jointId, .sphericalJoint)?.spherical.enableSpring ?? false
    }

    public func sphericalJointLowerTwistLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .sphericalJoint)?.spherical.lowerTwistAngle
    }

    public func sphericalJointUpperTwistLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .sphericalJoint)?.spherical.upperTwistAngle
    }

    public func sphericalJointSpringHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .sphericalJoint)?.spherical.hertz
    }

    @discardableResult
    public func setSphericalJointSpringHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.hertz = hertz }
    }

    public func sphericalJointSpringDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .sphericalJoint)?.spherical.dampingRatio
    }

    @discardableResult
    public func setSphericalJointSpringDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .sphericalJoint) { $0.spherical.dampingRatio = ratio }
    }

    /// The current twist angle about the cone axis.
    public func sphericalJointTwistAngle(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .sphericalJoint) else { return nil }
        let xfA = bodyTransformQuick(sim.bodyIdA)
        let xfB = bodyTransformQuick(sim.bodyIdB)
        let qA = mulQuat(xfA.q, sim.localFrameA.q)
        let qB = mulQuat(xfB.q, sim.localFrameB.q)
        let rel = invMulQuat(qA, qB)
        return 2 * atan2Approx(length(rel.v), absFloat(rel.s))
    }

    /// The motor torque in newton-meters.
    public func sphericalJointMotorTorque(_ jointId: JointID) -> Vec3? {
        guard let sim = jointSim(jointId, .sphericalJoint) else { return nil }
        return inv_h * sim.sphericalJoint.motorImpulse
    }

    // MARK: Motor

    public func motorJointLinearHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .motorJoint)?.motor.linearHertz
    }

    @discardableResult
    public func setMotorJointLinearHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.linearHertz = hertz }
    }

    public func motorJointLinearDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .motorJoint)?.motor.linearDampingRatio
    }

    @discardableResult
    public func setMotorJointLinearDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.linearDampingRatio = ratio }
    }

    public func motorJointAngularHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .motorJoint)?.motor.angularHertz
    }

    @discardableResult
    public func setMotorJointAngularHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.angularHertz = hertz }
    }

    public func motorJointAngularDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .motorJoint)?.motor.angularDampingRatio
    }

    @discardableResult
    public func setMotorJointAngularDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .motorJoint) { $0.motor.angularDampingRatio = ratio }
    }

    // MARK: Weld

    public func weldJointLinearHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .weldJoint)?.weld.linearHertz
    }

    @discardableResult
    public func setWeldJointLinearHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .weldJoint) { $0.weld.linearHertz = hertz }
    }

    public func weldJointLinearDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .weldJoint)?.weld.linearDampingRatio
    }

    @discardableResult
    public func setWeldJointLinearDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .weldJoint) { $0.weld.linearDampingRatio = ratio }
    }

    public func weldJointAngularHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .weldJoint)?.weld.angularHertz
    }

    @discardableResult
    public func setWeldJointAngularHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .weldJoint) { $0.weld.angularHertz = hertz }
    }

    public func weldJointAngularDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .weldJoint)?.weld.angularDampingRatio
    }

    @discardableResult
    public func setWeldJointAngularDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .weldJoint) { $0.weld.angularDampingRatio = ratio }
    }

    // MARK: Parallel

    public func parallelJointHertz(_ jointId: JointID) -> Float? {
        jointSim(jointId, .parallelJoint)?.parallel.hertz
    }

    @discardableResult
    public func setParallelJointHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        mutateJointSim(jointId, .parallelJoint) { $0.parallel.hertz = hertz }
    }

    public func parallelJointDampingRatio(_ jointId: JointID) -> Float? {
        jointSim(jointId, .parallelJoint)?.parallel.dampingRatio
    }

    @discardableResult
    public func setParallelJointDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        mutateJointSim(jointId, .parallelJoint) { $0.parallel.dampingRatio = ratio }
    }
}

// MARK: - Limit and enable accessors

extension World {
    // MARK: Revolute




    public func revoluteJointLowerLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .revoluteJoint)?.revolute.lowerAngle
    }

    public func revoluteJointUpperLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .revoluteJoint)?.revolute.upperAngle
    }

    public func revoluteJointTargetAngle(_ jointId: JointID) -> Float? {
        jointSim(jointId, .revoluteJoint)?.revolute.targetAngle
    }

    @discardableResult
    public func setRevoluteJointTargetAngle(_ jointId: JointID, _ angle: Float) -> Bool {
        mutateJointSim(jointId, .revoluteJoint) { $0.revolute.targetAngle = angle }
    }

    // MARK: Prismatic




    public func prismaticJointLowerLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .prismaticJoint)?.prismatic.lowerTranslation
    }

    public func prismaticJointUpperLimit(_ jointId: JointID) -> Float? {
        jointSim(jointId, .prismaticJoint)?.prismatic.upperTranslation
    }

    /// The current translation along the axis.
    public func prismaticJointTranslationValue(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .prismaticJoint) else { return nil }
        let xfA = bodyTransformQuick(sim.bodyIdA)
        let xfB = bodyTransformQuick(sim.bodyIdB)

        let axisA = rotateVector(xfA.q, rotateVector(sim.localFrameA.q, .axisX))
        let anchorA = rotateVector(xfA.q, sim.localFrameA.p)
        let anchorB = rotateVector(xfB.q, sim.localFrameB.p)
        let d = (subPos(xfB.p, xfA.p) + (anchorB - anchorA))
        return dot(d, axisA)
    }

    /// The current translation rate along the axis.
    public func prismaticJointSpeedValue(_ jointId: JointID) -> Float? {
        guard let sim = jointSim(jointId, .prismaticJoint) else { return nil }
        let simA = solverSets[Int(bodies[Int(sim.bodyIdA)].setIndex)]
            .bodySims[Int(bodies[Int(sim.bodyIdA)].localIndex)]
        let simB = solverSets[Int(bodies[Int(sim.bodyIdB)].setIndex)]
            .bodySims[Int(bodies[Int(sim.bodyIdB)].localIndex)]

        let qA = simA.transform.q
        let qB = simB.transform.q

        let axisA = rotateVector(qA, rotateVector(sim.localFrameA.q, .axisX))
        let rA = rotateVector(qA, sim.localFrameA.p - simA.localCenter)
        let rB = rotateVector(qB, sim.localFrameB.p - simB.localCenter)

        let d = subPos(simB.center, simA.center) + (rB - rA)

        let vA = bodyLinearVelocityAt(sim.bodyIdA)
        let vB = bodyLinearVelocityAt(sim.bodyIdB)
        let wA = bodyAngularVelocityAt(sim.bodyIdA)
        let wB = bodyAngularVelocityAt(sim.bodyIdB)

        let vRel = (vB + cross(wB, rB)) - (vA + cross(wA, rA))

        // The axis moves with body A, so account for its rotation.
        return dot(d, cross(wA, axisA)) + dot(axisA, vRel)
    }

    // MARK: Distance



}

// MARK: - Parallel spring aliases

extension World {
    /// The parallel joint's spring frequency, matching C's `b3ParallelJoint_GetSpringHertz`.
    public func parallelJointSpringHertz(_ jointId: JointID) -> Float? {
        parallelJointHertz(jointId)
    }

    /// Set the parallel joint's spring frequency.
    @discardableResult
    public func setParallelJointSpringHertz(_ jointId: JointID, _ hertz: Float) -> Bool {
        setParallelJointHertz(jointId, hertz)
    }

    /// The parallel joint's spring damping ratio.
    public func parallelJointSpringDampingRatio(_ jointId: JointID) -> Float? {
        parallelJointDampingRatio(jointId)
    }

    /// Set the parallel joint's spring damping ratio.
    @discardableResult
    public func setParallelJointSpringDampingRatio(_ jointId: JointID, _ ratio: Float) -> Bool {
        setParallelJointDampingRatio(jointId, ratio)
    }
}

// MARK: - Joint reaction and events

extension World {
    /// The magnitude of a joint's reaction force and torque, matching C's `b3GetJointReaction`.
    func jointReaction(_ sim: JointSim, _ invTimeStep: Float) -> (force: Float, torque: Float) {
        var linearImpulse: Float = 0
        var angularImpulse: Float = 0

        switch sim.type {
        case .parallelJoint:
            let joint = sim.parallel
            let impulse = Vec3(joint.perpImpulse.x, joint.perpImpulse.y, 0)
            angularImpulse = length(impulse)

        case .distanceJoint:
            let joint = sim.distance
            linearImpulse = absFloat(
                joint.impulse + joint.lowerImpulse - joint.upperImpulse + joint.motorImpulse)

        case .motorJoint:
            let joint = sim.motor
            linearImpulse = length(joint.linearVelocityImpulse + joint.linearSpringImpulse)
            angularImpulse = length(
                joint.angularVelocityImpulse + joint.angularSpringImpulse)

        case .prismaticJoint:
            let joint = sim.prismatic
            let impulse = Vec3(
                joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse,
                joint.perpImpulse.x, joint.perpImpulse.y)
            linearImpulse = length(impulse)
            angularImpulse = length(joint.angularImpulse)

        case .revoluteJoint:
            let joint = sim.revolute
            linearImpulse = length(joint.linearImpulse)
            let impulse = Vec3(
                joint.perpImpulse.x, joint.perpImpulse.y,
                joint.motorImpulse + joint.lowerImpulse - joint.upperImpulse)
            angularImpulse = length(impulse)

        case .sphericalJoint:
            let joint = sim.sphericalJoint
            linearImpulse = length(joint.linearImpulse)

            let xfA = bodyTransformQuick(sim.bodyIdA)
            let xfB = bodyTransformQuick(sim.bodyIdB)
            let qA = mulQuat(xfA.q, sim.localFrameA.q)
            let qB = mulQuat(xfB.q, sim.localFrameB.q)

            // Cone axis is the z-axis of body A.
            let coneAxis = rotateVector(qA, .axisZ)
            let twistAxis = rotateVector(qB, .axisZ)
            let swingAxis = normalize(cross(coneAxis, twistAxis))

            var impulse = joint.springImpulse + joint.motorImpulse
            impulse = mulAdd(
                impulse, joint.lowerTwistImpulse - joint.upperTwistImpulse, twistAxis)
            impulse = mulAdd(impulse, joint.swingImpulse, swingAxis)
            angularImpulse = length(impulse)

        case .weldJoint:
            let joint = sim.weld
            linearImpulse = length(joint.linearImpulse)
            angularImpulse = length(joint.angularImpulse)

        case .wheelJoint:
            let joint = sim.wheel
            let axialImpulse =
                joint.suspensionSpringImpulse + joint.lowerSuspensionLimit
                - joint.upperSuspensionImpulse
            linearImpulse = (joint.linearImpulse.x * joint.linearImpulse.x
                + joint.linearImpulse.y * joint.linearImpulse.y
                + axialImpulse * axialImpulse).squareRoot()
            angularImpulse = absFloat(joint.spinImpulse)

        case .filterJoint:
            break

        case .gearJoint, .rackAndPinionJoint, .pulleyJoint, .sixDOFJoint, .pathJoint:
            (linearImpulse, angularImpulse) = couplingJointImpulses(sim)
        }

        return (linearImpulse * invTimeStep, angularImpulse * invTimeStep)
    }

    /// Collect joint events for joints whose reaction exceeds their threshold.
    func emitJointEvents(_ invTimeStep: Float) {
        jointEvents.removeAll()

        for index in 0..<Int32(joints.count) {
            let record = joints[Int(index)]
            if record.jointId == B3.nullIndex { continue }
            if record.setIndex != SetType.awakeSet.rawValue { continue }

            let sim = readJointSim(index)
            guard sim.forceThreshold < .greatestFiniteMagnitude
                || sim.torqueThreshold < .greatestFiniteMagnitude
            else { continue }

            let reaction = jointReaction(sim, invTimeStep)

            // A zero threshold reports every awake joint.
            if reaction.force >= sim.forceThreshold || reaction.torque >= sim.torqueThreshold {
                var event = JointEvent()
                event.jointId = JointID(
                    index1: index + 1, world0: worldId, generation: record.generation)
                event.userData = record.userData
                jointEvents.append(event)
            }
        }
    }

    /// Joint events from the most recent step.
    public func jointEventList() -> [JointEvent] {
        jointEvents
    }
}
