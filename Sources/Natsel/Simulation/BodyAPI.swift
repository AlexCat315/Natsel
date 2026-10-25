// Body API: transforms, velocities, forces, impulses, and mass properties (body.c).

extension World {
    // MARK: Helpers

    @inline(__always)
    func bodySimFor(_ id: BodyID) -> BodySim {
        let body = body(id)
        return solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]
    }

    @inline(__always)
    func setBodySim(_ id: BodyID, _ sim: BodySim) {
        let body = body(id)
        solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)] = sim
    }

    @inline(__always)
    func setBodyState(_ id: BodyID, _ state: BodyState) {
        let body = body(id)
        guard body.setIndex == SetType.awakeSet.rawValue else { return }
        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)] = state
    }

    @inline(__always)
    func withBodySim<R>(_ id: BodyID, _ body_: (inout BodySim) -> R) -> R {
        var sim = bodySimFor(id)
        let result = body_(&sim)
        setBodySim(id, sim)
        return result
    }

    @inline(__always)
    func withBodyState<R>(_ id: BodyID, _ body_: (inout BodyState) -> R) -> R? {
        guard let state = bodyState(body(id)) else { return nil }
        var s = state
        let result = body_(&s)
        setBodyState(id, s)
        return result
    }

    // MARK: Transforms

    public func bodyPosition(_ id: BodyID) -> Pos {
        bodySimFor(id).transform.p
    }

    public func bodyRotation(_ id: BodyID) -> Quat {
        bodySimFor(id).transform.q
    }

    public func bodyTransform(_ id: BodyID) -> WorldTransform {
        bodySimFor(id).transform
    }

    public func bodyLocalPoint(_ id: BodyID, _ worldPoint: Pos) -> Vec3 {
        invTransformWorldPoint(bodyTransform(id), worldPoint)
    }

    public func bodyWorldPoint(_ id: BodyID, _ localPoint: Vec3) -> Pos {
        transformWorldPoint(bodyTransform(id), localPoint)
    }

    public func bodyLocalVector(_ id: BodyID, _ worldVector: Vec3) -> Vec3 {
        invRotateVector(bodyTransform(id).q, worldVector)
    }

    public func bodyWorldVector(_ id: BodyID, _ localVector: Vec3) -> Vec3 {
        rotateVector(bodyTransform(id).q, localVector)
    }

    public func setBodyTransform(_ id: BodyID, _ position: Pos, _ rotation: Quat) {
        b3Assert(isValidPosition(position))
        b3Assert(isValidQuat(rotation))
        b3Assert(isValidBody(id))
        b3Assert(locked == false)

        var sim = bodySimFor(id)
        sim.transform.p = position
        sim.transform.q = rotation
        sim.center = transformWorldPoint(sim.transform, sim.localCenter)

        let rotationMatrix = makeMatrixFromQuat(sim.transform.q)
        sim.invInertiaWorld = mulMM(mulMM(rotationMatrix, sim.invInertiaLocal), transpose(rotationMatrix))

        sim.rotation0 = sim.transform.q
        sim.center0 = sim.center
        setBodySim(id, sim)

        let transform = sim.transform
        let speculativeDistance = B3.speculativeDistance

        let bodyIndex = bodyIndex(id)
        var shapeId = bodies[Int(bodyIndex)].headShapeId
        while shapeId != B3.nullIndex {
            var shape = shapes[Int(shapeId)]
            let aabb = computeFatShapeAABB(shape, transform, speculativeDistance)
            shape.aabb = aabb

            let fatAABB = fatAABBs[Int(shapeId)]
            if aabbContains(fatAABB, aabb) == false {
                let inflated = aabbInflate(aabb, shape.aabbMargin)
                fatAABBs[Int(shapeId)] = inflated

                // The body could be disabled
                if shape.proxyKey != B3.nullIndex {
                    broadPhase.moveProxy(shape.proxyKey, inflated)
                }
            }

            shapes[Int(shapeId)] = shape
            shapeId = shape.nextShapeId
        }
    }

    // MARK: Velocities

    public func bodyLinearVelocity(_ id: BodyID) -> Vec3 {
        bodyState(body(id))?.linearVelocity ?? Vec3()
    }

    public func bodyAngularVelocity(_ id: BodyID) -> Vec3 {
        bodyState(body(id))?.angularVelocity ?? Vec3()
    }

    public func setBodyLinearVelocity(_ id: BodyID, _ linearVelocity: Vec3) {
        b3Assert(isValidVec3(linearVelocity))

        var record = body(id)
        if record.type == .staticBody {
            return
        }

        // A non-zero velocity wakes the body first, then the state is written in its new set.
        if lengthSquared(linearVelocity) > 0 {
            wakeBody(bodyIndex(id))
            record = body(id)
        }

        guard record.setIndex == SetType.awakeSet.rawValue else {
            return
        }

        var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)]
        state.linearVelocity = linearVelocity
        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)] = state
    }

    public func setBodyAngularVelocity(_ id: BodyID, _ angularVelocity: Vec3) {
        b3Assert(isValidVec3(angularVelocity))

        var record = body(id)
        if record.type == .staticBody {
            return
        }

        if lengthSquared(angularVelocity) > 0 {
            wakeBody(bodyIndex(id))
            record = body(id)
        }

        guard record.setIndex == SetType.awakeSet.rawValue else {
            return
        }

        var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)]
        state.angularVelocity = angularVelocity
        solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(record.localIndex)] = state
    }

    public func bodyLocalPointVelocity(_ id: BodyID, _ localPoint: Vec3) -> Vec3 {
        let sim = bodySimFor(id)
        let r = rotateVector(sim.transform.q, localPoint - sim.localCenter)
        let state = bodyState(body(id)) ?? BodyState.identity
        return state.linearVelocity + cross(state.angularVelocity, r)
    }

    public func bodyWorldPointVelocity(_ id: BodyID, _ worldPoint: Pos) -> Vec3 {
        let sim = bodySimFor(id)
        let r = worldPoint - sim.center
        let state = bodyState(body(id)) ?? BodyState.identity
        return state.linearVelocity + cross(state.angularVelocity, r)
    }

    // MARK: Forces

    public func applyBodyForce(_ id: BodyID, _ force: Vec3, _ point: Pos, _ wake: Bool) {
        b3Assert(isValidVec3(force))
        let body = body(id)

        if wake && body.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeBody(bodyIndex(id))
        }

        if body.setIndex == SetType.awakeSet.rawValue {
            withBodySim(id) { sim in
                sim.force = sim.force + force
                sim.torque = sim.torque + cross(subPos(point, sim.center), force)
            }
        }
    }

    public func applyBodyForceToCenter(_ id: BodyID, _ force: Vec3, _ wake: Bool) {
        b3Assert(isValidVec3(force))
        let body = body(id)

        if wake && body.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeBody(bodyIndex(id))
        }

        if body.setIndex == SetType.awakeSet.rawValue {
            withBodySim(id) { sim in
                sim.force = sim.force + force
            }
        }
    }

    public func applyBodyTorque(_ id: BodyID, _ torque: Vec3, _ wake: Bool) {
        b3Assert(isValidVec3(torque))
        let body = body(id)

        if wake && body.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeBody(bodyIndex(id))
        }

        if body.setIndex == SetType.awakeSet.rawValue {
            withBodySim(id) { sim in
                sim.torque = sim.torque + torque
            }
        }
    }

    public func applyBodyLinearImpulse(_ id: BodyID, _ impulse: Vec3, _ point: Pos, _ wake: Bool) {
        b3Assert(isValidVec3(impulse))
        b3Assert(isValidPosition(point))
        let body = body(id)

        if wake && body.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeBody(bodyIndex(id))
        }

        if body.setIndex == SetType.awakeSet.rawValue {
            var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)]
            let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]

            state.linearVelocity = mulAdd(state.linearVelocity, sim.invMass, impulse)

            let maxLinearSpeed = maxLinearSpeed
            if lengthSquared(state.linearVelocity) > maxLinearSpeed * maxLinearSpeed {
                state.linearVelocity = maxLinearSpeed * normalize(state.linearVelocity)
            }

            let delta = mulMV(sim.invInertiaWorld, cross(subPos(point, sim.center), impulse))
            state.angularVelocity = state.angularVelocity + delta

            solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)] = state
        }
    }

    public func applyBodyLinearImpulseToCenter(_ id: BodyID, _ impulse: Vec3, _ wake: Bool) {
        b3Assert(isValidVec3(impulse))
        let body = body(id)

        if wake && body.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeBody(bodyIndex(id))
        }

        if body.setIndex == SetType.awakeSet.rawValue {
            var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)]
            let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]

            state.linearVelocity = mulAdd(state.linearVelocity, sim.invMass, impulse)

            let maxLinearSpeed = maxLinearSpeed
            if lengthSquared(state.linearVelocity) > maxLinearSpeed * maxLinearSpeed {
                state.linearVelocity = maxLinearSpeed * normalize(state.linearVelocity)
            }

            solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)] = state
        }
    }

    public func applyBodyAngularImpulse(_ id: BodyID, _ impulse: Vec3, _ wake: Bool) {
        b3Assert(isValidVec3(impulse))
        b3Assert(isValidBody(id))
        let body = body(id)

        if wake && body.setIndex >= SetType.firstSleepingSet.rawValue {
            wakeBody(bodyIndex(id))
        }

        if body.setIndex == SetType.awakeSet.rawValue {
            var state = solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)]
            let sim = solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)]

            let localImpulse = invRotateVector(sim.transform.q, impulse)
            let localAngularVelocityDelta = mulMV(sim.invInertiaLocal, localImpulse)
            state.angularVelocity = state.angularVelocity + rotateVector(sim.transform.q, localAngularVelocityDelta)

            solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)] = state
        }
    }

    // MARK: Mass properties

    public func bodyMass(_ id: BodyID) -> Float {
        body(id).mass
    }

    public func bodyLocalRotationalInertia(_ id: BodyID) -> Matrix3 {
        bodySimFor(id).inertia
    }

    public func bodyInverseMass(_ id: BodyID) -> Float {
        bodySimFor(id).invMass
    }

    public func bodyWorldInverseRotationalInertia(_ id: BodyID) -> Matrix3 {
        bodySimFor(id).invInertiaWorld
    }

    public func bodyLocalCenter(_ id: BodyID) -> Vec3 {
        bodySimFor(id).localCenter
    }

    public func bodyWorldCenter(_ id: BodyID) -> Pos {
        bodySimFor(id).center
    }

    public func bodyMassData(_ id: BodyID) -> MassData {
        let sim = bodySimFor(id)
        var massData = MassData()
        massData.mass = body(id).mass
        massData.center = sim.localCenter
        massData.inertia = sim.inertia
        return massData
    }

    public func setBodyMassData(_ id: BodyID, _ massData: MassData) {
        b3Assert(isValidFloat(massData.mass) && massData.mass >= 0)
        b3Assert(isValidMatrix3(massData.inertia))
        b3Assert(isValidVec3(massData.center))

        let bodyIndex = bodyIndex(id)
        var body = bodies[Int(bodyIndex)]
        var sim = bodySimFor(id)

        // Mass is no longer dirty
        body.flags &= ~BodyFlags.dirtyMass.rawValue
        bodies[Int(bodyIndex)] = body
        syncBodyFlags(bodyIndex)
        body = bodies[Int(bodyIndex)]

        body.mass = massData.mass
        bodies[Int(bodyIndex)] = body
        sim.localCenter = massData.center

        let oldCenter = sim.center
        let center = transformWorldPoint(sim.transform, massData.center)
        sim.center = center
        sim.center0 = center
        sim.invMass = body.mass > 0 ? 1 / body.mass : 0
        sim.inertia = massData.inertia
        setBodySim(id, sim)

        // Update center of mass velocity
        if let state = bodyState(body) {
            var newState = state
            let deltaLinear = cross(state.angularVelocity, subPos(sim.center, oldCenter))
            newState.linearVelocity = newState.linearVelocity + deltaLinear
            setBodyState(id, newState)
        }

        var newSim = bodySimFor(id)
        let det = det(newSim.inertia)
        b3Assert(det >= 0)

        if det > 0 {
            newSim.invInertiaLocal = invertT(newSim.inertia)

            let rotationMatrix = makeMatrixFromQuat(newSim.transform.q)
            newSim.invInertiaWorld = mulMM(
                mulMM(rotationMatrix, newSim.invInertiaLocal), transpose(rotationMatrix)
            )
        } else {
            newSim.invInertiaLocal = Matrix3()
            newSim.invInertiaWorld = Matrix3()
        }

        // Apply fixed rotation
        if (newSim.flags & BodyFlags.fixedRotation.rawValue) == BodyFlags.fixedRotation.rawValue {
            newSim.inertia = Matrix3()
            newSim.invInertiaLocal = Matrix3()
            newSim.invInertiaWorld = Matrix3()
        }

        setBodySim(id, newSim)
        updateIsotropicInertia(bodyIndex)

        // Update extents using supplied mass center.
        var updatedSim = bodySimFor(id)
        updatedSim.minExtent = B3.huge
        updatedSim.maxExtent = Vec3()

        var shapeId = bodies[Int(bodyIndex)].headShapeId
        while shapeId != B3.nullIndex {
            let shape = shapes[Int(shapeId)]
            let extent = computeShapeExtent(shape, massData.center)
            updatedSim.minExtent = minFloat(updatedSim.minExtent, extent.minExtent)
            updatedSim.maxExtent = maxVec3(updatedSim.maxExtent, extent.maxExtent)
            shapeId = shape.nextShapeId
        }

        setBodySim(id, updatedSim)

        // When the center of mass changes, any cached contact manifold becomes invalid.
        var edgeKey = bodies[Int(bodyIndex)].headContactKey
        while edgeKey != B3.nullIndex {
            let contactId = edgeKey >> 1
            let edgeIndex = Int(edgeKey & 1)

            var contact = contacts[Int(contactId)]
            contact.flags &= ~ContactFlags.relativeTransformValid.rawValue
            contacts[Int(contactId)] = contact

            edgeKey = contact.edges[edgeIndex].nextKey
        }
    }

    public func applyMassFromShapes(_ id: BodyID) {
        updateBodyMassData(bodyIndex(id))
    }

    // MARK: Mass computation

    func updateIsotropicInertia(_ bodyIndex: Int32) {
        var body = bodies[Int(bodyIndex)]
        var sim = bodySim(body)

        let inertia = sim.inertia
        let mean = (inertia.cx.x + inertia.cy.y + inertia.cz.z) / 3.0
        let dx = inertia.cx.x - mean
        let dy = inertia.cy.y - mean
        let dz = inertia.cz.z - mean
        let offDiagonal = inertia.cy.x * inertia.cy.x + inertia.cz.x * inertia.cz.x + inertia.cz.y * inertia.cz.y

        // This is the square of the Frobenius matrix norm.
        let deviation = dx * dx + dy * dy + dz * dz + 2.0 * offDiagonal

        let tolerance = isotropicInertiaTolerance

        if 2.0 * deviation > 3.0 * tolerance * tolerance * mean * mean {
            body.flags |= BodyFlags.enableGyroscopicTorque.rawValue
        } else {
            body.flags &= ~BodyFlags.enableGyroscopicTorque.rawValue
        }

        bodies[Int(bodyIndex)] = body
        sim = bodySim(body)
        solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)] = sim

        syncBodyFlags(bodyIndex)
    }

    func updateBodyMassData(_ bodyIndex: Int32) {
        var body = bodies[Int(bodyIndex)]

        // Mass is no longer dirty
        body.flags &= ~BodyFlags.dirtyMass.rawValue
        body.flags &= ~BodyFlags.enableGyroscopicTorque.rawValue
        bodies[Int(bodyIndex)] = body
        syncBodyFlags(bodyIndex)
        body = bodies[Int(bodyIndex)]

        // Compute mass data from shapes. Each shape has its own density.
        body.mass = 0
        var sim = bodySim(body)
        sim.inertia = Matrix3()
        sim.invMass = 0
        sim.invInertiaLocal = Matrix3()
        sim.invInertiaWorld = Matrix3()
        sim.localCenter = Vec3()
        sim.minExtent = B3.huge
        sim.maxExtent = Vec3()
        solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)] = sim

        if body.headShapeId == B3.nullIndex {
            bodies[Int(bodyIndex)] = body
            return
        }

        // Static and kinematic sims have zero mass.
        if body.type != .dynamicBody {
            sim.center = sim.transform.p
            sim.center0 = sim.center

            // Need extents for kinematic bodies for sleeping to work correctly.
            if body.type == .kinematicBody {
                var shapeId = body.headShapeId
                while shapeId != B3.nullIndex {
                    let s = shapes[Int(shapeId)]

                    let extent = computeShapeExtent(s, Vec3())
                    sim.minExtent = minFloat(sim.minExtent, extent.minExtent)
                    sim.maxExtent = maxVec3(sim.maxExtent, extent.maxExtent)

                    shapeId = s.nextShapeId
                }
            }

            solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)] = sim
            bodies[Int(bodyIndex)] = body
            return
        }

        let shapeCount = Int(body.shapeCount)
        var masses = [MassData](repeating: MassData(), count: shapeCount)

        // Accumulate mass over all shapes.
        var localCenter = Vec3()
        var shapeId = body.headShapeId
        var shapeIndex = 0
        while shapeId != B3.nullIndex {
            let s = shapes[Int(shapeId)]
            shapeId = s.nextShapeId

            if s.density == 0 {
                masses[shapeIndex] = MassData()
                shapeIndex += 1
                continue
            }

            let massData = computeShapeMass(s)
            body.mass += massData.mass
            localCenter = mulAdd(localCenter, massData.mass, massData.center)

            masses[shapeIndex] = massData
            shapeIndex += 1
        }

        // Compute center of mass.
        if body.mass > 0 {
            sim.invMass = 1 / body.mass
            localCenter = sim.invMass * localCenter
        }

        // Second loop to accumulate the rotational inertia about the center of mass
        for massData in masses {
            if massData.mass == 0 {
                continue
            }

            // Shift to center of mass. This is safe because it can only increase.
            let offset = localCenter - massData.center
            let inertia = addMM(massData.inertia, steiner(massData.mass, offset))
            sim.inertia = addMM(sim.inertia, inertia)
        }

        let determinant = det(sim.inertia)
        b3Assert(determinant >= 0)

        if determinant > 0 {
            // This call is faster than invert
            sim.invInertiaLocal = invertT(sim.inertia)

            let rotationMatrix = makeMatrixFromQuat(sim.transform.q)
            sim.invInertiaWorld = mulMM(mulMM(rotationMatrix, sim.invInertiaLocal), transpose(rotationMatrix))
        }

        // Move center of mass.
        let oldCenter = sim.center
        sim.localCenter = localCenter
        sim.center = transformWorldPoint(sim.transform, sim.localCenter)
        sim.center0 = sim.center

        // Update center of mass velocity
        if let state = bodyState(body) {
            var newState = state
            let deltaLinear = cross(state.angularVelocity, subPos(sim.center, oldCenter))
            newState.linearVelocity = newState.linearVelocity + deltaLinear
            solverSets[Int(SetType.awakeSet.rawValue)].bodyStates[Int(body.localIndex)] = newState
        }

        // Compute body extents relative to center of mass
        shapeId = body.headShapeId
        while shapeId != B3.nullIndex {
            let s = shapes[Int(shapeId)]

            let extent = computeShapeExtent(s, localCenter)
            sim.minExtent = minFloat(sim.minExtent, extent.minExtent)
            sim.maxExtent = maxVec3(sim.maxExtent, extent.maxExtent)

            shapeId = s.nextShapeId
        }

        // When the center of mass changes, any cached contact manifold becomes invalid.
        var edgeKey = body.headContactKey
        while edgeKey != B3.nullIndex {
            let contactId = edgeKey >> 1
            let edgeIndex = Int(edgeKey & 1)

            var contact = contacts[Int(contactId)]
            contact.flags &= ~ContactFlags.relativeTransformValid.rawValue
            contacts[Int(contactId)] = contact

            edgeKey = contact.edges[edgeIndex].nextKey
        }

        // Apply fixed rotation
        if (sim.flags & BodyFlags.fixedRotation.rawValue) == BodyFlags.fixedRotation.rawValue {
            sim.inertia = Matrix3()
            sim.invInertiaLocal = Matrix3()
            sim.invInertiaWorld = Matrix3()
        }

        solverSets[Int(body.setIndex)].bodySims[Int(body.localIndex)] = sim
        bodies[Int(bodyIndex)] = body

        updateIsotropicInertia(bodyIndex)
    }

    // MARK: Damping and gravity

    public func setBodyLinearDamping(_ id: BodyID, _ linearDamping: Float) {
        b3Assert(isValidFloat(linearDamping) && linearDamping >= 0)
        withBodySim(id) { $0.linearDamping = linearDamping }
    }

    public func bodyLinearDamping(_ id: BodyID) -> Float {
        bodySimFor(id).linearDamping
    }

    public func setBodyAngularDamping(_ id: BodyID, _ angularDamping: Float) {
        b3Assert(isValidFloat(angularDamping) && angularDamping >= 0)
        withBodySim(id) { $0.angularDamping = angularDamping }
    }

    public func bodyAngularDamping(_ id: BodyID) -> Float {
        bodySimFor(id).angularDamping
    }

    public func setBodyGravityScale(_ id: BodyID, _ gravityScale: Float) {
        b3Assert(isValidFloat(gravityScale))
        b3Assert(gravityScale >= 0)
        withBodySim(id) { $0.gravityScale = gravityScale }
    }

    public func bodyGravityScale(_ id: BodyID) -> Float {
        bodySimFor(id).gravityScale
    }

    // MARK: Sleep, safety, extents

    public func bodyIsAwake(_ id: BodyID) -> Bool {
        isBodyAwake(body(id))
    }

    public func bodyIsSleepEnabled(_ id: BodyID) -> Bool {
        (body(id).flags & BodyFlags.enableSleep.rawValue) != 0
    }

    public func setBodySleepThreshold(_ id: BodyID, _ sleepThreshold: Float) {
        b3Assert(isValidFloat(sleepThreshold) && sleepThreshold >= 0)
        var body = body(id)
        body.sleepThreshold = sleepThreshold
        bodies[Int(bodyIndex(id))] = body
    }

    public func bodySleepThreshold(_ id: BodyID) -> Float {
        body(id).sleepThreshold
    }

    public func setBodySafetyFactor(_ id: BodyID, _ safetyFactor: Float) {
        b3Assert(isValidFloat(safetyFactor) && safetyFactor >= 0)
        var body = body(id)
        body.safetyFactor = safetyFactor
        bodies[Int(bodyIndex(id))] = body
    }

    public func bodySafetyFactor(_ id: BodyID) -> Float {
        body(id).safetyFactor
    }

    /// Enable or disable sleeping for a body.
    public func enableBodySleep(_ id: BodyID, _ enableSleep: Bool) {
        var body = body(id)
        if enableSleep {
            body.flags |= BodyFlags.enableSleep.rawValue
        } else {
            body.flags &= ~BodyFlags.enableSleep.rawValue
        }
        bodies[Int(bodyIndex(id))] = body
        syncBodyFlags(bodyIndex(id))
    }

    /// Is sleeping enabled for this body?
    public func isBodySleepEnabled(_ id: BodyID) -> Bool {
        (body(id).flags & BodyFlags.enableSleep.rawValue) != 0
    }

    public func bodyMinExtent(_ id: BodyID) -> Float {
        bodySimFor(id).minExtent
    }

    public func bodyMaxExtent(_ id: BodyID) -> Vec3 {
        bodySimFor(id).maxExtent
    }

    public func bodyMaxExtentOrigin(_ id: BodyID) -> Pos {
        bodySimFor(id).center
    }
}

// MARK: - Contact flags

/// Contact flags (contact.h).
struct ContactFlags: OptionSet, BitwiseCopyable {
    var rawValue: UInt32

    init(rawValue: UInt32) {
        self.rawValue = rawValue
    }

    static let touching = ContactFlags(rawValue: 0x0000_0001)
    static let hitEvent = ContactFlags(rawValue: 0x0000_0002)
    static let enableContactEvents = ContactFlags(rawValue: 0x0000_0004)
    static let staticFlag = ContactFlags(rawValue: 0x0000_0008)
    static let recycle = ContactFlags(rawValue: 0x0000_0010)

    static let simTouching = ContactFlags(rawValue: 0x0001_0000)
    static let simDisjoint = ContactFlags(rawValue: 0x0002_0000)
    static let simStartedTouching = ContactFlags(rawValue: 0x0004_0000)
    static let simStoppedTouching = ContactFlags(rawValue: 0x0008_0000)
    static let simEnableHitEvent = ContactFlags(rawValue: 0x0010_0000)
    static let simEnablePreSolveEvents = ContactFlags(rawValue: 0x0020_0000)
    static let simMeshContact = ContactFlags(rawValue: 0x0040_0000)
    static let relativeTransformValid = ContactFlags(rawValue: 0x0080_0000)
    static let enableSpeculativePoints = ContactFlags(rawValue: 0x0100_0000)
}

extension World {
    /// Linear velocity by body index, bypassing id validation. For internal derived queries.
    func bodyLinearVelocityAt(_ bodyIndex: Int32) -> Vec3 {
        bodyState(bodies[Int(bodyIndex)])?.linearVelocity ?? Vec3()
    }

    /// Angular velocity by body index, bypassing id validation.
    func bodyAngularVelocityAt(_ bodyIndex: Int32) -> Vec3 {
        bodyState(bodies[Int(bodyIndex)])?.angularVelocity ?? Vec3()
    }
}
