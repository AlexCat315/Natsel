// Kinematic character controller. The character is a capsule moved by collide-and-slide against
// the world, outside the rigid body step. It detects ground and slopes, walks up stairs, sticks to
// the floor going down slopes and steps, rides moving platforms, pushes dynamic bodies, and can own
// a kinematic inner body so rigid bodies and queries see it.

public enum CharacterGroundState: Sendable, BitwiseCopyable {
    /// Standing on walkable ground.
    case onGround
    /// Touching ground that is too steep to stand on.
    case onSteepGround
    /// Touching something, but nothing below.
    case notSupported
    /// Touching nothing.
    case inAir
}

/// A shape touched by the character in the last update.
public struct CharacterContact: Sendable {
    public var shapeId = ShapeID()
    public var bodyId = BodyID()
    public var position = Pos()
    /// Points from the shape toward the character.
    public var normal = Vec3()
    /// Penetration depth with the padded capsule.
    public var depth: Float = 0
    public var userMaterialId: UInt64 = 0

    public init() {}
}

public struct CharacterDef {
    /// Position of the character's feet: the bottom of the capsule.
    public var position = Pos()
    public var up = Vec3(0, 1, 0)

    /// Capsule radius and total height (at least twice the radius).
    public var radius: Float = 0.3 * lengthUnitsPerMeter
    public var height: Float = 1.8 * lengthUnitsPerMeter

    /// Steepest walkable slope.
    public var maxSlopeAngle: Float = 50 * B3.degToRad

    /// Used to push dynamic bodies and to weigh down the ground.
    public var mass: Float = 70
    /// Largest force the character can push with (N).
    public var maxStrength: Float = 100

    /// Skin kept between the capsule and the world.
    public var characterPadding: Float = 0.02 * lengthUnitsPerMeter

    /// Contacts this far beyond the padding are still reported, so the character knows about the
    /// ground before it lands.
    public var predictiveContactDistance: Float = 0.1 * lengthUnitsPerMeter

    /// Collide and slide passes per update.
    public var maxCollisionIterations: Int32 = 5

    /// Highest step `extendedUpdate` walks up.
    public var stepUpHeight: Float = 0.4 * lengthUnitsPerMeter
    /// How far `extendedUpdate` looks down to stay on the floor.
    public var stepDownHeight: Float = 0.5 * lengthUnitsPerMeter

    /// When a stair step lands on the edge of the step, look this much farther ahead for its
    /// walkable top before giving up.
    public var stairStepForwardTest: Float = 0.15 * lengthUnitsPerMeter

    public var enableStairStepping = true
    public var enableStickToFloor = true

    /// Move with the body the character stands on.
    public var inheritGroundVelocity = true

    public var filter = QueryFilter.defaultFilter

    /// Create a kinematic capsule body that follows the character, so rigid bodies collide with it
    /// and queries find it. The character itself ignores it.
    public var createInnerBody = true
    public var innerBodyShape = ShapeDef()

    public var userData: UnsafeMutableRawPointer?

    public init() {}
}

/// A virtual character driven by `update` or `extendedUpdate` once per frame.
public final class CharacterVirtual {
    public unowned let world: World

    /// Position of the feet.
    public private(set) var position: Pos
    /// Facing rotation. The capsule is symmetric about `up`, so this only affects the inner body.
    public var rotation = Quat.identity

    /// Desired velocity. The update clips it against what the character touched.
    public var linearVelocity = Vec3()

    public private(set) var up: Vec3
    public private(set) var radius: Float
    public private(set) var height: Float
    public var maxSlopeAngle: Float {
        didSet { cosMaxSlope = computeCosSin(maxSlopeAngle).cosine }
    }
    public var mass: Float
    public var maxStrength: Float
    public var characterPadding: Float
    public var predictiveContactDistance: Float
    public var maxCollisionIterations: Int32
    public var stepUpHeight: Float
    public var stepDownHeight: Float
    public var stairStepForwardTest: Float
    public var enableStairStepping: Bool
    public var enableStickToFloor: Bool
    public var inheritGroundVelocity: Bool
    public var filter: QueryFilter
    public var userData: UnsafeMutableRawPointer?

    /// Return false to ignore a shape.
    public var contactFilter: ((ShapeID, BodyID) -> Bool)?

    public private(set) var groundState = CharacterGroundState.inAir
    public private(set) var groundNormal = Vec3()
    public private(set) var groundPosition = Pos()
    public private(set) var groundVelocity = Vec3()
    public private(set) var groundBodyId = BodyID()
    public private(set) var groundShapeId = ShapeID()
    public private(set) var groundUserMaterialId: UInt64 = 0
    public private(set) var contacts: [CharacterContact] = []
    public private(set) var innerBodyId = BodyID()

    private var cosMaxSlope: Float
    private var innerBodyShape: ShapeDef?
    private var planes: [CollisionPlane] = []
    private var planeContacts: [CharacterContact] = []

    public init?(world: World, _ def: CharacterDef) {
        guard def.radius > 0, def.height >= 2 * def.radius, isValidPosition(def.position),
            lengthSquared(def.up) > 0
        else { return nil }
        self.world = world
        position = def.position
        up = normalize(def.up)
        radius = def.radius
        height = def.height
        maxSlopeAngle = def.maxSlopeAngle
        cosMaxSlope = computeCosSin(def.maxSlopeAngle).cosine
        mass = def.mass
        maxStrength = def.maxStrength
        characterPadding = def.characterPadding
        predictiveContactDistance = def.predictiveContactDistance
        maxCollisionIterations = max(1, def.maxCollisionIterations)
        stepUpHeight = def.stepUpHeight
        stepDownHeight = def.stepDownHeight
        stairStepForwardTest = def.stairStepForwardTest
        enableStairStepping = def.enableStairStepping
        enableStickToFloor = def.enableStickToFloor
        inheritGroundVelocity = def.inheritGroundVelocity
        filter = def.filter
        userData = def.userData

        if def.createInnerBody {
            innerBodyShape = def.innerBodyShape
            createInnerBody()
        }
        refreshContacts()
    }

    private func createInnerBody() {
        guard let shapeDef = innerBodyShape else { return }
        var bodyDef = BodyDef()
        bodyDef.type = .kinematicBody
        bodyDef.position = position
        bodyDef.rotation = rotation
        bodyDef.enableSleep = false
        innerBodyId = world.createBody(bodyDef)
        _ = world.createCapsuleShape(innerBodyId, shapeDef, capsule(radius: radius))
    }

    /// Remove the inner body. Call before discarding the character if the world lives on.
    public func destroy() {
        if innerBodyId.isNull == false && world.bodyIsValid(innerBodyId) {
            world.destroyBody(innerBodyId)
        }
        innerBodyId = BodyID()
    }

    // MARK: Shape

    /// Capsule relative to the feet.
    private func capsule(radius r: Float) -> Capsule {
        Capsule(center1: radius * up, center2: (height - radius) * up, radius: r)
    }

    private var ignoreBody: Int32 {
        innerBodyId.isNull ? B3.nullIndex : world.bodyIndex(innerBodyId)
    }

    /// Change the capsule, for example to crouch. Fails, keeping the old shape, when the new shape
    /// would overlap the world by more than `maxPenetration`.
    @discardableResult
    public func setShape(radius newRadius: Float, height newHeight: Float, maxPenetration: Float = 0) -> Bool {
        guard newRadius > 0, newHeight >= 2 * newRadius else { return false }
        let shrink = maxPenetration + B3.linearSlop
        let testRadius = maxFloat(newRadius - shrink, 0.5 * newRadius)
        let points = [newRadius * up, (newHeight - newRadius) * up]
        let blocked = points.withUnsafeBufferPointer { buffer in
            world.overlapsIgnoring(position, ShapeProxy(points: buffer.baseAddress, count: 2, radius: testRadius), filter,
                                   ignoreBody: ignoreBody, accept: { self.accepts($0) })
        }
        guard blocked == false else { return false }
        radius = newRadius
        height = newHeight
        if innerBodyId.isNull == false {
            // Rebuild the inner body with the new capsule.
            destroy()
            createInnerBody()
        }
        refreshContacts()
        return true
    }

    /// Teleport the character.
    public func setPosition(_ newPosition: Pos) {
        guard isValidPosition(newPosition) else { return }
        position = newPosition
        if innerBodyId.isNull == false {
            world.setBodyTransform(innerBodyId, position, rotation)
        }
        refreshContacts()
    }

    public var isSupported: Bool { groundState == .onGround || groundState == .onSteepGround }

    /// True when `normal` is walkable.
    public func isSlopeWalkable(_ normal: Vec3) -> Bool { dot(normal, up) >= cosMaxSlope }

    // MARK: Update

    /// Move by `linearVelocity * dt` with collide and slide, then refresh ground contact.
    public func update(_ dt: Float) {
        guard dt > 0 else { return }
        var displacement = dt * linearVelocity
        if inheritGroundVelocity && groundState == .onGround {
            displacement = mulAdd(displacement, dt, groundVelocity)
        }

        let start = position
        let wasOnGround = groundState == .onGround
        moveAndSlide(displacement, onGround: wasOnGround)

        // Do not keep velocity into what stopped us.
        linearVelocity = clipVector(linearVelocity, planes)

        refreshContacts()
        pushBodies(dt, start)
        followInnerBody(dt)
    }

    /// `update` plus stair walking and sticking to the floor. `gravity` gives the down direction
    /// and is used to weigh down the ground body; the caller adds it to `linearVelocity` as usual.
    public func extendedUpdate(_ dt: Float, gravity: Vec3) {
        guard dt > 0 else { return }
        let start = position
        let wasOnGround = groundState == .onGround
        let desired = dt * linearVelocity
        let desiredHorizontal = desired - dot(desired, up) * up

        update(dt)

        // Walk stairs when the horizontal motion was blocked by something steep.
        if enableStairStepping && wasOnGround && lengthSquared(desiredHorizontal) > 1.0e-10 {
            let moved = position - start
            let achieved = dot(moved - dot(moved, up) * up, normalize(desiredHorizontal))
            let wanted = length(desiredHorizontal)
            if achieved < 0.9 * wanted && touchesSteepContact(against: desiredHorizontal) {
                walkStairs(from: start, horizontal: desiredHorizontal, achieved: achieved, dt: dt)
            }
        }

        // Stick to the floor when walking off a step or down a slope.
        if enableStickToFloor && wasOnGround && groundState != .onGround && dot(linearVelocity, up) <= 1.0e-3 {
            stickToFloor()
        }

        // Weigh down a dynamic ground body.
        if groundState == .onGround && groundBodyId.isNull == false && world.bodyIsValid(groundBodyId)
            && world.bodyType(groundBodyId) == .dynamicBody
        {
            let weight = dt * mass * dot(gravity, -groundNormal)
            if weight > 0 {
                world.applyBodyLinearImpulse(groundBodyId, -weight * groundNormal, groundPosition, true)
            }
        }
        followInnerBody(dt)
    }

    // MARK: Collide and slide

    private func accepts(_ shapeIndex: Int32) -> Bool {
        guard let contactFilter else { return true }
        let shape = world.shapes[Int(shapeIndex)]
        let shapeId = ShapeID(index1: shape.id + 1, world0: world.worldId, generation: shape.generation)
        return contactFilter(shapeId, world.makeBodyID(shape.bodyId))
    }

    /// Collision planes of the padded capsule placed at `target`.
    private func gatherPlanes(at target: Pos, extra: Float, groundedSlopes: Bool) {
        planes.removeAll(keepingCapacity: true)
        planeContacts.removeAll(keepingCapacity: true)
        let mover = capsule(radius: radius + characterPadding + extra)
        world.collideMoverIgnoring(target, mover, filter, ignoreBody: ignoreBody) { shapeIndex, results in
            guard self.accepts(shapeIndex) else { return }
            let shape = self.world.shapes[Int(shapeIndex)]
            for result in results {
                var plane = CollisionPlane()
                plane.plane = result.plane
                plane.plane.offset -= extra
                plane.pushLimit = .greatestFiniteMagnitude
                plane.clipVelocity = true

                // A slope too steep to walk acts as a wall for a grounded character.
                let nUp = dot(result.plane.normal, self.up)
                if groundedSlopes && nUp > 0 && nUp < self.cosMaxSlope {
                    let horizontal = result.plane.normal - nUp * self.up
                    let len = length(horizontal)
                    if len > 1.0e-4 {
                        plane.plane.normal = (1 / len) * horizontal
                        plane.plane.offset /= len
                    }
                }
                self.planes.append(plane)

                var contact = CharacterContact()
                contact.shapeId = ShapeID(index1: shape.id + 1, world0: self.world.worldId, generation: shape.generation)
                contact.bodyId = self.world.makeBodyID(shape.bodyId)
                contact.position = offsetPos(target, result.point)
                contact.normal = result.plane.normal
                contact.depth = result.plane.offset - extra
                contact.userMaterialId = shape.material(result.materialIndex).userMaterialId
                self.planeContacts.append(contact)
            }
        }
    }

    /// Cast the capsule. Returns the free fraction of `translation`, backed off by the padding.
    private func castFraction(from origin: Pos, _ translation: Vec3) -> (fraction: Float, hit: QueryHit) {
        let len = length(translation)
        guard len > 1.0e-7 else { return (1, QueryHit()) }
        let c = capsule(radius: radius)
        let points = [c.center1, c.center2]
        let hit = points.withUnsafeBufferPointer { buffer in
            world.closestShapeCastHit(
                origin, ShapeProxy(points: buffer.baseAddress, count: 2, radius: radius), translation, filter,
                ignoreBody: ignoreBody, canEncroach: true,
                accept: { _ in true })
        }
        guard hit.hit else { return (1, hit) }
        let backed = maxFloat(0, hit.fraction - characterPadding / len)
        return (backed, hit)
    }

    private func moveAndSlide(_ displacement: Vec3, onGround: Bool) {
        var target = offsetPos(position, displacement)
        let tolerance = 0.1 * B3.linearSlop
        for _ in 0..<Int(maxCollisionIterations) {
            gatherPlanes(at: target, extra: 0, groundedSlopes: onGround)
            let solved = solvePlanes(Vec3(), &planes)
            let corrected = offsetPos(target, solved.delta)
            let translation = subPos(corrected, position)
            let (fraction, _) = castFraction(from: position, translation)
            let delta = fraction * translation
            position = offsetPos(position, delta)
            target = corrected
            if lengthSquared(delta) < tolerance * tolerance || fraction == 1 && lengthSquared(subPos(target, position)) < tolerance * tolerance {
                break
            }
        }
    }

    // MARK: Ground

    /// Contacts and ground state at the current position.
    private func refreshContacts() {
        gatherPlanes(at: position, extra: predictiveContactDistance, groundedSlopes: false)
        // Velocity clipping uses only touching planes.
        planes.removeAll { $0.plane.offset < -B3.linearSlop }
        contacts = planeContacts

        var bestUp: Float = -1
        var best = -1
        var supportSum = Vec3()
        var touching = false
        for (i, contact) in contacts.enumerated() where contact.depth > -(characterPadding + B3.linearSlop) {
            touching = true
            let nUp = dot(contact.normal, up)
            if nUp > 0 { supportSum += contact.normal }
            if nUp > bestUp {
                bestUp = nUp
                best = i
            }
        }

        groundBodyId = BodyID()
        groundShapeId = ShapeID()
        groundVelocity = Vec3()
        groundUserMaterialId = 0
        guard touching else {
            groundState = .inAir
            groundNormal = Vec3()
            return
        }
        guard best >= 0, bestUp > 1.0e-3 else {
            groundState = .notSupported
            groundNormal = Vec3()
            return
        }

        let ground = contacts[best]
        // Two steep slopes facing each other still hold the character.
        let walkable = bestUp >= cosMaxSlope
            || (lengthSquared(supportSum) > 0 && dot(normalize(supportSum), up) >= cosMaxSlope)
        groundState = walkable ? .onGround : .onSteepGround
        groundNormal = walkable && bestUp < cosMaxSlope ? normalize(supportSum) : ground.normal
        groundPosition = ground.position
        groundBodyId = ground.bodyId
        groundShapeId = ground.shapeId
        groundUserMaterialId = ground.userMaterialId
        if world.bodyIsValid(ground.bodyId) {
            groundVelocity = world.pointVelocity(world.bodyIndex(ground.bodyId), ground.position)
        }
    }

    private func touchesSteepContact(against horizontal: Vec3) -> Bool {
        let direction = normalize(horizontal)
        return contacts.contains { contact in
            contact.depth > -(characterPadding + 2 * B3.linearSlop) && dot(contact.normal, up) < cosMaxSlope
                && dot(contact.normal, direction) < -0.1
        }
    }

    /// Up, forward, down. Accepts the result when it lands on walkable ground and gets farther.
    private func walkStairs(from start: Pos, horizontal: Vec3, achieved: Float, dt: Float) {
        let saved = (position, linearVelocity, groundState)
        position = start

        // Up, stopping at the ceiling.
        let rise = castFraction(from: position, stepUpHeight * up).fraction * stepUpHeight
        guard rise > B3.linearSlop else {
            (position, linearVelocity, groundState) = saved
            refreshContacts()
            return
        }
        position = mulAdd(position, rise, up)

        // Forward.
        moveAndSlide(horizontal, onGround: false)

        // Down onto the step.
        let drop = rise + characterPadding + B3.linearSlop
        let (fraction, hit) = castFraction(from: position, -drop * up)
        let direction = normalize(horizontal)
        let moved = position - start
        let progress = dot(moved - dot(moved, up) * up, direction)
        guard hit.hit, progress > achieved + B3.linearSlop else {
            (position, linearVelocity, groundState) = saved
            refreshContacts()
            return
        }

        var supported = isSlopeWalkable(hit.normal)
        if supported == false && stairStepForwardTest > 0 {
            // Landed on the edge of the step: the top must be walkable a little farther ahead.
            let ahead = mulAdd(position, stairStepForwardTest, direction)
            let blocked = castFraction(from: position, stairStepForwardTest * direction).fraction < 1
            if blocked == false {
                let (_, test) = castFraction(from: ahead, -drop * up)
                supported = test.hit && isSlopeWalkable(test.normal)
            }
        }
        guard supported else {
            (position, linearVelocity, groundState) = saved
            refreshContacts()
            return
        }
        position = mulAdd(position, -fraction * drop, up)
        refreshContacts()
        if groundState == .onSteepGround {
            // Standing on the step's edge counts as ground; the next move finishes the step.
            groundState = .onGround
            groundNormal = up
        }
    }

    private func stickToFloor() {
        let (fraction, hit) = castFraction(from: position, -stepDownHeight * up)
        guard hit.hit, isSlopeWalkable(hit.normal) else { return }
        position = mulAdd(position, -fraction * stepDownHeight, up)
        refreshContacts()
    }

    // MARK: Interaction

    /// Push dynamic bodies the character walked into, limited by `maxStrength`.
    private func pushBodies(_ dt: Float, _ start: Pos) {
        let velocity = (1 / dt) * subPos(position, start) + linearVelocity
        let maxImpulse = maxStrength * dt
        for contact in contacts where contact.depth > -(characterPadding + B3.linearSlop) {
            guard world.bodyIsValid(contact.bodyId), world.bodyType(contact.bodyId) == .dynamicBody else { continue }
            let bodyVelocity = world.pointVelocity(world.bodyIndex(contact.bodyId), contact.position)
            let approach = dot(velocity - bodyVelocity, -contact.normal)
            guard approach > 0 else { continue }
            let impulse = minFloat(mass * approach, maxImpulse)
            world.applyBodyLinearImpulse(contact.bodyId, -impulse * contact.normal, contact.position, true)
        }
    }

    private func followInnerBody(_ dt: Float) {
        guard innerBodyId.isNull == false, world.bodyIsValid(innerBodyId) else { return }
        world.setBodyTargetTransform(innerBodyId, WorldTransform(position, rotation), timeStep: dt)
    }
}
