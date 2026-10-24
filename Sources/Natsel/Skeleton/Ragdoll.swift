// Ragdolls built from a skeleton (Jolt's RagdollSettings and Ragdoll). Each skeleton joint gets one
// body whose origin and rotation are the joint's model transform, and every joint with a parent is
// connected to it by a six DOF joint: swing-twist, hinge, or fixed. A ragdoll can be posed
// directly, driven to an animated pose with joint motors (it stays physical and reacts to hits),
// or driven kinematically by velocities.

/// Collision geometry of a ragdoll part, in the part's joint space.
public enum RagdollShape: Sendable {
    case capsule(Capsule)
    case sphere(Sphere)
    case box(halfExtents: Vec3, transform: Transform = .identity)
    case hull(Hull)
}

/// Connection of a part to its parent. Angles in radians; the twist axis is x of `jointFrame`.
public enum RagdollJointKind: Sendable {
    /// Twist about x within [twistMin, twistMax], and swing of x inside a cone.
    case swingTwist(coneAngle: Float, twistMin: Float, twistMax: Float)
    /// Twist about x, and separate swing limits about y and z.
    case pyramid(twist: ClosedRange<Float>, swingY: ClosedRange<Float>, swingZ: ClosedRange<Float>)
    /// Rotation about x only.
    case hinge(lower: Float, upper: Float)
    case fixed
}

public struct RagdollPart {
    public var shapes: [RagdollShape] = []
    /// Density, friction, filter, and events for the part's shapes.
    public var shapeDef = ShapeDef()
    /// Damping, gravity scale, sleep, bullet, and so on. Type, position, and rotation are set by
    /// the ragdoll.
    public var bodyDef = BodyDef()
    /// Ignored for root parts.
    public var joint = RagdollJointKind.swingTwist(coneAngle: 0.5, twistMin: -0.3, twistMax: 0.3)
    /// Joint frame in the part's joint space; its x axis is the twist axis.
    public var jointFrame = Transform.identity
    /// Torque resisting rotation in the joint (N m).
    public var jointFriction: Float = 0
    /// Soft limits; zero hertz is rigid.
    public var limitHertz: Float = 0
    public var limitDampingRatio: Float = 0

    public init() {}

    public init(_ shapes: [RagdollShape], joint: RagdollJointKind = .fixed, jointFrame: Transform = .identity) {
        self.shapes = shapes
        self.joint = joint
        self.jointFrame = jointFrame
    }
}

public enum RagdollSelfCollision: Sendable {
    /// Parts never collide with each other (a shared negative collision group).
    case none
    /// Only parts connected by a joint skip collision.
    case skipConnected
    /// Also skip parts whose bounds overlap in the creation pose, enlarged by `margin`, like Jolt's
    /// `DisableParentChildCollisions`.
    case skipOverlapping(margin: Float)
    /// Every pair collides, connected ones too.
    case all
}

public struct RagdollSettings {
    public var skeleton: Skeleton
    /// One part per skeleton joint, in skeleton order.
    public var parts: [RagdollPart]
    public var selfCollision = RagdollSelfCollision.skipOverlapping(margin: 0.02)
    /// Negative group used by `.none`.
    public var collisionGroup: Int32 = -1
    /// Limit the mass ratio across each joint (Jolt's Stabilize). Very light parts hanging from
    /// heavy ones, or the reverse, make joint chains converge slowly. Zero disables it.
    public var maxMassRatio: Float = 4

    public init(skeleton: Skeleton, parts: [RagdollPart]) {
        self.skeleton = skeleton
        self.parts = parts
    }
}

public final class Ragdoll {
    public let settings: RagdollSettings
    public let world: World
    /// One body per skeleton joint.
    public private(set) var bodies: [BodyID] = []
    /// The joint connecting each part to its parent; null for roots.
    public private(set) var joints: [JointID] = []
    /// Joints that only disable collision between overlapping parts.
    public private(set) var filterJoints: [JointID] = []
    private var motorsEnabled = false

    /// Create the parts at `pose`'s joint matrices. Returns nil when the part count does not
    /// match the skeleton.
    public init?(world: World, settings: RagdollSettings, pose: SkeletonPose, userData: UnsafeMutableRawPointer? = nil) {
        let count = settings.skeleton.jointCount
        guard settings.parts.count == count, pose.jointCount == count, count > 0 else { return nil }
        self.world = world
        self.settings = settings

        for i in 0..<count {
            let part = settings.parts[i]
            var def = part.bodyDef
            def.type = .dynamicBody
            let transform = pose.worldTransform(i)
            def.position = transform.p
            def.rotation = transform.q
            def.userData = userData
            let body = world.createBody(def)
            var shapeDef = part.shapeDef
            if case .none = settings.selfCollision { shapeDef.filter.groupIndex = settings.collisionGroup }
            for shape in part.shapes {
                switch shape {
                case .capsule(let capsule): _ = world.createCapsuleShape(body, shapeDef, capsule)
                case .sphere(let sphere): _ = world.createSphereShape(body, shapeDef, sphere)
                case .box(let half, let transform):
                    _ = world.createHullShape(body, shapeDef, makeTransformedBoxHull(half.x, half.y, half.z, transform))
                case .hull(let hull): _ = world.createHullShape(body, shapeDef, hull)
                }
            }
            bodies.append(body)
        }

        if settings.maxMassRatio > 1 { stabilizeMasses(settings.maxMassRatio) }

        let collideConnected: Bool
        if case .all = settings.selfCollision { collideConnected = true } else { collideConnected = false }
        for i in 0..<count {
            let parent = settings.skeleton.joints[i].parentIndex
            guard parent >= 0 else {
                joints.append(JointID())
                continue
            }
            joints.append(createJoint(i, parent, pose, collideConnected))
        }

        if case .skipOverlapping(let margin) = settings.selfCollision {
            createFilterJoints(margin)
        }
    }

    // MARK: Construction

    private func createJoint(_ child: Int, _ parent: Int, _ pose: SkeletonPose, _ collideConnected: Bool) -> JointID {
        let part = settings.parts[child]
        var def = SixDOFJointDef()
        def.base.bodyIdA = bodies[parent]
        def.base.bodyIdB = bodies[child]
        def.base.collideConnected = collideConnected
        def.base.localFrameB = part.jointFrame
        def.base.localFrameA = mulTransforms(invMulTransforms(pose.jointMatrices[parent], pose.jointMatrices[child]),
                                             part.jointFrame)
        def.lockAll(except: [.rotationX, .rotationY, .rotationZ])

        func limited(_ range: ClosedRange<Float>) -> SixDOFAxisSettings {
            var axis = SixDOFAxisSettings(.limited, lower: range.lowerBound, upper: range.upperBound)
            axis.limitHertz = part.limitHertz
            axis.limitDampingRatio = part.limitDampingRatio
            return axis
        }
        switch part.joint {
        case .swingTwist(let cone, let twistMin, let twistMax):
            def[.rotationX] = limited(twistMin...max(twistMin, twistMax))
            if cone > 0 {
                def[.rotationY] = limited(-cone...cone)
                def[.rotationZ] = limited(-cone...cone)
                def.swingLimit = .cone(angle: cone)
            } else {
                def[.rotationY].motion = .locked
                def[.rotationZ].motion = .locked
            }
        case .pyramid(let twist, let swingY, let swingZ):
            def[.rotationX] = limited(twist)
            def[.rotationY] = limited(swingY)
            def[.rotationZ] = limited(swingZ)
        case .hinge(let lower, let upper):
            def[.rotationX] = limited(lower...max(lower, upper))
            def[.rotationY].motion = .locked
            def[.rotationZ].motion = .locked
        case .fixed:
            def.lockAll()
        }
        for axis in [SixDOFAxis.rotationX, .rotationY, .rotationZ] {
            def[axis].maxFriction = part.jointFriction
        }
        return world.createSixDOFJoint(def)
    }

    /// Clamp the mass ratio across each joint, parents first, then restore the total mass.
    /// Inertia scales with mass, so each part keeps its shape of inertia.
    private func stabilizeMasses(_ maxRatio: Float) {
        var masses = bodies.map { world.bodyMassData($0) }
        let total = masses.reduce(0) { $0 + $1.mass }
        guard total > 0 else { return }
        var changed = false
        for i in 0..<bodies.count {
            let parent = settings.skeleton.joints[i].parentIndex
            guard parent >= 0, masses[parent].mass > 0, masses[i].mass > 0 else { continue }
            let ratio = masses[i].mass / masses[parent].mass
            let clamped = clampFloat(ratio, 1 / maxRatio, maxRatio)
            guard clamped != ratio else { continue }
            let scale = clamped / ratio
            masses[i].mass *= scale
            masses[i].inertia = mulSM(scale, masses[i].inertia)
            changed = true
        }
        guard changed else { return }
        let scale = total / masses.reduce(0) { $0 + $1.mass }
        for i in 0..<bodies.count {
            masses[i].mass *= scale
            masses[i].inertia = mulSM(scale, masses[i].inertia)
            world.setBodyMassData(bodies[i], masses[i])
        }
    }

    /// Filter joints between unconnected parts whose bounds overlap.
    private func createFilterJoints(_ margin: Float) {
        let boxes = bodies.map { aabbInflate(world.bodyComputeAABB($0), margin) }
        let skeleton = settings.skeleton
        for i in 0..<bodies.count {
            for j in (i + 1)..<bodies.count {
                if skeleton.joints[j].parentIndex == i || skeleton.joints[i].parentIndex == j { continue }
                guard aabbOverlaps(boxes[i], boxes[j]) else { continue }
                var def = FilterJointDef()
                def.base.bodyIdA = bodies[i]
                def.base.bodyIdB = bodies[j]
                filterJoints.append(world.createFilterJoint(def))
            }
        }
    }

    /// Destroy every body and joint of the ragdoll.
    public func destroy() {
        for body in bodies where world.bodyIsValid(body) { world.destroyBody(body) }
        bodies.removeAll()
        joints.removeAll()
        filterJoints.removeAll()
    }

    // MARK: Pose

    public var rootBody: BodyID { bodies[0] }

    /// Current pose, with the root body's position as the root offset.
    public func pose() -> SkeletonPose {
        var pose = SkeletonPose(skeleton: settings.skeleton)
        pose.rootOffset = world.bodyPosition(bodies[0])
        for i in 0..<bodies.count {
            pose.setWorldTransform(i, world.bodyTransform(bodies[i]))
        }
        pose.calculateJointStates()
        return pose
    }

    /// Teleport the parts to a pose. Velocities are kept unless `resetVelocities`.
    public func setPose(_ pose: SkeletonPose, resetVelocities: Bool = false) {
        for i in 0..<bodies.count {
            let transform = pose.worldTransform(i)
            world.setBodyTransform(bodies[i], transform.p, transform.q)
            if resetVelocities {
                world.setBodyLinearVelocity(bodies[i], Vec3())
                world.setBodyAngularVelocity(bodies[i], Vec3())
            }
        }
    }

    /// Drive each joint toward the pose's relative rotations with position motors. The parts stay
    /// dynamic, so the ragdoll follows the animation while it reacts to contacts and gravity.
    public func driveToPose(usingMotors pose: SkeletonPose, maxTorque: Float, hertz: Float = 8, dampingRatio: Float = 1) {
        for i in 0..<bodies.count {
            let parent = settings.skeleton.joints[i].parentIndex
            guard parent >= 0, world.jointIsValid(joints[i]) else { continue }
            guard let localFrameA = jointLocalFrameA(i) else { continue }
            // Frame B relative to frame A in the target pose.
            let qA = mulQuat(pose.jointMatrices[parent].q, localFrameA.q)
            let qB = mulQuat(pose.jointMatrices[i].q, settings.parts[i].jointFrame.q)
            let targetB = invMulQuat(qA, qB)
            for axis in [SixDOFAxis.rotationX, .rotationY, .rotationZ] {
                guard var settings = world.sixDOFJointAxis(joints[i], axis) else { continue }
                settings.motorMode = .position
                settings.maxMotorForce = maxTorque
                settings.motorHertz = hertz
                settings.motorDampingRatio = dampingRatio
                world.setSixDOFJointAxis(joints[i], axis, settings)
            }
            world.setSixDOFJointTargetRotation(joints[i], normalizeQuat(targetB))
        }
        motorsEnabled = true
    }

    /// Turn the joint motors off again.
    public func disableMotors() {
        guard motorsEnabled else { return }
        for i in 0..<joints.count where world.jointIsValid(joints[i]) {
            for axis in [SixDOFAxis.rotationX, .rotationY, .rotationZ] {
                guard var settings = world.sixDOFJointAxis(joints[i], axis) else { continue }
                settings.motorMode = .off
                world.setSixDOFJointAxis(joints[i], axis, settings)
            }
        }
        motorsEnabled = false
    }

    /// Set every part's velocity so it reaches the pose in `timeStep` (Jolt's
    /// DriveToPoseUsingKinematics). Call before each step.
    public func driveToPose(usingKinematics pose: SkeletonPose, timeStep: Float) {
        for i in 0..<bodies.count {
            world.setBodyTargetTransform(bodies[i], pose.worldTransform(i), timeStep: timeStep)
        }
    }

    private func jointLocalFrameA(_ part: Int) -> Transform? {
        guard let sim = world.jointSim(joints[part]) else { return nil }
        return sim.localFrameA
    }

    // MARK: Whole body

    public func setLinearVelocity(_ velocity: Vec3) {
        for body in bodies { world.setBodyLinearVelocity(body, velocity) }
    }

    /// Apply an impulse spread over the parts by mass, so every part gains the same velocity.
    public func applyLinearImpulse(_ impulse: Vec3) {
        let total = totalMass
        guard total > 0 else { return }
        for body in bodies {
            world.applyBodyLinearImpulseToCenter(body, (world.bodyMass(body) / total) * impulse, true)
        }
    }

    public var totalMass: Float { bodies.reduce(0) { $0 + world.bodyMass($1) } }

    public var isAwake: Bool { bodies.contains { world.isBodyAwake($0) } }

    public func wake() {
        for body in bodies { world.setBodyAwake(body, true) }
    }

    /// Bounds of all parts.
    public var bounds: AABB {
        var box = world.bodyComputeAABB(bodies[0])
        for body in bodies.dropFirst() { box = aabbUnion(box, world.bodyComputeAABB(body)) }
        return box
    }
}
