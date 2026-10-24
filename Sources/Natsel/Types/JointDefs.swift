

/// Joint type enumeration. This is useful because all joint types use `JointID` and sometimes you
/// want to get the type of a joint.
public enum JointType: Int32, Sendable, CaseIterable, BitwiseCopyable {
    case parallelJoint
    case distanceJoint
    case filterJoint
    case motorJoint
    case prismaticJoint
    case revoluteJoint
    case sphericalJoint
    case weldJoint
    case wheelJoint
    /// Couples the rotation of two bodies about their hinge axes.
    case gearJoint
    /// Couples the rotation of a pinion to the translation of a rack.
    case rackAndPinionJoint
    /// Two bodies hanging from fixed points by one rope.
    case pulleyJoint
    /// Configurable joint: each translation and rotation axis is free, limited, or locked, with
    /// optional motors and friction.
    case sixDOFJoint
    /// A point on body B follows a spline fixed in body A.
    case pathJoint
}

/// Base joint definition used by all joint types. The local frames are measured from the body's
/// origin rather than the center of mass because:
/// 1. You might not know where the center of mass will be.
/// 2. If you add/remove shapes from a body and recompute the mass, the joints will be broken.
public struct JointDef {
    /// User data pointer
    public var userData: UnsafeMutableRawPointer?

    /// The first attached body
    public var bodyIdA = BodyID()

    /// The second attached body
    public var bodyIdB = BodyID()

    /// The first local joint frame
    public var localFrameA = Transform(Vec3(), Quat.identity)

    /// The second local joint frame
    public var localFrameB = Transform(Vec3(), Quat.identity)

    /// Force threshold for joint events
    public var forceThreshold: Float = .greatestFiniteMagnitude

    /// Torque threshold for joint events
    public var torqueThreshold: Float = .greatestFiniteMagnitude

    /// Constraint hertz (advanced feature)
    public var constraintHertz: Float = 60

    /// Constraint damping ratio (advanced feature)
    public var constraintDampingRatio: Float = 2

    /// Debug draw scale
    public var drawScale: Float = lengthUnitsPerMeter

    /// Set this flag to true if the attached bodies should collide
    public var collideConnected = false

    public init() {}
}

/// Distance joint definition. Connects a point on body A with a point on body B by a segment.
/// Useful for ropes and springs.
public struct DistanceJointDef {
    /// Base joint definition
    public var base = JointDef()

    /// The rest length of this joint. Clamped to a stable minimum value.
    public var length: Float = 1

    /// Enable the distance constraint to behave like a spring. If false then the distance joint
    /// will be rigid, overriding the limit and motor.
    public var enableSpring = false

    /// The lower spring force controls how much tension it can sustain
    public var lowerSpringForce: Float = -.greatestFiniteMagnitude

    /// The upper spring force controls how much compression it can sustain
    public var upperSpringForce: Float = .greatestFiniteMagnitude

    /// The spring linear stiffness Hertz, cycles per second
    public var hertz: Float = 0

    /// The spring linear damping ratio, non-dimensional
    public var dampingRatio: Float = 0

    /// Enable/disable the joint limit
    public var enableLimit = false

    /// Minimum length. Clamped to a stable minimum value.
    public var minLength: Float = 0

    /// Maximum length. Must be greater than or equal to the minimum length.
    public var maxLength: Float = B3.huge

    /// Enable/disable the joint motor
    public var enableMotor = false

    /// The maximum motor force, usually in newtons
    public var maxMotorForce: Float = 0

    /// The desired motor speed, usually in meters per second
    public var motorSpeed: Float = 0

    public init() {}
}

/// A motor joint is used to control the relative position and velocity between two bodies.
public struct MotorJointDef {
    /// Base joint definition
    public var base = JointDef()

    /// The desired linear velocity
    public var linearVelocity = Vec3()

    /// The maximum motor force in newtons
    public var maxVelocityForce: Float = 0

    /// The desired angular velocity
    public var angularVelocity = Vec3()

    /// The maximum motor torque in newton-meters
    public var maxVelocityTorque: Float = 0

    /// Linear spring hertz for position control
    public var linearHertz: Float = 0

    /// Linear spring damping ratio
    public var linearDampingRatio: Float = 0

    /// Maximum spring force in newtons
    public var maxSpringForce: Float = 0

    /// Angular spring hertz for position control
    public var angularHertz: Float = 0

    /// Angular spring damping ratio
    public var angularDampingRatio: Float = 0

    /// Maximum spring torque in newton-meters
    public var maxSpringTorque: Float = 0

    public init() {}
}

/// A filter joint is used to disable collision between two specific bodies.
public struct FilterJointDef {
    /// Base joint definition
    public var base = JointDef()

    public init() {}
}

/// Parallel joint definition. Constrains the angle between axis z in body A and axis z in body B
/// using a spring. Useful to keep a body upright.
public struct ParallelJointDef {
    /// Base joint definition
    public var base = JointDef()

    /// The spring stiffness Hertz, cycles per second
    public var hertz: Float = 1

    /// The spring damping ratio, non-dimensional
    public var dampingRatio: Float = 1

    /// The maximum spring torque, typically in newton-meters.
    public var maxTorque: Float = .greatestFiniteMagnitude

    public init() {}
}

/// Prismatic joint definition. Body B may slide along the x-axis in local frame A. Body B cannot
/// rotate relative to body A. The joint translation is zero when the local frame origins coincide
/// in world space.
public struct PrismaticJointDef {
    /// Base joint definition
    public var base = JointDef()

    /// Enable a linear spring along the prismatic joint axis
    public var enableSpring = false

    /// The spring stiffness Hertz, cycles per second
    public var hertz: Float = 0

    /// The spring damping ratio, non-dimensional
    public var dampingRatio: Float = 0

    /// The target translation for the joint in meters. The spring-damper will drive to this translation.
    public var targetTranslation: Float = 0

    /// Enable/disable the joint limit
    public var enableLimit = false

    /// The lower translation limit
    public var lowerTranslation: Float = 0

    /// The upper translation limit
    public var upperTranslation: Float = 0

    /// Enable/disable the joint motor
    public var enableMotor = false

    /// The maximum motor force, typically in newtons
    public var maxMotorForce: Float = 0

    /// The desired motor speed, typically in meters per second
    public var motorSpeed: Float = 0

    public init() {}
}

/// Revolute joint definition. A point on body B is fixed to a point on body A. Allows relative
/// rotation about the z-axis.
public struct RevoluteJointDef {
    /// Base joint definition.
    public var base = JointDef()

    /// The bodyB angle minus bodyA angle in the reference state (radians).
    /// This defines the zero angle for the joint limit.
    public var targetAngle: Float = 0

    /// Enable a rotational spring on the revolute hinge axis.
    public var enableSpring = false

    /// The spring stiffness Hertz, cycles per second.
    public var hertz: Float = 0

    /// The spring damping ratio, non-dimensional.
    public var dampingRatio: Float = 0

    /// A flag to enable joint limits.
    public var enableLimit = false

    /// The lower angle for the joint limit in radians. Minimum of -0.99*pi radians.
    public var lowerAngle: Float = 0

    /// The upper angle for the joint limit in radians. Maximum of 0.99*pi radians.
    public var upperAngle: Float = 0

    /// A flag to enable the joint motor.
    public var enableMotor = false

    /// The maximum motor torque, typically in newton-meters.
    public var maxMotorTorque: Float = 0

    /// The desired motor speed in radians per second.
    public var motorSpeed: Float = 0

    public init() {}
}

/// Spherical joint definition. A point on body B is fixed to a point on body A. Allows rotation
/// about the shared point.
public struct SphericalJointDef {
    /// Base joint definition
    public var base = JointDef()

    /// Enable a rotational spring that attempts to align the two joint frames.
    public var enableSpring = false

    /// The spring stiffness Hertz, cycles per second. Non-negative number.
    public var hertz: Float = 0

    /// The spring damping ratio, non-dimensional. Non-negative number.
    public var dampingRatio: Float = 0

    /// Target spring rotation, joint frame B relative to joint frame A.
    public var targetRotation = Quat.identity

    /// A flag to enable the cone limit. The cone is centered on the frameA z-axis.
    public var enableConeLimit = false

    /// The angle for the cone limit in radians. Valid range is [0, pi]
    public var coneAngle: Float = 0

    /// A flag to enable the twist limit. The twist is centered on the frameB z-axis.
    public var enableTwistLimit = false

    /// The angle for the lower twist limit in radians. Minimum of -0.99*pi radians.
    public var lowerTwistAngle: Float = 0

    /// The angle for the upper twist limit in radians. Maximum of 0.99*pi radians.
    public var upperTwistAngle: Float = 0

    /// A flag to enable the joint motor
    public var enableMotor = false

    /// The maximum motor torque, typically in newton-meters. Non-negative number.
    public var maxMotorTorque: Float = 0

    /// The desired motor angular velocity in radians per second, the relative angular velocity
    /// between the two bodies in world space: angularVelocityB - angularVelocityA
    public var motorVelocity = Vec3()

    public init() {}
}

/// Weld joint definition. Connects two bodies together rigidly. This constraint provides springs
/// to mimic soft-body simulation.
public struct WeldJointDef {
    /// Base joint definition
    public var base = JointDef()

    /// Linear stiffness expressed as Hertz (cycles per second). Use zero for maximum stiffness.
    public var linearHertz: Float = 0

    /// Angular stiffness as Hertz (cycles per second). Use zero for maximum stiffness.
    public var angularHertz: Float = 0

    /// Linear damping ratio, non-dimensional. Use 1 for critical damping.
    public var linearDampingRatio: Float = 0

    /// Angular damping ratio, non-dimensional. Use 1 for critical damping.
    public var angularDampingRatio: Float = 0

    public init() {}
}

/// Wheel joint definition. Body A is the chassis and body B is the wheel. The wheel rotates around
/// the local z-axis in frame B, translates along the local x-axis in frame A, and can optionally
/// steer along the x-axis in frame A.
public struct WheelJointDef {
    /// Base joint definition
    public var base = JointDef()

    /// Enable a linear spring along the local axis
    public var enableSuspensionSpring = true

    /// Spring stiffness in Hertz
    public var suspensionHertz: Float = 1

    /// Spring damping ratio, non-dimensional
    public var suspensionDampingRatio: Float = 0.7

    /// Enable/disable the joint linear limit
    public var enableSuspensionLimit = false

    /// The lower suspension translation limit
    public var lowerSuspensionLimit: Float = 0

    /// The upper translation limit
    public var upperSuspensionLimit: Float = 0

    /// Enable/disable the joint rotational motor
    public var enableSpinMotor = false

    /// The maximum motor torque, typically in newton-meters
    public var maxSpinTorque: Float = 0

    /// The desired motor speed in radians per second
    public var spinSpeed: Float = 0

    /// Enable steering, otherwise the steering is fixed forward
    public var enableSteering = false

    /// Steering stiffness in Hertz
    public var steeringHertz: Float = 1

    /// Spring damping ratio, non-dimensional
    public var steeringDampingRatio: Float = 0.7

    /// The target steering angle in radians
    public var targetSteeringAngle: Float = 0

    /// The maximum steering torque in N*m
    public var maxSteeringTorque: Float = 0

    /// Enable/disable the steering angular limit
    public var enableSteeringLimit = false

    /// The lower steering angle in radians
    public var lowerSteeringLimit: Float = 0

    /// The upper steering angle in radians
    public var upperSteeringLimit: Float = 0

    public init() {}
}

/// Gear joint definition. Body A and body B turn about the z axes of their local frames, coupled
/// by `angleA + ratio * angleB = constant`. For two meshing gears `ratio` is the tooth count of B
/// over the tooth count of A; use a negative ratio for a belt or chain (same direction).
/// The bodies are usually held by revolute joints. Passing those as references measures the angles
/// relative to the gear mounts, which keeps the coupling exact when the mounts move.
public struct GearJointDef {
    public var base = JointDef()
    public var ratio: Float = 1
    /// Optional revolute joint holding body A (angle drift correction).
    public var referenceJointA = JointID()
    /// Optional revolute joint holding body B (angle drift correction).
    public var referenceJointB = JointID()

    public init() {}
}

/// Rack and pinion joint definition. Body A (the pinion) turns about the z axis of its local frame,
/// body B (the rack) slides along the x axis of its local frame, coupled by
/// `translationB = ratio * angleA + constant`. `ratio` is the pinion pitch radius.
public struct RackAndPinionJointDef {
    public var base = JointDef()
    public var ratio: Float = 1
    /// Optional revolute joint holding the pinion (angle drift correction).
    public var referencePinionJoint = JointID()
    /// Optional prismatic joint holding the rack (translation drift correction).
    public var referenceRackJoint = JointID()

    public init() {}
}

/// Pulley joint definition. The anchors (local frame origins) hang from fixed ground points by
/// one rope: `lengthA + ratio * lengthB` stays within [minLength, maxLength].
public struct PulleyJointDef {
    public var base = JointDef()
    public var groundAnchorA = Pos()
    public var groundAnchorB = Pos()
    public var ratio: Float = 1
    /// Rope length bounds. Zero (the default) uses the length at creation, making the rope taut.
    public var minLength: Float = 0
    public var maxLength: Float = 0

    public init() {}
}
