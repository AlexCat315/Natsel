

/// The body simulation type. The type determines how the body behaves in the simulation.
public enum BodyType: Int32, Sendable, CaseIterable, BitwiseCopyable {
    /// zero mass, zero velocity, may be manually moved
    case staticBody = 0

    /// zero mass, velocity set by user, moved by solver
    case kinematicBody = 1

    /// positive mass, velocity determined by forces, moved by solver
    case dynamicBody = 2

    /// number of body types
    static let count: Int32 = 3
}

/// Motion locks to restrict the body movement.
public struct MotionLocks: Sendable, Equatable, BitwiseCopyable {
    /// Prevent translation along the x-axis
    public var linearX = false
    /// Prevent translation along the y-axis
    public var linearY = false
    /// Prevent translation along the z-axis
    public var linearZ = false
    /// Prevent rotation around the x-axis
    public var angularX = false
    /// Prevent rotation around the y-axis
    public var angularY = false
    /// Prevent rotation around the z-axis
    public var angularZ = false

    public init() {}

    public init(linearX: Bool = false, linearY: Bool = false, linearZ: Bool = false,
                angularX: Bool = false, angularY: Bool = false, angularZ: Bool = false) {
        self.linearX = linearX
        self.linearY = linearY
        self.linearZ = linearZ
        self.angularX = angularX
        self.angularY = angularY
        self.angularZ = angularZ
    }
}

/// A body definition holds all the data needed to construct a rigid body. Shapes are added to a
/// body after construction.
public struct BodyDef {
    /// The body type: static, kinematic, or dynamic.
    public var type: BodyType = .staticBody

    /// The initial world position of the body. Bodies should be created with the desired position.
    public var position = Pos()

    /// The initial world rotation of the body.
    public var rotation = Quat.identity

    /// The initial linear velocity of the body's origin. Usually in meters per second.
    public var linearVelocity = Vec3()

    /// The initial angular velocity of the body. Radians per second.
    public var angularVelocity = Vec3()

    /// Linear damping is used to reduce the linear velocity.
    public var linearDamping: Float = 0

    /// Angular damping is used to reduce the angular velocity.
    public var angularDamping: Float = 0

    /// Scale the gravity applied to this body. Non-dimensional.
    public var gravityScale: Float = 1

    /// Sleep speed threshold, default is 0.05 meters per second
    public var sleepThreshold: Float = 0.05 * lengthUnitsPerMeter

    /// Continuous collision safety factor. Non-dimensional. Recommended range [0.01, 0.5].
    public var safetyFactor: Float = 0.5

    /// Optional body name for debugging.
    public var name: String?

    /// Use this to store application specific body data.
    public var userData: UnsafeMutableRawPointer?

    /// Motions locks to restrict linear and angular movement
    public var motionLocks = MotionLocks()

    /// Set this flag to false if this body should never fall asleep.
    public var enableSleep = true

    /// Is this body initially awake or sleeping?
    public var isAwake = true

    /// Treat this body as a high speed object that performs continuous collision detection
    /// against dynamic and kinematic bodies, but not other bullet bodies.
    public var isBullet = false

    /// Used to disable a body. A disabled body does not move or collide.
    public var isEnabled = true

    /// This allows this body to bypass rotational speed limits. Should only be used for circular
    /// objects, like wheels.
    public var allowFastRotation = false

    /// Enable contact recycling. Leaving this enabled improves performance but may lead to ghost
    /// collision that should be avoided on characters.
    public var enableContactRecycling = true

    public init() {}
}
