// Vehicle definitions: wheels, suspension, tires, engine, transmission, differentials, anti-roll
// bars, and the wheeled / motorcycle / tracked controllers.
//
// A vehicle is a constraint between one dynamic chassis body and whatever each wheel touches. The
// wheels are not bodies: every step each wheel is cast against the world, and the suspension,
// tire friction, drivetrain, and pitch/roll limits are solved with the joints of every substep.

/// Vehicle id references a vehicle constraint. This should be treated as an opaque handle.
public struct VehicleID: Sendable, Hashable, BitwiseCopyable {
    var index1: Int32 = 0
    var world0: UInt16 = 0
    var generation: UInt16 = 0

    public init() {}

    init(index1: Int32, world0: UInt16, generation: UInt16) {
        self.index1 = index1
        self.world0 = world0
        self.generation = generation
    }

    public static let null = VehicleID()

    public var isNull: Bool { index1 == 0 }
}

// MARK: - Curves

/// A piecewise linear curve, clamped at both ends. Used for engine torque and tire friction.
public struct LinearCurve: Sendable, Equatable {
    /// Points sorted by increasing `x`.
    public private(set) var points: [Vec2]

    public init(_ points: [Vec2] = []) {
        self.points = points.sorted { $0.x < $1.x }
    }

    /// Add a point, keeping the points sorted.
    public mutating func addPoint(_ x: Float, _ y: Float) {
        let index = points.firstIndex { $0.x > x } ?? points.count
        points.insert(Vec2(x, y), at: index)
    }

    /// Evaluate the curve. An empty curve evaluates to zero.
    public func evaluate(_ x: Float) -> Float {
        guard let first = points.first, let last = points.last else { return 0 }
        if x <= first.x { return first.y }
        if x >= last.x { return last.y }

        // Binary search the segment.
        var lo = 0
        var hi = points.count - 1
        while hi - lo > 1 {
            let mid = (lo + hi) >> 1
            if points[mid].x <= x { lo = mid } else { hi = mid }
        }

        let a = points[lo]
        let b = points[hi]
        let span = b.x - a.x
        if span <= 0 { return b.y }
        return a.y + (x - a.x) / span * (b.y - a.y)
    }
}

// MARK: - Wheel

/// Wheel and suspension settings. Positions and directions are in the chassis body frame.
public struct WheelSettings: Sendable {
    /// Attachment point of the suspension. The wheel center is at
    /// `position + suspensionLength * suspensionDirection`.
    public var position = Vec3()

    /// Where the suspension force is applied when `enableSuspensionForcePoint` is set. Moving the
    /// force point up toward the center of mass reduces body roll.
    public var suspensionForcePoint = Vec3()

    /// Apply the suspension force at `suspensionForcePoint` instead of the tire contact point.
    public var enableSuspensionForcePoint = false

    /// Direction the suspension extends, usually down.
    public var suspensionDirection = Vec3(0, -1, 0)

    /// The steering rotation axis, usually up.
    public var steeringAxis = Vec3(0, 1, 0)

    /// Up direction of the wheel when it is not steered.
    public var wheelUp = Vec3(0, 1, 0)

    /// Forward (rolling) direction of the wheel when it is not steered.
    public var wheelForward = Vec3(0, 0, 1)

    /// Shortest suspension length. Compression past this is resisted by a rigid stop.
    public var suspensionMinLength: Float = 0.3

    /// Longest suspension length. The wheel loses contact when the ground is farther away.
    public var suspensionMaxLength: Float = 0.5

    /// The spring's natural length is `suspensionMaxLength + suspensionPreloadLength`, so a positive
    /// preload keeps the spring compressed at full extension.
    public var suspensionPreloadLength: Float = 0

    /// Suspension spring stiffness in Hertz, relative to the mass carried by this wheel.
    public var suspensionFrequency: Float = 1.5

    /// Suspension spring damping ratio, non-dimensional.
    public var suspensionDampingRatio: Float = 0.5

    /// Absolute spring stiffness (N/m). When positive it overrides `suspensionFrequency`.
    public var suspensionStiffness: Float = 0

    /// Absolute spring damping (N·s/m). Used when `suspensionStiffness` is positive.
    public var suspensionDamping: Float = 0

    /// Wheel radius.
    public var radius: Float = 0.3

    /// Wheel width. Used by the cylinder collision tester and debug draw.
    public var width: Float = 0.1

    // MARK: Wheeled controller

    /// Moment of inertia of the wheel about its axle (kg m^2).
    public var inertia: Float = 0.9

    /// Angular damping of the wheel: dw/dt = -c * w.
    public var angularDamping: Float = 0.2

    /// Largest steering angle in radians.
    public var maxSteerAngle: Float = 70 * B3.degToRad

    /// Brake torque at full brake input (N m).
    public var maxBrakeTorque: Float = 1500

    /// Hand brake torque at full hand brake input (N m).
    public var maxHandBrakeTorque: Float = 4000

    /// Longitudinal friction coefficient as a function of slip ratio |w r - v| / |v|.
    public var longitudinalFriction = LinearCurve([Vec2(0, 0), Vec2(0.06, 1.2), Vec2(0.2, 1)])

    /// Lateral friction coefficient as a function of slip angle in degrees.
    public var lateralFriction = LinearCurve([Vec2(0, 0), Vec2(3, 1.2), Vec2(20, 1)])

    // MARK: Tracked controller

    /// Longitudinal friction of a track wheel.
    public var trackLongitudinalFriction: Float = 4

    /// Lateral friction of a track wheel.
    public var trackLateralFriction: Float = 2

    public init() {}
}

/// An anti-roll bar couples the suspension of two wheels on one axle to resist body roll.
public struct VehicleAntiRollBar: Sendable {
    public var leftWheel: Int32 = 0
    public var rightWheel: Int32 = 1

    /// Force per meter of suspension length difference (N/m).
    public var stiffness: Float = 1000

    public init() {}

    public init(leftWheel: Int32, rightWheel: Int32, stiffness: Float = 1000) {
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel
        self.stiffness = stiffness
    }
}

// MARK: - Drivetrain

/// A combustion engine.
public struct VehicleEngineSettings: Sendable {
    /// Peak torque (N m).
    public var maxTorque: Float = 500

    /// Idle speed. The engine never turns slower than this.
    public var minRPM: Float = 1000

    /// Red line. The engine never turns faster than this.
    public var maxRPM: Float = 6000

    /// Torque as a fraction of `maxTorque` over the normalized speed range [minRPM, maxRPM] -> [0, 1].
    public var normalizedTorque = LinearCurve([Vec2(0, 0.8), Vec2(0.66, 1), Vec2(1, 0.8)])

    /// Moment of inertia of the engine and flywheel (kg m^2).
    public var inertia: Float = 0.5

    /// Angular damping (friction) of the engine.
    public var angularDamping: Float = 0.2

    public init() {}
}

public enum TransmissionMode: Sendable, BitwiseCopyable {
    /// Gears and clutch are chosen from engine speed and throttle.
    case automatic
    /// Gears and clutch are set by `setVehicleTransmission`.
    case manual
}

/// A gearbox with a clutch between the engine and the differentials.
public struct VehicleTransmissionSettings: Sendable {
    public var mode = TransmissionMode.automatic

    /// Forward gear ratios, first gear first.
    public var gearRatios: [Float] = [2.66, 1.78, 1.3, 1.0, 0.74]

    /// Reverse gear ratios. These must be negative.
    public var reverseGearRatios: [Float] = [-2.90]

    /// Time the clutch is disengaged while changing gear (s).
    public var switchTime: Float = 0.5

    /// Time to re-engage the clutch after a gear change (s).
    public var clutchReleaseTime: Float = 0.3

    /// Minimum time between automatic gear changes (s).
    public var switchLatency: Float = 0.5

    /// Engine speed at which the automatic gearbox shifts up.
    public var shiftUpRPM: Float = 4000

    /// Engine speed at which the automatic gearbox shifts down.
    public var shiftDownRPM: Float = 2000

    /// Viscous clutch strength: clutch torque = strength * (w_engine - w_clutch) when engaged.
    public var clutchStrength: Float = 10

    public init() {}
}

/// A differential distributes torque between two wheels of one axle.
public struct VehicleDifferentialSettings: Sendable {
    /// Left driven wheel, or -1 for none.
    public var leftWheel: Int32 = -1

    /// Right driven wheel, or -1 for none.
    public var rightWheel: Int32 = -1

    /// Ratio between the transmission output and the wheel speed.
    public var differentialRatio: Float = 3.42

    /// Fraction of torque to the left wheel; the remainder goes right.
    public var leftRightSplit: Float = 0.5

    /// When the faster wheel turns more than this ratio faster than the slower one, torque moves
    /// to the slower wheel. Use `Float.greatestFiniteMagnitude` for an open differential.
    public var limitedSlipRatio: Float = 1.4

    /// Fraction of engine torque delivered to this differential.
    public var engineTorqueRatio: Float = 1

    public init() {}

    public init(leftWheel: Int32, rightWheel: Int32, engineTorqueRatio: Float = 1) {
        self.leftWheel = leftWheel
        self.rightWheel = rightWheel
        self.engineTorqueRatio = engineTorqueRatio
    }
}

// MARK: - Controllers

/// Controller for cars and trucks.
public struct WheeledVehicleControllerSettings: Sendable {
    public var engine = VehicleEngineSettings()
    public var transmission = VehicleTransmissionSettings()
    public var differentials: [VehicleDifferentialSettings] = []

    /// Limited slip ratio between differentials (front/rear torque bias). Use
    /// `Float.greatestFiniteMagnitude` for an open center differential.
    public var differentialLimitedSlipRatio: Float = 1.4

    public init() {}
}

/// Controller for two wheeled vehicles. The body leans into turns.
public struct MotorcycleControllerSettings: Sendable {
    public var wheeled = WheeledVehicleControllerSettings()

    /// Largest lean angle in radians.
    public var maxLeanAngle: Float = 45 * B3.degToRad

    /// Lean spring stiffness (N m / rad).
    public var leanSpringConstant: Float = 5000

    /// Lean spring damping (N m s / rad).
    public var leanSpringDamping: Float = 1000

    /// Integral gain removing steady state lean error.
    public var leanSpringIntegrationCoefficient: Float = 0

    /// How fast the integral term decays (1/s).
    public var leanSpringIntegrationCoefficientDecay: Float = 4

    /// Low pass factor in [0, 1) applied to the target lean angle each step.
    public var leanSmoothingFactor: Float = 0.8

    /// Reduce steering at speed so the required lean never exceeds `maxLeanAngle`.
    public var enableLeanSteeringLimit = true

    public init() {}
}

/// One track of a tracked vehicle.
public struct VehicleTrackSettings: Sendable {
    /// Wheel that drives the track. Its radius converts track speed to angular speed.
    public var drivenWheel: Int32 = 0

    /// Wheels on this track, including the driven wheel.
    public var wheels: [Int32] = []

    /// Moment of inertia of the track and its wheels, measured at the driven wheel (kg m^2).
    public var inertia: Float = 10

    /// Angular damping of the track.
    public var angularDamping: Float = 0.5

    /// Brake torque at full brake input (N m).
    public var maxBrakeTorque: Float = 15000

    /// Ratio between the transmission output and the driven wheel.
    public var differentialRatio: Float = 6

    public init() {}
}

/// Controller for tanks and bulldozers. Steering is done by driving the tracks at different speeds.
public struct TrackedVehicleControllerSettings: Sendable {
    public var engine = VehicleEngineSettings()
    public var transmission = VehicleTransmissionSettings()

    /// Left and right track, in that order.
    public var tracks: [VehicleTrackSettings] = [VehicleTrackSettings(), VehicleTrackSettings()]

    public init() {}
}

public enum VehicleControllerSettings: Sendable {
    case wheeled(WheeledVehicleControllerSettings)
    case motorcycle(MotorcycleControllerSettings)
    case tracked(TrackedVehicleControllerSettings)
}

// MARK: - Collision

/// How wheels find the ground.
public enum VehicleCollisionTester: Sendable {
    /// A ray from the suspension attachment. Cheapest; misses curbs between ray and tire edge.
    case ray
    /// A sphere of the wheel radius swept along the suspension.
    case sphere
    /// A rounded cylinder matching the wheel swept along the suspension. Most accurate.
    /// `convexRadiusFraction` of the radius/half width is used for rounding.
    case cylinder(convexRadiusFraction: Float)
}

// MARK: - Definition

/// Tire friction override: given wheel index, the curve friction values and the ground surface
/// friction, return the longitudinal and lateral friction coefficients.
public typealias TireFrictionCallback =
    (_ wheelIndex: Int32, _ longitudinal: Float, _ lateral: Float, _ surface: Float, _ userMaterialId: UInt64)
    -> (longitudinal: Float, lateral: Float)

public struct VehicleDef {
    /// The dynamic chassis.
    public var bodyId = BodyID()

    /// Vehicle up in the chassis frame.
    public var up = Vec3(0, 1, 0)

    /// Vehicle forward in the chassis frame.
    public var forward = Vec3(0, 0, 1)

    /// Largest allowed angle between the chassis up and the world up. Values of pi or more disable
    /// the limit. Useful to stop a car from rolling over.
    public var maxPitchRollAngle: Float = B3.pi

    public var wheels: [WheelSettings] = []
    public var antiRollBars: [VehicleAntiRollBar] = []
    public var controller = VehicleControllerSettings.wheeled(WheeledVehicleControllerSettings())

    public var collisionTester = VehicleCollisionTester.ray

    /// Shapes the wheels collide with. The chassis' own shapes and sensors are always ignored.
    public var filter = QueryFilter.defaultFilter

    /// Contacts steeper than this angle relative to the suspension are ignored.
    public var maxSlopeAngle: Float = 80 * B3.degToRad

    public var userData: UnsafeMutableRawPointer?

    public init() {}
}

// MARK: - Inspection

/// Snapshot of a wheel after the last step.
public struct VehicleWheelState: Sendable {
    public var hasContact = false
    public var contactBodyId = BodyID()
    public var contactShapeId = ShapeID()
    public var contactPosition = Pos()
    public var contactNormal = Vec3()
    /// Rolling direction projected on the ground.
    public var contactLongitudinal = Vec3()
    /// Sideways direction on the ground.
    public var contactLateral = Vec3()
    public var suspensionLength: Float = 0
    /// Suspension force from the last step (N).
    public var suspensionForce: Float = 0
    /// Tire forces from the last step (N).
    public var longitudinalForce: Float = 0
    public var lateralForce: Float = 0
    /// Slip ratio |w r - v| / |v|.
    public var longitudinalSlip: Float = 0
    /// Slip angle in radians.
    public var lateralSlip: Float = 0
    public var angularVelocity: Float = 0
    public var rotationAngle: Float = 0
    public var steerAngle: Float = 0

    public init() {}
}

/// Drivetrain snapshot.
public struct VehicleDrivetrainState: Sendable {
    public var engineRPM: Float = 0
    /// Current gear: 0 neutral, negative reverse.
    public var currentGear: Int32 = 0
    public var clutchFriction: Float = 0
    public var isSwitchingGear = false
    /// Lean angle of a motorcycle, radians.
    public var leanAngle: Float = 0
    /// Track angular velocities for tracked vehicles.
    public var trackAngularVelocity = Vec2()

    public init() {}
}
