//! Vehicle definitions: wheels, suspension, tires, engine, transmission, differentials, anti-roll
//! bars, and the wheeled / motorcycle / tracked controllers.
//!
//! A vehicle is a constraint between one dynamic chassis body and whatever each wheel touches. The
//! wheels are not bodies: every step each wheel is cast against the world, and the suspension,
//! tire friction, drivetrain, and pitch/roll limits are solved with the joints of every substep.
//!
//! Translated from `Sources/Natsel/Vehicle/VehicleTypes.swift`.
//!
//! Swift default arguments become explicit caller-side constants, documented at each constructor:
//! - `LinearCurve(_ points: [Vec2] = [])` -> `LinearCurve::new(&[])`, or `LinearCurve::default()`.
//! - `VehicleAntiRollBar(leftWheel:rightWheel:stiffness: Float = 1000)` ->
//!   `VehicleAntiRollBar::with_wheels(left_wheel, right_wheel, 1000.0)`.
//! - `VehicleDifferentialSettings(leftWheel:rightWheel:engineTorqueRatio: Float = 1)` ->
//!   `VehicleDifferentialSettings::with_wheels(left_wheel, right_wheel, 1.0)`.

use core::cmp::Ordering;
use std::sync::Arc;

use crate::base::constants as b3;
use crate::math::types::{Pos, Vec2, Vec3};
use crate::types::ids::{BodyID, ShapeID};
use crate::types::shape_def::QueryFilter;

/// Vehicle id references a vehicle constraint. This should be treated as an opaque handle.
///
/// Swift's zero-argument `public init()` is `VehicleID::default()`, which equals [`VehicleID::NULL`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct VehicleID {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

impl VehicleID {
    /// Swift's `init(index1:world0:generation:)`.
    #[inline]
    pub fn new(index1: i32, world0: u16, generation: u16) -> Self {
        Self {
            index1,
            world0,
            generation,
        }
    }

    /// Swift's `public static let null`.
    pub const NULL: VehicleID = VehicleID {
        index1: 0,
        world0: 0,
        generation: 0,
    };

    /// Swift's `public var isNull: Bool`.
    #[inline]
    pub fn is_null(&self) -> bool {
        self.index1 == 0
    }
}

// MARK: - Curves

/// A piecewise linear curve, clamped at both ends. Used for engine torque and tire friction.
///
/// Swift's `public private(set) var points` becomes a plain `pub` field: reading matches Swift, and
/// the only mutator Swift offered is [`LinearCurve::add_point`], so direct writes are the one
/// widening here. Swift's copy-on-write array becomes an owned `Vec`, so a curve (and everything
/// holding one) is `Clone` rather than `Copy`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct LinearCurve {
    /// Points sorted by increasing `x`.
    pub points: Vec<Vec2>,
}

impl LinearCurve {
    /// Swift's `init(_ points: [Vec2] = [])` points sorted by increasing `x`. Swift's default
    /// argument has no Rust equivalent: ported callers pass `&[]` (or use `LinearCurve::default()`)
    /// for the empty curve.
    pub fn new(points: &[Vec2]) -> Self {
        let mut points = points.to_vec();
        // Swift: points.sorted { $0.x < $1.x }. The comparator reproduces Swift's `<` exactly, so a
        // NaN `x` compares equal to every other point. Swift's `sorted` is unstable; Rust's
        // `sort_by` is stable, which keeps equal-`x` points in input order deterministically.
        points.sort_by(|a, b| {
            if a.x < b.x {
                Ordering::Less
            } else if a.x > b.x {
                Ordering::Greater
            } else {
                Ordering::Equal
            }
        });
        Self { points }
    }

    /// Add a point, keeping the points sorted.
    pub fn add_point(&mut self, x: f32, y: f32) {
        let index = self
            .points
            .iter()
            .position(|point| point.x > x)
            .unwrap_or(self.points.len());
        self.points.insert(index, Vec2::new(x, y));
    }

    /// Evaluate the curve. An empty curve evaluates to zero.
    pub fn evaluate(&self, x: f32) -> f32 {
        let (first, last) = match (self.points.first(), self.points.last()) {
            (Some(first), Some(last)) => (*first, *last),
            _ => return 0.0,
        };
        if x <= first.x {
            return first.y;
        }
        if x >= last.x {
            return last.y;
        }

        // Binary search the segment.
        let mut lo: usize = 0;
        let mut hi: usize = self.points.len() - 1;
        while hi - lo > 1 {
            let mid = (lo + hi) >> 1;
            if self.points[mid].x <= x {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        let a = self.points[lo];
        let b = self.points[hi];
        let span = b.x - a.x;
        if span <= 0.0 {
            return b.y;
        }
        a.y + (x - a.x) / span * (b.y - a.y)
    }
}

// MARK: - Wheel

/// Wheel and suspension settings. Positions and directions are in the chassis body frame.
///
/// Holds [`LinearCurve`] storage, so it is `Clone` rather than Swift's value-semantics `Copy`; the
/// field order is unchanged.
#[derive(Clone, Debug, PartialEq)]
pub struct WheelSettings {
    /// Attachment point of the suspension. The wheel center is at
    /// `position + suspensionLength * suspensionDirection`.
    pub position: Vec3,

    /// Where the suspension force is applied when `enableSuspensionForcePoint` is set. Moving the
    /// force point up toward the center of mass reduces body roll.
    pub suspension_force_point: Vec3,

    /// Apply the suspension force at `suspensionForcePoint` instead of the tire contact point.
    pub enable_suspension_force_point: bool,

    /// Direction the suspension extends, usually down.
    pub suspension_direction: Vec3,

    /// The steering rotation axis, usually up.
    pub steering_axis: Vec3,

    /// Up direction of the wheel when it is not steered.
    pub wheel_up: Vec3,

    /// Forward (rolling) direction of the wheel when it is not steered.
    pub wheel_forward: Vec3,

    /// Shortest suspension length. Compression past this is resisted by a rigid stop.
    pub suspension_min_length: f32,

    /// Longest suspension length. The wheel loses contact when the ground is farther away.
    pub suspension_max_length: f32,

    /// The spring's natural length is `suspensionMaxLength + suspensionPreloadLength`, so a positive
    /// preload keeps the spring compressed at full extension.
    pub suspension_preload_length: f32,

    /// Suspension spring stiffness in Hertz, relative to the mass carried by this wheel.
    pub suspension_frequency: f32,

    /// Suspension spring damping ratio, non-dimensional.
    pub suspension_damping_ratio: f32,

    /// Absolute spring stiffness (N/m). When positive it overrides `suspensionFrequency`.
    pub suspension_stiffness: f32,

    /// Absolute spring damping (N·s/m). Used when `suspensionStiffness` is positive.
    pub suspension_damping: f32,

    /// Wheel radius.
    pub radius: f32,

    /// Wheel width. Used by the cylinder collision tester and debug draw.
    pub width: f32,

    // MARK: Wheeled controller
    /// Moment of inertia of the wheel about its axle (kg m^2).
    pub inertia: f32,

    /// Angular damping of the wheel: dw/dt = -c * w.
    pub angular_damping: f32,

    /// Largest steering angle in radians.
    pub max_steer_angle: f32,

    /// Brake torque at full brake input (N m).
    pub max_brake_torque: f32,

    /// Hand brake torque at full hand brake input (N m).
    pub max_hand_brake_torque: f32,

    /// Longitudinal friction coefficient as a function of slip ratio |w r - v| / |v|.
    pub longitudinal_friction: LinearCurve,

    /// Lateral friction coefficient as a function of slip angle in degrees.
    pub lateral_friction: LinearCurve,

    // MARK: Tracked controller
    /// Longitudinal friction of a track wheel.
    pub track_longitudinal_friction: f32,

    /// Lateral friction of a track wheel.
    pub track_lateral_friction: f32,
}

/// Swift's property initializers are non-zero, so `Default` cannot be derived.
impl Default for WheelSettings {
    #[inline]
    fn default() -> Self {
        Self {
            position: Vec3::ZERO,
            suspension_force_point: Vec3::ZERO,
            enable_suspension_force_point: false,
            suspension_direction: Vec3::new(0.0, -1.0, 0.0),
            steering_axis: Vec3::new(0.0, 1.0, 0.0),
            wheel_up: Vec3::new(0.0, 1.0, 0.0),
            wheel_forward: Vec3::new(0.0, 0.0, 1.0),
            suspension_min_length: 0.3,
            suspension_max_length: 0.5,
            suspension_preload_length: 0.0,
            suspension_frequency: 1.5,
            suspension_damping_ratio: 0.5,
            suspension_stiffness: 0.0,
            suspension_damping: 0.0,
            radius: 0.3,
            width: 0.1,
            inertia: 0.9,
            angular_damping: 0.2,
            max_steer_angle: 70.0 * b3::DEG_TO_RAD,
            max_brake_torque: 1500.0,
            max_hand_brake_torque: 4000.0,
            longitudinal_friction: LinearCurve::new(&[
                Vec2::new(0.0, 0.0),
                Vec2::new(0.06, 1.2),
                Vec2::new(0.2, 1.0),
            ]),
            lateral_friction: LinearCurve::new(&[
                Vec2::new(0.0, 0.0),
                Vec2::new(3.0, 1.2),
                Vec2::new(20.0, 1.0),
            ]),
            track_longitudinal_friction: 4.0,
            track_lateral_friction: 2.0,
        }
    }
}

impl WheelSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// An anti-roll bar couples the suspension of two wheels on one axle to resist body roll.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct VehicleAntiRollBar {
    pub left_wheel: i32,
    pub right_wheel: i32,

    /// Force per meter of suspension length difference (N/m).
    pub stiffness: f32,
}

/// Swift's property initializers `leftWheel = 0`, `rightWheel = 1`, `stiffness = 1000` are not all
/// zero, so `Default` cannot be derived.
impl Default for VehicleAntiRollBar {
    #[inline]
    fn default() -> Self {
        Self {
            left_wheel: 0,
            right_wheel: 1,
            stiffness: 1000.0,
        }
    }
}

impl VehicleAntiRollBar {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Swift's `init(leftWheel: Int32, rightWheel: Int32, stiffness: Float = 1000)`; ported callers
    /// pass Swift's defaulted stiffness explicitly as `1000.0`.
    #[inline]
    pub fn with_wheels(left_wheel: i32, right_wheel: i32, stiffness: f32) -> Self {
        Self {
            left_wheel,
            right_wheel,
            stiffness,
        }
    }
}

// MARK: - Drivetrain

/// A combustion engine.
///
/// Holds [`LinearCurve`] storage, so it is `Clone` rather than Swift's value-semantics `Copy`.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleEngineSettings {
    /// Peak torque (N m).
    pub max_torque: f32,

    /// Idle speed. The engine never turns slower than this.
    pub min_rpm: f32,

    /// Red line. The engine never turns faster than this.
    pub max_rpm: f32,

    /// Torque as a fraction of `maxTorque` over the normalized speed range [minRPM, maxRPM] -> [0, 1].
    pub normalized_torque: LinearCurve,

    /// Moment of inertia of the engine and flywheel (kg m^2).
    pub inertia: f32,

    /// Angular damping (friction) of the engine.
    pub angular_damping: f32,
}

/// Swift's property initializers are non-zero, so `Default` cannot be derived.
impl Default for VehicleEngineSettings {
    #[inline]
    fn default() -> Self {
        Self {
            max_torque: 500.0,
            min_rpm: 1000.0,
            max_rpm: 6000.0,
            normalized_torque: LinearCurve::new(&[
                Vec2::new(0.0, 0.8),
                Vec2::new(0.66, 1.0),
                Vec2::new(1.0, 0.8),
            ]),
            inertia: 0.5,
            angular_damping: 0.2,
        }
    }
}

impl VehicleEngineSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// The gearbox control mode.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum TransmissionMode {
    /// Gears and clutch are chosen from engine speed and throttle.
    #[default]
    Automatic = 0,
    /// Gears and clutch are set by `setVehicleTransmission`.
    Manual = 1,
}

/// A gearbox with a clutch between the engine and the differentials.
///
/// Owns the gear-ratio arrays, so it is `Clone` rather than Swift's value-semantics `Copy`.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleTransmissionSettings {
    pub mode: TransmissionMode,

    /// Forward gear ratios, first gear first.
    pub gear_ratios: Vec<f32>,

    /// Reverse gear ratios. These must be negative.
    pub reverse_gear_ratios: Vec<f32>,

    /// Time the clutch is disengaged while changing gear (s).
    pub switch_time: f32,

    /// Time to re-engage the clutch after a gear change (s).
    pub clutch_release_time: f32,

    /// Minimum time between automatic gear changes (s).
    pub switch_latency: f32,

    /// Engine speed at which the automatic gearbox shifts up.
    pub shift_up_rpm: f32,

    /// Engine speed at which the automatic gearbox shifts down.
    pub shift_down_rpm: f32,

    /// Viscous clutch strength: clutch torque = strength * (w_engine - w_clutch) when engaged.
    pub clutch_strength: f32,
}

/// Swift's property initializers are non-zero, so `Default` cannot be derived.
impl Default for VehicleTransmissionSettings {
    #[inline]
    fn default() -> Self {
        Self {
            mode: TransmissionMode::Automatic,
            gear_ratios: vec![2.66, 1.78, 1.3, 1.0, 0.74],
            reverse_gear_ratios: vec![-2.90],
            switch_time: 0.5,
            clutch_release_time: 0.3,
            switch_latency: 0.5,
            shift_up_rpm: 4000.0,
            shift_down_rpm: 2000.0,
            clutch_strength: 10.0,
        }
    }
}

impl VehicleTransmissionSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// A differential distributes torque between two wheels of one axle.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct VehicleDifferentialSettings {
    /// Left driven wheel, or -1 for none.
    pub left_wheel: i32,

    /// Right driven wheel, or -1 for none.
    pub right_wheel: i32,

    /// Ratio between the transmission output and the wheel speed.
    pub differential_ratio: f32,

    /// Fraction of torque to the left wheel; the remainder goes right.
    pub left_right_split: f32,

    /// When the faster wheel turns more than this ratio faster than the slower one, torque moves
    /// to the slower wheel. Use `f32::MAX` (Swift's `Float.greatestFiniteMagnitude`) for an open
    /// differential.
    pub limited_slip_ratio: f32,

    /// Fraction of engine torque delivered to this differential.
    pub engine_torque_ratio: f32,
}

/// Swift's property initializers are non-zero, so `Default` cannot be derived.
impl Default for VehicleDifferentialSettings {
    #[inline]
    fn default() -> Self {
        Self {
            left_wheel: -1,
            right_wheel: -1,
            differential_ratio: 3.42,
            left_right_split: 0.5,
            limited_slip_ratio: 1.4,
            engine_torque_ratio: 1.0,
        }
    }
}

impl VehicleDifferentialSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Swift's `init(leftWheel: Int32, rightWheel: Int32, engineTorqueRatio: Float = 1)`; ported
    /// callers pass Swift's defaulted ratio explicitly as `1.0`. The other settings keep
    /// [`VehicleDifferentialSettings::default`], as in Swift.
    #[inline]
    pub fn with_wheels(left_wheel: i32, right_wheel: i32, engine_torque_ratio: f32) -> Self {
        Self {
            left_wheel,
            right_wheel,
            engine_torque_ratio,
            ..Self::default()
        }
    }
}

// MARK: - Controllers

/// Controller for cars and trucks.
#[derive(Clone, Debug, PartialEq)]
pub struct WheeledVehicleControllerSettings {
    pub engine: VehicleEngineSettings,
    pub transmission: VehicleTransmissionSettings,
    pub differentials: Vec<VehicleDifferentialSettings>,

    /// Limited slip ratio between differentials (front/rear torque bias). Use `f32::MAX` (Swift's
    /// `Float.greatestFiniteMagnitude`) for an open center differential.
    pub differential_limited_slip_ratio: f32,
}

/// `differentialLimitedSlipRatio = 1.4` is non-zero, so `Default` cannot be derived.
impl Default for WheeledVehicleControllerSettings {
    #[inline]
    fn default() -> Self {
        Self {
            engine: VehicleEngineSettings::default(),
            transmission: VehicleTransmissionSettings::default(),
            differentials: Vec::new(),
            differential_limited_slip_ratio: 1.4,
        }
    }
}

impl WheeledVehicleControllerSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// Controller for two wheeled vehicles. The body leans into turns.
#[derive(Clone, Debug, PartialEq)]
pub struct MotorcycleControllerSettings {
    pub wheeled: WheeledVehicleControllerSettings,

    /// Largest lean angle in radians.
    pub max_lean_angle: f32,

    /// Lean spring stiffness (N m / rad).
    pub lean_spring_constant: f32,

    /// Lean spring damping (N m s / rad).
    pub lean_spring_damping: f32,

    /// Integral gain removing steady state lean error.
    pub lean_spring_integration_coefficient: f32,

    /// How fast the integral term decays (1/s).
    pub lean_spring_integration_coefficient_decay: f32,

    /// Low pass factor in [0, 1) applied to the target lean angle each step.
    pub lean_smoothing_factor: f32,

    /// Reduce steering at speed so the required lean never exceeds `maxLeanAngle`.
    pub enable_lean_steering_limit: bool,
}

/// Swift's property initializers are non-zero, so `Default` cannot be derived.
impl Default for MotorcycleControllerSettings {
    #[inline]
    fn default() -> Self {
        Self {
            wheeled: WheeledVehicleControllerSettings::default(),
            max_lean_angle: 45.0 * b3::DEG_TO_RAD,
            lean_spring_constant: 5000.0,
            lean_spring_damping: 1000.0,
            lean_spring_integration_coefficient: 0.0,
            lean_spring_integration_coefficient_decay: 4.0,
            lean_smoothing_factor: 0.8,
            enable_lean_steering_limit: true,
        }
    }
}

impl MotorcycleControllerSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// One track of a tracked vehicle.
#[derive(Clone, Debug, PartialEq)]
pub struct VehicleTrackSettings {
    /// Wheel that drives the track. Its radius converts track speed to angular speed.
    pub driven_wheel: i32,

    /// Wheels on this track, including the driven wheel.
    pub wheels: Vec<i32>,

    /// Moment of inertia of the track and its wheels, measured at the driven wheel (kg m^2).
    pub inertia: f32,

    /// Angular damping of the track.
    pub angular_damping: f32,

    /// Brake torque at full brake input (N m).
    pub max_brake_torque: f32,

    /// Ratio between the transmission output and the driven wheel.
    pub differential_ratio: f32,
}

/// Swift's property initializers are non-zero, so `Default` cannot be derived.
impl Default for VehicleTrackSettings {
    #[inline]
    fn default() -> Self {
        Self {
            driven_wheel: 0,
            wheels: Vec::new(),
            inertia: 10.0,
            angular_damping: 0.5,
            max_brake_torque: 15000.0,
            differential_ratio: 6.0,
        }
    }
}

impl VehicleTrackSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// Controller for tanks and bulldozers. Steering is done by driving the tracks at different speeds.
#[derive(Clone, Debug, PartialEq)]
pub struct TrackedVehicleControllerSettings {
    pub engine: VehicleEngineSettings,
    pub transmission: VehicleTransmissionSettings,

    /// Left and right track, in that order.
    pub tracks: Vec<VehicleTrackSettings>,
}

/// Swift initializes `tracks` with two default tracks, so `Default` cannot be derived.
impl Default for TrackedVehicleControllerSettings {
    #[inline]
    fn default() -> Self {
        Self {
            engine: VehicleEngineSettings::default(),
            transmission: VehicleTransmissionSettings::default(),
            tracks: vec![
                VehicleTrackSettings::default(),
                VehicleTrackSettings::default(),
            ],
        }
    }
}

impl TrackedVehicleControllerSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// The controller variant of a [`VehicleDef`]. Swift's `case wheeled` is
/// [`VehicleControllerSettings::Wheeled`].
#[derive(Clone, Debug, PartialEq)]
pub enum VehicleControllerSettings {
    Wheeled(WheeledVehicleControllerSettings),
    Motorcycle(MotorcycleControllerSettings),
    Tracked(TrackedVehicleControllerSettings),
}

// MARK: - Collision

/// How wheels find the ground.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum VehicleCollisionTester {
    /// A ray from the suspension attachment. Cheapest; misses curbs between ray and tire edge.
    #[default]
    Ray,
    /// A sphere of the wheel radius swept along the suspension.
    Sphere,
    /// A rounded cylinder matching the wheel swept along the suspension. Most accurate.
    /// `convexRadiusFraction` of the radius/half width is used for rounding.
    ///
    /// Swift's `case cylinder(convexRadiusFraction: Float)` associated value becomes the tuple
    /// payload.
    Cylinder(f32),
}

// MARK: - Definition

/// Tire friction override: given wheel index, the curve friction values and the ground surface
/// friction, return the longitudinal and lateral friction coefficients.
///
/// Swift's result tuple labels become the positional result `(longitudinal, lateral)`.
pub type TireFrictionCallback =
    Arc<dyn Fn(i32, f32, f32, f32, u64) -> (f32, f32) + Send + Sync>;

/// A vehicle definition holds all the data needed to create a vehicle constraint.
#[derive(Clone, Debug)]
pub struct VehicleDef {
    /// The dynamic chassis.
    pub body_id: BodyID,

    /// Vehicle up in the chassis frame.
    pub up: Vec3,

    /// Vehicle forward in the chassis frame.
    pub forward: Vec3,

    /// Largest allowed angle between the chassis up and the world up. Values of pi or more disable
    /// the limit. Useful to stop a car from rolling over.
    pub max_pitch_roll_angle: f32,

    pub wheels: Vec<WheelSettings>,
    pub anti_roll_bars: Vec<VehicleAntiRollBar>,
    pub controller: VehicleControllerSettings,

    pub collision_tester: VehicleCollisionTester,

    /// Shapes the wheels collide with. The chassis' own shapes and sensors are always ignored.
    pub filter: QueryFilter,

    /// Contacts steeper than this angle relative to the suspension are ignored.
    pub max_slope_angle: f32,

    pub user_data: Option<*mut core::ffi::c_void>,
}

/// Swift's property initializers are non-zero, so `Default` cannot be derived.
impl Default for VehicleDef {
    #[inline]
    fn default() -> Self {
        Self {
            body_id: BodyID::default(),
            up: Vec3::new(0.0, 1.0, 0.0),
            forward: Vec3::new(0.0, 0.0, 1.0),
            max_pitch_roll_angle: b3::PI,
            wheels: Vec::new(),
            anti_roll_bars: Vec::new(),
            controller: VehicleControllerSettings::Wheeled(
                WheeledVehicleControllerSettings::default(),
            ),
            collision_tester: VehicleCollisionTester::Ray,
            filter: QueryFilter::default_filter(),
            max_slope_angle: 80.0 * b3::DEG_TO_RAD,
            user_data: None,
        }
    }
}

impl VehicleDef {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

// MARK: - Inspection

/// Snapshot of a wheel after the last step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct VehicleWheelState {
    pub has_contact: bool,
    pub contact_body_id: BodyID,
    pub contact_shape_id: ShapeID,
    pub contact_position: Pos,
    pub contact_normal: Vec3,
    /// Rolling direction projected on the ground.
    pub contact_longitudinal: Vec3,
    /// Sideways direction on the ground.
    pub contact_lateral: Vec3,
    pub suspension_length: f32,
    /// Suspension force from the last step (N).
    pub suspension_force: f32,
    /// Tire forces from the last step (N).
    pub longitudinal_force: f32,
    pub lateral_force: f32,
    /// Slip ratio |w r - v| / |v|.
    pub longitudinal_slip: f32,
    /// Slip angle in radians.
    pub lateral_slip: f32,
    pub angular_velocity: f32,
    pub rotation_angle: f32,
    pub steer_angle: f32,
}

impl VehicleWheelState {
    /// Swift's `init()`. Every Swift initializer value is zero or `false`, so `Default` is derived.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// Drivetrain snapshot.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct VehicleDrivetrainState {
    pub engine_rpm: f32,
    /// Current gear: 0 neutral, negative reverse.
    pub current_gear: i32,
    pub clutch_friction: f32,
    pub is_switching_gear: bool,
    /// Lean angle of a motorcycle, radians.
    pub lean_angle: f32,
    /// Track angular velocities for tracked vehicles.
    pub track_angular_velocity: Vec2,
}

impl VehicleDrivetrainState {
    /// Swift's `init()`. Every Swift initializer value is zero or `false`, so `Default` is derived.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}
