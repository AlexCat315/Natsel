//! Engine, gearbox, differentials, and tracks. The drivetrain owns the rotational degrees of
//! freedom of the engine, the wheels, and the tracks. Ground contact couples them to the chassis
//! in the vehicle solver; here they are coupled to each other.
//!
//! Translated from `Sources/Natsel/Vehicle/VehicleDrivetrain.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md):
//! - `crate::vehicle::vehicle_types`: the definition types from `VehicleTypes.swift` —
//!   `VehicleEngineSettings` (`max_torque`, `min_rpm`, `max_rpm`, `normalized_torque: LinearCurve`,
//!   `inertia`, `angular_damping`), `VehicleTransmissionSettings` (`mode: TransmissionMode`,
//!   `gear_ratios: Vec<f32>`, `reverse_gear_ratios: Vec<f32>`, `switch_time`,
//!   `clutch_release_time`, `switch_latency`, `shift_up_rpm`, `shift_down_rpm`,
//!   `clutch_strength`), `TransmissionMode::{Automatic, Manual}` (`PartialEq`),
//!   `VehicleDifferentialSettings`, `WheeledVehicleControllerSettings`,
//!   `MotorcycleControllerSettings` (`Default` from Swift's `= MotorcycleControllerSettings()`),
//!   `VehicleTrackSettings` (`driven_wheel: i32`, `wheels: Vec<i32>`, `inertia`,
//!   `angular_damping`, `max_brake_torque`, `differential_ratio`), and
//!   `VehicleControllerSettings::{Wheeled(..), Motorcycle(..), Tracked(..)}` tuple payloads
//!   (payloads accessed as the Swift members: `.engine`, `.transmission`, `.differentials`,
//!   `.differential_limited_slip_ratio`, `.wheeled`, `.tracks`).
//!   `LinearCurve::evaluate(&self, x: f32) -> f32`.
//! - `crate::vehicle::vehicle_types::WheelSettings` (`radius`, `max_steer_angle`, `position`).
//! - `crate::simulation::solver_integration::Softness` with `Default` (Swift's `Softness()`).

use crate::base::constants as b3;
use crate::math::functions::{abs_float, clamp_float, clamp_int, dot, max_float, normalize};
use crate::math::types::Vec3;
use crate::simulation::solver_integration::Softness;
use crate::vehicle::vehicle_types::{
    MotorcycleControllerSettings, TransmissionMode, VehicleControllerSettings,
    VehicleDifferentialSettings, VehicleEngineSettings, VehicleTrackSettings,
    VehicleTransmissionSettings, WheelSettings,
};

pub(crate) const RPM_TO_ANGULAR_VELOCITY: f32 = b3::TWO_PI / 60.0;
pub(crate) const ANGULAR_VELOCITY_TO_RPM: f32 = 60.0 / b3::TWO_PI;

// MARK: - Engine

#[derive(Clone, Debug)]
pub(crate) struct VehicleEngine {
    pub(crate) settings: VehicleEngineSettings,

    /// Crankshaft angular velocity (rad/s).
    pub(crate) angular_velocity: f32,
}

impl VehicleEngine {
    pub(crate) fn new(settings: VehicleEngineSettings) -> Self {
        let angular_velocity = settings.min_rpm * RPM_TO_ANGULAR_VELOCITY;
        Self {
            settings,
            angular_velocity,
        }
    }

    pub(crate) fn rpm(&self) -> f32 {
        self.angular_velocity * ANGULAR_VELOCITY_TO_RPM
    }

    /// Torque available at the current speed for a throttle in [0, 1].
    pub(crate) fn torque(&self, throttle: f32) -> f32 {
        let span = self.settings.max_rpm - self.settings.min_rpm;
        let normalized = if span > 0.0 {
            clamp_float((self.rpm() - self.settings.min_rpm) / span, 0.0, 1.0)
        } else {
            0.0
        };
        throttle * self.settings.max_torque * self.settings.normalized_torque.evaluate(normalized)
    }

    pub(crate) fn inverse_inertia(&self) -> f32 {
        if self.settings.inertia > 0.0 {
            1.0 / self.settings.inertia
        } else {
            0.0
        }
    }

    /// Keep the engine between idle and red line.
    pub(crate) fn clamp_speed(&mut self) {
        let lower = self.settings.min_rpm * RPM_TO_ANGULAR_VELOCITY;
        let upper = self.settings.max_rpm * RPM_TO_ANGULAR_VELOCITY;
        self.angular_velocity = clamp_float(self.angular_velocity, lower, upper);
    }
}

// MARK: - Transmission

#[derive(Clone, Debug)]
pub(crate) struct VehicleTransmission {
    pub(crate) settings: VehicleTransmissionSettings,

    /// 0 neutral, positive forward gears, negative reverse gears.
    pub(crate) current_gear: i32,
    pub(crate) clutch_friction: f32,
    pub(crate) gear_switch_time_left: f32,
    pub(crate) clutch_release_time_left: f32,
    pub(crate) gear_switch_latency_time_left: f32,
}

impl VehicleTransmission {
    pub(crate) fn new(settings: VehicleTransmissionSettings) -> Self {
        Self {
            settings,
            current_gear: 0,
            clutch_friction: 1.0,
            gear_switch_time_left: 0.0,
            clutch_release_time_left: 0.0,
            gear_switch_latency_time_left: 0.0,
        }
    }

    pub(crate) fn is_switching_gear(&self) -> bool {
        self.gear_switch_time_left > 0.0
    }

    /// Gear ratio between engine and transmission output. Zero in neutral.
    pub(crate) fn current_ratio(&self) -> f32 {
        if self.current_gear > 0 {
            let index = self.current_gear as usize - 1;
            if index < self.settings.gear_ratios.len() {
                return self.settings.gear_ratios[index];
            }
        }
        if self.current_gear < 0 {
            let index = (-self.current_gear) as usize - 1;
            if index < self.settings.reverse_gear_ratios.len() {
                return self.settings.reverse_gear_ratios[index];
            }
        }
        0.0
    }

    /// Manual control. Swift's `set(gear:clutchFriction:)`.
    pub(crate) fn set(&mut self, gear: i32, clutch_friction: f32) {
        let forward_count = self.settings.gear_ratios.len() as i32;
        let reverse_count = self.settings.reverse_gear_ratios.len() as i32;
        self.current_gear = clamp_int(gear, -reverse_count, forward_count);
        self.clutch_friction = clamp_float(clutch_friction, 0.0, 1.0);
    }

    /// Automatic gear selection and clutch release.
    pub(crate) fn update(
        &mut self,
        dt: f32,
        engine_rpm: f32,
        forward_input: f32,
        can_shift_up: bool,
    ) {
        if self.settings.mode != TransmissionMode::Automatic {
            return;
        }

        let old_gear = self.current_gear;
        let forward_count = self.settings.gear_ratios.len() as i32;
        let reverse_count = self.settings.reverse_gear_ratios.len() as i32;

        if self.current_gear == 0 || forward_input * (self.current_gear as f32) < 0.0 {
            // From neutral, or reversing direction: pick first or reverse from the input.
            if forward_input > 0.0 {
                self.current_gear = if forward_count > 0 { 1 } else { 0 };
            } else if forward_input < 0.0 {
                self.current_gear = if reverse_count > 0 { -1 } else { 0 };
            } else {
                self.current_gear = 0;
            }
        } else if self.gear_switch_latency_time_left == 0.0 {
            if can_shift_up && engine_rpm > self.settings.shift_up_rpm {
                if self.current_gear < 0 {
                    if self.current_gear > -reverse_count {
                        self.current_gear -= 1;
                    }
                } else if self.current_gear < forward_count {
                    self.current_gear += 1;
                }
            } else if engine_rpm < self.settings.shift_down_rpm {
                if self.current_gear < 0 {
                    // Without throttle the gearbox goes to neutral so the engine can idle.
                    let highest: i32 = if forward_input != 0.0 { -1 } else { 0 };
                    if self.current_gear < highest {
                        self.current_gear += 1;
                    }
                } else {
                    let lowest: i32 = if forward_input != 0.0 { 1 } else { 0 };
                    if self.current_gear > lowest {
                        self.current_gear -= 1;
                    }
                }
            }
        }

        if old_gear != self.current_gear {
            // Leaving neutral is instant, everything else takes the switch time.
            self.gear_switch_time_left = if old_gear != 0 { self.settings.switch_time } else { 0.0 };
            self.clutch_release_time_left = self.settings.clutch_release_time;
            self.gear_switch_latency_time_left = self.settings.switch_latency;
            self.clutch_friction = 0.0;
        } else if self.gear_switch_time_left > 0.0 {
            self.gear_switch_time_left = max_float(0.0, self.gear_switch_time_left - dt);
            self.clutch_friction = 0.0;
        } else if self.clutch_release_time_left > 0.0 {
            self.clutch_release_time_left = max_float(0.0, self.clutch_release_time_left - dt);
            let release = self.settings.clutch_release_time;
            self.clutch_friction = if release > 0.0 {
                1.0 - self.clutch_release_time_left / release
            } else {
                1.0
            };
        } else {
            self.clutch_friction = 1.0;
            self.gear_switch_latency_time_left =
                max_float(0.0, self.gear_switch_latency_time_left - dt);
        }
    }
}

// MARK: - Torque bias

/// Torque fraction for the slower and faster output of a limited slip coupling. Below the ratio
/// limit the base split is kept; at the limit all torque goes to the slower output.
///
/// Swift's labeled tuple `(left: Float, right: Float)` becomes the plain tuple `(f32, f32)`;
/// `.left`/`.right` map to `.0`/`.1`.
#[inline(always)]
pub(crate) fn limited_slip_fractions(
    base_left: f32,
    left_speed: f32,
    right_speed: f32,
    limited_slip_ratio: f32,
) -> (f32, f32) {
    let base_right = 1.0 - base_left;
    if !(limited_slip_ratio < f32::MAX && limited_slip_ratio > 1.0) {
        return (base_left, base_right);
    }

    let omega_l = max_float(1.0e-3, abs_float(left_speed));
    let omega_r = max_float(1.0e-3, abs_float(right_speed));
    if omega_l > omega_r {
        let t = clamp_float((omega_l / omega_r - 1.0) / (limited_slip_ratio - 1.0), 0.0, 1.0);
        let left = base_left * (1.0 - t);
        (left, 1.0 - left)
    } else if omega_r > omega_l {
        let t = clamp_float((omega_r / omega_l - 1.0) / (limited_slip_ratio - 1.0), 0.0, 1.0);
        let right = base_right * (1.0 - t);
        (1.0 - right, right)
    } else {
        (base_left, base_right)
    }
}

// MARK: - Drivetrain

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum VehicleControllerKind {
    Wheeled,
    Motorcycle,
    Tracked,
}

/// Scalar track settings; the wheel list lives in `VehicleDrivetrain.track_wheels`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub(crate) struct TrackParameters {
    pub(crate) driven_wheel: i32,
    pub(crate) inertia: f32,
    pub(crate) angular_damping: f32,
    pub(crate) max_brake_torque: f32,
    pub(crate) differential_ratio: f32,
}

impl TrackParameters {
    pub(crate) fn new(s: &VehicleTrackSettings) -> Self {
        Self {
            driven_wheel: s.driven_wheel,
            inertia: s.inertia,
            angular_damping: s.angular_damping,
            max_brake_torque: s.max_brake_torque,
            differential_ratio: s.differential_ratio,
        }
    }
}

/// Track runtime state.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct VehicleTrack {
    pub(crate) settings: TrackParameters,
    pub(crate) angular_velocity: f32,
    pub(crate) brake_impulse: f32,
    pub(crate) max_brake_impulse: f32,
    /// Radius of the driven wheel; track speed = angular_velocity * driven_radius.
    pub(crate) driven_radius: f32,
}

impl VehicleTrack {
    /// Swift's `VehicleTrack(settings:)`.
    pub(crate) fn new(settings: TrackParameters) -> Self {
        Self {
            settings,
            angular_velocity: 0.0,
            brake_impulse: 0.0,
            max_brake_impulse: 0.0,
            driven_radius: 1.0,
        }
    }

    pub(crate) fn inverse_inertia(&self) -> f32 {
        if self.settings.inertia > 0.0 {
            1.0 / self.settings.inertia
        } else {
            0.0
        }
    }
}

/// Motorcycle lean runtime state. Not `Copy`: `settings` holds the wheeled controller's arrays.
#[derive(Clone, Debug)]
pub(crate) struct VehicleLean {
    pub(crate) settings: MotorcycleControllerSettings,
    pub(crate) target_lean: f32,
    pub(crate) current_lean: f32,
    pub(crate) integral: f32,
    pub(crate) wheel_base: f32,
    pub(crate) front_wheel: i32,

    // Solver data
    pub(crate) active: bool,
    pub(crate) axis: Vec3,
    pub(crate) mass: f32,
    pub(crate) position: f32,
    pub(crate) softness: Softness,
    pub(crate) impulse: f32,
}

/// Swift's property initializers, including `settings = MotorcycleControllerSettings()`.
impl Default for VehicleLean {
    #[inline]
    fn default() -> Self {
        Self {
            settings: MotorcycleControllerSettings::default(),
            target_lean: 0.0,
            current_lean: 0.0,
            integral: 0.0,
            wheel_base: 0.0,
            front_wheel: 0,
            active: false,
            axis: Vec3::default(),
            mass: 0.0,
            position: 0.0,
            softness: Softness::default(),
            impulse: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub(crate) struct VehicleDrivetrain {
    pub(crate) kind: VehicleControllerKind,
    pub(crate) engine: VehicleEngine,
    pub(crate) transmission: VehicleTransmission,
    pub(crate) differentials: Vec<VehicleDifferentialSettings>,
    pub(crate) differential_limited_slip_ratio: f32,
    pub(crate) tracks: Vec<VehicleTrack>,
    pub(crate) track_wheels: Vec<Vec<i32>>,
    pub(crate) lean: VehicleLean,

    /// The automatic gearbox opened the clutch for this step.
    pub(crate) clutch_open: bool,

    /// Wheel index -> track index for tracked vehicles, -1 otherwise.
    pub(crate) wheel_track: Vec<i32>,
}

impl VehicleDrivetrain {
    pub(crate) fn new(settings: VehicleControllerSettings, wheels: &[WheelSettings]) -> Self {
        let mut kind = VehicleControllerKind::Wheeled;
        let mut tracks: Vec<VehicleTrack> = Vec::new();
        let mut track_wheels: Vec<Vec<i32>> = Vec::new();
        let mut lean = VehicleLean::default();
        let engine;
        let transmission;
        let differentials;
        // Swift's field default `differentialLimitedSlipRatio = 1.4`; the tracked branch never
        // assigns it.
        let differential_limited_slip_ratio;

        match settings {
            VehicleControllerSettings::Wheeled(s) => {
                kind = VehicleControllerKind::Wheeled;
                engine = VehicleEngine::new(s.engine);
                transmission = VehicleTransmission::new(s.transmission);
                differentials = s.differentials;
                differential_limited_slip_ratio = s.differential_limited_slip_ratio;
            }
            VehicleControllerSettings::Motorcycle(s) => {
                kind = VehicleControllerKind::Motorcycle;
                // `s` itself is stored in `lean.settings` afterwards, so the wheeled parts are
                // copied rather than moved (Swift's COW copies on read).
                engine = VehicleEngine::new(s.wheeled.engine.clone());
                transmission = VehicleTransmission::new(s.wheeled.transmission.clone());
                differentials = s.wheeled.differentials.clone();
                differential_limited_slip_ratio = s.wheeled.differential_limited_slip_ratio;
                lean.settings = s;
            }
            VehicleControllerSettings::Tracked(s) => {
                kind = VehicleControllerKind::Tracked;
                engine = VehicleEngine::new(s.engine);
                transmission = VehicleTransmission::new(s.transmission);
                differentials = Vec::new();
                differential_limited_slip_ratio = 1.4;
                tracks = s
                    .tracks
                    .iter()
                    .map(|t| VehicleTrack::new(TrackParameters::new(t)))
                    .collect();
                track_wheels = s.tracks.into_iter().map(|t| t.wheels).collect();
            }
        }

        let mut wheel_track = vec![-1i32; wheels.len()];
        for t in 0..tracks.len() {
            for &w in track_wheels[t].iter() {
                if w >= 0 && (w as usize) < wheels.len() {
                    wheel_track[w as usize] = t as i32;
                }
            }
            let driven = tracks[t].settings.driven_wheel;
            if driven >= 0 && (driven as usize) < wheels.len() {
                tracks[t].driven_radius =
                    max_float(wheels[driven as usize].radius, b3::linear_slop());
            }
        }

        let mut drivetrain = Self {
            kind,
            engine,
            transmission,
            differentials,
            differential_limited_slip_ratio,
            tracks,
            track_wheels,
            lean,
            clutch_open: false,
            wheel_track,
        };

        if kind == VehicleControllerKind::Motorcycle {
            drivetrain.configure_lean_geometry(wheels);
        }
        drivetrain
    }

    /// The steered wheel is the front; the wheel base is the forward distance to the farthest
    /// non-steered wheel.
    pub(crate) fn configure_lean_geometry(&mut self, wheels: &[WheelSettings]) {
        let mut front: i32 = 0;
        let mut best_steer: f32 = -1.0;
        for (i, w) in wheels.iter().enumerate() {
            if w.max_steer_angle > best_steer {
                best_steer = w.max_steer_angle;
                front = i as i32;
            }
        }
        let mut wheel_base: f32 = 0.0;
        if (front as usize) < wheels.len() {
            let f = &wheels[front as usize];
            let forward = normalize(f.wheel_forward);
            for (i, w) in wheels.iter().enumerate() {
                if i as i32 != front {
                    wheel_base =
                        max_float(wheel_base, abs_float(dot(f.position - w.position, forward)));
                }
            }
        }
        self.lean.front_wheel = front;
        self.lean.wheel_base = wheel_base;
    }

    pub(crate) fn is_driven(&self) -> bool {
        if self.kind != VehicleControllerKind::Tracked {
            !self.differentials.is_empty()
        } else {
            !self.tracks.is_empty()
        }
    }
}
