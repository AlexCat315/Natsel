//! Vehicle storage, creation, input, and inspection.
//!
//! Translated from `Sources/Natsel/Vehicle/VehicleRuntime.swift`.
//!
//! This file owns the vehicle store (`World::vehicles`, declared in
//! `crate::simulation::world_runtime`), the per-wheel runtime state, and the world-facing vehicle
//! API. Ray/sphere contact casting lives in `vehicle::vehicle_collision` and the per-substep
//! solving in `vehicle::vehicle_solver`; the settings types live in `vehicle::vehicle_types` and
//! the drivetrain in `vehicle::vehicle_drivetrain`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::vehicle::vehicle_types`: `VehicleID` (`index1: i32`, `world0: u16`, `generation: u16`,
//!   `VehicleID::new(index1, world0, generation)`, `VehicleID::default()` for Swift's `VehicleID()`),
//!   `WheelSettings`, `VehicleDef` (its `Default` mirrors Swift's `init() {}`), `VehicleAntiRollBar`,
//!   `VehicleEngineSettings`, `VehicleTransmissionSettings`, `VehicleDifferentialSettings`,
//!   `VehicleCollisionTester` (Swift's `.ray` becomes `VehicleCollisionTester::Ray`),
//!   `TireFrictionCallback` (Swift's closure typealias; an `Arc<dyn Fn(i32, f32, f32, f32, u64)
//!   -> (f32, f32) + Send + Sync>` following the `types::world_def` callback aliases),
//!   `VehicleWheelState`, `VehicleDrivetrainState`. Swift's `Int32`/`Float`/`UInt16`/`UInt64` map to
//!   `i32`/`f32`/`u16`/`u64`.
//! - `crate::vehicle::vehicle_drivetrain`: `VehicleDrivetrain` with the `kind`, `engine`,
//!   `transmission`, `differentials`, `tracks`, `lean` fields, `VehicleDrivetrain::new(
//!   &VehicleControllerSettings, &[WheelSettings])` for Swift's `init(_ settings:, wheels:)`,
//!   `configure_lean_geometry(&mut self, &[WheelSettings])`, and
//!   `VehicleControllerKind::{Wheeled, Motorcycle, Tracked}`. `VehicleEngine` carries the stored
//!   `settings` field, the `clamp_speed()` method and the `rpm()` getter; `VehicleTransmission`
//!   carries `settings`, `current_gear`, `clutch_friction`, the `set(gear, clutch_friction)` method
//!   and the `is_switching_gear()` getter; `VehicleTrack::angular_velocity` and
//!   `VehicleLean::current_lean` are stored fields.
//! - `crate::types::shape_def::QueryFilter::default_filter()` for Swift's
//!   `QueryFilter.defaultFilter`.
//!
//! Swift value semantics preserved: `VehicleWheel` and `WheelParameters` were `BitwiseCopyable`, so
//! they are `Copy`; `VehicleRecord` holds arrays and a shared callback, so it is `Clone` only (the
//! `WorldState` snapshot clones it, and derives `Debug` for the store, hence the hand-written
//! `Debug` impl in the `types::debug_draw::DebugDraw` style below). Swift's `[VehicleWheel]` becomes
//! `Vec<VehicleWheel>`.
//!
//! Swift read `vehicles[i].wheelSettings` while mutating `vehicles[i].drivetrain` inside
//! `setVehicleWheelSettings`; that overlapping access has no Rust equivalent, so the settings are
//! cloned first (only in the motorcycle branch) and the clone is passed.
//!
//! `normalized_wheel_settings` takes `&WheelSettings` and clones into a local, mirroring Swift's
//! by-value parameter and `var s = settings` copy.

use std::ffi::c_void;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::math::functions::{
    clamp_float, compute_cos_sin, cross, dot, is_valid_vec3, length_squared,
    make_quat_from_axis_angle, make_quat_from_matrix, mul_add, mul_mm, mul_quat,
    mul_world_transforms, normalize, normalize_quat, rotate_vector, transpose,
};
use crate::math::types::{Matrix3, Pos, Transform, Vec2, Vec3, WorldTransform};
use crate::simulation::solver_integration::Softness;
use crate::simulation::world_runtime::{SetType, World};
use crate::types::body_def::BodyType;
use crate::types::ids::{BodyID, ShapeID};
use crate::types::shape_def::QueryFilter;
use crate::vehicle::vehicle_drivetrain::{VehicleControllerKind, VehicleDrivetrain};
use crate::vehicle::vehicle_types::{
    TireFrictionCallback, VehicleAntiRollBar, VehicleCollisionTester, VehicleDef,
    VehicleDifferentialSettings, VehicleDrivetrainState, VehicleEngineSettings, VehicleID,
    VehicleTransmissionSettings, VehicleWheelState, WheelSettings,
};

/// The scalar wheel settings used by the solver. Plain data, so copying a wheel never touches
/// reference counts; the tire curves live in `VehicleRecord::wheel_settings`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct WheelParameters {
    pub position: Vec3,
    pub suspension_force_point: Vec3,
    pub enable_suspension_force_point: bool,
    pub suspension_direction: Vec3,
    pub steering_axis: Vec3,
    pub wheel_up: Vec3,
    pub wheel_forward: Vec3,
    pub suspension_min_length: f32,
    pub suspension_max_length: f32,
    pub suspension_preload_length: f32,
    pub suspension_frequency: f32,
    pub suspension_damping_ratio: f32,
    pub suspension_stiffness: f32,
    pub suspension_damping: f32,
    pub radius: f32,
    pub width: f32,
    pub inertia: f32,
    pub angular_damping: f32,
    pub max_steer_angle: f32,
    pub max_brake_torque: f32,
    pub max_hand_brake_torque: f32,
    pub track_longitudinal_friction: f32,
    pub track_lateral_friction: f32,
}

impl WheelParameters {
    #[inline]
    pub(crate) fn new(s: &WheelSettings) -> Self {
        Self {
            position: s.position,
            suspension_force_point: s.suspension_force_point,
            enable_suspension_force_point: s.enable_suspension_force_point,
            suspension_direction: s.suspension_direction,
            steering_axis: s.steering_axis,
            wheel_up: s.wheel_up,
            wheel_forward: s.wheel_forward,
            suspension_min_length: s.suspension_min_length,
            suspension_max_length: s.suspension_max_length,
            suspension_preload_length: s.suspension_preload_length,
            suspension_frequency: s.suspension_frequency,
            suspension_damping_ratio: s.suspension_damping_ratio,
            suspension_stiffness: s.suspension_stiffness,
            suspension_damping: s.suspension_damping,
            radius: s.radius,
            width: s.width,
            inertia: s.inertia,
            angular_damping: s.angular_damping,
            max_steer_angle: s.max_steer_angle,
            max_brake_torque: s.max_brake_torque,
            max_hand_brake_torque: s.max_hand_brake_torque,
            track_longitudinal_friction: s.track_longitudinal_friction,
            track_lateral_friction: s.track_lateral_friction,
        }
    }
}

/// Runtime state of one wheel: collision result, rotational state, and per-step solver data.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct VehicleWheel {
    pub settings: WheelParameters,

    // Rotation
    pub angular_velocity: f32,
    pub rotation_angle: f32,
    pub steer_angle: f32,

    // Collision
    pub has_contact: bool,
    pub contact_body: i32,
    pub contact_body_id: BodyID,
    pub contact_shape_id: ShapeID,
    pub contact_position: Pos,
    pub contact_normal: Vec3,
    pub longitudinal: Vec3,
    pub lateral: Vec3,
    pub suspension_length: f32,
    pub surface_friction: f32,
    pub user_material_id: u64,
    pub longitudinal_slip: f32,
    pub lateral_slip: f32,
    pub longitudinal_friction: f32,
    pub lateral_friction: f32,

    // Solver
    pub ground_index: i32,
    pub ground_inv_mass: f32,
    pub ground_inv_i: Matrix3,
    /// Chassis center of mass to the tire contact.
    pub r_a: Vec3,
    /// Chassis center of mass to the suspension force point.
    pub r_as: Vec3,
    /// Ground center of mass to the tire contact.
    pub r_b: Vec3,
    pub suspension_position: f32,
    pub min_length_position: f32,
    pub suspension_mass: f32,
    pub longitudinal_body_k: f32,
    pub lateral_mass: f32,
    pub suspension_softness: Softness,
    pub suspension_impulse: f32,
    pub stop_impulse: f32,
    pub longitudinal_impulse: f32,
    pub lateral_impulse: f32,
    pub brake_impulse: f32,
    pub max_brake_impulse: f32,
    /// Share of the clutch shaft speed, rebuilt every substep.
    pub clutch_coefficient: f32,

    // Last step totals for inspection
    pub total_suspension_impulse: f32,
    pub total_longitudinal_impulse: f32,
    pub total_lateral_impulse: f32,
}

impl VehicleWheel {
    /// Swift's `init(_ settings:)`: every property default applies first, then `settings` and
    /// `suspensionLength = settings.suspensionMaxLength`. The two `B3.nullIndex` indices mean the
    /// derived all-zeroes default would be wrong, so this is the only constructor.
    #[inline]
    pub(crate) fn new(settings: &WheelSettings) -> Self {
        Self {
            settings: WheelParameters::new(settings),

            // Rotation
            angular_velocity: 0.0,
            rotation_angle: 0.0,
            steer_angle: 0.0,

            // Collision
            has_contact: false,
            contact_body: b3::NULL_INDEX,
            contact_body_id: BodyID::default(),
            contact_shape_id: ShapeID::default(),
            contact_position: Pos::default(),
            contact_normal: Vec3::default(),
            longitudinal: Vec3::default(),
            lateral: Vec3::default(),
            suspension_length: settings.suspension_max_length,
            surface_friction: 0.0,
            user_material_id: 0,
            longitudinal_slip: 0.0,
            lateral_slip: 0.0,
            longitudinal_friction: 0.0,
            lateral_friction: 0.0,

            // Solver
            ground_index: b3::NULL_INDEX,
            ground_inv_mass: 0.0,
            ground_inv_i: Matrix3::default(),
            r_a: Vec3::default(),
            r_as: Vec3::default(),
            r_b: Vec3::default(),
            suspension_position: 0.0,
            min_length_position: 0.0,
            suspension_mass: 0.0,
            longitudinal_body_k: 0.0,
            lateral_mass: 0.0,
            suspension_softness: Softness::default(),
            suspension_impulse: 0.0,
            stop_impulse: 0.0,
            longitudinal_impulse: 0.0,
            lateral_impulse: 0.0,
            brake_impulse: 0.0,
            max_brake_impulse: 0.0,
            clutch_coefficient: 0.0,

            // Last step totals for inspection
            total_suspension_impulse: 0.0,
            total_longitudinal_impulse: 0.0,
            total_lateral_impulse: 0.0,
        }
    }

    #[inline]
    pub(crate) fn inverse_inertia(&self) -> f32 {
        if self.settings.inertia > 0.0 {
            1.0 / self.settings.inertia
        } else {
            0.0
        }
    }
}

pub(crate) struct VehicleRecord {
    pub id: i32,
    pub generation: u16,
    pub user_data: Option<*mut c_void>,

    pub body_id: BodyID,
    pub local_up: Vec3,
    pub local_forward: Vec3,
    pub world_up: Option<Vec3>,
    pub max_pitch_roll_angle: f32,

    pub wheels: Vec<VehicleWheel>,
    /// Full wheel settings including the tire curves.
    pub wheel_settings: Vec<WheelSettings>,
    pub anti_roll_bars: Vec<VehicleAntiRollBar>,
    pub drivetrain: VehicleDrivetrain,
    pub tester: VehicleCollisionTester,
    pub filter: QueryFilter,
    pub cos_max_slope_angle: f32,
    pub tire_friction: Option<TireFrictionCallback>,

    // Driver input
    pub forward_input: f32,
    pub right_input: f32,
    pub brake_input: f32,
    pub hand_brake_input: f32,
    pub left_track_ratio: f32,
    pub right_track_ratio: f32,

    // Solver
    pub active: bool,
    pub chassis_index: i32,
    pub inv_mass: f32,
    pub inv_i: Matrix3,
    pub step_up: Vec3,
    pub pitch_roll_active: bool,
    pub pitch_roll_axis: Vec3,
    pub pitch_roll_mass: f32,
    pub pitch_roll_chassis_up: Vec3,
    pub pitch_roll_impulse: f32,
    /// Throttle after automatic brake-on-reverse handling.
    pub effective_throttle: f32,
    pub effective_brake: f32,
}

impl std::fmt::Debug for VehicleRecord {
    /// Swift had no `Debug` conformance; `WorldState`'s derived `Debug` needs the store to be
    /// `Debug`, and the tire friction callback (an `Arc<dyn Fn>`) is not printable.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("VehicleRecord")
            .field("id", &self.id)
            .field("generation", &self.generation)
            .field("body_id", &self.body_id)
            .field("wheel_count", &self.wheels.len())
            .field("active", &self.active)
            .field(
                "tire_friction",
                &self.tire_friction.as_ref().map(|_| "<callback>"),
            )
            .finish_non_exhaustive()
    }
}

impl Clone for VehicleRecord {
    /// Swift copied the record by value; the shared tire friction callback and the chassis user
    /// pointer keep the same identity, matching Swift's reference semantics for the closure.
    fn clone(&self) -> Self {
        Self {
            id: self.id,
            generation: self.generation,
            user_data: self.user_data,

            body_id: self.body_id,
            local_up: self.local_up,
            local_forward: self.local_forward,
            world_up: self.world_up,
            max_pitch_roll_angle: self.max_pitch_roll_angle,

            wheels: self.wheels.clone(),
            wheel_settings: self.wheel_settings.clone(),
            anti_roll_bars: self.anti_roll_bars.clone(),
            drivetrain: self.drivetrain.clone(),
            tester: self.tester.clone(),
            filter: self.filter.clone(),
            cos_max_slope_angle: self.cos_max_slope_angle,
            tire_friction: self.tire_friction.clone(),

            forward_input: self.forward_input,
            right_input: self.right_input,
            brake_input: self.brake_input,
            hand_brake_input: self.hand_brake_input,
            left_track_ratio: self.left_track_ratio,
            right_track_ratio: self.right_track_ratio,

            active: self.active,
            chassis_index: self.chassis_index,
            inv_mass: self.inv_mass,
            inv_i: self.inv_i,
            step_up: self.step_up,
            pitch_roll_active: self.pitch_roll_active,
            pitch_roll_axis: self.pitch_roll_axis,
            pitch_roll_mass: self.pitch_roll_mass,
            pitch_roll_chassis_up: self.pitch_roll_chassis_up,
            pitch_roll_impulse: self.pitch_roll_impulse,
            effective_throttle: self.effective_throttle,
            effective_brake: self.effective_brake,
        }
    }
}

impl VehicleRecord {
    /// A free slot, also used as a placeholder while a record is moved out of the array.
    /// Swift's `nonisolated(unsafe) static let empty`; a record owns `Vec`s and a shared callback,
    /// so it is a function call rather than a `const`.
    #[inline]
    pub(crate) fn empty() -> Self {
        VehicleRecord::new(&VehicleDef::default())
    }

    pub(crate) fn new(def: &VehicleDef) -> Self {
        let wheel_settings: Vec<WheelSettings> =
            def.wheels.iter().map(normalized_wheel_settings).collect();
        let wheels: Vec<VehicleWheel> = wheel_settings
            .iter()
            .map(VehicleWheel::new)
            .collect();
        let drivetrain = VehicleDrivetrain::new(&def.controller, &def.wheels);

        Self {
            id: b3::NULL_INDEX,
            generation: 0,
            user_data: def.user_data,

            body_id: def.body_id,
            local_up: normalize(def.up),
            local_forward: normalize(def.forward),
            world_up: None,
            max_pitch_roll_angle: def.max_pitch_roll_angle,

            wheels,
            wheel_settings,
            anti_roll_bars: def.anti_roll_bars.clone(),
            drivetrain,
            tester: def.collision_tester.clone(),
            filter: def.filter.clone(),
            cos_max_slope_angle: compute_cos_sin(clamp_float(
                def.max_slope_angle,
                0.0,
                b3::PI,
            ))
            .cosine,
            tire_friction: None,

            forward_input: 0.0,
            right_input: 0.0,
            brake_input: 0.0,
            hand_brake_input: 0.0,
            left_track_ratio: 1.0,
            right_track_ratio: 1.0,

            active: false,
            chassis_index: b3::NULL_INDEX,
            inv_mass: 0.0,
            inv_i: Matrix3::default(),
            step_up: Vec3::new(0.0, 1.0, 0.0),
            pitch_roll_active: false,
            pitch_roll_axis: Vec3::default(),
            pitch_roll_mass: 0.0,
            pitch_roll_chassis_up: Vec3::default(),
            pitch_roll_impulse: 0.0,
            effective_throttle: 0.0,
            effective_brake: 0.0,
        }
    }
}

impl World {
    // MARK: Creation

    /// Create a vehicle constraint on a dynamic chassis. Returns a null id on invalid input.
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_vehicle(&mut self, def: &VehicleDef) -> VehicleID {
        b3_assert(!self.locked);
        if !(self.is_valid_body(def.body_id)
            && self.body(def.body_id).type_ == BodyType::DynamicBody)
        {
            return VehicleID::default();
        }
        if def.wheels.is_empty() {
            return VehicleID::default();
        }

        for wheel in &def.wheels {
            if !(wheel.radius > 0.0
                && wheel.suspension_max_length >= wheel.suspension_min_length
                && wheel.suspension_min_length >= 0.0
                && is_valid_vec3(wheel.position)
                && length_squared(wheel.suspension_direction) > 0.0
                && length_squared(wheel.wheel_forward) > 0.0
                && length_squared(wheel.wheel_up) > 0.0)
            {
                return VehicleID::default();
            }
        }
        let wheel_count = def.wheels.len() as i32;
        for bar in &def.anti_roll_bars {
            if !(bar.left_wheel >= 0
                && bar.left_wheel < wheel_count
                && bar.right_wheel >= 0
                && bar.right_wheel < wheel_count
                && bar.left_wheel != bar.right_wheel)
            {
                return VehicleID::default();
            }
        }

        let mut record = VehicleRecord::new(def);

        let id = self.vehicle_id_pool.alloc();
        if id as usize == self.vehicles.len() {
            self.vehicles.push(record);
        } else {
            record.generation = self.vehicles[id as usize].generation.wrapping_add(1);
            self.vehicles[id as usize] = record;
        }
        self.vehicles[id as usize].id = id;
        VehicleID::new(
            id + 1,
            self.world_id,
            self.vehicles[id as usize].generation,
        )
    }

    /// Destroy a vehicle. The chassis body is not affected.
    pub fn destroy_vehicle(&mut self, vehicle_id: VehicleID) {
        if !self.vehicle_is_valid(vehicle_id) {
            return;
        }
        let index = (vehicle_id.index1 - 1) as usize;
        let generation = self.vehicles[index].generation;
        self.vehicles[index] = VehicleRecord::empty();
        self.vehicles[index].generation = generation;
        self.vehicle_id_pool.free(index as i32);
    }

    pub fn vehicle_is_valid(&self, vehicle_id: VehicleID) -> bool {
        if !(vehicle_id.world0 == self.world_id
            && vehicle_id.index1 >= 1
            && (vehicle_id.index1 as usize) <= self.vehicles.len())
        {
            return false;
        }
        let record = &self.vehicles[(vehicle_id.index1 - 1) as usize];
        record.id != b3::NULL_INDEX && record.generation == vehicle_id.generation
    }

    /// Number of live vehicles.
    pub fn vehicle_count(&self) -> i32 {
        self.vehicle_id_pool.id_count()
    }

    /// Swift's `internal` `vehicleIndex`; `pub(crate)` mirrors Swift's target-wide internal
    /// visibility so the sibling vehicle modules can reach it.
    #[inline(always)]
    pub(crate) fn vehicle_index(&self, vehicle_id: VehicleID) -> Option<usize> {
        if self.vehicle_is_valid(vehicle_id) {
            Some((vehicle_id.index1 - 1) as usize)
        } else {
            None
        }
    }

    // MARK: Input

    /// Driver input for wheeled vehicles and motorcycles.
    /// - forward: -1 full reverse ... 1 full throttle. With the automatic gearbox, input against
    ///   the direction of travel brakes until the vehicle nearly stops.
    /// - right: -1 full left ... 1 full right.
    /// - brake, handBrake: 0 ... 1.
    /// Swift defaulted `brake` and `handBrake` to 0; ported callers pass them explicitly.
    pub fn set_vehicle_driver_input(
        &mut self,
        vehicle_id: VehicleID,
        forward: f32,
        right: f32,
        brake: f32,
        hand_brake: f32,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        self.vehicles[i].forward_input = clamp_float(forward, -1.0, 1.0);
        self.vehicles[i].right_input = clamp_float(right, -1.0, 1.0);
        self.vehicles[i].brake_input = clamp_float(brake, 0.0, 1.0);
        self.vehicles[i].hand_brake_input = clamp_float(hand_brake, 0.0, 1.0);
        self.wake_vehicle_if_driven(i);
    }

    /// Driver input for tracked vehicles. The track ratios in [-1, 1] scale the drive to each track;
    /// opposite signs turn on the spot.
    /// Swift defaulted `brake` to 0; ported callers pass it explicitly.
    pub fn set_tracked_vehicle_driver_input(
        &mut self,
        vehicle_id: VehicleID,
        forward: f32,
        left_ratio: f32,
        right_ratio: f32,
        brake: f32,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        self.vehicles[i].forward_input = clamp_float(forward, -1.0, 1.0);
        self.vehicles[i].left_track_ratio = clamp_float(left_ratio, -1.0, 1.0);
        self.vehicles[i].right_track_ratio = clamp_float(right_ratio, -1.0, 1.0);
        self.vehicles[i].brake_input = clamp_float(brake, 0.0, 1.0);
        self.wake_vehicle_if_driven(i);
    }

    /// Manual gearbox control. Only used with `TransmissionMode.manual`.
    pub fn set_vehicle_transmission(
        &mut self,
        vehicle_id: VehicleID,
        gear: i32,
        clutch_friction: f32,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        self.vehicles[i]
            .drivetrain
            .transmission
            .set(gear, clutch_friction);
    }

    fn wake_vehicle_if_driven(&mut self, index: usize) {
        let record = &self.vehicles[index];
        if !((record.forward_input != 0.0 || record.right_input != 0.0)
            && self.is_valid_body(record.body_id))
        {
            return;
        }
        let body_index = self.body_index(record.body_id);
        if self.bodies[body_index as usize].set_index
            >= SetType::FirstSleepingSet.raw_value()
        {
            self.wake_body(body_index);
        }
    }

    // MARK: Tuning

    /// The chassis body.
    pub fn vehicle_body(&self, vehicle_id: VehicleID) -> Option<BodyID> {
        self.vehicle_index(vehicle_id).map(|i| self.vehicles[i].body_id)
    }

    pub fn vehicle_wheel_count(&self, vehicle_id: VehicleID) -> i32 {
        match self.vehicle_index(vehicle_id) {
            Some(i) => self.vehicles[i].wheels.len() as i32,
            None => 0,
        }
    }

    pub fn vehicle_wheel_settings(
        &self,
        vehicle_id: VehicleID,
        wheel: i32,
    ) -> Option<WheelSettings> {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return None;
        };
        if !(wheel >= 0 && (wheel as usize) < self.vehicles[i].wheels.len()) {
            return None;
        }
        Some(self.vehicles[i].wheel_settings[wheel as usize].clone())
    }

    /// Replace a wheel's settings, for example to tune suspension or tires at run time.
    pub fn set_vehicle_wheel_settings(
        &mut self,
        vehicle_id: VehicleID,
        wheel: i32,
        settings: &WheelSettings,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        if !(wheel >= 0
            && (wheel as usize) < self.vehicles[i].wheels.len()
            && settings.radius > 0.0
            && settings.suspension_max_length >= settings.suspension_min_length)
        {
            return;
        }
        let s = normalized_wheel_settings(settings);
        self.vehicles[i].wheel_settings[wheel as usize] = s.clone();
        self.vehicles[i].wheels[wheel as usize].settings = WheelParameters::new(&s);
        if self.vehicles[i].drivetrain.kind == VehicleControllerKind::Motorcycle {
            // Swift read `vehicles[i].wheelSettings` while mutating `vehicles[i].drivetrain`; the
            // clone keeps that access disjoint here.
            let wheel_settings = self.vehicles[i].wheel_settings.clone();
            self.vehicles[i]
                .drivetrain
                .configure_lean_geometry(&wheel_settings);
        }
    }

    /// Replace the engine settings. The current engine speed is kept within the new range.
    pub fn set_vehicle_engine(
        &mut self,
        vehicle_id: VehicleID,
        settings: &VehicleEngineSettings,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        self.vehicles[i].drivetrain.engine.settings = settings.clone();
        self.vehicles[i].drivetrain.engine.clamp_speed();
    }

    /// Replace the gearbox settings. The current gear is kept when it still exists.
    pub fn set_vehicle_transmission_settings(
        &mut self,
        vehicle_id: VehicleID,
        settings: &VehicleTransmissionSettings,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        let gear = self.vehicles[i].drivetrain.transmission.current_gear;
        let clutch = self.vehicles[i].drivetrain.transmission.clutch_friction;
        self.vehicles[i].drivetrain.transmission.settings = settings.clone();
        self.vehicles[i]
            .drivetrain
            .transmission
            .set(gear, clutch);
    }

    pub fn set_vehicle_differentials(
        &mut self,
        vehicle_id: VehicleID,
        differentials: &[VehicleDifferentialSettings],
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        let count = self.vehicles[i].wheels.len() as i32;
        self.vehicles[i].drivetrain.differentials = differentials
            .iter()
            .filter(|d| d.left_wheel < count && d.right_wheel < count)
            .cloned()
            .collect();
    }

    pub fn set_vehicle_anti_roll_bars(
        &mut self,
        vehicle_id: VehicleID,
        bars: &[VehicleAntiRollBar],
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        let count = self.vehicles[i].wheels.len() as i32;
        self.vehicles[i].anti_roll_bars = bars
            .iter()
            .filter(|b| {
                b.left_wheel >= 0
                    && b.left_wheel < count
                    && b.right_wheel >= 0
                    && b.right_wheel < count
            })
            .cloned()
            .collect();
    }

    /// World up used by the pitch/roll limit and motorcycle lean. Defaults to minus gravity.
    pub fn set_vehicle_world_up(&mut self, vehicle_id: VehicleID, up: Option<Vec3>) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        self.vehicles[i].world_up = up.map(normalize);
    }

    pub fn set_vehicle_max_pitch_roll_angle(
        &mut self,
        vehicle_id: VehicleID,
        angle: f32,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        self.vehicles[i].max_pitch_roll_angle = angle;
    }

    pub fn set_vehicle_collision_tester(
        &mut self,
        vehicle_id: VehicleID,
        tester: VehicleCollisionTester,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        self.vehicles[i].tester = tester;
    }

    pub fn set_vehicle_tire_friction_callback(
        &mut self,
        vehicle_id: VehicleID,
        callback: Option<TireFrictionCallback>,
    ) {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return;
        };
        self.vehicles[i].tire_friction = callback;
    }

    // MARK: Inspection

    pub fn vehicle_wheel_state(
        &self,
        vehicle_id: VehicleID,
        wheel: i32,
    ) -> Option<VehicleWheelState> {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return None;
        };
        if !(wheel >= 0 && (wheel as usize) < self.vehicles[i].wheels.len()) {
            return None;
        }
        let w = self.vehicles[i].wheels[wheel as usize];
        let mut state = VehicleWheelState::default();
        state.has_contact = w.has_contact;
        state.contact_body_id = if w.has_contact {
            w.contact_body_id
        } else {
            BodyID::default()
        };
        state.contact_shape_id = if w.has_contact {
            w.contact_shape_id
        } else {
            ShapeID::default()
        };
        state.contact_position = w.contact_position;
        state.contact_normal = w.contact_normal;
        state.contact_longitudinal = w.longitudinal;
        state.contact_lateral = w.lateral;
        state.suspension_length = w.suspension_length;
        state.suspension_force = w.total_suspension_impulse * self.inv_dt;
        state.longitudinal_force = w.total_longitudinal_impulse * self.inv_dt;
        state.lateral_force = w.total_lateral_impulse * self.inv_dt;
        state.longitudinal_slip = w.longitudinal_slip;
        state.lateral_slip = w.lateral_slip;
        state.angular_velocity = w.angular_velocity;
        state.rotation_angle = w.rotation_angle;
        state.steer_angle = w.steer_angle;
        Some(state)
    }

    pub fn vehicle_drivetrain_state(
        &self,
        vehicle_id: VehicleID,
    ) -> Option<VehicleDrivetrainState> {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return None;
        };
        let d = &self.vehicles[i].drivetrain;
        let mut state = VehicleDrivetrainState::default();
        state.engine_rpm = d.engine.rpm();
        state.current_gear = d.transmission.current_gear;
        state.clutch_friction = d.transmission.clutch_friction;
        state.is_switching_gear = d.transmission.is_switching_gear();
        state.lean_angle = d.lean.current_lean;
        if d.tracks.len() >= 2 {
            state.track_angular_velocity =
                Vec2::new(d.tracks[0].angular_velocity, d.tracks[1].angular_velocity);
        }
        Some(state)
    }

    /// Chassis speed along the vehicle forward direction.
    pub fn vehicle_forward_speed(&self, vehicle_id: VehicleID) -> f32 {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return 0.0;
        };
        if !self.is_valid_body(self.vehicles[i].body_id) {
            return 0.0;
        }
        let record = &self.vehicles[i];
        let forward = rotate_vector(
            self.body_rotation(record.body_id),
            record.local_forward,
        );
        dot(self.body_linear_velocity(record.body_id), forward)
    }

    /// Wheel transform in the chassis frame. The returned rotation maps `wheelRight` to the axle and
    /// `wheelUp` to the wheel up, so a wheel model authored with those axes can be drawn with it.
    /// Swift defaulted `wheelRight` to `(1, 0, 0)` and `wheelUp` to `(0, 1, 0)`; ported callers pass
    /// those axes explicitly.
    pub fn vehicle_wheel_local_transform(
        &self,
        vehicle_id: VehicleID,
        wheel: i32,
        wheel_right: Vec3,
        wheel_up: Vec3,
    ) -> Option<Transform> {
        let Some(i) = self.vehicle_index(vehicle_id) else {
            return None;
        };
        if !(wheel >= 0 && (wheel as usize) < self.vehicles[i].wheels.len()) {
            return None;
        }
        let w = self.vehicles[i].wheels[wheel as usize];
        let s = w.settings;

        // Model basis -> wheel basis (right = axle, up, forward).
        let axle = normalize(cross(s.wheel_up, s.wheel_forward));
        let model_forward = normalize(cross(wheel_right, wheel_up));
        let model = Matrix3::new(
            normalize(wheel_right),
            normalize(wheel_up),
            model_forward,
        );
        let wheel_basis = Matrix3::new(axle, s.wheel_up, s.wheel_forward);
        let basis = make_quat_from_matrix(mul_mm(wheel_basis, transpose(model)));

        let steer = make_quat_from_axis_angle(s.steering_axis, w.steer_angle);
        let spin = make_quat_from_axis_angle(axle, w.rotation_angle);
        let rotation = normalize_quat(mul_quat(steer, mul_quat(spin, basis)));
        let position = mul_add(s.position, w.suspension_length, s.suspension_direction);
        Some(Transform::new(position, rotation))
    }

    /// Wheel transform in world space. See `vehicle_wheel_local_transform`.
    pub fn vehicle_wheel_transform(
        &self,
        vehicle_id: VehicleID,
        wheel: i32,
        wheel_right: Vec3,
        wheel_up: Vec3,
    ) -> Option<WorldTransform> {
        let local = self.vehicle_wheel_local_transform(
            vehicle_id,
            wheel,
            wheel_right,
            wheel_up,
        )?;
        let i = self.vehicle_index(vehicle_id)?;
        if !self.is_valid_body(self.vehicles[i].body_id) {
            return None;
        }
        Some(mul_world_transforms(
            self.body_transform(self.vehicles[i].body_id),
            local,
        ))
    }

    pub fn vehicle_user_data(&self, vehicle_id: VehicleID) -> Option<*mut c_void> {
        self.vehicle_index(vehicle_id)
            .and_then(|i| self.vehicles[i].user_data)
    }
}

/// Unit length directions.
pub(crate) fn normalized_wheel_settings(settings: &WheelSettings) -> WheelSettings {
    let mut s = settings.clone();
    s.suspension_direction = normalize(s.suspension_direction);
    s.steering_axis = normalize(s.steering_axis);
    s.wheel_up = normalize(s.wheel_up);
    s.wheel_forward = normalize(s.wheel_forward);
    s
}
