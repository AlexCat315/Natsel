//! Vehicle constraint solver. Each substep:
//! - warm start: engine torque, drivetrain coupling through the clutch, anti-roll bars, and the
//!   accumulated suspension/tire impulses of the previous substep;
//! - solve (with and without bias): suspension spring and bump stop, brakes, longitudinal and
//!   lateral tire friction inside a friction ellipse, pitch/roll limit, and motorcycle lean.
//!
//! Sign conventions: the contact normal points from the ground toward the chassis. A positive
//! suspension impulse pushes the chassis away from the ground. Positive wheel angular velocity rolls
//! the vehicle forward, so the tire surface moves at `w * r` relative to the chassis.
//!
//! Translated from `Sources/Natsel/Vehicle/VehicleSolver.swift`.
//!
//! Wheel iteration order, the per-wheel solve sequence (suspension spring, bump stop, longitudinal,
//! lateral, friction-ellipse correction), the impulse write-back order into the body states, the
//! slip clamps, and every early-out follow the Swift source line by line; float operand order,
//! association and the explicit `-0.0`/`max(0, x)` choices are preserved so runs stay bit-identical.
//!
//! Foreign vocabulary taken from the sibling vehicle modules (already translated):
//! - `crate::vehicle::vehicle_runtime`: `VehicleRecord` (fields `active`, `chassis_index`,
//!   `inv_mass`, `inv_i`, `step_up`, `wheels`, `wheel_settings`, `anti_roll_bars`, `drivetrain`,
//!   `tire_friction`, `forward_input`, `brake_input`, `hand_brake_input`, `left_track_ratio`,
//!   `right_track_ratio`, `world_up`, `local_up`, `local_forward`, `max_pitch_roll_angle`,
//!   `pitch_roll_*`, `effective_throttle`, `effective_brake`; `Clone` only, `VehicleRecord::empty()`),
//!   `VehicleWheel` (`Copy`, incl. `inverse_inertia()`), `WheelParameters` (`Copy`).
//! - `crate::vehicle::vehicle_drivetrain`: `VehicleDrivetrain` (`kind`, `engine`, `transmission`,
//!   `differentials`, `differential_limited_slip_ratio`, `tracks`, `track_wheels`, `lean`,
//!   `clutch_open`, `wheel_track`), `VehicleEngine` (`torque`, `inverse_inertia`, `clamp_speed`,
//!   `rpm`, `settings`), `VehicleTransmission` (`update`, `current_ratio`, `settings`,
//!   `clutch_friction`), `VehicleTrack` (`inverse_inertia()`, `settings: TrackParameters`),
//!   `VehicleLean` (`Clone`, `settings: MotorcycleControllerSettings`), `VehicleControllerKind`,
//!   `limited_slip_fractions`, `ANGULAR_VELOCITY_TO_RPM`.
//! - `crate::vehicle::vehicle_types`: `TransmissionMode::Automatic`, `LinearCurve`,
//!   `VehicleDifferentialSettings`; Swift's `Float.greatestFiniteMagnitude` is `f32::MAX` as used by
//!   those modules.
//! - the shared `World` anchor: `vehicles`, `vehicle_id_pool`, `bodies`, `solver_sets`, `gravity`,
//!   `body_index`, `SetType::AwakeSet.raw_value()`, and `contact_solver`'s `World::state_at` /
//!   `World::set_state_at` / `World::with_solver_states`.
//!
//! Port notes:
//! - Swift's `UnsafeMutableBufferPointer<BodyState>` parameter becomes `*mut BodyState`, the same
//!   substitution the joint and overflow-contact solvers made; `World::state_at`/`set_state_at` keep
//!   the static-body and dynamic-flag checks, while Swift's *direct* `states[chassisIndex]`
//!   read/write (unconditional) stays a direct pointer read/write so the checks are not added.
//! - Swift's nested `bodyK`, `axleSpeed`, `axleRatio` functions cannot capture in Rust, so they
//!   become nested `fn` items / a closure over the same captured scalars with the reads passed as
//!   explicit parameters; the arithmetic inside is unchanged.
//! - Swift's `defer { swap(...) }` in `prepareVehicle` becomes the explicit swap-back at every exit
//!   path, following the `World::apply_vehicle_steering` precedent in `vehicle_collision.rs`.
//! - Swift's value copies of `VehicleLean` / `VehicleEngine` / `MotorcycleControllerSettings`
//!   (copy-on-write) become `clone()`s of the same values, read or mutated locally and written back
//!   at exactly the Swift assignment points.
//! - Swift's `VehicleRecord.brake(_:)` helper is unused by the Swift solver itself; it is translated
//!   as-is for API parity and marked `allow(dead_code)` for that reason.

use crate::base::constants as b3;
use crate::math::functions::{
    abs_float, atan2_approx, clamp_float, compute_cos_sin, cross, dot, get_length_and_normalize,
    length, length_squared, max_float, min_float, mul_add, mul_mv, mul_sub, normalize,
    rotate_vector, sub_pos, transform_world_point,
};
use crate::math::types::{Matrix3, Vec3};
use crate::simulation::solver_integration::{make_soft, StepContext};
use crate::simulation::world_runtime::{BodySim, BodyState, SetType, World};
use crate::vehicle::vehicle_drivetrain::{
    limited_slip_fractions, ANGULAR_VELOCITY_TO_RPM, VehicleControllerKind, VehicleEngine,
};
use crate::vehicle::vehicle_runtime::{VehicleRecord, VehicleWheel};
use crate::vehicle::vehicle_types::{LinearCurve, TransmissionMode, VehicleDifferentialSettings};

impl World {
    // MARK: Prepare

    pub(crate) fn prepare_vehicles(&mut self, context: &StepContext) {
        if !(self.vehicle_id_pool.id_count() > 0) {
            return;
        }
        for i in 0..self.vehicles.len() {
            if self.vehicles[i].active {
                self.prepare_vehicle(i, context);
            }
        }
    }

    fn prepare_vehicle(&mut self, index: usize, context: &StepContext) {
        let awake = SetType::AwakeSet.raw_value() as usize;
        let mut record = VehicleRecord::empty();
        std::mem::swap(&mut record, &mut self.vehicles[index]);
        // Swift's `defer { swap(&record, &vehicles[index]) }`: the swap-back has to run on every
        // exit path, so it is written out explicitly at each return below.

        let chassis_index = self.body_index(record.body_id);
        let chassis = &self.bodies[chassis_index as usize];
        let chassis_set_index = chassis.set_index;
        let chassis_local_index = chassis.local_index;

        if chassis_set_index != SetType::AwakeSet.raw_value() {
            record.active = false;
            std::mem::swap(&mut record, &mut self.vehicles[index]);
            return;
        }

        let sim_a = self.solver_sets[awake].body_sims[chassis_local_index as usize];
        let state_a = self.solver_sets[awake].body_states[chassis_local_index as usize];
        record.chassis_index = chassis_local_index;
        record.inv_mass = sim_a.inv_mass;
        record.inv_i = sim_a.inv_inertia_world;

        let q = sim_a.transform.q;
        let chassis_up = rotate_vector(q, record.local_up);
        let chassis_forward = rotate_vector(q, record.local_forward);
        if let Some(up) = record.world_up {
            record.step_up = up;
        } else if length_squared(self.gravity) > 0.0 {
            record.step_up = -normalize(self.gravity);
        } else {
            record.step_up = chassis_up;
        }

        // Throttle against the direction of travel brakes first (automatic gearbox only).
        let forward_speed = dot(state_a.linear_velocity, chassis_forward);
        let mut throttle = record.forward_input;
        let mut brake = record.brake_input;
        if record.drivetrain.transmission.settings.mode == TransmissionMode::Automatic
            && throttle * forward_speed < 0.0
            && abs_float(forward_speed) > 1.0 * b3::length_units_per_meter()
        {
            brake = max_float(brake, abs_float(throttle));
            throttle = 0.0;
        }
        record.effective_throttle = throttle;
        record.effective_brake = brake;

        // Per wheel contact geometry, masses, slip, and friction.
        let h = context.h;
        let mut can_shift_up = true;
        for w in 0..record.wheels.len() {
            let mut wheel = record.wheels[w];
            let s = wheel.settings;
            wheel.suspension_impulse = 0.0;
            wheel.stop_impulse = 0.0;
            wheel.longitudinal_impulse = 0.0;
            wheel.lateral_impulse = 0.0;
            wheel.brake_impulse = 0.0;
            wheel.max_brake_impulse = if record.drivetrain.kind == VehicleControllerKind::Tracked {
                0.0
            } else {
                (brake * s.max_brake_torque + record.hand_brake_input * s.max_hand_brake_torque) * h
            };

            if !wheel.has_contact {
                wheel.ground_index = b3::NULL_INDEX;
                wheel.longitudinal_slip = 0.0;
                wheel.lateral_slip = 0.0;
                record.wheels[w] = wheel;
                if self.is_driven_wheel(&record, w as i32) {
                    can_shift_up = false;
                }
                continue;
            }

            let n = wheel.contact_normal;
            let ground = &self.bodies[wheel.contact_body as usize];
            let ground_set_index = ground.set_index;
            let ground_local_index = ground.local_index;
            let mut v_b = Vec3::default();
            let mut w_b = Vec3::default();
            if ground_set_index == SetType::AwakeSet.raw_value() {
                let sim_b = self.solver_sets[awake].body_sims[ground_local_index as usize];
                let state_b = self.solver_sets[awake].body_states[ground_local_index as usize];
                wheel.ground_index = ground_local_index;
                wheel.ground_inv_mass = sim_b.inv_mass;
                wheel.ground_inv_i = sim_b.inv_inertia_world;
                wheel.r_b = sub_pos(wheel.contact_position, sim_b.center);
                v_b = state_b.linear_velocity;
                w_b = state_b.angular_velocity;
            } else {
                wheel.ground_index = b3::NULL_INDEX;
                wheel.ground_inv_mass = 0.0;
                wheel.ground_inv_i = Matrix3::default();
                wheel.r_b = Vec3::default();
            }

            wheel.r_a = sub_pos(wheel.contact_position, sim_a.center);
            wheel.r_as = if s.enable_suspension_force_point {
                sub_pos(
                    transform_world_point(sim_a.transform, s.suspension_force_point),
                    sim_a.center,
                )
            } else {
                wheel.r_a
            };

            let m_a = record.inv_mass;
            let m_b = wheel.ground_inv_mass;
            let i_a = record.inv_i;
            let i_b = wheel.ground_inv_i;

            // Swift's nested `bodyK(_::_:_:)` over the four effective masses.
            let body_k = |r_a: Vec3, r_b: Vec3, axis: Vec3| -> f32 {
                let rn_a = cross(r_a, axis);
                let rn_b = cross(r_b, axis);
                m_a + m_b + dot(rn_a, mul_mv(i_a, rn_a)) + dot(rn_b, mul_mv(i_b, rn_b))
            };

            let k_n = body_k(wheel.r_as, wheel.r_b, n);
            wheel.suspension_mass = if k_n > 0.0 { 1.0 / k_n } else { 0.0 };
            wheel.longitudinal_body_k = body_k(wheel.r_a, wheel.r_b, wheel.longitudinal);
            let k_l = body_k(wheel.r_a, wheel.r_b, wheel.lateral);
            wheel.lateral_mass = if k_l > 0.0 { 1.0 / k_l } else { 0.0 };

            // Spring: frequency relative to the carried mass, or absolute stiffness and damping.
            wheel.suspension_softness = if s.suspension_stiffness > 0.0 && wheel.suspension_mass > 0.0
            {
                let m = wheel.suspension_mass;
                let omega = (s.suspension_stiffness / m).sqrt();
                let zeta = s.suspension_damping / (2.0 * (s.suspension_stiffness * m).sqrt());
                make_soft(omega / b3::TWO_PI, zeta, h)
            } else {
                make_soft(s.suspension_frequency, s.suspension_damping_ratio, h)
            };
            wheel.suspension_position =
                wheel.suspension_length - (s.suspension_max_length + s.suspension_preload_length);
            wheel.min_length_position = wheel.suspension_length - s.suspension_min_length;

            // Slip from the velocities at the start of the step.
            let v_rel = (state_a.linear_velocity + cross(state_a.angular_velocity, wheel.r_a))
                - (v_b + cross(w_b, wheel.r_b));
            let v_long = dot(v_rel, wheel.longitudinal);
            let v_lat = dot(v_rel, wheel.lateral);
            let surface_speed = self.wheel_surface_speed(&record, w);
            let slow_speed: f32 = 0.05 * b3::length_units_per_meter();
            wheel.longitudinal_slip =
                abs_float(surface_speed - v_long) / max_float(abs_float(v_long), slow_speed);
            wheel.lateral_slip = atan2_approx(abs_float(v_lat), abs_float(v_long));

            let mut longitudinal: f32;
            let mut lateral: f32;
            if record.drivetrain.kind == VehicleControllerKind::Tracked {
                longitudinal = s.track_longitudinal_friction;
                lateral = s.track_lateral_friction;
            } else if abs_float(v_long) < slow_speed
                && abs_float(v_lat) < slow_speed
                && abs_float(surface_speed) < slow_speed
            {
                // Static grip: a parked tire does not slip.
                longitudinal = self.curve_peak(&record.wheel_settings[w].longitudinal_friction);
                lateral = self.curve_peak(&record.wheel_settings[w].lateral_friction);
            } else {
                longitudinal = record.wheel_settings[w]
                    .longitudinal_friction
                    .evaluate(wheel.longitudinal_slip);
                lateral = record.wheel_settings[w]
                    .lateral_friction
                    .evaluate(wheel.lateral_slip * b3::RAD_TO_DEG);
            }

            let surface = wheel.surface_friction;
            if let Some(callback) = &record.tire_friction {
                let result = callback(
                    w as i32,
                    longitudinal,
                    lateral,
                    surface,
                    wheel.user_material_id,
                );
                wheel.longitudinal_friction = max_float(0.0, result.0);
                wheel.lateral_friction = max_float(0.0, result.1);
            } else {
                let scale = max_float(0.0, surface).sqrt();
                wheel.longitudinal_friction = longitudinal * scale;
                wheel.lateral_friction = lateral * scale;
            }

            record.wheels[w] = wheel;
        }

        // Gearbox once per step. Shifting up is held while a driven wheel is in the air.
        let engine_rpm = record.drivetrain.engine.rpm();
        record
            .drivetrain
            .transmission
            .update(context.dt, engine_rpm, throttle, can_shift_up);

        // An automatic gearbox decouples the engine when coasting down to idle or braking, so the
        // idling engine neither creeps the vehicle nor fights the brakes.
        record.drivetrain.clutch_open = false;
        if record.drivetrain.transmission.settings.mode == TransmissionMode::Automatic
            && throttle == 0.0
        {
            let idle = record.drivetrain.engine.settings.min_rpm;
            let open = brake > 0.0 || self.driven_engine_rpm(&record) < 1.5 * idle;
            record.drivetrain.clutch_open = open;
        }

        for t in 0..record.drivetrain.tracks.len() {
            record.drivetrain.tracks[t].brake_impulse = 0.0;
            let max_brake_impulse =
                brake * record.drivetrain.tracks[t].settings.max_brake_torque * h;
            record.drivetrain.tracks[t].max_brake_impulse = max_brake_impulse;
        }

        // Pitch/roll limit.
        record.pitch_roll_active = false;
        record.pitch_roll_impulse = 0.0;
        if record.max_pitch_roll_angle < b3::PI {
            let cos_angle = dot(chassis_up, record.step_up);
            let limit = compute_cos_sin(record.max_pitch_roll_angle).cosine;
            // Engage slightly before the limit so the constraint is speculative.
            if cos_angle < limit + 0.05 {
                let (len, axis) = get_length_and_normalize(cross(chassis_up, record.step_up));
                if len > 1.0e-5 {
                    let k = dot(axis, mul_mv(record.inv_i, axis));
                    record.pitch_roll_active = k > 0.0;
                    record.pitch_roll_axis = axis;
                    record.pitch_roll_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
                    record.pitch_roll_chassis_up = chassis_up;
                }
            }
        }

        if record.drivetrain.kind == VehicleControllerKind::Motorcycle {
            self.prepare_motorcycle_lean(&mut record, sim_a, state_a, context);
        }

        std::mem::swap(&mut record, &mut self.vehicles[index]);
    }

    #[inline(always)]
    fn is_driven_wheel(&self, record: &VehicleRecord, wheel: i32) -> bool {
        if record.drivetrain.kind == VehicleControllerKind::Tracked {
            return (wheel as usize) < record.drivetrain.wheel_track.len()
                && record.drivetrain.wheel_track[wheel as usize] >= 0;
        }
        record
            .drivetrain
            .differentials
            .iter()
            .any(|d| d.left_wheel == wheel || d.right_wheel == wheel)
    }

    /// Engine speed the driven wheels would impose through the current gear.
    fn driven_engine_rpm(&self, record: &VehicleRecord) -> f32 {
        let ratio = abs_float(record.drivetrain.transmission.current_ratio());
        let mut sum: f32 = 0.0;
        let mut count: f32 = 0.0;
        if record.drivetrain.kind == VehicleControllerKind::Tracked {
            for track in record.drivetrain.tracks.iter() {
                sum += abs_float(track.angular_velocity) * track.settings.differential_ratio;
                count += 1.0;
            }
        } else {
            for diff in record.drivetrain.differentials.iter() {
                for wheel in [diff.left_wheel, diff.right_wheel] {
                    if wheel >= 0 {
                        sum += abs_float(record.wheels[wheel as usize].angular_velocity)
                            * diff.differential_ratio;
                        count += 1.0;
                    }
                }
            }
        }
        if count > 0.0 {
            ratio * sum / count * ANGULAR_VELOCITY_TO_RPM
        } else {
            0.0
        }
    }

    #[inline(always)]
    fn wheel_surface_speed(&self, record: &VehicleRecord, w: usize) -> f32 {
        if record.drivetrain.kind == VehicleControllerKind::Tracked {
            let t = record.drivetrain.wheel_track[w];
            if t < 0 {
                return record.wheels[w].angular_velocity * record.wheels[w].settings.radius;
            }
            let track = record.drivetrain.tracks[t as usize];
            return track.angular_velocity * track.driven_radius;
        }
        record.wheels[w].angular_velocity * record.wheels[w].settings.radius
    }

    fn curve_peak(&self, curve: &LinearCurve) -> f32 {
        curve.points.iter().fold(0.0, |acc, p| max_float(acc, p.y))
    }

    fn prepare_motorcycle_lean(
        &self,
        record: &mut VehicleRecord,
        sim_a: BodySim,
        state_a: BodyState,
        context: &StepContext,
    ) {
        let mut lean = record.drivetrain.lean.clone();
        let settings = lean.settings.clone();
        let q = sim_a.transform.q;
        let forward = rotate_vector(q, record.local_forward);
        let chassis_up = rotate_vector(q, record.local_up);

        // Current lean about the forward axis, relative to world up.
        let (up_length, projected_up) = get_length_and_normalize(
            record.step_up - dot(record.step_up, forward) * forward,
        );
        if !(up_length > 1.0e-4) {
            lean.active = false;
            record.drivetrain.lean = lean;
            return;
        }
        let side = cross(forward, projected_up);
        lean.current_lean = atan2_approx(dot(chassis_up, side), dot(chassis_up, projected_up));

        // Target lean balances gravity against the centripetal acceleration of the steered turn.
        let g = length(self.gravity);
        let mut target: f32 = 0.0;
        let front = lean.front_wheel as usize;
        if g > 0.0 && front < record.wheels.len() && lean.wheel_base > 0.0 {
            let steer = record.wheels[front].steer_angle;
            let cs = compute_cos_sin(abs_float(steer));
            if cs.sine > 1.0e-4 {
                let speed = dot(state_a.linear_velocity, forward);
                let radius = lean.wheel_base * cs.cosine / cs.sine;
                let acceleration = speed * speed / radius;
                target = atan2_approx(if steer > 0.0 { -acceleration } else { acceleration }, g);
            }
        }
        target = clamp_float(target, -settings.max_lean_angle, settings.max_lean_angle);
        let smoothing = clamp_float(settings.lean_smoothing_factor, 0.0, 0.999);
        lean.target_lean = smoothing * lean.target_lean + (1.0 - smoothing) * target;

        // Integral term against steady state error.
        let dt = context.dt;
        lean.integral += (lean.target_lean - lean.current_lean) * dt;
        lean.integral *= max_float(
            0.0,
            1.0 - settings.lean_spring_integration_coefficient_decay * dt,
        );

        let k = dot(forward, mul_mv(record.inv_i, forward));
        lean.active = k > 0.0 && settings.lean_spring_constant > 0.0;
        lean.axis = forward;
        lean.mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        let mut effective_target = lean.target_lean;
        if settings.lean_spring_constant > 0.0 {
            effective_target += settings.lean_spring_integration_coefficient
                / settings.lean_spring_constant
                * lean.integral;
        }
        lean.position = lean.current_lean - effective_target;
        if lean.active {
            let omega = (settings.lean_spring_constant * k).sqrt();
            let zeta = settings.lean_spring_damping * (k / settings.lean_spring_constant).sqrt()
                * 0.5;
            lean.softness = make_soft(omega / b3::TWO_PI, zeta, context.h);
        }
        lean.impulse = 0.0;
        record.drivetrain.lean = lean;
    }

    // MARK: Substep stages

    pub(crate) fn warm_start_vehicles(&mut self, context: &StepContext) {
        if !(self.vehicle_id_pool.id_count() > 0) {
            return;
        }
        let records = self.vehicles.as_mut_ptr();
        let count = self.vehicles.len();
        // SAFETY: `records` addresses the heap buffer of `vehicles`, which is not resized while a
        // solver phase runs (the vehicle count and the `active` list are fixed before the substeps
        // start), so every pointer stays in bounds and initialized. `with_solver_states` lends the
        // awake set's body-state array, a field disjoint from `vehicles`; Swift formed exactly the
        // same two borrows (`withSolverStates` plus `vehicles[i].warmStart(...)` on the same array)
        // and relied on unchecked exclusivity. Only one record is written at a time and each record
        // touches only its own chassis and ground state slots, so the pointer view never aliases a
        // live borrow of another record.
        self.with_solver_states(|states| {
            // SAFETY: `states` is the whole awake set's state array for this phase and is never
            // resized here; the pointer is only handed to `state_at`/`set_state_at` and the
            // per-record chassis indexing, all of which stay inside it by construction.
            let states = states.as_mut_ptr();
            for i in 0..count {
                // SAFETY: `i < count` indexes a live slot of the vehicles buffer; the `&mut` is the
                // exclusive handle for that record for the duration of the call, and the only other
                // aliasing storage reached through it is the state array above (disjoint field).
                let record = unsafe { &mut *records.add(i) };
                if record.active {
                    record.warm_start(states, context);
                }
            }
        });
    }

    pub(crate) fn solve_vehicles(&mut self, context: &StepContext, use_bias: bool) {
        if !(self.vehicle_id_pool.id_count() > 0) {
            return;
        }
        let records = self.vehicles.as_mut_ptr();
        let count = self.vehicles.len();
        // SAFETY: identical to [`World::warm_start_vehicles`]; the solve phase lends the same
        // awake-set state array and walks one vehicle record at a time.
        self.with_solver_states(|states| {
            // SAFETY: as in `warm_start_vehicles`; the pointer covers the awake set's state array
            // for the whole phase and is never used past a resize.
            let states = states.as_mut_ptr();
            for i in 0..count {
                // SAFETY: as in `warm_start_vehicles`; exclusive handle to record `i` for the call.
                let record = unsafe { &mut *records.add(i) };
                if record.active {
                    record.solve(states, context, use_bias);
                }
            }
        });
    }

    /// Record totals and advance the visual wheel rotation.
    pub(crate) fn finalize_vehicles(&mut self, context: &StepContext) {
        if !(self.vehicle_id_pool.id_count() > 0) {
            return;
        }
        let sub_steps = context.sub_step_count as f32;
        for i in 0..self.vehicles.len() {
            if !self.vehicles[i].active {
                continue;
            }
            let mut record = self.vehicles[i].clone();
            for w in 0..record.wheels.len() {
                let mut wheel = record.wheels[w];
                if record.drivetrain.kind == VehicleControllerKind::Tracked {
                    let t = record.drivetrain.wheel_track[w];
                    if t >= 0 {
                        let track = record.drivetrain.tracks[t as usize];
                        wheel.angular_velocity = track.angular_velocity * track.driven_radius
                            / wheel.settings.radius;
                    }
                }
                wheel.rotation_angle =
                    remainder_angle(wheel.rotation_angle + wheel.angular_velocity * context.dt);
                wheel.total_suspension_impulse =
                    (wheel.suspension_impulse + wheel.stop_impulse) * sub_steps;
                wheel.total_longitudinal_impulse = wheel.longitudinal_impulse * sub_steps;
                wheel.total_lateral_impulse = wheel.lateral_impulse * sub_steps;
                record.wheels[w] = wheel;
            }
            self.vehicles[i] = record;
        }
    }
}

#[inline(always)]
fn remainder_angle(angle: f32) -> f32 {
    let mut a = angle;
    if a > b3::PI || a < -b3::PI {
        // Swift's `rounded()` rounds half away from zero, which is Rust's `f32::round()` (not
        // `round_ties_even`). `b3::TWO_PI` is the stored constant, not `2.0 * b3::PI`, so
        // `math::functions::unwind_angle` is *not* interchangeable with this helper.
        a -= b3::TWO_PI * (a / b3::TWO_PI).round();
    }
    a
}

// MARK: - Constraint math

impl VehicleRecord {
    #[inline(always)]
    fn ground_state(&self, states: *mut BodyState, index: i32) -> BodyState {
        // SAFETY: `states` is the awake set's state array lent for this solver phase (see
        // `World::warm_start_vehicles`); `state_at` bounds-checks by contract: `index < 0` is a
        // static body and returns identity, and an awake body's solver index is inside the array.
        World::state_at(states, index)
    }

    /// Apply an impulse along `axis` to the chassis at rA and the opposite to the ground at rB.
    #[inline(always)]
    fn apply_pair(
        &self,
        state_a: &mut BodyState,
        state_b: &mut BodyState,
        wheel: VehicleWheel,
        r_a: Vec3,
        axis: Vec3,
        impulse: f32,
    ) {
        let p = impulse * axis;
        state_a.linear_velocity = mul_add(state_a.linear_velocity, self.inv_mass, p);
        state_a.angular_velocity =
            state_a.angular_velocity + mul_mv(self.inv_i, cross(r_a, p));
        state_b.linear_velocity = mul_sub(state_b.linear_velocity, wheel.ground_inv_mass, p);
        state_b.angular_velocity =
            state_b.angular_velocity - mul_mv(wheel.ground_inv_i, cross(wheel.r_b, p));
    }

    #[inline(always)]
    fn relative_velocity(
        &self,
        state_a: BodyState,
        state_b: BodyState,
        r_a: Vec3,
        r_b: Vec3,
    ) -> Vec3 {
        (state_a.linear_velocity + cross(state_a.angular_velocity, r_a))
            - (state_b.linear_velocity + cross(state_b.angular_velocity, r_b))
    }

    /// Change of the suspension length since the step started.
    #[inline(always)]
    fn suspension_delta(&self, state_a: BodyState, state_b: BodyState, wheel: VehicleWheel) -> f32 {
        let d_a = state_a.delta_position
            + (rotate_vector(state_a.delta_rotation, wheel.r_as) - wheel.r_as);
        let d_b =
            state_b.delta_position + (rotate_vector(state_b.delta_rotation, wheel.r_b) - wheel.r_b);
        dot(wheel.contact_normal, d_a - d_b)
    }

    // MARK: Warm start

    pub(crate) fn warm_start(&mut self, states: *mut BodyState, context: &StepContext) {
        let h = context.h;
        self.drive_substep(h);

        // Brakes restart every substep.
        for w in 0..self.wheels.len() {
            self.wheels[w].brake_impulse = 0.0;
        }
        for t in 0..self.drivetrain.tracks.len() {
            self.drivetrain.tracks[t].brake_impulse = 0.0;
        }

        let chassis = self.chassis_index;
        // SAFETY: `chassis` is the awake-set solver index recorded for this vehicle during
        // prepare, inside the state array `states` addresses for this phase.
        let mut state_a = unsafe { *states.add(chassis as usize) };

        // Anti-roll bars: an explicit spring on the difference of the suspension lengths.
        for bar in self.anti_roll_bars.iter() {
            let left = bar.left_wheel as usize;
            let right = bar.right_wheel as usize;
            if !(self.wheels[left].has_contact && self.wheels[right].has_contact) {
                continue;
            }

            let mut left_state = self.ground_state(states, self.wheels[left].ground_index);
            let left_length = self.wheels[left].suspension_length
                + self.suspension_delta(state_a, left_state, self.wheels[left]);
            let mut right_state = self.ground_state(states, self.wheels[right].ground_index);
            let right_length = self.wheels[right].suspension_length
                + self.suspension_delta(state_a, right_state, self.wheels[right]);

            // The longer side is higher; push it down and the other side up.
            let impulse = bar.stiffness * (right_length - left_length) * h;
            self.apply_pair(
                &mut state_a,
                &mut left_state,
                self.wheels[left],
                self.wheels[left].r_as,
                self.wheels[left].contact_normal,
                impulse,
            );
            World::set_state_at(states, self.wheels[left].ground_index, left_state);
            right_state = self.ground_state(states, self.wheels[right].ground_index);
            self.apply_pair(
                &mut state_a,
                &mut right_state,
                self.wheels[right],
                self.wheels[right].r_as,
                self.wheels[right].contact_normal,
                -impulse,
            );
            World::set_state_at(states, self.wheels[right].ground_index, right_state);
        }

        // Accumulated impulses of the previous substep.
        for w in 0..self.wheels.len() {
            if self.wheels[w].has_contact {
                let wheel = self.wheels[w];
                let mut state_b = self.ground_state(states, wheel.ground_index);
                self.apply_pair(
                    &mut state_a,
                    &mut state_b,
                    wheel,
                    wheel.r_as,
                    wheel.contact_normal,
                    wheel.suspension_impulse + wheel.stop_impulse,
                );
                self.apply_pair(
                    &mut state_a,
                    &mut state_b,
                    wheel,
                    wheel.r_a,
                    wheel.longitudinal,
                    wheel.longitudinal_impulse,
                );
                self.apply_pair(
                    &mut state_a,
                    &mut state_b,
                    wheel,
                    wheel.r_a,
                    wheel.lateral,
                    wheel.lateral_impulse,
                );
                World::set_state_at(states, wheel.ground_index, state_b);
                let radius = self.rolling_properties(w).0;
                self.apply_rolling_angular_impulse(w, -radius * wheel.longitudinal_impulse, 0.0);
            }
        }

        if self.pitch_roll_active {
            state_a.angular_velocity = mul_add(
                state_a.angular_velocity,
                self.pitch_roll_impulse,
                mul_mv(self.inv_i, self.pitch_roll_axis),
            );
        }
        if self.drivetrain.kind == VehicleControllerKind::Motorcycle && self.drivetrain.lean.active {
            let lean = &self.drivetrain.lean;
            state_a.angular_velocity = mul_add(
                state_a.angular_velocity,
                lean.impulse,
                mul_mv(self.inv_i, lean.axis),
            );
        }

        // SAFETY: as at the top of the phase; Swift wrote the chassis state back unconditionally
        // (`states[chassisIndex] = stateA`), so no dynamic-flag filter is added here.
        unsafe { *states.add(chassis as usize) = state_a };
    }

    /// Brake accumulator and capacity of the rolling part at wheel `w`.
    ///
    /// Swift keeps this helper unused in the solver (the solve path reads the wheel or track
    /// directly); it is translated for parity and kept dead.
    #[allow(dead_code)]
    #[inline(always)]
    fn brake(&self, w: usize) -> (f32, f32) {
        if self.drivetrain.kind == VehicleControllerKind::Tracked {
            let t = self.drivetrain.wheel_track[w];
            if t >= 0 {
                let track = self.drivetrain.tracks[t as usize];
                return (track.brake_impulse, track.max_brake_impulse);
            }
        }
        (self.wheels[w].brake_impulse, self.wheels[w].max_brake_impulse)
    }

    /// Change the rotation of the rolling part at wheel `w` by an angular impulse, optionally
    /// recording the part of it delivered by the brake.
    #[inline(always)]
    fn apply_rolling_angular_impulse(&mut self, w: usize, angular: f32, brake: f32) {
        if self.drivetrain.kind == VehicleControllerKind::Tracked {
            let t = self.drivetrain.wheel_track[w];
            if t >= 0 {
                let track_index = t as usize;
                let inv_i = self.drivetrain.tracks[track_index].inverse_inertia();
                self.drivetrain.tracks[track_index].angular_velocity += angular * inv_i;
                self.drivetrain.tracks[track_index].brake_impulse += brake;
                return;
            }
        }
        let inv_i = self.wheels[w].inverse_inertia();
        self.wheels[w].angular_velocity += angular * inv_i;
        self.wheels[w].brake_impulse += brake;
    }

    /// Solve the tire and the brake of one wheel together. Unknowns are the tire impulse on the
    /// chassis (dLambda) and the brake's angular impulse on the wheel (dBeta):
    ///   v' = v + kB dLambda,  w' = w - R invI dLambda + invI dBeta,
    /// with v' = R w' (no slip) and w' = 0 (brake holds) while neither impulse is clamped. Solving
    /// the pair exactly avoids the slow convergence of a light wheel between a heavy chassis and its
    /// brake.
    #[inline(always)]
    fn solve_braked_tire(
        &self,
        v: f32,
        omega: f32,
        radius: f32,
        inv_i: f32,
        k_b: f32,
        lambda: f32,
        max_lambda: f32,
        beta: f32,
        max_beta: f32,
    ) -> (f32, f32) {
        if !(k_b > 0.0 && inv_i > 0.0) {
            return (0.0, 0.0);
        }

        // Both free: the wheel locks and the contact sticks.
        let mut d_lambda = -v / k_b;
        let mut new_lambda = clamp_float(lambda + d_lambda, -max_lambda, max_lambda);
        d_lambda = new_lambda - lambda;
        let mut d_beta = -omega / inv_i + radius * d_lambda;
        let new_beta = clamp_float(beta + d_beta, -max_beta, max_beta);
        if new_beta == beta + d_beta {
            return (d_lambda, d_beta);
        }

        // The brake slips: it delivers its limit and the tire sees a rolling wheel.
        d_beta = new_beta - beta;
        let k = k_b + radius * radius * inv_i;
        d_lambda = -(v - radius * (omega + inv_i * d_beta)) / k;
        new_lambda = clamp_float(lambda + d_lambda, -max_lambda, max_lambda);
        d_lambda = new_lambda - lambda;
        (d_lambda, d_beta)
    }

    // MARK: Drivetrain

    /// Engine torque, damping, and the clutch for one substep.
    fn drive_substep(&mut self, h: f32) {
        let throttle = abs_float(self.effective_throttle);
        let mut engine = self.drivetrain.engine.clone();
        let inv_i_e = engine.inverse_inertia();
        let torque_impulse = engine.torque(throttle) * h * inv_i_e;
        engine.angular_velocity += torque_impulse;
        engine.angular_velocity *= 1.0 / (1.0 + h * engine.settings.angular_damping);

        for w in 0..self.wheels.len() {
            let damping = self.wheels[w].settings.angular_damping;
            self.wheels[w].angular_velocity *= 1.0 / (1.0 + h * damping);
        }
        for t in 0..self.drivetrain.tracks.len() {
            let damping = self.drivetrain.tracks[t].settings.angular_damping;
            self.drivetrain.tracks[t].angular_velocity *= 1.0 / (1.0 + h * damping);
        }

        let ratio = self.drivetrain.transmission.current_ratio();
        let clutch = if self.drivetrain.clutch_open {
            0.0
        } else {
            self.drivetrain.transmission.settings.clutch_strength
                * self.drivetrain.transmission.clutch_friction
        };
        if ratio != 0.0 && clutch > 0.0 {
            if self.drivetrain.kind == VehicleControllerKind::Tracked {
                self.couple_tracks(&mut engine, ratio, clutch, h);
            } else {
                self.couple_wheels(&mut engine, ratio, clutch, h);
            }
        }

        engine.clamp_speed();
        self.drivetrain.engine = engine;
    }

    /// Implicit viscous clutch between the engine and the driven wheels. Each wheel's coupling
    /// coefficient is its share of the clutch shaft speed, so power is conserved:
    /// w_clutch = sum(c_i * w_i).
    fn couple_wheels(
        &mut self,
        engine: &mut VehicleEngine,
        ratio: f32,
        clutch: f32,
        h: f32,
    ) {
        // Swift's `let differentials = drivetrain.differentials` (a COW copy) plus its reads of
        // `wheels` inside the nested helpers; the split borrow below keeps the two disjoint.
        let VehicleRecord {
            wheels, drivetrain, ..
        } = self;

        if drivetrain.differentials.is_empty() {
            return;
        }

        // Torque between differentials, biased toward the slower axle.
        let limited = drivetrain.differential_limited_slip_ratio;
        let biased = drivetrain.differentials.len() > 1
            && limited < f32::MAX
            && limited > 1.0;

        // Swift's nested `axleSpeed(_:)`; `wheels` is passed instead of captured.
        fn axle_speed(wheels: &[VehicleWheel], diff: &VehicleDifferentialSettings) -> f32 {
            let mut sum: f32 = 0.0;
            let mut count: f32 = 0.0;
            if diff.left_wheel >= 0 {
                sum += abs_float(wheels[diff.left_wheel as usize].angular_velocity);
                count += 1.0;
            }
            if diff.right_wheel >= 0 {
                sum += abs_float(wheels[diff.right_wheel as usize].angular_velocity);
                count += 1.0;
            }
            max_float(1.0e-3, if count > 0.0 { sum / count } else { 0.0 })
        }

        let mut slowest: f32 = f32::MAX;
        if biased {
            for diff in drivetrain.differentials.iter() {
                slowest = min_float(slowest, axle_speed(wheels, diff));
            }
        }

        // Swift's nested `axleRatio(_:)`.
        fn axle_ratio(
            wheels: &[VehicleWheel],
            diff: &VehicleDifferentialSettings,
            biased: bool,
            slowest: f32,
            limited: f32,
        ) -> f32 {
            let base = max_float(0.0, diff.engine_torque_ratio);
            if !biased {
                return base;
            }
            let t = clamp_float(
                (axle_speed(wheels, diff) / slowest - 1.0) / (limited - 1.0),
                0.0,
                1.0,
            );
            base * (1.0 - t)
        }

        let mut total: f32 = 0.0;
        for diff in drivetrain.differentials.iter() {
            total += axle_ratio(wheels, diff, biased, slowest, limited);
        }
        if !(total > 0.0) {
            return;
        }

        // Each wheel's share of the clutch shaft speed: w_clutch = sum(c_i * w_i).
        for w in 0..wheels.len() {
            wheels[w].clutch_coefficient = 0.0;
        }
        for diff in drivetrain.differentials.iter() {
            let axle = ratio
                * diff.differential_ratio
                * axle_ratio(wheels, diff, biased, slowest, limited)
                / total;
            let left = if diff.left_wheel >= 0 {
                wheels[diff.left_wheel as usize].angular_velocity
            } else {
                0.0
            };
            let right = if diff.right_wheel >= 0 {
                wheels[diff.right_wheel as usize].angular_velocity
            } else {
                0.0
            };
            let mut split =
                limited_slip_fractions(diff.left_right_split, left, right, diff.limited_slip_ratio);
            if diff.left_wheel < 0 {
                split = (0.0, 1.0);
            }
            if diff.right_wheel < 0 {
                split = (1.0, 0.0);
            }
            if diff.left_wheel >= 0 {
                wheels[diff.left_wheel as usize].clutch_coefficient += axle * split.0;
            }
            if diff.right_wheel >= 0 {
                wheels[diff.right_wheel as usize].clutch_coefficient += axle * split.1;
            }
        }

        let inv_i_e = engine.inverse_inertia();
        let mut clutch_speed: f32 = 0.0;
        let mut inv_k = inv_i_e;
        for w in 0..wheels.len() {
            let c = wheels[w].clutch_coefficient;
            clutch_speed += c * wheels[w].angular_velocity;
            inv_k += c * c * wheels[w].inverse_inertia();
        }
        let damping = clutch * h;
        let impulse = damping * (engine.angular_velocity - clutch_speed) / (1.0 + damping * inv_k);
        engine.angular_velocity -= impulse * inv_i_e;
        for w in 0..wheels.len() {
            if wheels[w].clutch_coefficient != 0.0 {
                let coefficient = wheels[w].clutch_coefficient;
                let inv_i = wheels[w].inverse_inertia();
                wheels[w].angular_velocity += impulse * coefficient * inv_i;
            }
        }
    }

    /// Implicit viscous clutch between the engine and both tracks.
    fn couple_tracks(
        &mut self,
        engine: &mut VehicleEngine,
        ratio: f32,
        clutch: f32,
        h: f32,
    ) {
        if !(self.drivetrain.tracks.len() >= 2) {
            return;
        }
        let inv_i_e = engine.inverse_inertia();
        let c0 = 0.5
            * ratio
            * self.drivetrain.tracks[0].settings.differential_ratio
            * self.left_track_ratio;
        let c1 = 0.5
            * ratio
            * self.drivetrain.tracks[1].settings.differential_ratio
            * self.right_track_ratio;
        let clutch_speed = c0 * self.drivetrain.tracks[0].angular_velocity
            + c1 * self.drivetrain.tracks[1].angular_velocity;
        let inv_k = inv_i_e
            + c0 * c0 * self.drivetrain.tracks[0].inverse_inertia()
            + c1 * c1 * self.drivetrain.tracks[1].inverse_inertia();
        let damping = clutch * h;
        let impulse = damping * (engine.angular_velocity - clutch_speed) / (1.0 + damping * inv_k);
        engine.angular_velocity -= impulse * inv_i_e;
        let inv_0 = self.drivetrain.tracks[0].inverse_inertia();
        self.drivetrain.tracks[0].angular_velocity += impulse * c0 * inv_0;
        let inv_1 = self.drivetrain.tracks[1].inverse_inertia();
        self.drivetrain.tracks[1].angular_velocity += impulse * c1 * inv_1;
    }

    // MARK: Solve

    pub(crate) fn solve(
        &mut self,
        states: *mut BodyState,
        context: &StepContext,
        use_bias: bool,
    ) {
        let chassis = self.chassis_index;
        // SAFETY: `chassis` is this vehicle's awake-set solver index, inside the state array
        // `states` addresses for the whole solve phase (see `World::solve_vehicles`).
        let mut state_a = unsafe { *states.add(chassis as usize) };
        let inv_h = context.inv_h;
        let contact_softness = context.contact_softness;

        // Brakes on rolling parts in the air. Grounded parts brake together with their tire.
        for w in 0..self.wheels.len() {
            if self.wheels[w].max_brake_impulse > 0.0 && !self.wheels[w].has_contact {
                let inv_i = self.wheels[w].inverse_inertia();
                if !(inv_i > 0.0) {
                    continue;
                }
                let max_impulse = self.wheels[w].max_brake_impulse;
                let old = self.wheels[w].brake_impulse;
                let omega = self.wheels[w].angular_velocity;
                let new = clamp_float(old - omega / inv_i, -max_impulse, max_impulse);
                self.wheels[w].brake_impulse = new;
                self.wheels[w].angular_velocity += (new - old) * inv_i;
            }
        }
        for t in 0..self.drivetrain.tracks.len() {
            let mut grounded = false;
            for &w in self.drivetrain.track_wheels[t].iter() {
                if self.wheels[w as usize].has_contact {
                    grounded = true;
                    break;
                }
            }
            if self.drivetrain.tracks[t].max_brake_impulse > 0.0 && !grounded {
                let inv_i = self.drivetrain.tracks[t].inverse_inertia();
                if !(inv_i > 0.0) {
                    continue;
                }
                let max_impulse = self.drivetrain.tracks[t].max_brake_impulse;
                let old = self.drivetrain.tracks[t].brake_impulse;
                let omega = self.drivetrain.tracks[t].angular_velocity;
                let new = clamp_float(old - omega / inv_i, -max_impulse, max_impulse);
                self.drivetrain.tracks[t].brake_impulse = new;
                self.drivetrain.tracks[t].angular_velocity += (new - old) * inv_i;
            }
        }

        let tracked = self.drivetrain.kind == VehicleControllerKind::Tracked;
        for w in 0..self.wheels.len() {
            if !self.wheels[w].has_contact {
                continue;
            }
            // One copy in and out per wheel; the rolling part is the wheel or its track.
            let mut wheel = self.wheels[w];
            let track: i32 = if tracked {
                self.drivetrain.wheel_track[w]
            } else {
                -1
            };
            let mut state_b = self.ground_state(states, wheel.ground_index);
            let n = wheel.contact_normal;

            // Suspension first: tire friction is bounded by the load it carries.
            let delta = self.suspension_delta(state_a, state_b, wheel);
            {
                // Spring. It can push the chassis up but never pull it down.
                let vn = dot(
                    self.relative_velocity(state_a, state_b, wheel.r_as, wheel.r_b),
                    n,
                );
                let soft = wheel.suspension_softness;
                let c = wheel.suspension_position + delta;
                let mut impulse = -soft.mass_scale
                    * wheel.suspension_mass
                    * (vn + soft.bias_rate * c)
                    - soft.impulse_scale * wheel.suspension_impulse;
                let new_impulse = max_float(wheel.suspension_impulse + impulse, 0.0);
                impulse = new_impulse - wheel.suspension_impulse;
                wheel.suspension_impulse = new_impulse;
                self.apply_pair(&mut state_a, &mut state_b, wheel, wheel.r_as, n, impulse);
            }
            {
                // Bump stop at the minimum length.
                let c = wheel.min_length_position + delta;
                let vn = dot(
                    self.relative_velocity(state_a, state_b, wheel.r_as, wheel.r_b),
                    n,
                );
                let mut bias: f32 = 0.0;
                let mut mass_scale: f32 = 1.0;
                let mut impulse_scale: f32 = 0.0;
                if c > 0.0 {
                    bias = c * inv_h;
                } else if use_bias {
                    bias = max_float(
                        contact_softness.bias_rate * c,
                        -4.0 * b3::length_units_per_meter(),
                    );
                    mass_scale = contact_softness.mass_scale;
                    impulse_scale = contact_softness.impulse_scale;
                }
                let mut impulse = -mass_scale * wheel.suspension_mass * (vn + bias)
                    - impulse_scale * wheel.stop_impulse;
                let new_impulse = max_float(wheel.stop_impulse + impulse, 0.0);
                impulse = new_impulse - wheel.stop_impulse;
                wheel.stop_impulse = new_impulse;
                self.apply_pair(&mut state_a, &mut state_b, wheel, wheel.r_as, n, impulse);
            }

            // Tire friction inside the friction ellipse.
            let load = wheel.suspension_impulse + wheel.stop_impulse;
            let max_longitudinal = wheel.longitudinal_friction * load;
            let max_lateral = wheel.lateral_friction * load;

            let mut radius = wheel.settings.radius;
            let mut inv_inertia = wheel.inverse_inertia();
            let mut omega = wheel.angular_velocity;
            let mut brake_accumulated = wheel.brake_impulse;
            let mut brake_max = wheel.max_brake_impulse;
            if track >= 0 {
                let t = track as usize;
                radius = self.drivetrain.tracks[t].driven_radius;
                inv_inertia = self.drivetrain.tracks[t].inverse_inertia();
                omega = self.drivetrain.tracks[t].angular_velocity;
                brake_accumulated = self.drivetrain.tracks[t].brake_impulse;
                brake_max = self.drivetrain.tracks[t].max_brake_impulse;
            }

            // Longitudinal: the contact patch moves with the tire surface.
            {
                let v = dot(
                    self.relative_velocity(state_a, state_b, wheel.r_a, wheel.r_b),
                    wheel.longitudinal,
                );
                let mut impulse: f32;
                let mut brake_impulse: f32 = 0.0;
                if brake_max > 0.0 {
                    let result = self.solve_braked_tire(
                        v,
                        omega,
                        radius,
                        inv_inertia,
                        wheel.longitudinal_body_k,
                        wheel.longitudinal_impulse,
                        max_longitudinal,
                        brake_accumulated,
                        brake_max,
                    );
                    impulse = result.0;
                    brake_impulse = result.1;
                } else {
                    let k = wheel.longitudinal_body_k + radius * radius * inv_inertia;
                    impulse = if k > 0.0 {
                        -(v - omega * radius) / k
                    } else {
                        0.0
                    };
                    let new_impulse = clamp_float(
                        wheel.longitudinal_impulse + impulse,
                        -max_longitudinal,
                        max_longitudinal,
                    );
                    impulse = new_impulse - wheel.longitudinal_impulse;
                }
                wheel.longitudinal_impulse += impulse;
                self.apply_pair(
                    &mut state_a,
                    &mut state_b,
                    wheel,
                    wheel.r_a,
                    wheel.longitudinal,
                    impulse,
                );
                omega += (brake_impulse - radius * impulse) * inv_inertia;
                brake_accumulated += brake_impulse;
            }

            // Lateral.
            {
                let c_dot = dot(
                    self.relative_velocity(state_a, state_b, wheel.r_a, wheel.r_b),
                    wheel.lateral,
                );
                let mut impulse = -wheel.lateral_mass * c_dot;
                let new_impulse =
                    clamp_float(wheel.lateral_impulse + impulse, -max_lateral, max_lateral);
                impulse = new_impulse - wheel.lateral_impulse;
                wheel.lateral_impulse = new_impulse;
                self.apply_pair(
                    &mut state_a,
                    &mut state_b,
                    wheel,
                    wheel.r_a,
                    wheel.lateral,
                    impulse,
                );
            }

            // Combined slip: the tire cannot deliver both peak forces at once.
            if max_longitudinal > 0.0 && max_lateral > 0.0 {
                let x = wheel.longitudinal_impulse / max_longitudinal;
                let y = wheel.lateral_impulse / max_lateral;
                let r2 = x * x + y * y;
                if r2 > 1.0 {
                    let scale = 1.0 / r2.sqrt();
                    let new_longitudinal = wheel.longitudinal_impulse * scale;
                    let new_lateral = wheel.lateral_impulse * scale;
                    let d_long = new_longitudinal - wheel.longitudinal_impulse;
                    let d_lat = new_lateral - wheel.lateral_impulse;
                    wheel.longitudinal_impulse = new_longitudinal;
                    wheel.lateral_impulse = new_lateral;
                    self.apply_pair(
                        &mut state_a,
                        &mut state_b,
                        wheel,
                        wheel.r_a,
                        wheel.longitudinal,
                        d_long,
                    );
                    self.apply_pair(
                        &mut state_a,
                        &mut state_b,
                        wheel,
                        wheel.r_a,
                        wheel.lateral,
                        d_lat,
                    );
                    omega -= radius * d_long * inv_inertia;
                }
            }

            if track >= 0 {
                let t = track as usize;
                self.drivetrain.tracks[t].angular_velocity = omega;
                self.drivetrain.tracks[t].brake_impulse = brake_accumulated;
            } else {
                wheel.angular_velocity = omega;
                wheel.brake_impulse = brake_accumulated;
            }
            World::set_state_at(states, wheel.ground_index, state_b);
            self.wheels[w] = wheel;
        }

        // Pitch/roll limit against the world up.
        if self.pitch_roll_active {
            let up = rotate_vector(state_a.delta_rotation, self.pitch_roll_chassis_up);
            let angle = atan2_approx(
                length(cross(up, self.step_up)),
                dot(up, self.step_up),
            );
            let c = self.max_pitch_roll_angle - angle;
            let c_dot = dot(state_a.angular_velocity, self.pitch_roll_axis);
            let mut bias: f32 = 0.0;
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if c > 0.0 {
                bias = c * inv_h;
            } else if use_bias {
                bias = contact_softness.bias_rate * c;
                mass_scale = contact_softness.mass_scale;
                impulse_scale = contact_softness.impulse_scale;
            }
            let mut impulse = -mass_scale * self.pitch_roll_mass * (c_dot + bias)
                - impulse_scale * self.pitch_roll_impulse;
            let new_impulse = max_float(self.pitch_roll_impulse + impulse, 0.0);
            impulse = new_impulse - self.pitch_roll_impulse;
            self.pitch_roll_impulse = new_impulse;
            state_a.angular_velocity = mul_add(
                state_a.angular_velocity,
                impulse,
                mul_mv(self.inv_i, self.pitch_roll_axis),
            );
        }

        // Motorcycle lean spring about the forward axis.
        if self.drivetrain.kind == VehicleControllerKind::Motorcycle && self.drivetrain.lean.active {
            let mut lean = self.drivetrain.lean.clone();
            // Small rotation of the chassis about the lean axis since the step started.
            let rotated = 2.0
                * dot(state_a.delta_rotation.v, lean.axis)
                * (if state_a.delta_rotation.s < 0.0 { -1.0 } else { 1.0 });
            let c = lean.position + rotated;
            let c_dot = dot(state_a.angular_velocity, lean.axis);
            let soft = lean.softness;
            let impulse = -soft.mass_scale * lean.mass * (c_dot + soft.bias_rate * c)
                - soft.impulse_scale * lean.impulse;
            lean.impulse += impulse;
            state_a.angular_velocity =
                mul_add(state_a.angular_velocity, impulse, mul_mv(self.inv_i, lean.axis));
            self.drivetrain.lean = lean;
        }

        // SAFETY: as at the top of the phase; Swift's `states[chassisIndex] = stateA` write is
        // unconditional, so the dynamic-flag filter of `World::set_state_at` is deliberately absent.
        unsafe { *states.add(chassis as usize) = state_a };
    }

    /// Radius, inverse inertia, and surface speed of the part rolling on the ground at wheel `w`.
    #[inline(always)]
    fn rolling_properties(&self, w: usize) -> (f32, f32, f32) {
        if self.drivetrain.kind == VehicleControllerKind::Tracked {
            let t = self.drivetrain.wheel_track[w];
            if t >= 0 {
                let track = self.drivetrain.tracks[t as usize];
                return (
                    track.driven_radius,
                    track.inverse_inertia(),
                    track.angular_velocity * track.driven_radius,
                );
            }
        }
        let wheel = self.wheels[w];
        (
            wheel.settings.radius,
            wheel.inverse_inertia(),
            wheel.angular_velocity * wheel.settings.radius,
        )
    }
}
