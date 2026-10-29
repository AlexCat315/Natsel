//! Solver integration, finalization, and a contact-free step.
//!
//! Translated from `Sources/Natsel/Simulation/SolverIntegration.swift`.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::base::platform::{get_milliseconds, get_milliseconds_and_reset, get_ticks};
use crate::collision::internal_math::{integrate_rotation, modified_cross};
use crate::containers::bit_set::BitSet;
use crate::math::functions::{
    aabb_contains, aabb_inflate, abs_vec3, blend2, dot, is_valid_vec3, length,
    make_matrix_from_quat, max_float, max_int, min_float, mul_add, mul_mm, mul_mv, mul_quat,
    normalize_quat, offset_pos, rotate_vector, inv_rotate_vector, solve3, transpose,
};
use crate::math::types::{AABB, Matrix3, Quat, Vec3};
use crate::types::body_def::BodyType;
use crate::types::events::BodyMoveEvent;
use crate::types::ids::BodyID;

use crate::simulation::continuous::dynamic_impact::ContinuousStart;
use crate::simulation::shape_runtime::{Shape, compute_fat_shape_aabb};
use crate::simulation::solver_stages::SolverStageKind;
use crate::simulation::world_runtime::{
    Body, BodyFlags, BodySim, BodyState, Island, SetType, SolverSet, World,
};

/// Rebuild a Swift `UnsafeBufferPointer` as a slice. A null or empty buffer is an empty slice.
///
/// SAFETY: the caller guarantees `ptr..ptr + len` is live, initialized storage that no other
/// reference writes for `'a`. This is the invariant Swift places on `UnsafeBufferPointer`.
pub(crate) unsafe fn buffer_view<'a, T>(ptr: *const T, len: usize) -> &'a [T] {
    if ptr.is_null() || len == 0 {
        &[]
    } else {
        std::slice::from_raw_parts(ptr, len)
    }
}

/// Rebuild a Swift `UnsafeMutableBufferPointer` as a mutable slice.
///
/// SAFETY: as [`buffer_view`], plus the range must be uniquely writable for `'a`.
pub(crate) unsafe fn mutable_buffer_view<'a, T>(ptr: *mut T, len: usize) -> &'a mut [T] {
    if ptr.is_null() || len == 0 {
        &mut []
    } else {
        std::slice::from_raw_parts_mut(ptr, len)
    }
}

/// Constraint softness.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Softness {
    pub bias_rate: f32,
    pub mass_scale: f32,
    pub impulse_scale: f32,
}

impl Softness {
    #[inline]
    pub const fn new() -> Self {
        Self {
            bias_rate: 0.0,
            mass_scale: 0.0,
            impulse_scale: 0.0,
        }
    }
}

/// Make a softness from a Hertz frequency and damping ratio.
#[inline(always)]
pub fn make_soft(hertz: f32, zeta: f32, h: f32) -> Softness {
    if hertz == 0.0 {
        return Softness::new();
    }

    let omega = 2.0 * b3::PI * hertz;
    let a1 = 2.0 * zeta + h * omega;
    let a2 = h * omega * a1;
    let a3 = 1.0 / (1.0 + a2);

    let mut softness = Softness::new();
    softness.bias_rate = omega / a1;
    softness.mass_scale = a2 * a3;
    softness.impulse_scale = a3;
    softness
}

/// Work deferred from parallel body finalization to the ordered serial pass.
pub(crate) struct FinalizeFlag;

impl FinalizeFlag {
    pub(crate) const CONTINUOUS: u8 = 1;
    pub(crate) const BULLET: u8 = 2;
    pub(crate) const MOVED_PROXIES: u8 = 3;
}

/// Per-thread island results of body finalization.
#[derive(Clone, Debug)]
pub struct FinalizeThreadData {
    pub islands: BitSet,
    pub split_island_id: i32,
    pub split_sleep_time: f32,
}

impl Default for FinalizeThreadData {
    #[inline]
    fn default() -> Self {
        Self {
            islands: BitSet::new(),
            split_island_id: b3::NULL_INDEX,
            split_sleep_time: 0.0,
        }
    }
}

impl FinalizeThreadData {
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// Pointers for parallel body finalization.
///
/// Swift's buffer pointers keep their element count beside the pointer; the pointers are null when
/// the buffer is empty, exactly like `UnsafeMutableBufferPointer(start: nil, count: 0)`.
#[derive(Clone, Copy)]
pub(crate) struct FinalizeViews {
    pub sims: *mut BodySim,
    pub sims_count: usize,
    pub states: *mut BodyState,
    pub states_count: usize,
    pub bodies: *mut Body,
    pub islands: *const Island,
    pub shapes: *mut Shape,
    pub fat_aabbs: *mut AABB,
    pub move_events: *mut BodyMoveEvent,
    pub flags: *mut u8,
    pub shape_moved: *mut u8,
    pub threads: *mut FinalizeThreadData,
    pub island_count: i32,
    pub time_step: f32,
    pub inv_time_step: f32,
}

impl Default for FinalizeViews {
    #[inline]
    fn default() -> Self {
        Self {
            sims: std::ptr::null_mut(),
            sims_count: 0,
            states: std::ptr::null_mut(),
            states_count: 0,
            bodies: std::ptr::null_mut(),
            islands: std::ptr::null(),
            shapes: std::ptr::null_mut(),
            fat_aabbs: std::ptr::null_mut(),
            move_events: std::ptr::null_mut(),
            flags: std::ptr::null_mut(),
            shape_moved: std::ptr::null_mut(),
            threads: std::ptr::null_mut(),
            island_count: 0,
            time_step: 0.0,
            inv_time_step: 0.0,
        }
    }
}

/// A continuous-collision sensor hit recorded by one worker.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SensorHit {
    pub sensor_id: i32,
    pub visitor_id: i32,
}

/// Per-thread task storage.
#[derive(Clone, Debug)]
pub struct TaskContext {
    /// Broad-phase pairs.
    pub pair_keys: Vec<u64>,

    /// Used to put islands to sleep.
    pub awake_island_bit_set: BitSet,

    /// Per worker split island candidate.
    pub split_sleep_time: f32,
    pub split_island_id: i32,

    /// Continuous collision sensor hits.
    pub sensor_hits: Vec<SensorHit>,

    /// Number of contacts recycled this step.
    pub recycled_contact_count: i32,

    /// Per-worker SAT diagnostics, merged after the collide pass.
    pub sat_call_count: i32,
    pub sat_cache_hit_count: i32,

    /// Contact ids whose state changed this pass.
    pub contact_state_bits: BitSet,

    /// Diagnostic iteration counts.
    pub distance_iterations: i32,
    pub push_back_iterations: i32,
    pub root_iterations: i32,
}

impl Default for TaskContext {
    #[inline]
    fn default() -> Self {
        Self {
            pair_keys: Vec::new(),
            awake_island_bit_set: BitSet::new(),
            split_sleep_time: 0.0,
            split_island_id: b3::NULL_INDEX,
            sensor_hits: Vec::new(),
            recycled_contact_count: 0,
            sat_call_count: 0,
            sat_cache_hit_count: 0,
            contact_state_bits: BitSet::new(),
            distance_iterations: 0,
            push_back_iterations: 0,
            root_iterations: 0,
        }
    }
}

impl TaskContext {
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

impl World {
    // MARK: Velocity integration

    /// Integrate velocities, apply damping, and apply gyroscopic torque.
    pub(crate) fn integrate_velocities(&mut self, h: f32) {
        let awake = SetType::AwakeSet.raw_value() as usize;
        let sims_base = self.solver_sets[awake].body_sims.as_ptr();
        let states_base = self.solver_sets[awake].body_states.as_mut_ptr();
        if sims_base.is_null() || states_base.is_null() {
            return;
        }
        let sims_count = self.solver_sets[awake].body_sims.len();
        let states_count = self.solver_sets[awake].body_states.len();
        // SAFETY: the awake set owns both buffers for the duration of this call, they do not
        // overlap, and the range below is bounded by `sims_count`.
        let (sims, states) = unsafe {
            (
                buffer_view(sims_base, sims_count),
                mutable_buffer_view(states_base, states_count),
            )
        };
        let range = 0..sims_count;
        self.integrate_velocities_range(sims, states, range, h);
    }

    /// Integrate the velocities of a range of awake bodies.
    pub(crate) fn integrate_velocities_range(
        &self,
        sims: &[BodySim],
        states: &mut [BodyState],
        range: std::ops::Range<usize>,
        h: f32,
    ) {
        let gravity = self.gravity;
        for i in range {
            let sim = sims[i];
            let mut state = states[i];

            let mut v = state.linear_velocity;
            let mut w = state.angular_velocity;

            // Damping math: v2 = v1 * 1 / (1 + c * dt)
            let linear_damping = 1.0 / (1.0 + h * sim.linear_damping);
            let angular_damping = 1.0 / (1.0 + h * sim.angular_damping);

            // Gravity scale will be zero for kinematic bodies
            let gravity_scale = if sim.inv_mass > 0.0 { sim.gravity_scale } else { 0.0 };

            let linear_velocity_delta =
                blend2(h * sim.inv_mass, sim.force, h * gravity_scale, gravity);
            v = mul_add(linear_velocity_delta, linear_damping, v);

            let angular_velocity_delta = h * mul_mv(sim.inv_inertia_world, sim.torque);
            w = mul_add(angular_velocity_delta, angular_damping, w);

            // Gyroscopic torque by solving this nonlinear equation using Newton-Raphson.
            // I * (w2 - w1) + h * cross(w2, I * w2) = 0
            if (sim.flags & BodyFlags::ENABLE_GYROSCOPIC_TORQUE.0) != 0 {
                // Get current rotation.
                let q0 = sim.transform.q;
                let q = mul_quat(state.delta_rotation, q0);

                // Compute local angular velocity
                let omega1 = inv_rotate_vector(q, w);
                let mut omega2 = omega1;

                // Symmetric inertia tensor: 6 unique entries (column-major)
                let i00 = sim.inertia.cx.x;
                let i01 = sim.inertia.cy.x;
                let i02 = sim.inertia.cz.x;
                let i11 = sim.inertia.cy.y;
                let i12 = sim.inertia.cz.y;
                let i22 = sim.inertia.cz.z;

                for _ in 0..b3::GYROSCOPIC_ITERATIONS as usize {
                    let w1 = omega2.x;
                    let w2 = omega2.y;
                    let w3 = omega2.z;

                    // Iw = I * omega2 (shared between residual and Jacobian)
                    let iw1 = i00 * w1 + i01 * w2 + i02 * w3;
                    let iw2 = i01 * w1 + i11 * w2 + i12 * w3;
                    let iw3 = i02 * w1 + i12 * w2 + i22 * w3;

                    // Residual: b = I * (omega2 - omega1) + h * cross(omega2, I * omega2)
                    let dw = omega2 - omega1;
                    let b = Vec3::new(
                        i00 * dw.x + i01 * dw.y + i02 * dw.z + h * (w2 * iw3 - w3 * iw2),
                        i01 * dw.x + i11 * dw.y + i12 * dw.z + h * (w3 * iw1 - w1 * iw3),
                        i02 * dw.x + i12 * dw.y + i22 * dw.z + h * (w1 * iw2 - w2 * iw1),
                    );

                    // Jacobian J = I + h * (skew(omega2) * I - skew(I * omega2))
                    let j = Matrix3::new(
                        Vec3::new(
                            i00 + h * (w2 * i02 - w3 * i01),
                            i01 + h * (w3 * i00 - w1 * i02 - iw3),
                            i02 + h * (w1 * i01 - w2 * i00 + iw2),
                        ),
                        Vec3::new(
                            i01 + h * (w2 * i12 - w3 * i11 + iw3),
                            i11 + h * (w3 * i01 - w1 * i12),
                            i12 + h * (w1 * i11 - w2 * i01 - iw1),
                        ),
                        Vec3::new(
                            i02 + h * (w2 * i22 - w3 * i12 - iw2),
                            i12 + h * (w3 * i02 - w1 * i22 + iw1),
                            i22 + h * (w1 * i12 - w2 * i02),
                        ),
                    );

                    omega2 = omega2 - solve3(j, b);
                }

                w = rotate_vector(q, omega2);
            }

            state.linear_velocity = v;
            state.angular_velocity = w;
            states[i] = state;
        }
    }

    // MARK: Position integration

    /// Integrate positions and apply motion locks.
    pub(crate) fn integrate_positions(&mut self, h: f32, inv_dt: f32) {
        let awake = SetType::AwakeSet.raw_value() as usize;
        let states_base = self.solver_sets[awake].body_states.as_mut_ptr();
        if states_base.is_null() {
            return;
        }
        let states_count = self.solver_sets[awake].body_states.len();
        // SAFETY: the awake set owns the buffer for the duration of this call.
        let states = unsafe { mutable_buffer_view(states_base, states_count) };
        let range = 0..states_count;
        self.integrate_positions_range(states, range, h, inv_dt);
    }

    /// Integrate the positions of a range of awake bodies.
    pub(crate) fn integrate_positions_range(
        &self,
        states: &mut [BodyState],
        range: std::ops::Range<usize>,
        h: f32,
        inv_dt: f32,
    ) {
        let max_linear_speed = self.max_linear_speed;
        let max_angular_speed = b3::MAX_ROTATION * inv_dt;
        let max_linear_speed_squared = max_linear_speed * max_linear_speed;
        let max_angular_speed_squared = max_angular_speed * max_angular_speed;

        for i in range {
            let mut state = states[i];

            let mut v = state.linear_velocity;
            let mut w = state.angular_velocity;

            // Motion locks - these can be viewed as a constraint that comes last
            v.x = if (state.flags & BodyFlags::LOCK_LINEAR_X.0) != 0 { 0.0 } else { v.x };
            v.y = if (state.flags & BodyFlags::LOCK_LINEAR_Y.0) != 0 { 0.0 } else { v.y };
            v.z = if (state.flags & BodyFlags::LOCK_LINEAR_Z.0) != 0 { 0.0 } else { v.z };
            w.x = if (state.flags & BodyFlags::LOCK_ANGULAR_X.0) != 0 { 0.0 } else { w.x };
            w.y = if (state.flags & BodyFlags::LOCK_ANGULAR_Y.0) != 0 { 0.0 } else { w.y };
            w.z = if (state.flags & BodyFlags::LOCK_ANGULAR_Z.0) != 0 { 0.0 } else { w.z };

            // Clamp to max linear speed
            if dot(v, v) > max_linear_speed_squared {
                let ratio = max_linear_speed / length(v);
                v = ratio * v;
                state.flags |= BodyFlags::IS_SPEED_CAPPED.0;
            }

            // Clamp to max angular speed
            if dot(w, w) > max_angular_speed_squared
                && (state.flags & BodyFlags::ALLOW_FAST_ROTATION.0) == 0
            {
                let ratio = max_angular_speed / length(w);
                w = ratio * w;
                state.flags |= BodyFlags::IS_SPEED_CAPPED.0;
            }

            state.linear_velocity = v;
            state.angular_velocity = w;
            state.delta_position = mul_add(state.delta_position, h, v);
            state.delta_rotation = integrate_rotation(state.delta_rotation, h * w);

            states[i] = state;
        }
    }

    // MARK: Finalize bodies

    /// Update the body transforms from the state deltas.
    ///
    /// The per-body work runs in parallel blocks. Everything that touches shared structures runs
    /// afterwards in body order: the continuous pass of fast bodies, the bullet list, and moving
    /// enlarged proxies in the broad phase, so the move buffer order matches a serial pass.
    pub(crate) fn finalize_bodies(&mut self, context: &mut StepContext) {
        let awake = SetType::AwakeSet.raw_value() as usize;

        // Move the awake set out so its arrays are uniquely referenced; a copy would make the first
        // write duplicate every body sim and state.
        let mut awake_set = SolverSet::default();
        std::mem::swap(&mut awake_set, &mut self.solver_sets[awake]);
        self.finalize_bodies_with(&mut awake_set, context);
        std::mem::swap(&mut awake_set, &mut self.solver_sets[awake]);
    }

    /// The body of `finalize_bodies` with the awake set borrowed out of `solverSets`.
    fn finalize_bodies_with(&mut self, awake_set: &mut SolverSet, context: &mut StepContext) {
        let count = awake_set.body_sims.len();

        self.body_move_events.clear();
        self.body_move_events.resize(count, BodyMoveEvent::default());
        self.finalize_flags.clear();
        self.finalize_flags.resize(count, 0);
        if self.shape_move_flags.len() < self.shapes.len() {
            let extra = self.shapes.len() - self.shape_move_flags.len();
            self.shape_move_flags.resize(self.shape_move_flags.len() + extra, 0);
        }

        // One island bit set and split candidate per thread; thread zero uses the step's own.
        let threads = self.active_pool.as_ref().map_or(1, |pool| pool.thread_count());
        let island_count = awake_set.island_sims.len() as u32;
        while self.finalize_threads.len() < threads {
            self.finalize_threads.push(FinalizeThreadData::new());
        }
        // Swift assigned the step's bit set to thread zero by value, which shares its storage; the
        // merged result is written back below, so a copy of the bit set is equivalent.
        self.finalize_threads[0].islands = self.task_contexts[0].awake_island_bit_set.clone();
        for t in 1..std::cmp::max(threads, 1) {
            self.finalize_threads[t]
                .islands
                .set_bit_count_and_clear(island_count);
        }
        for t in 0..threads {
            self.finalize_threads[t].split_island_id = b3::NULL_INDEX;
            self.finalize_threads[t].split_sleep_time = 0.0;
        }

        let mut views = FinalizeViews::default();
        views.sims = awake_set.body_sims.as_mut_ptr();
        views.sims_count = awake_set.body_sims.len();
        views.states = awake_set.body_states.as_mut_ptr();
        views.states_count = awake_set.body_states.len();
        views.bodies = self.bodies.as_mut_ptr();
        views.islands = self.islands.as_ptr();
        views.shapes = self.shapes.as_mut_ptr();
        views.fat_aabbs = self.fat_aabbs.as_mut_ptr();
        views.move_events = self.body_move_events.as_mut_ptr();
        views.flags = self.finalize_flags.as_mut_ptr();
        views.shape_moved = self.shape_move_flags.as_mut_ptr();
        views.threads = self.finalize_threads.as_mut_ptr();
        views.island_count = island_count as i32;
        views.time_step = context.dt;
        views.inv_time_step = context.inv_dt;
        self.finalize_views = views;

        let continuous_starts = if self.enable_continuous && self.enable_dynamic_continuous {
            awake_set
                .body_sims
                .iter()
                .map(|sim| ContinuousStart {
                    center: sim.center,
                    rotation: sim.transform.q,
                })
                .collect::<Vec<ContinuousStart>>()
        } else {
            Vec::new()
        };
        self.run_stage(SolverStageKind::FinalizeBodies, 0, count, Self::PARALLEL_BODY_ITEMS);

        if !continuous_starts.is_empty() {
            // SAFETY: `views` was just built from the awake set and the step's own arrays; the awake
            // set lives until `finalize_bodies` swaps it back, and no other reference writes these
            // elements while the continuous pass runs on this thread.
            unsafe {
                let sims = mutable_buffer_view(views.sims, views.sims_count);
                let states = mutable_buffer_view(views.states, views.states_count);
                self.solve_dynamic_continuous(&continuous_starts, sims, states);
            }
        }

        // Merge the per-thread island data.
        self.task_contexts[0].awake_island_bit_set = self.finalize_threads[0].islands.clone();
        for t in 0..threads {
            if t > 0 {
                self.task_contexts[0]
                    .awake_island_bit_set
                    .in_place_union(&self.finalize_threads[t].islands);
            }
            let candidate = self.finalize_threads[t].clone();
            if candidate.split_island_id != b3::NULL_INDEX
                && (candidate.split_sleep_time > self.task_contexts[0].split_sleep_time
                    || (candidate.split_sleep_time == self.task_contexts[0].split_sleep_time
                        && candidate.split_island_id > self.task_contexts[0].split_island_id))
            {
                self.task_contexts[0].split_island_id = candidate.split_island_id;
                self.task_contexts[0].split_sleep_time = candidate.split_sleep_time;
            }
        }

        // Shared work in body order.
        for sim_index in 0..count {
            // SAFETY: `finalize_flags` was sized to `count` above and `views.flags` points at it.
            let flag = unsafe { *views.flags.add(sim_index) };
            if flag == 0 {
                continue;
            }
            if flag == FinalizeFlag::BULLET {
                context.bullet_bodies.push(sim_index as i32);
            } else if flag == FinalizeFlag::CONTINUOUS {
                // SAFETY: the awake set's sim and state buffers live for the whole pass and are
                // written only by this thread here.
                unsafe {
                    let sims = mutable_buffer_view(views.sims, views.sims_count);
                    let states = mutable_buffer_view(views.states, views.states_count);
                    self.solve_continuous(sim_index as i32, context.dt, sims, states);
                }
                // The continuous pass may have moved the body back.
                // SAFETY: `views.sims` is the awake set's storage, non-null for `count` elements.
                unsafe {
                    let sim = views.sims.add(sim_index);
                    let rotation_matrix = make_matrix_from_quat((*sim).transform.q);
                    (*sim).inv_inertia_world = mul_mm(
                        mul_mm(rotation_matrix, (*sim).inv_inertia_local),
                        transpose(rotation_matrix),
                    );
                }
            } else if flag == FinalizeFlag::MOVED_PROXIES {
                let body_id = unsafe { (*views.sims.add(sim_index)).body_id };
                let mut shape_id = self.bodies[body_id as usize].head_shape_id;
                while shape_id != b3::NULL_INDEX {
                    let index = shape_id as usize;
                    if self.shape_move_flags[index] != 0 {
                        self.shape_move_flags[index] = 0;
                        let proxy_key = self.shapes[index].proxy_key;
                        let fat_aabb = self.fat_aabbs[index];
                        self.broad_phase.mark_proxy_moved(proxy_key, fat_aabb);
                    }
                    shape_id = self.shapes[index].next_shape_id;
                }
            }
        }
        self.finalize_views = FinalizeViews::default();
    }

    /// Finalize bodies `range` of the awake set. Called on any thread; writes only those bodies,
    /// their shapes, and the thread's own island data.
    pub(crate) fn finalize_body_range(&self, range: std::ops::Range<usize>, thread: usize) {
        let views = self.finalize_views;
        if views.bodies.is_null()
            || views.islands.is_null()
            || views.shapes.is_null()
            || views.fat_aabbs.is_null()
            || views.move_events.is_null()
            || views.flags.is_null()
            || views.shape_moved.is_null()
            || views.threads.is_null()
        {
            return;
        }
        // SAFETY: the pointers above are non-null and were installed by `finalize_bodies`, which
        // owns the awake set and the world's body/shape/event arrays for the whole stage.
        let sims = unsafe { mutable_buffer_view(views.sims, views.sims_count) };
        let states = unsafe { mutable_buffer_view(views.states, views.states_count) };
        let enable_sleep = self.enable_sleep;
        let enable_continuous = self.enable_continuous;
        let time_step = views.time_step;
        let inv_time_step = views.inv_time_step;
        let world_id = self.world_id;
        let speculative_scalar = b3::speculative_distance();
        // SAFETY: `finalize_threads` holds one entry per worker the stage was partitioned for.
        let thread_data = unsafe { &mut *views.threads.add(thread) };
        // Swift aliased the thread's bit set (a pointer-owning struct copy); Rust writes the
        // thread's own bit set in place, which is the same storage.
        let mut split_island_id = thread_data.split_island_id;
        let mut split_sleep_time = thread_data.split_sleep_time;

        for sim_index in range {
            let mut state = states[sim_index];
            let mut sim = sims[sim_index];

            let v = state.linear_velocity;
            let w = state.angular_velocity;
            let local_omega = inv_rotate_vector(sim.transform.q, w);
            let local_delta_rotation = inv_rotate_vector(sim.transform.q, state.delta_rotation.v);

            b3_assert(is_valid_vec3(v));
            b3_assert(is_valid_vec3(w));

            sim.center = offset_pos(sim.center, state.delta_position);
            sim.transform.q = normalize_quat(mul_quat(state.delta_rotation, sim.transform.q));

            // Use the velocity of the farthest point on the body to account for rotation.
            let velocity_arc = modified_cross(abs_vec3(local_omega), sim.max_extent);
            let max_velocity = length(v) + length(velocity_arc);

            // Sleep needs to observe position correction as well as true velocity.
            let rotation_arc = modified_cross(abs_vec3(local_delta_rotation), sim.max_extent);
            let max_delta_position = length(state.delta_position) + 2.0 * length(rotation_arc);

            // Position correction is not as important for sleep as true velocity.
            let position_sleep_factor: f32 = 0.5;
            let sleep_velocity =
                max_float(max_velocity, position_sleep_factor * inv_time_step * max_delta_position);

            // reset state deltas
            state.delta_position = Vec3::ZERO;
            state.delta_rotation = Quat::IDENTITY;

            sim.transform.p = offset_pos(
                sim.center,
                -rotate_vector(sim.transform.q, sim.local_center),
            );

            let body_index = sim.body_id as usize;
            // SAFETY: `views.bodies` is the world's body array, indexed by the same ids the sims
            // carry. `Body` holds a `Vec`-free value layout, so a pointer copy mirrors Swift's
            // array subscript; the value is written back at the end of this iteration.
            let mut body = unsafe { (*views.bodies.add(body_index)).clone() };
            body.body_move_index = sim_index as i32;
            body.sleep_velocity = sleep_velocity;

            let mut move_event = BodyMoveEvent::default();
            move_event.user_data = body.user_data;
            move_event.transform = sim.transform;
            move_event.body_id = BodyID::new(
                sim.body_id + 1,
                world_id,
                body.generation,
            );
            move_event.fell_asleep = false;
            // SAFETY: `move_events` was sized to the awake body count.
            unsafe { *views.move_events.add(sim_index) = move_event };

            // reset applied force and torque
            sim.force = Vec3::ZERO;
            sim.torque = Vec3::ZERO;

            b3_assert((body.flags & BodyFlags::DIRTY_MASS.0) == 0);

            // Clear the transient flags.
            body.flags &= !BodyFlags::BODY_TRANSIENT_FLAGS.0;
            sim.flags &= !(BodyFlags::IS_FAST.0 | BodyFlags::BODY_TRANSIENT_FLAGS.0);

            // The body state flag knows about speed capping.
            body.flags |= state.flags & BodyFlags::IS_SPEED_CAPPED.0;
            state.flags &= !BodyFlags::BODY_TRANSIENT_FLAGS.0;

            if !enable_sleep
                || (body.flags & BodyFlags::ENABLE_SLEEP.0) == 0
                || sleep_velocity > body.sleep_threshold
            {
                // Body is not sleepy
                body.sleep_time = 0.0;

                let safety_factor = body.safety_factor;
                let max_motion = max_float(max_delta_position, max_velocity * time_step);
                if body.type_ == BodyType::DynamicBody
                    && enable_continuous
                    && max_motion > safety_factor * sim.min_extent
                {
                    // This flag is used for debug draw and contact recycling.
                    sim.flags |= BodyFlags::IS_FAST.0;

                    // Bullets run after all bodies; other fast bodies run their continuous pass in the
                    // ordered serial phase, which also updates their shape bounds.
                    // SAFETY: `flags` is the `finalize_flags` array sized to the awake body count.
                    unsafe {
                        *views.flags.add(sim_index) = if (sim.flags & BodyFlags::IS_BULLET.0) != 0
                        {
                            FinalizeFlag::BULLET
                        } else {
                            FinalizeFlag::CONTINUOUS
                        };
                    }
                } else {
                    // Body is safe to advance
                    sim.center0 = sim.center;
                    sim.rotation0 = sim.transform.q;
                }
            } else {
                // Body is safe to advance and is falling asleep
                sim.center0 = sim.center;
                sim.rotation0 = sim.transform.q;
                body.sleep_time += time_step;
            }

            // Update world space inverse inertia tensor.
            let rotation_matrix = make_matrix_from_quat(sim.transform.q);
            sim.inv_inertia_world = mul_mm(
                mul_mm(rotation_matrix, sim.inv_inertia_local),
                transpose(rotation_matrix),
            );

            // Any single body in an island can keep it awake.
            if body.sleep_time < b3::TIME_TO_SLEEP {
                let island_index = unsafe { (*views.islands.add(body.island_id as usize)).local_index };
                b3_assert(island_index >= 0 && island_index < views.island_count);
                if island_index >= 0 && island_index < views.island_count {
                    thread_data.islands.set_bit(island_index as u32);
                }
            } else if unsafe {
                (*views.islands.add(body.island_id as usize)).constraint_remove_count
            } > 0
            {
                if body.sleep_time > split_sleep_time
                    || (body.sleep_time == split_sleep_time && body.island_id > split_island_id)
                {
                    // pick the sleepiest candidate
                    split_island_id = body.island_id;
                    split_sleep_time = body.sleep_time;
                }
            }

            // SAFETY: the body slot is exclusively owned by this sim index's body.
            unsafe { *views.bodies.add(body_index) = body };

            // Update shape bounds. Fast bodies get theirs from the continuous pass.
            if (sim.flags & BodyFlags::IS_FAST.0) == 0 {
                let transform = sim.transform;
                // SAFETY: `views.shape_moved` and `views.fat_aabbs` are indexed by shape id, which
                // the body's shape list owns; this thread is the only writer of these slots.
                unsafe {
                    let mut shape_id = (*views.bodies.add(body_index)).head_shape_id;
                    let mut moved = false;
                    while shape_id != b3::NULL_INDEX {
                        let shape_index = shape_id as usize;
                        let next_shape_id = (*views.shapes.add(shape_index)).next_shape_id;

                        let aabb = compute_fat_shape_aabb(
                            &*views.shapes.add(shape_index),
                            transform,
                            speculative_scalar,
                        );
                        (*views.shapes.add(shape_index)).aabb = aabb;

                        if !aabb_contains(*views.fat_aabbs.add(shape_index), aabb) {
                            *views.fat_aabbs.add(shape_index) = aabb_inflate(
                                aabb,
                                (*views.shapes.add(shape_index)).aabb_margin,
                            );
                            *views.shape_moved.add(shape_index) = 1;
                            moved = true;
                        }

                        shape_id = next_shape_id;
                    }
                    if moved {
                        *views.flags.add(sim_index) = FinalizeFlag::MOVED_PROXIES;
                    }
                }
            }

            states[sim_index] = state;
            sims[sim_index] = sim;
        }

        thread_data.split_island_id = split_island_id;
        thread_data.split_sleep_time = split_sleep_time;
    }

    // MARK: Step context

    /// Advance the simulation. This performs
    /// broad-phase pair updates, integration, and body finalization.
    ///
    /// Swift defaulted `subStepCount` to 4; Rust callers pass it explicitly.
    pub fn step(&mut self, time_step: f32, sub_step_count: i32) {
        b3_assert(time_step >= 0.0);
        self.profile = Default::default();
        let step_ticks = get_ticks();
        let mut ticks = step_ticks;

        self.body_move_events.clear();
        self.sensor_begin_events.clear();
        self.contact_begin_events.clear();
        self.continuous_impact_events.clear();

        self.update_broad_phase_pairs();
        self.profile.pairs = get_milliseconds_and_reset(&mut ticks);
        let mut context = StepContext::default();
        context.dt = time_step;
        context.sub_step_count = max_int(1, sub_step_count);

        if time_step > 0.0 {
            context.inv_dt = 1.0 / time_step;
            context.h = time_step / context.sub_step_count as f32;
            context.inv_h = context.sub_step_count as f32 * context.inv_dt;
        } else {
            context.inv_dt = 0.0;
            context.h = 0.0;
            context.inv_h = 0.0;
        }

        self.inv_h = context.inv_h;
        self.inv_dt = context.inv_dt;

        // Hertz values get reduced for large time steps
        let contact_hertz = min_float(self.contact_hertz, 0.125 * context.inv_h);
        context.contact_softness =
            make_soft(contact_hertz, self.contact_damping_ratio, context.h);
        context.static_softness = make_soft(
            2.0 * contact_hertz,
            0.5 * self.contact_damping_ratio,
            context.h,
        );

        context.max_linear_velocity = self.max_linear_speed;
        context.enable_warm_starting = self.enable_warm_starting;

        // Narrow phase: update contacts
        self.task_contexts[0].recycled_contact_count = 0;
        if time_step > 0.0 {
            #[cfg(debug_assertions)]
            b3_assert(self.validate_islands().is_none());
            self.collide(&mut context);
        }
        self.profile.collide = get_milliseconds_and_reset(&mut ticks);

        if time_step > 0.0 {
            self.step_index += 1;

            // Wheel casts may wake bodies, so they run before the awake set is sized.
            self.update_vehicle_collisions();
            self.profile.vehicles = get_milliseconds_and_reset(&mut ticks);

            let awake = SetType::AwakeSet.raw_value() as usize;
            let awake_count = self.solver_sets[awake].body_sims.len();
            if awake_count == 0 {
                self.step_soft_bodies(context.dt);
                self.accept_sensor_hits();
                self.overlap_sensors();
                self.sensor_end_events.publish();
                self.contact_end_events.publish();
                return;
            }

            context.bullet_bodies.clear();

            // Finish a pending island split from the previous step.
            let pending_split = self.split_island_id;
            if pending_split != b3::NULL_INDEX {
                self.split_island(pending_split);
                self.split_island_id = b3::NULL_INDEX;
            }

            // Workers spin from preparation through impulse storage.
            self.begin_solver_session();
            self.prepare_all_contacts(&context);
            self.prepare_vehicles(&context);
            self.profile.prepare_constraints = get_milliseconds_and_reset(&mut ticks);

            // Substeps, restitution, and impulse storage, on the worker pool when there is one.
            self.solve_constraint_stages(&context);
            ticks = get_ticks();
            self.finalize_vehicles(&context);
            self.profile.store_impulses = get_milliseconds_and_reset(&mut ticks);
            self.emit_contact_hit_events();

            // Prepare the island bit set used in body finalization.
            let awake_island_count = self.solver_sets[awake].island_sims.len() as u32;
            for i in 0..self.task_contexts.len() {
                self.task_contexts[i]
                    .awake_island_bit_set
                    .set_bit_count_and_clear(awake_island_count);
                self.task_contexts[i].split_island_id = b3::NULL_INDEX;
                self.task_contexts[i].split_sleep_time = 0.0;
            }

            self.emit_joint_events(context.inv_h);

            self.finalize_bodies(&mut context);
            self.end_solver_session();
            // Sleeping targets can only be migrated after finalization releases its raw views.
            // Swift's `Set(...).sorted()` is the deduplicated ascending order of the ids.
            let mut active_bodies = self.dynamic_continuous_active_bodies.clone();
            active_bodies.sort_unstable();
            active_bodies.dedup();
            for body_id in active_bodies {
                let old_count = self.solver_sets[awake].island_sims.len();
                self.wake_body(body_id);
                let new_count = self.solver_sets[awake].island_sims.len();
                for i in old_count..new_count {
                    self.task_contexts[0]
                        .awake_island_bit_set
                        .set_bit_grow(i as u32);
                }
                let island_id = self.bodies[body_id as usize].island_id;
                let local_index = self.islands[island_id as usize].local_index;
                self.task_contexts[0]
                    .awake_island_bit_set
                    .set_bit_grow(local_index as u32);
                self.bodies[body_id as usize].sleep_time = 0.0;
            }
            // Swift iterated the impulse array by value; cloning lets the loop mutate the world.
            let impacts = self.dynamic_continuous_impulses.clone();
            for impact in impacts {
                let id = self.make_body_id(impact.body);
                self.apply_body_linear_impulse(id, impact.impulse, impact.point, false);
            }
            self.dynamic_continuous_impulses.clear();
            self.dynamic_continuous_active_bodies.clear();
            self.profile.transforms = get_milliseconds_and_reset(&mut ticks);

            // Refit the kinematic and dynamic trees after body finalization.
            self.broad_phase.trees[BodyType::KinematicBody as usize].refit();
            self.broad_phase.trees[BodyType::DynamicBody as usize].refit();
            self.profile.refit = get_milliseconds_and_reset(&mut ticks);

            // Fast bullet bodies: run the swept pass, then enlarge their broad-phase proxies.
            if !context.bullet_bodies.is_empty() {
                for i in 0..context.bullet_bodies.len() {
                    let index = context.bullet_bodies[i];
                    let dt = context.dt;
                    self.solve_continuous(index, dt);
                }

                let dynamic_index = BodyType::DynamicBody as usize;
                // Swift copied the awake set out and back; swapping keeps the same final state
                // without duplicating every sim and state.
                let mut awake_set = SolverSet::default();
                std::mem::swap(&mut awake_set, &mut self.solver_sets[awake]);
                for i in 0..context.bullet_bodies.len() {
                    let index = context.bullet_bodies[i];
                    let mut sim = awake_set.body_sims[index as usize];
                    if (sim.flags & BodyFlags::ENLARGE_BULLET_BOUNDS.0) == 0 {
                        continue;
                    }

                    // Clear flag
                    sim.flags &= !BodyFlags::ENLARGE_BULLET_BOUNDS.0;

                    let bullet_body = self.bodies[sim.body_id as usize].clone();
                    let mut shape_id = bullet_body.head_shape_id;
                    while shape_id != b3::NULL_INDEX {
                        let shape = self.shapes[shape_id as usize].clone();
                        let proxy_id =
                            crate::simulation::broad_phase::proxy_id_of_key(shape.proxy_key);
                        let tree_aabb =
                            self.broad_phase.trees[dynamic_index].get_aabb(proxy_id);
                        let shape_fat_aabb = self.fat_aabbs[shape_id as usize];

                        if !aabb_contains(tree_aabb, shape_fat_aabb) {
                            self.broad_phase.trees[dynamic_index].enlarge_proxy(
                                proxy_id,
                                shape_fat_aabb,
                            );
                        }

                        shape_id = shape.next_shape_id;
                    }

                    awake_set.body_sims[index as usize] = sim;
                }
                std::mem::swap(&mut awake_set, &mut self.solver_sets[awake]);
            }

            self.profile.bullets = get_milliseconds_and_reset(&mut ticks);

            // Soft bodies see the finalized rigid bodies and feed impulses into the next step.
            self.step_soft_bodies(context.dt);
            self.profile.soft_bodies = get_milliseconds_and_reset(&mut ticks);

            if self.enable_sleep {
                self.process_sleep();
            }
            self.profile.sleep_islands = get_milliseconds_and_reset(&mut ticks);

            self.accept_sensor_hits();
            self.overlap_sensors();
        }

        self.profile.step = get_milliseconds(step_ticks);
        self.sensor_end_events.publish();
        self.contact_end_events.publish();
    }
}

/// Step context: the scalars every solver stage needs.
///
/// Swift passed this struct by value to the solver kernels and used `inout` only where the kernels
/// write (`collide`, `finalizeBodies`). Rust borrows it: `&StepContext` where Swift copied,
/// `&mut StepContext` where Swift used `inout`.
#[derive(Clone, Debug, Default)]
pub struct StepContext {
    pub dt: f32,
    pub inv_dt: f32,
    pub h: f32,
    pub inv_h: f32,
    pub sub_step_count: i32,
    pub max_linear_velocity: f32,
    pub contact_softness: Softness,
    pub static_softness: Softness,
    pub enable_warm_starting: bool,

    /// Fast bullet bodies that need continuous collision handling.
    pub bullet_bodies: Vec<i32>,
}
