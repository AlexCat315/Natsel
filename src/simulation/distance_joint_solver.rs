//! Joint solver pipeline and the distance joint.
//!
//! Translated from `Sources/Natsel/Simulation/DistanceJointSolver.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::joint`: `World::create_joint(&mut self, &JointDef, JointType) -> JointPair`,
//!   `World::write_joint_sim(&mut self, i32, JointSim)`, and the `JointSim::distance_joint` /
//!   `set_distance_joint` accessors.
//! - `crate::simulation::contact_solver`: the `World::state_at` / `World::set_state_at` buffer
//!   accessors over the raw awake-set state pointer, mirroring Swift.
//! - `crate::simulation::solver_prepare`: `World::joint_prepare_body` / `joint_prepare_sim`.
//! - `crate::simulation::wide_joint_solver`: `SphericalConstraintW`, `World::warm_start_spherical_w`
//!   (by value, like Swift's `warmStartSphericalW`) and `World::solve_spherical_w` (inout + `useBias`).
//! - `crate::simulation::revolute_joint`, `weld_joint`, `parallel_joint`, `motor_joint`,
//!   `prismatic_joint`, `spherical_joint`, `wheel_joint`, `coupling_joints`, `six_dof_joint`,
//!   `path_joint`: the per-kind `prepare_*_joint` / `warm_start_*_joint` / `solve_*_joint` stages.

use std::ops::Range;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::math::functions::{
    clamp_float, cross, dot, length, max_float, min_float, mul_mv, normalize, rotate_vector,
    sub_pos,
};
use crate::simulation::solver_integration::{StepContext, make_soft, mutable_buffer_view};
use crate::simulation::wide_joint_solver::SphericalConstraintW;
use crate::simulation::world_runtime::{
    BodyFlags, BodySim, BodyState, JointSim, SetType, World,
};
use crate::types::joint_defs::JointType;

/// One color's view into its joints and wide batches, with raw pointers into the solver arrays.
/// Mirrors Swift's `World.JointColorView`.
///
/// Swift held `UnsafeMutablePointer<JointSim>?` / `UnsafePointer<Int>?` optionals; the landed
/// `solver_stages::build_solver_views` installs plain raw pointers and uses null for Swift's nil,
/// so the optionals become nullable pointers checked with `is_null()`.
#[derive(Clone, Copy)]
pub(crate) struct JointColorView {
    pub joints: *mut JointSim,
    pub joint_count: usize,
    pub starts: *const isize,
    pub batches: *mut SphericalConstraintW,
    pub units: *const i32,
    /// Units to solve: wide batches count once.
    pub unit_count: usize,
}

impl Default for JointColorView {
    /// Swift's `JointColorView()` defaults every field to nil/0, which the derived pointer
    /// default cannot express.
    #[inline]
    fn default() -> Self {
        Self {
            joints: std::ptr::null_mut(),
            joint_count: 0,
            starts: std::ptr::null(),
            batches: std::ptr::null_mut(),
            units: std::ptr::null(),
            unit_count: 0,
        }
    }
}

impl World {
    /// Fetch the body sim for a body id from whatever set owns it.
    pub(crate) fn joint_body_sim(&self, body_id: i32) -> BodySim {
        let body = &self.bodies[body_id as usize];
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
    }

    pub(crate) fn prepare_joint(&self, sim: *mut JointSim, context: &StepContext) {
        // SAFETY: the caller (`solver_prepare::prepare_joint_range` and `prepare_joints` below)
        // passes a pointer into a color's `joint_sims` list installed for the whole prepare stage;
        // it is non-null, aligned, and points to one live `JointSim` that nothing else aliases
        // while this joint is prepared, exactly Swift's `inout JointSim`.
        let base = unsafe { &mut *sim };

        // Clamp joint hertz based on the time step to reduce jitter.
        let hertz = min_float(base.constraint_hertz, 0.25 * context.inv_h);
        base.constraint_softness = make_soft(hertz, base.constraint_damping_ratio, context.h);

        match base.type_ {
            JointType::DistanceJoint => self.prepare_distance_joint(base, context),
            JointType::RevoluteJoint => self.prepare_revolute_joint(base, context),
            JointType::WeldJoint => self.prepare_weld_joint(base, context),
            JointType::ParallelJoint => self.prepare_parallel_joint(base, context),
            JointType::MotorJoint => self.prepare_motor_joint(base, context),
            JointType::PrismaticJoint => self.prepare_prismatic_joint(base, context),
            JointType::SphericalJoint => self.prepare_spherical_joint(base, context),
            JointType::WheelJoint => self.prepare_wheel_joint(base, context),
            JointType::GearJoint => self.prepare_gear_joint(base, context),
            JointType::RackAndPinionJoint => self.prepare_rack_and_pinion_joint(base, context),
            JointType::PulleyJoint => self.prepare_pulley_joint(base, context),
            JointType::SixDOFJoint => self.prepare_six_dof_joint(base, context),
            JointType::PathJoint => self.prepare_path_joint(base, context),
            _ => {}
        }
    }

    pub(crate) fn warm_start_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        states: &mut [BodyState],
    ) {
        match base.type_ {
            JointType::DistanceJoint => {
                self.warm_start_distance_joint(base, context, states.as_mut_ptr())
            }
            JointType::RevoluteJoint => {
                self.warm_start_revolute_joint(base, context, states.as_mut_ptr())
            }
            JointType::WeldJoint => self.warm_start_weld_joint(base, context, states.as_mut_ptr()),
            JointType::ParallelJoint => {
                self.warm_start_parallel_joint(base, context, states.as_mut_ptr())
            }
            JointType::MotorJoint => self.warm_start_motor_joint(base, context, states.as_mut_ptr()),
            JointType::PrismaticJoint => {
                self.warm_start_prismatic_joint(base, context, states.as_mut_ptr())
            }
            JointType::SphericalJoint => {
                self.warm_start_spherical_joint(base, context, states.as_mut_ptr())
            }
            JointType::WheelJoint => self.warm_start_wheel_joint(base, context, states.as_mut_ptr()),
            JointType::GearJoint => self.warm_start_gear_joint(base, states.as_mut_ptr()),
            JointType::RackAndPinionJoint => {
                self.warm_start_rack_and_pinion_joint(base, states.as_mut_ptr())
            }
            JointType::PulleyJoint => self.warm_start_pulley_joint(base, states.as_mut_ptr()),
            JointType::SixDOFJoint => self.warm_start_six_dof_joint(base, states),
            JointType::PathJoint => self.warm_start_path_joint(base, states),
            _ => {}
        }
    }

    pub(crate) fn solve_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        use_bias: bool,
        states: &mut [BodyState],
    ) {
        match base.type_ {
            JointType::DistanceJoint => {
                self.solve_distance_joint(base, context, use_bias, states.as_mut_ptr())
            }
            JointType::RevoluteJoint => {
                self.solve_revolute_joint(base, context, use_bias, states.as_mut_ptr())
            }
            JointType::WeldJoint => self.solve_weld_joint(base, context, use_bias, states.as_mut_ptr()),
            JointType::ParallelJoint => self.solve_parallel_joint(base, context, states.as_mut_ptr()),
            JointType::MotorJoint => self.solve_motor_joint(base, context, states.as_mut_ptr()),
            JointType::PrismaticJoint => {
                self.solve_prismatic_joint(base, context, use_bias, states.as_mut_ptr())
            }
            JointType::SphericalJoint => {
                self.solve_spherical_joint(base, context, use_bias, states.as_mut_ptr())
            }
            JointType::WheelJoint => self.solve_wheel_joint(base, context, use_bias, states.as_mut_ptr()),
            JointType::GearJoint => self.solve_gear_joint(base, context, use_bias, states.as_mut_ptr()),
            JointType::RackAndPinionJoint => {
                self.solve_rack_and_pinion_joint(base, context, use_bias, states.as_mut_ptr())
            }
            JointType::PulleyJoint => {
                self.solve_pulley_joint(base, context, use_bias, states.as_mut_ptr())
            }
            JointType::SixDOFJoint => self.solve_six_dof_joint(base, context, use_bias, states),
            JointType::PathJoint => self.solve_path_joint(base, context, use_bias, states),
            _ => {}
        }
    }

    // MARK: Pipeline

    pub(crate) fn warm_start_all_joints(&mut self, context: &StepContext) {
        for ci in 0..b3::GRAPH_COLOR_COUNT as usize {
            self.warm_start_joints(ci as i32, context);
        }
    }

    pub(crate) fn solve_all_joints(&mut self, context: &StepContext, use_bias: bool) {
        for ci in 0..b3::GRAPH_COLOR_COUNT as usize {
            self.solve_joints(ci as i32, context, use_bias);
        }
    }

    // MARK: Per-color joint stages

    /// Prepare the joints of a single graph color.
    ///
    /// The array is mutated through raw pointers so the (large) `JointSim` value does not get
    /// copied out and back per color, and no exclusive-access nesting is needed.
    pub(crate) fn prepare_joints(&mut self, color_index: i32, context: &StepContext) {
        let list = &mut self.constraint_graph.colors[color_index as usize].joint_sims;
        let base = list.as_mut_ptr();
        let count = list.len();
        for i in 0..count {
            // SAFETY: `base` is the color's `joint_sims` buffer, which is not resized during the
            // prepare stage, and `i` is inside `0..count`. Each joint is touched only once.
            self.prepare_joint(unsafe { base.add(i) }, context);
        }
    }

    /// Lend one color's joints with fresh pointers (the serial path), exactly as Swift's
    /// `withJointColor` nested the four buffer borrows.
    fn joint_color_view(&mut self, ci: usize) -> JointColorView {
        // Local copies of the read-only arrays: reading `solver_joints` while its batches are
        // borrowed for writing would be an overlapping access.
        let start_list = &self.solver_joints.starts[ci];
        let wide = self.enable_wide_solver && start_list.is_empty() == false;

        let mut view = JointColorView::default();
        view.joints = self.constraint_graph.colors[ci].joint_sims.as_mut_ptr();
        view.joint_count = self.constraint_graph.colors[ci].joint_sims.len();
        if wide {
            view.starts = self.solver_joints.starts[ci].as_ptr();
            view.batches = self.solver_joints.batches[ci].as_mut_ptr();
            view.units = self.solver_joints.units[ci].as_ptr();
            view.unit_count = self.solver_joints.units[ci].len();
        } else {
            view.unit_count = view.joint_count;
        }
        view
    }

    /// Warm start joint units of one color. A unit is a scalar joint or a wide spherical batch.
    pub(crate) fn warm_start_joints_range(
        &self,
        view: JointColorView,
        range: Range<usize>,
        context: &StepContext,
        states: &mut [BodyState],
    ) {
        if view.joints.is_null() {
            return;
        }
        for u in range {
            let i = if view.units.is_null() {
                u
            } else {
                // SAFETY: `units` is Swift's `view.units.map { Int($0[u]) }`; it points into
                // `solver_joints.units[ci]`, which has exactly `unit_count` entries, and `u` is a
                // unit index clamped below `unit_count` by the caller.
                (unsafe { *view.units.add(u) }) as usize
            };
            let start = if view.starts.is_null() || view.batches.is_null() {
                // Swift's `if let starts = view.starts, let batches = view.batches` fell through to
                // the scalar joint when either pointer was nil; a negative sentinel reproduces that
                // branch choice exactly.
                -1
            } else {
                // SAFETY: `starts` points into `solver_joints.starts[ci]`, which has one entry per
                // joint, and `i < view.joint_count` because it came from a joint index.
                unsafe { *view.starts.add(i) }
            };
            if start >= 0 {
                let batch = start as usize;
                // SAFETY: `batches` points into `solver_joints.batches[ci]`, which has an entry at
                // every non-negative start. Swift passed the batch by value here.
                let c = unsafe { *view.batches.add(batch) };
                self.warm_start_spherical_w(c, states);
            } else {
                // SAFETY: `joints` is the color's `joint_sims` buffer, not resized while the units
                // are solved, and `i` indexes a live joint; only this joint's lane is written, the
                // graph color keeping its bodies disjoint from every other lane.
                let joint = unsafe { &mut *view.joints.add(i) };
                self.warm_start_joint(joint, context, states);
            }
        }
    }

    pub(crate) fn solve_joints_range(
        &self,
        view: JointColorView,
        range: Range<usize>,
        context: &StepContext,
        use_bias: bool,
        states: &mut [BodyState],
    ) {
        if view.joints.is_null() {
            return;
        }
        for u in range {
            let i = if view.units.is_null() {
                u
            } else {
                // SAFETY: as in `warm_start_joints_range`.
                (unsafe { *view.units.add(u) }) as usize
            };
            let start = if view.starts.is_null() || view.batches.is_null() {
                -1
            } else {
                // SAFETY: as in `warm_start_joints_range`.
                unsafe { *view.starts.add(i) }
            };
            if start >= 0 {
                let batch = start as usize;
                // SAFETY: as in `warm_start_joints_range`; Swift passed the batch `inout`, so the
                // wide solver mutates the batch stored in `solver_joints.batches[ci]`.
                let c = unsafe { &mut *view.batches.add(batch) };
                self.solve_spherical_w(c, states, use_bias);
            } else {
                // SAFETY: as in `warm_start_joints_range`.
                let joint = unsafe { &mut *view.joints.add(i) };
                self.solve_joint(joint, context, use_bias, states);
            }
        }
    }

    /// Warm start the joints of a single graph color.
    pub(crate) fn warm_start_joints(&mut self, color_index: i32, context: &StepContext) {
        let ci = color_index as usize;
        if self.constraint_graph.colors[ci].joint_sims.is_empty() {
            return;
        }
        let view = self.joint_color_view(ci);
        let awake = SetType::AwakeSet.raw_value() as usize;
        let states_base = self.solver_sets[awake].body_states.as_mut_ptr();
        let states_count = self.solver_sets[awake].body_states.len();
        // SAFETY: the awake set's state array is Swift's borrowed `UnsafeMutableBufferPointer` for
        // one solver phase; the color's joints touch only its own bodies, the array is not resized
        // or freed during the phase, and `mutable_buffer_view` detaches the lifetime exactly as
        // `solver_stages::build_solver_views` does for the parallel path.
        let states = unsafe { mutable_buffer_view(states_base, states_count) };
        self.warm_start_joints_range(view, 0..view.unit_count, context, states);
    }

    /// Solve the joints of a single graph color.
    pub(crate) fn solve_joints(&mut self, color_index: i32, context: &StepContext, use_bias: bool) {
        let ci = color_index as usize;
        if self.constraint_graph.colors[ci].joint_sims.is_empty() {
            return;
        }
        let view = self.joint_color_view(ci);
        let awake = SetType::AwakeSet.raw_value() as usize;
        let states_base = self.solver_sets[awake].body_states.as_mut_ptr();
        let states_count = self.solver_sets[awake].body_states.len();
        // SAFETY: as in `warm_start_joints`.
        let states = unsafe { mutable_buffer_view(states_base, states_count) };
        self.solve_joints_range(view, 0..view.unit_count, context, use_bias, states);
    }

    // MARK: Distance joint

    pub(crate) fn prepare_distance_joint(&self, base: &mut JointSim, context: &StepContext) {
        b3_assert(base.type_ == JointType::DistanceJoint);

        let body_a = self.joint_prepare_body(base.body_id_a);
        let body_b = self.joint_prepare_body(base.body_id_b);

        b3_assert(
            body_a.set_index == SetType::AwakeSet.raw_value()
                || body_b.set_index == SetType::AwakeSet.raw_value(),
        );

        let sim_a = self.joint_prepare_sim(&body_a);
        let sim_b = self.joint_prepare_sim(&body_b);

        let mA = sim_a.inv_mass;
        let i_a = sim_a.inv_inertia_world;
        let mB = sim_b.inv_mass;
        let i_b = sim_b.inv_inertia_world;

        base.inv_mass_a = mA;
        base.inv_mass_b = mB;
        base.inv_ia = i_a;
        base.inv_ib = i_b;

        let mut joint = base.distance_joint();

        joint.index_a = if body_a.set_index == SetType::AwakeSet.raw_value() {
            body_a.local_index
        } else {
            b3::NULL_INDEX
        };
        joint.index_b = if body_b.set_index == SetType::AwakeSet.raw_value() {
            body_b.local_index
        } else {
            b3::NULL_INDEX
        };

        // initial anchors in world space
        joint.anchor_a = rotate_vector(sim_a.transform.q, base.local_frame_a.p - sim_a.local_center);
        joint.anchor_b = rotate_vector(sim_b.transform.q, base.local_frame_b.p - sim_b.local_center);
        joint.delta_center = sub_pos(sim_b.center, sim_a.center);

        let r_a = joint.anchor_a;
        let r_b = joint.anchor_b;
        let separation = (r_b - r_a) + joint.delta_center;
        let axis = normalize(separation);

        // compute effective mass
        let cr_a = cross(r_a, axis);
        let cr_b = cross(r_b, axis);
        let k = mA + mB + dot(cr_a, mul_mv(i_a, cr_a)) + dot(cr_b, mul_mv(i_b, cr_b));
        joint.axial_mass = if k > 0.0 { 1.0 / k } else { 0.0 };

        joint.distance_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

        if context.enable_warm_starting == false {
            joint.impulse = 0.0;
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
            joint.motor_impulse = 0.0;
        }

        base.set_distance_joint(joint);
    }

    pub(crate) fn warm_start_distance_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::DistanceJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let i_a = base.inv_ia;
        let i_b = base.inv_ib;

        let joint = base.distance_joint();
        // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
        // awake set's state array, installed by the solver stage for the whole warm-start phase.
        // It outlives this call and the indices were encoded during prepare, exactly as in Swift;
        // no aliasing exists between this buffer and `base`.
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);

        let r_a = rotate_vector(state_a.delta_rotation, joint.anchor_a);
        let r_b = rotate_vector(state_b.delta_rotation, joint.anchor_b);

        let d_s = (state_b.delta_position - state_a.delta_position) + (r_b - r_a);
        let separation = joint.delta_center + d_s;
        let axis = normalize(separation);

        let axial_impulse =
            joint.impulse + joint.lower_impulse - joint.upper_impulse + joint.motor_impulse;
        let p = axial_impulse * axis;

        if (state_a.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_a.linear_velocity = state_a.linear_velocity - mA * p;
            state_a.angular_velocity = state_a.angular_velocity - mul_mv(i_a, cross(r_a, p));
        }

        if (state_b.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_b.linear_velocity = state_b.linear_velocity + mB * p;
            state_b.angular_velocity = state_b.angular_velocity + mul_mv(i_b, cross(r_b, p));
        }

        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
    }

    pub(crate) fn solve_distance_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        use_bias: bool,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::DistanceJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let i_a = base.inv_ia;
        let i_b = base.inv_ib;

        let mut joint = base.distance_joint();
        // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
        // awake set's state array, installed by the solver stage for the whole solve phase. The
        // sim in `base` and this buffer do not alias; Swift read and wrote whole state values via
        // `stateAt`/`setStateAt` and the same in-place copy-back happens here.
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);

        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;

        // current anchors
        let r_a = rotate_vector(state_a.delta_rotation, joint.anchor_a);
        let r_b = rotate_vector(state_b.delta_rotation, joint.anchor_b);

        // current separation
        let d_s = (state_b.delta_position - state_a.delta_position) + (r_b - r_a);
        let separation = joint.delta_center + d_s;

        let length = length(separation);
        let axis = normalize(separation);

        // joint is soft if
        // - spring is enabled
        // - and (joint limit is disabled or limits are not equal)
        if joint.enable_spring && (joint.min_length < joint.max_length || joint.enable_limit == false)
        {
            // spring
            if joint.hertz > 0.0 {
                // Cdot = dot(u, v + cross(w, r))
                let v_r = (v_b - v_a) + (cross(w_b, r_b) - cross(w_a, r_a));
                let c_dot = dot(axis, v_r);
                let c = length - joint.length;
                let bias = joint.distance_softness.bias_rate * c;

                let m = joint.distance_softness.mass_scale * joint.axial_mass;
                let old_impulse = joint.impulse;
                let mut impulse =
                    -m * (c_dot + bias) - joint.distance_softness.impulse_scale * old_impulse;
                let h = context.h;
                joint.impulse = clamp_float(
                    joint.impulse + impulse,
                    joint.lower_spring_force * h,
                    joint.upper_spring_force * h,
                );
                impulse = joint.impulse - old_impulse;

                let p = impulse * axis;
                v_a = v_a - mA * p;
                w_a = w_a - mul_mv(i_a, cross(r_a, p));
                v_b = v_b + mB * p;
                w_b = w_b + mul_mv(i_b, cross(r_b, p));
            }

            if joint.enable_limit {
                // lower limit
                {
                    let v_r = (v_b - v_a) + (cross(w_b, r_b) - cross(w_a, r_a));
                    let c_dot = dot(axis, v_r);

                    let c = length - joint.min_length;

                    let mut bias: f32 = 0.0;
                    let mut mass_coeff: f32 = 1.0;
                    let mut impulse_coeff: f32 = 0.0;
                    if c > 0.0 {
                        // speculative
                        bias = c * context.inv_h;
                    } else if use_bias {
                        bias = base.constraint_softness.bias_rate * c;
                        mass_coeff = base.constraint_softness.mass_scale;
                        impulse_coeff = base.constraint_softness.impulse_scale;
                    }

                    let mut impulse = -mass_coeff * joint.axial_mass * (c_dot + bias)
                        - impulse_coeff * joint.lower_impulse;
                    let new_impulse = max_float(0.0, joint.lower_impulse + impulse);
                    impulse = new_impulse - joint.lower_impulse;
                    joint.lower_impulse = new_impulse;

                    let p = impulse * axis;
                    v_a = v_a - mA * p;
                    w_a = w_a - mul_mv(i_a, cross(r_a, p));
                    v_b = v_b + mB * p;
                    w_b = w_b + mul_mv(i_b, cross(r_b, p));
                }

                // upper
                {
                    let v_r = (v_a - v_b) + (cross(w_a, r_a) - cross(w_b, r_b));
                    let c_dot = dot(axis, v_r);

                    let c = joint.max_length - length;

                    let mut bias: f32 = 0.0;
                    let mut mass_scale: f32 = 1.0;
                    let mut impulse_scale: f32 = 0.0;
                    if c > 0.0 {
                        // speculative
                        bias = c * context.inv_h;
                    } else if use_bias {
                        bias = base.constraint_softness.bias_rate * c;
                        mass_scale = base.constraint_softness.mass_scale;
                        impulse_scale = base.constraint_softness.impulse_scale;
                    }

                    let mut impulse = -mass_scale * joint.axial_mass * (c_dot + bias)
                        - impulse_scale * joint.upper_impulse;
                    let new_impulse = max_float(0.0, joint.upper_impulse + impulse);
                    impulse = new_impulse - joint.upper_impulse;
                    joint.upper_impulse = new_impulse;

                    let p = (-impulse) * axis;
                    v_a = v_a - mA * p;
                    w_a = w_a - mul_mv(i_a, cross(r_a, p));
                    v_b = v_b + mB * p;
                    w_b = w_b + mul_mv(i_b, cross(r_b, p));
                }
            }

            if joint.enable_motor {
                let v_r = (v_b - v_a) + (cross(w_b, r_b) - cross(w_a, r_a));
                let c_dot = dot(axis, v_r);
                let mut impulse = joint.axial_mass * (joint.motor_speed - c_dot);
                let old_impulse = joint.motor_impulse;
                let max_impulse = context.h * joint.max_motor_force;
                joint.motor_impulse =
                    clamp_float(joint.motor_impulse + impulse, -max_impulse, max_impulse);
                impulse = joint.motor_impulse - old_impulse;

                let p = impulse * axis;
                v_a = v_a - mA * p;
                w_a = w_a - mul_mv(i_a, cross(r_a, p));
                v_b = v_b + mB * p;
                w_b = w_b + mul_mv(i_b, cross(r_b, p));
            }
        } else {
            // rigid constraint
            let v_r = (v_b - v_a) + (cross(w_b, r_b) - cross(w_a, r_a));
            let c_dot = dot(axis, v_r);

            let c = length - joint.length;

            let mut bias: f32 = 0.0;
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias {
                bias = base.constraint_softness.bias_rate * c;
                mass_scale = base.constraint_softness.mass_scale;
                impulse_scale = base.constraint_softness.impulse_scale;
            }

            let impulse =
                -mass_scale * joint.axial_mass * (c_dot + bias) - impulse_scale * joint.impulse;
            joint.impulse += impulse;

            let p = impulse * axis;
            v_a = v_a - mA * p;
            w_a = w_a - mul_mv(i_a, cross(r_a, p));
            v_b = v_b + mB * p;
            w_b = w_b + mul_mv(i_b, cross(r_b, p));
        }

        if (state_a.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_a.linear_velocity = v_a;
            state_a.angular_velocity = w_a;
        }

        if (state_b.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_b.linear_velocity = v_b;
            state_b.angular_velocity = w_b;
        }

        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);

        base.set_distance_joint(joint);
    }
}
