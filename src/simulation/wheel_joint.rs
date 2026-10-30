//! Wheel joint. Suspension along A's x-axis, spin about B's z-axis, optional
//! steering about A's x-axis.
//!
//! Translated from `Sources/Natsel/Simulation/WheelJoint.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::joint`: `World::create_joint(&mut self, JointDef, JointType) -> JointPair`
//!   and `World::write_joint_sim(&mut self, i32, JointSim)`.
//! - `crate::simulation::contact_solver`: the `World::state_at` / `World::set_state_at` buffer
//!   accessors, kept safe wrappers over the raw awake-set state pointer, mirroring Swift.
//! - `crate::simulation::solver_prepare`: `World::joint_prepare_body` / `joint_prepare_sim`.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::internal_math::{blend3, solve2, Matrix2};
use crate::math::functions::{
    add_mm, atan2_approx, blend2, clamp_float, cross, det, dot, dot_quat, inv_mul_quat,
    make_matrix_from_quat, max_float, mul_mv, mul_quat, negate_quat, rotate_vector, sub_pos,
};
use crate::math::types::{Matrix3, Transform, Vec2, Vec3};
use crate::simulation::solver_integration::{Softness, StepContext, make_soft};
use crate::simulation::world_runtime::{BodyFlags, BodyState, JointSim, SetType, World};
use crate::types::ids::JointID;
use crate::types::joint_defs::{JointType, WheelJointDef};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct WheelJointSim {
    pub linear_impulse: Vec2,
    pub angular_impulse: Vec2,
    pub spin_impulse: f32,
    pub max_spin_torque: f32,
    pub spin_speed: f32,
    pub suspension_spring_impulse: f32,
    pub lower_suspension_impulse: f32,
    pub upper_suspension_impulse: f32,
    pub lower_suspension_limit: f32,
    pub upper_suspension_limit: f32,
    pub suspension_hertz: f32,
    pub suspension_damping_ratio: f32,
    pub steering_spring_impulse: f32,
    pub lower_steering_impulse: f32,
    pub upper_steering_impulse: f32,
    pub lower_steering_limit: f32,
    pub upper_steering_limit: f32,
    pub target_steering_angle: f32,
    pub max_steering_torque: f32,
    pub steering_hertz: f32,
    pub steering_damping_ratio: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,
    pub spin_mass: f32,
    pub suspension_mass: f32,
    pub steering_mass: f32,
    pub suspension_softness: Softness,
    pub steering_softness: Softness,

    pub enable_spin_motor: bool,
    pub enable_suspension_spring: bool,
    pub enable_suspension_limit: bool,
    pub enable_steering: bool,
    pub enable_steering_limit: bool,
    pub enable_steering_motor: bool,
}

impl Default for WheelJointSim {
    /// Swift's `init() {}` applies every property default; the indices are `B3.nullIndex`, so
    /// the derived all-zeroes default is not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            linear_impulse: Vec2::default(),
            angular_impulse: Vec2::default(),
            spin_impulse: 0.0,
            max_spin_torque: 0.0,
            spin_speed: 0.0,
            suspension_spring_impulse: 0.0,
            lower_suspension_impulse: 0.0,
            upper_suspension_impulse: 0.0,
            lower_suspension_limit: 0.0,
            upper_suspension_limit: 0.0,
            suspension_hertz: 0.0,
            suspension_damping_ratio: 0.0,
            steering_spring_impulse: 0.0,
            lower_steering_impulse: 0.0,
            upper_steering_impulse: 0.0,
            lower_steering_limit: 0.0,
            upper_steering_limit: 0.0,
            target_steering_angle: 0.0,
            max_steering_torque: 0.0,
            steering_hertz: 0.0,
            steering_damping_ratio: 0.0,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            frame_a: Transform::default(),
            frame_b: Transform::default(),
            delta_center: Vec3::default(),
            spin_mass: 0.0,
            suspension_mass: 0.0,
            steering_mass: 0.0,
            suspension_softness: Softness::default(),
            steering_softness: Softness::default(),
            enable_spin_motor: false,
            enable_suspension_spring: false,
            enable_suspension_limit: false,
            enable_steering: false,
            enable_steering_limit: false,
            enable_steering_motor: false,
        }
    }
}

impl JointSim {
    #[inline(always)]
    pub(crate) fn wheel_joint(&self) -> WheelJointSim {
        self.wheel()
    }

    #[inline(always)]
    pub(crate) fn set_wheel_joint(&mut self, value: WheelJointSim) {
        self.set_wheel(value);
    }
}

impl World {
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_wheel_joint(&mut self, def: &WheelJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::WheelJoint);

        let mut sim = pair.sim;
        let mut joint = WheelJointSim::default();
        joint.max_spin_torque = def.max_spin_torque;
        joint.spin_speed = def.spin_speed;
        joint.lower_suspension_limit = def.lower_suspension_limit;
        joint.upper_suspension_limit = def.upper_suspension_limit;
        joint.suspension_hertz = def.suspension_hertz;
        joint.suspension_damping_ratio = def.suspension_damping_ratio;
        joint.lower_steering_limit = def.lower_steering_limit;
        joint.upper_steering_limit = def.upper_steering_limit;
        joint.target_steering_angle = def.target_steering_angle;
        joint.max_steering_torque = def.max_steering_torque;
        joint.steering_hertz = def.steering_hertz;
        joint.steering_damping_ratio = def.steering_damping_ratio;
        joint.enable_spin_motor = def.enable_spin_motor;
        joint.enable_suspension_spring = def.enable_suspension_spring;
        joint.enable_suspension_limit = def.enable_suspension_limit;
        joint.enable_steering = def.enable_steering;
        joint.enable_steering_limit = def.enable_steering_limit;
        sim.set_wheel_joint(joint);
        self.write_joint_sim(pair.joint_id, sim);

        let j = &self.joints[pair.joint_id as usize];
        JointID::new(pair.joint_id + 1, self.world_id, j.generation)
    }

    /// Steering axis shared by prepare, warm start, and solve.
    #[inline(always)]
    fn wheel_steering_axis(&self, matrix_a: Matrix3, matrix_b: Matrix3) -> (Vec3, f32, f32) {
        // Steering param ib = cz_b, ia = cz_a, ja = -cy_a
        let cs = dot(matrix_b.cz, matrix_a.cz);
        let ss = -dot(matrix_b.cz, matrix_a.cy);
        let mut den = cs * cs + ss * ss;
        den = if den > 0.0 { 1.0 / den } else { 0.0 };
        let axis = den * cross(matrix_b.cz, (-cs) * matrix_a.cy - ss * matrix_a.cz);
        (axis, cs, ss)
    }

    pub(crate) fn prepare_wheel_joint(&self, base: &mut JointSim, context: &StepContext) {
        b3_assert(base.type_ == JointType::WheelJoint);

        let body_a = self.joint_prepare_body(base.body_id_a);
        let body_b = self.joint_prepare_body(base.body_id_b);

        b3_assert(
            body_a.set_index == SetType::AwakeSet.raw_value()
                || body_b.set_index == SetType::AwakeSet.raw_value(),
        );

        let sim_a = self.joint_prepare_sim(&body_a);
        let sim_b = self.joint_prepare_sim(&body_b);

        base.inv_mass_a = sim_a.inv_mass;
        base.inv_mass_b = sim_b.inv_mass;
        base.inv_ia = sim_a.inv_inertia_world;
        base.inv_ib = sim_b.inv_inertia_world;

        let inv_inertia_sum = add_mm(base.inv_ia, base.inv_ib);
        base.fixed_rotation = det(inv_inertia_sum) < 1000.0 * f32::MIN_POSITIVE;

        let mut joint = base.wheel_joint();
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

        // Compute joint anchor frames with world space rotation, relative to center of mass
        joint.frame_a.q = mul_quat(sim_a.transform.q, base.local_frame_a.q);
        joint.frame_a.p = rotate_vector(sim_a.transform.q, base.local_frame_a.p - sim_a.local_center);
        joint.frame_b.q = mul_quat(sim_b.transform.q, base.local_frame_b.q);
        joint.frame_b.p = rotate_vector(sim_b.transform.q, base.local_frame_b.p - sim_b.local_center);

        // Compute the initial center delta. Incremental position updates are relative to this.
        joint.delta_center = sub_pos(sim_b.center, sim_a.center);

        let r_a = joint.frame_a.p;
        let r_b = joint.frame_b.p;

        let matrix_a = make_matrix_from_quat(joint.frame_a.q);
        let matrix_b = make_matrix_from_quat(joint.frame_b.q);

        {
            let suspension_axis = matrix_a.cx;
            let r_an = cross(r_a, suspension_axis);
            let r_bn = cross(r_b, suspension_axis);

            let k = base.inv_mass_a + base.inv_mass_b + dot(r_an, mul_mv(base.inv_ia, r_an))
                + dot(r_bn, mul_mv(base.inv_ib, r_bn));
            joint.suspension_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        }

        joint.suspension_softness = make_soft(
            joint.suspension_hertz,
            joint.suspension_damping_ratio,
            context.h,
        );
        joint.steering_softness =
            make_soft(joint.steering_hertz, joint.steering_damping_ratio, context.h);

        {
            // Rotation axis is the z-axis of body A.
            let spin_axis = matrix_b.cz;
            let k = dot(spin_axis, mul_mv(inv_inertia_sum, spin_axis));
            joint.spin_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        }

        {
            // Twist constraint around x-axis
            let (axis, _, _) = self.wheel_steering_axis(matrix_a, matrix_b);
            let k = dot(axis, mul_mv(inv_inertia_sum, axis));
            joint.steering_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        }

        if !context.enable_warm_starting {
            joint.linear_impulse = Vec2::default();
            joint.angular_impulse = Vec2::default();
            joint.spin_impulse = 0.0;
            joint.suspension_spring_impulse = 0.0;
            joint.lower_suspension_impulse = 0.0;
            joint.upper_suspension_impulse = 0.0;
            joint.steering_spring_impulse = 0.0;
            joint.lower_steering_impulse = 0.0;
            joint.upper_steering_impulse = 0.0;
        }

        base.set_wheel_joint(joint);
    }

    pub(crate) fn warm_start_wheel_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::WheelJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let joint = base.wheel_joint();
        // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
        // awake set's state array, installed by the solver stage for the whole warm-start phase.
        // It outlives this call and the indices were encoded during prepare, exactly as in Swift;
        // no aliasing exists between this buffer and `base`.
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);

        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        let d = ((state_b.delta_position - state_a.delta_position) + joint.delta_center)
            + (r_b - r_a);

        let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
        let mut quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);
        if dot_quat(quat_a, quat_b) < 0.0 {
            quat_b = negate_quat(quat_b);
        }

        let matrix_a = make_matrix_from_quat(quat_a);
        let matrix_b = make_matrix_from_quat(quat_b);

        let s_ax = cross(d + r_a, matrix_a.cx);
        let s_bx = cross(r_b, matrix_a.cx);
        let s_ay = cross(d + r_a, matrix_a.cy);
        let s_by = cross(r_b, matrix_a.cy);
        let s_az = cross(d + r_a, matrix_a.cz);
        let s_bz = cross(r_b, matrix_a.cz);

        let suspension_impulse = joint.suspension_spring_impulse + joint.lower_suspension_impulse
            - joint.upper_suspension_impulse;

        let linear_impulse_y = joint.linear_impulse.x;
        let linear_impulse_z = joint.linear_impulse.y;
        let angular_impulse_x = joint.angular_impulse.x;
        let angular_impulse_y = joint.angular_impulse.y;

        let linear_impulse = blend3(
            suspension_impulse,
            matrix_a.cx,
            linear_impulse_y,
            matrix_a.cy,
            linear_impulse_z,
            matrix_a.cz,
        );
        let angular_impulse_a =
            blend3(suspension_impulse, s_ax, linear_impulse_y, s_ay, linear_impulse_z, s_az);
        let angular_impulse_b =
            blend3(suspension_impulse, s_bx, linear_impulse_y, s_by, linear_impulse_z, s_bz);
        let mut angular_impulse = joint.spin_impulse * matrix_a.cz;

        let spin_axis = matrix_b.cz;

        if joint.enable_steering {
            // Twist constraint around x-axis
            let (steering_axis, _, _) = self.wheel_steering_axis(matrix_a, matrix_b);

            let perp_axis = cross(spin_axis, matrix_a.cx);
            let steering_impulse = joint.steering_spring_impulse + joint.lower_steering_impulse
                - joint.upper_steering_impulse;
            angular_impulse = blend3(
                angular_impulse_x,
                perp_axis,
                joint.spin_impulse,
                spin_axis,
                steering_impulse,
                steering_axis,
            );
        } else {
            let rel_q = inv_mul_quat(quat_a, quat_b);
            let perp_axis_x =
                0.5 * rotate_vector(quat_a, rel_q.s * Vec3::AXIS_X + cross(rel_q.v, Vec3::AXIS_X));
            let perp_axis_y =
                0.5 * rotate_vector(quat_a, rel_q.s * Vec3::AXIS_Y + cross(rel_q.v, Vec3::AXIS_Y));
            angular_impulse = angular_impulse
                + blend3(
                    angular_impulse_x,
                    perp_axis_x,
                    angular_impulse_y,
                    perp_axis_y,
                    joint.spin_impulse,
                    spin_axis,
                );
        }

        if (state_a.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_a.linear_velocity = state_a.linear_velocity - mA * linear_impulse;
            state_a.angular_velocity =
                state_a.angular_velocity - mul_mv(iA, angular_impulse_a + angular_impulse);
        }

        if (state_b.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_b.linear_velocity = state_b.linear_velocity + mB * linear_impulse;
            state_b.angular_velocity =
                state_b.angular_velocity + mul_mv(iB, angular_impulse_b + angular_impulse);
        }

        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
    }

    pub(crate) fn solve_wheel_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        use_bias: bool,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::WheelJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let mut joint = base.wheel_joint();
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

        let fixed_rotation = base.fixed_rotation;

        // current anchors
        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
        let mut quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

        if dot_quat(quat_a, quat_b) < 0.0 {
            quat_b = negate_quat(quat_b);
        }

        let rel_q = inv_mul_quat(quat_a, quat_b);
        let matrix_a = make_matrix_from_quat(quat_a);
        let matrix_b = make_matrix_from_quat(quat_b);

        let d = ((state_b.delta_position - state_a.delta_position) + joint.delta_center)
            + (r_b - r_a);
        let s_ax = cross(d + r_a, matrix_a.cx);
        let s_bx = cross(r_b, matrix_a.cx);
        let s_ay = cross(d + r_a, matrix_a.cy);
        let s_by = cross(r_b, matrix_a.cy);
        let s_az = cross(d + r_a, matrix_a.cz);
        let s_bz = cross(r_b, matrix_a.cz);

        let translation = dot(matrix_a.cx, d);

        // Steering param ib = cz_b, ia = cz_a, ja = -cy_a
        let (steering_axis, cs, ss) = self.wheel_steering_axis(matrix_a, matrix_b);

        // motor constraint
        if joint.enable_spin_motor && !fixed_rotation {
            let spin_axis = matrix_b.cz;
            let cdot = dot(w_b - w_a, spin_axis) - joint.spin_speed;
            let mut impulse = -joint.spin_mass * cdot;
            let old_impulse = joint.spin_impulse;
            let max_impulse = context.h * joint.max_spin_torque;
            joint.spin_impulse =
                clamp_float(joint.spin_impulse + impulse, -max_impulse, max_impulse);
            impulse = joint.spin_impulse - old_impulse;

            w_a = w_a - mul_mv(iA, impulse * spin_axis);
            w_b = w_b + mul_mv(iB, impulse * spin_axis);
        }

        // suspension
        if joint.enable_suspension_spring {
            // This is a real spring and should be applied even during relax
            let c = translation;
            let bias = joint.suspension_softness.bias_rate * c;
            let mass_scale = joint.suspension_softness.mass_scale;
            let impulse_scale = joint.suspension_softness.impulse_scale;

            let cdot = dot(matrix_a.cx, v_b - v_a) + dot(s_bx, w_b) - dot(s_ax, w_a);
            let impulse = -mass_scale * joint.suspension_mass * (cdot + bias)
                - impulse_scale * joint.suspension_spring_impulse;
            joint.suspension_spring_impulse += impulse;

            let linear_impulse = impulse * matrix_a.cx;
            let angular_impulse_a = impulse * s_ax;
            let angular_impulse_b = impulse * s_bx;

            v_a = v_a - mA * linear_impulse;
            w_a = w_a - mul_mv(iA, angular_impulse_a);
            v_b = v_b + mB * linear_impulse;
            w_b = w_b + mul_mv(iB, angular_impulse_b);
        }

        // steering
        if joint.enable_steering && !fixed_rotation {
            let steering_angle = atan2_approx(ss, cs);

            {
                // This is a real spring and should be applied even during relax
                let c = steering_angle - joint.target_steering_angle;
                let bias = joint.steering_softness.bias_rate * c;
                let mass_scale = joint.steering_softness.mass_scale;
                let impulse_scale = joint.steering_softness.impulse_scale;

                let cdot = dot(steering_axis, w_b - w_a);
                let old_impulse = joint.steering_spring_impulse;
                let mut impulse =
                    -mass_scale * joint.steering_mass * (cdot + bias) - impulse_scale * old_impulse;
                let max_impulse = context.h * joint.max_steering_torque;
                joint.steering_spring_impulse =
                    clamp_float(old_impulse + impulse, -max_impulse, max_impulse);
                impulse = joint.steering_spring_impulse - old_impulse;

                w_a = w_a - mul_mv(iA, impulse * steering_axis);
                w_b = w_b + mul_mv(iB, impulse * steering_axis);
            }

            if joint.enable_steering_limit {
                // Lower limit
                {
                    let c = steering_angle - joint.lower_steering_limit;
                    let mut bias: f32 = 0.0;
                    let mut mass_scale: f32 = 1.0;
                    let mut impulse_scale: f32 = 0.0;

                    if c > 0.0 {
                        // speculation
                        bias = c * context.inv_h;
                    } else if use_bias {
                        bias = base.constraint_softness.bias_rate * c;
                        mass_scale = base.constraint_softness.mass_scale;
                        impulse_scale = base.constraint_softness.impulse_scale;
                    }

                    let cdot = dot(steering_axis, w_b - w_a);
                    let old_impulse = joint.lower_steering_impulse;
                    let mut impulse = -mass_scale * joint.steering_mass * (cdot + bias)
                        - impulse_scale * old_impulse;
                    joint.lower_steering_impulse = max_float(old_impulse + impulse, 0.0);
                    impulse = joint.lower_steering_impulse - old_impulse;

                    w_a = w_a - mul_mv(iA, impulse * steering_axis);
                    w_b = w_b + mul_mv(iB, impulse * steering_axis);
                }

                // Upper limit
                // Note: signs are flipped to keep c positive when the constraint is satisfied.
                {
                    // sign flipped
                    let c = joint.upper_steering_limit - steering_angle;
                    let mut bias: f32 = 0.0;
                    let mut mass_scale: f32 = 1.0;
                    let mut impulse_scale: f32 = 0.0;

                    if c > 0.0 {
                        // speculation
                        bias = c * context.inv_h;
                    } else if use_bias {
                        bias = base.constraint_softness.bias_rate * c;
                        mass_scale = base.constraint_softness.mass_scale;
                        impulse_scale = base.constraint_softness.impulse_scale;
                    }

                    // sign flipped on cdot
                    let cdot = dot(steering_axis, w_a - w_b);
                    let old_impulse = joint.upper_steering_impulse;
                    let mut impulse = -mass_scale * joint.steering_mass * (cdot + bias)
                        - impulse_scale * old_impulse;
                    joint.upper_steering_impulse = max_float(old_impulse + impulse, 0.0);
                    impulse = joint.upper_steering_impulse - old_impulse;

                    // sign flipped on applied impulse
                    w_a = w_a + mul_mv(iA, impulse * steering_axis);
                    w_b = w_b - mul_mv(iB, impulse * steering_axis);
                }
            }
        }

        if joint.enable_suspension_limit {
            // Lower limit
            {
                let c = translation - joint.lower_suspension_limit;
                let mut bias: f32 = 0.0;
                let mut mass_scale: f32 = 1.0;
                let mut impulse_scale: f32 = 0.0;

                if c > 0.0 {
                    // speculation
                    bias = c * context.inv_h;
                } else if use_bias {
                    bias = base.constraint_softness.bias_rate * c;
                    mass_scale = base.constraint_softness.mass_scale;
                    impulse_scale = base.constraint_softness.impulse_scale;
                }

                let cdot = dot(matrix_a.cx, v_b - v_a) + dot(s_bx, w_b) - dot(s_ax, w_a);
                let mut impulse = -mass_scale * joint.suspension_mass * (cdot + bias)
                    - impulse_scale * joint.lower_suspension_impulse;
                let old_impulse = joint.lower_suspension_impulse;
                joint.lower_suspension_impulse = max_float(old_impulse + impulse, 0.0);
                impulse = joint.lower_suspension_impulse - old_impulse;

                let linear_impulse = impulse * matrix_a.cx;
                let angular_impulse_a = impulse * s_ax;
                let angular_impulse_b = impulse * s_bx;

                v_a = v_a - mA * linear_impulse;
                w_a = w_a - mul_mv(iA, angular_impulse_a);
                v_b = v_b + mB * linear_impulse;
                w_b = w_b + mul_mv(iB, angular_impulse_b);
            }

            // Upper limit
            // Note: signs are flipped to keep c positive when the constraint is satisfied.
            {
                // sign flipped
                let c = joint.upper_suspension_limit - translation;
                let mut bias: f32 = 0.0;
                let mut mass_scale: f32 = 1.0;
                let mut impulse_scale: f32 = 0.0;

                if c > 0.0 {
                    // speculation
                    bias = c * context.inv_h;
                } else if use_bias {
                    bias = base.constraint_softness.bias_rate * c;
                    mass_scale = base.constraint_softness.mass_scale;
                    impulse_scale = base.constraint_softness.impulse_scale;
                }

                // sign flipped on cdot
                let cdot = dot(matrix_a.cx, v_a - v_b) + dot(s_ax, w_a) - dot(s_bx, w_b);
                let mut impulse = -mass_scale * joint.suspension_mass * (cdot + bias)
                    - impulse_scale * joint.upper_suspension_impulse;
                let old_impulse = joint.upper_suspension_impulse;
                joint.upper_suspension_impulse = max_float(old_impulse + impulse, 0.0);
                impulse = joint.upper_suspension_impulse - old_impulse;

                let linear_impulse = impulse * matrix_a.cx;
                let angular_impulse_a = impulse * s_ax;
                let angular_impulse_b = impulse * s_bx;

                // sign flipped on applied impulse
                v_a = v_a + mA * linear_impulse;
                w_a = w_a + mul_mv(iA, angular_impulse_a);
                v_b = v_b - mB * linear_impulse;
                w_b = w_b - mul_mv(iB, angular_impulse_b);
            }
        }

        // Collinearity constraint
        if !fixed_rotation {
            if joint.enable_steering {
                let mut bias: f32 = 0.0;
                let mut mass_scale: f32 = 1.0;
                let mut impulse_scale: f32 = 0.0;
                if use_bias {
                    let c = dot(matrix_a.cx, matrix_b.cz);

                    bias = base.constraint_softness.bias_rate * c;
                    mass_scale = base.constraint_softness.mass_scale;
                    impulse_scale = base.constraint_softness.impulse_scale;
                }

                let u = cross(matrix_b.cz, matrix_a.cx);
                let cdot = dot(w_b - w_a, u);

                let inv_inertia_sum = add_mm(iA, iB);
                let k = dot(u, mul_mv(inv_inertia_sum, u));
                let perp_mass: f32 = if k > 0.0 { 1.0 / k } else { 0.0 };

                let delta_impulse =
                    -mass_scale * perp_mass * (cdot + bias) - impulse_scale * joint.angular_impulse.x;
                joint.angular_impulse.x += delta_impulse;

                w_a = w_a - delta_impulse * mul_mv(iA, u);
                w_b = w_b + delta_impulse * mul_mv(iB, u);
            } else {
                let mut bias = Vec2::default();
                let mut mass_scale: f32 = 1.0;
                let mut impulse_scale: f32 = 0.0;

                if use_bias {
                    let c = Vec2::new(rel_q.v.x, rel_q.v.y);
                    bias = Vec2::new(
                        base.constraint_softness.bias_rate * c.x,
                        base.constraint_softness.bias_rate * c.y,
                    );
                    mass_scale = base.constraint_softness.mass_scale;
                    impulse_scale = base.constraint_softness.impulse_scale;
                }

                // Collinearity constraint as 2-by-2
                let perp_axis_x = 0.5
                    * rotate_vector(quat_a, rel_q.s * Vec3::AXIS_X + cross(rel_q.v, Vec3::AXIS_X));
                let perp_axis_y = 0.5
                    * rotate_vector(quat_a, rel_q.s * Vec3::AXIS_Y + cross(rel_q.v, Vec3::AXIS_Y));

                let inv_inertia_sum = add_mm(iA, iB);
                let kxx = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_x));
                let kyy = dot(perp_axis_y, mul_mv(inv_inertia_sum, perp_axis_y));
                let kxy = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_y));

                let k = Matrix2::new(Vec2::new(kxx, kxy), Vec2::new(kxy, kyy));

                let w_rel = w_b - w_a;
                let cdot = Vec2::new(dot(w_rel, perp_axis_x), dot(w_rel, perp_axis_y));
                let old_impulse = joint.angular_impulse;
                let sol = solve2(k, Vec2::new(cdot.x + bias.x, cdot.y + bias.y));
                let delta_impulse = Vec2::new(
                    -mass_scale * sol.x - impulse_scale * old_impulse.x,
                    -mass_scale * sol.y - impulse_scale * old_impulse.y,
                );
                joint.angular_impulse =
                    Vec2::new(old_impulse.x + delta_impulse.x, old_impulse.y + delta_impulse.y);

                let angular_impulse =
                    blend2(delta_impulse.x, perp_axis_x, delta_impulse.y, perp_axis_y);
                w_a = w_a - mul_mv(iA, angular_impulse);
                w_b = w_b + mul_mv(iB, angular_impulse);
            }
        }

        // Solve point-to-line constraint
        {
            let perp_y = matrix_a.cy;
            let perp_z = matrix_a.cz;

            let mut bias = Vec2::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias {
                let c = Vec2::new(dot(perp_y, d), dot(perp_z, d));
                bias = Vec2::new(
                    base.constraint_softness.bias_rate * c.x,
                    base.constraint_softness.bias_rate * c.y,
                );
                mass_scale = base.constraint_softness.mass_scale;
                impulse_scale = base.constraint_softness.impulse_scale;
            }

            let v_rel = (v_b + cross(w_b, r_b)) - v_a - cross(w_a, r_a + d);
            let cdot = Vec2::new(dot(perp_y, v_rel), dot(perp_z, v_rel));

            let kyy =
                mA + mB + dot(s_ay, mul_mv(iA, s_ay)) + dot(s_by, mul_mv(iB, s_by));
            let kyz = dot(s_ay, mul_mv(iA, s_az)) + dot(s_by, mul_mv(iB, s_bz));
            let kzz =
                mA + mB + dot(s_az, mul_mv(iA, s_az)) + dot(s_bz, mul_mv(iB, s_bz));

            let k = Matrix2::new(Vec2::new(kyy, kyz), Vec2::new(kyz, kzz));

            let old_impulse = joint.linear_impulse;
            let sol = solve2(k, Vec2::new(cdot.x + bias.x, cdot.y + bias.y));
            let delta_impulse = Vec2::new(
                -mass_scale * sol.x - impulse_scale * old_impulse.x,
                -mass_scale * sol.y - impulse_scale * old_impulse.y,
            );
            joint.linear_impulse =
                Vec2::new(old_impulse.x + delta_impulse.x, old_impulse.y + delta_impulse.y);

            let linear_impulse = blend2(delta_impulse.x, perp_y, delta_impulse.y, perp_z);

            v_a = v_a - mA * linear_impulse;
            w_a = w_a - mul_mv(iA, blend2(delta_impulse.x, s_ay, delta_impulse.y, s_az));
            v_b = v_b + mB * linear_impulse;
            w_b = w_b + mul_mv(iB, blend2(delta_impulse.x, s_by, delta_impulse.y, s_bz));
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

        base.set_wheel_joint(joint);
    }
}
