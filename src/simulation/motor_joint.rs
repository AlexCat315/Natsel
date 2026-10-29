//! Motor joint. Controls relative velocity and optionally a spring pose.
//!
//! Translated from `Sources/Natsel/Simulation/MotorJoint.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::joint`: `World::create_joint(&mut self, &JointDef, JointType) -> JointPair`
//!   and `World::write_joint_sim(&mut self, i32, JointSim)`.
//! - `crate::simulation::contact_solver`: the `World::state_at` / `World::set_state_at` buffer
//!   accessors, kept safe wrappers over the raw awake-set state pointer, mirroring Swift.
//! - `crate::simulation::solver_prepare`: `World::joint_prepare_body` / `joint_prepare_sim`.
//! - `crate::simulation::revolute_joint`: `solve_point_constraint`.
//! - `crate::simulation::world_runtime`: the `JointSim::motor` / `set_motor` payload accessors.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::internal_math::delta_quat_to_rotation;
use crate::math::functions::{
    add_mm, cross, det, dot_quat, inv_mul_quat, invert_matrix, length_squared, mul_mv, mul_quat,
    negate_quat, normalize, rotate_vector, sub_pos,
};
use crate::math::types::{Matrix3, Quat, Transform, Vec3};
use crate::simulation::revolute_joint::solve_point_constraint;
use crate::simulation::solver_integration::{Softness, StepContext, make_soft};
use crate::simulation::world_runtime::{BodyFlags, BodyState, JointSim, SetType, World};
use crate::types::ids::JointID;
use crate::types::joint_defs::{JointType, MotorJointDef};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct MotorJointSim {
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub max_velocity_force: f32,
    pub max_velocity_torque: f32,
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub max_spring_force: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,
    pub max_spring_torque: f32,

    pub linear_velocity_impulse: Vec3,
    pub angular_velocity_impulse: Vec3,
    pub linear_spring_impulse: Vec3,
    pub angular_spring_impulse: Vec3,

    pub linear_spring: Softness,
    pub angular_spring: Softness,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,
    pub angular_mass: Matrix3,
}

impl Default for MotorJointSim {
    /// Swift's `init() {}` applies every property default; the indices are `B3.nullIndex`, so
    /// the derived all-zeroes default is not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            linear_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            max_velocity_force: 0.0,
            max_velocity_torque: 0.0,
            linear_hertz: 0.0,
            linear_damping_ratio: 0.0,
            max_spring_force: 0.0,
            angular_hertz: 0.0,
            angular_damping_ratio: 0.0,
            max_spring_torque: 0.0,
            linear_velocity_impulse: Vec3::default(),
            angular_velocity_impulse: Vec3::default(),
            linear_spring_impulse: Vec3::default(),
            angular_spring_impulse: Vec3::default(),
            linear_spring: Softness::default(),
            angular_spring: Softness::default(),
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            frame_a: Transform::default(),
            frame_b: Transform::default(),
            delta_center: Vec3::default(),
            angular_mass: Matrix3::default(),
        }
    }
}

impl JointSim {
    #[inline(always)]
    pub(crate) fn motor_joint(&self) -> MotorJointSim {
        self.motor()
    }

    #[inline(always)]
    pub(crate) fn set_motor_joint(&mut self, value: MotorJointSim) {
        self.set_motor(value);
    }
}

impl World {
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_motor_joint(&mut self, def: &MotorJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::MotorJoint);

        let mut sim = pair.sim;
        let mut joint = MotorJointSim::default();
        joint.linear_velocity = def.linear_velocity;
        joint.angular_velocity = def.angular_velocity;
        joint.max_velocity_force = def.max_velocity_force;
        joint.max_velocity_torque = def.max_velocity_torque;
        joint.linear_hertz = def.linear_hertz;
        joint.linear_damping_ratio = def.linear_damping_ratio;
        joint.max_spring_force = def.max_spring_force;
        joint.angular_hertz = def.angular_hertz;
        joint.angular_damping_ratio = def.angular_damping_ratio;
        joint.max_spring_torque = def.max_spring_torque;
        sim.set_motor_joint(joint);
        self.write_joint_sim(pair.joint_id, sim);

        let j = &self.joints[pair.joint_id as usize];
        JointID::new(pair.joint_id + 1, self.world_id, j.generation)
    }

    pub(crate) fn prepare_motor_joint(&self, base: &mut JointSim, context: &StepContext) {
        b3_assert(base.type_ == JointType::MotorJoint);

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

        let mut joint = base.motor_joint();
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

        joint.linear_spring = make_soft(joint.linear_hertz, joint.linear_damping_ratio, context.h);
        joint.angular_spring = make_soft(joint.angular_hertz, joint.angular_damping_ratio, context.h);

        joint.angular_mass = invert_matrix(inv_inertia_sum);

        if context.enable_warm_starting == false {
            joint.linear_velocity_impulse = Vec3::default();
            joint.angular_velocity_impulse = Vec3::default();
            joint.linear_spring_impulse = Vec3::default();
            joint.angular_spring_impulse = Vec3::default();
        }

        base.set_motor_joint(joint);
    }

    pub(crate) fn warm_start_motor_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::MotorJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let i_a = base.inv_ia;
        let i_b = base.inv_ib;

        let joint = base.motor_joint();
        // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
        // awake set's state array, installed by the solver stage for the whole warm-start phase.
        // It outlives this call and the indices were encoded during prepare, exactly as in Swift;
        // no aliasing exists between this buffer and `base`.
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);

        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        let linear_impulse = joint.linear_velocity_impulse + joint.linear_spring_impulse;
        let angular_impulse = joint.angular_velocity_impulse + joint.angular_spring_impulse;

        // Warm start writes the velocities unconditionally, so kinematic bodies are modified.
        state_a.linear_velocity = state_a.linear_velocity - mA * linear_impulse;
        state_a.angular_velocity = state_a.angular_velocity
            - mul_mv(i_a, cross(r_a, linear_impulse) + angular_impulse);
        state_b.linear_velocity = state_b.linear_velocity + mB * linear_impulse;
        state_b.angular_velocity = state_b.angular_velocity
            + mul_mv(i_b, cross(r_b, linear_impulse) + angular_impulse);

        if joint.index_a >= 0 {
            // SAFETY: `states` is the awake set's state buffer for the whole phase and
            // `joint.index_a` is an index encoded during prepare into it; Swift wrote through the
            // buffer pointer subscript, bypassing the dynamic-flag filter of `setStateAt`, and the
            // graph color keeps this body slot exclusive to this joint.
            unsafe { states.add(joint.index_a as usize).write(state_a) };
        }
        if joint.index_b >= 0 {
            // SAFETY: as for body A above.
            unsafe { states.add(joint.index_b as usize).write(state_b) };
        }
    }

    pub(crate) fn solve_motor_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::MotorJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let i_a = base.inv_ia;
        let i_b = base.inv_ib;

        let mut joint = base.motor_joint();
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

        let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
        let mut quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

        if dot_quat(quat_a, quat_b) < 0.0 {
            // this keeps the rotation angle in the range [-pi, pi]
            quat_b = negate_quat(quat_b);
        }

        let rel_q = inv_mul_quat(quat_a, quat_b);

        // angular spring
        if joint.max_spring_torque > 0.0 && joint.angular_hertz > 0.0 {
            let target_quat = Quat::IDENTITY;
            let delta_rotation = delta_quat_to_rotation(rel_q, target_quat);
            let c = -(rotate_vector(quat_a, delta_rotation));

            let bias = joint.angular_spring.bias_rate * c;
            let mass_scale = joint.angular_spring.mass_scale;
            let impulse_scale = joint.angular_spring.impulse_scale;

            let cdot = w_b - w_a;

            let max_impulse = context.h * joint.max_spring_torque;
            let old_impulse = joint.angular_spring_impulse;
            let mut impulse = (-mass_scale) * mul_mv(joint.angular_mass, cdot + bias)
                - impulse_scale * old_impulse;
            joint.angular_spring_impulse = old_impulse + impulse;
            if length_squared(joint.angular_spring_impulse) > max_impulse * max_impulse {
                joint.angular_spring_impulse =
                    max_impulse * normalize(joint.angular_spring_impulse);
            }
            impulse = joint.angular_spring_impulse - old_impulse;

            w_a = w_a - mul_mv(i_a, impulse);
            w_b = w_b + mul_mv(i_b, impulse);
        }

        // angular velocity
        if joint.max_velocity_torque > 0.0 {
            let cdot = (w_b - w_a) - joint.angular_velocity;
            let mut impulse = -(mul_mv(joint.angular_mass, cdot));

            let max_impulse = context.h * joint.max_velocity_torque;
            let old_impulse = joint.angular_velocity_impulse;
            joint.angular_velocity_impulse = old_impulse + impulse;
            if length_squared(joint.angular_velocity_impulse) > max_impulse * max_impulse {
                joint.angular_velocity_impulse =
                    max_impulse * normalize(joint.angular_velocity_impulse);
            }
            impulse = joint.angular_velocity_impulse - old_impulse;

            w_a = w_a - mul_mv(i_a, impulse);
            w_b = w_b + mul_mv(i_b, impulse);
        }

        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        // linear spring
        if joint.max_spring_force > 0.0 && joint.linear_hertz > 0.0 {
            let d_ca = state_a.delta_position;
            let d_cb = state_b.delta_position;
            let c = ((d_cb - d_ca) + (r_b - r_a)) + joint.delta_center;

            let bias = joint.linear_spring.bias_rate * c;
            let mass_scale = joint.linear_spring.mass_scale;
            let impulse_scale = joint.linear_spring.impulse_scale;

            let cdot = (v_b + cross(w_b, r_b)) - (v_a + cross(w_a, r_a));

            let b = solve_point_constraint(
                mA,
                mB,
                base.inv_ia,
                base.inv_ib,
                r_a,
                r_b,
                cdot + bias,
            );

            let old_impulse = joint.linear_spring_impulse;
            let mut impulse = (-mass_scale) * b - impulse_scale * old_impulse;
            let max_impulse = context.h * joint.max_spring_force;
            joint.linear_spring_impulse = joint.linear_spring_impulse + impulse;

            if length_squared(joint.linear_spring_impulse) > max_impulse * max_impulse {
                joint.linear_spring_impulse = max_impulse * normalize(joint.linear_spring_impulse);
            }

            impulse = joint.linear_spring_impulse - old_impulse;

            v_a = v_a - mA * impulse;
            w_a = w_a - mul_mv(i_a, cross(r_a, impulse));
            v_b = v_b + mB * impulse;
            w_b = w_b + mul_mv(i_b, cross(r_b, impulse));
        }

        // linear velocity
        if joint.max_velocity_force > 0.0 {
            let mut cdot = (v_b + cross(w_b, r_b)) - (v_a + cross(w_a, r_a));
            cdot = cdot - joint.linear_velocity;

            let b = solve_point_constraint(mA, mB, base.inv_ia, base.inv_ib, r_a, r_b, cdot);
            let mut impulse = -(b);

            let old_impulse = joint.linear_velocity_impulse;
            let max_impulse = context.h * joint.max_velocity_force;
            joint.linear_velocity_impulse = joint.linear_velocity_impulse + impulse;

            if length_squared(joint.linear_velocity_impulse) > max_impulse * max_impulse {
                joint.linear_velocity_impulse =
                    max_impulse * normalize(joint.linear_velocity_impulse);
            }

            impulse = joint.linear_velocity_impulse - old_impulse;

            v_a = v_a - mA * impulse;
            w_a = w_a - mul_mv(i_a, cross(r_a, impulse));
            v_b = v_b + mB * impulse;
            w_b = w_b + mul_mv(i_b, cross(r_b, impulse));
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

        base.set_motor_joint(joint);
    }
}
