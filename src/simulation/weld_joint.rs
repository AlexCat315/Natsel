//! Weld joint.
//!
//! Translated from `Sources/Natsel/Simulation/WeldJoint.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (see MIGRATION.md "Shared simulation
//! vocabulary"):
//! - `crate::simulation::joint`: `World::create_joint(&mut self, &JointDef, JointType) -> JointPair`
//!   and `World::write_joint_sim(&mut self, i32, JointSim)`.
//! - `crate::simulation::contact_solver`: the `World::state_at` / `World::set_state_at` buffer
//!   accessors, kept safe wrappers over the raw awake-set state pointer, mirroring Swift.
//! - `crate::simulation::solver_prepare`: `World::joint_prepare_body` / `joint_prepare_sim`.
//! - `crate::simulation::revolute_joint`: `solve_point_constraint` (shared 3-by-3 point solver).

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::internal_math::delta_quat_to_rotation;
use crate::math::functions::{
    add_mm, cross, det, dot_quat, inv_mul_quat, invert_matrix, mul_mv, mul_quat, negate_quat,
    rotate_vector, sub_pos,
};
use crate::math::types::{Matrix3, Quat, Transform, Vec3};
use crate::simulation::revolute_joint::solve_point_constraint;
use crate::simulation::solver_integration::{Softness, StepContext, make_soft};
use crate::simulation::world_runtime::{BodyFlags, BodyState, JointSim, SetType, World};
use crate::types::ids::JointID;
use crate::types::joint_defs::{JointType, WeldJointDef};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct WeldJointSim {
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,

    pub linear_spring: Softness,
    pub angular_spring: Softness,
    pub linear_impulse: Vec3,
    pub angular_impulse: Vec3,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,

    pub angular_mass: Matrix3,
}

impl Default for WeldJointSim {
    /// Swift's `init() {}` applies every property default; the indices are `B3.nullIndex`, so
    /// the derived all-zeroes default is not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            linear_hertz: 0.0,
            linear_damping_ratio: 0.0,
            angular_hertz: 0.0,
            angular_damping_ratio: 0.0,
            linear_spring: Softness::default(),
            angular_spring: Softness::default(),
            linear_impulse: Vec3::default(),
            angular_impulse: Vec3::default(),
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
    pub(crate) fn weld_joint(&self) -> WeldJointSim {
        self.weld()
    }

    #[inline(always)]
    pub(crate) fn set_weld_joint(&mut self, value: WeldJointSim) {
        self.set_weld(value);
    }
}

impl World {
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_weld_joint(&mut self, def: &WeldJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::WeldJoint);

        let mut sim = pair.sim;
        let mut joint = WeldJointSim::default();
        joint.linear_hertz = def.linear_hertz;
        joint.linear_damping_ratio = def.linear_damping_ratio;
        joint.angular_hertz = def.angular_hertz;
        joint.angular_damping_ratio = def.angular_damping_ratio;
        sim.set_weld_joint(joint);
        self.write_joint_sim(pair.joint_id, sim);

        let j = &self.joints[pair.joint_id as usize];
        JointID::new(pair.joint_id + 1, self.world_id, j.generation)
    }

    pub(crate) fn prepare_weld_joint(&self, base: &mut JointSim, context: &StepContext) {
        b3_assert(base.type_ == JointType::WeldJoint);

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

        let mut joint = base.weld_joint();
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

        joint.delta_center = sub_pos(sim_b.center, sim_a.center);
        joint.angular_mass = invert_matrix(inv_inertia_sum);

        if joint.linear_hertz == 0.0 {
            joint.linear_spring = base.constraint_softness;
        } else {
            joint.linear_spring = make_soft(joint.linear_hertz, joint.linear_damping_ratio, context.h);
        }

        if joint.angular_hertz == 0.0 {
            joint.angular_spring = base.constraint_softness;
        } else {
            joint.angular_spring =
                make_soft(joint.angular_hertz, joint.angular_damping_ratio, context.h);
        }

        if !context.enable_warm_starting {
            joint.linear_impulse = Vec3::default();
            joint.angular_impulse = Vec3::default();
        }

        base.set_weld_joint(joint);
    }

    pub(crate) fn warm_start_weld_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::WeldJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let joint = base.weld_joint();
        // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
        // awake set's state array, installed by the solver stage for the whole warm-start phase.
        // It outlives this call and the indices were encoded during prepare, exactly as in Swift;
        // no aliasing exists between this buffer and `base`.
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);

        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;

        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        v_a = v_a - mA * joint.linear_impulse;
        w_a = w_a - mul_mv(iA, cross(r_a, joint.linear_impulse) + joint.angular_impulse);

        v_b = v_b + mB * joint.linear_impulse;
        w_b = w_b + mul_mv(iB, cross(r_b, joint.linear_impulse) + joint.angular_impulse);

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
    }

    pub(crate) fn solve_weld_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        use_bias: bool,
        states: *mut BodyState,
    ) {
        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let mut joint = base.weld_joint();
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
        let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
        let mut quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

        if dot_quat(quat_a, quat_b) < 0.0 {
            // this keeps the rotation angle in the range [-pi, pi]
            quat_b = negate_quat(quat_b);
        }

        let rel_q = inv_mul_quat(quat_a, quat_b);

        // angular constraint
        if !fixed_rotation {
            let mut bias = Vec3::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias || joint.angular_hertz > 0.0 {
                let target_quat = Quat::IDENTITY;
                let delta_rotation = delta_quat_to_rotation(rel_q, target_quat);
                let c = -(rotate_vector(quat_a, delta_rotation));

                bias = joint.angular_spring.bias_rate * c;
                mass_scale = joint.angular_spring.mass_scale;
                impulse_scale = joint.angular_spring.impulse_scale;
            }

            let cdot = w_b - w_a;
            let impulse = (-mass_scale) * mul_mv(joint.angular_mass, cdot + bias)
                - impulse_scale * joint.angular_impulse;
            joint.angular_impulse = joint.angular_impulse + impulse;

            w_a = w_a - mul_mv(iA, impulse);
            w_b = w_b + mul_mv(iB, impulse);
        }

        // linear constraint
        {
            let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
            let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

            let cdot = (v_b + cross(w_b, r_b)) - (v_a + cross(w_a, r_a));

            let mut bias = Vec3::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias || joint.linear_hertz > 0.0 {
                let dc_a = state_a.delta_position;
                let dc_b = state_b.delta_position;

                let separation = ((dc_b - dc_a) + (r_b - r_a)) + joint.delta_center;

                bias = joint.linear_spring.bias_rate * separation;
                mass_scale = joint.linear_spring.mass_scale;
                impulse_scale = joint.linear_spring.impulse_scale;
            }

            let b = solve_point_constraint(
                mA, mB, base.inv_ia, base.inv_ib, r_a, r_b, cdot + bias,
            );

            let impulse = (-mass_scale) * b - impulse_scale * joint.linear_impulse;
            joint.linear_impulse = joint.linear_impulse + impulse;

            v_a = v_a - mA * impulse;
            w_a = w_a - mul_mv(iA, cross(r_a, impulse));
            v_b = v_b + mB * impulse;
            w_b = w_b + mul_mv(iB, cross(r_b, impulse));
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

        base.set_weld_joint(joint);
    }
}
