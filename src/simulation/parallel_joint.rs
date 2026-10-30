//! Parallel joint. Keeps the bodies aligned along their z-axes.
//!
//! Translated from `Sources/Natsel/Simulation/ParallelJoint.swift`.
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
use crate::collision::internal_math::{Matrix2, solve2};
use crate::math::functions::{
    add_mm, blend2, cross, det, dot, dot_quat, inv_mul_quat, mul_mv, mul_quat, negate_quat,
    rotate_vector,
};
use crate::math::types::{Quat, Vec2, Vec3};
use crate::simulation::solver_integration::{Softness, StepContext, make_soft};
use crate::simulation::world_runtime::{BodyFlags, BodyState, JointSim, SetType, World};
use crate::types::ids::JointID;
use crate::types::joint_defs::{JointType, ParallelJointDef};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct ParallelJointSim {
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_torque: f32,

    pub perp_impulse: Vec2,
    pub perp_axis_x: Vec3,
    pub perp_axis_y: Vec3,

    pub quat_a: Quat,
    pub quat_b: Quat,
    pub index_a: i32,
    pub index_b: i32,
    pub softness: Softness,
}

impl Default for ParallelJointSim {
    /// Swift's `init() {}` applies every property default; the indices are `B3.nullIndex`, so
    /// the derived all-zeroes default is not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            hertz: 0.0,
            damping_ratio: 0.0,
            max_torque: 0.0,
            perp_impulse: Vec2::default(),
            perp_axis_x: Vec3::default(),
            perp_axis_y: Vec3::default(),
            quat_a: Quat::default(),
            quat_b: Quat::default(),
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            softness: Softness::default(),
        }
    }
}

impl JointSim {
    #[inline(always)]
    pub(crate) fn parallel_joint(&self) -> ParallelJointSim {
        self.parallel()
    }

    #[inline(always)]
    pub(crate) fn set_parallel_joint(&mut self, value: ParallelJointSim) {
        self.set_parallel(value);
    }
}

impl World {
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_parallel_joint(&mut self, def: &ParallelJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::ParallelJoint);

        let mut sim = pair.sim;
        let mut joint = ParallelJointSim::default();
        joint.hertz = def.hertz;
        joint.damping_ratio = def.damping_ratio;
        joint.max_torque = def.max_torque;
        sim.set_parallel_joint(joint);
        self.write_joint_sim(pair.joint_id, sim);

        let j = &self.joints[pair.joint_id as usize];
        JointID::new(pair.joint_id + 1, self.world_id, j.generation)
    }

    pub(crate) fn prepare_parallel_joint(&self, base: &mut JointSim, context: &StepContext) {
        b3_assert(base.type_ == JointType::ParallelJoint);

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

        let mut joint = base.parallel_joint();
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
        joint.quat_a = mul_quat(sim_a.transform.q, base.local_frame_a.q);
        joint.quat_b = mul_quat(sim_b.transform.q, base.local_frame_b.q);

        let rel_q = inv_mul_quat(joint.quat_a, joint.quat_b);

        {
            // These are needed for warm starting
            joint.perp_axis_x =
                0.5 * rotate_vector(joint.quat_a, rel_q.s * Vec3::AXIS_X + cross(rel_q.v, Vec3::AXIS_X));
            joint.perp_axis_y =
                0.5 * rotate_vector(joint.quat_a, rel_q.s * Vec3::AXIS_Y + cross(rel_q.v, Vec3::AXIS_Y));
        }

        joint.softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

        if !context.enable_warm_starting {
            joint.perp_impulse = Vec2::default();
        }

        base.set_parallel_joint(joint);
    }

    pub(crate) fn warm_start_parallel_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::ParallelJoint);

        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let joint = base.parallel_joint();
        // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
        // awake set's state array, installed by the solver stage for the whole warm-start phase.
        // It outlives this call and the indices were encoded during prepare, exactly as in Swift;
        // no aliasing exists between this buffer and `base`.
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);

        let mut w_a = state_a.angular_velocity;
        let mut w_b = state_b.angular_velocity;

        let angular_impulse = blend2(
            joint.perp_impulse.x,
            joint.perp_axis_x,
            joint.perp_impulse.y,
            joint.perp_axis_y,
        );

        w_a = w_a - mul_mv(iA, angular_impulse);
        w_b = w_b + mul_mv(iB, angular_impulse);

        if (state_a.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_a.angular_velocity = w_a;
        }

        if (state_b.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_b.angular_velocity = w_b;
        }

        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
    }

    pub(crate) fn solve_parallel_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        states: *mut BodyState,
    ) {
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let mut joint = base.parallel_joint();
        // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
        // awake set's state array, installed by the solver stage for the whole solve phase. The
        // sim in `base` and this buffer do not alias; Swift read and wrote whole state values via
        // `stateAt`/`setStateAt` and the same in-place copy-back happens here.
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);

        let mut w_a = state_a.angular_velocity;
        let mut w_b = state_b.angular_velocity;

        let fixed_rotation = base.fixed_rotation;
        let quat_a = mul_quat(state_a.delta_rotation, joint.quat_a);
        let mut quat_b = mul_quat(state_b.delta_rotation, joint.quat_b);

        if dot_quat(quat_a, quat_b) < 0.0 {
            // this keeps the rotation angle in the range [-pi, pi]
            quat_b = negate_quat(quat_b);
        }

        let rel_q = inv_mul_quat(quat_a, quat_b);

        if !fixed_rotation && joint.max_torque > 0.0 {
            let c = Vec2::new(rel_q.v.x, rel_q.v.y);
            let bias = Vec2::new(joint.softness.bias_rate * c.x, joint.softness.bias_rate * c.y);
            let mass_scale = joint.softness.mass_scale;
            let impulse_scale = joint.softness.impulse_scale;

            // Collinearity constraint as 2-by-2
            let perp_axis_x = 0.5
                * rotate_vector(quat_a, rel_q.s * Vec3::AXIS_X + cross(rel_q.v, Vec3::AXIS_X));
            let perp_axis_y = 0.5
                * rotate_vector(quat_a, rel_q.s * Vec3::AXIS_Y + cross(rel_q.v, Vec3::AXIS_Y));
            joint.perp_axis_x = perp_axis_x;
            joint.perp_axis_y = perp_axis_y;

            let inv_inertia_sum = add_mm(iA, iB);
            let kxx = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_x));
            let kyy = dot(perp_axis_y, mul_mv(inv_inertia_sum, perp_axis_y));
            let kxy = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_y));

            let k = Matrix2::new(Vec2::new(kxx, kxy), Vec2::new(kxy, kyy));

            let w_rel = w_b - w_a;
            let cdot = Vec2::new(dot(w_rel, perp_axis_x), dot(w_rel, perp_axis_y));

            let max_impulse = context.h * joint.max_torque;
            let old_impulse = joint.perp_impulse;
            let cdot_plus_bias = Vec2::new(cdot.x + bias.x, cdot.y + bias.y);
            let sol = solve2(k, cdot_plus_bias);
            let mut delta_impulse = Vec2::new(
                -mass_scale * sol.x - impulse_scale * old_impulse.x,
                -mass_scale * sol.y - impulse_scale * old_impulse.y,
            );
            joint.perp_impulse =
                Vec2::new(old_impulse.x + delta_impulse.x, old_impulse.y + delta_impulse.y);
            let length_squared = joint.perp_impulse.x * joint.perp_impulse.x
                + joint.perp_impulse.y * joint.perp_impulse.y;
            if length_squared > max_impulse * max_impulse {
                let s = max_impulse / length_squared.sqrt();
                joint.perp_impulse =
                    Vec2::new(s * joint.perp_impulse.x, s * joint.perp_impulse.y);
            }

            delta_impulse = Vec2::new(
                joint.perp_impulse.x - old_impulse.x,
                joint.perp_impulse.y - old_impulse.y,
            );

            let angular_impulse = blend2(delta_impulse.x, perp_axis_x, delta_impulse.y, perp_axis_y);
            w_a = w_a - mul_mv(iA, angular_impulse);
            w_b = w_b + mul_mv(iB, angular_impulse);
        }

        if (state_a.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_a.angular_velocity = w_a;
        }

        if (state_b.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_b.angular_velocity = w_b;
        }

        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);

        base.set_parallel_joint(joint);
    }
}
