//! Prismatic joint. Body B slides along body A's x-axis.
//!
//! Translated from `Sources/Natsel/Simulation/PrismaticJoint.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::joint`: `World::create_joint(&mut self, &JointDef, JointType) -> JointPair`
//!   and `World::write_joint_sim(&mut self, i32, JointSim)`.
//! - `crate::simulation::contact_solver`: the `World::state_at` / `World::set_state_at` buffer
//!   accessors, kept safe wrappers over the raw awake-set state pointer, mirroring Swift.
//! - `crate::simulation::solver_prepare`: `World::joint_prepare_body` / `joint_prepare_sim`.
//! - `crate::collision::internal_math`: `Matrix2` / `solve2` (shared 2-by-2 point solver) and
//!   `delta_quat_to_rotation` (shared rotation-error extraction).

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::internal_math::{blend3, delta_quat_to_rotation, solve2, Matrix2};
use crate::math::functions::{
    add_mm, blend2, clamp_float, cross, det, dot, inv_mul_quat, invert_matrix,
    make_matrix_from_quat, max_float, mul_mv, mul_quat, rotate_vector, sub_pos,
};
use crate::math::types::{Matrix3, Quat, Transform, Vec2, Vec3};
use crate::simulation::solver_integration::{Softness, StepContext, make_soft};
use crate::simulation::world_runtime::{BodyFlags, BodyState, JointSim, SetType, World};
use crate::types::ids::JointID;
use crate::types::joint_defs::{JointType, PrismaticJointDef};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct PrismaticJointSim {
    pub perp_impulse: Vec2,
    pub angular_impulse: Vec3,
    pub spring_impulse: f32,
    pub motor_impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_motor_force: f32,
    pub motor_speed: f32,
    pub target_translation: f32,
    pub lower_translation: f32,
    pub upper_translation: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub joint_axis: Vec3,
    pub perp_axis_y: Vec3,
    pub perp_axis_z: Vec3,
    pub delta_center: Vec3,
    pub delta_angle: f32,
    pub rotation_mass: Matrix3,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_limit: bool,
    pub enable_motor: bool,
}

impl Default for PrismaticJointSim {
    /// Swift's `init() {}` applies every property default; the indices are `B3.nullIndex`, so
    /// the derived all-zeroes default is not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            perp_impulse: Vec2::default(),
            angular_impulse: Vec3::default(),
            spring_impulse: 0.0,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            max_motor_force: 0.0,
            motor_speed: 0.0,
            target_translation: 0.0,
            lower_translation: 0.0,
            upper_translation: 0.0,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            frame_a: Transform::default(),
            frame_b: Transform::default(),
            joint_axis: Vec3::default(),
            perp_axis_y: Vec3::default(),
            perp_axis_z: Vec3::default(),
            delta_center: Vec3::default(),
            delta_angle: 0.0,
            rotation_mass: Matrix3::default(),
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_limit: false,
            enable_motor: false,
        }
    }
}

impl JointSim {
    #[inline(always)]
    pub(crate) fn prismatic_joint(&self) -> PrismaticJointSim {
        self.prismatic()
    }

    #[inline(always)]
    pub(crate) fn set_prismatic_joint(&mut self, value: PrismaticJointSim) {
        self.set_prismatic(value);
    }
}

impl World {
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_prismatic_joint(&mut self, def: &PrismaticJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::PrismaticJoint);

        let mut sim = pair.sim;
        let mut joint = PrismaticJointSim::default();
        joint.hertz = def.hertz;
        joint.damping_ratio = def.damping_ratio;
        joint.max_motor_force = def.max_motor_force;
        joint.motor_speed = def.motor_speed;
        joint.target_translation = def.target_translation;
        joint.lower_translation = def.lower_translation;
        joint.upper_translation = def.upper_translation;
        joint.enable_spring = def.enable_spring;
        joint.enable_limit = def.enable_limit;
        joint.enable_motor = def.enable_motor;
        sim.set_prismatic_joint(joint);
        self.write_joint_sim(pair.joint_id, sim);

        let j = &self.joints[pair.joint_id as usize];
        JointID::new(pair.joint_id + 1, self.world_id, j.generation)
    }

    pub(crate) fn prepare_prismatic_joint(&self, base: &mut JointSim, context: &StepContext) {
        b3_assert(base.type_ == JointType::PrismaticJoint);

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

        let mut joint = base.prismatic_joint();
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
        joint.rotation_mass = invert_matrix(inv_inertia_sum);

        // Initial joint axes in world space
        let matrix_a = make_matrix_from_quat(joint.frame_a.q);
        joint.joint_axis = matrix_a.cx;
        joint.perp_axis_y = matrix_a.cy;
        joint.perp_axis_z = matrix_a.cz;

        joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

        if context.enable_warm_starting == false {
            joint.perp_impulse = Vec2::default();
            joint.angular_impulse = Vec3::default();
            joint.motor_impulse = 0.0;
            joint.spring_impulse = 0.0;
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
        }

        base.set_prismatic_joint(joint);
    }

    pub(crate) fn warm_start_prismatic_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::PrismaticJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let joint = base.prismatic_joint();
        // SAFETY: `states` is Swift's borrowed `UnsafeMutableBufferPointer<BodyState>` over the
        // awake set's state array, installed by the solver stage for the whole warm-start phase.
        // It outlives this call and the indices were encoded during prepare, exactly as in Swift;
        // no aliasing exists between this buffer and `base`.
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);

        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);
        let d = ((state_b.delta_position - state_a.delta_position) + joint.delta_center) + (r_b - r_a);
        let joint_axis = rotate_vector(state_a.delta_rotation, joint.joint_axis);
        let s_ax = cross(r_a + d, joint_axis);
        let s_bx = cross(r_b, joint_axis);

        let perp_y = rotate_vector(state_a.delta_rotation, joint.perp_axis_y);
        let perp_z = rotate_vector(state_a.delta_rotation, joint.perp_axis_z);
        let s_ay = cross(r_a + d, perp_y);
        let s_by = cross(r_b, perp_y);
        let s_az = cross(r_a + d, perp_z);
        let s_bz = cross(r_b, perp_z);

        let axial_impulse =
            joint.spring_impulse + joint.motor_impulse + joint.lower_impulse - joint.upper_impulse;
        let perp_impulse = joint.perp_impulse;

        let p = blend3(
            axial_impulse,
            joint_axis,
            perp_impulse.x,
            perp_y,
            perp_impulse.y,
            perp_z,
        );
        let la = blend3(
            axial_impulse,
            s_ax,
            perp_impulse.x,
            s_ay,
            perp_impulse.y,
            s_az,
        ) + joint.angular_impulse;
        let lb = blend3(
            axial_impulse,
            s_bx,
            perp_impulse.x,
            s_by,
            perp_impulse.y,
            s_bz,
        ) + joint.angular_impulse;

        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;
        v_a = v_a - mA * p;
        w_a = w_a - mul_mv(iA, la);
        v_b = v_b + mB * p;
        w_b = w_b + mul_mv(iB, lb);

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

    pub(crate) fn solve_prismatic_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        use_bias: bool,
        states: *mut BodyState,
    ) {
        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let mut joint = base.prismatic_joint();
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
        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

        let dc_a = state_a.delta_position;
        let dc_b = state_b.delta_position;
        let d = ((dc_b - dc_a) + joint.delta_center) + (r_b - r_a);

        let joint_axis = rotate_vector(state_a.delta_rotation, joint.joint_axis);
        let s_ax = cross(r_a + d, joint_axis);
        let s_bx = cross(r_b, joint_axis);
        let joint_translation = dot(d, joint_axis);
        let target_translation = joint.target_translation;

        // The axial effective mass must be fresh to avoid divergence when the joint is stressed
        let ka = mA + mB + dot(s_ax, mul_mv(iA, s_ax)) + dot(s_bx, mul_mv(iB, s_bx));
        let axial_mass: f32 = if ka > 0.0 { 1.0 / ka } else { 0.0 };

        // Solve spring
        if joint.enable_spring && fixed_rotation == false {
            // Get the substep relative rotation
            let c = joint_translation - target_translation;

            let bias = joint.spring_softness.bias_rate * c;
            let mass_scale = joint.spring_softness.mass_scale;
            let impulse_scale = joint.spring_softness.impulse_scale;

            let v_rel = (v_b + cross(w_b, r_b)) - v_a - cross(w_a, r_a + d);
            let cdot = dot(v_rel, joint_axis);
            let delta_impulse = -mass_scale * axial_mass * (cdot + bias)
                - impulse_scale * joint.spring_impulse;
            joint.spring_impulse += delta_impulse;

            let p = delta_impulse * joint_axis;
            let la = delta_impulse * s_ax;
            let lb = delta_impulse * s_bx;

            v_a = v_a - mA * p;
            w_a = w_a - mul_mv(iA, la);
            v_b = v_b + mB * p;
            w_b = w_b + mul_mv(iB, lb);
        }

        if joint.enable_motor && fixed_rotation == false {
            let v_rel = (v_b + cross(w_b, r_b)) - v_a - cross(w_a, r_a + d);
            let cdot = dot(v_rel, joint_axis) - joint.motor_speed;

            let mut delta_impulse = -axial_mass * cdot;
            let mut new_impulse = joint.motor_impulse + delta_impulse;
            let max_impulse = joint.max_motor_force * context.h;
            new_impulse = clamp_float(new_impulse, -max_impulse, max_impulse);
            delta_impulse = new_impulse - joint.motor_impulse;
            joint.motor_impulse = new_impulse;

            let p = delta_impulse * joint_axis;
            let la = delta_impulse * s_ax;
            let lb = delta_impulse * s_bx;

            v_a = v_a - mA * p;
            w_a = w_a - mul_mv(iA, la);
            v_b = v_b + mB * p;
            w_b = w_b + mul_mv(iB, lb);
        }

        if joint.enable_limit && fixed_rotation == false {
            let speculative_distance = 0.25 * (joint.upper_translation - joint.lower_translation);

            // Lower limit
            {
                let c = joint_translation - joint.lower_translation;

                if c < speculative_distance {
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

                    let v_rel = (v_b + cross(w_b, r_b)) - v_a - cross(w_a, r_a + d);
                    let cdot = dot(v_rel, joint_axis);
                    let old_impulse = joint.lower_impulse;
                    let mut delta_impulse = -mass_scale * axial_mass * (cdot + bias)
                        - impulse_scale * old_impulse;
                    joint.lower_impulse = max_float(old_impulse + delta_impulse, 0.0);
                    delta_impulse = joint.lower_impulse - old_impulse;

                    let p = delta_impulse * joint_axis;
                    let la = delta_impulse * s_ax;
                    let lb = delta_impulse * s_bx;

                    v_a = v_a - mA * p;
                    w_a = w_a - mul_mv(iA, la);
                    v_b = v_b + mB * p;
                    w_b = w_b + mul_mv(iB, lb);
                } else {
                    joint.lower_impulse = 0.0;
                }
            }

            // Upper limit
            {
                let c = joint.upper_translation - joint_translation;

                if c < speculative_distance {
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

                    // sign flipped on Cdot
                    let v_rel = (v_b + cross(w_b, r_b)) - v_a - cross(w_a, r_a + d);
                    let cdot = -dot(v_rel, joint_axis);
                    let old_impulse = joint.upper_impulse;
                    let delta_impulse = -mass_scale * axial_mass * (cdot + bias)
                        - impulse_scale * old_impulse;
                    joint.upper_impulse = max_float(old_impulse + delta_impulse, 0.0);

                    // sign flipped on applied impulse
                    let neg_delta_impulse = old_impulse - joint.upper_impulse;
                    let p = neg_delta_impulse * joint_axis;
                    let la = neg_delta_impulse * s_ax;
                    let lb = neg_delta_impulse * s_bx;

                    v_a = v_a - mA * p;
                    w_a = w_a - mul_mv(iA, la);
                    v_b = v_b + mB * p;
                    w_b = w_b + mul_mv(iB, lb);
                } else {
                    joint.upper_impulse = 0.0;
                }
            }
        }

        // Rotation constraint
        if fixed_rotation == false {
            let mut bias = Vec3::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;

            if use_bias {
                let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
                let quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

                let rel_q = inv_mul_quat(quat_a, quat_b);
                let target_quat = Quat::IDENTITY;
                let delta_rotation = delta_quat_to_rotation(rel_q, target_quat);
                let c = -(rotate_vector(quat_a, delta_rotation));

                bias = base.constraint_softness.bias_rate * c;
                mass_scale = base.constraint_softness.mass_scale;
                impulse_scale = base.constraint_softness.impulse_scale;
            }

            let cdot = w_b - w_a;
            let impulse = (-mass_scale) * mul_mv(joint.rotation_mass, cdot + bias)
                - impulse_scale * joint.angular_impulse;
            joint.angular_impulse = joint.angular_impulse + impulse;

            w_a = w_a - mul_mv(iA, impulse);
            w_b = w_b + mul_mv(iB, impulse);
        }

        // Solve point-to-line constraint
        {
            let perp_y = rotate_vector(state_a.delta_rotation, joint.perp_axis_y);
            let perp_z = rotate_vector(state_a.delta_rotation, joint.perp_axis_z);

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

            // K = [(1/mA + 1/mB) * eye(2) - skew(rA) * invIA * skew(rA) - skew(rB) * invIB * skew(rB)]
            let s_ay = cross(r_a + d, perp_y);
            let s_by = cross(r_b, perp_y);
            let s_az = cross(r_a + d, perp_z);
            let s_bz = cross(r_b, perp_z);

            let kyy = mA + mB + dot(s_ay, mul_mv(iA, s_ay)) + dot(s_by, mul_mv(iB, s_by));
            let kyz = dot(s_ay, mul_mv(iA, s_az)) + dot(s_by, mul_mv(iB, s_bz));
            let kzz = mA + mB + dot(s_az, mul_mv(iA, s_az)) + dot(s_bz, mul_mv(iB, s_bz));

            let k = Matrix2::new(Vec2::new(kyy, kyz), Vec2::new(kyz, kzz));

            let old_impulse = joint.perp_impulse;
            let sol = solve2(k, Vec2::new(cdot.x + bias.x, cdot.y + bias.y));
            let delta_impulse = Vec2::new(
                (-mass_scale) * sol.x - impulse_scale * old_impulse.x,
                (-mass_scale) * sol.y - impulse_scale * old_impulse.y,
            );
            joint.perp_impulse = Vec2::new(
                old_impulse.x + delta_impulse.x,
                old_impulse.y + delta_impulse.y,
            );

            let p = blend2(delta_impulse.x, perp_y, delta_impulse.y, perp_z);

            v_a = v_a - mA * p;
            w_a = w_a - mul_mv(iA, blend2(delta_impulse.x, s_ay, delta_impulse.y, s_az));
            v_b = v_b + mB * p;
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

        base.set_prismatic_joint(joint);
    }
}
