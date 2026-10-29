//! Revolute joint. A hinge: point-to-point translation, collinearity of the frame x/y axes,
//! and an optional twist spring, motor and angle limits about body A's z-axis.
//!
//! Translated from `Sources/Natsel/Simulation/RevoluteJoint.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::joint`: `World::create_joint(&mut self, &JointDef, JointType) -> JointPair`
//!   and `World::write_joint_sim(&mut self, i32, JointSim)`.
//! - `crate::simulation::contact_solver`: the `World::state_at` / `World::set_state_at` buffer
//!   accessors, kept safe wrappers over the raw awake-set state pointer, mirroring Swift.
//! - `crate::simulation::solver_prepare`: `World::joint_prepare_body` / `joint_prepare_sim`.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::internal_math::{Matrix2, solve2};
use crate::math::functions::{
    abs_float, add_mm, clamp_float, cross, det, dot, dot_quat, get_twist_angle, inv_mul_quat,
    max_float, mul_add, mul_mv, mul_quat, negate_quat, rotate_vector, sub_pos,
};
use crate::math::types::{Matrix3, Transform, Vec2, Vec3};
use crate::simulation::solver_integration::{Softness, StepContext, make_soft};
use crate::simulation::world_runtime::{BodyFlags, BodyState, JointSim, SetType, World};
use crate::types::ids::JointID;
use crate::types::joint_defs::{JointType, RevoluteJointDef};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct RevoluteJointSim {
    pub linear_impulse: Vec3,
    pub perp_impulse: Vec2,
    pub spring_impulse: f32,
    pub motor_impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_motor_torque: f32,
    pub motor_speed: f32,
    pub target_angle: f32,
    pub lower_angle: f32,
    pub upper_angle: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub rotation_axis_z: Vec3,
    pub perp_axis_x: Vec3,
    pub perp_axis_y: Vec3,
    pub delta_center: Vec3,
    pub delta_angle: f32,
    pub axial_mass: f32,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_motor: bool,
    pub enable_limit: bool,
}

impl Default for RevoluteJointSim {
    /// Swift's `init() {}` applies every property default; the indices are `B3.nullIndex`, so
    /// the derived all-zeroes default is not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            linear_impulse: Vec3::default(),
            perp_impulse: Vec2::default(),
            spring_impulse: 0.0,
            motor_impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            max_motor_torque: 0.0,
            motor_speed: 0.0,
            target_angle: 0.0,
            lower_angle: 0.0,
            upper_angle: 0.0,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            frame_a: Transform::default(),
            frame_b: Transform::default(),
            rotation_axis_z: Vec3::default(),
            perp_axis_x: Vec3::default(),
            perp_axis_y: Vec3::default(),
            delta_center: Vec3::default(),
            delta_angle: 0.0,
            axial_mass: 0.0,
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_motor: false,
            enable_limit: false,
        }
    }
}

/// Solve a 3-by-3 point-to-point constraint.
#[inline(always)]
pub(crate) fn solve_point_constraint(
    m_a: f32,
    m_b: f32,
    i_a: Matrix3,
    i_b: Matrix3,
    r_a: Vec3,
    r_b: Vec3,
    rhs: Vec3,
) -> Vec3 {
    let axy = r_a.z * i_a.cy.y - r_a.y * i_a.cz.y;
    let axz = r_a.z * i_a.cy.z - r_a.y * i_a.cz.z;
    let ay = r_a.x * i_a.cz - r_a.z * i_a.cx;
    let az = r_a.y * i_a.cx - r_a.x * i_a.cy;

    let bxy = r_b.z * i_b.cy.y - r_b.y * i_b.cz.y;
    let bxz = r_b.z * i_b.cy.z - r_b.y * i_b.cz.z;
    let by = r_b.x * i_b.cz - r_b.z * i_b.cx;
    let bz = r_b.y * i_b.cx - r_b.x * i_b.cy;

    let m = m_a + m_b;
    let kxx = (axy * r_a.z - axz * r_a.y) + (bxy * r_b.z - bxz * r_b.y) + m;
    let kyy = (ay.z * r_a.x - ay.x * r_a.z) + (by.z * r_b.x - by.x * r_b.z) + m;
    let kzz = (az.x * r_a.y - az.y * r_a.x) + (bz.x * r_b.y - bz.y * r_b.x) + m;
    let kxy = (ay.y * r_a.z - ay.z * r_a.y) + (by.y * r_b.z - by.z * r_b.y);
    let kxz = (az.y * r_a.z - az.z * r_a.y) + (bz.y * r_b.z - bz.z * r_b.y);
    let kyz = (az.z * r_a.x - az.x * r_a.z) + (bz.z * r_b.x - bz.x * r_b.z);

    let cxx = kyy * kzz - kyz * kyz;
    let cyy = kxx * kzz - kxz * kxz;
    let czz = kxx * kyy - kxy * kxy;
    let cxy = kxz * kyz - kxy * kzz;
    let cxz = kxy * kyz - kyy * kxz;
    let cyz = kxy * kxz - kxx * kyz;

    let determinant = kxx * cxx + kxy * cxy + kxz * cxz;
    if abs_float(determinant) > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / determinant;
        return Vec3::new(
            inv_det * (cxx * rhs.x + cxy * rhs.y + cxz * rhs.z),
            inv_det * (cxy * rhs.x + cyy * rhs.y + cyz * rhs.z),
            inv_det * (cxz * rhs.x + cyz * rhs.y + czz * rhs.z),
        );
    }

    Vec3::default()
}

impl JointSim {
    #[inline(always)]
    pub(crate) fn revolute_joint(&self) -> RevoluteJointSim {
        self.revolute()
    }

    #[inline(always)]
    pub(crate) fn set_revolute_joint(&mut self, value: RevoluteJointSim) {
        self.set_revolute(value);
    }
}

impl World {
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_revolute_joint(&mut self, def: &RevoluteJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::RevoluteJoint);

        let mut sim = pair.sim;
        let mut joint = RevoluteJointSim::default();
        joint.hertz = def.hertz;
        joint.damping_ratio = def.damping_ratio;
        joint.max_motor_torque = def.max_motor_torque;
        joint.motor_speed = def.motor_speed;
        joint.target_angle = def.target_angle;
        joint.lower_angle = def.lower_angle;
        joint.upper_angle = def.upper_angle;
        joint.enable_spring = def.enable_spring;
        joint.enable_motor = def.enable_motor;
        joint.enable_limit = def.enable_limit;
        sim.set_revolute_joint(joint);
        self.write_joint_sim(pair.joint_id, sim);

        let j = &self.joints[pair.joint_id as usize];
        JointID::new(pair.joint_id + 1, self.world_id, j.generation)
    }

    pub(crate) fn prepare_revolute_joint(&self, base: &mut JointSim, context: &StepContext) {
        b3_assert(base.type_ == JointType::RevoluteJoint);

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

        let mut joint = base.revolute_joint();
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

        {
            // Rotation axis is the z-axis of body A.
            let rotation_axis_z = rotate_vector(joint.frame_a.q, Vec3::AXIS_Z);
            let k = dot(rotation_axis_z, mul_mv(inv_inertia_sum, rotation_axis_z));
            joint.axial_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
            joint.rotation_axis_z = rotation_axis_z;
        }

        let rel_q = inv_mul_quat(joint.frame_a.q, joint.frame_b.q);

        {
            // These are needed for warm starting
            joint.perp_axis_x = 0.5 * rotate_vector(
                joint.frame_a.q,
                rel_q.s * Vec3::AXIS_X + cross(rel_q.v, Vec3::AXIS_X),
            );
            joint.perp_axis_y = 0.5 * rotate_vector(
                joint.frame_a.q,
                rel_q.s * Vec3::AXIS_Y + cross(rel_q.v, Vec3::AXIS_Y),
            );
        }

        joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

        if !context.enable_warm_starting {
            joint.linear_impulse = Vec3::default();
            joint.perp_impulse = Vec2::default();
            joint.motor_impulse = 0.0;
            joint.spring_impulse = 0.0;
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
        }

        base.set_revolute_joint(joint);
    }

    pub(crate) fn warm_start_revolute_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::RevoluteJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let joint = base.revolute_joint();
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

        let axial_impulse =
            joint.spring_impulse + joint.motor_impulse + joint.lower_impulse - joint.upper_impulse;
        let mut angular_impulse =
            joint.perp_impulse.x * joint.perp_axis_x + joint.perp_impulse.y * joint.perp_axis_y;
        angular_impulse = mul_add(angular_impulse, axial_impulse, joint.rotation_axis_z);

        v_a = v_a - mA * joint.linear_impulse;
        w_a = w_a - mul_mv(iA, cross(r_a, joint.linear_impulse) + angular_impulse);

        v_b = v_b + mB * joint.linear_impulse;
        w_b = w_b + mul_mv(iB, cross(r_b, joint.linear_impulse) + angular_impulse);

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

    pub(crate) fn solve_revolute_joint(
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

        let mut joint = base.revolute_joint();
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

        if !fixed_rotation && (joint.enable_spring || joint.enable_motor || joint.enable_limit) {
            let axis = joint.rotation_axis_z;
            let g_a = mul_mv(iA, axis);
            let g_b = mul_mv(iB, axis);
            let k = dot(axis, g_a + g_b);
            let mut axial_c_dot = dot(w_b - w_a, axis);
            let mut axial_impulse: f32 = 0.0;

            // Solve spring
            if joint.enable_spring {
                // Get the substep relative rotation
                let target_angle = joint.target_angle;
                let angle = get_twist_angle(rel_q);
                let c = angle - target_angle;

                let bias = joint.spring_softness.bias_rate * c;
                let mass_scale = joint.spring_softness.mass_scale;
                let impulse_scale = joint.spring_softness.impulse_scale;

                let delta_impulse = -mass_scale * joint.axial_mass * (axial_c_dot + bias)
                    - impulse_scale * joint.spring_impulse;
                joint.spring_impulse += delta_impulse;

                axial_c_dot += k * delta_impulse;
                axial_impulse += delta_impulse;
            }

            if joint.enable_motor {
                let cdot = axial_c_dot - joint.motor_speed;
                let old_impulse = joint.motor_impulse;
                let max_impulse = joint.max_motor_torque * context.h;
                let delta_impulse =
                    clamp_float(-joint.axial_mass * cdot, -max_impulse - old_impulse, max_impulse - old_impulse);
                joint.motor_impulse = old_impulse + delta_impulse;

                axial_c_dot += k * delta_impulse;
                axial_impulse += delta_impulse;
            }

            if joint.enable_limit {
                let angle = get_twist_angle(rel_q);

                // todo does an updated twist axis help?

                // Lower limit
                {
                    let c = angle - joint.lower_angle;
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

                    let old_impulse = joint.lower_impulse;
                    let delta_impulse = max_float(
                        -mass_scale * joint.axial_mass * (axial_c_dot + bias)
                            - impulse_scale * old_impulse,
                        -old_impulse,
                    );
                    joint.lower_impulse = old_impulse + delta_impulse;

                    axial_c_dot += k * delta_impulse;
                    axial_impulse += delta_impulse;
                }

                // Upper limit
                {
                    let c = joint.upper_angle - angle;
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
                    let cdot = -axial_c_dot;
                    let old_impulse = joint.upper_impulse;
                    let delta_impulse = max_float(
                        -mass_scale * joint.axial_mass * (cdot + bias) - impulse_scale * old_impulse,
                        -old_impulse,
                    );
                    joint.upper_impulse = old_impulse + delta_impulse;

                    // sign flipped on applied impulse
                    axial_impulse -= delta_impulse;
                }
            }

            w_a = w_a - axial_impulse * g_a;
            w_b = w_b + axial_impulse * g_b;
        }

        // Collinearity constraint
        if !fixed_rotation {
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
            joint.perp_axis_x = perp_axis_x;
            joint.perp_axis_y = perp_axis_y;

            let inv_inertia_sum = add_mm(iA, iB);
            let kxx = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_x));
            let kyy = dot(perp_axis_y, mul_mv(inv_inertia_sum, perp_axis_y));
            let kxy = dot(perp_axis_x, mul_mv(inv_inertia_sum, perp_axis_y));

            let k = Matrix2::new(Vec2::new(kxx, kxy), Vec2::new(kxy, kyy));

            let w_rel = w_b - w_a;
            let cdot = Vec2::new(dot(w_rel, perp_axis_x), dot(w_rel, perp_axis_y));
            let old_impulse = joint.perp_impulse;
            let sol = solve2(k, Vec2::new(cdot.x + bias.x, cdot.y + bias.y));
            let delta_impulse = Vec2::new(
                (-mass_scale) * sol.x - impulse_scale * old_impulse.x,
                (-mass_scale) * sol.y - impulse_scale * old_impulse.y,
            );
            joint.perp_impulse = Vec2::new(
                joint.perp_impulse.x + delta_impulse.x,
                joint.perp_impulse.y + delta_impulse.y,
            );

            let angular_impulse =
                delta_impulse.x * perp_axis_x + delta_impulse.y * perp_axis_y;
            w_a = w_a - mul_mv(iA, angular_impulse);
            w_b = w_b + mul_mv(iB, angular_impulse);
        }

        // Solve point-to-point constraint
        {
            let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
            let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);

            let cdot = (v_b + cross(w_b, r_b)) - v_a - cross(w_a, r_a);

            let mut bias = Vec3::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias {
                let dc_a = state_a.delta_position;
                let dc_b = state_b.delta_position;

                let separation = ((dc_b - dc_a) + (r_b - r_a)) + joint.delta_center;

                bias = base.constraint_softness.bias_rate * separation;
                mass_scale = base.constraint_softness.mass_scale;
                impulse_scale = base.constraint_softness.impulse_scale;
            }

            let b = solve_point_constraint(mA, mB, base.inv_ia, base.inv_ib, r_a, r_b, cdot + bias);

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

        base.set_revolute_joint(joint);
    }
}
