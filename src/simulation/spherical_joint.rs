//! Spherical joint. A ball-and-socket: point-to-point translation with an optional 3-by-3
//! rotation spring and motor, plus cone (swing) and twist limits about the frame axes.
//!
//! Translated from `Sources/Natsel/Simulation/SphericalJoint.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
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
    add_mm, cross, det, dot, get_swing_angle, get_twist_angle, inv_mul_quat, invert_matrix,
    length, max_float, mul_add, mul_mv, mul_quat, normalize, rotate_vector, sub_pos,
};
use crate::math::types::{Matrix3, Quat, Transform, Vec3};
use crate::simulation::revolute_joint::solve_point_constraint;
use crate::simulation::solver_integration::{Softness, StepContext, make_soft};
use crate::simulation::world_runtime::{BodyFlags, BodyState, JointSim, SetType, World};
use crate::types::ids::JointID;
use crate::types::joint_defs::{JointType, SphericalJointDef};

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct SphericalJointSim {
    pub linear_impulse: Vec3,
    pub spring_impulse: Vec3,
    pub motor_impulse: Vec3,
    pub lower_twist_impulse: f32,
    pub upper_twist_impulse: f32,
    pub swing_impulse: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_motor_torque: f32,
    pub motor_velocity: Vec3,
    pub lower_twist_angle: f32,
    pub upper_twist_angle: f32,
    pub cone_angle: f32,
    pub target_rotation: Quat,

    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,
    pub swing_axis: Vec3,
    pub twist_jacobian: Vec3,

    pub rotation_mass: Matrix3,
    pub swing_mass: f32,
    pub twist_mass: f32,
    pub spring_softness: Softness,

    pub enable_spring: bool,
    pub enable_motor: bool,
    pub enable_cone_limit: bool,
    pub enable_twist_limit: bool,
}

impl Default for SphericalJointSim {
    /// Swift's `init() {}` applies every property default; the indices are `B3.nullIndex`, so
    /// the derived all-zeroes default is not correct here.
    #[inline]
    fn default() -> Self {
        Self {
            linear_impulse: Vec3::default(),
            spring_impulse: Vec3::default(),
            motor_impulse: Vec3::default(),
            lower_twist_impulse: 0.0,
            upper_twist_impulse: 0.0,
            swing_impulse: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            max_motor_torque: 0.0,
            motor_velocity: Vec3::default(),
            lower_twist_angle: 0.0,
            upper_twist_angle: 0.0,
            cone_angle: 0.0,
            target_rotation: Quat::default(),
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            frame_a: Transform::default(),
            frame_b: Transform::default(),
            delta_center: Vec3::default(),
            swing_axis: Vec3::default(),
            twist_jacobian: Vec3::default(),
            rotation_mass: Matrix3::default(),
            swing_mass: 0.0,
            twist_mass: 0.0,
            spring_softness: Softness::default(),
            enable_spring: false,
            enable_motor: false,
            enable_cone_limit: false,
            enable_twist_limit: false,
        }
    }
}

impl JointSim {
    #[inline(always)]
    pub(crate) fn spherical_joint(&self) -> SphericalJointSim {
        self.spherical()
    }

    #[inline(always)]
    pub(crate) fn set_spherical_joint(&mut self, value: SphericalJointSim) {
        self.set_spherical(value);
    }
}

impl World {
    /// Swift marked this `@discardableResult`; ignore the return value where Swift did.
    pub fn create_spherical_joint(&mut self, def: &SphericalJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::SphericalJoint);

        let mut sim = pair.sim;
        let mut joint = SphericalJointSim::default();
        joint.hertz = def.hertz;
        joint.damping_ratio = def.damping_ratio;
        joint.max_motor_torque = def.max_motor_torque;
        joint.motor_velocity = def.motor_velocity;
        joint.lower_twist_angle = def.lower_twist_angle;
        joint.upper_twist_angle = def.upper_twist_angle;
        joint.cone_angle = def.cone_angle;
        joint.target_rotation = def.target_rotation;
        joint.enable_spring = def.enable_spring;
        joint.enable_motor = def.enable_motor;
        joint.enable_cone_limit = def.enable_cone_limit;
        joint.enable_twist_limit = def.enable_twist_limit;
        sim.set_spherical_joint(joint);
        self.write_joint_sim(pair.joint_id, sim);

        let j = &self.joints[pair.joint_id as usize];
        JointID::new(pair.joint_id + 1, self.world_id, j.generation)
    }

    pub(crate) fn prepare_spherical_joint(&self, base: &mut JointSim, context: &StepContext) {
        b3_assert(base.type_ == JointType::SphericalJoint);

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

        let mut joint = base.spherical_joint();
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

        // Cone axis is the z-axis of body A.
        let cone_axis = rotate_vector(joint.frame_a.q, Vec3::AXIS_Z);

        // Twist axis is the z-axis of body B.
        let twist_axis = rotate_vector(joint.frame_b.q, Vec3::AXIS_Z);

        if joint.enable_cone_limit {
            // Swing axis may be zero
            let swing_axis = normalize(cross(cone_axis, twist_axis));
            let k = dot(swing_axis, mul_mv(inv_inertia_sum, swing_axis));
            joint.swing_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
            joint.swing_axis = swing_axis;
        }

        if joint.enable_twist_limit {
            let rel_q = inv_mul_quat(joint.frame_a.q, joint.frame_b.q);
            let tan_theta_over_2 = ((rel_q.v.x * rel_q.v.x + rel_q.v.y * rel_q.v.y)
                / (rel_q.v.z * rel_q.v.z + rel_q.s * rel_q.s))
                .sqrt();

            let swing_axis = normalize(cross(cone_axis, twist_axis));
            let perp_axis = cross(swing_axis, cone_axis);
            let twist_jacobian = mul_add(cone_axis, tan_theta_over_2, perp_axis);
            let k = dot(twist_jacobian, mul_mv(inv_inertia_sum, twist_jacobian));
            joint.twist_mass = if k > 0.0 { 1.0 / k } else { 0.0 };
            joint.twist_jacobian = twist_jacobian;
        }

        if !base.fixed_rotation {
            joint.rotation_mass = invert_matrix(inv_inertia_sum);
        } else {
            joint.rotation_mass = Matrix3::default();
        }

        joint.spring_softness = make_soft(joint.hertz, joint.damping_ratio, context.h);

        if !context.enable_warm_starting {
            joint.linear_impulse = Vec3::default();
            joint.motor_impulse = Vec3::default();
            joint.spring_impulse = Vec3::default();
            joint.swing_impulse = 0.0;
            joint.lower_twist_impulse = 0.0;
            joint.upper_twist_impulse = 0.0;
        }

        base.set_spherical_joint(joint);
    }

    pub(crate) fn warm_start_spherical_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        states: *mut BodyState,
    ) {
        b3_assert(base.type_ == JointType::SphericalJoint);

        let mA = base.inv_mass_a;
        let mB = base.inv_mass_b;
        let iA = base.inv_ia;
        let iB = base.inv_ib;

        let joint = base.spherical_joint();
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

        let mut angular_impulse = joint.spring_impulse + joint.motor_impulse;
        angular_impulse = angular_impulse - joint.swing_impulse * joint.swing_axis;
        angular_impulse = mul_add(
            angular_impulse,
            joint.lower_twist_impulse - joint.upper_twist_impulse,
            joint.twist_jacobian,
        );

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

    pub(crate) fn solve_spherical_joint(
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

        let mut joint = base.spherical_joint();
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
        let quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);

        let rel_q = inv_mul_quat(quat_a, quat_b);

        if !fixed_rotation
            && (joint.enable_spring
                || joint.enable_motor
                || joint.enable_twist_limit
                || joint.enable_cone_limit)
        {
            let inv_inertia_sum = add_mm(iA, iB);
            let mut cdot = w_b - w_a;
            let mut angular_impulse = Vec3::default();

            // Solve spring
            if joint.enable_spring {
                // Rotation constraint error
                let delta_rotation = delta_quat_to_rotation(rel_q, joint.target_rotation);
                let c = -(rotate_vector(quat_a, delta_rotation));

                let bias = joint.spring_softness.bias_rate * c;
                let mass_scale = joint.spring_softness.mass_scale;
                let impulse_scale = joint.spring_softness.impulse_scale;

                let impulse = (-mass_scale) * mul_mv(joint.rotation_mass, cdot + bias)
                    - impulse_scale * joint.spring_impulse;
                joint.spring_impulse = joint.spring_impulse + impulse;

                cdot = cdot + mul_mv(inv_inertia_sum, impulse);
                angular_impulse = angular_impulse + impulse;
            }

            if joint.enable_motor {
                let mut lambda = -(mul_mv(joint.rotation_mass, cdot - joint.motor_velocity));
                let mut new_impulse = joint.motor_impulse + lambda;
                let impulse_length = length(new_impulse);
                let max_impulse = joint.max_motor_torque * context.h;
                if impulse_length > max_impulse {
                    new_impulse = (max_impulse / impulse_length) * new_impulse;
                }

                lambda = new_impulse - joint.motor_impulse;
                joint.motor_impulse = new_impulse;

                cdot = cdot + mul_mv(inv_inertia_sum, lambda);
                angular_impulse = angular_impulse + lambda;
            }

            if joint.enable_twist_limit {
                let twist_angle = get_twist_angle(rel_q);

                let twist_jacobian = joint.twist_jacobian;
                let twist_response = mul_mv(inv_inertia_sum, twist_jacobian);

                // Lower limit
                {
                    let c = twist_angle - joint.lower_twist_angle;
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

                    let twist_c_dot = dot(cdot, twist_jacobian);
                    let old_impulse = joint.lower_twist_impulse;
                    let delta_impulse = max_float(
                        -mass_scale * joint.twist_mass * (twist_c_dot + bias)
                            - impulse_scale * old_impulse,
                        -old_impulse,
                    );
                    joint.lower_twist_impulse = old_impulse + delta_impulse;

                    cdot = mul_add(cdot, delta_impulse, twist_response);
                    angular_impulse = mul_add(angular_impulse, delta_impulse, twist_jacobian);
                }

                // Upper limit
                {
                    let c = joint.upper_twist_angle - twist_angle;
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
                    let twist_c_dot = -dot(cdot, twist_jacobian);
                    let old_impulse = joint.upper_twist_impulse;
                    let delta_impulse = max_float(
                        -mass_scale * joint.twist_mass * (twist_c_dot + bias)
                            - impulse_scale * old_impulse,
                        -old_impulse,
                    );
                    joint.upper_twist_impulse = old_impulse + delta_impulse;

                    // sign flipped on applied impulse
                    cdot = cdot - delta_impulse * twist_response;
                    angular_impulse = angular_impulse - delta_impulse * twist_jacobian;
                }
            }

            if joint.enable_cone_limit {
                let swing_angle = get_swing_angle(rel_q);

                let swing_axis = joint.swing_axis;

                let c = joint.cone_angle - swing_angle;
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
                let swing_c_dot = -dot(cdot, swing_axis);
                let old_impulse = joint.swing_impulse;
                let delta_impulse = max_float(
                    -mass_scale * joint.swing_mass * (swing_c_dot + bias) - impulse_scale * old_impulse,
                    -old_impulse,
                );
                joint.swing_impulse = old_impulse + delta_impulse;

                // sign flipped on applied impulse
                angular_impulse = angular_impulse - delta_impulse * swing_axis;
            }

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

        base.set_spherical_joint(joint);
    }
}
