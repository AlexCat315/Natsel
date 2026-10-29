//! Coupling joints: gear, rack and pinion, and pulley. Each couples one degree of freedom of two
//! bodies. Angles and translations are tracked across steps without wrapping, so a gear may turn
//! any number of revolutions. Gear and rack joints can measure their coordinates through the
//! revolute and prismatic joints that hold the bodies, which keeps the coupling exact on moving
//! mounts.
//!
//! Translated from `Sources/Natsel/Simulation/CouplingJoints.swift`.
//!
//! Foreign members assumed before their files landed (resolve at build time):
//! - `crate::simulation::world_queries::World::body_transform_quick` (`WorldQueries.swift`)
//! - `crate::simulation::joint::{World::create_joint, World::read_joint_sim,
//!   World::write_joint_sim, World::wake_joint_bodies, JointPair}` (`Joint.swift`)
//! - `crate::simulation::contact_solver::{World::state_at, World::set_state_at}`
//!   (`ContactSolver.swift`); Swift's `UnsafeMutableBufferPointer<BodyState>` solver parameter is
//!   the `*mut BodyState` convention already used by `solver_stages.rs` / `solver_prepare.rs`
//! - `crate::simulation::six_dof_joint::World::six_dof_impulses` (`SixDOFJoint.swift`)
//! - `crate::simulation::path_joint::{World::path_joint_impulses, World::draw_path_joint}`
//!   (`PathJoint.swift`)

use crate::base::constants as b3;
use crate::math::functions::{
    abs_float, cross, distance_pos, dot, get_length_and_normalize, get_twist_angle, inv_mul_quat,
    is_valid_position, length, max_float, mul_add, mul_mv, mul_quat, mul_world_transforms,
    offset_pos, pos_mul_add, pos_mul_sub, rotate_vector, sub_pos, transform_world_point,
    unwind_angle,
};
use crate::math::types::{Pos, Quat, Transform, Vec3, WorldTransform};
use crate::types::debug_draw::{HexColor, debug_colors};
use crate::types::ids::JointID;
use crate::types::joint_defs::{
    GearJointDef, JointDef, JointType, PulleyJointDef, RackAndPinionJointDef,
};
use crate::simulation::solver_integration::StepContext;
use crate::simulation::world_runtime::{BodySim, BodyState, JointSim, SetType, World};

/// Measures the rotation of a body about a hinge axis, unwrapped across steps.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct AngleProbe {
    // Through a revolute joint
    pub uses_joint: bool,
    pub joint_index: i32,
    pub joint_generation: u16,
    pub joint_body_a: i32,
    pub joint_body_b: i32,
    pub joint_frame_a: Quat,
    pub joint_frame_b: Quat,
    pub sign: f32,

    // Directly from the body
    pub body: i32,
    pub hinge_frame: Quat,
    pub rotation0: Quat,

    /// Last wrapped measurement and the unwrapped total.
    pub twist: f32,
    pub angle: f32,
}

impl Default for AngleProbe {
    /// Swift's memberwise defaults: `B3.nullIndex` sentinels, identity frames, `sign = 1`.
    #[inline]
    fn default() -> Self {
        Self {
            uses_joint: false,
            joint_index: b3::NULL_INDEX,
            joint_generation: 0,
            joint_body_a: b3::NULL_INDEX,
            joint_body_b: b3::NULL_INDEX,
            joint_frame_a: Quat::IDENTITY,
            joint_frame_b: Quat::IDENTITY,
            sign: 1.0,
            body: b3::NULL_INDEX,
            hinge_frame: Quat::IDENTITY,
            rotation0: Quat::IDENTITY,
            twist: 0.0,
            angle: 0.0,
        }
    }
}

/// Measures the translation of a body along an axis.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct TranslationProbe {
    pub uses_joint: bool,
    pub joint_index: i32,
    pub joint_generation: u16,
    pub joint_body_a: i32,
    pub joint_body_b: i32,
    pub joint_frame_a: Transform,
    pub joint_anchor_b: Vec3,
    pub sign: f32,

    pub body: i32,
    pub center0: Pos,
    pub axis0: Vec3,

    pub value: f32,
}

impl Default for TranslationProbe {
    #[inline]
    fn default() -> Self {
        Self {
            uses_joint: false,
            joint_index: b3::NULL_INDEX,
            joint_generation: 0,
            joint_body_a: b3::NULL_INDEX,
            joint_body_b: b3::NULL_INDEX,
            joint_frame_a: Transform::IDENTITY,
            joint_anchor_b: Vec3::ZERO,
            sign: 1.0,
            body: b3::NULL_INDEX,
            center0: Pos::ZERO,
            axis0: Vec3::ZERO,
            value: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct GearJointSim {
    pub ratio: f32,
    pub impulse: f32,
    pub probe_a: AngleProbe,
    pub probe_b: AngleProbe,
    pub offset: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub axis_a: Vec3,
    pub axis_b: Vec3,
    pub mass: f32,
    pub position: f32,
}

impl Default for GearJointSim {
    #[inline]
    fn default() -> Self {
        Self {
            ratio: 1.0,
            impulse: 0.0,
            probe_a: AngleProbe::default(),
            probe_b: AngleProbe::default(),
            offset: 0.0,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            axis_a: Vec3::ZERO,
            axis_b: Vec3::ZERO,
            mass: 0.0,
            position: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct RackAndPinionJointSim {
    pub ratio: f32,
    pub impulse: f32,
    pub pinion: AngleProbe,
    pub rack: TranslationProbe,
    pub offset: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub axis_a: Vec3,
    pub axis_b: Vec3,
    pub mass: f32,
    pub position: f32,
}

impl Default for RackAndPinionJointSim {
    #[inline]
    fn default() -> Self {
        Self {
            ratio: 1.0,
            impulse: 0.0,
            pinion: AngleProbe::default(),
            rack: TranslationProbe::default(),
            offset: 0.0,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            axis_a: Vec3::ZERO,
            axis_b: Vec3::ZERO,
            mass: 0.0,
            position: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct PulleyJointSim {
    pub ground_anchor_a: Pos,
    pub ground_anchor_b: Pos,
    pub ratio: f32,
    pub min_length: f32,
    pub max_length: f32,
    pub impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub center_a: Pos,
    pub center_b: Pos,
    pub r_a: Vec3,
    pub r_b: Vec3,
    pub mass: f32,
}

impl Default for PulleyJointSim {
    #[inline]
    fn default() -> Self {
        Self {
            ground_anchor_a: Pos::ZERO,
            ground_anchor_b: Pos::ZERO,
            ratio: 1.0,
            min_length: 0.0,
            max_length: 0.0,
            impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            center_a: Pos::ZERO,
            center_b: Pos::ZERO,
            r_a: Vec3::ZERO,
            r_b: Vec3::ZERO,
            mass: 0.0,
        }
    }
}

/// Small rotation of a delta quaternion about an axis.
#[inline(always)]
fn delta_angle(dq: Quat, axis: Vec3) -> f32 {
    let s: f32 = if dq.s < 0.0 { -2.0 } else { 2.0 };
    s * dot(dq.v, axis)
}

impl World {
    // MARK: Probes

    fn make_angle_probe(
        &self,
        body: i32,
        hinge_frame: Quat,
        reference: JointID,
    ) -> AngleProbe {
        let mut probe = AngleProbe::default();
        probe.body = body;
        probe.hinge_frame = hinge_frame;
        probe.rotation0 = self.body_transform_quick(body).q;

        let index = reference.index1 - 1;
        if index >= 0 && (index as usize) < self.joints.len() {
            let joint = &self.joints[index as usize];
            if joint.joint_id != b3::NULL_INDEX
                && joint.generation == reference.generation
                && joint.type_ == JointType::RevoluteJoint
                && (joint.edges[0].body_id == body || joint.edges[1].body_id == body)
            {
                let sim = self.read_joint_sim(index);
                probe.uses_joint = true;
                probe.joint_index = index;
                probe.joint_generation = joint.generation;
                probe.joint_body_a = sim.body_id_a;
                probe.joint_body_b = sim.body_id_b;
                probe.joint_frame_a = sim.local_frame_a.q;
                probe.joint_frame_b = sim.local_frame_b.q;
                probe.sign = if sim.body_id_b == body { 1.0 } else { -1.0 };
            }
        }
        probe.twist = self.measure_angle(&probe);
        probe.angle = probe.twist;
        probe
    }

    fn measure_angle(&self, probe: &AngleProbe) -> f32 {
        if probe.uses_joint {
            let q_a = mul_quat(self.body_transform_quick(probe.joint_body_a).q, probe.joint_frame_a);
            let q_b = mul_quat(self.body_transform_quick(probe.joint_body_b).q, probe.joint_frame_b);
            return probe.sign * get_twist_angle(inv_mul_quat(q_a, q_b));
        }
        // Rotation since creation, expressed in the hinge frame.
        let relative = inv_mul_quat(probe.rotation0, self.body_transform_quick(probe.body).q);
        let in_hinge = inv_mul_quat(probe.hinge_frame, mul_quat(relative, probe.hinge_frame));
        get_twist_angle(in_hinge)
    }

    /// Accumulate the change since the last step without wrapping.
    fn update_angle(&self, probe: &mut AngleProbe) {
        if probe.uses_joint {
            // `joint_index` is only ever set to a validated non-negative index, matching
            // Swift's unchecked `Int(probe.jointIndex)`.
            let index = probe.joint_index as usize;
            let alive = index < self.joints.len()
                && self.joints[index].joint_id != b3::NULL_INDEX
                && self.joints[index].generation == probe.joint_generation;
            if !alive {
                // The reference joint is gone: continue from the body rotation.
                probe.uses_joint = false;
                probe.rotation0 = self.body_transform_quick(probe.body).q;
                probe.twist = 0.0;
                return;
            }
        }
        let twist = self.measure_angle(probe);
        probe.angle += unwind_angle(twist - probe.twist);
        probe.twist = twist;
    }

    fn make_translation_probe(
        &self,
        body: i32,
        axis_local: Vec3,
        reference: JointID,
    ) -> TranslationProbe {
        let mut probe = TranslationProbe::default();
        probe.body = body;
        let xf = self.body_transform_quick(body);
        probe.center0 = transform_world_point(
            xf,
            self.body_sim(&self.bodies[body as usize]).local_center,
        );
        probe.axis0 = rotate_vector(xf.q, axis_local);

        let index = reference.index1 - 1;
        if index >= 0 && (index as usize) < self.joints.len() {
            let joint = &self.joints[index as usize];
            if joint.joint_id != b3::NULL_INDEX
                && joint.generation == reference.generation
                && joint.type_ == JointType::PrismaticJoint
                && (joint.edges[0].body_id == body || joint.edges[1].body_id == body)
            {
                let sim = self.read_joint_sim(index);
                probe.uses_joint = true;
                probe.joint_index = index;
                probe.joint_generation = joint.generation;
                probe.joint_body_a = sim.body_id_a;
                probe.joint_body_b = sim.body_id_b;
                probe.joint_frame_a = sim.local_frame_a;
                probe.joint_anchor_b = sim.local_frame_b.p;
                probe.sign = if sim.body_id_b == body { 1.0 } else { -1.0 };
            }
        }
        probe.value = self.measure_translation(&probe);
        probe
    }

    fn measure_translation(&self, probe: &TranslationProbe) -> f32 {
        if probe.uses_joint {
            let xf_a = self.body_transform_quick(probe.joint_body_a);
            let xf_b = self.body_transform_quick(probe.joint_body_b);
            let axis = rotate_vector(
                xf_a.q,
                rotate_vector(probe.joint_frame_a.q, Vec3::AXIS_X),
            );
            let p_a = transform_world_point(xf_a, probe.joint_frame_a.p);
            let p_b = transform_world_point(xf_b, probe.joint_anchor_b);
            return probe.sign * dot(sub_pos(p_b, p_a), axis);
        }
        let xf = self.body_transform_quick(probe.body);
        let center = transform_world_point(
            xf,
            self.body_sim(&self.bodies[probe.body as usize]).local_center,
        );
        dot(sub_pos(center, probe.center0), probe.axis0)
    }

    fn update_translation(&self, probe: &mut TranslationProbe) {
        if probe.uses_joint {
            // `joint_index` is only ever set to a validated non-negative index, matching
            // Swift's unchecked `Int(probe.jointIndex)`.
            let index = probe.joint_index as usize;
            let alive = index < self.joints.len()
                && self.joints[index].joint_id != b3::NULL_INDEX
                && self.joints[index].generation == probe.joint_generation;
            if !alive {
                let value = probe.value;
                probe.uses_joint = false;
                let xf = self.body_transform_quick(probe.body);
                probe.center0 = transform_world_point(
                    xf,
                    self.body_sim(&self.bodies[probe.body as usize]).local_center,
                );
                probe.axis0 = rotate_vector(xf.q, probe.axis0);
                probe.value = value;
                return;
            }
            probe.value = self.measure_translation(probe);
        } else {
            probe.value = self.measure_translation(probe);
        }
    }

    // MARK: Creation

    fn valid_joint_bodies(&self, def: &JointDef) -> bool {
        self.is_valid_body(def.body_id_a)
            && self.is_valid_body(def.body_id_b)
            && def.body_id_a != def.body_id_b
    }

    /// Swift's `@discardableResult`; the null `JointID()` failure result is `JointID::NULL`.
    pub fn create_gear_joint(&mut self, def: &GearJointDef) -> JointID {
        if !self.valid_joint_bodies(&def.base) || !def.ratio.is_finite() {
            return JointID::NULL;
        }
        let pair = self.create_joint(&def.base, JointType::GearJoint);
        let mut sim = pair.sim;
        let mut gear = GearJointSim::default();
        gear.ratio = def.ratio;
        gear.probe_a =
            self.make_angle_probe(sim.body_id_a, sim.local_frame_a.q, def.reference_joint_a);
        gear.probe_b =
            self.make_angle_probe(sim.body_id_b, sim.local_frame_b.q, def.reference_joint_b);
        gear.offset = gear.probe_a.angle + gear.ratio * gear.probe_b.angle;
        sim.set_gear(gear);
        self.write_joint_sim(pair.joint_id, sim);
        JointID::new(
            pair.joint_id + 1,
            self.world_id,
            self.joints[pair.joint_id as usize].generation,
        )
    }

    pub fn create_rack_and_pinion_joint(&mut self, def: &RackAndPinionJointDef) -> JointID {
        if !self.valid_joint_bodies(&def.base) || !def.ratio.is_finite() {
            return JointID::NULL;
        }
        let pair = self.create_joint(&def.base, JointType::RackAndPinionJoint);
        let mut sim = pair.sim;
        let mut joint = RackAndPinionJointSim::default();
        joint.ratio = def.ratio;
        joint.pinion =
            self.make_angle_probe(sim.body_id_a, sim.local_frame_a.q, def.reference_pinion_joint);
        joint.rack = self.make_translation_probe(
            sim.body_id_b,
            rotate_vector(sim.local_frame_b.q, Vec3::AXIS_X),
            def.reference_rack_joint,
        );
        joint.offset = joint.rack.value - joint.ratio * joint.pinion.angle;
        sim.set_rack_and_pinion(joint);
        self.write_joint_sim(pair.joint_id, sim);
        JointID::new(
            pair.joint_id + 1,
            self.world_id,
            self.joints[pair.joint_id as usize].generation,
        )
    }

    pub fn create_pulley_joint(&mut self, def: &PulleyJointDef) -> JointID {
        if !self.valid_joint_bodies(&def.base)
            || !(def.ratio > 0.0)
            || !is_valid_position(def.ground_anchor_a)
            || !is_valid_position(def.ground_anchor_b)
        {
            return JointID::NULL;
        }
        let pair = self.create_joint(&def.base, JointType::PulleyJoint);
        let mut sim = pair.sim;
        let mut joint = PulleyJointSim::default();
        joint.ground_anchor_a = def.ground_anchor_a;
        joint.ground_anchor_b = def.ground_anchor_b;
        joint.ratio = def.ratio;
        let p_a = transform_world_point(
            self.body_transform_quick(sim.body_id_a),
            sim.local_frame_a.p,
        );
        let p_b = transform_world_point(
            self.body_transform_quick(sim.body_id_b),
            sim.local_frame_b.p,
        );
        let current = distance_pos(p_a, def.ground_anchor_a)
            + def.ratio * distance_pos(p_b, def.ground_anchor_b);
        joint.min_length = if def.min_length > 0.0 { def.min_length } else { current };
        joint.max_length = if def.max_length > 0.0 {
            max_float(def.max_length, joint.min_length)
        } else {
            max_float(current, joint.min_length)
        };
        sim.set_pulley(joint);
        self.write_joint_sim(pair.joint_id, sim);
        JointID::new(
            pair.joint_id + 1,
            self.world_id,
            self.joints[pair.joint_id as usize].generation,
        )
    }

    // MARK: Shared solver helpers

    #[inline(always)]
    fn prepare_bodies(&self, base: &mut JointSim) -> (BodySim, BodySim, i32, i32) {
        let body_a = self.joint_prepare_body(base.body_id_a);
        let body_b = self.joint_prepare_body(base.body_id_b);
        let sim_a = self.joint_prepare_sim(&body_a);
        let sim_b = self.joint_prepare_sim(&body_b);
        base.inv_mass_a = sim_a.inv_mass;
        base.inv_mass_b = sim_b.inv_mass;
        base.inv_ia = sim_a.inv_inertia_world;
        base.inv_ib = sim_b.inv_inertia_world;
        let index_a = if body_a.set_index == SetType::AwakeSet.raw_value() {
            body_a.local_index
        } else {
            b3::NULL_INDEX
        };
        let index_b = if body_b.set_index == SetType::AwakeSet.raw_value() {
            body_b.local_index
        } else {
            b3::NULL_INDEX
        };
        (sim_a, sim_b, index_a, index_b)
    }

    #[inline(always)]
    fn soft_terms(
        &self,
        base: &JointSim,
        c: f32,
        use_bias: bool,
    ) -> (f32, f32, f32) {
        if !use_bias {
            return (0.0, 1.0, 0.0);
        }
        let soft = base.constraint_softness;
        (soft.bias_rate * c, soft.mass_scale, soft.impulse_scale)
    }

    // MARK: Gear

    pub(crate) fn prepare_gear_joint(&self, base: &mut JointSim, context: &StepContext) {
        let (sim_a, sim_b, index_a, index_b) = self.prepare_bodies(base);
        let mut joint = base.gear();
        joint.index_a = index_a;
        joint.index_b = index_b;
        self.update_angle(&mut joint.probe_a);
        self.update_angle(&mut joint.probe_b);
        joint.axis_a = rotate_vector(
            sim_a.transform.q,
            rotate_vector(base.local_frame_a.q, Vec3::AXIS_Z),
        );
        joint.axis_b = rotate_vector(
            sim_b.transform.q,
            rotate_vector(base.local_frame_b.q, Vec3::AXIS_Z),
        );
        let k = dot(joint.axis_a, mul_mv(base.inv_ia, joint.axis_a))
            + joint.ratio * joint.ratio * dot(joint.axis_b, mul_mv(base.inv_ib, joint.axis_b));
        joint.mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        joint.position = joint.probe_a.angle + joint.ratio * joint.probe_b.angle - joint.offset;
        if !context.enable_warm_starting {
            joint.impulse = 0.0;
        }
        base.set_gear(joint);
    }

    pub(crate) fn warm_start_gear_joint(&self, base: &mut JointSim, states: *mut BodyState) {
        let joint = base.gear();
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);
        state_a.angular_velocity =
            mul_add(state_a.angular_velocity, joint.impulse, mul_mv(base.inv_ia, joint.axis_a));
        state_b.angular_velocity = mul_add(
            state_b.angular_velocity,
            joint.ratio * joint.impulse,
            mul_mv(base.inv_ib, joint.axis_b),
        );
        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
    }

    pub(crate) fn solve_gear_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        use_bias: bool,
        states: *mut BodyState,
    ) {
        let mut joint = base.gear();
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);
        let c = joint.position
            + delta_angle(state_a.delta_rotation, joint.axis_a)
            + joint.ratio * delta_angle(state_b.delta_rotation, joint.axis_b);
        let cdot = dot(state_a.angular_velocity, joint.axis_a)
            + joint.ratio * dot(state_b.angular_velocity, joint.axis_b);
        let (bias, mass_scale, impulse_scale) = self.soft_terms(base, c, use_bias);
        let impulse = -mass_scale * joint.mass * (cdot + bias) - impulse_scale * joint.impulse;
        joint.impulse += impulse;
        state_a.angular_velocity =
            mul_add(state_a.angular_velocity, impulse, mul_mv(base.inv_ia, joint.axis_a));
        state_b.angular_velocity = mul_add(
            state_b.angular_velocity,
            joint.ratio * impulse,
            mul_mv(base.inv_ib, joint.axis_b),
        );
        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
        base.set_gear(joint);
    }

    // MARK: Rack and pinion

    pub(crate) fn prepare_rack_and_pinion_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
    ) {
        let (sim_a, sim_b, index_a, index_b) = self.prepare_bodies(base);
        let mut joint = base.rack_and_pinion();
        joint.index_a = index_a;
        joint.index_b = index_b;
        self.update_angle(&mut joint.pinion);
        self.update_translation(&mut joint.rack);
        joint.axis_a = rotate_vector(
            sim_a.transform.q,
            rotate_vector(base.local_frame_a.q, Vec3::AXIS_Z),
        );
        joint.axis_b = rotate_vector(
            sim_b.transform.q,
            rotate_vector(base.local_frame_b.q, Vec3::AXIS_X),
        );
        let k = base.inv_mass_b
            + joint.ratio
                * joint.ratio
                * dot(joint.axis_a, mul_mv(base.inv_ia, joint.axis_a));
        joint.mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        joint.position = joint.rack.value - joint.ratio * joint.pinion.angle - joint.offset;
        if !context.enable_warm_starting {
            joint.impulse = 0.0;
        }
        base.set_rack_and_pinion(joint);
    }

    pub(crate) fn warm_start_rack_and_pinion_joint(
        &self,
        base: &mut JointSim,
        states: *mut BodyState,
    ) {
        let joint = base.rack_and_pinion();
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);
        state_a.angular_velocity = mul_add(
            state_a.angular_velocity,
            -joint.ratio * joint.impulse,
            mul_mv(base.inv_ia, joint.axis_a),
        );
        state_b.linear_velocity = mul_add(
            state_b.linear_velocity,
            base.inv_mass_b * joint.impulse,
            joint.axis_b,
        );
        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
    }

    pub(crate) fn solve_rack_and_pinion_joint(
        &self,
        base: &mut JointSim,
        _context: &StepContext,
        use_bias: bool,
        states: *mut BodyState,
    ) {
        let mut joint = base.rack_and_pinion();
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);
        let c = joint.position
            + dot(state_b.delta_position, joint.axis_b)
            - joint.ratio * delta_angle(state_a.delta_rotation, joint.axis_a);
        let cdot = dot(state_b.linear_velocity, joint.axis_b)
            - joint.ratio * dot(state_a.angular_velocity, joint.axis_a);
        let (bias, mass_scale, impulse_scale) = self.soft_terms(base, c, use_bias);
        let impulse = -mass_scale * joint.mass * (cdot + bias) - impulse_scale * joint.impulse;
        joint.impulse += impulse;
        state_a.angular_velocity = mul_add(
            state_a.angular_velocity,
            -joint.ratio * impulse,
            mul_mv(base.inv_ia, joint.axis_a),
        );
        state_b.linear_velocity = mul_add(
            state_b.linear_velocity,
            base.inv_mass_b * impulse,
            joint.axis_b,
        );
        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
        base.set_rack_and_pinion(joint);
    }

    // MARK: Pulley

    pub(crate) fn prepare_pulley_joint(&self, base: &mut JointSim, context: &StepContext) {
        let (sim_a, sim_b, index_a, index_b) = self.prepare_bodies(base);
        let mut joint = base.pulley();
        joint.index_a = index_a;
        joint.index_b = index_b;
        joint.center_a = sim_a.center;
        joint.center_b = sim_b.center;
        joint.r_a = rotate_vector(
            sim_a.transform.q,
            base.local_frame_a.p - sim_a.local_center,
        );
        joint.r_b = rotate_vector(
            sim_b.transform.q,
            base.local_frame_b.p - sim_b.local_center,
        );

        let u_a = self.pulley_direction(offset_pos(joint.center_a, joint.r_a), joint.ground_anchor_a);
        let u_b = self.pulley_direction(offset_pos(joint.center_b, joint.r_b), joint.ground_anchor_b);
        let ru_a = cross(joint.r_a, u_a);
        let ru_b = cross(joint.r_b, u_b);
        let k = base.inv_mass_a
            + dot(ru_a, mul_mv(base.inv_ia, ru_a))
            + joint.ratio
                * joint.ratio
                * (base.inv_mass_b + dot(ru_b, mul_mv(base.inv_ib, ru_b)));
        joint.mass = if k > 0.0 { 1.0 / k } else { 0.0 };
        // Only the accumulators of the active mode may be warm started.
        if joint.max_length - joint.min_length < b3::linear_slop() {
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
        } else {
            joint.impulse = 0.0;
        }
        if !context.enable_warm_starting {
            joint.impulse = 0.0;
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
        }
        base.set_pulley(joint);
    }

    fn pulley_direction(&self, anchor: Pos, ground: Pos) -> Vec3 {
        let (len, u) = get_length_and_normalize(sub_pos(anchor, ground));
        if len > b3::linear_slop() { u } else { Vec3::ZERO }
    }

    /// Current anchors, directions, and rope length. Mirrors Swift's labeled tuple
    /// `(rA, rB, uA, uB, length)`.
    #[inline(always)]
    fn pulley_geometry(
        &self,
        joint: PulleyJointSim,
        state_a: BodyState,
        state_b: BodyState,
    ) -> (Vec3, Vec3, Vec3, Vec3, f32) {
        let r_a = rotate_vector(state_a.delta_rotation, joint.r_a);
        let r_b = rotate_vector(state_b.delta_rotation, joint.r_b);
        let p_a = offset_pos(offset_pos(joint.center_a, state_a.delta_position), r_a);
        let p_b = offset_pos(offset_pos(joint.center_b, state_b.delta_position), r_b);
        let (mut len_a, mut u_a) = get_length_and_normalize(sub_pos(p_a, joint.ground_anchor_a));
        let (mut len_b, mut u_b) = get_length_and_normalize(sub_pos(p_b, joint.ground_anchor_b));
        if len_a <= b3::linear_slop() {
            u_a = Vec3::ZERO;
        }
        if len_b <= b3::linear_slop() {
            u_b = Vec3::ZERO;
        }
        (r_a, r_b, u_a, u_b, len_a + joint.ratio * len_b)
    }

    /// Swift's lazily-invoked `cdot()` closure; re-evaluated at every call site exactly as
    /// Swift's closure reads the current velocities.
    #[inline(always)]
    fn pulley_cdot(
        joint: &PulleyJointSim,
        r_a: Vec3,
        r_b: Vec3,
        u_a: Vec3,
        u_b: Vec3,
        state_a: &BodyState,
        state_b: &BodyState,
    ) -> f32 {
        dot(u_a, state_a.linear_velocity + cross(state_a.angular_velocity, r_a))
            + joint.ratio * dot(u_b, state_b.linear_velocity + cross(state_b.angular_velocity, r_b))
    }

    fn apply_pulley_impulse(
        &self,
        joint: PulleyJointSim,
        base: &JointSim,
        g: (Vec3, Vec3, Vec3, Vec3, f32),
        impulse: f32,
        state_a: &mut BodyState,
        state_b: &mut BodyState,
    ) {
        let (r_a, r_b, u_a, u_b, _length) = g;
        let p_a = impulse * u_a;
        let p_b = (joint.ratio * impulse) * u_b;
        state_a.linear_velocity = mul_add(state_a.linear_velocity, base.inv_mass_a, p_a);
        state_a.angular_velocity =
            state_a.angular_velocity + mul_mv(base.inv_ia, cross(r_a, p_a));
        state_b.linear_velocity = mul_add(state_b.linear_velocity, base.inv_mass_b, p_b);
        state_b.angular_velocity =
            state_b.angular_velocity + mul_mv(base.inv_ib, cross(r_b, p_b));
    }

    pub(crate) fn warm_start_pulley_joint(&self, base: &mut JointSim, states: *mut BodyState) {
        let joint = base.pulley();
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);
        let g = self.pulley_geometry(joint, state_a, state_b);
        self.apply_pulley_impulse(
            joint,
            base,
            g,
            joint.impulse + joint.lower_impulse - joint.upper_impulse,
            &mut state_a,
            &mut state_b,
        );
        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
    }

    pub(crate) fn solve_pulley_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        use_bias: bool,
        states: *mut BodyState,
    ) {
        let mut joint = base.pulley();
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);
        let g = self.pulley_geometry(joint, state_a, state_b);
        let (g_r_a, g_r_b, g_u_a, g_u_b, g_length) = g;

        if joint.max_length - joint.min_length < b3::linear_slop() {
            // Taut rope: equality.
            let c = g_length - joint.min_length;
            let (bias, mass_scale, impulse_scale) = self.soft_terms(base, c, use_bias);
            let cdot = Self::pulley_cdot(
                &joint, g_r_a, g_r_b, g_u_a, g_u_b, &state_a, &state_b,
            );
            let impulse = -mass_scale * joint.mass * (cdot + bias) - impulse_scale * joint.impulse;
            joint.impulse += impulse;
            self.apply_pulley_impulse(joint, base, g, impulse, &mut state_a, &mut state_b);
        } else {
            // Lower bound: lengthA + ratio * lengthB >= minLength.
            {
                let c = g_length - joint.min_length;
                let mut bias: f32 = 0.0;
                let mut mass_scale: f32 = 1.0;
                let mut impulse_scale: f32 = 0.0;
                if c > 0.0 {
                    bias = c * context.inv_h;
                } else if use_bias {
                    (bias, mass_scale, impulse_scale) = self.soft_terms(base, c, true);
                }
                let cdot = Self::pulley_cdot(
                    &joint, g_r_a, g_r_b, g_u_a, g_u_b, &state_a, &state_b,
                );
                let mut impulse =
                    -mass_scale * joint.mass * (cdot + bias) - impulse_scale * joint.lower_impulse;
                let new_impulse = max_float(0.0, joint.lower_impulse + impulse);
                impulse = new_impulse - joint.lower_impulse;
                joint.lower_impulse = new_impulse;
                self.apply_pulley_impulse(joint, base, g, impulse, &mut state_a, &mut state_b);
            }
            // Upper bound: the rope cannot stretch.
            {
                let c = joint.max_length - g_length;
                let mut bias: f32 = 0.0;
                let mut mass_scale: f32 = 1.0;
                let mut impulse_scale: f32 = 0.0;
                if c > 0.0 {
                    bias = c * context.inv_h;
                } else if use_bias {
                    (bias, mass_scale, impulse_scale) = self.soft_terms(base, c, true);
                }
                let cdot = Self::pulley_cdot(
                    &joint, g_r_a, g_r_b, g_u_a, g_u_b, &state_a, &state_b,
                );
                let mut impulse =
                    -mass_scale * joint.mass * (-cdot + bias) - impulse_scale * joint.upper_impulse;
                let new_impulse = max_float(0.0, joint.upper_impulse + impulse);
                impulse = new_impulse - joint.upper_impulse;
                joint.upper_impulse = new_impulse;
                self.apply_pulley_impulse(joint, base, g, -impulse, &mut state_a, &mut state_b);
            }
        }

        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
        base.set_pulley(joint);
    }

    // MARK: Reactions

    pub(crate) fn coupling_joint_force(&self, sim: JointSim) -> Vec3 {
        match sim.type_ {
            JointType::SixDOFJoint => self.inv_h * self.six_dof_impulses(sim).linear,
            JointType::PathJoint => self.inv_h * self.path_joint_impulses(sim).linear,
            JointType::RackAndPinionJoint => {
                let xf_b = self.body_transform_quick(sim.body_id_b);
                let axis = rotate_vector(xf_b.q, rotate_vector(sim.local_frame_b.q, Vec3::AXIS_X));
                (self.inv_h * sim.rack_and_pinion().impulse) * axis
            }
            JointType::PulleyJoint => {
                let joint = sim.pulley();
                let p_a = transform_world_point(
                    self.body_transform_quick(sim.body_id_a),
                    sim.local_frame_a.p,
                );
                let u = self.pulley_direction(p_a, joint.ground_anchor_a);
                (self.inv_h * (joint.impulse + joint.lower_impulse - joint.upper_impulse)) * u
            }
            _ => Vec3::ZERO,
        }
    }

    pub(crate) fn coupling_joint_torque(&self, sim: JointSim) -> Vec3 {
        match sim.type_ {
            JointType::SixDOFJoint => self.inv_h * self.six_dof_impulses(sim).angular,
            JointType::PathJoint => self.inv_h * self.path_joint_impulses(sim).angular,
            JointType::GearJoint => {
                let xf_a = self.body_transform_quick(sim.body_id_a);
                let axis = rotate_vector(xf_a.q, rotate_vector(sim.local_frame_a.q, Vec3::AXIS_Z));
                (self.inv_h * sim.gear().impulse) * axis
            }
            JointType::RackAndPinionJoint => {
                let xf_a = self.body_transform_quick(sim.body_id_a);
                let axis = rotate_vector(xf_a.q, rotate_vector(sim.local_frame_a.q, Vec3::AXIS_Z));
                (-self.inv_h * sim.rack_and_pinion().ratio * sim.rack_and_pinion().impulse) * axis
            }
            _ => Vec3::ZERO,
        }
    }

    /// Linear and angular impulse magnitudes for joint events.
    pub(crate) fn coupling_joint_impulses(&self, sim: JointSim) -> (f32, f32) {
        match sim.type_ {
            JointType::PathJoint => {
                let impulses = self.path_joint_impulses(sim);
                (length(impulses.linear), length(impulses.angular))
            }
            JointType::SixDOFJoint => {
                let impulses = self.six_dof_impulses(sim);
                (length(impulses.linear), length(impulses.angular))
            }
            JointType::GearJoint => (0.0, abs_float(sim.gear().impulse)),
            JointType::RackAndPinionJoint => {
                let joint = sim.rack_and_pinion();
                (abs_float(joint.impulse), abs_float(joint.ratio * joint.impulse))
            }
            JointType::PulleyJoint => {
                let joint = sim.pulley();
                (
                    abs_float(joint.impulse + joint.lower_impulse - joint.upper_impulse),
                    0.0,
                )
            }
            _ => (0.0, 0.0),
        }
    }

    // MARK: Debug draw

    pub(crate) fn draw_coupling_joint(
        &self,
        sim: JointSim,
        xf_a: WorldTransform,
        xf_b: WorldTransform,
        p_a: Pos,
        p_b: Pos,
        scale: f32,
        draw_segment: &dyn Fn(Pos, Pos, HexColor),
        draw_point: Option<&dyn Fn(Pos, f32, HexColor)>,
    ) {
        match sim.type_ {
            JointType::GearJoint | JointType::RackAndPinionJoint => {
                let length = 0.25 * scale;
                let axis_a =
                    rotate_vector(xf_a.q, rotate_vector(sim.local_frame_a.q, Vec3::AXIS_Z));
                let axis_b = if sim.type_ == JointType::GearJoint {
                    rotate_vector(xf_b.q, rotate_vector(sim.local_frame_b.q, Vec3::AXIS_Z))
                } else {
                    rotate_vector(xf_b.q, rotate_vector(sim.local_frame_b.q, Vec3::AXIS_X))
                };
                draw_segment(
                    pos_mul_sub(p_a, length, axis_a),
                    pos_mul_add(p_a, length, axis_a),
                    debug_colors::ORANGE,
                );
                draw_segment(
                    pos_mul_sub(p_b, length, axis_b),
                    pos_mul_add(p_b, length, axis_b),
                    debug_colors::CYAN,
                );
                draw_segment(p_a, p_b, debug_colors::GRAY);
            }
            JointType::SixDOFJoint => {
                let length = 0.15 * scale;
                let frame_a = mul_world_transforms(xf_a, sim.local_frame_a);
                let frame_b = mul_world_transforms(xf_b, sim.local_frame_b);
                let axes = [
                    (Vec3::AXIS_X, debug_colors::RED),
                    (Vec3::AXIS_Y, debug_colors::GREEN),
                    (Vec3::AXIS_Z, debug_colors::BLUE),
                ];
                for (axis, color) in axes {
                    draw_segment(
                        frame_a.p,
                        offset_pos(frame_a.p, length * rotate_vector(frame_a.q, axis)),
                        color,
                    );
                    draw_segment(
                        frame_b.p,
                        offset_pos(frame_b.p, 0.6 * length * rotate_vector(frame_b.q, axis)),
                        color,
                    );
                }
                draw_segment(p_a, p_b, debug_colors::GRAY);
            }
            JointType::PathJoint => {
                self.draw_path_joint(sim, xf_a, p_b, scale, draw_segment, draw_point);
            }
            JointType::PulleyJoint => {
                let joint = sim.pulley();
                draw_segment(p_a, joint.ground_anchor_a, debug_colors::LIGHT_BLUE);
                draw_segment(
                    joint.ground_anchor_a,
                    joint.ground_anchor_b,
                    debug_colors::LIGHT_BLUE,
                );
                draw_segment(joint.ground_anchor_b, p_b, debug_colors::LIGHT_BLUE);
                if let Some(draw_point) = draw_point {
                    draw_point(joint.ground_anchor_a, 6.0, debug_colors::WHITE);
                    draw_point(joint.ground_anchor_b, 6.0, debug_colors::WHITE);
                }
            }
            _ => {}
        }
    }

    // MARK: Accessors

    fn coupling_sim(&self, joint_id: JointID, type_: JointType) -> Option<(i32, JointSim)> {
        let index = joint_id.index1 - 1;
        if joint_id.world0 != self.world_id || index < 0 || (index as usize) >= self.joints.len() {
            return None;
        }
        let joint = &self.joints[index as usize];
        if joint.joint_id == b3::NULL_INDEX
            || joint.generation != joint_id.generation
            || joint.type_ != type_
        {
            return None;
        }
        Some((index, self.read_joint_sim(index)))
    }

    pub fn gear_joint_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.coupling_sim(joint_id, JointType::GearJoint)
            .map(|(_index, sim)| sim.gear().ratio)
    }

    /// Change the gear ratio. The current angles become the new reference.
    /// Swift's `@discardableResult`; callers may ignore the `bool`.
    pub fn set_gear_joint_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        if !ratio.is_finite() {
            return false;
        }
        let Some((_index, mut sim)) = self.coupling_sim(joint_id, JointType::GearJoint) else {
            return false;
        };
        let mut gear = sim.gear();
        gear.ratio = ratio;
        gear.offset = gear.probe_a.angle + ratio * gear.probe_b.angle;
        gear.impulse = 0.0;
        sim.set_gear(gear);
        self.write_joint_sim(joint_id.index1 - 1, sim);
        true
    }

    pub fn rack_and_pinion_joint_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.coupling_sim(joint_id, JointType::RackAndPinionJoint)
            .map(|(_index, sim)| sim.rack_and_pinion().ratio)
    }

    /// Swift's `@discardableResult`; callers may ignore the `bool`.
    pub fn set_rack_and_pinion_joint_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        if !ratio.is_finite() {
            return false;
        }
        let Some((_index, mut sim)) =
            self.coupling_sim(joint_id, JointType::RackAndPinionJoint)
        else {
            return false;
        };
        let mut joint = sim.rack_and_pinion();
        joint.ratio = ratio;
        joint.offset = joint.rack.value - ratio * joint.pinion.angle;
        joint.impulse = 0.0;
        sim.set_rack_and_pinion(joint);
        self.write_joint_sim(joint_id.index1 - 1, sim);
        true
    }

    pub fn pulley_joint_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.coupling_sim(joint_id, JointType::PulleyJoint)
            .map(|(_index, sim)| sim.pulley().ratio)
    }

    pub fn pulley_joint_ground_anchors(&self, joint_id: JointID) -> Option<(Pos, Pos)> {
        let (_index, sim) = self.coupling_sim(joint_id, JointType::PulleyJoint)?;
        let joint = sim.pulley();
        Some((joint.ground_anchor_a, joint.ground_anchor_b))
    }

    /// Current rope segment lengths from the ground anchors.
    pub fn pulley_joint_current_lengths(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let (_index, sim) = self.coupling_sim(joint_id, JointType::PulleyJoint)?;
        let p_a = transform_world_point(
            self.body_transform_quick(sim.body_id_a),
            sim.local_frame_a.p,
        );
        let p_b = transform_world_point(
            self.body_transform_quick(sim.body_id_b),
            sim.local_frame_b.p,
        );
        let joint = sim.pulley();
        Some((
            distance_pos(p_a, joint.ground_anchor_a),
            distance_pos(p_b, joint.ground_anchor_b),
        ))
    }

    pub fn pulley_joint_length_range(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let (_index, sim) = self.coupling_sim(joint_id, JointType::PulleyJoint)?;
        let joint = sim.pulley();
        Some((joint.min_length, joint.max_length))
    }

    /// Allow slack (min < max) or change the rope length.
    /// Swift's `@discardableResult`; callers may ignore the `bool`.
    pub fn set_pulley_joint_length_range(
        &mut self,
        joint_id: JointID,
        min_length: f32,
        max_length: f32,
    ) -> bool {
        if !(min_length > 0.0) || !(max_length >= min_length) {
            return false;
        }
        let Some((_index, mut sim)) = self.coupling_sim(joint_id, JointType::PulleyJoint) else {
            return false;
        };
        let mut joint = sim.pulley();
        joint.min_length = min_length;
        joint.max_length = max_length;
        sim.set_pulley(joint);
        self.write_joint_sim(joint_id.index1 - 1, sim);
        self.wake_joint_bodies(joint_id);
        true
    }
}
