//! Joint model, creation, and destruction.
//!
//! Translated from `Sources/Natsel/Simulation/Joint.swift`.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::math::functions::{
    abs_float, atan2_approx, clamp_float, cross, dot, get_twist_angle, inv_mul_quat, length,
    make_matrix_from_quat, max_float, max_int, min_float, mul_add, mul_quat, normalize,
    rotate_vector, sub_pos, transform_point, transform_world_point,
};
use crate::math::types::{Quat, Transform, Vec2, Vec3, WorldTransform};
use crate::simulation::solver_integration::Softness;
use crate::simulation::world_runtime::{Joint, JointSim, SetType, World, remove_swap_array};
use crate::types::events::JointEvent;
use crate::types::ids::{BodyID, JointID};
use crate::types::joint_defs::{DistanceJointDef, FilterJointDef, JointDef, JointType};

/// Distance joint solver data.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct DistanceJointSim {
    pub length: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub lower_spring_force: f32,
    pub upper_spring_force: f32,
    pub min_length: f32,
    pub max_length: f32,

    pub max_motor_force: f32,
    pub motor_speed: f32,

    pub impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub motor_impulse: f32,

    pub index_a: i32,
    pub index_b: i32,
    pub anchor_a: Vec3,
    pub anchor_b: Vec3,
    pub delta_center: Vec3,
    pub distance_softness: Softness,
    pub axial_mass: f32,

    pub enable_spring: bool,
    pub enable_limit: bool,
    pub enable_motor: bool,
}

impl Default for DistanceJointSim {
    /// Mirrors Swift's `init() {}` with the declared member defaults; the index fields start at
    /// `B3.nullIndex`, so the derived all-zeroes default does not apply.
    #[inline]
    fn default() -> Self {
        Self {
            length: 0.0,
            hertz: 0.0,
            damping_ratio: 0.0,
            lower_spring_force: 0.0,
            upper_spring_force: 0.0,
            min_length: 0.0,
            max_length: 0.0,
            max_motor_force: 0.0,
            motor_speed: 0.0,
            impulse: 0.0,
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            motor_impulse: 0.0,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            anchor_a: Vec3::default(),
            anchor_b: Vec3::default(),
            delta_center: Vec3::default(),
            distance_softness: Softness::default(),
            axial_mass: 0.0,
            enable_spring: false,
            enable_limit: false,
            enable_motor: false,
        }
    }
}

// Swift's `extension JointSim { var distanceJoint }` — an alias for the payload accessor pair
// that `world_runtime.rs` provides.
impl JointSim {
    #[inline(always)]
    pub(crate) fn distance_joint(&self) -> DistanceJointSim {
        self.distance()
    }

    #[inline(always)]
    pub(crate) fn set_distance_joint(&mut self, value: DistanceJointSim) {
        self.set_distance(value);
    }
}

/// Result of creating a joint.
#[derive(Clone, Debug)]
pub struct JointPair {
    pub joint_id: i32,
    pub sim: JointSim,
}

impl Default for JointPair {
    #[inline]
    fn default() -> Self {
        Self {
            joint_id: b3::NULL_INDEX,
            sim: JointSim::default(),
        }
    }
}

impl World {
    /// Create a filter joint. This disables collision between the attached bodies and keeps them
    /// in the same island.
    pub fn create_filter_joint(&mut self, def: &FilterJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::FilterJoint);
        let joint_id = JointID::new(
            pair.joint_id + 1,
            self.world_id,
            self.joints[pair.joint_id as usize].generation,
        );
        joint_id
    }

    /// Destroy all contacts between two bodies.
    pub(crate) fn destroy_contacts_between_bodies(&mut self, body_id_a: i32, body_id_b: i32) {
        let body_a = self.bodies[body_id_a as usize].clone();
        let body_b = self.bodies[body_id_b as usize].clone();

        // use the smaller of the two contact lists
        let mut contact_key: i32;
        let mut other_body_id: i32;
        if body_a.contact_count < body_b.contact_count {
            contact_key = body_a.head_contact_key;
            other_body_id = body_id_b;
        } else {
            contact_key = body_b.head_contact_key;
            other_body_id = body_id_a;
        }

        // no need to wake bodies when a joint removes collision between them
        let wake_bodies = false;

        while contact_key != b3::NULL_INDEX {
            let contact_id = contact_key >> 1;
            let edge_index = (contact_key & 1) as usize;

            let contact = self.contacts[contact_id as usize];
            contact_key = contact.edges[edge_index].next_key;

            let other_edge_index = edge_index ^ 1;
            if contact.edges[other_edge_index].body_id == other_body_id {
                // Careful, this removes the contact from the current doubly linked list
                self.destroy_contact(contact_id, wake_bodies);
            }
        }
    }

    /// Create a joint and place it in the correct solver set and graph color.
    pub(crate) fn create_joint(&mut self, def: &JointDef, type_: JointType) -> JointPair {
        let body_id_a = def.body_id_a.index1 - 1;
        let body_id_b = def.body_id_b.index1 - 1;

        b3_assert(body_id_a != body_id_b);

        // If the joint prevents collisions, then destroy all contacts between attached bodies.
        if def.collide_connected == false {
            self.destroy_contacts_between_bodies(body_id_a, body_id_b);
        }

        let mut body_a = self.bodies[body_id_a as usize].clone();
        let mut body_b = self.bodies[body_id_b as usize].clone();

        let max_set_index = max_int(body_a.set_index, body_b.set_index);

        // Create joint id and joint
        let joint_id = self.joint_id_pool.alloc();
        if joint_id as usize == self.joints.len() {
            self.joints.push(Joint::default());
        }

        let mut joint = Joint::default();
        joint.joint_id = joint_id;
        joint.user_data = def.user_data;
        // A recycled slot keeps its revision so stale ids stay invalid.
        joint.generation = self.joints[joint_id as usize].generation.wrapping_add(1);
        joint.set_index = b3::NULL_INDEX;
        joint.color_index = b3::NULL_INDEX;
        joint.local_index = b3::NULL_INDEX;
        joint.island_id = b3::NULL_INDEX;
        joint.island_index = b3::NULL_INDEX;
        joint.draw_scale = def.draw_scale;
        joint.type_ = type_;
        joint.collide_connected = def.collide_connected;

        // Doubly linked list on bodyA
        joint.edges[0].body_id = body_id_a;
        joint.edges[0].prev_key = b3::NULL_INDEX;
        joint.edges[0].next_key = body_a.head_joint_key;

        let key_a = joint_id << 1;
        if body_a.head_joint_key != b3::NULL_INDEX {
            let joint_a_id = body_a.head_joint_key >> 1;
            let mut joint_a = self.joints[joint_a_id as usize].clone();
            let edge_index = (body_a.head_joint_key & 1) as usize;
            joint_a.edges[edge_index].prev_key = key_a;
            self.joints[joint_a_id as usize] = joint_a;
        }
        body_a.head_joint_key = key_a;
        body_a.joint_count += 1;

        // Doubly linked list on bodyB
        joint.edges[1].body_id = body_id_b;
        joint.edges[1].prev_key = b3::NULL_INDEX;
        joint.edges[1].next_key = body_b.head_joint_key;

        let key_b = (joint_id << 1) | 1;
        if body_b.head_joint_key != b3::NULL_INDEX {
            let joint_b_id = body_b.head_joint_key >> 1;
            let mut joint_b = self.joints[joint_b_id as usize].clone();
            let edge_index = (body_b.head_joint_key & 1) as usize;
            joint_b.edges[edge_index].prev_key = key_b;
            self.joints[joint_b_id as usize] = joint_b;
        }
        body_b.head_joint_key = key_b;
        body_b.joint_count += 1;

        self.bodies[body_id_a as usize] = body_a.clone();
        self.bodies[body_id_b as usize] = body_b.clone();
        self.joints[joint_id as usize] = joint.clone();

        let mut sim = JointSim::default();
        let mut pair = JointPair { joint_id, sim };

        if body_a.set_index == SetType::DisabledSet.raw_value()
            || body_b.set_index == SetType::DisabledSet.raw_value()
        {
            // if either body is disabled, create in disabled set
            let disabled = SetType::DisabledSet.raw_value() as usize;
            joint.set_index = SetType::DisabledSet.raw_value();
            joint.local_index = self.solver_sets[disabled].joint_sims.len() as i32;

            sim = JointSim::default();
            sim.joint_id = joint_id;
            sim.body_id_a = body_id_a;
            sim.body_id_b = body_id_b;
            self.solver_sets[disabled].joint_sims.push(sim);

            self.joints[joint_id as usize] = joint.clone();
            pair.sim = sim;
        } else if body_a.type_ != crate::types::body_def::BodyType::DynamicBody
            && body_b.type_ != crate::types::body_def::BodyType::DynamicBody
        {
            // joint is not attached to a dynamic body
            let static_set = SetType::StaticSet.raw_value() as usize;
            joint.set_index = SetType::StaticSet.raw_value();
            joint.local_index = self.solver_sets[static_set].joint_sims.len() as i32;

            sim = JointSim::default();
            sim.joint_id = joint_id;
            sim.body_id_a = body_id_a;
            sim.body_id_b = body_id_b;
            self.solver_sets[static_set].joint_sims.push(sim);

            self.joints[joint_id as usize] = joint.clone();
            pair.sim = sim;
        } else if body_a.set_index == SetType::AwakeSet.raw_value()
            || body_b.set_index == SetType::AwakeSet.raw_value()
        {
            // if either body is sleeping, wake it
            if max_set_index >= SetType::FirstSleepingSet.raw_value() {
                self.wake_solver_set(max_set_index);
                body_a = self.bodies[body_id_a as usize].clone();
                body_b = self.bodies[body_id_b as usize].clone();
            }

            joint.set_index = SetType::AwakeSet.raw_value();
            self.joints[joint_id as usize] = joint.clone();

            let index = self.create_joint_in_graph(joint_id);
            joint = self.joints[joint_id as usize].clone();
            let color_index = joint.color_index;
            sim = self.constraint_graph.colors[color_index as usize].joint_sims[index as usize];
            sim.joint_id = joint_id;
            sim.body_id_a = body_id_a;
            sim.body_id_b = body_id_b;
            pair.sim = sim;
        } else {
            // joint connected between sleeping and/or static bodies
            b3_assert(
                body_a.set_index >= SetType::FirstSleepingSet.raw_value()
                    || body_b.set_index >= SetType::FirstSleepingSet.raw_value(),
            );
            b3_assert(
                body_a.set_index != SetType::StaticSet.raw_value()
                    || body_b.set_index != SetType::StaticSet.raw_value(),
            );

            // joint should go into the sleeping set (not static set)
            let mut set_index = max_set_index;

            joint.set_index = set_index;
            joint.local_index =
                self.solver_sets[set_index as usize].joint_sims.len() as i32;

            sim = JointSim::default();
            sim.joint_id = joint_id;
            sim.body_id_a = body_id_a;
            sim.body_id_b = body_id_b;
            self.solver_sets[set_index as usize].joint_sims.push(sim);

            // Publish the local index before any set merge can move this joint.
            self.joints[joint_id as usize] = joint.clone();

            if body_a.set_index != body_b.set_index
                && body_a.set_index >= SetType::FirstSleepingSet.raw_value()
                && body_b.set_index >= SetType::FirstSleepingSet.raw_value()
            {
                // merge sleeping sets
                self.merge_solver_sets(body_a.set_index, body_b.set_index);
                b3_assert(
                    self.bodies[body_id_a as usize].set_index
                        == self.bodies[body_id_b as usize].set_index,
                );

                // fix potentially invalid set index
                set_index = self.bodies[body_id_a as usize].set_index;
                joint = self.joints[joint_id as usize].clone();
                joint.set_index = set_index;
                self.joints[joint_id as usize] = joint.clone();

                // Careful! The joint sim was orphaned by the set merge.
                sim = self.solver_sets[set_index as usize].joint_sims
                    [joint.local_index as usize];
                sim.joint_id = joint_id;
                sim.body_id_a = body_id_a;
                sim.body_id_b = body_id_b;
            }

            b3_assert(joint.set_index == set_index);
            pair.sim = sim;
        }

        sim.local_frame_a = def.local_frame_a;
        sim.local_frame_b = def.local_frame_b;
        sim.type_ = type_;
        sim.constraint_hertz = def.constraint_hertz;
        sim.constraint_damping_ratio = def.constraint_damping_ratio;
        sim.constraint_softness = Softness::default();
        sim.constraint_softness.bias_rate = 0.0;
        sim.constraint_softness.mass_scale = 1.0;
        sim.constraint_softness.impulse_scale = 0.0;
        sim.force_threshold = def.force_threshold;
        sim.torque_threshold = def.torque_threshold;

        b3_assert(sim.joint_id == joint_id);
        b3_assert(sim.body_id_a == body_id_a);
        b3_assert(sim.body_id_b == body_id_b);

        self.write_joint_sim(joint_id, sim);

        joint = self.joints[joint_id as usize].clone();
        if joint.set_index > SetType::DisabledSet.raw_value() {
            // Add edge to island graph
            self.link_joint(joint_id);
        }

        pair.sim = sim;
        pair.joint_id = joint_id;
        pair
    }

    /// Write the sim back to wherever it lives (graph color or solver set).
    pub(crate) fn write_joint_sim(&mut self, joint_id: i32, sim: JointSim) {
        let joint = self.joints[joint_id as usize].clone();
        if joint.set_index == SetType::AwakeSet.raw_value() {
            self.constraint_graph.colors[joint.color_index as usize].joint_sims
                [joint.local_index as usize] = sim;
        } else if joint.set_index != b3::NULL_INDEX {
            self.solver_sets[joint.set_index as usize].joint_sims[joint.local_index as usize] =
                sim;
        }
    }

    /// Read the sim from wherever it lives.
    pub(crate) fn read_joint_sim(&self, joint_id: i32) -> JointSim {
        let joint = &self.joints[joint_id as usize];
        if joint.set_index == SetType::AwakeSet.raw_value() {
            return self.constraint_graph.colors[joint.color_index as usize].joint_sims
                [joint.local_index as usize];
        }
        if joint.set_index != b3::NULL_INDEX {
            return self.solver_sets[joint.set_index as usize].joint_sims
                [joint.local_index as usize];
        }
        JointSim::default()
    }

    // MARK: Distance joint

    pub fn create_distance_joint(&mut self, def: &DistanceJointDef) -> JointID {
        let pair = self.create_joint(&def.base, JointType::DistanceJoint);

        let mut sim = pair.sim;
        let mut joint = DistanceJointSim::default();
        joint.length = max_float(def.length, b3::linear_slop());
        joint.hertz = def.hertz;
        joint.damping_ratio = def.damping_ratio;
        joint.lower_spring_force = def.lower_spring_force;
        joint.upper_spring_force = def.upper_spring_force;
        joint.min_length = max_float(def.min_length, b3::linear_slop());
        joint.max_length = max_float(def.min_length, def.max_length);
        joint.max_motor_force = def.max_motor_force;
        joint.motor_speed = def.motor_speed;
        joint.enable_spring = def.enable_spring;
        joint.enable_limit = def.enable_limit;
        joint.enable_motor = def.enable_motor;
        joint.impulse = 0.0;
        joint.lower_impulse = 0.0;
        joint.upper_impulse = 0.0;
        joint.motor_impulse = 0.0;
        sim.set_distance_joint(joint);
        self.write_joint_sim(pair.joint_id, sim);

        let j = self.joints[pair.joint_id as usize].clone();
        JointID::new(pair.joint_id + 1, self.world_id, j.generation)
    }

    // MARK: Distance joint tuning

    fn update_distance_joint<F: FnOnce(&mut DistanceJointSim)>(
        &mut self,
        joint_id: JointID,
        body: F,
    ) -> bool {
        let index = joint_id.index1 - 1;
        if index < 0 || index as usize >= self.joints.len() {
            return false;
        }
        let joint = self.joints[index as usize].clone();
        if joint.joint_id == b3::NULL_INDEX || joint.type_ != JointType::DistanceJoint {
            return false;
        }
        let mut sim = self.read_joint_sim(index);
        let mut distance = sim.distance_joint();
        body(&mut distance);
        sim.set_distance_joint(distance);
        self.write_joint_sim(index, sim);
        true
    }

    /// Set the rest length. Resets the axial impulses.
    pub fn set_distance_joint_length(&mut self, joint_id: JointID, length: f32) {
        self.update_distance_joint(joint_id, move |joint| {
            joint.length = clamp_float(length, b3::linear_slop(), b3::huge());
            joint.impulse = 0.0;
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
        });
    }

    pub fn distance_joint_length(&self, joint_id: JointID) -> Option<f32> {
        self.distance_joint_sim(joint_id).map(|joint| joint.length)
    }

    pub fn enable_distance_joint_limit(&mut self, joint_id: JointID, enable: bool) {
        self.update_distance_joint(joint_id, move |joint| {
            joint.enable_limit = enable;
        });
    }

    pub fn is_distance_joint_limit_enabled(&self, joint_id: JointID) -> bool {
        self.distance_joint_sim(joint_id).map(|joint| joint.enable_limit).unwrap_or(false)
    }

    /// Set the length range. Resets the limit impulses.
    pub fn set_distance_joint_length_range(
        &mut self,
        joint_id: JointID,
        min_length: f32,
        max_length: f32,
    ) {
        self.update_distance_joint(joint_id, move |joint| {
            let lower = clamp_float(min_length, b3::linear_slop(), b3::huge());
            let upper = clamp_float(max_length, b3::linear_slop(), b3::huge());
            joint.min_length = min_float(lower, upper);
            joint.max_length = max_float(lower, upper);
            joint.impulse = 0.0;
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
        });
    }

    pub fn distance_joint_min_length(&self, joint_id: JointID) -> Option<f32> {
        self.distance_joint_sim(joint_id).map(|joint| joint.min_length)
    }

    pub fn distance_joint_max_length(&self, joint_id: JointID) -> Option<f32> {
        self.distance_joint_sim(joint_id).map(|joint| joint.max_length)
    }

    /// Current distance between the joint anchors.
    pub fn distance_joint_current_length(&self, joint_id: JointID) -> Option<f32> {
        let index = joint_id.index1 - 1;
        if index < 0 || index as usize >= self.joints.len() {
            return None;
        }
        let joint = self.joints[index as usize].clone();
        if joint.joint_id == b3::NULL_INDEX || joint.type_ != JointType::DistanceJoint {
            return None;
        }
        let sim = self.read_joint_sim(index);

        let transform_a = self.body_transform_quick(sim.body_id_a);
        let transform_b = self.body_transform_quick(sim.body_id_b);
        let p_a = transform_point(transform_a, sim.local_frame_a.p);
        let p_b = transform_point(transform_b, sim.local_frame_b.p);
        Some(length(sub_pos(p_b, p_a)))
    }

    pub fn enable_distance_joint_spring(&mut self, joint_id: JointID, enable: bool) {
        self.update_distance_joint(joint_id, move |joint| {
            joint.enable_spring = enable;
        });
    }

    pub fn is_distance_joint_spring_enabled(&self, joint_id: JointID) -> bool {
        self.distance_joint_sim(joint_id).map(|joint| joint.enable_spring).unwrap_or(false)
    }

    pub fn set_distance_joint_spring_force_range(
        &mut self,
        joint_id: JointID,
        lower_force: f32,
        upper_force: f32,
    ) {
        self.update_distance_joint(joint_id, move |joint| {
            joint.lower_spring_force = lower_force;
            joint.upper_spring_force = upper_force;
        });
    }

    pub fn distance_joint_spring_force_range(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.distance_joint_sim(joint_id)?;
        Some((sim.lower_spring_force, sim.upper_spring_force))
    }

    pub fn set_distance_joint_spring_hertz(&mut self, joint_id: JointID, hertz: f32) {
        self.update_distance_joint(joint_id, move |joint| {
            joint.hertz = hertz;
        });
    }

    pub fn set_distance_joint_spring_damping_ratio(
        &mut self,
        joint_id: JointID,
        damping_ratio: f32,
    ) {
        self.update_distance_joint(joint_id, move |joint| {
            joint.damping_ratio = damping_ratio;
        });
    }

    pub fn distance_joint_spring_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.distance_joint_sim(joint_id).map(|joint| joint.hertz)
    }

    pub fn distance_joint_spring_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.distance_joint_sim(joint_id).map(|joint| joint.damping_ratio)
    }

    pub fn enable_distance_joint_motor(&mut self, joint_id: JointID, enable: bool) {
        self.update_distance_joint(joint_id, move |joint| {
            if enable != joint.enable_motor {
                joint.enable_motor = enable;
                joint.motor_impulse = 0.0;
            }
        });
    }

    pub fn is_distance_joint_motor_enabled(&self, joint_id: JointID) -> bool {
        self.distance_joint_sim(joint_id).map(|joint| joint.enable_motor).unwrap_or(false)
    }

    pub fn set_distance_joint_motor_speed(&mut self, joint_id: JointID, motor_speed: f32) {
        self.update_distance_joint(joint_id, move |joint| {
            joint.motor_speed = motor_speed;
        });
    }

    pub fn distance_joint_motor_speed(&self, joint_id: JointID) -> Option<f32> {
        self.distance_joint_sim(joint_id).map(|joint| joint.motor_speed)
    }

    pub fn set_distance_joint_max_motor_force(&mut self, joint_id: JointID, force: f32) {
        self.update_distance_joint(joint_id, move |joint| {
            joint.max_motor_force = force;
        });
    }

    pub fn distance_joint_max_motor_force(&self, joint_id: JointID) -> Option<f32> {
        self.distance_joint_sim(joint_id).map(|joint| joint.max_motor_force)
    }

    /// Motor force reported by the solver for the last step.
    pub fn distance_joint_motor_force(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.distance_joint_sim(joint_id)?;
        Some(self.inv_h * sim.motor_impulse)
    }

    pub(crate) fn distance_joint_sim(&self, joint_id: JointID) -> Option<DistanceJointSim> {
        let index = joint_id.index1 - 1;
        if index < 0 || index as usize >= self.joints.len() {
            return None;
        }
        let joint = self.joints[index as usize].clone();
        if joint.joint_id == b3::NULL_INDEX || joint.type_ != JointType::DistanceJoint {
            return None;
        }
        Some(self.read_joint_sim(index).distance_joint())
    }

    // MARK: Destruction

    /// Destroy a joint and remove it from its body lists, island, and solver set.
    pub fn destroy_joint(&mut self, joint_id: i32, wake_attached: bool) {
        let mut joint = self.joints[joint_id as usize].clone();
        if joint.type_ == JointType::SixDOFJoint {
            let slot = self.read_joint_sim(joint_id).six_dof().slot;
            self.release_six_dof_data(slot);
        } else if joint.type_ == JointType::PathJoint {
            let slot = self.read_joint_sim(joint_id).path().slot;
            self.release_path_data(slot);
        }

        let edge_a = joint.edges[0];
        let edge_b = joint.edges[1];

        let id_a = edge_a.body_id;
        let id_b = edge_b.body_id;
        let mut body_a = self.bodies[id_a as usize].clone();
        let mut body_b = self.bodies[id_b as usize].clone();

        // Remove from body A
        if edge_a.prev_key != b3::NULL_INDEX {
            let prev_joint_id = edge_a.prev_key >> 1;
            let mut prev_joint = self.joints[prev_joint_id as usize].clone();
            let edge_index = (edge_a.prev_key & 1) as usize;
            prev_joint.edges[edge_index].next_key = edge_a.next_key;
            self.joints[prev_joint_id as usize] = prev_joint;
        }

        if edge_a.next_key != b3::NULL_INDEX {
            let next_joint_id = edge_a.next_key >> 1;
            let mut next_joint = self.joints[next_joint_id as usize].clone();
            let edge_index = (edge_a.next_key & 1) as usize;
            next_joint.edges[edge_index].prev_key = edge_a.prev_key;
            self.joints[next_joint_id as usize] = next_joint;
        }

        let edge_key_a = joint_id << 1;
        if body_a.head_joint_key == edge_key_a {
            body_a.head_joint_key = edge_a.next_key;
        }
        body_a.joint_count -= 1;

        // Remove from body B
        if edge_b.prev_key != b3::NULL_INDEX {
            let prev_joint_id = edge_b.prev_key >> 1;
            let mut prev_joint = self.joints[prev_joint_id as usize].clone();
            let edge_index = (edge_b.prev_key & 1) as usize;
            prev_joint.edges[edge_index].next_key = edge_b.next_key;
            self.joints[prev_joint_id as usize] = prev_joint;
        }

        if edge_b.next_key != b3::NULL_INDEX {
            let next_joint_id = edge_b.next_key >> 1;
            let mut next_joint = self.joints[next_joint_id as usize].clone();
            let edge_index = (edge_b.next_key & 1) as usize;
            next_joint.edges[edge_index].prev_key = edge_b.prev_key;
            self.joints[next_joint_id as usize] = next_joint;
        }

        let edge_key_b = (joint_id << 1) | 1;
        if body_b.head_joint_key == edge_key_b {
            body_b.head_joint_key = edge_b.next_key;
        }
        body_b.joint_count -= 1;

        self.bodies[id_a as usize] = body_a;
        self.bodies[id_b as usize] = body_b;

        if joint.island_id != b3::NULL_INDEX {
            b3_assert(joint.set_index > SetType::DisabledSet.raw_value());
            self.unlink_joint(joint_id);
            joint = self.joints[joint_id as usize].clone();
        }

        // Remove joint from solver set that owns it
        let set_index = joint.set_index;
        let local_index = joint.local_index;

        if set_index == SetType::AwakeSet.raw_value() {
            self.remove_joint_from_graph(
                joint.edges[0].body_id,
                joint.edges[1].body_id,
                joint.color_index,
                local_index,
            );
        } else if set_index != b3::NULL_INDEX {
            // Swift operated on a `set` snapshot and committed it back; the direct in-place
            // mutation below produces the same final solver-set state.
            let moved_index = remove_swap_array(
                &mut self.solver_sets[set_index as usize].joint_sims,
                local_index as usize,
            );
            if moved_index != b3::NULL_INDEX {
                let moved_id =
                    self.solver_sets[set_index as usize].joint_sims[local_index as usize].joint_id;
                let mut moved_joint = self.joints[moved_id as usize].clone();
                b3_assert(moved_joint.local_index == moved_index);
                moved_joint.local_index = local_index;
                self.joints[moved_id as usize] = moved_joint;
            }
        }

        // Free joint and id (preserve joint revision)
        joint.set_index = b3::NULL_INDEX;
        joint.local_index = b3::NULL_INDEX;
        joint.color_index = b3::NULL_INDEX;
        joint.joint_id = b3::NULL_INDEX;
        self.joints[joint_id as usize] = joint;
        self.joint_id_pool.free(joint_id);

        if wake_attached {
            self.wake_body(id_a);
            self.wake_body(id_b);
        }
    }
}

// MARK: - Generic joint tuning

impl World {
    /// Read-modify-write the sim of a joint of a given type.
    /// Swift's `mutateJointSim(_:_:_:)`; the untyped variant in the inspection extension below
    /// keeps the bare name because Rust cannot overload.
    fn mutate_joint_sim_of_type<F: FnOnce(&mut JointSim)>(
        &mut self,
        joint_id: JointID,
        type_: JointType,
        body: F,
    ) -> bool {
        let index = joint_id.index1 - 1;
        if index < 0 || index as usize >= self.joints.len() {
            return false;
        }
        let joint = self.joints[index as usize].clone();
        if joint.joint_id == b3::NULL_INDEX || joint.type_ != type_ {
            return false;
        }
        let mut sim = self.read_joint_sim(index);
        body(&mut sim);
        self.write_joint_sim(index, sim);
        true
    }

    /// Read the sim of a joint of a given type.
    /// Swift's fileprivate `jointSim(_:_:)`; the untyped variant keeps the bare name.
    fn joint_sim_of_type(&self, joint_id: JointID, type_: JointType) -> Option<JointSim> {
        let index = joint_id.index1 - 1;
        if index < 0 || index as usize >= self.joints.len() {
            return None;
        }
        let joint = &self.joints[index as usize];
        if joint.joint_id == b3::NULL_INDEX || joint.type_ != type_ {
            return None;
        }
        Some(self.read_joint_sim(index))
    }

    /// The type of a joint, or nil if the id is invalid.
    pub fn joint_type(&self, joint_id: JointID) -> Option<JointType> {
        let index = joint_id.index1 - 1;
        if joint_id.world0 != self.world_id
            || index < 0
            || index as usize >= self.joints.len()
        {
            return None;
        }
        let joint = &self.joints[index as usize];
        if joint.joint_id == b3::NULL_INDEX || joint.generation != joint_id.generation {
            return None;
        }
        Some(joint.type_)
    }

    /// The two bodies attached to a joint.
    pub fn joint_bodies(&self, joint_id: JointID) -> Option<(BodyID, BodyID)> {
        let index = joint_id.index1 - 1;
        if index < 0 || index as usize >= self.joints.len() {
            return None;
        }
        let joint = &self.joints[index as usize];
        if joint.joint_id == b3::NULL_INDEX {
            return None;
        }
        Some((
            self.make_body_id(joint.edges[0].body_id),
            self.make_body_id(joint.edges[1].body_id),
        ))
    }

    /// True while the joint id refers to a live joint.
    pub fn joint_is_valid(&self, joint_id: JointID) -> bool {
        self.joint_type(joint_id).is_some()
    }

    // MARK: Revolute

    pub fn set_revolute_joint_limits(&mut self, joint_id: JointID, lower: f32, upper: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            joint.lower_angle = lower;
            joint.upper_angle = upper;
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
            sim.set_revolute(joint);
        })
    }

    pub fn revolute_joint_limits(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)?;
        let joint = sim.revolute();
        Some((joint.lower_angle, joint.upper_angle))
    }

    pub fn enable_revolute_joint_limit(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            joint.enable_limit = enable;
            sim.set_revolute(joint);
        })
    }

    pub fn is_revolute_joint_limit_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().enable_limit)
            .unwrap_or(false)
    }

    pub fn enable_revolute_joint_motor(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            if enable != joint.enable_motor {
                joint.enable_motor = enable;
                joint.motor_impulse = 0.0;
                sim.set_revolute(joint);
            }
        })
    }

    pub fn is_revolute_joint_motor_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().enable_motor)
            .unwrap_or(false)
    }

    pub fn set_revolute_joint_motor_speed(&mut self, joint_id: JointID, speed: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            joint.motor_speed = speed;
            sim.set_revolute(joint);
        })
    }

    pub fn revolute_joint_motor_speed(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().motor_speed)
    }

    pub fn set_revolute_joint_max_motor_torque(&mut self, joint_id: JointID, torque: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            joint.max_motor_torque = torque;
            sim.set_revolute(joint);
        })
    }

    pub fn revolute_joint_max_motor_torque(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().max_motor_torque)
    }

    /// Motor torque reported by the solver for the last step.
    pub fn revolute_joint_motor_torque(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)?;
        Some(self.inv_h * sim.revolute().motor_impulse)
    }

    /// Current hinge angle relative to the joint target.
    pub fn revolute_joint_angle(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)?;
        let quat_a = mul_quat(self.body_transform_quick(sim.body_id_a).q, sim.local_frame_a.q);
        let quat_b = mul_quat(self.body_transform_quick(sim.body_id_b).q, sim.local_frame_b.q);
        Some(get_twist_angle(inv_mul_quat(quat_a, quat_b)))
    }

    pub fn enable_revolute_joint_spring(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            joint.enable_spring = enable;
            sim.set_revolute(joint);
        })
    }

    pub fn is_revolute_joint_spring_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().enable_spring)
            .unwrap_or(false)
    }

    pub fn set_revolute_joint_spring_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            joint.hertz = hertz;
            sim.set_revolute(joint);
        })
    }

    pub fn set_revolute_joint_spring_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            joint.damping_ratio = ratio;
            sim.set_revolute(joint);
        })
    }

    pub fn revolute_joint_spring_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().hertz)
    }

    pub fn revolute_joint_spring_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().damping_ratio)
    }

    // MARK: Prismatic

    pub fn set_prismatic_joint_limits(
        &mut self,
        joint_id: JointID,
        lower: f32,
        upper: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            joint.lower_translation = lower;
            joint.upper_translation = upper;
            joint.lower_impulse = 0.0;
            joint.upper_impulse = 0.0;
            sim.set_prismatic(joint);
        })
    }

    pub fn prismatic_joint_limits(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)?;
        let joint = sim.prismatic();
        Some((joint.lower_translation, joint.upper_translation))
    }

    pub fn enable_prismatic_joint_limit(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            joint.enable_limit = enable;
            sim.set_prismatic(joint);
        })
    }

    pub fn is_prismatic_joint_limit_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().enable_limit)
            .unwrap_or(false)
    }

    pub fn enable_prismatic_joint_motor(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            if enable != joint.enable_motor {
                joint.enable_motor = enable;
                joint.motor_impulse = 0.0;
                sim.set_prismatic(joint);
            }
        })
    }

    pub fn is_prismatic_joint_motor_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().enable_motor)
            .unwrap_or(false)
    }

    pub fn set_prismatic_joint_motor_speed(&mut self, joint_id: JointID, speed: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            joint.motor_speed = speed;
            sim.set_prismatic(joint);
        })
    }

    pub fn prismatic_joint_motor_speed(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().motor_speed)
    }

    pub fn set_prismatic_joint_max_motor_force(&mut self, joint_id: JointID, force: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            joint.max_motor_force = force;
            sim.set_prismatic(joint);
        })
    }

    pub fn prismatic_joint_max_motor_force(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().max_motor_force)
    }

    pub fn prismatic_joint_motor_force(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)?;
        Some(self.inv_h * sim.prismatic().motor_impulse)
    }

    pub fn enable_prismatic_joint_spring(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            joint.enable_spring = enable;
            sim.set_prismatic(joint);
        })
    }

    pub fn is_prismatic_joint_spring_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().enable_spring)
            .unwrap_or(false)
    }

    pub fn set_prismatic_joint_spring_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            joint.hertz = hertz;
            sim.set_prismatic(joint);
        })
    }

    pub fn set_prismatic_joint_spring_damping_ratio(
        &mut self,
        joint_id: JointID,
        ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            joint.damping_ratio = ratio;
            sim.set_prismatic(joint);
        })
    }

    pub fn prismatic_joint_spring_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().hertz)
    }

    pub fn prismatic_joint_spring_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().damping_ratio)
    }

    pub fn set_prismatic_joint_target_translation(
        &mut self,
        joint_id: JointID,
        value: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::PrismaticJoint, move |sim| {
            let mut joint = sim.prismatic();
            joint.target_translation = value;
            sim.set_prismatic(joint);
        })
    }

    pub fn prismatic_joint_target_translation(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().target_translation)
    }
}

// MARK: - Wheel, spherical, motor, weld, parallel tuning

impl World {
    // MARK: Wheel

    pub fn enable_wheel_joint_spin_motor(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.enable_spin_motor = enable;
            sim.set_wheel(joint);
        })
    }

    pub fn is_wheel_joint_spin_motor_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().enable_spin_motor)
            .unwrap_or(false)
    }

    pub fn set_wheel_joint_spin_speed(&mut self, joint_id: JointID, speed: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.spin_speed = speed;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_spin_speed(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().spin_speed)
    }

    pub fn set_wheel_joint_max_spin_torque(&mut self, joint_id: JointID, torque: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.max_spin_torque = torque;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_max_spin_torque(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().max_spin_torque)
    }

    pub fn wheel_joint_motor_torque(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        Some(self.inv_h * sim.wheel().spin_impulse)
    }

    pub fn enable_wheel_joint_suspension_spring(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.enable_suspension_spring = enable;
            sim.set_wheel(joint);
        })
    }

    pub fn set_wheel_joint_suspension_spring(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.suspension_hertz = hertz;
            joint.suspension_damping_ratio = damping_ratio;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_suspension_spring(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        let joint = sim.wheel();
        Some((joint.suspension_hertz, joint.suspension_damping_ratio))
    }

    pub fn set_wheel_joint_suspension_limits(
        &mut self,
        joint_id: JointID,
        lower: f32,
        upper: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.lower_suspension_limit = lower;
            joint.upper_suspension_limit = upper;
            sim.set_wheel(joint);
        })
    }

    pub fn enable_wheel_joint_suspension_limit(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.enable_suspension_limit = enable;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_suspension_limits(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        let joint = sim.wheel();
        Some((joint.lower_suspension_limit, joint.upper_suspension_limit))
    }

    pub fn enable_wheel_joint_steering(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.enable_steering = enable;
            sim.set_wheel(joint);
        })
    }

    pub fn is_wheel_joint_steering_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().enable_steering)
            .unwrap_or(false)
    }

    pub fn set_wheel_joint_target_steering_angle(&mut self, joint_id: JointID, angle: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.target_steering_angle = angle;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_target_steering_angle(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().target_steering_angle)
    }

    pub fn set_wheel_joint_max_steering_torque(&mut self, joint_id: JointID, torque: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.max_steering_torque = torque;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_max_steering_torque(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().max_steering_torque)
    }

    pub fn set_wheel_joint_steering_spring(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.steering_hertz = hertz;
            joint.steering_damping_ratio = damping_ratio;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_steering_spring(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        let joint = sim.wheel();
        Some((joint.steering_hertz, joint.steering_damping_ratio))
    }

    pub fn set_wheel_joint_steering_limits(
        &mut self,
        joint_id: JointID,
        lower: f32,
        upper: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.lower_steering_limit = lower;
            joint.upper_steering_limit = upper;
            sim.set_wheel(joint);
        })
    }

    pub fn enable_wheel_joint_steering_limit(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.enable_steering_limit = enable;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_steering_limits(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        let joint = sim.wheel();
        Some((joint.lower_steering_limit, joint.upper_steering_limit))
    }

    // MARK: Spherical

    pub fn set_spherical_joint_cone_limit(&mut self, joint_id: JointID, cone_angle: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.cone_angle = cone_angle;
            sim.set_spherical(joint);
        })
    }

    pub fn enable_spherical_joint_cone_limit(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.enable_cone_limit = enable;
            sim.set_spherical(joint);
        })
    }

    pub fn spherical_joint_cone_limit(&self, joint_id: JointID) -> Option<(bool, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::SphericalJoint)?;
        let joint = sim.spherical();
        Some((joint.enable_cone_limit, joint.cone_angle))
    }

    pub fn set_spherical_joint_twist_limits(
        &mut self,
        joint_id: JointID,
        lower: f32,
        upper: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.lower_twist_angle = lower;
            joint.upper_twist_angle = upper;
            sim.set_spherical(joint);
        })
    }

    pub fn enable_spherical_joint_twist_limit(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.enable_twist_limit = enable;
            sim.set_spherical(joint);
        })
    }

    pub fn spherical_joint_twist_limits(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::SphericalJoint)?;
        let joint = sim.spherical();
        Some((joint.lower_twist_angle, joint.upper_twist_angle))
    }

    pub fn enable_spherical_joint_motor(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.enable_motor = enable;
            sim.set_spherical(joint);
        })
    }

    pub fn set_spherical_joint_motor_velocity(
        &mut self,
        joint_id: JointID,
        velocity: Vec3,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.motor_velocity = velocity;
            sim.set_spherical(joint);
        })
    }

    pub fn spherical_joint_motor_velocity(&self, joint_id: JointID) -> Option<Vec3> {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().motor_velocity)
    }

    pub fn set_spherical_joint_max_motor_torque(&mut self, joint_id: JointID, torque: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.max_motor_torque = torque;
            sim.set_spherical(joint);
        })
    }

    pub fn spherical_joint_max_motor_torque(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().max_motor_torque)
    }

    pub fn enable_spherical_joint_spring(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.enable_spring = enable;
            sim.set_spherical(joint);
        })
    }

    pub fn set_spherical_joint_spring(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.hertz = hertz;
            joint.damping_ratio = damping_ratio;
            sim.set_spherical(joint);
        })
    }

    pub fn spherical_joint_spring(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::SphericalJoint)?;
        let joint = sim.spherical();
        Some((joint.hertz, joint.damping_ratio))
    }

    pub fn set_spherical_joint_target_rotation(
        &mut self,
        joint_id: JointID,
        rotation: Quat,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.target_rotation = rotation;
            sim.set_spherical(joint);
        })
    }

    pub fn spherical_joint_target_rotation(&self, joint_id: JointID) -> Option<Quat> {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().target_rotation)
    }

    // MARK: Motor

    pub fn set_motor_joint_linear_velocity(&mut self, joint_id: JointID, velocity: Vec3) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.linear_velocity = velocity;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_linear_velocity(&self, joint_id: JointID) -> Option<Vec3> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().linear_velocity)
    }

    pub fn set_motor_joint_angular_velocity(&mut self, joint_id: JointID, velocity: Vec3) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.angular_velocity = velocity;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_angular_velocity(&self, joint_id: JointID) -> Option<Vec3> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().angular_velocity)
    }

    pub fn set_motor_joint_max_velocity_force(&mut self, joint_id: JointID, force: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.max_velocity_force = force;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_max_velocity_force(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().max_velocity_force)
    }

    pub fn set_motor_joint_max_velocity_torque(&mut self, joint_id: JointID, torque: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.max_velocity_torque = torque;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_max_velocity_torque(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().max_velocity_torque)
    }

    pub fn set_motor_joint_linear_spring(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.linear_hertz = hertz;
            joint.linear_damping_ratio = damping_ratio;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_linear_spring(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::MotorJoint)?;
        let joint = sim.motor();
        Some((joint.linear_hertz, joint.linear_damping_ratio))
    }

    pub fn set_motor_joint_angular_spring(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.angular_hertz = hertz;
            joint.angular_damping_ratio = damping_ratio;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_angular_spring(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::MotorJoint)?;
        let joint = sim.motor();
        Some((joint.angular_hertz, joint.angular_damping_ratio))
    }

    pub fn set_motor_joint_max_spring_force(&mut self, joint_id: JointID, force: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.max_spring_force = force;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_max_spring_force(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().max_spring_force)
    }

    pub fn set_motor_joint_max_spring_torque(&mut self, joint_id: JointID, torque: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.max_spring_torque = torque;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_max_spring_torque(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().max_spring_torque)
    }

    // MARK: Weld

    pub fn set_weld_joint_linear_spring(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WeldJoint, move |sim| {
            let mut joint = sim.weld();
            joint.linear_hertz = hertz;
            joint.linear_damping_ratio = damping_ratio;
            sim.set_weld(joint);
        })
    }

    pub fn weld_joint_linear_spring(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WeldJoint)?;
        let joint = sim.weld();
        Some((joint.linear_hertz, joint.linear_damping_ratio))
    }

    pub fn set_weld_joint_angular_spring(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WeldJoint, move |sim| {
            let mut joint = sim.weld();
            joint.angular_hertz = hertz;
            joint.angular_damping_ratio = damping_ratio;
            sim.set_weld(joint);
        })
    }

    pub fn weld_joint_angular_spring(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WeldJoint)?;
        let joint = sim.weld();
        Some((joint.angular_hertz, joint.angular_damping_ratio))
    }

    // MARK: Parallel

    pub fn set_parallel_joint_spring(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::ParallelJoint, move |sim| {
            let mut joint = sim.parallel();
            joint.hertz = hertz;
            joint.damping_ratio = damping_ratio;
            sim.set_parallel(joint);
        })
    }

    pub fn parallel_joint_spring(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim_of_type(joint_id, JointType::ParallelJoint)?;
        let joint = sim.parallel();
        Some((joint.hertz, joint.damping_ratio))
    }

    pub fn set_parallel_joint_max_torque(&mut self, joint_id: JointID, torque: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::ParallelJoint, move |sim| {
            let mut joint = sim.parallel();
            joint.max_torque = torque;
            sim.set_parallel(joint);
        })
    }

    pub fn parallel_joint_max_torque(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::ParallelJoint)
            .map(|sim| sim.parallel().max_torque)
    }
}

// MARK: - Joint inspection

impl World {
    /// Read any joint's sim regardless of type.
    pub(crate) fn joint_sim(&self, joint_id: JointID) -> Option<JointSim> {
        let index = joint_id.index1 - 1;
        if index < 0 || index as usize >= self.joints.len() {
            return None;
        }
        if self.joints[index as usize].joint_id == b3::NULL_INDEX {
            return None;
        }
        Some(self.read_joint_sim(index))
    }

    /// Mutate any joint's sim regardless of type.
    pub(crate) fn mutate_joint_sim<F: FnOnce(&mut JointSim)>(
        &mut self,
        joint_id: JointID,
        body: F,
    ) -> bool {
        let index = joint_id.index1 - 1;
        if index < 0 || index as usize >= self.joints.len() {
            return false;
        }
        if self.joints[index as usize].joint_id == b3::NULL_INDEX {
            return false;
        }
        let mut sim = self.read_joint_sim(index);
        body(&mut sim);
        self.write_joint_sim(index, sim);
        true
    }

    /// The joint record for an id.
    pub(crate) fn joint(&self, joint_id: JointID) -> Joint {
        self.joints[(joint_id.index1 - 1) as usize].clone()
    }

    pub(crate) fn joint_index(&self, joint_id: JointID) -> i32 {
        joint_id.index1 - 1
    }

    /// Is the joint awake? True when at least one attached body is awake.
    pub fn joint_is_awake(&self, joint_id: JointID) -> bool {
        let joint = self.joint(joint_id);
        if joint.set_index == b3::NULL_INDEX {
            return false;
        }
        joint.set_index == SetType::AwakeSet.raw_value()
    }

    /// Wake both bodies attached to the joint.
    pub fn wake_joint_bodies(&mut self, joint_id: JointID) {
        let joint = self.joint(joint_id);
        self.wake_body(joint.edges[0].body_id);
        self.wake_body(joint.edges[1].body_id);
    }

    /// Is collision allowed between the connected bodies?
    pub fn joint_collide_connected(&self, joint_id: JointID) -> bool {
        self.joint(joint_id).collide_connected
    }

    /// Allow or prevent collision between the connected bodies.
    pub fn set_joint_collide_connected(&mut self, joint_id: JointID, should_collide: bool) {
        let index = self.joint_index(joint_id);
        let mut record = self.joints[index as usize].clone();
        record.collide_connected = should_collide;
        self.joints[index as usize] = record;
    }

    /// The joint's constraint tuning.
    pub fn joint_constraint_tuning(&self, joint_id: JointID) -> Option<(f32, f32)> {
        let sim = self.joint_sim(joint_id)?;
        Some((sim.constraint_hertz, sim.constraint_damping_ratio))
    }

    /// Set the joint's constraint tuning.
    pub fn set_joint_constraint_tuning(
        &mut self,
        joint_id: JointID,
        hertz: f32,
        damping_ratio: f32,
    ) -> bool {
        self.mutate_joint_sim(joint_id, move |sim| {
            sim.constraint_hertz = hertz;
            sim.constraint_damping_ratio = damping_ratio;
        })
    }

    /// The force threshold for joint events.
    pub fn joint_force_threshold(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim(joint_id).map(|sim| sim.force_threshold)
    }

    /// Set the force threshold for joint events.
    pub fn set_joint_force_threshold(&mut self, joint_id: JointID, threshold: f32) -> bool {
        self.mutate_joint_sim(joint_id, move |sim| {
            sim.force_threshold = threshold;
        })
    }

    /// The torque threshold for joint events.
    pub fn joint_torque_threshold(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim(joint_id).map(|sim| sim.torque_threshold)
    }

    /// Set the torque threshold for joint events.
    pub fn set_joint_torque_threshold(&mut self, joint_id: JointID, threshold: f32) -> bool {
        self.mutate_joint_sim(joint_id, move |sim| {
            sim.torque_threshold = threshold;
        })
    }

    /// The local frame on body A.
    pub fn joint_local_frame_a(&self, joint_id: JointID) -> Option<Transform> {
        self.joint_sim(joint_id).map(|sim| sim.local_frame_a)
    }

    /// The local frame on body B.
    pub fn joint_local_frame_b(&self, joint_id: JointID) -> Option<Transform> {
        self.joint_sim(joint_id).map(|sim| sim.local_frame_b)
    }

    /// Set the local frame on body A.
    pub fn set_joint_local_frame_a(&mut self, joint_id: JointID, frame: Transform) -> bool {
        self.mutate_joint_sim(joint_id, move |sim| {
            sim.local_frame_a = frame;
        })
    }

    /// Set the local frame on body B.
    pub fn set_joint_local_frame_b(&mut self, joint_id: JointID, frame: Transform) -> bool {
        self.mutate_joint_sim(joint_id, move |sim| {
            sim.local_frame_b = frame;
        })
    }

    /// A world point on the joint's body A.
    pub fn joint_world_anchor_a(&self, joint_id: JointID) -> Option<crate::math::types::Pos> {
        let sim = self.joint_sim(joint_id)?;
        Some(transform_world_point(
            self.body_transform_quick(sim.body_id_a),
            sim.local_frame_a.p,
        ))
    }

    /// The current linear separation error. Zero for a coincident anchor.
    pub fn joint_linear_separation(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim(joint_id)?;
        let xf_a = self.body_transform_quick(sim.body_id_a);
        let xf_b = self.body_transform_quick(sim.body_id_b);
        let p_a = transform_world_point(xf_a, sim.local_frame_a.p);
        let p_b = transform_world_point(xf_b, sim.local_frame_b.p);

        match self.joint_type(joint_id) {
            Some(JointType::ParallelJoint) => Some(0.0),
            _ => Some(length(sub_pos(p_b, p_a))),
        }
    }

    /// The current angular separation error in radians.
    pub fn joint_angular_separation(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim(joint_id)?;
        let xf_a = self.body_transform_quick(sim.body_id_a);
        let xf_b = self.body_transform_quick(sim.body_id_b);
        let q_a = mul_quat(xf_a.q, sim.local_frame_a.q);
        let q_b = mul_quat(xf_b.q, sim.local_frame_b.q);
        let rel = inv_mul_quat(q_a, q_b);

        // The angle of the relative rotation, clamped for numerical safety.
        Some(2.0 * atan2_approx(length(rel.v), abs_float(rel.s)))
    }

    /// The current constraint force on the joint, in newtons.
    pub fn joint_constraint_force(&self, joint_id: JointID) -> Option<Vec3> {
        let sim = self.joint_sim(joint_id)?;
        Some(self.joint_constraint_force_for_sim(sim))
    }

    /// The current constraint torque on the joint, in newton-meters.
    pub fn joint_constraint_torque(&self, joint_id: JointID) -> Option<Vec3> {
        let sim = self.joint_sim(joint_id)?;
        Some(self.joint_constraint_torque_for_sim(sim))
    }

    // Swift's private overload `jointConstraintForce(_ sim:)`; Rust cannot overload.
    fn joint_constraint_force_for_sim(&self, sim: JointSim) -> Vec3 {
        match sim.type_ {
            JointType::ParallelJoint | JointType::FilterJoint => Vec3::default(),
            JointType::GearJoint
            | JointType::RackAndPinionJoint
            | JointType::PulleyJoint
            | JointType::SixDOFJoint
            | JointType::PathJoint => self.coupling_joint_force(sim),
            JointType::DistanceJoint => {
                let joint = sim.distance();
                let xf_a = self.body_transform_quick(sim.body_id_a);
                let xf_b = self.body_transform_quick(sim.body_id_b);
                let p_a = transform_world_point(xf_a, sim.local_frame_a.p);
                let p_b = transform_world_point(xf_b, sim.local_frame_b.p);
                let axis = normalize(sub_pos(p_b, p_a));
                let force =
                    (joint.impulse + joint.lower_impulse - joint.upper_impulse
                        + joint.motor_impulse)
                        * self.inv_h;
                force * axis
            }
            JointType::MotorJoint => {
                let motor = sim.motor();
                self.inv_h * (motor.linear_velocity_impulse + motor.linear_spring_impulse)
            }
            JointType::PrismaticJoint => {
                let joint = sim.prismatic();
                let xf_a = self.body_transform_quick(sim.body_id_a);
                let impulse = Vec3::new(
                    joint.perp_impulse.x,
                    joint.perp_impulse.y,
                    joint.motor_impulse + joint.lower_impulse + joint.upper_impulse
                        + joint.spring_impulse,
                );
                // Convert the impulse from joint space to world.
                let mut force = self.inv_h * impulse;
                force = rotate_vector(mul_quat(xf_a.q, sim.local_frame_a.q), force);
                force
            }
            JointType::RevoluteJoint => {
                let revolute = sim.revolute();
                self.inv_h * revolute.linear_impulse
            }
            JointType::SphericalJoint => {
                let spherical = sim.spherical();
                self.inv_h * spherical.linear_impulse
            }
            JointType::WeldJoint => {
                let weld = sim.weld();
                self.inv_h * weld.linear_impulse
            }
            JointType::WheelJoint => {
                let joint = sim.wheel();
                let xf_a = self.body_transform_quick(sim.body_id_a);
                let impulse = Vec3::new(
                    joint.linear_impulse.x,
                    joint.linear_impulse.y,
                    joint.lower_suspension_limit + joint.upper_suspension_impulse
                        + joint.suspension_spring_impulse,
                );
                let mut force = self.inv_h * impulse;
                force = rotate_vector(mul_quat(xf_a.q, sim.local_frame_a.q), force);
                force
            }
        }
    }

    // Swift's private overload `jointConstraintTorque(_ sim:)`; Rust cannot overload.
    fn joint_constraint_torque_for_sim(&self, sim: JointSim) -> Vec3 {
        match sim.type_ {
            JointType::DistanceJoint | JointType::FilterJoint => Vec3::default(),
            JointType::GearJoint
            | JointType::RackAndPinionJoint
            | JointType::PulleyJoint
            | JointType::SixDOFJoint
            | JointType::PathJoint => self.coupling_joint_torque(sim),
            JointType::ParallelJoint => {
                let parallel = sim.parallel();
                self.inv_h * Vec3::new(0.0, 0.0, parallel.max_torque)
            }
            JointType::MotorJoint => {
                let motor = sim.motor();
                self.inv_h * (motor.angular_velocity_impulse + motor.angular_spring_impulse)
            }
            JointType::PrismaticJoint => {
                let xf_a = self.body_transform_quick(sim.body_id_a);
                let prismatic = sim.prismatic();
                let torque = rotate_vector(
                    mul_quat(xf_a.q, sim.local_frame_a.q),
                    self.inv_h * prismatic.angular_impulse,
                );
                torque
            }
            JointType::RevoluteJoint => {
                let joint = sim.revolute();
                let xf_a = self.body_transform_quick(sim.body_id_a);
                let axis = rotate_vector(
                    xf_a.q,
                    rotate_vector(sim.local_frame_a.q, Vec3::AXIS_Z),
                );

                let rel_q = inv_mul_quat(joint.frame_a.q, joint.frame_b.q);

                // These are needed for warm starting.
                let perp_axis_x = 0.5
                    * rotate_vector(
                        joint.frame_a.q,
                        rel_q.s * Vec3::AXIS_X + cross(rel_q.v, Vec3::AXIS_X),
                    );
                let perp_axis_y = 0.5
                    * rotate_vector(
                        joint.frame_a.q,
                        rel_q.s * Vec3::AXIS_Y + cross(rel_q.v, Vec3::AXIS_Y),
                    );

                let axial_impulse = joint.spring_impulse + joint.motor_impulse
                    + joint.lower_impulse
                    - joint.upper_impulse;
                let mut angular_impulse =
                    joint.perp_impulse.x * perp_axis_x + joint.perp_impulse.y * perp_axis_y;
                angular_impulse = mul_add(angular_impulse, axial_impulse, joint.rotation_axis_z);
                let impulse = mul_add(angular_impulse, axial_impulse, axis);
                self.inv_h * impulse
            }
            JointType::SphericalJoint => {
                let joint = sim.spherical();
                let xf_a = self.body_transform_quick(sim.body_id_a);
                let xf_b = self.body_transform_quick(sim.body_id_b);
                let q_a = mul_quat(xf_a.q, sim.local_frame_a.q);
                let q_b = mul_quat(xf_b.q, sim.local_frame_b.q);

                // Cone axis is the z-axis of body A.
                let cone_axis = rotate_vector(q_a, Vec3::AXIS_Z);
                let twist_axis = rotate_vector(q_b, Vec3::AXIS_Z);
                let swing_axis = normalize(cross(cone_axis, twist_axis));

                let mut impulse = joint.spring_impulse + joint.motor_impulse;
                impulse = mul_add(
                    impulse,
                    joint.lower_twist_impulse - joint.upper_twist_impulse,
                    twist_axis,
                );
                impulse = mul_add(impulse, joint.swing_impulse, swing_axis);
                self.inv_h * impulse
            }
            JointType::WeldJoint => {
                let weld = sim.weld();
                self.inv_h * weld.angular_impulse
            }
            JointType::WheelJoint => {
                let body_a = self.bodies[sim.body_id_a as usize].clone();
                let sim_a =
                    self.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
                let q_a = mul_quat(sim_a.transform.q, sim.local_frame_a.q);
                let matrix_a = make_matrix_from_quat(q_a);
                (self.inv_h * sim.wheel().spin_impulse) * matrix_a.cz
            }
        }
    }
}

// MARK: - Per-type scalar accessors

impl World {
    // MARK: Wheel

    pub fn enable_wheel_joint_suspension(&mut self, joint_id: JointID, enable: bool) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.enable_suspension_spring = enable;
            sim.set_wheel(joint);
        })
    }

    pub fn is_wheel_joint_suspension_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().enable_suspension_spring)
            .unwrap_or(false)
    }

    pub fn is_wheel_joint_suspension_limit_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().enable_suspension_limit)
            .unwrap_or(false)
    }

    pub fn is_wheel_joint_steering_limit_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().enable_steering_limit)
            .unwrap_or(false)
    }

    pub fn wheel_joint_suspension_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().suspension_hertz)
    }

    pub fn set_wheel_joint_suspension_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.suspension_hertz = hertz;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_suspension_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().suspension_damping_ratio)
    }

    pub fn set_wheel_joint_suspension_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.suspension_damping_ratio = ratio;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_lower_suspension_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().lower_suspension_limit)
    }

    pub fn wheel_joint_upper_suspension_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().upper_suspension_limit)
    }

    pub fn wheel_joint_spin_motor_speed(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().spin_speed)
    }

    pub fn set_wheel_joint_spin_motor_speed(&mut self, joint_id: JointID, speed: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.spin_speed = speed;
            sim.set_wheel(joint);
        })
    }

    /// The wheel's angular speed about its spin axis, in radians per second.
    pub fn wheel_joint_spin_speed_value(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        let body_b = self.bodies[sim.body_id_b as usize].clone();
        let sim_b =
            self.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];
        let quat_b = mul_quat(sim_b.transform.q, sim.local_frame_b.q);
        let spin_axis = rotate_vector(quat_b, Vec3::AXIS_Z);

        let w_a = self.body_angular_velocity_at(sim.body_id_a);
        let w_b = self.body_angular_velocity_at(sim.body_id_b);
        Some(dot(w_b - w_a, spin_axis))
    }

    /// The motor torque applied about the spin axis, in newton-meters.
    pub fn wheel_joint_spin_torque_value(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        Some(self.inv_h * sim.wheel().spin_impulse)
    }

    /// The steering angle in radians.
    pub fn wheel_joint_steering_angle(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        let body_a = self.bodies[sim.body_id_a as usize].clone();
        let sim_a =
            self.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
        let body_b = self.bodies[sim.body_id_b as usize].clone();
        let sim_b =
            self.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];

        let quat_a = mul_quat(sim_a.transform.q, sim.local_frame_a.q);
        let quat_b = mul_quat(sim_b.transform.q, sim.local_frame_b.q);
        let matrix_a = make_matrix_from_quat(quat_a);
        let matrix_b = make_matrix_from_quat(quat_b);

        // Twist around the x-axis.
        let cs = dot(matrix_b.cz, matrix_a.cz);
        let ss = -dot(matrix_b.cz, matrix_a.cy);
        Some(atan2_approx(ss, cs))
    }

    /// The steering torque in newton-meters.
    pub fn wheel_joint_steering_torque(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::WheelJoint)?;
        Some(self.inv_h * sim.wheel().steering_spring_impulse)
    }

    pub fn wheel_joint_steering_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().steering_hertz)
    }

    pub fn set_wheel_joint_steering_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.steering_hertz = hertz;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_steering_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().steering_damping_ratio)
    }

    pub fn set_wheel_joint_steering_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WheelJoint, move |sim| {
            let mut joint = sim.wheel();
            joint.steering_damping_ratio = ratio;
            sim.set_wheel(joint);
        })
    }

    pub fn wheel_joint_lower_steering_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().lower_steering_limit)
    }

    pub fn wheel_joint_upper_steering_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WheelJoint)
            .map(|sim| sim.wheel().upper_steering_limit)
    }
}

// MARK: - Remaining per-type accessors

impl World {
    // MARK: Spherical

    pub fn spherical_joint_cone_angle(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().cone_angle)
    }

    pub fn is_spherical_joint_cone_limit_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().enable_cone_limit)
            .unwrap_or(false)
    }

    pub fn is_spherical_joint_twist_limit_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().enable_twist_limit)
            .unwrap_or(false)
    }

    pub fn is_spherical_joint_motor_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().enable_motor)
            .unwrap_or(false)
    }

    pub fn is_spherical_joint_spring_enabled(&self, joint_id: JointID) -> bool {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().enable_spring)
            .unwrap_or(false)
    }

    pub fn spherical_joint_lower_twist_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().lower_twist_angle)
    }

    pub fn spherical_joint_upper_twist_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().upper_twist_angle)
    }

    pub fn spherical_joint_spring_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().hertz)
    }

    pub fn set_spherical_joint_spring_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.hertz = hertz;
            sim.set_spherical(joint);
        })
    }

    pub fn spherical_joint_spring_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::SphericalJoint)
            .map(|sim| sim.spherical().damping_ratio)
    }

    pub fn set_spherical_joint_spring_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::SphericalJoint, move |sim| {
            let mut joint = sim.spherical();
            joint.damping_ratio = ratio;
            sim.set_spherical(joint);
        })
    }

    /// The current twist angle about the cone axis.
    pub fn spherical_joint_twist_angle(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::SphericalJoint)?;
        let xf_a = self.body_transform_quick(sim.body_id_a);
        let xf_b = self.body_transform_quick(sim.body_id_b);
        let q_a = mul_quat(xf_a.q, sim.local_frame_a.q);
        let q_b = mul_quat(xf_b.q, sim.local_frame_b.q);
        let rel = inv_mul_quat(q_a, q_b);
        Some(2.0 * atan2_approx(length(rel.v), abs_float(rel.s)))
    }

    /// The motor torque in newton-meters.
    pub fn spherical_joint_motor_torque(&self, joint_id: JointID) -> Option<Vec3> {
        let sim = self.joint_sim_of_type(joint_id, JointType::SphericalJoint)?;
        Some(self.inv_h * sim.spherical().motor_impulse)
    }

    // MARK: Motor

    pub fn motor_joint_linear_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().linear_hertz)
    }

    pub fn set_motor_joint_linear_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.linear_hertz = hertz;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_linear_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().linear_damping_ratio)
    }

    pub fn set_motor_joint_linear_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.linear_damping_ratio = ratio;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_angular_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().angular_hertz)
    }

    pub fn set_motor_joint_angular_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.angular_hertz = hertz;
            sim.set_motor(joint);
        })
    }

    pub fn motor_joint_angular_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::MotorJoint)
            .map(|sim| sim.motor().angular_damping_ratio)
    }

    pub fn set_motor_joint_angular_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::MotorJoint, move |sim| {
            let mut joint = sim.motor();
            joint.angular_damping_ratio = ratio;
            sim.set_motor(joint);
        })
    }

    // MARK: Weld

    pub fn weld_joint_linear_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WeldJoint)
            .map(|sim| sim.weld().linear_hertz)
    }

    pub fn set_weld_joint_linear_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WeldJoint, move |sim| {
            let mut joint = sim.weld();
            joint.linear_hertz = hertz;
            sim.set_weld(joint);
        })
    }

    pub fn weld_joint_linear_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WeldJoint)
            .map(|sim| sim.weld().linear_damping_ratio)
    }

    pub fn set_weld_joint_linear_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WeldJoint, move |sim| {
            let mut joint = sim.weld();
            joint.linear_damping_ratio = ratio;
            sim.set_weld(joint);
        })
    }

    pub fn weld_joint_angular_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WeldJoint)
            .map(|sim| sim.weld().angular_hertz)
    }

    pub fn set_weld_joint_angular_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WeldJoint, move |sim| {
            let mut joint = sim.weld();
            joint.angular_hertz = hertz;
            sim.set_weld(joint);
        })
    }

    pub fn weld_joint_angular_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::WeldJoint)
            .map(|sim| sim.weld().angular_damping_ratio)
    }

    pub fn set_weld_joint_angular_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::WeldJoint, move |sim| {
            let mut joint = sim.weld();
            joint.angular_damping_ratio = ratio;
            sim.set_weld(joint);
        })
    }

    // MARK: Parallel

    pub fn parallel_joint_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::ParallelJoint)
            .map(|sim| sim.parallel().hertz)
    }

    pub fn set_parallel_joint_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::ParallelJoint, move |sim| {
            let mut joint = sim.parallel();
            joint.hertz = hertz;
            sim.set_parallel(joint);
        })
    }

    pub fn parallel_joint_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::ParallelJoint)
            .map(|sim| sim.parallel().damping_ratio)
    }

    pub fn set_parallel_joint_damping_ratio(&mut self, joint_id: JointID, ratio: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::ParallelJoint, move |sim| {
            let mut joint = sim.parallel();
            joint.damping_ratio = ratio;
            sim.set_parallel(joint);
        })
    }
}

// MARK: - Limit and enable accessors

impl World {
    // MARK: Revolute

    pub fn revolute_joint_lower_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().lower_angle)
    }

    pub fn revolute_joint_upper_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().upper_angle)
    }

    pub fn revolute_joint_target_angle(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::RevoluteJoint)
            .map(|sim| sim.revolute().target_angle)
    }

    pub fn set_revolute_joint_target_angle(&mut self, joint_id: JointID, angle: f32) -> bool {
        self.mutate_joint_sim_of_type(joint_id, JointType::RevoluteJoint, move |sim| {
            let mut joint = sim.revolute();
            joint.target_angle = angle;
            sim.set_revolute(joint);
        })
    }

    // MARK: Prismatic

    pub fn prismatic_joint_lower_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().lower_translation)
    }

    pub fn prismatic_joint_upper_limit(&self, joint_id: JointID) -> Option<f32> {
        self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)
            .map(|sim| sim.prismatic().upper_translation)
    }

    /// The current translation along the axis.
    pub fn prismatic_joint_translation_value(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)?;
        let xf_a = self.body_transform_quick(sim.body_id_a);
        let xf_b = self.body_transform_quick(sim.body_id_b);

        let axis_a = rotate_vector(xf_a.q, rotate_vector(sim.local_frame_a.q, Vec3::AXIS_X));
        let anchor_a = rotate_vector(xf_a.q, sim.local_frame_a.p);
        let anchor_b = rotate_vector(xf_b.q, sim.local_frame_b.p);
        let d = sub_pos(xf_b.p, xf_a.p) + (anchor_b - anchor_a);
        Some(dot(d, axis_a))
    }

    /// The current translation rate along the axis.
    pub fn prismatic_joint_speed_value(&self, joint_id: JointID) -> Option<f32> {
        let sim = self.joint_sim_of_type(joint_id, JointType::PrismaticJoint)?;
        let body_a = self.bodies[sim.body_id_a as usize].clone();
        let sim_a =
            self.solver_sets[body_a.set_index as usize].body_sims[body_a.local_index as usize];
        let body_b = self.bodies[sim.body_id_b as usize].clone();
        let sim_b =
            self.solver_sets[body_b.set_index as usize].body_sims[body_b.local_index as usize];

        let q_a = sim_a.transform.q;
        let q_b = sim_b.transform.q;

        let axis_a = rotate_vector(q_a, rotate_vector(sim.local_frame_a.q, Vec3::AXIS_X));
        let r_a = rotate_vector(q_a, sim.local_frame_a.p - sim_a.local_center);
        let r_b = rotate_vector(q_b, sim.local_frame_b.p - sim_b.local_center);

        let d = sub_pos(sim_b.center, sim_a.center) + (r_b - r_a);

        let v_a = self.body_linear_velocity_at(sim.body_id_a);
        let v_b = self.body_linear_velocity_at(sim.body_id_b);
        let w_a = self.body_angular_velocity_at(sim.body_id_a);
        let w_b = self.body_angular_velocity_at(sim.body_id_b);

        let v_rel = (v_b + cross(w_b, r_b)) - (v_a + cross(w_a, r_a));

        // The axis moves with body A, so account for its rotation.
        Some(dot(d, cross(w_a, axis_a)) + dot(axis_a, v_rel))
    }

    // MARK: Distance
}

// MARK: - Parallel spring aliases

impl World {
    /// The parallel joint's spring frequency.
    pub fn parallel_joint_spring_hertz(&self, joint_id: JointID) -> Option<f32> {
        self.parallel_joint_hertz(joint_id)
    }

    /// Set the parallel joint's spring frequency.
    pub fn set_parallel_joint_spring_hertz(&mut self, joint_id: JointID, hertz: f32) -> bool {
        self.set_parallel_joint_hertz(joint_id, hertz)
    }

    /// The parallel joint's spring damping ratio.
    pub fn parallel_joint_spring_damping_ratio(&self, joint_id: JointID) -> Option<f32> {
        self.parallel_joint_damping_ratio(joint_id)
    }

    /// Set the parallel joint's spring damping ratio.
    pub fn set_parallel_joint_spring_damping_ratio(
        &mut self,
        joint_id: JointID,
        ratio: f32,
    ) -> bool {
        self.set_parallel_joint_damping_ratio(joint_id, ratio)
    }
}

// MARK: - Joint reaction and events

impl World {
    /// The magnitude of a joint's reaction force and torque.
    pub(crate) fn joint_reaction(&self, sim: JointSim, inv_time_step: f32) -> (f32, f32) {
        let mut linear_impulse: f32 = 0.0;
        let mut angular_impulse: f32 = 0.0;

        match sim.type_ {
            JointType::ParallelJoint => {
                let joint = sim.parallel();
                let impulse = Vec3::new(joint.perp_impulse.x, joint.perp_impulse.y, 0.0);
                angular_impulse = length(impulse);
            }

            JointType::DistanceJoint => {
                let joint = sim.distance();
                linear_impulse = abs_float(
                    joint.impulse + joint.lower_impulse - joint.upper_impulse + joint.motor_impulse,
                );
            }

            JointType::MotorJoint => {
                let joint = sim.motor();
                linear_impulse =
                    length(joint.linear_velocity_impulse + joint.linear_spring_impulse);
                angular_impulse =
                    length(joint.angular_velocity_impulse + joint.angular_spring_impulse);
            }

            JointType::PrismaticJoint => {
                let joint = sim.prismatic();
                let impulse = Vec3::new(
                    joint.motor_impulse + joint.lower_impulse - joint.upper_impulse,
                    joint.perp_impulse.x,
                    joint.perp_impulse.y,
                );
                linear_impulse = length(impulse);
                angular_impulse = length(joint.angular_impulse);
            }

            JointType::RevoluteJoint => {
                let joint = sim.revolute();
                linear_impulse = length(joint.linear_impulse);
                let impulse = Vec3::new(
                    joint.perp_impulse.x,
                    joint.perp_impulse.y,
                    joint.motor_impulse + joint.lower_impulse - joint.upper_impulse,
                );
                angular_impulse = length(impulse);
            }

            JointType::SphericalJoint => {
                let joint = sim.spherical();
                linear_impulse = length(joint.linear_impulse);

                let xf_a = self.body_transform_quick(sim.body_id_a);
                let xf_b = self.body_transform_quick(sim.body_id_b);
                let q_a = mul_quat(xf_a.q, sim.local_frame_a.q);
                let q_b = mul_quat(xf_b.q, sim.local_frame_b.q);

                // Cone axis is the z-axis of body A.
                let cone_axis = rotate_vector(q_a, Vec3::AXIS_Z);
                let twist_axis = rotate_vector(q_b, Vec3::AXIS_Z);
                let swing_axis = normalize(cross(cone_axis, twist_axis));

                let mut impulse = joint.spring_impulse + joint.motor_impulse;
                impulse = mul_add(
                    impulse,
                    joint.lower_twist_impulse - joint.upper_twist_impulse,
                    twist_axis,
                );
                impulse = mul_add(impulse, joint.swing_impulse, swing_axis);
                angular_impulse = length(impulse);
            }

            JointType::WeldJoint => {
                let joint = sim.weld();
                linear_impulse = length(joint.linear_impulse);
                angular_impulse = length(joint.angular_impulse);
            }

            JointType::WheelJoint => {
                let joint = sim.wheel();
                let axial_impulse = joint.suspension_spring_impulse
                    + joint.lower_suspension_limit
                    - joint.upper_suspension_impulse;
                linear_impulse = (joint.linear_impulse.x * joint.linear_impulse.x
                    + joint.linear_impulse.y * joint.linear_impulse.y
                    + axial_impulse * axial_impulse)
                    .sqrt();
                angular_impulse = abs_float(joint.spin_impulse);
            }

            JointType::FilterJoint => {}

            JointType::GearJoint
            | JointType::RackAndPinionJoint
            | JointType::PulleyJoint
            | JointType::SixDOFJoint
            | JointType::PathJoint => {
                (linear_impulse, angular_impulse) = self.coupling_joint_impulses(sim);
            }
        }

        (linear_impulse * inv_time_step, angular_impulse * inv_time_step)
    }

    /// Collect joint events for joints whose reaction exceeds their threshold.
    pub(crate) fn emit_joint_events(&mut self, inv_time_step: f32) {
        self.joint_events.clear();

        for index in 0..self.joints.len() as i32 {
            let record = self.joints[index as usize].clone();
            if record.joint_id == b3::NULL_INDEX {
                continue;
            }
            if record.set_index != SetType::AwakeSet.raw_value() {
                continue;
            }

            let sim = self.read_joint_sim(index);
            if !(sim.force_threshold < f32::MAX || sim.torque_threshold < f32::MAX) {
                continue;
            }

            let reaction = self.joint_reaction(sim, inv_time_step);

            // A zero threshold reports every awake joint.
            if reaction.0 >= sim.force_threshold || reaction.1 >= sim.torque_threshold {
                let mut event = JointEvent::default();
                event.joint_id =
                    JointID::new(index + 1, self.world_id, record.generation);
                event.user_data = record.user_data;
                self.joint_events.push(event);
            }
        }
    }

    /// Joint events from the most recent step.
    pub fn joint_event_list(&self) -> Vec<JointEvent> {
        self.joint_events.clone()
    }
}
