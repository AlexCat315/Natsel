//! Body API: transforms, velocities, forces, impulses, and mass properties.
//!
//! Translated from `Sources/Natsel/Simulation/BodyAPI.swift`.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::math::functions::*;
use crate::math::types::{Matrix3, Pos, Quat, Vec3, WorldTransform};
use crate::simulation::shape_runtime::{
    compute_fat_shape_aabb, compute_shape_extent, compute_shape_mass,
};
use crate::simulation::world_runtime::{BodyFlags, BodySim, BodyState, SetType, World};
use crate::types::body_def::BodyType;
use crate::types::geometry::MassData;
use crate::types::ids::BodyID;

// MARK: - Contact flags

/// Contact flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct ContactFlags(pub u32);

impl ContactFlags {
    pub const TOUCHING: ContactFlags = ContactFlags(0x0000_0001);
    pub const HIT_EVENT: ContactFlags = ContactFlags(0x0000_0002);
    pub const ENABLE_CONTACT_EVENTS: ContactFlags = ContactFlags(0x0000_0004);
    pub const STATIC_FLAG: ContactFlags = ContactFlags(0x0000_0008);
    pub const RECYCLE: ContactFlags = ContactFlags(0x0000_0010);

    pub const SIM_TOUCHING: ContactFlags = ContactFlags(0x0001_0000);
    pub const SIM_DISJOINT: ContactFlags = ContactFlags(0x0002_0000);
    pub const SIM_STARTED_TOUCHING: ContactFlags = ContactFlags(0x0004_0000);
    pub const SIM_STOPPED_TOUCHING: ContactFlags = ContactFlags(0x0008_0000);
    pub const SIM_ENABLE_HIT_EVENT: ContactFlags = ContactFlags(0x0010_0000);
    pub const SIM_ENABLE_PRE_SOLVE_EVENTS: ContactFlags = ContactFlags(0x0020_0000);
    pub const SIM_MESH_CONTACT: ContactFlags = ContactFlags(0x0040_0000);
    pub const RELATIVE_TRANSFORM_VALID: ContactFlags = ContactFlags(0x0080_0000);
    pub const ENABLE_SPECULATIVE_POINTS: ContactFlags = ContactFlags(0x0100_0000);

    /// Mirrors `OptionSet.contains`.
    #[inline(always)]
    pub fn contains(self, other: ContactFlags) -> bool {
        self.0 & other.0 == other.0
    }

    /// Mirrors `OptionSet.insert`.
    #[inline(always)]
    pub fn insert(&mut self, other: ContactFlags) {
        self.0 |= other.0;
    }

    /// Mirrors `OptionSet.remove`.
    #[inline(always)]
    pub fn remove(&mut self, other: ContactFlags) {
        self.0 &= !other.0;
    }

    /// Mirrors `OptionSet.toggle`.
    #[inline(always)]
    pub fn toggle(&mut self, other: ContactFlags) {
        self.0 ^= other.0;
    }
}

impl std::ops::BitOr for ContactFlags {
    type Output = ContactFlags;
    #[inline(always)]
    fn bitor(self, rhs: ContactFlags) -> ContactFlags {
        ContactFlags(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for ContactFlags {
    #[inline(always)]
    fn bitor_assign(&mut self, rhs: ContactFlags) {
        self.0 |= rhs.0;
    }
}

impl std::ops::BitAnd for ContactFlags {
    type Output = ContactFlags;
    #[inline(always)]
    fn bitand(self, rhs: ContactFlags) -> ContactFlags {
        ContactFlags(self.0 & rhs.0)
    }
}

impl std::ops::BitAndAssign for ContactFlags {
    #[inline(always)]
    fn bitand_assign(&mut self, rhs: ContactFlags) {
        self.0 &= rhs.0;
    }
}

impl std::ops::BitXor for ContactFlags {
    type Output = ContactFlags;
    #[inline(always)]
    fn bitxor(self, rhs: ContactFlags) -> ContactFlags {
        ContactFlags(self.0 ^ rhs.0)
    }
}

impl std::ops::BitXorAssign for ContactFlags {
    #[inline(always)]
    fn bitxor_assign(&mut self, rhs: ContactFlags) {
        self.0 ^= rhs.0;
    }
}

impl std::ops::Not for ContactFlags {
    type Output = ContactFlags;
    #[inline(always)]
    fn not(self) -> ContactFlags {
        ContactFlags(!self.0)
    }
}

// MARK: - World body API

impl World {
    // MARK: Helpers

    #[inline(always)]
    pub(crate) fn body_sim_for(&self, id: BodyID) -> BodySim {
        let body = self.body(id);
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize]
    }

    #[inline(always)]
    pub(crate) fn set_body_sim(&mut self, id: BodyID, sim: BodySim) {
        let body = self.body(id);
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize] = sim;
    }

    #[inline(always)]
    pub(crate) fn set_body_state(&mut self, id: BodyID, state: BodyState) {
        let body = self.body(id);
        if body.set_index != SetType::AwakeSet.raw_value() {
            return;
        }
        self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
            [body.local_index as usize] = state;
    }

    /// Swift read the value-type sim out of the set, ran the closure on the mutable copy, and wrote
    /// the copy back to the same slot; mutating the slot in place is the same single move.
    #[inline(always)]
    pub(crate) fn with_body_sim<R, F>(&mut self, id: BodyID, f: F) -> R
    where
        F: FnOnce(&mut BodySim) -> R,
    {
        let body = self.body(id);
        let set_index = body.set_index as usize;
        let local_index = body.local_index as usize;
        let sim = &mut self.solver_sets[set_index].body_sims[local_index];
        f(sim)
    }

    #[inline(always)]
    pub(crate) fn with_body_state<R, F>(&mut self, id: BodyID, f: F) -> Option<R>
    where
        F: FnOnce(&mut BodyState) -> R,
    {
        let body = self.body(id);
        if body.set_index != SetType::AwakeSet.raw_value() {
            return None;
        }
        // Only the awake set carries body states, so the slot below is the one `bodyState` read.
        let local_index = body.local_index as usize;
        let slot = &mut self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
            [local_index];
        Some(f(slot))
    }

    // MARK: Transforms

    pub fn body_position(&self, id: BodyID) -> Pos {
        self.body_sim_for(id).transform.p
    }

    pub fn body_rotation(&self, id: BodyID) -> Quat {
        self.body_sim_for(id).transform.q
    }

    pub fn body_transform(&self, id: BodyID) -> WorldTransform {
        self.body_sim_for(id).transform
    }

    pub fn body_local_point(&self, id: BodyID, world_point: Pos) -> Vec3 {
        inv_transform_world_point(self.body_transform(id), world_point)
    }

    pub fn body_world_point(&self, id: BodyID, local_point: Vec3) -> Pos {
        transform_world_point(self.body_transform(id), local_point)
    }

    pub fn body_local_vector(&self, id: BodyID, world_vector: Vec3) -> Vec3 {
        inv_rotate_vector(self.body_transform(id).q, world_vector)
    }

    pub fn body_world_vector(&self, id: BodyID, local_vector: Vec3) -> Vec3 {
        rotate_vector(self.body_transform(id).q, local_vector)
    }

    pub fn set_body_transform(&mut self, id: BodyID, position: Pos, rotation: Quat) {
        b3_assert(is_valid_position(position));
        b3_assert(is_valid_quat(rotation));
        b3_assert(self.is_valid_body(id));
        b3_assert(self.locked == false);

        let mut sim = self.body_sim_for(id);
        sim.transform.p = position;
        sim.transform.q = rotation;
        sim.center = transform_world_point(sim.transform, sim.local_center);

        let rotation_matrix = make_matrix_from_quat(sim.transform.q);
        sim.inv_inertia_world =
            mul_mm(mul_mm(rotation_matrix, sim.inv_inertia_local), transpose(rotation_matrix));

        sim.rotation0 = sim.transform.q;
        sim.center0 = sim.center;
        self.set_body_sim(id, sim);

        let transform = sim.transform;
        let speculative_distance = b3::speculative_distance();

        let body_index = self.body_index(id);
        let mut shape_id = self.bodies[body_index as usize].head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let mut shape = self.shapes[shape_id as usize].clone();
            let aabb = compute_fat_shape_aabb(&shape, transform, speculative_distance);
            shape.aabb = aabb;

            let fat_aabb = self.fat_aabbs[shape_id as usize];
            if aabb_contains(fat_aabb, aabb) == false {
                let inflated = aabb_inflate(aabb, shape.aabb_margin);
                self.fat_aabbs[shape_id as usize] = inflated;

                // The body could be disabled
                if shape.proxy_key != b3::NULL_INDEX {
                    self.broad_phase.move_proxy(shape.proxy_key, inflated);
                }
            }

            let next_shape_id = shape.next_shape_id;
            self.shapes[shape_id as usize] = shape;
            shape_id = next_shape_id;
        }
    }

    // MARK: Velocities

    pub fn body_linear_velocity(&self, id: BodyID) -> Vec3 {
        let body = self.body(id);
        match self.body_state(&body) {
            Some(state) => state.linear_velocity,
            None => Vec3::ZERO,
        }
    }

    pub fn body_angular_velocity(&self, id: BodyID) -> Vec3 {
        let body = self.body(id);
        match self.body_state(&body) {
            Some(state) => state.angular_velocity,
            None => Vec3::ZERO,
        }
    }

    pub fn set_body_linear_velocity(&mut self, id: BodyID, linear_velocity: Vec3) {
        b3_assert(is_valid_vec3(linear_velocity));

        let mut record = self.body(id);
        if record.type_ == BodyType::StaticBody {
            return;
        }

        // A non-zero velocity wakes the body first, then the state is written in its new set.
        if length_squared(linear_velocity) > 0.0 {
            self.wake_body(self.body_index(id));
            record = self.body(id);
        }

        if record.set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        let local_index = record.local_index as usize;
        let awake_set = SetType::AwakeSet.raw_value() as usize;
        let state = &mut self.solver_sets[awake_set].body_states[local_index];
        state.linear_velocity = linear_velocity;
    }

    pub fn set_body_angular_velocity(&mut self, id: BodyID, angular_velocity: Vec3) {
        b3_assert(is_valid_vec3(angular_velocity));

        let mut record = self.body(id);
        if record.type_ == BodyType::StaticBody {
            return;
        }

        if length_squared(angular_velocity) > 0.0 {
            self.wake_body(self.body_index(id));
            record = self.body(id);
        }

        if record.set_index != SetType::AwakeSet.raw_value() {
            return;
        }

        let local_index = record.local_index as usize;
        let awake_set = SetType::AwakeSet.raw_value() as usize;
        let state = &mut self.solver_sets[awake_set].body_states[local_index];
        state.angular_velocity = angular_velocity;
    }

    pub fn body_local_point_velocity(&self, id: BodyID, local_point: Vec3) -> Vec3 {
        let sim = self.body_sim_for(id);
        let r = rotate_vector(sim.transform.q, local_point - sim.local_center);
        let body = self.body(id);
        let state = self.body_state(&body).unwrap_or(BodyState::IDENTITY);
        state.linear_velocity + cross(state.angular_velocity, r)
    }

    pub fn body_world_point_velocity(&self, id: BodyID, world_point: Pos) -> Vec3 {
        let sim = self.body_sim_for(id);
        let r = world_point - sim.center;
        let body = self.body(id);
        let state = self.body_state(&body).unwrap_or(BodyState::IDENTITY);
        state.linear_velocity + cross(state.angular_velocity, r)
    }

    // MARK: Forces

    pub fn apply_body_force(&mut self, id: BodyID, force: Vec3, point: Pos, wake: bool) {
        b3_assert(is_valid_vec3(force));
        let body = self.body(id);

        if wake && body.set_index >= SetType::FirstSleepingSet.raw_value() {
            self.wake_body(self.body_index(id));
        }

        if body.set_index == SetType::AwakeSet.raw_value() {
            self.with_body_sim(id, |sim| {
                sim.force = sim.force + force;
                sim.torque = sim.torque + cross(sub_pos(point, sim.center), force);
            });
        }
    }

    pub fn apply_body_force_to_center(&mut self, id: BodyID, force: Vec3, wake: bool) {
        b3_assert(is_valid_vec3(force));
        let body = self.body(id);

        if wake && body.set_index >= SetType::FirstSleepingSet.raw_value() {
            self.wake_body(self.body_index(id));
        }

        if body.set_index == SetType::AwakeSet.raw_value() {
            self.with_body_sim(id, |sim| {
                sim.force = sim.force + force;
            });
        }
    }

    pub fn apply_body_torque(&mut self, id: BodyID, torque: Vec3, wake: bool) {
        b3_assert(is_valid_vec3(torque));
        let body = self.body(id);

        if wake && body.set_index >= SetType::FirstSleepingSet.raw_value() {
            self.wake_body(self.body_index(id));
        }

        if body.set_index == SetType::AwakeSet.raw_value() {
            self.with_body_sim(id, |sim| {
                sim.torque = sim.torque + torque;
            });
        }
    }

    pub fn apply_body_linear_impulse(&mut self, id: BodyID, impulse: Vec3, point: Pos, wake: bool) {
        b3_assert(is_valid_vec3(impulse));
        b3_assert(is_valid_position(point));
        let body = self.body(id);

        if wake && body.set_index >= SetType::FirstSleepingSet.raw_value() {
            self.wake_body(self.body_index(id));
        }

        if body.set_index == SetType::AwakeSet.raw_value() {
            let local_index = body.local_index as usize;
            let set_index = body.set_index as usize;
            let mut state = self.solver_sets[SetType::AwakeSet.raw_value() as usize]
                .body_states
                [local_index];
            let sim = self.solver_sets[set_index].body_sims[local_index];

            state.linear_velocity = mul_add(state.linear_velocity, sim.inv_mass, impulse);

            let max_linear_speed = self.max_linear_speed;
            if length_squared(state.linear_velocity) > max_linear_speed * max_linear_speed {
                state.linear_velocity = max_linear_speed * normalize(state.linear_velocity);
            }

            let delta = mul_mv(sim.inv_inertia_world, cross(sub_pos(point, sim.center), impulse));
            state.angular_velocity = state.angular_velocity + delta;

            self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states[local_index] =
                state;
        }
    }

    pub fn apply_body_linear_impulse_to_center(
        &mut self,
        id: BodyID,
        impulse: Vec3,
        wake: bool,
    ) {
        b3_assert(is_valid_vec3(impulse));
        let body = self.body(id);

        if wake && body.set_index >= SetType::FirstSleepingSet.raw_value() {
            self.wake_body(self.body_index(id));
        }

        if body.set_index == SetType::AwakeSet.raw_value() {
            let local_index = body.local_index as usize;
            let set_index = body.set_index as usize;
            let mut state = self.solver_sets[SetType::AwakeSet.raw_value() as usize]
                .body_states
                [local_index];
            let sim = self.solver_sets[set_index].body_sims[local_index];

            state.linear_velocity = mul_add(state.linear_velocity, sim.inv_mass, impulse);

            let max_linear_speed = self.max_linear_speed;
            if length_squared(state.linear_velocity) > max_linear_speed * max_linear_speed {
                state.linear_velocity = max_linear_speed * normalize(state.linear_velocity);
            }

            self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states[local_index] =
                state;
        }
    }

    /// This variant applies the impulse through the body's *local* inertia tensor, rotated into
    /// world space. `BodyQueries.swift`'s overload of the same Swift name uses
    /// `invInertiaWorld` directly instead, so that one is translated as its own Rust function.
    pub fn apply_body_angular_impulse(&mut self, id: BodyID, impulse: Vec3, wake: bool) {
        b3_assert(is_valid_vec3(impulse));
        b3_assert(self.is_valid_body(id));
        let body = self.body(id);

        if wake && body.set_index >= SetType::FirstSleepingSet.raw_value() {
            self.wake_body(self.body_index(id));
        }

        if body.set_index == SetType::AwakeSet.raw_value() {
            let local_index = body.local_index as usize;
            let set_index = body.set_index as usize;
            let mut state = self.solver_sets[SetType::AwakeSet.raw_value() as usize]
                .body_states
                [local_index];
            let sim = self.solver_sets[set_index].body_sims[local_index];

            let local_impulse = inv_rotate_vector(sim.transform.q, impulse);
            let local_angular_velocity_delta = mul_mv(sim.inv_inertia_local, local_impulse);
            state.angular_velocity =
                state.angular_velocity + rotate_vector(sim.transform.q, local_angular_velocity_delta);

            self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states[local_index] =
                state;
        }
    }

    // MARK: Mass properties

    pub fn body_mass(&self, id: BodyID) -> f32 {
        self.body(id).mass
    }

    pub fn body_local_rotational_inertia(&self, id: BodyID) -> Matrix3 {
        self.body_sim_for(id).inertia
    }

    pub fn body_inverse_mass(&self, id: BodyID) -> f32 {
        self.body_sim_for(id).inv_mass
    }

    pub fn body_world_inverse_rotational_inertia(&self, id: BodyID) -> Matrix3 {
        self.body_sim_for(id).inv_inertia_world
    }

    pub fn body_local_center(&self, id: BodyID) -> Vec3 {
        self.body_sim_for(id).local_center
    }

    pub fn body_world_center(&self, id: BodyID) -> Pos {
        self.body_sim_for(id).center
    }

    pub fn body_mass_data(&self, id: BodyID) -> MassData {
        let sim = self.body_sim_for(id);
        let mut mass_data = MassData::default();
        mass_data.mass = self.body(id).mass;
        mass_data.center = sim.local_center;
        mass_data.inertia = sim.inertia;
        mass_data
    }

    pub fn set_body_mass_data(&mut self, id: BodyID, mass_data: MassData) {
        b3_assert(is_valid_float(mass_data.mass) && mass_data.mass >= 0.0);
        b3_assert(is_valid_matrix3(mass_data.inertia));
        b3_assert(is_valid_vec3(mass_data.center));

        let body_index = self.body_index(id);
        let mut body = self.bodies[body_index as usize].clone();
        let mut sim = self.body_sim_for(id);

        // Mass is no longer dirty
        body.flags &= !BodyFlags::DIRTY_MASS.0;
        self.bodies[body_index as usize] = body.clone();
        self.sync_body_flags(body_index);
        body = self.bodies[body_index as usize].clone();

        body.mass = mass_data.mass;
        self.bodies[body_index as usize] = body.clone();
        sim.local_center = mass_data.center;

        let old_center = sim.center;
        let center = transform_world_point(sim.transform, mass_data.center);
        sim.center = center;
        sim.center0 = center;
        sim.inv_mass = if body.mass > 0.0 { 1.0 / body.mass } else { 0.0 };
        sim.inertia = mass_data.inertia;
        self.set_body_sim(id, sim);

        // Update center of mass velocity
        let state = self.body_state(&body);
        if let Some(state) = state {
            let mut new_state = state;
            let delta_linear = cross(state.angular_velocity, sub_pos(sim.center, old_center));
            new_state.linear_velocity = new_state.linear_velocity + delta_linear;
            self.set_body_state(id, new_state);
        }

        let mut new_sim = self.body_sim_for(id);
        let det = det(new_sim.inertia);
        b3_assert(det >= 0.0);

        if det > 0.0 {
            new_sim.inv_inertia_local = invert_t(new_sim.inertia);

            let rotation_matrix = make_matrix_from_quat(new_sim.transform.q);
            new_sim.inv_inertia_world = mul_mm(
                mul_mm(rotation_matrix, new_sim.inv_inertia_local),
                transpose(rotation_matrix),
            );
        } else {
            new_sim.inv_inertia_local = Matrix3::ZERO;
            new_sim.inv_inertia_world = Matrix3::ZERO;
        }

        // Apply fixed rotation
        if (new_sim.flags & BodyFlags::FIXED_ROTATION.0) == BodyFlags::FIXED_ROTATION.0 {
            new_sim.inertia = Matrix3::ZERO;
            new_sim.inv_inertia_local = Matrix3::ZERO;
            new_sim.inv_inertia_world = Matrix3::ZERO;
        }

        self.set_body_sim(id, new_sim);
        self.update_isotropic_inertia(body_index);

        // Update extents using supplied mass center.
        let mut updated_sim = self.body_sim_for(id);
        updated_sim.min_extent = b3::huge();
        updated_sim.max_extent = Vec3::ZERO;

        let mut shape_id = self.bodies[body_index as usize].head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = self.shapes[shape_id as usize].clone();
            let extent = compute_shape_extent(&shape, mass_data.center);
            updated_sim.min_extent = min_float(updated_sim.min_extent, extent.min_extent);
            updated_sim.max_extent = max_vec3(updated_sim.max_extent, extent.max_extent);
            shape_id = shape.next_shape_id;
        }

        self.set_body_sim(id, updated_sim);

        // When the center of mass changes, any cached contact manifold becomes invalid.
        let mut edge_key = self.bodies[body_index as usize].head_contact_key;
        while edge_key != b3::NULL_INDEX {
            let contact_id = edge_key >> 1;
            let edge_index = (edge_key & 1) as usize;

            let mut contact = self.contacts[contact_id as usize];
            contact.flags &= !ContactFlags::RELATIVE_TRANSFORM_VALID.0;
            self.contacts[contact_id as usize] = contact;

            edge_key = contact.edges[edge_index].next_key;
        }
    }

    pub fn apply_mass_from_shapes(&mut self, id: BodyID) {
        let body_index = self.body_index(id);
        self.update_body_mass_data(body_index);
    }

    // MARK: Mass computation

    pub(crate) fn update_isotropic_inertia(&mut self, body_index: i32) {
        let mut body = self.bodies[body_index as usize].clone();
        let mut sim = self.body_sim(&body);

        let inertia = sim.inertia;
        let mean = (inertia.cx.x + inertia.cy.y + inertia.cz.z) / 3.0;
        let dx = inertia.cx.x - mean;
        let dy = inertia.cy.y - mean;
        let dz = inertia.cz.z - mean;
        let off_diagonal = inertia.cy.x * inertia.cy.x
            + inertia.cz.x * inertia.cz.x
            + inertia.cz.y * inertia.cz.y;

        // This is the square of the Frobenius matrix norm.
        let deviation = dx * dx + dy * dy + dz * dz + 2.0 * off_diagonal;

        let tolerance = self.isotropic_inertia_tolerance;

        if 2.0 * deviation > 3.0 * tolerance * tolerance * mean * mean {
            body.flags |= BodyFlags::ENABLE_GYROSCOPIC_TORQUE.0;
        } else {
            body.flags &= !BodyFlags::ENABLE_GYROSCOPIC_TORQUE.0;
        }

        self.bodies[body_index as usize] = body.clone();
        sim = self.body_sim(&body);
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize] = sim;

        self.sync_body_flags(body_index);
    }

    pub(crate) fn update_body_mass_data(&mut self, body_index: i32) {
        let mut body = self.bodies[body_index as usize].clone();

        // Mass is no longer dirty
        body.flags &= !BodyFlags::DIRTY_MASS.0;
        body.flags &= !BodyFlags::ENABLE_GYROSCOPIC_TORQUE.0;
        self.bodies[body_index as usize] = body.clone();
        self.sync_body_flags(body_index);
        body = self.bodies[body_index as usize].clone();

        // Compute mass data from shapes. Each shape has its own density.
        body.mass = 0.0;
        let mut sim = self.body_sim(&body);
        sim.inertia = Matrix3::ZERO;
        sim.inv_mass = 0.0;
        sim.inv_inertia_local = Matrix3::ZERO;
        sim.inv_inertia_world = Matrix3::ZERO;
        sim.local_center = Vec3::ZERO;
        sim.min_extent = b3::huge();
        sim.max_extent = Vec3::ZERO;
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize] = sim;

        if body.head_shape_id == b3::NULL_INDEX {
            self.bodies[body_index as usize] = body;
            return;
        }

        // Static and kinematic sims have zero mass.
        if body.type_ != BodyType::DynamicBody {
            sim.center = sim.transform.p;
            sim.center0 = sim.center;

            // Need extents for kinematic bodies for sleeping to work correctly.
            if body.type_ == BodyType::KinematicBody {
                let mut shape_id = body.head_shape_id;
                while shape_id != b3::NULL_INDEX {
                    let s = self.shapes[shape_id as usize].clone();

                    let extent = compute_shape_extent(&s, Vec3::ZERO);
                    sim.min_extent = min_float(sim.min_extent, extent.min_extent);
                    sim.max_extent = max_vec3(sim.max_extent, extent.max_extent);

                    shape_id = s.next_shape_id;
                }
            }

            self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize] = sim;
            self.bodies[body_index as usize] = body;
            return;
        }

        let shape_count = body.shape_count as usize;
        let mut masses = vec![MassData::default(); shape_count];

        // Accumulate mass over all shapes.
        let mut local_center = Vec3::ZERO;
        let mut shape_id = body.head_shape_id;
        let mut shape_index = 0usize;
        while shape_id != b3::NULL_INDEX {
            let s = self.shapes[shape_id as usize].clone();
            shape_id = s.next_shape_id;

            if s.density == 0.0 {
                masses[shape_index] = MassData::default();
                shape_index += 1;
                continue;
            }

            let mass_data = compute_shape_mass(&s);
            body.mass += mass_data.mass;
            local_center = mul_add(local_center, mass_data.mass, mass_data.center);

            masses[shape_index] = mass_data;
            shape_index += 1;
        }

        // Compute center of mass.
        if body.mass > 0.0 {
            sim.inv_mass = 1.0 / body.mass;
            local_center = sim.inv_mass * local_center;
        }

        // Second loop to accumulate the rotational inertia about the center of mass
        for mass_data in masses.iter() {
            if mass_data.mass == 0.0 {
                continue;
            }

            // Shift to center of mass. This is safe because it can only increase.
            let offset = local_center - mass_data.center;
            let inertia = add_mm(mass_data.inertia, steiner(mass_data.mass, offset));
            sim.inertia = add_mm(sim.inertia, inertia);
        }

        let determinant = det(sim.inertia);
        b3_assert(determinant >= 0.0);

        if determinant > 0.0 {
            // This call is faster than invert
            sim.inv_inertia_local = invert_t(sim.inertia);

            let rotation_matrix = make_matrix_from_quat(sim.transform.q);
            sim.inv_inertia_world = mul_mm(
                mul_mm(rotation_matrix, sim.inv_inertia_local),
                transpose(rotation_matrix),
            );
        }

        // Move center of mass.
        let old_center = sim.center;
        sim.local_center = local_center;
        sim.center = transform_world_point(sim.transform, sim.local_center);
        sim.center0 = sim.center;

        // Update center of mass velocity
        let state = self.body_state(&body);
        if let Some(state) = state {
            let mut new_state = state;
            let delta_linear = cross(state.angular_velocity, sub_pos(sim.center, old_center));
            new_state.linear_velocity = new_state.linear_velocity + delta_linear;
            self.solver_sets[SetType::AwakeSet.raw_value() as usize].body_states
                [body.local_index as usize] = new_state;
        }

        // Compute body extents relative to center of mass
        shape_id = body.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let s = self.shapes[shape_id as usize].clone();

            let extent = compute_shape_extent(&s, local_center);
            sim.min_extent = min_float(sim.min_extent, extent.min_extent);
            sim.max_extent = max_vec3(sim.max_extent, extent.max_extent);

            shape_id = s.next_shape_id;
        }

        // When the center of mass changes, any cached contact manifold becomes invalid.
        let mut edge_key = body.head_contact_key;
        while edge_key != b3::NULL_INDEX {
            let contact_id = edge_key >> 1;
            let edge_index = (edge_key & 1) as usize;

            let mut contact = self.contacts[contact_id as usize];
            contact.flags &= !ContactFlags::RELATIVE_TRANSFORM_VALID.0;
            self.contacts[contact_id as usize] = contact;

            edge_key = contact.edges[edge_index].next_key;
        }

        // Apply fixed rotation
        if (sim.flags & BodyFlags::FIXED_ROTATION.0) == BodyFlags::FIXED_ROTATION.0 {
            sim.inertia = Matrix3::ZERO;
            sim.inv_inertia_local = Matrix3::ZERO;
            sim.inv_inertia_world = Matrix3::ZERO;
        }

        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize] = sim;
        self.bodies[body_index as usize] = body;

        self.update_isotropic_inertia(body_index);
    }

    // MARK: Damping and gravity

    pub fn set_body_linear_damping(&mut self, id: BodyID, linear_damping: f32) {
        b3_assert(is_valid_float(linear_damping) && linear_damping >= 0.0);
        self.with_body_sim(id, |sim| sim.linear_damping = linear_damping);
    }

    pub fn body_linear_damping(&self, id: BodyID) -> f32 {
        self.body_sim_for(id).linear_damping
    }

    pub fn set_body_angular_damping(&mut self, id: BodyID, angular_damping: f32) {
        b3_assert(is_valid_float(angular_damping) && angular_damping >= 0.0);
        self.with_body_sim(id, |sim| sim.angular_damping = angular_damping);
    }

    pub fn body_angular_damping(&self, id: BodyID) -> f32 {
        self.body_sim_for(id).angular_damping
    }

    pub fn set_body_gravity_scale(&mut self, id: BodyID, gravity_scale: f32) {
        b3_assert(is_valid_float(gravity_scale));
        b3_assert(gravity_scale >= 0.0);
        self.with_body_sim(id, |sim| sim.gravity_scale = gravity_scale);
    }

    pub fn body_gravity_scale(&self, id: BodyID) -> f32 {
        self.body_sim_for(id).gravity_scale
    }

    // MARK: Sleep, safety, extents

    pub fn body_is_awake(&self, id: BodyID) -> bool {
        let body = self.body(id);
        self.is_body_awake(&body)
    }

    pub fn body_is_sleep_enabled(&self, id: BodyID) -> bool {
        (self.body(id).flags & BodyFlags::ENABLE_SLEEP.0) != 0
    }

    pub fn set_body_sleep_threshold(&mut self, id: BodyID, sleep_threshold: f32) {
        b3_assert(is_valid_float(sleep_threshold) && sleep_threshold >= 0.0);
        let mut body = self.body(id);
        body.sleep_threshold = sleep_threshold;
        let index = self.body_index(id) as usize;
        self.bodies[index] = body;
    }

    pub fn body_sleep_threshold(&self, id: BodyID) -> f32 {
        self.body(id).sleep_threshold
    }

    pub fn set_body_safety_factor(&mut self, id: BodyID, safety_factor: f32) {
        b3_assert(is_valid_float(safety_factor) && safety_factor >= 0.0);
        let mut body = self.body(id);
        body.safety_factor = safety_factor;
        let index = self.body_index(id) as usize;
        self.bodies[index] = body;
    }

    pub fn body_safety_factor(&self, id: BodyID) -> f32 {
        self.body(id).safety_factor
    }

    /// Enable or disable sleeping for a body.
    pub fn enable_body_sleep(&mut self, id: BodyID, enable_sleep: bool) {
        let mut body = self.body(id);
        if enable_sleep {
            body.flags |= BodyFlags::ENABLE_SLEEP.0;
        } else {
            body.flags &= !BodyFlags::ENABLE_SLEEP.0;
        }
        let body_index = self.body_index(id);
        self.bodies[body_index as usize] = body;
        self.sync_body_flags(body_index);
    }

    /// Is sleeping enabled for this body?
    pub fn is_body_sleep_enabled(&self, id: BodyID) -> bool {
        (self.body(id).flags & BodyFlags::ENABLE_SLEEP.0) != 0
    }

    pub fn body_min_extent(&self, id: BodyID) -> f32 {
        self.body_sim_for(id).min_extent
    }

    pub fn body_max_extent(&self, id: BodyID) -> Vec3 {
        self.body_sim_for(id).max_extent
    }

    pub fn body_max_extent_origin(&self, id: BodyID) -> Pos {
        self.body_sim_for(id).center
    }

    // MARK: Derived velocity by index

    /// Linear velocity by body index, bypassing id validation. For internal derived queries.
    pub(crate) fn body_linear_velocity_at(&self, body_index: i32) -> Vec3 {
        match self.body_state(&self.bodies[body_index as usize]) {
            Some(state) => state.linear_velocity,
            None => Vec3::ZERO,
        }
    }

    /// Angular velocity by body index, bypassing id validation.
    pub(crate) fn body_angular_velocity_at(&self, body_index: i32) -> Vec3 {
        match self.body_state(&self.bodies[body_index as usize]) {
            Some(state) => state.angular_velocity,
            None => Vec3::ZERO,
        }
    }
}
