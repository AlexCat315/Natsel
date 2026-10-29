//! Island graph maintenance.
//!
//! Translated from `Sources/Natsel/Simulation/Island.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (see MIGRATION.md "Shared simulation
//! vocabulary"):
//! - `crate::simulation::world_runtime`: `World` storage (`bodies`, `contacts`, `joints`,
//!   `islands`), `Island`, `ContactLink`, `JointLink`, `SetType`, and the `remove_swap_array`
//!   swap-removal helper, plus `World::destroy_island`.
//! - `crate::simulation::solver_set_ops`: `World::wake_solver_set`.
//! - `crate::simulation::body_api`: `ContactFlags`.
//!
//! Merge order is load-bearing for determinism: the island with the larger `bodies` array
//! survives (ties keep `island_id_a`), migrated contacts and joints keep the smaller island's
//! link order, and the survivor's `constraint_remove_count` accumulates before the loser is
//! destroyed.

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::world_runtime::{
    ContactLink, JointLink, SetType, World, remove_swap_array,
};
use crate::types::body_def::BodyType;

impl World {
    /// Merge two islands. Keeps the bigger island to reduce cache misses.
    ///
    /// Swift read `islands[...]` through copy-on-write value semantics; the snapshots below
    /// (`clone`) reproduce that behavior exactly - the loops iterate the *pre-merge* contents
    /// of the smaller island while the survivor grows.
    pub(crate) fn merge_islands(&mut self, island_id_a: i32, island_id_b: i32) -> i32 {
        if island_id_a == island_id_b {
            return island_id_a;
        }

        if island_id_a == b3::NULL_INDEX {
            b3_assert(island_id_b != b3::NULL_INDEX);
            return island_id_b;
        }

        if island_id_b == b3::NULL_INDEX {
            b3_assert(island_id_a != b3::NULL_INDEX);
            return island_id_a;
        }

        let big_id: i32;
        let small_id: i32;
        {
            let island_a = &self.islands[island_id_a as usize];
            let island_b = &self.islands[island_id_b as usize];

            if island_a.bodies.len() >= island_b.bodies.len() {
                big_id = island_id_a;
                small_id = island_id_b;
            } else {
                big_id = island_id_b;
                small_id = island_id_a;
            }
        }

        let big_island_id = big_id;
        let small_island = self.islands[small_id as usize].clone();

        // Grow the surviving island in place. Copying it to a local var and appending there made
        // every append copy the whole array (copy-on-write), which turned island construction into
        // O(n^2) data movement for large scenes.
        for body_id in &small_island.bodies {
            let mut body = self.bodies[*body_id as usize].clone();
            b3_validate(body.island_id == small_island.island_id);
            body.island_id = big_island_id;
            body.island_index = self.islands[big_id as usize].bodies.len() as i32;
            self.bodies[*body_id as usize] = body;
            self.islands[big_id as usize].bodies.push(*body_id);
        }

        // Migrate contacts from smaller island to larger island
        if !small_island.contacts.is_empty() {
            for link in &small_island.contacts {
                let mut contact = self.contacts[link.contact_id as usize];
                contact.island_id = big_island_id;
                contact.island_index = self.islands[big_id as usize].contacts.len() as i32;
                self.contacts[link.contact_id as usize] = contact;
                self.islands[big_id as usize].contacts.push(*link);
            }
        }

        // Migrate joints from smaller island to larger island
        if !small_island.joints.is_empty() {
            for link in &small_island.joints {
                let mut joint = self.joints[link.joint_id as usize].clone();
                joint.island_id = big_island_id;
                joint.island_index = self.islands[big_id as usize].joints.len() as i32;
                self.joints[link.joint_id as usize] = joint;
                self.islands[big_id as usize].joints.push(*link);
            }
        }

        // Track removed constraints
        self.islands[big_id as usize].constraint_remove_count +=
            small_island.constraint_remove_count;

        self.destroy_island(small_island.island_id);

        big_island_id
    }

    fn add_contact_to_island(&mut self, island_id: i32, contact_id: i32) {
        let mut contact = self.contacts[contact_id as usize];
        b3_assert(contact.island_id == b3::NULL_INDEX);
        b3_assert(contact.island_index == b3::NULL_INDEX);

        contact.island_id = island_id;
        contact.island_index = self.islands[island_id as usize].contacts.len() as i32;
        self.contacts[contact_id as usize] = contact;

        let mut link = ContactLink::default();
        link.contact_id = contact.contact_id;
        link.body_id_a = contact.edges[0].body_id;
        link.body_id_b = contact.edges[1].body_id;
        self.islands[island_id as usize].contacts.push(link);
    }

    /// Link a touching contact into the island graph.
    pub(crate) fn link_contact(&mut self, contact_id: i32) {
        let contact = self.contacts[contact_id as usize];
        b3_assert((contact.flags & ContactFlags::SIM_TOUCHING.0) != 0);

        let body_id_a = contact.edges[0].body_id;
        let body_id_b = contact.edges[1].body_id;

        let mut body_a = self.bodies[body_id_a as usize].clone();
        let mut body_b = self.bodies[body_id_b as usize].clone();

        b3_assert(
            body_a.set_index != SetType::DisabledSet.raw_value()
                && body_b.set_index != SetType::DisabledSet.raw_value(),
        );
        b3_assert(
            body_a.set_index != SetType::StaticSet.raw_value()
                || body_b.set_index != SetType::StaticSet.raw_value(),
        );

        // Wake bodyB if bodyA is awake and bodyB is sleeping
        if body_a.set_index == SetType::AwakeSet.raw_value()
            && body_b.set_index >= SetType::FirstSleepingSet.raw_value()
        {
            self.wake_solver_set(body_b.set_index);
        }

        // Wake bodyA if bodyB is awake and bodyA is sleeping
        if body_b.set_index == SetType::AwakeSet.raw_value()
            && body_a.set_index >= SetType::FirstSleepingSet.raw_value()
        {
            self.wake_solver_set(body_a.set_index);
        }

        body_a = self.bodies[body_id_a as usize].clone();
        body_b = self.bodies[body_id_b as usize].clone();

        let island_id_a = body_a.island_id;
        let island_id_b = body_b.island_id;

        // Static bodies have null island indices.
        b3_assert(body_a.set_index != SetType::StaticSet.raw_value() || island_id_a == b3::NULL_INDEX);
        b3_assert(body_b.set_index != SetType::StaticSet.raw_value() || island_id_b == b3::NULL_INDEX);
        b3_assert(island_id_a != b3::NULL_INDEX || island_id_b != b3::NULL_INDEX);

        // Merge islands. This will destroy one of the islands.
        let final_island_id = self.merge_islands(island_id_a, island_id_b);

        // Add contact to the island that survived
        self.add_contact_to_island(final_island_id, contact_id);

        let contact = self.contacts[contact_id as usize];
        b3_validate(contact.island_id == final_island_id);
    }

    /// Unlink a contact that no longer has contact points or is being destroyed.
    pub(crate) fn unlink_contact(&mut self, contact_id: i32) {
        let mut contact = self.contacts[contact_id as usize];
        b3_assert(contact.island_id != b3::NULL_INDEX);

        // remove from island
        let island_id = contact.island_id;

        let remove_index = contact.island_index as usize;
        b3_assert(
            0 <= contact.island_index
                && remove_index < self.islands[island_id as usize].contacts.len(),
        );
        b3_assert(
            self.islands[island_id as usize].contacts[remove_index].contact_id == contact.contact_id,
        );

        let moved_index =
            remove_swap_array(&mut self.islands[island_id as usize].contacts, remove_index);
        if moved_index != b3::NULL_INDEX {
            // Fix islandIndex on the contact that was swapped into removeIndex
            let moved_link = self.islands[island_id as usize].contacts[remove_index];
            let mut moved_contact = self.contacts[moved_link.contact_id as usize];
            b3_assert(moved_contact.island_index == moved_index);
            moved_contact.island_index = remove_index as i32;
            self.contacts[moved_link.contact_id as usize] = moved_contact;
        }

        contact.island_id = b3::NULL_INDEX;
        contact.island_index = b3::NULL_INDEX;
        self.contacts[contact_id as usize] = contact;

        self.islands[island_id as usize].constraint_remove_count += 1;
    }
}

impl World {
    fn add_joint_to_island(&mut self, island_id: i32, joint_id: i32) {
        let mut joint = self.joints[joint_id as usize].clone();
        b3_assert(joint.island_id == b3::NULL_INDEX);
        b3_assert(joint.island_index == b3::NULL_INDEX);

        joint.island_id = island_id;
        joint.island_index = self.islands[island_id as usize].joints.len() as i32;
        self.joints[joint_id as usize] = joint;

        let mut link = JointLink::default();
        link.joint_id = joint.joint_id;
        link.body_id_a = joint.edges[0].body_id;
        link.body_id_b = joint.edges[1].body_id;
        self.islands[island_id as usize].joints.push(link);
    }

    /// Link a joint into the island graph.
    pub(crate) fn link_joint(&mut self, joint_id: i32) {
        let joint = self.joints[joint_id as usize].clone();
        let mut body_a = self.bodies[joint.edges[0].body_id as usize].clone();
        let body_b = self.bodies[joint.edges[1].body_id as usize].clone();

        b3_assert(body_a.type_ == BodyType::DynamicBody || body_b.type_ == BodyType::DynamicBody);

        if body_a.set_index == SetType::AwakeSet.raw_value()
            && body_b.set_index >= SetType::FirstSleepingSet.raw_value()
        {
            self.wake_solver_set(body_b.set_index);
        } else if body_b.set_index == SetType::AwakeSet.raw_value()
            && body_a.set_index >= SetType::FirstSleepingSet.raw_value()
        {
            self.wake_solver_set(body_a.set_index);
        }

        body_a = self.bodies[joint.edges[0].body_id as usize].clone();
        let body_b2 = self.bodies[joint.edges[1].body_id as usize].clone();

        let island_id_a = body_a.island_id;
        let island_id_b = body_b2.island_id;

        b3_assert(island_id_a != b3::NULL_INDEX || island_id_b != b3::NULL_INDEX);

        // Merge islands. This will destroy one of the islands.
        let final_island_id = self.merge_islands(island_id_a, island_id_b);

        // Add joint to the island that survived
        self.add_joint_to_island(final_island_id, joint_id);
    }

    /// Unlink a joint from its island.
    pub(crate) fn unlink_joint(&mut self, joint_id: i32) {
        let mut joint = self.joints[joint_id as usize].clone();
        if joint.island_id == b3::NULL_INDEX {
            return;
        }

        // remove from island
        let island_id = joint.island_id;

        let remove_index = joint.island_index as usize;
        b3_assert(
            0 <= joint.island_index && remove_index < self.islands[island_id as usize].joints.len(),
        );
        b3_assert(
            self.islands[island_id as usize].joints[remove_index].joint_id == joint.joint_id,
        );

        let moved_index =
            remove_swap_array(&mut self.islands[island_id as usize].joints, remove_index);
        if moved_index != b3::NULL_INDEX {
            // Fix islandIndex on the joint that was swapped into removeIndex
            let moved_link = self.islands[island_id as usize].joints[remove_index];
            let mut moved_joint = self.joints[moved_link.joint_id as usize].clone();
            b3_assert(moved_joint.island_index == moved_index);
            moved_joint.island_index = remove_index as i32;
            self.joints[moved_link.joint_id as usize] = moved_joint;
        }

        joint.island_id = b3::NULL_INDEX;
        joint.island_index = b3::NULL_INDEX;
        self.joints[joint_id as usize] = joint;

        self.islands[island_id as usize].constraint_remove_count += 1;
    }
}
