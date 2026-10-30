// Solver set transfers, waking, and merging.
//
// Translated from `Sources/Natsel/Simulation/SolverSetOps.swift`.
//
// Swift read a set through a copy-on-write value (`let set = solverSets[i]`) and appended into the
// stored sets. Rust has no implicit copies, so the sets are indexed directly: every read is a short
// borrow that ends before the write it feeds. That is equivalent because none of these passes
// mutate the set they read from until the set is destroyed at the end; only the destination set
// grows. Where Swift's snapshot *was* load-bearing - the `setIndex` fields of the source and target
// sets in `transferBody` and `transferJoint`, which the pass reads after it has mutated both sets -
// the field values are captured up front.

use crate::base::constants as b3;
use crate::base::core::b3_assert;

use crate::simulation::body_api::ContactFlags;
use crate::simulation::world_runtime::{
    remove_swap_array, BodyFlags, BodyState, JointSim, SetType, World,
};

impl World {
    /// Wake a solver set. Does not merge islands.
    ///
    /// Contacts can be in several places:
    /// 1. non-touching contacts in the disabled set
    /// 2. non-touching contacts already in the awake set
    /// 3. touching contacts in the sleeping set
    /// This handles contact types 1 and 3.
    pub(crate) fn wake_solver_set(&mut self, set_index: i32) {
        b3_assert(set_index >= SetType::FirstSleepingSet.raw_value());

        let set = set_index as usize;
        let awake_set_index = SetType::AwakeSet.raw_value() as usize;
        let disabled_set_index = SetType::DisabledSet.raw_value() as usize;
        let awake_value = SetType::AwakeSet.raw_value();
        let disabled_value = SetType::DisabledSet.raw_value();

        // Swift's comment still applies: append directly into the stored sets, a local copy of a
        // set written back at the end would duplicate every element.
        let body_count = self.solver_sets[set].body_sims.len();
        for i in 0..body_count {
            let sim_src = self.solver_sets[set].body_sims[i];
            let body_id = sim_src.body_id as usize;

            let mut body = self.bodies[body_id].clone();
            b3_assert(body.set_index == set_index);
            body.set_index = awake_value;
            body.local_index = self.solver_sets[awake_set_index].body_sims.len() as i32;

            // Reset sleep timer
            body.sleep_time = 0.0;
            let encoded_body_sim_index = body.local_index;
            let mut contact_key = body.head_contact_key;
            let body_flags = body.flags;
            self.bodies[body_id] = body;

            self.solver_sets[awake_set_index].body_sims.push(sim_src);

            let mut state = BodyState::IDENTITY;
            state.flags = body_flags;
            self.solver_sets[awake_set_index].body_states.push(state);

            // Move non-touching contacts from disabled set to awake set.
            while contact_key != b3::NULL_INDEX {
                let edge_index = (contact_key & 1) as usize;
                let contact_id = contact_key >> 1;

                let mut contact = self.contacts[contact_id as usize];
                contact_key = contact.edges[edge_index].next_key;

                if contact.set_index != disabled_value {
                    b3_assert(contact.set_index == awake_value || contact.set_index == set_index);

                    // If this contact is awake but not touching, fix up the body sim index here.
                    if contact.set_index == awake_value {
                        b3_assert(contact.color_index == b3::NULL_INDEX);
                        if edge_index == 0 {
                            contact.encoded_body_sim_a = encoded_body_sim_index;
                        } else {
                            contact.encoded_body_sim_b = encoded_body_sim_index;
                        }
                        self.contacts[contact_id as usize] = contact;
                    }

                    continue;
                }

                let local_index = contact.local_index;

                b3_assert(
                    0 <= local_index
                        && (local_index as usize)
                            < self.solver_sets[disabled_set_index].contact_indices.len(),
                );
                b3_assert(
                    self.solver_sets[disabled_set_index].contact_indices[local_index as usize]
                        == contact_id,
                );

                b3_assert(
                    (contact.flags & ContactFlags::SIM_TOUCHING.0) == 0
                        && contact.manifold_count == 0,
                );

                contact.set_index = awake_value;
                contact.local_index =
                    self.solver_sets[awake_set_index].contact_indices.len() as i32;
                self.solver_sets[awake_set_index].contact_indices.push(contact_id);

                if edge_index == 0 {
                    contact.encoded_body_sim_a = encoded_body_sim_index;
                } else {
                    contact.encoded_body_sim_b = encoded_body_sim_index;
                }

                self.contacts[contact_id as usize] = contact;

                let moved_local_index = remove_swap_array(
                    &mut self.solver_sets[disabled_set_index].contact_indices,
                    local_index as usize,
                );
                if moved_local_index != b3::NULL_INDEX {
                    // fix moved element
                    let moved_contact_index =
                        self.solver_sets[disabled_set_index].contact_indices[local_index as usize];
                    let mut moved_contact = self.contacts[moved_contact_index as usize];
                    b3_assert(moved_contact.local_index == moved_local_index);
                    moved_contact.local_index = local_index;
                    self.contacts[moved_contact_index as usize] = moved_contact;
                }
            }
        }

        // Transfer touching contacts from sleeping set to constraint graph.
        let contact_count = self.solver_sets[set].contact_indices.len();
        for i in 0..contact_count {
            let contact_id = self.solver_sets[set].contact_indices[i];
            let contact = self.contacts[contact_id as usize];
            b3_assert((contact.flags & ContactFlags::SIM_TOUCHING.0) != 0);
            b3_assert(contact.set_index == set_index);
            self.add_contact_to_graph(contact_id);
            // `addContactToGraph` assigned the contact's color and graph index; re-read it.
            let mut contact = self.contacts[contact_id as usize];
            contact.set_index = awake_value;
            self.contacts[contact_id as usize] = contact;
        }

        // Transfer joints from sleeping set to awake set.
        let joint_count = self.solver_sets[set].joint_sims.len();
        for i in 0..joint_count {
            let joint_sim = self.solver_sets[set].joint_sims[i];
            let joint_id = joint_sim.joint_id as usize;
            let joint = self.joints[joint_id].clone();
            b3_assert(joint.set_index == set_index);
            self.add_joint_to_graph(joint_sim, joint_sim.joint_id);
            // `addJointToGraph` assigned the joint's color and graph index; re-read it.
            let mut joint = self.joints[joint_id].clone();
            joint.set_index = awake_value;
            self.joints[joint_id] = joint;
        }

        // Transfer islands from sleeping set to awake set.
        let island_count = self.solver_sets[set].island_sims.len();
        for i in 0..island_count {
            let island_src = self.solver_sets[set].island_sims[i];
            let island = island_src.island_id as usize;
            self.islands[island].set_index = awake_value;
            self.islands[island].local_index =
                self.solver_sets[awake_set_index].island_sims.len() as i32;
            self.solver_sets[awake_set_index].island_sims.push(island_src);
        }

        // Destroy the sleeping set.
        self.destroy_solver_set(set_index);
    }

    // MARK: Merging

    /// Called when joints are created between sets. The sets can continue sleeping if both are
    /// asleep. Islands get merged when the set is woken.
    pub(crate) fn merge_solver_sets(&mut self, set_id1: i32, set_id2: i32) {
        b3_assert(set_id1 >= SetType::FirstSleepingSet.raw_value());
        b3_assert(set_id2 >= SetType::FirstSleepingSet.raw_value());

        let mut big_id = set_id1;
        let mut small_id = set_id2;
        if self.solver_sets[big_id as usize].body_sims.len()
            < self.solver_sets[small_id as usize].body_sims.len()
        {
            std::mem::swap(&mut big_id, &mut small_id);
        }

        let big_set_index = big_id as usize;
        let small_set_index = small_id as usize;

        // transfer bodies
        let body_count = self.solver_sets[small_set_index].body_sims.len();
        for i in 0..body_count {
            let sim_src = self.solver_sets[small_set_index].body_sims[i];
            let body_id = sim_src.body_id as usize;

            let mut body = self.bodies[body_id].clone();
            b3_assert(body.set_index == small_id);
            body.set_index = big_id;
            body.local_index = self.solver_sets[big_set_index].body_sims.len() as i32;
            self.bodies[body_id] = body;

            self.solver_sets[big_set_index].body_sims.push(sim_src);
        }

        // transfer contacts
        let contact_count = self.solver_sets[small_set_index].contact_indices.len();
        for i in 0..contact_count {
            let contact_id = self.solver_sets[small_set_index].contact_indices[i];
            let mut contact = self.contacts[contact_id as usize];
            b3_assert(contact.set_index == small_id);
            contact.set_index = big_id;
            contact.local_index = self.solver_sets[big_set_index].contact_indices.len() as i32;
            self.contacts[contact_id as usize] = contact;
            self.solver_sets[big_set_index].contact_indices.push(contact_id);
        }

        // transfer joints
        let joint_count = self.solver_sets[small_set_index].joint_sims.len();
        for i in 0..joint_count {
            let joint_sim = self.solver_sets[small_set_index].joint_sims[i];
            let joint_id = joint_sim.joint_id as usize;

            let mut joint = self.joints[joint_id].clone();
            b3_assert(joint.set_index == small_id);
            joint.set_index = big_id;
            joint.local_index = self.solver_sets[big_set_index].joint_sims.len() as i32;
            self.joints[joint_id] = joint;

            self.solver_sets[big_set_index].joint_sims.push(joint_sim);
        }

        // transfer islands
        let island_count = self.solver_sets[small_set_index].island_sims.len();
        for i in 0..island_count {
            let island_src = self.solver_sets[small_set_index].island_sims[i];
            let island_id = island_src.island_id as usize;

            self.islands[island_id].set_index = big_id;
            self.islands[island_id].local_index =
                self.solver_sets[big_set_index].island_sims.len() as i32;

            self.solver_sets[big_set_index].island_sims.push(island_src);
        }

        // destroy the merged set
        self.destroy_solver_set(small_id);
    }

    // MARK: Transfers

    /// Move a body between solver sets. The body must have no contacts.
    pub(crate) fn transfer_body(&mut self, target_set_index: i32, source_set_index: i32, body_id: i32) {
        b3_assert(self.bodies[body_id as usize].head_contact_key == b3::NULL_INDEX);

        if target_set_index == source_set_index {
            return;
        }

        let source = source_set_index as usize;
        let target = target_set_index as usize;
        // Swift's `sourceSet` and `targetSet` were snapshots read again after both sets were
        // mutated; only the set ids are read, and no pass here changes them, so capture them once.
        let source_set_id = self.solver_sets[source].set_index;
        let target_set_id = self.solver_sets[target].set_index;
        let awake = SetType::AwakeSet.raw_value();

        let mut body = self.bodies[body_id as usize].clone();

        let source_index = body.local_index;
        let mut target_sim = self.solver_sets[source].body_sims[source_index as usize];

        let target_index = self.solver_sets[target].body_sims.len();
        self.solver_sets[target].body_sims.push(target_sim);

        // Clear transient body flags
        body.flags &= !BodyFlags::BODY_TRANSIENT_FLAGS.0;
        target_sim.flags &= !(BodyFlags::IS_FAST.0 | BodyFlags::BODY_TRANSIENT_FLAGS.0);
        self.solver_sets[target].body_sims[target_index] = target_sim;

        // Remove body sim from the solver set that owns it
        let moved_index =
            remove_swap_array(&mut self.solver_sets[source].body_sims, source_index as usize);
        if moved_index != b3::NULL_INDEX {
            // Fix moved body index
            let moved_id = self.solver_sets[source].body_sims[source_index as usize].body_id;
            let mut moved_body = self.bodies[moved_id as usize].clone();
            b3_assert(moved_body.local_index == moved_index);
            moved_body.local_index = source_index;
            self.bodies[moved_id as usize] = moved_body;

            // The moved body's contacts cache its old awake index, so refresh them.
            if source_set_id == awake {
                self.refresh_body_contact_indices(moved_id);
            }
        }

        if source_set_id == awake {
            remove_swap_array(&mut self.solver_sets[source].body_states, source_index as usize);
        } else if target_set_id == awake {
            let mut state = BodyState::IDENTITY;
            state.flags = body.flags;
            self.solver_sets[target].body_states.push(state);
        }

        body.set_index = target_set_id;
        body.local_index = target_index as i32;
        self.bodies[body_id as usize] = body;
    }

    /// Move a joint between solver sets.
    pub(crate) fn transfer_joint(
        &mut self,
        target_set_index: i32,
        source_set_index: i32,
        joint_id: i32,
    ) {
        if target_set_index == source_set_index {
            return;
        }

        let target = target_set_index as usize;
        let source = source_set_index as usize;
        let source_set_id = self.solver_sets[source].set_index;
        let target_set_id = self.solver_sets[target].set_index;
        let awake = SetType::AwakeSet.raw_value();

        let mut joint = self.joints[joint_id as usize].clone();
        let local_index = joint.local_index;
        let color_index = joint.color_index;
        // Swift still had the joint value after storing it back into the array; Rust moves it, so
        // the edge body ids needed by the source teardown are taken here. Creating or removing a
        // joint in the graph never edits its edges.
        let edge_body_id_a = joint.edges[0].body_id;
        let edge_body_id_b = joint.edges[1].body_id;

        // Retrieve source.
        let source_sim: JointSim = if source_set_id == awake {
            b3_assert(0 <= color_index && color_index < b3::GRAPH_COLOR_COUNT);
            self.constraint_graph.colors[color_index as usize].joint_sims[local_index as usize]
        } else {
            b3_assert(color_index == b3::NULL_INDEX);
            self.solver_sets[source].joint_sims[local_index as usize]
        };

        // Create target and copy. Fix joint.
        if target_set_id == awake {
            self.add_joint_to_graph(source_sim, joint_id);
            // `addJointToGraph` assigned the joint's color and graph index; re-read it.
            joint = self.joints[joint_id as usize].clone();
            joint.set_index = awake;
            self.joints[joint_id as usize] = joint;
        } else {
            joint.set_index = target_set_id;
            joint.local_index = self.solver_sets[target].joint_sims.len() as i32;
            joint.color_index = b3::NULL_INDEX;
            self.joints[joint_id as usize] = joint;

            self.solver_sets[target].joint_sims.push(source_sim);
        }

        // Destroy source.
        if source_set_id == awake {
            self.remove_joint_from_graph(edge_body_id_a, edge_body_id_b, color_index, local_index);
        } else {
            let moved_index =
                remove_swap_array(&mut self.solver_sets[source].joint_sims, local_index as usize);
            if moved_index != b3::NULL_INDEX {
                // fix swapped element
                let moved_id =
                    self.solver_sets[source].joint_sims[local_index as usize].joint_id;
                let mut moved_joint = self.joints[moved_id as usize].clone();
                moved_joint.local_index = local_index;
                self.joints[moved_id as usize] = moved_joint;
            }
        }
    }
}
