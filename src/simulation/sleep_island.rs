//! Island splitting and sleeping.
//!
//! Translated from `Sources/Natsel/Simulation/SleepIsland.swift`.
//!
//! Every scan below keeps Swift's exact order: the union-find arrays are walked by island index,
//! the sleep pass walks the awake set's islands in reverse, and the split candidate tie-break keeps
//! Swift's `>=` / `==` / `<` chain verbatim so results do not depend on worker completion order.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::simulation::body_api::ContactFlags;
use crate::simulation::constraint_graph::OVERFLOW_INDEX;
use crate::simulation::world_runtime::{
    IslandSim, SetType, SolverSet, World, encode_body_sim_index, remove_swap_array,
    sleep_body_sim_index,
};

// MARK: - Union find

/// Swift declared these as `private` `World` methods that touch no world state, so they are
/// module-private functions over the caller's arrays here.
#[inline(always)]
fn island_find_parent(parents: &mut [i32], node: i32) -> i32 {
    let mut node = node;
    while parents[node as usize] != node {
        let grand_parent = parents[parents[node as usize] as usize];
        parents[node as usize] = grand_parent;
        node = grand_parent;
    }
    node
}

#[inline(always)]
fn island_union(
    parents: &mut [i32],
    ranks: &mut [i32],
    node1: i32,
    node2: i32,
    contact_counts: &mut [i32],
    joint_counts: &mut [i32],
) {
    let root1 = island_find_parent(parents, node1);
    let root2 = island_find_parent(parents, node2);
    if root1 != root2 {
        if ranks[root1 as usize] < ranks[root2 as usize] {
            parents[root1 as usize] = root2;
            contact_counts[root2 as usize] += contact_counts[root1 as usize];
            joint_counts[root2 as usize] += joint_counts[root1 as usize];
        } else if ranks[root1 as usize] > ranks[root2 as usize] {
            parents[root2 as usize] = root1;
            contact_counts[root1 as usize] += contact_counts[root2 as usize];
            joint_counts[root1 as usize] += joint_counts[root2 as usize];
        } else {
            parents[root2 as usize] = root1;
            ranks[root1 as usize] += 1;
            contact_counts[root1 as usize] += contact_counts[root2 as usize];
            joint_counts[root1 as usize] += joint_counts[root2 as usize];
        }
    }
}

impl World {
    /// Refresh the encoded body sim index stored in this body's contacts.
    pub(crate) fn refresh_body_contact_indices(&mut self, body_id: i32) {
        let body = self.bodies[body_id as usize].clone();
        let encoded_body_sim_index = encode_body_sim_index(&body);

        let mut contact_key = body.head_contact_key;
        while contact_key != b3::NULL_INDEX {
            let edge_index = (contact_key & 1) as usize;
            let contact_id = contact_key >> 1;

            let mut contact = self.contacts[contact_id as usize];
            contact_key = contact.edges[edge_index].next_key;

            if edge_index == 0 {
                contact.encoded_body_sim_a = encoded_body_sim_index;
            } else {
                contact.encoded_body_sim_b = encoded_body_sim_index;
            }
            self.contacts[contact_id as usize] = contact;
        }
    }

    /// Split an island because some contacts and/or joints have been removed. Uses union find.
    pub(crate) fn split_island(&mut self, base_id: i32) {
        // Swift read the island struct (value semantics) and then took three copy-on-write array
        // references out of it; moving the buffers out of the owned clone below is the same, and
        // they stay alive while the stored island is detached and destroyed.
        let base_island = self.islands[base_id as usize].clone();
        b3_assert(base_island.constraint_remove_count > 0);
        b3_assert(base_island.set_index == SetType::AwakeSet.raw_value());

        let base_body_ids = base_island.bodies;
        let base_contacts = base_island.contacts;
        let base_joints = base_island.joints;

        let base_body_count = base_body_ids.len();
        let base_contact_count = base_contacts.len();
        let base_joint_count = base_joints.len();

        let mut parents = vec![0i32; base_body_count];
        let mut ranks = vec![0i32; base_body_count];
        let mut contact_counts = vec![0i32; base_body_count];
        let mut joint_counts = vec![0i32; base_body_count];
        for i in 0..base_body_count {
            parents[i] = i as i32;
        }

        // Union over contacts, tracking per-component contact counts
        for i in 0..base_contact_count {
            let body_id_a = base_contacts[i].body_id_a;
            let body_id_b = base_contacts[i].body_id_b;
            let island_index_a = self.bodies[body_id_a as usize].island_index;
            let island_index_b = self.bodies[body_id_b as usize].island_index;

            // Only connect non-static bodies that still belong to this island.
            let valid_a = island_index_a >= 0 && (island_index_a as usize) < base_body_count;
            let valid_b = island_index_b >= 0 && (island_index_b as usize) < base_body_count;
            if valid_a && valid_b {
                island_union(
                    &mut parents,
                    &mut ranks,
                    island_index_a,
                    island_index_b,
                    &mut contact_counts,
                    &mut joint_counts,
                );
                let root = island_find_parent(&mut parents, island_index_a);
                contact_counts[root as usize] += 1;
            } else if valid_a || valid_b {
                let island_index = if valid_a { island_index_a } else { island_index_b };
                let root = island_find_parent(&mut parents, island_index);
                contact_counts[root as usize] += 1;
            }
        }

        // Union over joints, tracking per-component joint counts
        for i in 0..base_joint_count {
            let body_id_a = base_joints[i].body_id_a;
            let body_id_b = base_joints[i].body_id_b;
            let island_index_a = self.bodies[body_id_a as usize].island_index;
            let island_index_b = self.bodies[body_id_b as usize].island_index;

            let valid_a = island_index_a >= 0 && (island_index_a as usize) < base_body_count;
            let valid_b = island_index_b >= 0 && (island_index_b as usize) < base_body_count;
            if valid_a && valid_b {
                island_union(
                    &mut parents,
                    &mut ranks,
                    island_index_a,
                    island_index_b,
                    &mut contact_counts,
                    &mut joint_counts,
                );
                let root = island_find_parent(&mut parents, island_index_a);
                joint_counts[root as usize] += 1;
            } else if valid_a || valid_b {
                let island_index = if valid_a { island_index_a } else { island_index_b };
                let root = island_find_parent(&mut parents, island_index);
                joint_counts[root as usize] += 1;
            }
        }

        // Flatten all parent indices and count connected components.
        let mut component_count = 0usize;
        for i in 0..base_body_count {
            parents[i] = island_find_parent(&mut parents, i as i32);
            if parents[i] == i as i32 {
                component_count += 1;
            }
        }

        // Island is still fully connected, no split needed.
        if component_count == 1 {
            self.islands[base_id as usize].constraint_remove_count = 0;
            return;
        }

        // Detach the base island so destroying it won't free the arrays.
        let mut base = self.islands[base_id as usize].clone();
        base.bodies = Vec::new();
        base.contacts = Vec::new();
        base.joints = Vec::new();
        self.islands[base_id as usize] = base;

        // Map from body index to new island index. Only set for root bodies.
        let mut root_map = vec![b3::NULL_INDEX; base_body_count];
        let mut component_body_counts = vec![0i32; component_count];
        let mut component_contact_counts = vec![0i32; component_count];
        let mut component_joint_counts = vec![0i32; component_count];
        let mut island_count = 0usize;

        for i in 0..base_body_count {
            let root_index = parents[i];
            if root_map[root_index as usize] == b3::NULL_INDEX {
                root_map[root_index as usize] = island_count as i32;
                component_body_counts[island_count] = 0;
                component_contact_counts[island_count] = contact_counts[root_index as usize];
                component_joint_counts[island_count] = joint_counts[root_index as usize];
                island_count += 1;
            }

            component_body_counts[root_map[root_index as usize] as usize] += 1;
        }

        b3_assert(island_count == component_count);

        // Map from new island index to island id
        let mut island_ids = vec![b3::NULL_INDEX; island_count];

        // Create new islands.
        for i in 0..island_count {
            island_ids[i] = self.create_island(SetType::AwakeSet.raw_value());
        }

        // Assign bodies to new islands. Reserve each island's final capacity up front so the
        // rebuild does not repeatedly grow (and copy) the arrays.
        for i in 0..island_count {
            let new_island_id = island_ids[i];
            self.islands[new_island_id as usize]
                .bodies
                .reserve(component_body_counts[i] as usize);
        }
        for i in 0..base_body_count {
            let body_id = base_body_ids[i];
            let root = island_find_parent(&mut parents, i as i32);

            // Swift's `islandIds[...] as Int32?` promotes a non-optional `Int32`, so the optional
            // is always `.some` and the `continue` arm is dead. Every root was assigned an id
            // above, so the lookup cannot reach `B3.nullIndex` either.
            let new_island_id = island_ids[root_map[root as usize] as usize];

            let mut body = self.bodies[body_id as usize].clone();
            body.island_id = new_island_id;
            body.island_index = self.islands[new_island_id as usize].bodies.len() as i32;
            self.islands[new_island_id as usize].bodies.push(body_id);
            self.bodies[body_id as usize] = body;
        }

        // Assign contacts to the island of their bodies
        for i in 0..base_contact_count {
            let link = base_contacts[i];
            let mut contact = self.contacts[link.contact_id as usize];

            // Static bodies don't have an island id.
            let island_id_a = self.bodies[link.body_id_a as usize].island_id;
            let island_id_b = self.bodies[link.body_id_b as usize].island_id;
            let target_island_id = if island_id_a != b3::NULL_INDEX {
                island_id_a
            } else {
                island_id_b
            };

            contact.island_id = target_island_id;
            contact.island_index = self.islands[target_island_id as usize].contacts.len() as i32;
            self.islands[target_island_id as usize].contacts.push(link);
            self.contacts[link.contact_id as usize] = contact;
        }

        // Assign joints to the island of their bodies
        for i in 0..base_joint_count {
            let link = base_joints[i];
            let mut joint = self.joints[link.joint_id as usize].clone();

            // Static bodies don't have an island id.
            let island_id_a = self.bodies[link.body_id_a as usize].island_id;
            let island_id_b = self.bodies[link.body_id_b as usize].island_id;
            let target_island_id = if island_id_a != b3::NULL_INDEX {
                island_id_a
            } else {
                island_id_b
            };

            joint.island_id = target_island_id;
            joint.island_index = self.islands[target_island_id as usize].joints.len() as i32;
            self.islands[target_island_id as usize].joints.push(link);
            self.joints[link.joint_id as usize] = joint;
        }

        // Swift filled these two per-component arrays and never read them; they are kept so the
        // scan, the writes, and the island counts stay exactly as the original wrote them.
        let _ = (component_contact_counts, component_joint_counts);

        // Destroy the base island
        self.destroy_island(base_id);
    }

    // MARK: Sleeping

    /// Put an island to sleep. Creates a new sleeping solver set for it.
    ///
    /// Swift's `sleepBodySimIndex` method is byte-identical to the free helper already declared in
    /// `world_runtime.rs`, so this calls that one instead of defining a second copy.
    pub(crate) fn try_sleep_island(&mut self, island_id: i32) {
        let mut island = self.islands[island_id as usize].clone();
        b3_assert(island.set_index == SetType::AwakeSet.raw_value());

        // Cannot put an island to sleep while it has a pending split and more than one body.
        if island.constraint_remove_count > 0 && island.bodies.len() > 1 {
            return;
        }

        let sleep_set_id = self.solver_set_id_pool.alloc();
        if sleep_set_id as usize == self.solver_sets.len() {
            self.solver_sets.push(SolverSet::default());
        }

        let mut sleep_set = SolverSet::default();
        sleep_set.set_index = sleep_set_id;

        let awake_set_index = SetType::AwakeSet.raw_value() as usize;
        let disabled_set_index = SetType::DisabledSet.raw_value() as usize;

        b3_assert(
            0 <= island.local_index
                && (island.local_index as usize)
                    < self.solver_sets[awake_set_index].island_sims.len(),
        );

        // move awake bodies to sleeping set
        // this shuffles around bodies in the awake set
        for i in 0..island.bodies.len() {
            let body_id = island.bodies[i];
            let mut body = self.bodies[body_id as usize].clone();
            b3_assert(body.set_index == SetType::AwakeSet.raw_value());
            b3_assert(body.island_id == island_id);
            b3_assert(body.island_index == i as i32);

            // Update the body move event to indicate this body fell asleep.
            if body.body_move_index != b3::NULL_INDEX
                && (body.body_move_index as usize) < self.body_move_events.len()
            {
                let move_index = body.body_move_index as usize;
                // Swift copied the event out, set the flag, and wrote it back; writing the field
                // in place is the equivalent operation.
                self.body_move_events[move_index].fell_asleep = true;
                body.body_move_index = b3::NULL_INDEX;
            }

            let awake_body_index = body.local_index;
            let awake_sim = self.solver_sets[awake_set_index].body_sims[awake_body_index as usize];

            // move body sim to sleep set
            let sleep_body_index = sleep_set.body_sims.len() as i32;
            sleep_set.body_sims.push(awake_sim);

            let moved_index = remove_swap_array(
                &mut self.solver_sets[awake_set_index].body_sims,
                awake_body_index as usize,
            );
            if moved_index != b3::NULL_INDEX {
                // fix local index on moved element
                let moved_id =
                    self.solver_sets[awake_set_index].body_sims[awake_body_index as usize].body_id;
                let mut moved_body = self.bodies[moved_id as usize].clone();
                b3_assert(moved_body.local_index == moved_index);
                moved_body.local_index = awake_body_index;
                // Swift read the local copy's island id after writing the copy back; the store
                // moves the value here, so the field is read one statement earlier. Nothing
                // between the two changes it.
                let moved_body_island_id = moved_body.island_id;
                self.bodies[moved_id as usize] = moved_body;

                if moved_body_island_id != island_id {
                    self.refresh_body_contact_indices(moved_id);
                }
            }

            // destroy state, no need to clone
            remove_swap_array(
                &mut self.solver_sets[awake_set_index].body_states,
                awake_body_index as usize,
            );

            body.set_index = sleep_set_id;
            body.local_index = sleep_body_index;
            // Swift read `body.headContactKey` from the same local copy it had just stored; the
            // store below moves the copy, so the key is taken one statement earlier. Nothing
            // between the two writes the field.
            let mut contact_key = body.head_contact_key;
            self.bodies[body_id as usize] = body;

            // Move non-touching contacts to the disabled set.
            while contact_key != b3::NULL_INDEX {
                let contact_id = contact_key >> 1;
                let edge_index = (contact_key & 1) as usize;

                let mut contact = self.contacts[contact_id as usize];
                b3_assert(contact.set_index == SetType::AwakeSet.raw_value());

                contact_key = contact.edges[edge_index].next_key;

                if contact.color_index != b3::NULL_INDEX {
                    // contact is touching and will be moved separately
                    b3_assert((contact.flags & ContactFlags::TOUCHING.0) != 0);
                    continue;
                }

                if edge_index == 0 {
                    contact.encoded_body_sim_a = b3::NULL_INDEX;
                } else {
                    contact.encoded_body_sim_b = b3::NULL_INDEX;
                }

                // the other body may still be awake
                let other_edge_index = edge_index ^ 1;
                let other_body_id = contact.edges[other_edge_index].body_id;
                if self.bodies[other_body_id as usize].set_index == SetType::AwakeSet.raw_value() {
                    self.contacts[contact_id as usize] = contact;
                    continue;
                }

                let local_index = contact.local_index;
                b3_assert(
                    self.solver_sets[awake_set_index].contact_indices[local_index as usize]
                        == contact_id,
                );

                b3_assert(contact.manifold_count == 0);
                b3_assert((contact.flags & ContactFlags::TOUCHING.0) == 0);

                // Move the non-touching contact to the disabled set.
                contact.set_index = SetType::DisabledSet.raw_value();
                contact.local_index =
                    self.solver_sets[disabled_set_index].contact_indices.len() as i32;
                self.solver_sets[disabled_set_index]
                    .contact_indices
                    .push(contact.contact_id);

                let moved_local_index = remove_swap_array(
                    &mut self.solver_sets[awake_set_index].contact_indices,
                    local_index as usize,
                );
                if moved_local_index != b3::NULL_INDEX {
                    let moved_contact_index =
                        self.solver_sets[awake_set_index].contact_indices[local_index as usize];
                    let mut moved_contact = self.contacts[moved_contact_index as usize];
                    b3_assert(moved_contact.local_index == moved_local_index);
                    moved_contact.local_index = local_index;
                    self.contacts[moved_contact_index as usize] = moved_contact;
                }

                self.contacts[contact_id as usize] = contact;
            }
        }

        // move touching contacts to sleeping set
        for i in 0..island.contacts.len() {
            let contact_id = island.contacts[i].contact_id;
            let mut contact = self.contacts[contact_id as usize];
            b3_assert(contact.set_index == SetType::AwakeSet.raw_value());
            b3_assert(contact.island_id == island_id);
            b3_assert(contact.island_index == i as i32);
            let color_index = contact.color_index;
            b3_assert(0 <= color_index && color_index < b3::GRAPH_COLOR_COUNT);

            // Remove bodies from graph coloring associated with this constraint
            if color_index != OVERFLOW_INDEX {
                // might clear a bit for a static body, but this has no effect
                self.constraint_graph
                    .clear_bit(color_index, contact.edges[0].body_id);
                self.constraint_graph
                    .clear_bit(color_index, contact.edges[1].body_id);
            }

            let sleep_contact_index = sleep_set.contact_indices.len() as i32;
            sleep_set.contact_indices.push(contact_id);

            let local_index = contact.local_index;
            if (contact.flags & ContactFlags::SIM_MESH_CONTACT.0) != 0
                || color_index == OVERFLOW_INDEX
            {
                let moved_local_index = remove_swap_array(
                    &mut self.constraint_graph.colors[color_index as usize].contacts,
                    local_index as usize,
                );
                if moved_local_index != b3::NULL_INDEX {
                    let color = &self.constraint_graph.colors[color_index as usize];
                    let moved_contact_id = color.contacts[local_index as usize].contact_id;
                    let mut moved_contact = self.contacts[moved_contact_id as usize];
                    b3_assert(moved_contact.local_index == moved_local_index);
                    moved_contact.local_index = local_index;
                    self.contacts[moved_contact_id as usize] = moved_contact;
                }
            } else {
                let moved_local_index = remove_swap_array(
                    &mut self.constraint_graph.colors[color_index as usize].convex_contacts,
                    local_index as usize,
                );
                if moved_local_index != b3::NULL_INDEX {
                    let color = &self.constraint_graph.colors[color_index as usize];
                    let moved_contact_id = color.convex_contacts[local_index as usize];
                    let mut moved_contact = self.contacts[moved_contact_id as usize];
                    b3_assert(moved_contact.local_index == moved_local_index);
                    moved_contact.local_index = local_index;
                    self.contacts[moved_contact_id as usize] = moved_contact;
                }
            }

            contact.set_index = sleep_set_id;
            contact.color_index = b3::NULL_INDEX;
            contact.local_index = sleep_contact_index;
            contact.encoded_body_sim_a = sleep_body_sim_index(contact.encoded_body_sim_a);
            contact.encoded_body_sim_b = sleep_body_sim_index(contact.encoded_body_sim_b);
            self.contacts[contact_id as usize] = contact;
        }

        // move joints
        for i in 0..island.joints.len() {
            let joint_id = island.joints[i].joint_id;
            let mut joint = self.joints[joint_id as usize].clone();
            b3_assert(joint.set_index == SetType::AwakeSet.raw_value());
            b3_assert(joint.island_id == island_id);
            b3_assert(joint.island_index == i as i32);
            let color_index = joint.color_index;
            let local_index = joint.local_index;

            b3_assert(0 <= color_index && color_index < b3::GRAPH_COLOR_COUNT);

            // might clear a bit for a static body, but this has no effect
            if color_index != OVERFLOW_INDEX {
                self.constraint_graph
                    .clear_bit(color_index, joint.edges[0].body_id);
                self.constraint_graph
                    .clear_bit(color_index, joint.edges[1].body_id);
            }

            let sleep_joint_index = sleep_set.joint_sims.len() as i32;
            sleep_set.joint_sims.push(
                self.constraint_graph.colors[color_index as usize].joint_sims[local_index as usize],
            );

            let moved_index = remove_swap_array(
                &mut self.constraint_graph.colors[color_index as usize].joint_sims,
                local_index as usize,
            );
            if moved_index != b3::NULL_INDEX {
                let color = &self.constraint_graph.colors[color_index as usize];
                let moved_id = color.joint_sims[local_index as usize].joint_id;
                let mut moved_joint = self.joints[moved_id as usize].clone();
                b3_assert(moved_joint.local_index == moved_index);
                moved_joint.local_index = local_index;
                self.joints[moved_id as usize] = moved_joint;
            }

            joint.set_index = sleep_set_id;
            joint.color_index = b3::NULL_INDEX;
            joint.local_index = sleep_joint_index;
            self.joints[joint_id as usize] = joint;
        }

        // move island struct
        {
            b3_assert(island.set_index == SetType::AwakeSet.raw_value());

            let island_index = island.local_index as usize;
            let mut island_sim = IslandSim::default();
            island_sim.island_id = island_id;
            sleep_set.island_sims.push(island_sim);

            let moved_island_index = remove_swap_array(
                &mut self.solver_sets[awake_set_index].island_sims,
                island_index,
            );
            if moved_island_index != b3::NULL_INDEX {
                let moved_island_id =
                    self.solver_sets[awake_set_index].island_sims[island_index].island_id;
                let mut moved_island = self.islands[moved_island_id as usize].clone();
                b3_assert(moved_island.local_index == moved_island_index);
                moved_island.local_index = island_index as i32;
                self.islands[moved_island_id as usize] = moved_island;
            }

            island.set_index = sleep_set_id;
            island.local_index = 0;
            self.islands[island_id as usize] = island;
        }

        if self.split_island_id == island_id {
            self.split_island_id = b3::NULL_INDEX;
        }

        self.solver_sets[sleep_set_id as usize] = sleep_set;
    }

    /// Collect the sleepiest island that needs splitting and put sleeping islands to sleep.
    pub(crate) fn process_sleep(&mut self) {
        // Collect split island candidate for the next time step.
        b3_assert(self.split_island_id == b3::NULL_INDEX);
        let mut split_sleep_timer: f32 = 0.0;
        for i in 0..self.task_contexts.len() {
            // Both scalars are read before the write below so the per-context scan stays in Swift's
            // order; the comparison chain is unchanged.
            let context_split_island_id = self.task_contexts[i].split_island_id;
            let context_split_sleep_time = self.task_contexts[i].split_sleep_time;
            if context_split_island_id != b3::NULL_INDEX
                && context_split_sleep_time >= split_sleep_timer
            {
                b3_assert(context_split_sleep_time > 0.0);

                // Tie breaking for determinism. Largest island id wins.
                if context_split_sleep_time == split_sleep_timer
                    && context_split_island_id < self.split_island_id
                {
                    continue;
                }

                self.split_island_id = context_split_island_id;
                split_sleep_timer = context_split_sleep_time;
            }
        }

        // Need to process in reverse because this moves islands to sleeping solver sets.
        let awake_set_index = SetType::AwakeSet.raw_value() as usize;
        let islands_copy = self.solver_sets[awake_set_index].island_sims.clone();
        // Swift held a reference to the class bit set, so it could call `trySleepIsland` while
        // reading it. Cloning releases the borrow of `task_contexts`; nothing between here and the
        // write-back mutates the set (`trySleepIsland` never touches the task contexts), so the
        // stored value is unchanged and the trailing assignment mirrors Swift's own store.
        let awake_island_bit_set = self.task_contexts[0].awake_island_bit_set.clone();
        for island_index in (0..islands_copy.len()).rev() {
            if awake_island_bit_set.get_bit(island_index as u32) {
                // this island is still awake
                continue;
            }

            self.try_sleep_island(islands_copy[island_index].island_id);
        }

        self.task_contexts[0].awake_island_bit_set = awake_island_bit_set;
    }
}
