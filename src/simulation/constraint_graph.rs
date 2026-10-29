//! Constraint graph coloring.
//!
//! Translated from `Sources/Natsel/Simulation/ConstraintGraph.swift`.
//!
//! Constraints involving two dynamic bodies get low color indices; constraints involving a static
//! body build from the end so they have higher solver priority. Constraints that cannot be colored
//! go to the overflow color.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::containers::bit_set::BitSet;
use crate::types::body_def::BodyType;

use crate::simulation::body_api::ContactFlags;
use crate::simulation::world_runtime::{
    Contact, Joint, JointSim, SetType, World, encode_body_sim_index, remove_swap_array,
};

/// This holds constraints that cannot fit the graph color limit.
pub(crate) const OVERFLOW_INDEX: i32 = b3::GRAPH_COLOR_COUNT - 1;

/// Dynamic constraints cannot encroach on colors reserved for static constraints.
pub(crate) const DYNAMIC_COLOR_COUNT: i32 = b3::GRAPH_COLOR_COUNT - 4;

/// A contact spec for mesh and overflow contacts.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ContactSpec {
    pub contact_id: i32,
    pub manifold_start: i32,
    pub manifold_count: u16,
}

impl ContactSpec {
    #[inline(always)]
    pub const fn new() -> Self {
        Self {
            contact_id: b3::NULL_INDEX,
            manifold_start: 0,
            manifold_count: 0,
        }
    }
}

/// One graph color: a body bitset and the constraints in this color.
#[derive(Clone, Debug)]
pub struct GraphColor {
    /// This bitset is indexed by bodyId so it is over-sized to encompass static bodies.
    pub body_set: BitSet,

    // Cache friendly arrays
    pub joint_sims: Vec<JointSim>,
    pub convex_contacts: Vec<i32>,
    pub contacts: Vec<ContactSpec>,
}

impl Default for GraphColor {
    #[inline]
    fn default() -> Self {
        Self {
            body_set: BitSet::new(),
            joint_sims: Vec::new(),
            convex_contacts: Vec::new(),
            contacts: Vec::new(),
        }
    }
}

/// The constraint graph.
///
/// Swift's `final class`; only `World` holds the reference, so it is an owned struct here.
#[derive(Clone, Debug)]
pub struct ConstraintGraph {
    /// Including overflow at the end.
    pub colors: Vec<GraphColor>,
}

impl ConstraintGraph {
    pub fn new(body_capacity: i32) -> Self {
        let mut colors = vec![GraphColor::default(); b3::GRAPH_COLOR_COUNT as usize];

        let capacity = std::cmp::max(body_capacity, 8);

        // Initialize graph color bit set. No bitset for the overflow color.
        for i in 0..OVERFLOW_INDEX as usize {
            colors[i].body_set = BitSet::with_bit_capacity(capacity as u32);
            colors[i].body_set.set_bit_count_and_clear(capacity as u32);
        }

        Self { colors }
    }

    pub(crate) fn get_bit(&self, color_index: i32, body_id: i32) -> bool {
        self.colors[color_index as usize].body_set.get_bit(body_id as u32)
    }

    pub(crate) fn set_bit_grow(&mut self, color_index: i32, body_id: i32) {
        self.colors[color_index as usize]
            .body_set
            .set_bit_grow(body_id as u32);
    }

    pub(crate) fn clear_bit(&mut self, color_index: i32, body_id: i32) {
        self.colors[color_index as usize]
            .body_set
            .clear_bit(body_id as u32);
    }
}

impl World {
    /// Contacts are always created as non-touching. They get cloned into the constraint graph once
    /// they are found to be touching.
    pub(crate) fn add_contact_to_graph(&mut self, contact_id: i32) {
        let mut contact = self.contacts[contact_id as usize];
        b3_assert(contact.manifold_count > 0);

        let mut color_index = OVERFLOW_INDEX;

        let body_id_a = contact.edges[0].body_id;
        let body_id_b = contact.edges[1].body_id;
        let body_a = self.bodies[body_id_a as usize].clone();
        let body_b = self.bodies[body_id_b as usize].clone();

        let type_a = body_a.type_;
        let type_b = body_b.type_;
        b3_assert(type_a == BodyType::DynamicBody || type_b == BodyType::DynamicBody);

        if type_a == BodyType::DynamicBody && type_b == BodyType::DynamicBody {
            // Dynamic constraint colors cannot encroach on colors reserved for static constraints
            for i in 0..DYNAMIC_COLOR_COUNT as usize {
                if self.constraint_graph.get_bit(i as i32, body_id_a)
                    || self.constraint_graph.get_bit(i as i32, body_id_b)
                {
                    continue;
                }

                self.constraint_graph.set_bit_grow(i as i32, body_id_a);
                self.constraint_graph.set_bit_grow(i as i32, body_id_b);
                color_index = i as i32;
                break;
            }
        } else if type_a == BodyType::DynamicBody {
            // Static constraint colors build from the end to get higher priority
            let mut i = OVERFLOW_INDEX - 1;
            while i >= 1 {
                if !self.constraint_graph.get_bit(i, body_id_a) {
                    self.constraint_graph.set_bit_grow(i, body_id_a);
                    color_index = i;
                    break;
                }
                i -= 1;
            }
        } else if type_b == BodyType::DynamicBody {
            let mut i = OVERFLOW_INDEX - 1;
            while i >= 1 {
                if !self.constraint_graph.get_bit(i, body_id_b) {
                    self.constraint_graph.set_bit_grow(i, body_id_b);
                    color_index = i;
                    break;
                }
                i -= 1;
            }
        }

        let is_scalar = (contact.flags & ContactFlags::SIM_MESH_CONTACT.0) != 0
            || color_index == OVERFLOW_INDEX;

        contact.color_index = color_index;
        contact.local_index = if is_scalar {
            self.constraint_graph.colors[color_index as usize]
                .contacts
                .len() as i32
        } else {
            self.constraint_graph.colors[color_index as usize]
                .convex_contacts
                .len() as i32
        };
        contact.encoded_body_sim_a = encode_body_sim_index(&body_a);
        contact.encoded_body_sim_b = encode_body_sim_index(&body_b);

        if is_scalar {
            b3_assert(contact.manifold_count < u16::MAX as i32);
            let mut spec = ContactSpec::new();
            spec.contact_id = contact.contact_id;
            spec.manifold_start = 0;
            spec.manifold_count = contact.manifold_count as u16;
            self.constraint_graph.colors[color_index as usize]
                .contacts
                .push(spec);
        } else {
            self.constraint_graph.colors[color_index as usize]
                .convex_contacts
                .push(contact.contact_id);
        }

        self.contacts[contact_id as usize] = contact;
    }

    /// Remove a contact from its graph color.
    pub(crate) fn remove_contact_from_graph(
        &mut self,
        body_id_a: i32,
        body_id_b: i32,
        color_index: i32,
        local_index: i32,
        mesh_contact: bool,
    ) {
        b3_assert(0 <= color_index && color_index < b3::GRAPH_COLOR_COUNT);

        if color_index != OVERFLOW_INDEX {
            // This might clear a bit for a static body, but this has no effect
            self.constraint_graph.clear_bit(color_index, body_id_a);
            self.constraint_graph.clear_bit(color_index, body_id_b);
        }

        if mesh_contact || color_index == OVERFLOW_INDEX {
            let moved_index = remove_swap_array(
                &mut self.constraint_graph.colors[color_index as usize].contacts,
                local_index as usize,
            );
            if moved_index != b3::NULL_INDEX {
                // Fix index on swapped contact
                let moved_contact_id = self.constraint_graph.colors[color_index as usize]
                    .contacts[local_index as usize]
                    .contact_id;
                let mut moved_contact = self.contacts[moved_contact_id as usize];
                b3_assert(moved_contact.set_index == SetType::AwakeSet.raw_value());
                b3_assert(moved_contact.color_index == color_index);
                b3_assert(moved_contact.local_index == moved_index);
                moved_contact.local_index = local_index;
                self.contacts[moved_contact_id as usize] = moved_contact;
            }
        } else {
            let moved_index = remove_swap_array(
                &mut self.constraint_graph.colors[color_index as usize].convex_contacts,
                local_index as usize,
            );
            if moved_index != b3::NULL_INDEX {
                // Fix index on swapped contact
                let moved_contact_id = self.constraint_graph.colors[color_index as usize]
                    .convex_contacts[local_index as usize];
                let mut moved_contact = self.contacts[moved_contact_id as usize];
                b3_assert(moved_contact.set_index == SetType::AwakeSet.raw_value());
                b3_assert(moved_contact.color_index == color_index);
                b3_assert(moved_contact.local_index == moved_index);
                b3_assert((moved_contact.flags & ContactFlags::SIM_MESH_CONTACT.0) == 0);
                moved_contact.local_index = local_index;
                self.contacts[moved_contact_id as usize] = moved_contact;
            }
        }
    }

    // MARK: Joints

    /// Assign a graph color for a joint.
    pub(crate) fn assign_joint_color(
        &mut self,
        body_id_a: i32,
        body_id_b: i32,
        type_a: BodyType,
        type_b: BodyType,
    ) -> i32 {
        b3_assert(type_a == BodyType::DynamicBody || type_b == BodyType::DynamicBody);

        if type_a == BodyType::DynamicBody && type_b == BodyType::DynamicBody {
            for i in 0..DYNAMIC_COLOR_COUNT as usize {
                if self.constraint_graph.get_bit(i as i32, body_id_a)
                    || self.constraint_graph.get_bit(i as i32, body_id_b)
                {
                    continue;
                }

                self.constraint_graph.set_bit_grow(i as i32, body_id_a);
                self.constraint_graph.set_bit_grow(i as i32, body_id_b);
                return i as i32;
            }
        } else if type_a == BodyType::DynamicBody {
            let mut i = OVERFLOW_INDEX - 1;
            while i >= 1 {
                if !self.constraint_graph.get_bit(i, body_id_a) {
                    self.constraint_graph.set_bit_grow(i, body_id_a);
                    return i;
                }
                i -= 1;
            }
        } else if type_b == BodyType::DynamicBody {
            let mut i = OVERFLOW_INDEX - 1;
            while i >= 1 {
                if !self.constraint_graph.get_bit(i, body_id_b) {
                    self.constraint_graph.set_bit_grow(i, body_id_b);
                    return i;
                }
                i -= 1;
            }
        }

        OVERFLOW_INDEX
    }

    /// Create a joint sim in the graph and return its index.
    pub(crate) fn create_joint_in_graph(&mut self, joint_id: i32) -> i32 {
        let mut joint = self.joints[joint_id as usize].clone();

        let body_id_a = joint.edges[0].body_id;
        let body_id_b = joint.edges[1].body_id;
        let body_a = self.bodies[body_id_a as usize].clone();
        let body_b = self.bodies[body_id_b as usize].clone();

        let color_index = self.assign_joint_color(body_id_a, body_id_b, body_a.type_, body_b.type_);

        let mut joint_sim = JointSim::default();
        joint_sim.joint_id = joint.joint_id;
        self.constraint_graph.colors[color_index as usize]
            .joint_sims
            .push(joint_sim);

        joint.color_index = color_index;
        joint.local_index =
            self.constraint_graph.colors[color_index as usize].joint_sims.len() as i32 - 1;
        self.joints[joint_id as usize] = joint;

        self.joints[joint_id as usize].local_index
    }

    /// Add a joint to the graph at the given sim.
    pub(crate) fn add_joint_to_graph(&mut self, joint_sim: JointSim, joint_id: i32) {
        let index = self.create_joint_in_graph(joint_id);
        let joint = self.joints[joint_id as usize].clone();
        self.constraint_graph.colors[joint.color_index as usize].joint_sims[index as usize] =
            joint_sim;
    }

    /// Remove a joint from its graph color.
    pub(crate) fn remove_joint_from_graph(
        &mut self,
        body_id_a: i32,
        body_id_b: i32,
        color_index: i32,
        local_index: i32,
    ) {
        b3_assert(0 <= color_index && color_index < b3::GRAPH_COLOR_COUNT);

        if color_index != OVERFLOW_INDEX {
            // May clear static bodies, no effect
            self.constraint_graph.clear_bit(color_index, body_id_a);
            self.constraint_graph.clear_bit(color_index, body_id_b);
        }

        let moved_index = remove_swap_array(
            &mut self.constraint_graph.colors[color_index as usize].joint_sims,
            local_index as usize,
        );
        if moved_index != b3::NULL_INDEX {
            // Fix moved joint
            let moved_id = self.constraint_graph.colors[color_index as usize]
                .joint_sims[local_index as usize]
                .joint_id;
            let mut moved_joint = self.joints[moved_id as usize].clone();
            b3_assert(moved_joint.set_index == SetType::AwakeSet.raw_value());
            b3_assert(moved_joint.color_index == color_index);
            b3_assert(moved_joint.local_index == moved_index);
            moved_joint.local_index = local_index;
            self.joints[moved_id as usize] = moved_joint;
        }
    }
}

impl World {
    /// Debug helper: verify every graph color's contact array matches the contacts' cached indices.
    pub(crate) fn validate_graph_indices(&self) -> Option<String> {
        for color_index in 0..b3::GRAPH_COLOR_COUNT as usize {
            let color = &self.constraint_graph.colors[color_index];

            for (i, contact_id) in color.convex_contacts.iter().enumerate() {
                let contact = self.contacts[*contact_id as usize];
                if contact.contact_id != *contact_id {
                    return Some(format!(
                        "color {color_index} convex[{i}] id={contact_id} slot=freed"
                    ));
                }
                if contact.color_index != color_index as i32 || contact.local_index != i as i32 {
                    return Some(format!(
                        "color {color_index} convex[{i}] id={contact_id} has color={} local={}",
                        contact.color_index, contact.local_index
                    ));
                }
            }

            for (i, spec) in color.contacts.iter().enumerate() {
                let contact_id = spec.contact_id;
                let contact = self.contacts[contact_id as usize];
                if contact.contact_id != contact_id {
                    return Some(format!(
                        "color {color_index} scalar[{i}] id={contact_id} slot=freed"
                    ));
                }
                if contact.color_index != color_index as i32 || contact.local_index != i as i32 {
                    return Some(format!(
                        "color {color_index} scalar[{i}] id={contact_id} has color={} local={}",
                        contact.color_index, contact.local_index
                    ));
                }
            }
        }
        None
    }
}

impl World {
    /// Debug helper: verify every contact agrees with the set and island that claim to own it.
    /// Returns the first violation or nil.
    pub(crate) fn validate_contact_ownership(&self) -> Option<String> {
        for i in 0..self.contacts.len() {
            let c = self.contacts[i];

            if c.contact_id == b3::NULL_INDEX {
                continue;
            }

            if c.contact_id != i as i32 {
                return Some(format!("contact [{i}] has contactId {}", c.contact_id));
            }

            // A live contact must sit in a real set.
            if c.set_index < 0 || c.set_index as usize >= self.solver_sets.len() {
                return Some(format!("contact {i} has setIndex {}", c.set_index));
            }

            // Touching awake contacts belong to the graph, not the set's non-touching list.
            if c.set_index == SetType::AwakeSet.raw_value() && c.color_index != b3::NULL_INDEX {
                if c.local_index < 0 {
                    return Some(format!(
                        "contact {i} graph color {} has localIndex {}",
                        c.color_index, c.local_index
                    ));
                }
            } else {
                // Non-graph contacts are listed in their set's contactIndices at localIndex.
                let set = &self.solver_sets[c.set_index as usize];
                if c.local_index < 0 || c.local_index as usize >= set.contact_indices.len() {
                    return Some(format!(
                        "contact {i} set {} localIndex {} beyond {}",
                        c.set_index,
                        c.local_index,
                        set.contact_indices.len()
                    ));
                }
                if set.contact_indices[c.local_index as usize] != i as i32 {
                    return Some(format!(
                        "contact {i} set {} localIndex {} holds {}",
                        c.set_index,
                        c.local_index,
                        set.contact_indices[c.local_index as usize]
                    ));
                }
            }

            // If it claims an island, the island must list it at islandIndex.
            if c.island_id != b3::NULL_INDEX {
                if c.island_id as usize >= self.islands.len() {
                    return Some(format!(
                        "contact {i} islandId {} out of range",
                        c.island_id
                    ));
                }
                let island = &self.islands[c.island_id as usize];
                if c.island_index < 0 || c.island_index as usize >= island.contacts.len() {
                    return Some(format!(
                        "contact {i} island {} islandIndex {} beyond {}",
                        c.island_id,
                        c.island_index,
                        island.contacts.len()
                    ));
                }
                if island.contacts[c.island_index as usize].contact_id != i as i32 {
                    return Some(format!(
                        "contact {i} island {} islandIndex {} holds {}",
                        c.island_id,
                        c.island_index,
                        island.contacts[c.island_index as usize].contact_id
                    ));
                }
            }
        }
        None
    }

    /// Debug helper: verify every solver set's contact list points at contacts that agree.
    pub(crate) fn validate_set_contacts(&self) -> Option<String> {
        for set_index in 0..self.solver_sets.len() {
            let set = &self.solver_sets[set_index];
            for (k, contact_id) in set.contact_indices.iter().enumerate() {
                if *contact_id < 0 || *contact_id as usize >= self.contacts.len() {
                    return Some(format!(
                        "set {set_index} slot {k} holds {contact_id} out of range"
                    ));
                }
                let c = self.contacts[*contact_id as usize];
                if c.contact_id == b3::NULL_INDEX {
                    return Some(format!(
                        "set {set_index} slot {k} holds freed contact {contact_id}"
                    ));
                }
                if c.set_index != set_index as i32 || c.local_index != k as i32 {
                    return Some(format!(
                        "set {set_index} slot {k} holds contact {contact_id} with set {} local {}",
                        c.set_index, c.local_index
                    ));
                }
            }
        }
        None
    }
}

impl World {
    /// Debug helper: verify island membership matches the bodies and solver sets.
    pub(crate) fn validate_islands(&self) -> Option<String> {
        for i in 0..self.islands.len() {
            let island = &self.islands[i];
            if island.island_id == b3::NULL_INDEX {
                continue;
            }
            if island.set_index < 0 || island.set_index as usize >= self.solver_sets.len() {
                return Some(format!("island {i} setIndex {}", island.set_index));
            }
            let set = &self.solver_sets[island.set_index as usize];
            if island.local_index < 0 || island.local_index as usize >= set.island_sims.len() {
                return Some(format!(
                    "island {i} set {} localIndex {} beyond {}",
                    island.set_index,
                    island.local_index,
                    set.island_sims.len()
                ));
            }
            if set.island_sims[island.local_index as usize].island_id != i as i32 {
                return Some(format!(
                    "island {i} set {} localIndex {} holds {}",
                    island.set_index,
                    island.local_index,
                    set.island_sims[island.local_index as usize].island_id
                ));
            }
            for (k, body_id) in island.bodies.iter().enumerate() {
                let body = self.bodies[*body_id as usize].clone();
                if body.island_id != i as i32 || body.island_index != k as i32 {
                    return Some(format!(
                        "island {i} body slot {k}={body_id} has island {} index {}",
                        body.island_id, body.island_index
                    ));
                }
                if body.set_index != island.set_index {
                    return Some(format!(
                        "island {i} body {body_id} set {} != {}",
                        body.set_index, island.set_index
                    ));
                }
            }
        }
        None
    }
}
