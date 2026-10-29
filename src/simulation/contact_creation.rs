//! Broad-phase pair generation and structural contact creation.
//!
//! Translated from `Sources/Natsel/Simulation/ContactCreation.swift`.

use std::sync::OnceLock;

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::base::platform::b3_prefetch;
use crate::collision::compound::compound::CompoundChildGeometry;
use crate::collision::dynamic_tree::{
    DynamicTree, MOVED_NODE_FLAG, ROOT_NODE, TREE_STACK_SIZE, TreeNode, is_leaf, left_child,
};
use crate::containers::hash_set::{key_hash, shape_pair_key};
use crate::math::functions::{aabb_overlaps, max_float, max_int, min_int};
use crate::types::body_def::BodyType;
use crate::types::ids::ShapeID;
use crate::types::shape_def::ShapeType;

use crate::simulation::body_api::ContactFlags;
use crate::simulation::mesh_contact::cache::MeshContactState;
use crate::simulation::shape_runtime::{Shape, ShapeFlags, ShapeGeometry, should_shapes_collide};
use crate::simulation::world_runtime::{
    Body, BodyFlags, Contact, SetType, World, encode_body_sim_index, remove_swap_array,
};

/// Stack entry for cross-tree traversal.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct TraversalPair {
    pub a: i32,
    pub b: i32,
}

/// Seed pair for the serial cross-tree breadth-first search. Swift declared this inside
/// `extension World` (`World.NodePair`); Rust has no nested types, so it sits at module level
/// with the same name `contact_creation::NodePair` that `world_runtime.rs` already imports.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct NodePair {
    pub a: TreeNode,
    pub b: TreeNode,
}

impl NodePair {
    /// Swift's `init(_ a: TreeNode, _ b: TreeNode)`.
    pub fn new(a: TreeNode, b: TreeNode) -> Self {
        Self { a, b }
    }
}

// MARK: - Contact registers

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ContactRegister {
    supported: bool,
    primary: bool,
}

/// Swift's `private let shapeTypeCount = Int(ShapeType.count)`.
const SHAPE_TYPE_COUNT: usize = ShapeType::COUNT as usize;

/// Swift's `private let contactRegisters: [[ContactRegister]]`, a lazily built table that is
/// initialised once (exactly like Swift's global `let`) behind a `OnceLock`.
fn contact_registers() -> &'static [[ContactRegister; SHAPE_TYPE_COUNT]; SHAPE_TYPE_COUNT] {
    static CONTACT_REGISTERS: OnceLock<
        [[ContactRegister; SHAPE_TYPE_COUNT]; SHAPE_TYPE_COUNT],
    > = OnceLock::new();

    CONTACT_REGISTERS.get_or_init(|| {
        let mut registers =
            [[ContactRegister::default(); SHAPE_TYPE_COUNT]; SHAPE_TYPE_COUNT];

        fn add_type(
            registers: &mut [[ContactRegister; SHAPE_TYPE_COUNT]; SHAPE_TYPE_COUNT],
            type1: ShapeType,
            type2: ShapeType,
        ) {
            registers[type1 as usize][type2 as usize].supported = true;
            registers[type1 as usize][type2 as usize].primary = true;

            if type1 != type2 {
                registers[type2 as usize][type1 as usize].supported = true;
                registers[type2 as usize][type1 as usize].primary = false;
            }
        }

        add_type(&mut registers, ShapeType::SphereShape, ShapeType::SphereShape);
        add_type(&mut registers, ShapeType::CapsuleShape, ShapeType::SphereShape);
        add_type(&mut registers, ShapeType::CapsuleShape, ShapeType::CapsuleShape);
        add_type(&mut registers, ShapeType::CompoundShape, ShapeType::SphereShape);
        add_type(&mut registers, ShapeType::CompoundShape, ShapeType::CapsuleShape);
        add_type(&mut registers, ShapeType::CompoundShape, ShapeType::HullShape);
        add_type(&mut registers, ShapeType::HullShape, ShapeType::SphereShape);
        add_type(&mut registers, ShapeType::HullShape, ShapeType::CapsuleShape);
        add_type(&mut registers, ShapeType::HullShape, ShapeType::HullShape);
        add_type(&mut registers, ShapeType::MeshShape, ShapeType::SphereShape);
        add_type(&mut registers, ShapeType::MeshShape, ShapeType::CapsuleShape);
        add_type(&mut registers, ShapeType::MeshShape, ShapeType::HullShape);
        add_type(&mut registers, ShapeType::HeightShape, ShapeType::SphereShape);
        add_type(&mut registers, ShapeType::HeightShape, ShapeType::CapsuleShape);
        add_type(&mut registers, ShapeType::HeightShape, ShapeType::HullShape);

        registers
    })
}

// MARK: - Contact bookkeeping

impl World {
    /// Do a body pair's joints allow collision?
    pub(crate) fn should_bodies_collide(&self, body_a: &Body, body_b: &Body) -> bool {
        let type_a = body_a.type_;
        let type_b = body_b.type_;
        if type_a != BodyType::DynamicBody && type_b != BodyType::DynamicBody {
            return false;
        }

        let joint_key: i32;
        let other_body_id: i32;
        if body_a.joint_count < body_b.joint_count {
            joint_key = body_a.head_joint_key;
            other_body_id = body_b.id;
        } else {
            joint_key = body_b.head_joint_key;
            other_body_id = body_a.id;
        }

        let mut key = joint_key;
        while key != b3::NULL_INDEX {
            let joint_id = key >> 1;
            let edge_index = (key & 1) as usize;
            let other_edge_index = edge_index ^ 1;

            let joint = &self.joints[joint_id as usize];
            if joint.collide_connected == false && joint.edges[other_edge_index].body_id == other_body_id {
                return false;
            }

            key = joint.edges[edge_index].next_key;
        }

        true
    }

    /// The shape pair gauntlet.
    pub(crate) fn should_create_pair(&self, shape_id_a: i32, shape_id_b: i32) -> bool {
        // Swift wrapped this in `shapes.withUnsafeBufferPointer`; the vector is never
        // re-allocated while this borrow lives, so the raw base pointer is equivalent.
        // SAFETY: `shape_id_a`/`shape_id_b` are live shape indices into `World::shapes`,
        // exactly as Swift's `baseAddress! + Int(id)` required.
        unsafe {
            let shape_base = self.shapes.as_ptr();
            self.should_create_pair_with_shapes(
                shape_id_a,
                shape_id_b,
                shape_base.add(shape_id_a as usize),
                shape_base.add(shape_id_b as usize),
            )
        }
    }

    /// The gauntlet on borrowed shapes; copying a `Shape` retains its geometry and materials.
    /// (Swift's `shouldCreatePair(_:_:_:_:)` overload; Rust overloads become named methods.)
    pub(crate) fn should_create_pair_with_shapes(
        &self,
        shape_id_a: i32,
        shape_id_b: i32,
        shape_a: *const Shape,
        shape_b: *const Shape,
    ) -> bool {
        // SAFETY: both pointers point into `World::shapes` and are alive for the whole call;
        // nothing in this function mutates or grows `shapes`, mirroring Swift's borrowed
        // `UnsafePointer<Shape>` parameters whose unchecked exclusivity the port documents.
        let shape_a = unsafe { &*shape_a };
        let shape_b = unsafe { &*shape_b };

        let body_id_a = shape_a.body_id;
        let body_id_b = shape_b.body_id;

        // Are the shapes on the same body?
        if body_id_a == body_id_b {
            return false;
        }

        // Sensors are handled elsewhere
        if shape_a.sensor_index != b3::NULL_INDEX || shape_b.sensor_index != b3::NULL_INDEX {
            return false;
        }

        if should_shapes_collide(&shape_a.filter, &shape_b.filter) == false {
            return false;
        }

        // Does a joint override collision?
        let body_a = &self.bodies[body_id_a as usize];
        let body_b = &self.bodies[body_id_b as usize];
        if self.should_bodies_collide(body_a, body_b) == false {
            return false;
        }

        // Custom user filter
        if (shape_a.flags & ShapeFlags::ENABLE_CUSTOM_FILTERING.0) != 0
            || (shape_b.flags & ShapeFlags::ENABLE_CUSTOM_FILTERING.0) != 0
        {
            if let Some(custom_filter) = &self.custom_filter {
                let id_a = ShapeID::new(shape_id_a + 1, self.world_id, shape_a.generation);
                let id_b = ShapeID::new(shape_id_b + 1, self.world_id, shape_b.generation);
                if custom_filter(id_a, id_b) == false {
                    return false;
                }
            }
        }

        true
    }

    /// Create the persistent contact for a shape pair. Returns the new contact id.
    pub(crate) fn create_contact(
        &mut self,
        shape_id_a: i32,
        shape_id_b: i32,
        child_index: i32,
    ) -> Option<i32> {
        let type_a = self.shapes[shape_id_a as usize].type_;
        let type_b = self.shapes[shape_id_b as usize].type_;

        let register = contact_registers()[type_a as usize][type_b as usize];
        if register.supported == false {
            // For example, no mesh vs mesh collision
            return None;
        }

        if register.primary == false {
            // flip order
            return self.create_contact(shape_id_b, shape_id_a, child_index);
        }

        // SAFETY: both pointers are derived from `World::shapes` immediately before the call;
        // `create_contact_with_shapes` never mutates or grows `shapes`, so the borrowed shapes
        // stay alive exactly as Swift's `withUnsafeBufferPointer` guaranteed.
        unsafe {
            let shape_base = self.shapes.as_ptr();
            Some(self.create_contact_with_shapes(
                shape_base.add(shape_id_a as usize),
                shape_base.add(shape_id_b as usize),
                child_index,
            ))
        }
    }

    /// Contact creation on borrowed shapes; copying a `Shape` retains its geometry and materials.
    fn create_contact_with_shapes(
        &mut self,
        shape_a: *const Shape,
        shape_b: *const Shape,
        child_index: i32,
    ) -> i32 {
        // SAFETY: pointers come from `World::shapes`; the function only writes other world
        // storage (bodies, contacts, solver sets, the pair set) and never touches `shapes`,
        // so the shared references stay valid for the whole body (Swift's unchecked aliasing).
        let shape_a = unsafe { &*shape_a };
        let shape_b = unsafe { &*shape_b };

        let type_a = shape_a.type_;
        let type_b = shape_b.type_;
        let body_id_a = shape_a.body_id as usize;
        let body_id_b = shape_b.body_id as usize;

        b3_assert(
            self.bodies[body_id_a].set_index != SetType::DisabledSet.raw_value()
                && self.bodies[body_id_b].set_index != SetType::DisabledSet.raw_value(),
        );
        b3_assert(
            self.bodies[body_id_a].set_index != SetType::StaticSet.raw_value()
                || self.bodies[body_id_b].set_index != SetType::StaticSet.raw_value(),
        );

        let set_index: i32;
        if self.bodies[body_id_a].set_index == SetType::AwakeSet.raw_value()
            || self.bodies[body_id_b].set_index == SetType::AwakeSet.raw_value()
        {
            set_index = SetType::AwakeSet.raw_value();
        } else {
            // sleeping and non-touching contacts live in the disabled set
            set_index = SetType::DisabledSet.raw_value();
        }

        let set_id = set_index as usize;

        // Create contact key and contact
        let contact_id = self.contact_id_pool.alloc();
        if contact_id as usize == self.contacts.len() {
            self.contacts.push(Contact::default());
        }

        let mut contact = Contact::default();
        contact.payload = Some(self.contact_payloads.initialize(contact_id as usize));
        contact.contact_id = contact_id;
        contact.generation = contact.generation.wrapping_add(1);
        contact.set_index = set_index;
        contact.color_index = b3::NULL_INDEX;
        contact.local_index = self.solver_sets[set_id].contact_indices.len() as i32;
        contact.island_id = b3::NULL_INDEX;
        contact.island_index = b3::NULL_INDEX;
        contact.shape_id_a = shape_a.id;
        contact.shape_id_b = shape_b.id;
        contact.child_index = child_index;
        contact.encoded_body_sim_a = encode_body_sim_index(&self.bodies[body_id_a]);
        contact.encoded_body_sim_b = encode_body_sim_index(&self.bodies[body_id_b]);

        if shape_a.type_ == ShapeType::MeshShape || shape_a.type_ == ShapeType::HeightShape {
            contact.flags |= ContactFlags::SIM_MESH_CONTACT.0;
        }

        // Swift's `if case .compound(let compound) = geometry, case .mesh = child(at:).geometry`
        // as nested `if let`s (let chains need a newer compiler than `rust-version` allows).
        if let ShapeGeometry::Compound(compound) = &shape_a.geometry {
            if let CompoundChildGeometry::Mesh(_) = compound.child(child_index as usize).geometry {
                contact.flags |= ContactFlags::SIM_MESH_CONTACT.0;
            }
        }

        if self.bodies[body_id_a].type_ == BodyType::StaticBody
            || self.bodies[body_id_b].type_ == BodyType::StaticBody
        {
            contact.flags |= ContactFlags::STATIC_FLAG.0;
        }

        b3_assert(
            shape_a.sensor_index == b3::NULL_INDEX && shape_b.sensor_index == b3::NULL_INDEX,
        );

        if (shape_a.flags & ShapeFlags::ENABLE_CONTACT_EVENTS.0) != 0
            || (shape_b.flags & ShapeFlags::ENABLE_CONTACT_EVENTS.0) != 0
        {
            contact.flags |= ContactFlags::ENABLE_CONTACT_EVENTS.0;
        }

        if (shape_a.flags & ShapeFlags::ENABLE_SPECULATIVE.0) != 0
            && (shape_b.flags & ShapeFlags::ENABLE_SPECULATIVE.0) != 0
        {
            contact.flags |= ContactFlags::ENABLE_SPECULATIVE_POINTS.0;
        }

        // Disabling speculative collision also disables recycling.
        if (self.bodies[body_id_a].flags & BodyFlags::ENABLE_CONTACT_RECYCLING.0) != 0
            && (self.bodies[body_id_b].flags & BodyFlags::ENABLE_CONTACT_RECYCLING.0) != 0
            && (contact.flags & ContactFlags::ENABLE_SPECULATIVE_POINTS.0) != 0
        {
            contact.flags |= ContactFlags::RECYCLE.0;
        }

        // Connect to body A
        contact.edges[0].body_id = shape_a.body_id;
        contact.edges[0].prev_key = b3::NULL_INDEX;
        contact.edges[0].next_key = self.bodies[body_id_a].head_contact_key;

        let key_a = (contact_id << 1) | 0;
        let head_contact_key_a = self.bodies[body_id_a].head_contact_key;
        if head_contact_key_a != b3::NULL_INDEX {
            self.contacts[(head_contact_key_a >> 1) as usize]
                .edges[(head_contact_key_a & 1) as usize]
                .prev_key = key_a;
        }
        self.bodies[body_id_a].head_contact_key = key_a;
        self.bodies[body_id_a].contact_count += 1;

        // Connect to body B
        contact.edges[1].body_id = shape_b.body_id;
        contact.edges[1].prev_key = b3::NULL_INDEX;
        contact.edges[1].next_key = self.bodies[body_id_b].head_contact_key;

        let key_b = (contact_id << 1) | 1;
        let head_contact_key_b = self.bodies[body_id_b].head_contact_key;
        if head_contact_key_b != b3::NULL_INDEX {
            self.contacts[(head_contact_key_b >> 1) as usize]
                .edges[(head_contact_key_b & 1) as usize]
                .prev_key = key_b;
        }
        self.bodies[body_id_b].head_contact_key = key_b;
        self.bodies[body_id_b].contact_count += 1;

        // Add to pair set for fast lookup
        let pair_key = shape_pair_key(shape_a.id, shape_b.id, child_index);
        let _ = self.broad_phase.pair_set.add_key(pair_key);

        // Contacts are created as non-touching.
        self.solver_sets[set_id].contact_indices.push(contact_id);

        let mut radius_a: f32 = 0.0;
        if type_a == ShapeType::SphereShape {
            radius_a = shape_a.sphere().radius;
        } else if type_a == ShapeType::CapsuleShape {
            radius_a = shape_a.capsule().radius;
        }

        let mut radius_b: f32 = 0.0;
        if type_b == ShapeType::SphereShape {
            radius_b = shape_b.sphere().radius;
        } else if type_b == ShapeType::CapsuleShape {
            radius_b = shape_b.capsule().radius;
        }

        let max_radius = max_float(radius_a, radius_b);

        // Assuming the rolling resistance doesn't change
        contact.rolling_resistance = max_float(
            shape_a.material(0).rolling_resistance,
            shape_b.material(0).rolling_resistance,
        ) * max_radius;

        if (shape_a.flags & ShapeFlags::ENABLE_PRE_SOLVE_EVENTS.0) != 0
            || (shape_b.flags & ShapeFlags::ENABLE_PRE_SOLVE_EVENTS.0) != 0
        {
            contact.flags |= ContactFlags::SIM_ENABLE_PRE_SOLVE_EVENTS.0;
        }

        self.contacts[contact_id as usize] = contact;

        contact_id
    }

    /// Destroy a contact and unlink it from the bodies, set, and graph.
    pub(crate) fn destroy_contact(&mut self, contact_id: i32, wake_bodies: bool) {
        let mut contact = self.contacts[contact_id as usize];

        // A stale entry can outlive its contact, for example a sleep-set index that the wake
        // path already migrated. Ignore it rather than tearing the slot down twice.
        if contact.contact_id == b3::NULL_INDEX {
            return;
        }

        // Remove pair from set
        let pair_key = shape_pair_key(contact.shape_id_a, contact.shape_id_b, contact.child_index);
        let _ = self.broad_phase.pair_set.remove_key(pair_key);

        contact.manifolds().remove_all(false);
        contact.manifold_count = 0;

        let edge_a = contact.edges[0];
        let edge_b = contact.edges[1];

        let body_id_a = edge_a.body_id;
        let body_id_b = edge_b.body_id;
        let mut body_a = self.bodies[body_id_a as usize].clone();
        let mut body_b = self.bodies[body_id_b as usize].clone();

        let flags = contact.flags;
        let touching = (flags & ContactFlags::TOUCHING.0) != 0;

        // End touch events are queued for publication.

        // Remove from body A
        if edge_a.prev_key != b3::NULL_INDEX {
            let mut prev_contact = self.contacts[(edge_a.prev_key >> 1) as usize];
            prev_contact.edges[(edge_a.prev_key & 1) as usize].next_key = edge_a.next_key;
            self.contacts[(edge_a.prev_key >> 1) as usize] = prev_contact;
        }

        if edge_a.next_key != b3::NULL_INDEX {
            let mut next_contact = self.contacts[(edge_a.next_key >> 1) as usize];
            next_contact.edges[(edge_a.next_key & 1) as usize].prev_key = edge_a.prev_key;
            self.contacts[(edge_a.next_key >> 1) as usize] = next_contact;
        }

        let edge_key_a = (contact_id << 1) | 0;
        if body_a.head_contact_key == edge_key_a {
            body_a.head_contact_key = edge_a.next_key;
        }
        body_a.contact_count -= 1;

        // Remove from body B
        if edge_b.prev_key != b3::NULL_INDEX {
            let mut prev_contact = self.contacts[(edge_b.prev_key >> 1) as usize];
            prev_contact.edges[(edge_b.prev_key & 1) as usize].next_key = edge_b.next_key;
            self.contacts[(edge_b.prev_key >> 1) as usize] = prev_contact;
        }

        if edge_b.next_key != b3::NULL_INDEX {
            let mut next_contact = self.contacts[(edge_b.next_key >> 1) as usize];
            next_contact.edges[(edge_b.next_key & 1) as usize].prev_key = edge_b.prev_key;
            self.contacts[(edge_b.next_key >> 1) as usize] = next_contact;
        }

        let edge_key_b = (contact_id << 1) | 1;
        if body_b.head_contact_key == edge_key_b {
            body_b.head_contact_key = edge_b.next_key;
        }
        body_b.contact_count -= 1;

        self.bodies[body_id_a as usize] = body_a;
        self.bodies[body_id_b as usize] = body_b;

        if (contact.flags & ContactFlags::SIM_MESH_CONTACT.0) != 0 {
            *contact.mesh_contact_mut() = MeshContactState::default();
        }

        if contact.island_id != b3::NULL_INDEX {
            // `unlink_contact` reads `contacts[contactId]` and the mutated local copy has not
            // been written back yet, exactly as in Swift.
            // Defined in `island.rs` (`World::unlink_contact`, from `Island.swift`).
            self.unlink_contact(contact_id);
            contact.island_id = b3::NULL_INDEX;
            contact.island_index = b3::NULL_INDEX;
        }

        // Remove contact from the array that owns it. Contacts in the awake set that are touching
        // live in the constraint graph and are handled by the solver.
        if contact.color_index == b3::NULL_INDEX {
            b3_assert(
                contact.set_index != SetType::AwakeSet.raw_value()
                    || (contact.flags & ContactFlags::TOUCHING.0) == 0,
            );
            let set_index = contact.set_index as usize;
            let local_index = contact.local_index;
            let moved_index = remove_swap_array(
                &mut self.solver_sets[set_index].contact_indices,
                local_index as usize,
            );
            if moved_index != b3::NULL_INDEX {
                let moved_contact_index =
                    self.solver_sets[set_index].contact_indices[local_index as usize];
                let mut moved_contact = self.contacts[moved_contact_index as usize];
                moved_contact.local_index = local_index;
                self.contacts[moved_contact_index as usize] = moved_contact;
            }
        } else {
            self.remove_contact_from_graph(
                body_id_a,
                body_id_b,
                contact.color_index,
                contact.local_index,
                (flags & ContactFlags::SIM_MESH_CONTACT.0) != 0,
            );
        }
        // Free contact and id (preserve generation)
        contact.contact_id = b3::NULL_INDEX;
        contact.set_index = b3::NULL_INDEX;
        contact.color_index = b3::NULL_INDEX;
        contact.local_index = b3::NULL_INDEX;
        self.contacts[contact_id as usize] = contact;
        self.contact_id_pool.free(contact_id);

        if wake_bodies && touching {
            self.wake_body(body_id_a);
            self.wake_body(body_id_b);
        }
    }

    /// Destroy all contacts attached to a body.
    pub(crate) fn destroy_body_contacts(&mut self, body_id: i32, wake_bodies: bool) {
        let mut edge_key = self.bodies[body_id as usize].head_contact_key;
        while edge_key != b3::NULL_INDEX {
            let contact_id = edge_key >> 1;
            let edge_index = (edge_key & 1) as usize;

            let contact = self.contacts[contact_id as usize];
            edge_key = contact.edges[edge_index].next_key;
            self.destroy_contact(contact_id, wake_bodies);
        }
    }

    // MARK: Pair generation

    /// Gather the sibling pairs with a moved node.
    pub(crate) fn gather_moved_siblings(
        &self,
        tree: &DynamicTree,
        pair_indices: &mut Vec<i32>,
    ) -> i32 {
        let mut count: i32 = 0;
        let mut pair: i32 = 2;
        while pair < tree.node_end {
            let flags1 = tree.nodes[pair as usize].flag_index;
            let flags2 = tree.nodes[pair as usize + 1].flag_index;
            if (flags1 | flags2) & MOVED_NODE_FLAG != 0 {
                pair_indices[count as usize] = pair;
                count += 1;
            }
            pair += 2;
        }

        count
    }

    /// Did either move and if so do they overlap?
    #[inline(always)]
    pub(crate) fn test_pair(&self, a: TreeNode, b: TreeNode) -> bool {
        if (a.flag_index | b.flag_index) & MOVED_NODE_FLAG == 0 {
            return false;
        }

        aabb_overlaps(a.aabb, b.aabb)
    }

    /// Queue a candidate and run the gauntlet in batches so the pair-set and shape loads can be
    /// prefetched. The pair keys are sorted afterwards, so batching does not
    /// change the result.
    #[inline(always)]
    pub(crate) fn add_candidate_pair(
        &mut self,
        shape_id_a: i32,
        shape_id_b: i32,
        pair_keys: &mut Vec<u64>,
    ) {
        self.candidate_batch[self.candidate_batch_count] =
            [min_int(shape_id_a, shape_id_b), max_int(shape_id_a, shape_id_b)];
        self.candidate_batch_count += 1;
        if self.candidate_batch_count == self.candidate_batch_size {
            self.flush_candidate_pairs(pair_keys);
        }
    }

    pub(crate) fn flush_candidate_pairs(&mut self, pair_keys: &mut Vec<u64>) {
        let count = self.candidate_batch_count;
        self.candidate_batch_count = 0;
        if count == 0 {
            return;
        }
        let mut plain = [[0i32; 2]; 32];
        let mut keys = [0u64; 32];
        let mut hashes = [0u32; 32];
        let mut plain_count = 0usize;
        // Swift wrapped the whole body in `shapes.withUnsafeBufferPointer`; the shape vector is
        // not mutated or grown during the traversal below, so the raw base pointer is equivalent.
        // SAFETY: `shape_base` stays valid for the whole function — `World::shapes` is untouched
        // while pair candidates are filtered, and every id below is a live shape index.
        let shape_base: *const Shape = self.shapes.as_ptr();
        for i in 0..count {
            let a = self.candidate_batch[i][0];
            let b = self.candidate_batch[i][1];
            // SAFETY: `a`/`b` were queued as live shape ids by `add_candidate_pair`.
            let shape_a = unsafe { &*shape_base.add(a as usize) };
            let shape_b = unsafe { &*shape_base.add(b as usize) };
            // Metadata rejects same-body pairs before touching reference-counted geometry.
            if shape_a.body_id == shape_b.body_id {
                continue;
            }
            if shape_a.type_ == ShapeType::CompoundShape {
                if let ShapeGeometry::Compound(compound) = &shape_a.geometry {
                    self.emit_compound_pairs(a, b, compound, pair_keys);
                    continue;
                }
            }
            if shape_b.type_ == ShapeType::CompoundShape {
                if let ShapeGeometry::Compound(compound) = &shape_b.geometry {
                    self.emit_compound_pairs(b, a, compound, pair_keys);
                    continue;
                }
            }
            plain[plain_count] = [a, b];
            plain_count += 1;
        }

        {
            // Prefetch hash set entries.
            let pair_set = &self.broad_phase.pair_set;
            for i in 0..plain_count {
                keys[i] = shape_pair_key(plain[i][0], plain[i][1], 0);
                hashes[i] = key_hash(keys[i]);
                pair_set.prefetch_hash_slot(hashes[i]);
            }

            // Cull existing pairs.
            let mut count2 = 0usize;
            for i in 0..plain_count {
                if pair_set.contains_hashed_key(keys[i], hashes[i]) == false {
                    plain[count2] = plain[i];
                    keys[count2] = keys[i];
                    count2 += 1;
                }
            }

            // Prefetch shapes.
            for i in 0..count2 {
                // SAFETY: `plain[i]` holds live shape ids; `count2` is bounded by `plain_count`.
                unsafe {
                    b3_prefetch(shape_base.add(plain[i][0] as usize));
                    b3_prefetch(shape_base.add(plain[i][1] as usize));
                }
            }

            for i in 0..count2 {
                // SAFETY: live shape ids into the unchanged `World::shapes` storage.
                let shape_a = unsafe { shape_base.add(plain[i][0] as usize) };
                let shape_b = unsafe { shape_base.add(plain[i][1] as usize) };
                if self.should_create_pair_with_shapes(plain[i][0], plain[i][1], shape_a, shape_b)
                {
                    pair_keys.push(keys[i]);
                }
            }
        }
    }

    pub(crate) fn collide_proxy_and_subtree(
        &mut self,
        proxy: TreeNode,
        nodes: &[TreeNode],
        pair_start: i32,
        pair_keys: &mut Vec<u64>,
        stack: *mut i32,
    ) {
        let proxy_mark = proxy.flag_index & MOVED_NODE_FLAG;
        let shape_id = proxy.shape_index();

        let mut stack_count = 0usize;
        // SAFETY: `stack` is `World::broad_phase_proxy_stack`, capacity `TREE_STACK_SIZE`,
        // and this push is guarded by the same depth check as the pushes below.
        unsafe {
            *stack.add(stack_count) = pair_start;
        }
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            // SAFETY: the slot was written by a push above; ids are plain `i32` values.
            let pair = unsafe { *stack.add(stack_count) };
            for i in 0..2 {
                let node = nodes[pair as usize + i];
                if (node.flag_index | proxy_mark) & MOVED_NODE_FLAG == 0 {
                    continue;
                }

                if aabb_overlaps(proxy.aabb, node.aabb) == false {
                    continue;
                }

                if is_leaf(node) {
                    self.add_candidate_pair(shape_id, node.shape_index(), pair_keys);
                } else {
                    if stack_count < TREE_STACK_SIZE {
                        // SAFETY: bounded by `TREE_STACK_SIZE`, the allocated stack capacity.
                        unsafe {
                            *stack.add(stack_count) = left_child(node);
                        }
                        stack_count += 1;
                    } else {
                        b3_assert(stack_count < TREE_STACK_SIZE);
                    }
                }
            }
        }
    }

    #[inline(always)]
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn visit_pair(
        &mut self,
        array_a: &[TreeNode],
        array_b: &[TreeNode],
        node_a: TreeNode,
        node_b: TreeNode,
        stack: *mut TraversalPair,
        stack_count: &mut usize,
        pair_keys: &mut Vec<u64>,
        proxy_stack: *mut i32,
    ) {
        if self.test_pair(node_a, node_b) == false {
            return;
        }

        let leaf_a = is_leaf(node_a);
        let leaf_b = is_leaf(node_b);
        if leaf_a && leaf_b {
            self.add_candidate_pair(node_a.shape_index(), node_b.shape_index(), pair_keys);
        } else if leaf_a {
            self.collide_proxy_and_subtree(node_a, array_b, left_child(node_b), pair_keys, proxy_stack);
        } else if leaf_b {
            self.collide_proxy_and_subtree(node_b, array_a, left_child(node_a), pair_keys, proxy_stack);
        } else {
            if *stack_count < TREE_STACK_SIZE {
                // SAFETY: `stack` is `World::broad_phase_pair_stack` with capacity
                // `TREE_STACK_SIZE`; the entry is fully initialised here before the matching pop.
                unsafe {
                    *stack.add(*stack_count) = TraversalPair {
                        a: left_child(node_a),
                        b: left_child(node_b),
                    };
                }
                *stack_count += 1;
            } else {
                b3_assert(*stack_count < TREE_STACK_SIZE);
            }
        }
    }

    pub(crate) fn collide_cross_pairs(
        &mut self,
        array_a: &[TreeNode],
        array_b: &[TreeNode],
        subtree_a: TreeNode,
        subtree_b: TreeNode,
        pair_keys: &mut Vec<u64>,
    ) {
        let mut stack_count = 0usize;
        let pair_stack = self.broad_phase_pair_stack;
        let proxy_stack = self.broad_phase_proxy_stack;

        // Seed the stack.
        self.visit_pair(
            array_a,
            array_b,
            subtree_a,
            subtree_b,
            pair_stack,
            &mut stack_count,
            pair_keys,
            proxy_stack,
        );

        while stack_count > 0 {
            stack_count -= 1;
            // SAFETY: the slot was seeded or pushed by `visit_pair` before this pop;
            // `TraversalPair` is a plain POD entry.
            let pair = unsafe { *pair_stack.add(stack_count) };
            for i in 0..2 {
                for j in 0..2 {
                    self.visit_pair(
                        array_a,
                        array_b,
                        array_a[pair.a as usize + i],
                        array_b[pair.b as usize + j],
                        pair_stack,
                        &mut stack_count,
                        pair_keys,
                        proxy_stack,
                    );
                }
            }
        }
    }

    /// This does a serial cross-tree breadth first search until the queue is full.
    /// The queue and result arrays are caller-owned scratch so a step does not allocate them.
    pub(crate) fn gather_cross_seeds(
        &self,
        tree_a: &DynamicTree,
        tree_b: &DynamicTree,
        queue: &mut Vec<NodePair>,
        seeds: &mut Vec<NodePair>,
    ) {
        let nodes_a = &tree_a.nodes;
        let nodes_b = &tree_b.nodes;

        let cross_seed_count = 64usize;
        let queue_capacity = 2 * cross_seed_count;
        if queue.len() < queue_capacity {
            queue.extend(
                std::iter::repeat(NodePair::default())
                    .take(queue_capacity - queue.len()),
            );
        }
        let mask = queue_capacity - 1;
        let mut head = 0usize;
        let mut tail = 0usize;

        seeds.clear();
        seeds.reserve(cross_seed_count);

        let root_a = nodes_a[ROOT_NODE as usize];
        let root_b = nodes_b[ROOT_NODE as usize];
        if self.test_pair(root_a, root_b) {
            queue[tail & mask] = NodePair::new(root_a, root_b);
            tail += 1;
        }

        while head < tail && seeds.len() + (tail - head) + 3 < cross_seed_count {
            let pair = queue[head & mask];
            head += 1;

            if is_leaf(pair.a) || is_leaf(pair.b) {
                seeds.push(pair);
                continue;
            }

            let a = left_child(pair.a) as usize;
            let b = left_child(pair.b) as usize;

            // Nodes have two children each, so four combinations.
            for i in 0..2 {
                for j in 0..2 {
                    if self.test_pair(nodes_a[a + i], nodes_b[b + j]) {
                        queue[tail & mask] = NodePair::new(nodes_a[a + i], nodes_b[b + j]);
                        tail += 1;
                    }
                }
            }
        }

        while head < tail {
            seeds.push(queue[head & mask]);
            head += 1;
        }
    }

    /// Generate broad-phase pairs and create contacts for new pairs.
    pub(crate) fn update_broad_phase_pairs(&mut self) {
        let static_index = BodyType::StaticBody as usize;
        let kinematic_index = BodyType::KinematicBody as usize;
        let dynamic_index = BodyType::DynamicBody as usize;

        let mut need_update = self.broad_phase.trees[static_index].has_moved();
        need_update = need_update || self.broad_phase.trees[kinematic_index].needs_rebuild();
        need_update = need_update || self.broad_phase.trees[dynamic_index].needs_rebuild();

        if need_update == false {
            return;
        }

        // Swift mutates the world-owned scratch arrays through `inout` while borrowing the trees
        // (disjoint stored properties). Rust cannot express a `&mut self` call with concurrent
        // field borrows, so the scratch is taken out for the traversal and restored afterwards;
        // contents, ordering and capacities — and therefore results — are unchanged.
        let mut pair_keys = std::mem::take(&mut self.pair_key_scratch);
        pair_keys.clear();
        if pair_keys.capacity() < 64 {
            pair_keys.reserve(64);
        }

        // Get the sibling pairs of the dynamic body tree that have moved. Size the buffer from the
        // actual pair count so a rebuild that leaves an odd nodeEnd cannot overflow it.
        let pair_capacity =
            max_int((self.broad_phase.trees[dynamic_index].node_end - 2) / 2 + 1, 1);
        let mut moved_sibling_scratch = std::mem::take(&mut self.moved_sibling_scratch);
        if moved_sibling_scratch.len() < pair_capacity as usize {
            moved_sibling_scratch.extend(
                std::iter::repeat(0i32)
                    .take(pair_capacity as usize - moved_sibling_scratch.len()),
            );
        }
        let dynamic_move_count =
            self.gather_moved_siblings(&self.broad_phase.trees[dynamic_index], &mut moved_sibling_scratch);

        // Get seeds for colliding against the static and kinematic trees.
        let mut cross_seed_queue = std::mem::take(&mut self.cross_seed_queue);
        let mut static_seed_scratch = std::mem::take(&mut self.static_seed_scratch);
        let mut kinematic_seed_scratch = std::mem::take(&mut self.kinematic_seed_scratch);
        self.gather_cross_seeds(
            &self.broad_phase.trees[dynamic_index],
            &self.broad_phase.trees[static_index],
            &mut cross_seed_queue,
            &mut static_seed_scratch,
        );
        self.gather_cross_seeds(
            &self.broad_phase.trees[dynamic_index],
            &self.broad_phase.trees[kinematic_index],
            &mut cross_seed_queue,
            &mut kinematic_seed_scratch,
        );
        b3_assert(static_seed_scratch.len() <= 64);
        b3_assert(kinematic_seed_scratch.len() <= 64);

        // End all read borrows before rebuilding the trees.
        // SAFETY: Swift nested `withUnsafeBufferPointer` over the three trees' node arrays. The
        // arrays are read-only for the whole traversal — `rebuild`/`clear_moved` below run only
        // after the last use — and the writes during the traversal touch only the candidate
        // batch and `pair_keys`, which are disjoint storage.
        let dynamic_nodes: &[TreeNode] = unsafe {
            let tree = &self.broad_phase.trees[dynamic_index];
            std::slice::from_raw_parts(tree.nodes.as_ptr(), tree.nodes.len())
        };
        let static_nodes: &[TreeNode] = unsafe {
            let tree = &self.broad_phase.trees[static_index];
            std::slice::from_raw_parts(tree.nodes.as_ptr(), tree.nodes.len())
        };
        let kinematic_nodes: &[TreeNode] = unsafe {
            let tree = &self.broad_phase.trees[kinematic_index];
            std::slice::from_raw_parts(tree.nodes.as_ptr(), tree.nodes.len())
        };

        // Collide the dynamic body tree against the static and kinematic trees.
        for seed in static_seed_scratch.iter() {
            self.collide_cross_pairs(dynamic_nodes, static_nodes, seed.a, seed.b, &mut pair_keys);
        }

        for seed in kinematic_seed_scratch.iter() {
            self.collide_cross_pairs(
                dynamic_nodes,
                kinematic_nodes,
                seed.a,
                seed.b,
                &mut pair_keys,
            );
        }

        // Collide the dynamic body tree against itself.
        for i in 0..dynamic_move_count as usize {
            let node_index = moved_sibling_scratch[i];
            self.collide_cross_pairs(
                dynamic_nodes,
                dynamic_nodes,
                dynamic_nodes[node_index as usize],
                dynamic_nodes[node_index as usize + 1],
                &mut pair_keys,
            );
        }

        // Restore the scratch the traversal no longer needs (Swift kept them in place all along;
        // nothing else reads them between here and the end of the function).
        self.moved_sibling_scratch = moved_sibling_scratch;
        self.cross_seed_queue = cross_seed_queue;
        self.static_seed_scratch = static_seed_scratch;
        self.kinematic_seed_scratch = kinematic_seed_scratch;

        self.flush_candidate_pairs(&mut pair_keys);

        self.broad_phase.trees[static_index].clear_moved();

        // Update stale trees.
        self.broad_phase.trees[dynamic_index].rebuild(false);
        self.broad_phase.trees[kinematic_index].rebuild(false);

        // Pairs arrive in deterministic order but scrambled relative to body and shape order.
        // Sort them here.
        pair_keys.sort();

        for i in 0..pair_keys.len() {
            let key = pair_keys[i];

            // The traversal is cross subtree and the seeds are disjoint, so a duplicate key means
            // a real bug. Skip rather than double add to the pair set.
            let duplicate = i > 0 && key == pair_keys[i - 1];
            b3_validate(duplicate == false);
            if duplicate {
                continue;
            }

            let shape_id_a =
                ((key >> (64 - b3::SHAPE_POWER as u64)) & (b3::MAX_SHAPES - 1) as u64) as i32;
            let shape_id_b =
                ((key >> (64 - 2 * b3::SHAPE_POWER) as u64) & (b3::MAX_SHAPES - 1) as u64) as i32;
            let child_index = (key & (b3::MAX_CHILD_SHAPES - 1) as u64) as i32;

            self.create_contact(shape_id_a, shape_id_b, child_index);
        }

        self.pair_key_scratch = pair_keys;
    }
}
