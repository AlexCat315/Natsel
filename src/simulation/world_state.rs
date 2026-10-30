//! World state snapshots for rollback, replays, and lockstep resynchronization (Jolt's
//! StateRecorder with all state flags). A snapshot holds every piece of state that influences
//! later steps: bodies and solver sets, shapes and broad-phase trees, contacts with their manifolds
//! and warm starting impulses, the constraint graph, islands, joints and their side stores,
//! sensors, vehicles, soft bodies, id pools, pending events, and the simulation settings. Restoring
//! it and stepping gives the same bits as stepping from the moment it was taken, even after bodies,
//! shapes, or joints were created or destroyed in between.
//!
//! Not part of the state: shape geometry objects (hulls, meshes, height fields, compounds and
//! constraint paths are shared by reference and must not be edited in place), user callbacks,
//! user data pointers' targets, `worker_count` (results do not depend on it), and character
//! controllers, which live outside the world and have their own `save_state`.
//!
//! Translated from `Sources/Natsel/Simulation/WorldState.swift`.

use std::sync::Arc;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::dynamic_tree::{DynamicTree, TreeNode, TreeProxy};
use crate::containers::bit_set::BitSet;
use crate::containers::dynamic_array::DynamicArray;
use crate::containers::hash_set::HashSet;
use crate::containers::id_pool::IDPool;
use crate::containers::name_cache::NameCache;
use crate::math::types::{AABB, Vec3};
use crate::simulation::contact_payload::ContactPayload;
use crate::simulation::constraint_graph::GraphColor;
use crate::simulation::end_event_buffer::EndEventBuffer;
use crate::simulation::path_joint::{ConstraintPath, PathJointData};
use crate::simulation::sensor::Sensor;
use crate::simulation::shape_creation::{HullDatabase, HullDatabaseSnapshot};
use crate::simulation::shape_runtime::Shape;
use crate::simulation::six_dof_joint::SixDOFJointData;
use crate::simulation::world_runtime::{
    Body, BodySim, BodyState, Contact, ContactLink, Island, IslandSim, Joint, JointLink, JointSim,
    SolverSet, World,
};
use crate::soft_body::soft_body_runtime::SoftBodyRecord;
use crate::types::events::{
    BodyMoveEvent, ContactBeginTouchEvent, ContactEndTouchEvent, ContactHitEvent, JointEvent,
    SensorBeginTouchEvent, SensorEndTouchEvent,
};
use crate::vehicle::vehicle_runtime::VehicleRecord;

// MARK: - Clones of manually managed containers

impl<T: Copy + Default> DynamicArray<T> {
    /// The copy owns new storage. Swift's `cloned()` allocated a fresh buffer at the source
    /// capacity, copied `count` elements, and set the count; `Vec::clone` already preserves both
    /// length (count) and capacity, so the derived clone is bit-for-bit equivalent.
    pub(crate) fn cloned(&self) -> Self {
        self.clone()
    }
}

impl IDPool {
    /// Swift's `cloned()` copied the pool struct then deep-copied `free_array`; the derived
    /// `Clone` already deep-copies the owned `DynamicArray`, so the result is identical: a pool
    /// with its own free-list storage and the same `next_index`.
    pub(crate) fn cloned(&self) -> Self {
        self.clone()
    }
}

impl BitSet {
    /// Swift's `cloned()` built a `BitSet(bitCapacity: blockCapacity * 64)`, copied `blockCount`,
    /// and updated the words from the source. `Vec::clone` preserves the word buffer exactly
    /// (`bits.len() == block_capacity`) plus `block_capacity` and `block_count`, so the derived
    /// clone matches without re-deriving the capacity.
    pub(crate) fn cloned(&self) -> Self {
        self.clone()
    }
}

impl HashSet {
    /// Swift's `cloned()` allocated a fresh `SetItem` buffer of `capacity` entries and copied the
    /// probe table with it; the derived `Clone` copies `items`, `capacity`, and `count` into new
    /// storage, which is the same snapshot. An empty set (capacity 0) clones to an empty set,
    /// matching Swift's early return.
    pub(crate) fn cloned(&self) -> Self {
        self.clone()
    }
}

// MARK: - Dynamic tree state

/// The persistent part of a dynamic tree. Rebuild scratch is left out.
#[derive(Clone, Debug, Default)]
pub(crate) struct DynamicTreeState {
    pub(crate) nodes: Vec<TreeNode>,
    pub(crate) parents: Vec<i32>,
    pub(crate) proxies: Vec<TreeProxy>,
    pub(crate) node_end: i32,
    pub(crate) node_capacity: i32,
    pub(crate) pair_free_list: i32,
    pub(crate) proxy_count: i32,
    pub(crate) proxy_capacity: i32,
    pub(crate) proxy_free_list: i32,
    pub(crate) dfs_ordered: bool,
}

impl DynamicTree {
    /// Swift's `extension DynamicTree { var state: DynamicTreeState }`. The node, parent, and
    /// proxy buffers are copied into the snapshot; `swap_nodes`, `rebuild_capacity`,
    /// `rebuild_stack`, and the other rebuild scratch stay behind.
    pub(crate) fn state(&self) -> DynamicTreeState {
        DynamicTreeState {
            nodes: self.nodes.clone(),
            parents: self.parents.clone(),
            proxies: self.proxies.clone(),
            node_end: self.node_end,
            node_capacity: self.node_capacity,
            pair_free_list: self.pair_free_list,
            proxy_count: self.proxy_count,
            proxy_capacity: self.proxy_capacity,
            proxy_free_list: self.proxy_free_list,
            dfs_ordered: self.dfs_ordered,
        }
    }

    /// Swift's `func restore(_ state: DynamicTreeState)`. Copy semantics mirror Swift's array
    /// assignment: the tree ends up owning the saved contents exactly.
    pub(crate) fn restore(&mut self, state: &DynamicTreeState) {
        self.nodes = state.nodes.clone();
        self.parents = state.parents.clone();
        self.proxies = state.proxies.clone();
        self.node_end = state.node_end;
        self.node_capacity = state.node_capacity;
        self.pair_free_list = state.pair_free_list;
        self.proxy_count = state.proxy_count;
        self.proxy_capacity = state.proxy_capacity;
        self.proxy_free_list = state.proxy_free_list;
        self.dfs_ordered = state.dfs_ordered;
    }
}

// MARK: - World state

/// A saved world state. Holds its own copies, so it can be restored any number of times. Only
/// the world that saved it can restore it.
///
/// Swift's `public final class WorldState` with a `weak var world: World?`; here `World` is an
/// owned struct, so the owning world is recorded as an identity-only raw pointer (the weak
/// reference's role was solely the `state.world === self` ownership check) and the class becomes
/// a plain struct owned by the caller.
#[derive(Clone, Debug, Default)]
pub struct WorldState {
    /// Identity of the saving world. Never dereferenced; only compared for address equality.
    world: Option<*const World>,
    /// Step count of the world when the state was saved.
    pub step_index: u64,

    // Bodies, shapes, and solver sets.
    bodies: Vec<Body>,
    body_id_pool: IDPool,
    solver_sets: Vec<SolverSet>,
    solver_set_id_pool: IDPool,
    islands: Vec<Island>,
    island_id_pool: IDPool,
    shapes: Vec<Shape>,
    shape_id_pool: IDPool,
    fat_aabbs: Vec<AABB>,
    hull_database: HullDatabaseSnapshot,
    names: NameCache,

    // Broad phase.
    trees: Vec<DynamicTreeState>,
    moved_siblings: Vec<i32>,
    pair_set: HashSet,

    // Contacts and the constraint graph.
    contacts: Vec<Contact>,
    /// Payload copies keyed by contact slot index; only contacts with a live id and payload are
    /// recorded, in contact-array order.
    payloads: Vec<(usize, ContactPayload)>,
    contact_id_pool: IDPool,
    colors: Vec<GraphColor>,

    // Joints.
    joints: Vec<Joint>,
    joint_id_pool: IDPool,
    six_dof: Vec<SixDOFJointData>,
    six_dof_pool: IDPool,
    path_joints: Vec<PathJointData>,
    /// Paths are shared, not copied: the snapshot keeps the same `Arc` allocations as the world,
    /// mirroring Swift's class references (a path must not be edited in place).
    path_joint_paths: Vec<Option<Arc<ConstraintPath>>>,
    path_joint_pool: IDPool,

    // Other systems.
    sensors: Vec<Sensor>,
    vehicles: Vec<VehicleRecord>,
    vehicle_id_pool: IDPool,
    soft_bodies: Vec<SoftBodyRecord>,
    soft_body_id_pool: IDPool,

    // Events of the last step and end events queued since.
    body_move_events: Vec<BodyMoveEvent>,
    sensor_begin_events: Vec<SensorBeginTouchEvent>,
    sensor_end_events: EndEventBuffer<SensorEndTouchEvent>,
    contact_begin_events: Vec<ContactBeginTouchEvent>,
    contact_end_events: EndEventBuffer<ContactEndTouchEvent>,
    contact_hit_events: Vec<ContactHitEvent>,
    continuous_impact_events: Vec<ContactHitEvent>,
    joint_events: Vec<JointEvent>,

    // Step scalars.
    inv_h: f32,
    inv_dt: f32,
    split_island_id: i32,
    compound_shape_count: i32,

    // Settings.
    settings: WorldSettingsState,
}

/// Settings that change results, saved with the state.
#[derive(Clone, Debug)]
pub(crate) struct WorldSettingsState {
    pub(crate) gravity: Vec3,
    pub(crate) hit_event_threshold: f32,
    pub(crate) restitution_threshold: f32,
    pub(crate) restitution_iterations: i32,
    pub(crate) enable_restitution_propagation: bool,
    pub(crate) max_linear_speed: f32,
    pub(crate) isotropic_inertia_tolerance: f32,
    pub(crate) contact_speed: f32,
    pub(crate) contact_hertz: f32,
    pub(crate) contact_damping_ratio: f32,
    pub(crate) contact_recycle_distance: f32,
    pub(crate) enable_sleep: bool,
    pub(crate) enable_warm_starting: bool,
    pub(crate) enable_continuous: bool,
    pub(crate) enable_dynamic_continuous: bool,
    pub(crate) enable_soft_body_query_acceleration: bool,
    pub(crate) enable_speculative: bool,
    pub(crate) enable_wide_solver: bool,
}

impl Default for WorldSettingsState {
    /// Swift's property defaults: thresholds start at zero, sleep, warm starting, continuous,
    /// soft-body query acceleration, speculative contacts, and the wide solver start enabled,
    /// dynamic continuous and restitution propagation start disabled.
    #[inline]
    fn default() -> Self {
        Self {
            gravity: Vec3::default(),
            hit_event_threshold: 0.0,
            restitution_threshold: 0.0,
            restitution_iterations: 0,
            enable_restitution_propagation: false,
            max_linear_speed: 0.0,
            isotropic_inertia_tolerance: 0.0,
            contact_speed: 0.0,
            contact_hertz: 0.0,
            contact_damping_ratio: 0.0,
            contact_recycle_distance: 0.0,
            enable_sleep: true,
            enable_warm_starting: true,
            enable_continuous: true,
            enable_dynamic_continuous: false,
            enable_soft_body_query_acceleration: true,
            enable_speculative: true,
            enable_wide_solver: true,
        }
    }
}

impl Drop for WorldState {
    /// Swift's `deinit`: release every pool and set the snapshot owns before the buffers drop.
    /// The Vec-backed Rust containers free themselves on unwind; the explicit destroys mirror the
    /// Swift teardown order and reset each container to its empty state first.
    fn drop(&mut self) {
        self.body_id_pool.destroy();
        self.solver_set_id_pool.destroy();
        self.island_id_pool.destroy();
        self.shape_id_pool.destroy();
        self.contact_id_pool.destroy();
        self.joint_id_pool.destroy();
        self.six_dof_pool.destroy();
        self.path_joint_pool.destroy();
        self.vehicle_id_pool.destroy();
        self.soft_body_id_pool.destroy();
        self.pair_set.destroy();
        for i in 0..self.colors.len() {
            self.colors[i].body_set.destroy();
        }
    }
}

impl WorldState {
    /// Approximate heap size of the snapshot's own copies, in bytes.
    pub fn byte_count(&self) -> usize {
        let mut total =
            self.bodies.len() * std::mem::size_of::<Body>()
                + self.shapes.len() * std::mem::size_of::<Shape>();
        total += self.fat_aabbs.len() * std::mem::size_of::<AABB>()
            + self.contacts.len() * std::mem::size_of::<Contact>();
        total += self.payloads.len() * std::mem::size_of::<ContactPayload>()
            + self.joints.len() * std::mem::size_of::<Joint>();
        for set in &self.solver_sets {
            total += set.body_sims.len() * std::mem::size_of::<BodySim>();
            total += set.body_states.len() * std::mem::size_of::<BodyState>();
            total += set.joint_sims.len() * std::mem::size_of::<JointSim>();
            total += set.contact_indices.len() * 4 + set.island_sims.len()
                * std::mem::size_of::<IslandSim>();
        }
        for tree in &self.trees {
            total += tree.nodes.len() * std::mem::size_of::<TreeNode>()
                + tree.parents.len() * 4;
            total += tree.proxies.len() * std::mem::size_of::<TreeProxy>();
        }
        for color in &self.colors {
            total += color.joint_sims.len() * std::mem::size_of::<JointSim>();
            total += color.contacts.len()
                * std::mem::size_of::<crate::simulation::constraint_graph::ContactSpec>()
                + color.convex_contacts.len() * 4;
        }
        for island in &self.islands {
            total += island.bodies.len() * 4
                + island.contacts.len() * std::mem::size_of::<ContactLink>();
            total += island.joints.len() * std::mem::size_of::<JointLink>();
        }
        total
    }
}

// MARK: - World save and restore

/// Swift's `private func replacePool(_:_:)`: destroy the live pool and install a fresh deep
/// copy of the saved one, so the world's pools never alias the snapshot's storage.
fn replace_pool(pool: &mut IDPool, saved: &IDPool) {
    pool.destroy();
    *pool = saved.cloned();
}

impl World {
    /// Save everything that influences later steps. Not allowed during a step or a callback.
    pub fn save_state(&self) -> WorldState {
        b3_assert(!self.locked);
        // `WorldState` implements `Drop`, so a functional update from `WorldState::default()`
        // cannot be used: it would move fields out of a `Drop` type (E0509). Build the default
        // value first, then overwrite the two fields the snapshot needs.
        let mut state = WorldState::default();
        state.step_index = self.step_index;
        state.world = Some(core::ptr::from_ref(self));

        state.bodies = self.bodies.clone();
        state.body_id_pool = self.body_id_pool.cloned();
        state.solver_sets = self.solver_sets.clone();
        state.solver_set_id_pool = self.solver_set_id_pool.cloned();
        state.islands = self.islands.clone();
        state.island_id_pool = self.island_id_pool.cloned();
        state.shapes = self.shapes.clone();
        state.shape_id_pool = self.shape_id_pool.cloned();
        state.fat_aabbs = self.fat_aabbs.clone();
        state.hull_database = self.hull_database.snapshot();
        state.names = self.names.clone();

        state.trees = self.broad_phase.trees.iter().map(|tree| tree.state()).collect();
        state.moved_siblings = self.broad_phase.moved_siblings.clone();
        state.pair_set = self.broad_phase.pair_set.cloned();

        state.contacts = self.contacts.clone();
        state.payloads.reserve(self.contacts.len());
        for (index, contact) in self.contacts.iter().enumerate() {
            if contact.contact_id != b3::NULL_INDEX {
                if let Some(payload) = contact.payload {
                    // SAFETY: `Contact::payload` is only ever `Some` of a pointer into the
                    // `ContactPayloadPool` slab for this live contact; the slot is initialized
                    // for the whole time the contact id is allocated, so the read is in-bounds,
                    // aligned, and of an initialized `ContactPayload`.
                    let copy = unsafe { (*payload).clone() };
                    state.payloads.push((index, copy));
                }
            }
        }
        state.contact_id_pool = self.contact_id_pool.cloned();
        state.colors = self.constraint_graph.colors.clone();
        for i in 0..state.colors.len() {
            state.colors[i].body_set = self.constraint_graph.colors[i].body_set.cloned();
        }

        state.joints = self.joints.clone();
        state.joint_id_pool = self.joint_id_pool.cloned();
        state.six_dof = self.six_dof_store.elements().to_vec();
        state.six_dof_pool = self.six_dof_pool.cloned();
        state.path_joints = self.path_joint_store.elements().to_vec();
        state.path_joint_paths = self.path_joint_paths.clone();
        state.path_joint_pool = self.path_joint_pool.cloned();

        state.sensors = self.sensors.clone();
        state.vehicles = self.vehicles.clone();
        state.vehicle_id_pool = self.vehicle_id_pool.cloned();
        state.soft_bodies = self.soft_bodies.clone();
        state.soft_body_id_pool = self.soft_body_id_pool.cloned();

        state.body_move_events = self.body_move_events.clone();
        state.sensor_begin_events = self.sensor_begin_events.clone();
        state.sensor_end_events = self.sensor_end_events.clone();
        state.contact_begin_events = self.contact_begin_events.clone();
        state.contact_end_events = self.contact_end_events.clone();
        state.contact_hit_events = self.contact_hit_events.clone();
        state.continuous_impact_events = self.continuous_impact_events.clone();
        state.joint_events = self.joint_events.clone();

        state.inv_h = self.inv_h;
        state.inv_dt = self.inv_dt;
        state.split_island_id = self.split_island_id;
        state.compound_shape_count = self.compound_shape_count;

        let mut settings = WorldSettingsState::default();
        settings.gravity = self.gravity;
        settings.hit_event_threshold = self.hit_event_threshold;
        settings.restitution_threshold = self.restitution_threshold;
        settings.restitution_iterations = self.restitution_iterations;
        settings.enable_restitution_propagation = self.enable_restitution_propagation;
        settings.max_linear_speed = self.max_linear_speed;
        settings.isotropic_inertia_tolerance = self.isotropic_inertia_tolerance;
        settings.contact_speed = self.contact_speed;
        settings.contact_hertz = self.contact_hertz;
        settings.contact_damping_ratio = self.contact_damping_ratio;
        settings.contact_recycle_distance = self.contact_recycle_distance;
        settings.enable_sleep = self.enable_sleep;
        settings.enable_warm_starting = self.enable_warm_starting;
        settings.enable_continuous = self.enable_continuous;
        settings.enable_dynamic_continuous = self.enable_dynamic_continuous;
        settings.enable_soft_body_query_acceleration = self.enable_soft_body_query_acceleration;
        settings.enable_speculative = self.enable_speculative;
        settings.enable_wide_solver = self.enable_wide_solver;
        state.settings = settings;
        state
    }

    /// Return the world to a saved state. Returns false when the state belongs to another world or
    /// the world is locked. Ids created after the save become invalid; ids destroyed after the save
    /// are valid again.
    pub fn restore_state(&mut self, state: &WorldState) -> bool {
        if state.world != Some(core::ptr::from_ref(self)) || self.locked {
            return false;
        }
        self.soft_body_ray_caches.clear();

        self.bodies = state.bodies.clone();
        replace_pool(&mut self.body_id_pool, &state.body_id_pool);
        self.solver_sets = state.solver_sets.clone();
        replace_pool(&mut self.solver_set_id_pool, &state.solver_set_id_pool);
        self.islands = state.islands.clone();
        replace_pool(&mut self.island_id_pool, &state.island_id_pool);
        self.shapes = state.shapes.clone();
        replace_pool(&mut self.shape_id_pool, &state.shape_id_pool);
        self.fat_aabbs = state.fat_aabbs.clone();
        self.hull_database.restore(state.hull_database.clone());
        self.names = state.names.clone();

        for (tree, saved) in self.broad_phase.trees.iter_mut().zip(state.trees.iter()) {
            tree.restore(saved);
        }
        self.broad_phase.moved_siblings = state.moved_siblings.clone();
        self.broad_phase.pair_set.destroy();
        self.broad_phase.pair_set = state.pair_set.cloned();

        self.contacts = state.contacts.clone();
        // Payload slots are keyed by contact id and never move, so the saved pointers are valid.
        for (index, payload) in &state.payloads {
            if let Some(slot) = self.contacts[*index].payload {
                // SAFETY: the restored `contacts` array was saved alongside these pointers from
                // the same world, and the `ContactPayloadPool` slabs are stable for the world's
                // lifetime, so `slot` is an aligned, initialized, live `ContactPayload`. The
                // assignment drops the current payload's owned buffers exactly as Swift's
                // `pointee = payload` did.
                unsafe {
                    *slot = payload.clone();
                }
            }
        }
        replace_pool(&mut self.contact_id_pool, &state.contact_id_pool);
        for i in 0..self.constraint_graph.colors.len() {
            self.constraint_graph.colors[i].body_set.destroy();
        }
        self.constraint_graph.colors = state.colors.clone();
        for i in 0..state.colors.len() {
            self.constraint_graph.colors[i].body_set = state.colors[i].body_set.cloned();
        }

        self.joints = state.joints.clone();
        replace_pool(&mut self.joint_id_pool, &state.joint_id_pool);
        self.six_dof_store.replace(&state.six_dof);
        replace_pool(&mut self.six_dof_pool, &state.six_dof_pool);
        self.path_joint_store.replace(&state.path_joints);
        self.path_joint_paths = state.path_joint_paths.clone();
        replace_pool(&mut self.path_joint_pool, &state.path_joint_pool);

        self.sensors = state.sensors.clone();
        self.vehicles = state.vehicles.clone();
        replace_pool(&mut self.vehicle_id_pool, &state.vehicle_id_pool);
        self.soft_bodies = state.soft_bodies.clone();
        replace_pool(&mut self.soft_body_id_pool, &state.soft_body_id_pool);

        self.body_move_events = state.body_move_events.clone();
        self.sensor_begin_events = state.sensor_begin_events.clone();
        self.sensor_end_events = state.sensor_end_events.clone();
        self.contact_begin_events = state.contact_begin_events.clone();
        self.contact_end_events = state.contact_end_events.clone();
        self.contact_hit_events = state.contact_hit_events.clone();
        self.continuous_impact_events = state.continuous_impact_events.clone();
        self.joint_events = state.joint_events.clone();

        self.step_index = state.step_index;
        self.inv_h = state.inv_h;
        self.inv_dt = state.inv_dt;
        self.split_island_id = state.split_island_id;
        self.compound_shape_count = state.compound_shape_count;

        let settings = &state.settings;
        self.gravity = settings.gravity;
        self.hit_event_threshold = settings.hit_event_threshold;
        self.restitution_threshold = settings.restitution_threshold;
        self.restitution_iterations = settings.restitution_iterations;
        self.enable_restitution_propagation = settings.enable_restitution_propagation;
        self.max_linear_speed = settings.max_linear_speed;
        self.isotropic_inertia_tolerance = settings.isotropic_inertia_tolerance;
        self.contact_speed = settings.contact_speed;
        self.contact_hertz = settings.contact_hertz;
        self.contact_damping_ratio = settings.contact_damping_ratio;
        self.contact_recycle_distance = settings.contact_recycle_distance;
        self.enable_sleep = settings.enable_sleep;
        self.enable_warm_starting = settings.enable_warm_starting;
        self.enable_continuous = settings.enable_continuous;
        self.enable_dynamic_continuous = settings.enable_dynamic_continuous;
        self.enable_soft_body_query_acceleration = settings.enable_soft_body_query_acceleration;
        self.enable_speculative = settings.enable_speculative;
        self.enable_wide_solver = settings.enable_wide_solver;
        true
    }
}
