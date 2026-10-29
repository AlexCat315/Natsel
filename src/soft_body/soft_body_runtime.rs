//! Soft body instances: creation, state access, tuning, and sleeping.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodyRuntime.swift`.
//!
//! This file owns the soft-body handle ([`SoftBodyID`]), the public definition
//! ([`SoftBodyDef`]), the inspection results ([`SoftBodyContact`], [`SoftBodyRayResult`]), the
//! per-body particle state ([`SoftBodyRecord`]) and the world-facing soft-body API over
//! `World::soft_bodies` / `World::soft_body_id_pool` / `World::soft_body_ray_caches` (all declared
//! in `crate::simulation::world_runtime`). The rest-state types (`SoftBodySettings`,
//! `SoftBodyVertex`, `SoftBodyEdge`, `SoftBodyFace`, `SoftBodySkinned`, ...) live in
//! `crate::soft_body::soft_body_settings`; the wide constraint batches in
//! `crate::soft_body::soft_body_batches`; the contact thickness, the self-contact exclusion tables
//! and the soft-soft contact solve in `crate::soft_body::soft_body_contacts`; the edge planes in
//! `crate::soft_body::soft_body_edge_collision`; the per-step vertex collision in
//! `crate::soft_body::soft_body_collision`; the integration and broad-phase registration in
//! `crate::soft_body::soft_body_solver`. Ray queries are in `crate::soft_body::soft_body_queries`
//! and `crate::soft_body::soft_body_ray_cache`, both already landed against the field shapes below.
//!
//! Foreign vocabulary assumed from those sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::soft_body::soft_body_settings`: `SoftBodySettings` (`Clone + Debug + Default`) with
//!   `vertices: Vec<SoftBodyVertex>` (`position: Vec3`, `velocity: Vec3`, `inv_mass: f32`),
//!   `faces: Vec<SoftBodyFace>` (`face.vertex.{x, y, z}`), `edges: Vec<SoftBodyEdge>`
//!   (`edge.vertex.{x, y}`, `edge.rest_length: f32`), `skinned: Vec<SoftBodySkinned>`
//!   (`vertex`, `joints: [u32; 4]`, `weights: [f32; 4]`, `normal_faces: Vec<u32>` - the Swift
//!   `SIMD4` lanes are indexed by `usize`), `inverse_bind_matrices: Vec<Transform>`,
//!   `patches` (only `is_empty()` is read here),
//!   `pub(crate) fn build_patches(&mut self)` and `pub fn validate(&self) -> Option<String>`
//!   (Swift's `validate() -> String?`).
//! - `crate::soft_body::soft_body_batches`: `EdgeBatch`, `BendBatch`, `VolumeBatch` (each
//!   `Clone + Copy + Debug`) and `pub(crate) fn build_batches(&mut self)` on `SoftBodyRecord`.
//! - `crate::soft_body::soft_body_edge_collision`: `SoftEdgePlane` (`Clone + Copy + Debug`).
//! - `crate::soft_body::soft_contact_tree`: `SoftContactTree` (`Clone + Debug + Default`).
//! - `crate::soft_body::soft_body_contacts`: `pub fn contact_thickness(&self) -> f32` and
//!   `pub(crate) fn build_self_exclusions(&mut self)` on `SoftBodyRecord` (Swift's computed
//!   property and mutating method).
//!
//! Swift `Float`/`Int32`/`UInt16`/`UInt32`/`UInt64`/`UInt8`/`Int` map to `f32`/`i32`/`u16`/`u32`/
//! `u64`/`u8`/`usize`; Swift's array indices become `usize` and its `Int32` public parameters stay
//! `i32` (converted at the array boundary exactly where Swift wrote `Int(vertex)`).
//!
//! Adaptations, all behaviour-preserving:
//! - Swift's `withUnsafeBufferPointer` views over `positions` (in `updateBounds`,
//!   `softBodyVolume`, `withSoftBodyVertexPositions`, `withSoftBodyLocalVertexPositions`) are plain
//!   slice borrows here, so this file needs no `unsafe` block.
//! - Swift's copy-on-write arrays: `previousPositions = positions` shares storage until a write, so
//!   the value clone below is the same state. `SoftBodyRecord(def)` starts from the property
//!   defaults (`SoftBodyRecord::default()`) and then assigns, exactly like Swift's `init(_ def:)`,
//!   which leaves the fields it never touches at their initializers (`averageEdgeLength`,
//!   `skinnedPositions`, `skinnedNormals`, and every bookkeeping array).
//! - Swift's `Int(softBodyId.index1) <= softBodies.count` etc. cast through `Int`; the Rust forms
//!   cast `i32` to `usize` after the `>= 1` test, which is the same value.
//! - Swift's `Float.rounded()` (ties away from zero) on the `Double` grid coordinates is
//!   `f64::round`, whose rounding mode is the same.
//! - Rust has no overloading, so the `[Transform]` convenience form of Swift's
//!   `setSoftBodySkinTransforms` is [`World::set_soft_body_skin_transforms_from_transforms`]
//!   (the `world_runtime::apply_buoyancy_in_aabb` precedent: the type goes into the name).
//! - Swift's throwing-closure `rethrows` on `withSoftBodyLocalVertexPositions` has no engine-side
//!   equivalent: the body is a plain `FnOnce`, and the Swift `throws` path was never used by the
//!   engine, only forwarded by callers.
//! - `SoftBodyRecord`, `SoftBodyCollider` and `SoftVertexPlane` are internal in Swift and are
//!   `pub(crate)` here, following `vehicle::vehicle_runtime::VehicleRecord`; their fields are `pub`
//!   because the sibling soft-body modules and the debug-draw pass read them directly.
//! - Swift's `static let planesPerVertex = 4` becomes the associated constant
//!   [`SoftBodyRecord::PLANES_PER_VERTEX`], referenced as `SoftBodyRecord::PLANES_PER_VERTEX` where
//!   Swift wrote `Self.planesPerVertex`.
//! - Swift's `[SIMD2<UInt32>]` for `meshEdges` becomes `Vec<[u32; 2]>`, the landed rendering of a
//!   Swift `SIMD2<Int32>` pair (`world_runtime::candidate_batch`). The sibling passes that fill or
//!   read `mesh_edges` map it to/from the settings' `edge.vertex` pair (`edge[0]`/`edge[1]`).

use std::collections::HashMap;
use std::ffi::c_void;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::math::functions::{
    cross, dot, is_valid_position, is_valid_quat, is_valid_vec3, length_squared, make_world_transform,
    max_float, max_int, min_vec3, mul_add, mul_transforms, normalize, normalize_quat, offset_aabb,
    offset_pos, rotate_vector, sub_pos, to_relative_transform, transform_point,
};
use crate::math::types::{AABB, Matrix3, Pos, Quat, Transform, Vec3, WorldTransform};
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_batches::{BendBatch, EdgeBatch, VolumeBatch};
use crate::soft_body::soft_body_edge_collision::SoftEdgePlane;
use crate::soft_body::soft_body_settings::{SoftBodyFace, SoftBodySettings};
use crate::soft_body::soft_contact_tree::SoftContactTree;
use crate::types::ids::{BodyID, ShapeID};
use crate::types::shape_def::Filter;

/// Soft body id references a soft body instance. This should be treated as an opaque handle.
///
/// Swift's zero-argument `public init()` is `SoftBodyID::default()`, which equals
/// [`SoftBodyID::NULL`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[repr(C)]
pub struct SoftBodyID {
    pub index1: i32,
    pub world0: u16,
    pub generation: u16,
}

impl SoftBodyID {
    /// Swift's `init(index1:world0:generation:)`.
    #[inline]
    pub fn new(index1: i32, world0: u16, generation: u16) -> Self {
        Self {
            index1,
            world0,
            generation,
        }
    }

    /// Swift's `public static let null`.
    pub const NULL: SoftBodyID = SoftBodyID {
        index1: 0,
        world0: 0,
        generation: 0,
    };

    /// Swift's `public var isNull: Bool`.
    #[inline]
    pub fn is_null(&self) -> bool {
        self.index1 == 0
    }
}

/// Used to create a soft body.
#[derive(Clone, Debug)]
pub struct SoftBodyDef {
    pub settings: SoftBodySettings,

    /// Placement of the settings' rest vertices.
    pub position: Pos,
    pub rotation: Quat,

    /// XPBD substeps per world step. More substeps make the body stiffer.
    pub iteration_count: i32,

    /// Linear velocity damping: dv/dt = -c * v.
    pub linear_damping: f32,

    /// Vertices never move faster than this.
    pub max_linear_velocity: f32,

    pub friction: f32,
    pub restitution: f32,

    /// n * R * T for closed surfaces (N m). Positive values inflate the body.
    pub pressure: f32,

    pub gravity_scale: f32,

    /// Collision radius of each vertex.
    pub vertex_radius: f32,

    /// Collision filter against rigid shapes.
    pub filter: Filter,

    /// Interaction with rigid bodies. With `two_way_coupling` off, rigid bodies push the soft body
    /// but do not feel it.
    pub collide_with_rigid_bodies: bool,
    pub two_way_coupling: bool,

    pub enable_sleep: bool,

    /// The soft body falls asleep when every vertex is slower than this for `time_to_sleep`.
    pub sleep_threshold: f32,

    /// Ray casts hit both sides of the faces.
    pub faces_double_sided: bool,

    /// Keep the body's own surfaces apart (cloth folding onto itself, a rope crossing itself).
    pub enable_self_collision: bool,

    /// Collide with other soft bodies.
    pub collide_with_soft_bodies: bool,

    /// Edge contacts: edges against rigid features that fit between two vertices (thin rods, blade
    /// edges), and edges against edges of soft bodies (crossing threads, narrow strips), on top of
    /// the vertex contacts.
    pub enable_edge_collision: bool,

    /// Distance kept between surfaces in self and soft body contact. Zero picks a quarter of the
    /// average edge length, or twice the vertex radius if larger.
    pub collision_thickness: f32,

    pub user_data: Option<*mut c_void>,
}

/// Swift's property initializers are non-zero, so `Default` cannot be derived.
impl Default for SoftBodyDef {
    #[inline]
    fn default() -> Self {
        Self {
            settings: SoftBodySettings::default(),
            position: Pos::default(),
            rotation: Quat::IDENTITY,
            iteration_count: 5,
            linear_damping: 0.1,
            max_linear_velocity: 500.0 * b3::length_units_per_meter(),
            friction: 0.2,
            restitution: 0.0,
            pressure: 0.0,
            gravity_scale: 1.0,
            vertex_radius: 0.0,
            filter: Filter::default_filter(),
            collide_with_rigid_bodies: true,
            two_way_coupling: true,
            enable_sleep: true,
            sleep_threshold: 0.03 * b3::length_units_per_meter(),
            faces_double_sided: false,
            enable_self_collision: false,
            collide_with_soft_bodies: true,
            enable_edge_collision: true,
            collision_thickness: 0.0,
            user_data: None,
        }
    }
}

impl SoftBodyDef {
    /// Swift's `public init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Swift's `public init(settings:position:rotation:)`. Swift defaulted `position` to `Pos()`
    /// and `rotation` to `.identity`; following the port's convention for defaulted Swift
    /// arguments, ported callers pass both explicitly.
    pub fn with_settings(settings: SoftBodySettings, position: Pos, rotation: Quat) -> Self {
        Self {
            settings,
            position,
            rotation,
            ..Self::default()
        }
    }
}

/// A rigid shape touched by a soft body during the last step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SoftBodyContact {
    pub shape_id: ShapeID,
    pub body_id: BodyID,
    /// Number of vertices touching the shape.
    pub vertex_count: i32,
}

impl SoftBodyContact {
    /// Swift's `public init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// Ray hit on a soft body surface.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SoftBodyRayResult {
    pub soft_body_id: SoftBodyID,
    pub point: Pos,
    pub normal: Vec3,
    pub fraction: f32,
    pub face_index: i32,
    pub hit: bool,
}

/// Swift's member defaults are not all zero (`fraction = 1`, `face_index = -1`), so `Default`
/// cannot be derived: the un-hit result starts the closest-hit search at fraction 1.0, and the
/// face-index tie-break needs `-1` to accept the first candidate.
impl Default for SoftBodyRayResult {
    #[inline]
    fn default() -> Self {
        Self {
            soft_body_id: SoftBodyID::default(),
            point: Pos::default(),
            normal: Vec3::default(),
            fraction: 1.0,
            face_index: -1,
            hit: false,
        }
    }
}

impl SoftBodyRayResult {
    /// Swift's `public init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// A rigid shape near the soft body, with the body's velocity tracked through the iterations so
/// later vertices see the impulses applied by earlier ones.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub(crate) struct SoftBodyCollider {
    pub shape_index: i32,
    pub body_index: i32,
    pub awake_index: i32,
    pub is_dynamic: bool,
    pub inv_mass: f32,
    pub inv_inertia: Matrix3,
    pub center: Pos,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub original_linear_velocity: Vec3,
    pub original_angular_velocity: Vec3,
    pub friction: f32,
    pub restitution: f32,
    pub updated: bool,
    pub contact_count: i32,
    /// Motion of the body since the start of the soft body step, from its tracked velocity, so
    /// edge planes follow a body the soft body pushes away.
    pub displacement: Vec3,
    pub rotation: Vec3,
}

/// Swift's `awakeIndex` default is `B3.nullIndex`, so `Default` cannot be derived.
impl Default for SoftBodyCollider {
    #[inline]
    fn default() -> Self {
        Self {
            shape_index: 0,
            body_index: 0,
            awake_index: b3::NULL_INDEX,
            is_dynamic: false,
            inv_mass: 0.0,
            inv_inertia: Matrix3::default(),
            center: Pos::default(),
            linear_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            original_linear_velocity: Vec3::default(),
            original_angular_velocity: Vec3::default(),
            friction: 0.0,
            restitution: 0.0,
            updated: false,
            contact_count: 0,
            displacement: Vec3::default(),
            rotation: Vec3::default(),
        }
    }
}

impl SoftBodyCollider {
    #[inline(always)]
    pub(crate) fn point_velocity(&self, r: Vec3) -> Vec3 {
        self.linear_velocity + cross(self.angular_velocity, r)
    }
}

/// A vertex can touch several independently oriented surfaces, for example a floor and a wall.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub(crate) struct SoftVertexPlane {
    pub normal: Vec3,
    pub offset: f32,
    pub separation: f32,
    pub collider: i32,
}

/// Swift's defaults are `Float.greatestFiniteMagnitude` for `separation` and `B3.nullIndex` for
/// `collider`, so `Default` cannot be derived.
impl Default for SoftVertexPlane {
    #[inline]
    fn default() -> Self {
        Self {
            normal: Vec3::default(),
            offset: 0.0,
            separation: f32::MAX,
            collider: b3::NULL_INDEX,
        }
    }
}

/// The particle state of one soft body. `WorldState`'s derived `Clone`/`Debug`/`Default` reach this
/// store, so the record is `Clone + Debug` and carries a hand-written `Default` (Swift's `init() {}`
/// property defaults); it is not `Copy` because it owns arrays and a contact history map.
#[derive(Clone, Debug)]
pub(crate) struct SoftBodyRecord {
    pub id: i32,
    pub generation: u16,
    pub user_data: Option<*mut c_void>,

    pub settings: SoftBodySettings,

    // Particle state, structure of arrays.
    pub origin: Pos,
    pub positions: Vec<Vec3>,
    pub previous_positions: Vec<Vec3>,
    pub velocities: Vec<Vec3>,
    pub inv_masses: Vec<f32>,

    // Collision state per vertex.
    pub plane_normals: Vec<Vec3>,
    pub plane_offsets: Vec<f32>,
    pub collider_indices: Vec<i32>,
    pub has_contact: Vec<bool>,
    pub colliders: Vec<SoftBodyCollider>,
    pub vertex_planes: Vec<SoftVertexPlane>,
    pub vertex_plane_counts: Vec<u8>,

    // Four wide constraint batches built from the settings.
    pub edge_batches: Vec<EdgeBatch>,
    pub bend_batches: Vec<BendBatch>,
    pub volume_batches: Vec<VolumeBatch>,

    // Skinning
    pub joint_transforms: Vec<Transform>,
    pub skinned_positions: Vec<Vec3>,
    pub skinned_normals: Vec<Vec3>,
    pub has_skin_pose: bool,

    // Tuning
    pub iteration_count: i32,
    pub linear_damping: f32,
    pub max_linear_velocity: f32,
    pub friction: f32,
    pub restitution: f32,
    pub pressure: f32,
    pub gravity_scale: f32,
    pub vertex_radius: f32,
    pub filter: Filter,
    pub collide_with_rigid_bodies: bool,
    pub two_way_coupling: bool,
    pub enable_sleep: bool,
    pub sleep_threshold: f32,
    pub faces_double_sided: bool,
    pub enable_self_collision: bool,
    pub collide_with_soft_bodies: bool,
    pub enable_edge_collision: bool,
    pub collision_thickness: f32,
    pub average_edge_length: f32,
    /// Edge neighbors of each vertex in compressed rows, for self contact.
    pub neighbor_starts: Vec<i32>,
    pub neighbors: Vec<u32>,
    /// Vertices referenced by faces, for soft contact.
    pub surface_vertices: Vec<i32>,
    pub closed_surface: bool,
    /// Surface edges, and the edges touching rigid features this step.
    pub mesh_edges: Vec<[u32; 2]>,
    /// Self contact exclusions in compressed rows, for `self_exclusion_distance`.
    pub excluded_face_starts: Vec<i32>,
    pub excluded_faces: Vec<i32>,
    pub excluded_edge_starts: Vec<i32>,
    pub excluded_edges: Vec<i32>,
    pub contact_face_tree: SoftContactTree,
    pub contact_edge_tree: SoftContactTree,
    pub self_exclusion_distance: f32,
    /// Contact history: the side of each vertex-face pair this body touched last step, keyed by
    /// `soft_contact_key`.
    pub face_sides: HashMap<u64, f32>,
    pub edge_contacts: Vec<SoftEdgePlane>,

    // State
    pub awake: bool,
    pub sleep_time: f32,
    pub bounds: AABB,
    pub spatial_revision: u64,
    pub volume: f32,
}

/// Swift's `init() {}` is the property-initializer set, which is not all zeroes (`id` is
/// `B3.nullIndex`, the tuning fields carry the definition's defaults, and the body starts awake), so
/// `Default` cannot be derived. `World::destroy_soft_body` writes this back into a freed slot.
impl Default for SoftBodyRecord {
    #[inline]
    fn default() -> Self {
        Self {
            id: b3::NULL_INDEX,
            generation: 0,
            user_data: None,

            settings: SoftBodySettings::default(),

            origin: Pos::default(),
            positions: Vec::new(),
            previous_positions: Vec::new(),
            velocities: Vec::new(),
            inv_masses: Vec::new(),

            plane_normals: Vec::new(),
            plane_offsets: Vec::new(),
            collider_indices: Vec::new(),
            has_contact: Vec::new(),
            colliders: Vec::new(),
            vertex_planes: Vec::new(),
            vertex_plane_counts: Vec::new(),

            edge_batches: Vec::new(),
            bend_batches: Vec::new(),
            volume_batches: Vec::new(),

            joint_transforms: Vec::new(),
            skinned_positions: Vec::new(),
            skinned_normals: Vec::new(),
            has_skin_pose: false,

            iteration_count: 5,
            linear_damping: 0.1,
            max_linear_velocity: 500.0,
            friction: 0.2,
            restitution: 0.0,
            pressure: 0.0,
            gravity_scale: 1.0,
            vertex_radius: 0.0,
            filter: Filter::default_filter(),
            collide_with_rigid_bodies: true,
            two_way_coupling: true,
            enable_sleep: true,
            sleep_threshold: 0.03,
            faces_double_sided: false,
            enable_self_collision: false,
            collide_with_soft_bodies: true,
            enable_edge_collision: true,
            collision_thickness: 0.0,
            average_edge_length: 0.0,
            neighbor_starts: Vec::new(),
            neighbors: Vec::new(),
            surface_vertices: Vec::new(),
            closed_surface: false,
            mesh_edges: Vec::new(),
            excluded_face_starts: Vec::new(),
            excluded_faces: Vec::new(),
            excluded_edge_starts: Vec::new(),
            excluded_edges: Vec::new(),
            contact_face_tree: SoftContactTree::default(),
            contact_edge_tree: SoftContactTree::default(),
            self_exclusion_distance: 0.0,
            face_sides: HashMap::new(),
            edge_contacts: Vec::new(),

            awake: true,
            sleep_time: 0.0,
            bounds: AABB::default(),
            spatial_revision: 0,
            volume: 0.0,
        }
    }
}

impl SoftBodyRecord {
    /// Swift's `static let planesPerVertex = 4`, written `Self.planesPerVertex` inside the
    /// soft-body modules.
    pub const PLANES_PER_VERTEX: usize = 4;

    /// Swift's `init(_ def: SoftBodyDef)`.
    pub(crate) fn new(def: &SoftBodyDef) -> Self {
        let mut record = SoftBodyRecord::default();

        let mut settings = def.settings.clone();
        if settings.patches.is_empty() {
            settings.build_patches();
        }
        record.settings = settings;
        record.user_data = def.user_data;

        // A shared grid frame avoids repeated rebasing of nearby cloth pieces. Power-of-two
        // cells keep the origin exact and bound local vertex magnitudes during travel.
        let cell = 1024.0;
        let origin = Pos::new(
            (def.position.x / cell).round() * cell,
            (def.position.y / cell).round() * cell,
            (def.position.z / cell).round() * cell,
        );
        let transform = Transform::new(
            sub_pos(def.position, origin),
            normalize_quat(def.rotation),
        );
        let count = record.settings.vertices.len();
        record.origin = origin;
        record.positions = record
            .settings
            .vertices
            .iter()
            .map(|v| transform_point(transform, v.position))
            .collect();
        record.previous_positions = record.positions.clone();
        record.velocities = record
            .settings
            .vertices
            .iter()
            .map(|v| rotate_vector(transform.q, v.velocity))
            .collect();
        record.inv_masses = record.settings.vertices.iter().map(|v| v.inv_mass).collect();
        record.plane_normals = vec![Vec3::default(); count];
        record.plane_offsets = vec![0.0; count];
        record.collider_indices = vec![b3::NULL_INDEX; count];
        record.vertex_planes = vec![
            SoftVertexPlane::default();
            count * SoftBodyRecord::PLANES_PER_VERTEX
        ];
        record.vertex_plane_counts = vec![0u8; count];
        record.has_contact = vec![false; count];
        if !record.settings.skinned.is_empty() {
            record.skinned_positions = vec![Vec3::default(); record.settings.skinned.len()];
            record.skinned_normals =
                vec![Vec3::new(0.0, 1.0, 0.0); record.settings.skinned.len()];
        }

        record.iteration_count = max_int(1, def.iteration_count);
        record.linear_damping = max_float(0.0, def.linear_damping);
        record.max_linear_velocity = def.max_linear_velocity;
        record.friction = def.friction;
        record.restitution = def.restitution;
        record.pressure = def.pressure;
        record.gravity_scale = def.gravity_scale;
        record.vertex_radius = max_float(0.0, def.vertex_radius);
        record.filter = def.filter;
        record.collide_with_rigid_bodies = def.collide_with_rigid_bodies;
        record.two_way_coupling = def.two_way_coupling;
        record.enable_sleep = def.enable_sleep;
        record.sleep_threshold = def.sleep_threshold;
        record.faces_double_sided = def.faces_double_sided;
        record.enable_self_collision = def.enable_self_collision;
        record.collide_with_soft_bodies = def.collide_with_soft_bodies;
        record.enable_edge_collision = def.enable_edge_collision;
        record.collision_thickness = max_float(0.0, def.collision_thickness);
        if !record.settings.edges.is_empty() {
            let sum: f32 = record
                .settings
                .edges
                .iter()
                .fold(0.0, |acc, e| acc + e.rest_length);
            record.average_edge_length = sum / record.settings.edges.len() as f32;
        }
        record.build_batches();
        record.update_bounds();
        record
    }

    pub fn update_bounds(&mut self) {
        self.spatial_revision = self.spatial_revision.wrapping_add(1);
        let Some(first) = self.positions.first() else {
            self.bounds = AABB::default();
            return;
        };
        let mut lower = *first;
        let mut upper = *first;
        for i in 1..self.positions.len() {
            lower = min_vec3(lower, self.positions[i]);
            upper = max_vec3(upper, self.positions[i]);
        }
        let r = Vec3::new(self.vertex_radius, self.vertex_radius, self.vertex_radius);
        self.bounds = AABB::new(lower - r, upper + r);
    }

    #[inline]
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }
}

impl World {
    // MARK: Creation

    /// Create a soft body. Returns a null id when the settings are invalid.
    pub fn create_soft_body(&mut self, def: &SoftBodyDef) -> SoftBodyID {
        b3_assert(!self.locked);
        if !(def.settings.validate().is_none()
            && is_valid_position(def.position)
            && is_valid_quat(def.rotation))
        {
            return SoftBodyID::default();
        }

        let mut record = SoftBodyRecord::new(def);
        // Self contact tables are built here rather than on the first step.
        if record.enable_self_collision {
            record.build_self_exclusions();
        }
        let id = self.soft_body_id_pool.alloc();
        if id as usize == self.soft_bodies.len() {
            self.soft_bodies.push(record);
        } else {
            record.generation = self.soft_bodies[id as usize].generation.wrapping_add(1);
            self.soft_bodies[id as usize] = record;
        }
        self.soft_bodies[id as usize].id = id;
        SoftBodyID::new(
            id + 1,
            self.world_id,
            self.soft_bodies[id as usize].generation,
        )
    }

    pub fn destroy_soft_body(&mut self, soft_body_id: SoftBodyID) {
        let Some(index) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_body_ray_caches.remove(&(index as i32));
        let generation = self.soft_bodies[index].generation;
        self.soft_bodies[index] = SoftBodyRecord::default();
        self.soft_bodies[index].generation = generation;
        self.soft_body_id_pool.free(index as i32);
    }

    pub fn soft_body_is_valid(&self, soft_body_id: SoftBodyID) -> bool {
        if !(soft_body_id.world0 == self.world_id
            && soft_body_id.index1 >= 1
            && (soft_body_id.index1 as usize) <= self.soft_bodies.len())
        {
            return false;
        }
        let record = &self.soft_bodies[(soft_body_id.index1 - 1) as usize];
        record.id != b3::NULL_INDEX && record.generation == soft_body_id.generation
    }

    pub fn soft_body_count(&self) -> i32 {
        self.soft_body_id_pool.id_count()
    }

    /// Swift's `internal` `softBodyIndex`; `pub(crate)` mirrors Swift's target-wide internal
    /// visibility so the sibling soft-body modules can reach it. `soft_body_queries.rs` already
    /// calls it as `World::soft_body_index(SoftBodyID) -> Option<usize>`.
    #[inline(always)]
    pub(crate) fn soft_body_index(&self, soft_body_id: SoftBodyID) -> Option<usize> {
        if self.soft_body_is_valid(soft_body_id) {
            Some((soft_body_id.index1 - 1) as usize)
        } else {
            None
        }
    }

    // MARK: State

    pub fn soft_body_vertex_count(&self, soft_body_id: SoftBodyID) -> i32 {
        self.soft_body_index(soft_body_id)
            .map(|i| self.soft_bodies[i].vertex_count() as i32)
            .unwrap_or(0)
    }

    /// Current vertex positions, in the order of the (optimized) settings.
    pub fn soft_body_vertex_positions(&self, soft_body_id: SoftBodyID) -> Vec<Pos> {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return Vec::new();
        };
        let record = &self.soft_bodies[i];
        record
            .positions
            .iter()
            .map(|p| offset_pos(record.origin, *p))
            .collect()
    }

    /// Materialize double precision world vertices for the callback. For allocation-free rendering,
    /// use [`World::with_soft_body_local_vertex_positions`].
    ///
    /// Swift handed the callback an `UnsafeBufferPointer<Pos>`; the Rust view is a `&[Pos]` over
    /// the same freshly materialized buffer, in the same order.
    pub fn with_soft_body_vertex_positions<R, F>(
        &self,
        soft_body_id: SoftBodyID,
        body: F,
    ) -> Option<R>
    where
        F: FnOnce(&[Pos]) -> R,
    {
        let index = self.soft_body_index(soft_body_id)?;
        let record = &self.soft_bodies[index];
        let world_positions: Vec<Pos> = record
            .positions
            .iter()
            .map(|p| offset_pos(record.origin, *p))
            .collect();
        Some(body(world_positions.as_slice()))
    }

    pub fn soft_body_vertex_velocities(&self, soft_body_id: SoftBodyID) -> Vec<Vec3> {
        self.soft_body_index(soft_body_id)
            .map(|i| self.soft_bodies[i].velocities.clone())
            .unwrap_or_default()
    }

    pub fn soft_body_vertex_position(&self, soft_body_id: SoftBodyID, vertex: i32) -> Option<Pos> {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return None;
        };
        if !(vertex >= 0 && (vertex as usize) < self.soft_bodies[i].vertex_count()) {
            return None;
        }
        let record = &self.soft_bodies[i];
        Some(offset_pos(record.origin, record.positions[vertex as usize]))
    }

    /// Move a vertex. Kinematic vertices (inverse mass zero) are animated this way.
    pub fn set_soft_body_vertex_position(
        &mut self,
        soft_body_id: SoftBodyID,
        vertex: i32,
        position: Pos,
    ) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        if !(vertex >= 0
            && (vertex as usize) < self.soft_bodies[i].vertex_count()
            && is_valid_position(position))
        {
            return;
        }
        let v = vertex as usize;
        // Swift wrote `subPos(position, origin)` twice, once per array; the expression is pure, so
        // one evaluation feeds both stores with the same bits.
        let local = sub_pos(position, self.soft_bodies[i].origin);
        self.soft_bodies[i].positions[v] = local;
        self.soft_bodies[i].previous_positions[v] = local;
        self.soft_bodies[i].update_bounds();
        self.wake_soft_body(i);
    }

    pub fn set_soft_body_vertex_velocity(
        &mut self,
        soft_body_id: SoftBodyID,
        vertex: i32,
        velocity: Vec3,
    ) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        if !(vertex >= 0
            && (vertex as usize) < self.soft_bodies[i].vertex_count()
            && is_valid_vec3(velocity))
        {
            return;
        }
        self.soft_bodies[i].velocities[vertex as usize] = velocity;
        self.wake_soft_body(i);
    }

    /// Pin (zero) or release a vertex.
    pub fn set_soft_body_vertex_inverse_mass(
        &mut self,
        soft_body_id: SoftBodyID,
        vertex: i32,
        inv_mass: f32,
    ) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        if !(vertex >= 0
            && (vertex as usize) < self.soft_bodies[i].vertex_count()
            && inv_mass >= 0.0)
        {
            return;
        }
        self.soft_bodies[i].inv_masses[vertex as usize] = inv_mass;
        self.wake_soft_body(i);
    }

    /// Add a velocity to every dynamic vertex.
    pub fn add_soft_body_velocity(&mut self, soft_body_id: SoftBodyID, velocity: Vec3) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        if !is_valid_vec3(velocity) {
            return;
        }
        let record = &mut self.soft_bodies[i];
        for v in 0..record.velocities.len() {
            if record.inv_masses[v] > 0.0 {
                record.velocities[v] += velocity;
            }
        }
        self.wake_soft_body(i);
    }

    /// Apply a linear impulse distributed by mass, so the change of momentum equals `impulse`.
    pub fn apply_soft_body_linear_impulse(&mut self, soft_body_id: SoftBodyID, impulse: Vec3) {
        if !(self.soft_body_is_valid(soft_body_id) && is_valid_vec3(impulse)) {
            return;
        }
        let mass = self.soft_body_mass(soft_body_id);
        if !(mass > 0.0) {
            return;
        }
        self.add_soft_body_velocity(soft_body_id, (1.0 / mass) * impulse);
    }

    /// Teleport the whole soft body by a rigid transform about its center of mass.
    pub fn set_soft_body_transform(
        &mut self,
        soft_body_id: SoftBodyID,
        position: Pos,
        rotation: Quat,
    ) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        if !(is_valid_position(position) && is_valid_quat(rotation)) {
            return;
        }
        let center = self.soft_body_center_of_mass(soft_body_id);
        let q = normalize_quat(rotation);
        let record = &mut self.soft_bodies[i];
        let local_center = sub_pos(center, record.origin);
        record.origin = position;
        for v in 0..record.positions.len() {
            let p = rotate_vector(q, record.positions[v] - local_center);
            record.positions[v] = p;
            record.previous_positions[v] = p;
            record.velocities[v] = rotate_vector(q, record.velocities[v]);
        }
        record.update_bounds();
        drop(record);
        self.wake_soft_body(i);
    }

    /// Total mass of the dynamic vertices.
    pub fn soft_body_mass(&self, soft_body_id: SoftBodyID) -> f32 {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return 0.0;
        };
        self.soft_bodies[i]
            .inv_masses
            .iter()
            .fold(0.0, |acc, m| acc + if *m > 0.0 { 1.0 / *m } else { 0.0 })
    }

    /// Mass weighted center of the dynamic vertices, or the vertex average when all are kinematic.
    pub fn soft_body_center_of_mass(&self, soft_body_id: SoftBodyID) -> Pos {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return Pos::default();
        };
        let record = &self.soft_bodies[i];
        let mut sum = Vec3::default();
        let mut mass: f32 = 0.0;
        for v in 0..record.positions.len() {
            if record.inv_masses[v] > 0.0 {
                let m = 1.0 / record.inv_masses[v];
                sum = mul_add(sum, m, record.positions[v]);
                mass += m;
            }
        }
        if mass > 0.0 {
            return offset_pos(record.origin, (1.0 / mass) * sum);
        }
        for p in &record.positions {
            sum += *p;
        }
        offset_pos(
            record.origin,
            if record.positions.is_empty() {
                Vec3::default()
            } else {
                (1.0 / record.positions.len() as f32) * sum
            },
        )
    }

    pub fn soft_body_linear_velocity(&self, soft_body_id: SoftBodyID) -> Vec3 {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return Vec3::default();
        };
        let record = &self.soft_bodies[i];
        let mut sum = Vec3::default();
        let mut mass: f32 = 0.0;
        for v in 0..record.velocities.len() {
            if record.inv_masses[v] > 0.0 {
                let m = 1.0 / record.inv_masses[v];
                sum = mul_add(sum, m, record.velocities[v]);
                mass += m;
            }
        }
        if mass > 0.0 {
            (1.0 / mass) * sum
        } else {
            Vec3::default()
        }
    }

    pub fn soft_body_bounds(&self, soft_body_id: SoftBodyID) -> AABB {
        self.soft_body_index(soft_body_id)
            .map(|i| {
                let record = &self.soft_bodies[i];
                offset_aabb(record.bounds, record.origin)
            })
            .unwrap_or_default()
    }

    /// Enclosed volume of a closed surface after the last step.
    pub fn soft_body_volume(&self, soft_body_id: SoftBodyID) -> f32 {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return 0.0;
        };
        let record = &self.soft_bodies[i];
        closed_volume(&record.settings.faces, &record.positions)
    }

    pub fn soft_body_settings(&self, soft_body_id: SoftBodyID) -> Option<SoftBodySettings> {
        self.soft_body_index(soft_body_id)
            .map(|i| self.soft_bodies[i].settings.clone())
    }

    /// Rigid shapes touched during the last step.
    pub fn soft_body_contacts(&self, soft_body_id: SoftBodyID) -> Vec<SoftBodyContact> {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return Vec::new();
        };
        let mut contacts: Vec<SoftBodyContact> = Vec::new();
        for collider in &self.soft_bodies[i].colliders {
            if !(collider.contact_count > 0) {
                continue;
            }
            let shape = &self.shapes[collider.shape_index as usize];
            let mut contact = SoftBodyContact::default();
            contact.shape_id = ShapeID::new(
                shape.id + 1,
                self.world_id,
                shape.generation,
            );
            contact.body_id = self.make_body_id(collider.body_index);
            contact.vertex_count = collider.contact_count;
            contacts.push(contact);
        }
        contacts
    }

    /// True when vertex `vertex` touched a rigid shape in the last step.
    pub fn soft_body_vertex_has_contact(&self, soft_body_id: SoftBodyID, vertex: i32) -> bool {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return false;
        };
        if !(vertex >= 0 && (vertex as usize) < self.soft_bodies[i].vertex_count()) {
            return false;
        }
        self.soft_bodies[i].has_contact[vertex as usize]
    }

    pub fn soft_body_user_data(&self, soft_body_id: SoftBodyID) -> Option<*mut c_void> {
        self.soft_body_index(soft_body_id)
            .and_then(|i| self.soft_bodies[i].user_data)
    }

    // MARK: Skinning

    /// Pose of the skeleton driving skinned vertices, one world transform per joint. Kinematic
    /// skinned vertices follow the skin; dynamic ones are constrained near it.
    pub fn set_soft_body_skin_transforms(
        &mut self,
        soft_body_id: SoftBodyID,
        transforms: &[WorldTransform],
        teleport: bool,
    ) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        if !(transforms.len() >= self.soft_bodies[i].settings.inverse_bind_matrices.len()) {
            return;
        }
        let origin = self.soft_bodies[i].origin;
        let record = &mut self.soft_bodies[i];
        record.joint_transforms = transforms
            .iter()
            .map(|t| to_relative_transform(*t, origin))
            .collect();
        record.has_skin_pose = true;
        record.update_skin();
        if teleport {
            // Swift enumerated `settings.skinned` while writing the particle arrays; the only value
            // read here is the vertex index, so the indices are hoisted (identical values, same
            // order) and the settings stay borrowed.
            let skinned_vertices: Vec<u32> =
                record.settings.skinned.iter().map(|s| s.vertex).collect();
            for (k, vertex) in skinned_vertices.iter().enumerate() {
                let v = *vertex as usize;
                record.positions[v] = record.skinned_positions[k];
                record.previous_positions[v] = record.skinned_positions[k];
                record.velocities[v] = Vec3::default();
            }
            record.update_bounds();
        }
        drop(record);
        self.wake_soft_body(i);
    }

    // MARK: Tuning

    pub fn set_soft_body_iteration_count(&mut self, soft_body_id: SoftBodyID, count: i32) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].iteration_count = max_int(1, count);
    }

    pub fn set_soft_body_pressure(&mut self, soft_body_id: SoftBodyID, pressure: f32) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].pressure = pressure;
        self.wake_soft_body(i);
    }

    pub fn soft_body_pressure(&self, soft_body_id: SoftBodyID) -> f32 {
        self.soft_body_index(soft_body_id)
            .map(|i| self.soft_bodies[i].pressure)
            .unwrap_or(0.0)
    }

    pub fn set_soft_body_friction(&mut self, soft_body_id: SoftBodyID, friction: f32) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].friction = max_float(0.0, friction);
    }

    pub fn set_soft_body_restitution(&mut self, soft_body_id: SoftBodyID, restitution: f32) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].restitution = max_float(0.0, restitution);
    }

    pub fn set_soft_body_linear_damping(&mut self, soft_body_id: SoftBodyID, damping: f32) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].linear_damping = max_float(0.0, damping);
    }

    pub fn set_soft_body_self_collision(&mut self, soft_body_id: SoftBodyID, enabled: bool) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].enable_self_collision = enabled;
        if enabled {
            self.soft_bodies[i].build_self_exclusions();
        }
        self.wake_soft_body(i);
    }

    pub fn set_soft_body_collide_with_soft_bodies(
        &mut self,
        soft_body_id: SoftBodyID,
        enabled: bool,
    ) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].collide_with_soft_bodies = enabled;
        self.wake_soft_body(i);
    }

    pub fn set_soft_body_edge_collision(&mut self, soft_body_id: SoftBodyID, enabled: bool) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].enable_edge_collision = enabled;
        self.wake_soft_body(i);
    }

    /// Zero restores the automatic thickness.
    pub fn set_soft_body_collision_thickness(&mut self, soft_body_id: SoftBodyID, thickness: f32) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].collision_thickness = max_float(0.0, thickness);
        if self.soft_bodies[i].enable_self_collision {
            self.soft_bodies[i].build_self_exclusions();
        }
        self.wake_soft_body(i);
    }

    /// Thickness used for self and soft body contact.
    pub fn soft_body_collision_thickness(&self, soft_body_id: SoftBodyID) -> f32 {
        self.soft_body_index(soft_body_id)
            .map(|i| self.soft_bodies[i].contact_thickness())
            .unwrap_or(0.0)
    }

    pub fn set_soft_body_gravity_scale(&mut self, soft_body_id: SoftBodyID, scale: f32) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].gravity_scale = scale;
        self.wake_soft_body(i);
    }

    pub fn set_soft_body_filter(&mut self, soft_body_id: SoftBodyID, filter: Filter) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        self.soft_bodies[i].filter = filter;
    }

    // MARK: Sleep

    pub fn soft_body_is_awake(&self, soft_body_id: SoftBodyID) -> bool {
        self.soft_body_index(soft_body_id)
            .map(|i| self.soft_bodies[i].awake)
            .unwrap_or(false)
    }

    pub fn set_soft_body_awake(&mut self, soft_body_id: SoftBodyID, awake: bool) {
        let Some(i) = self.soft_body_index(soft_body_id) else {
            return;
        };
        if awake {
            self.wake_soft_body(i);
        } else {
            self.soft_bodies[i].awake = false;
            for v in 0..self.soft_bodies[i].velocities.len() {
                self.soft_bodies[i].velocities[v] = Vec3::default();
            }
        }
    }

    pub(crate) fn wake_soft_body(&mut self, index: usize) {
        self.soft_bodies[index].awake = true;
        self.soft_bodies[index].sleep_time = 0.0;
    }
}

// MARK: - Skinning

impl SoftBodyRecord {
    /// Linear blend skinning of the skinned vertices and their normals.
    pub(crate) fn update_skin(&mut self) {
        if !(self.has_skin_pose && !self.settings.skinned.is_empty()) {
            return;
        }
        let binds = &self.settings.inverse_bind_matrices;
        let joints = &self.joint_transforms;
        if !(joints.len() >= binds.len()) {
            return;
        }

        for k in 0..self.settings.skinned.len() {
            let s = &self.settings.skinned[k];
            let rest = self.settings.vertices[s.vertex as usize].position;
            let mut p = Vec3::default();
            let mut weight_sum: f32 = 0.0;
            for j in 0..4usize {
                if !(s.weights[j] > 0.0) {
                    continue;
                }
                let joint = s.joints[j] as usize;
                let skin = mul_transforms(joints[joint], binds[joint]);
                p = mul_add(p, s.weights[j], transform_point(skin, rest));
                weight_sum += s.weights[j];
            }
            self.skinned_positions[k] = if weight_sum > 0.0 {
                (1.0 / weight_sum) * p
            } else {
                rest
            };
        }

        // Skinned normals from the skinned neighbourhood. Unskinned neighbours use their position.
        let mut skinned_of_vertex = vec![-1i32; self.positions.len()];
        for k in 0..self.settings.skinned.len() {
            skinned_of_vertex[self.settings.skinned[k].vertex as usize] = k as i32;
        }
        // The closure reads two fields that no later write touches, so it captures them by
        // reference (Swift's nested `skinnedPosition(_:)` function).
        let skinned_position = |v: u32| -> Vec3 {
            let k = skinned_of_vertex[v as usize];
            if k >= 0 {
                self.skinned_positions[k as usize]
            } else {
                self.positions[v as usize]
            }
        };
        for k in 0..self.settings.skinned.len() {
            let s = &self.settings.skinned[k];
            let mut n = Vec3::default();
            for f in &s.normal_faces {
                let face = &self.settings.faces[*f as usize];
                let a = skinned_position(face.vertex.x);
                let b = skinned_position(face.vertex.y);
                let c = skinned_position(face.vertex.z);
                n += cross(b - a, c - a);
            }
            self.skinned_normals[k] = if length_squared(n) > 0.0 {
                normalize(n)
            } else {
                Vec3::new(0.0, 1.0, 0.0)
            };
        }
    }
}

/// Volume enclosed by a closed, outward wound triangle surface.
///
/// Swift read the positions through an `UnsafeBufferPointer`; the Rust view is the same slice.
pub(crate) fn closed_volume(faces: &[SoftBodyFace], positions: &[Vec3]) -> f32 {
    let mut six_volume: f32 = 0.0;
    for face in faces {
        let a = positions[face.vertex.x as usize];
        let b = positions[face.vertex.y as usize];
        let c = positions[face.vertex.z as usize];
        six_volume += dot(a, cross(b, c));
    }
    six_volume / 6
}

impl SoftBodyRecord {
    /// Contact groups share a local frame; translation never passes through an absolute float.
    pub fn rebase(&mut self, reference: Pos) {
        if reference == self.origin {
            return;
        }
        let delta = sub_pos(self.origin, reference);
        for i in 0..self.positions.len() {
            self.positions[i] += delta;
            self.previous_positions[i] += delta;
        }
        for i in 0..self.skinned_positions.len() {
            self.skinned_positions[i] += delta;
        }
        for i in 0..self.joint_transforms.len() {
            self.joint_transforms[i].p += delta;
        }
        self.origin = reference;
        self.update_bounds();
    }
}

impl World {
    /// Read vertex offsets without allocation. Add `origin` in double precision when rendering.
    ///
    /// Swift handed the callback an `UnsafeBufferPointer<Vec3>` and `rethrows`; the Rust view is a
    /// `&[Vec3]` over the record's own storage, so nothing is copied.
    pub fn with_soft_body_local_vertex_positions<R, F>(
        &self,
        id: SoftBodyID,
        body: F,
    ) -> Option<R>
    where
        F: FnOnce(Pos, &[Vec3]) -> R,
    {
        let Some(i) = self.soft_body_index(id) else {
            return None;
        };
        let record = &self.soft_bodies[i];
        Some(body(record.origin, record.positions.as_slice()))
    }

    /// Convenience for existing single precision world poses.
    ///
    /// Swift overloaded this as `setSoftBodySkinTransforms(_:teleport:)`; Rust has no overloading,
    /// so the `[Transform]` form carries the type in its name and maps through
    /// `make_world_transform` exactly as Swift did.
    pub fn set_soft_body_skin_transforms_from_transforms(
        &mut self,
        id: SoftBodyID,
        transforms: &[Transform],
        teleport: bool,
    ) {
        let world_transforms: Vec<WorldTransform> =
            transforms.iter().map(|t| make_world_transform(*t)).collect();
        self.set_soft_body_skin_transforms(id, world_transforms.as_slice(), teleport);
    }
}
