//! Shape creation and proxy management.
//!
//! Translated from `Sources/Natsel/Simulation/ShapeCreation.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::sensor`: `World::create_sensor(shape_id)` /
//!   `World::destroy_sensor(shape_id)`, both `&mut self` over `i32` shape indices.
//! - `crate::simulation::contact_creation`: `World::destroy_contact(contact_id, wake_bodies)`.
//! - `crate::simulation::body_api`: `World::update_body_mass_data(body_index)` and
//!   `World::apply_mass_from_shapes(body_id)`.

use std::collections::HashMap;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::compound::compound::Compound;
use crate::collision::height_field::quantization::HeightField;
use crate::collision::hull::Hull;
use crate::collision::hull_builder::clone_and_transform_hull;
use crate::collision::mesh::storage::TriangleMesh;
use crate::math::functions::{
    aabb_inflate, distance, distance_squared, is_valid_float, max_float, min_float, safe_scale,
};
use crate::math::types::{AABB, Transform, Vec3, WorldTransform};
use crate::simulation::broad_phase::proxy_type_of;
use crate::simulation::shape_runtime::{
    Shape, ShapeFlags, ShapeGeometry, compute_fat_shape_aabb, destroy_shape_allocations,
    destroy_shape_proxy, get_shape_centroid,
};
use crate::simulation::world_runtime::{BodyFlags, SetType, World};
use crate::types::body_def::BodyType;
use crate::types::geometry::{Capsule, Mesh, Sphere};
use crate::types::ids::{BodyID, ShapeID};
use crate::types::shape_def::{ShapeDef, ShapeType};

/// Content-keyed, reference counted hull store. Hulls with identical bytes share one instance.
///
/// Swift's `final class HullDatabase`; only `World` owns it, so it becomes a plain owned struct.
/// The Swift class identity of a stored hull (used by `remove` and by the identity fast path of the
/// byte comparison) is carried by the geometry allocation: a `Hull` is a view onto a uniquely owned
/// zeroed slab, so two `Hull` values with the same storage base address are the same Swift instance.
#[derive(Clone, Debug, Default)]
pub struct HullDatabase {
    entries: HashMap<u64, Vec<HullDatabaseEntry>>,
}

/// One interned hull and the number of shapes holding it.
///
/// Swift's `private struct HullDatabase.Entry`; it is crate-visible here because the world-state
/// snapshot copies the whole entry table (`HullDatabaseSnapshot`).
#[derive(Clone, Debug)]
pub struct HullDatabaseEntry {
    pub hull: Hull,
    /// Swift `Int`; a reference count never needs more than the machine word.
    pub ref_count: i64,
}

/// Opaque copy of the reference counts, for world state snapshots. Hulls are shared, not copied.
///
/// Swift's snapshot was the `Any`-erased `[UInt64: [Entry]]` dictionary, and `restore` only applied
/// it when the dynamic type matched; the typed version keeps the same accept-or-ignore behaviour by
/// taking the snapshot value it was produced from.
#[derive(Clone, Debug, Default)]
pub struct HullDatabaseSnapshot {
    entries: HashMap<u64, Vec<HullDatabaseEntry>>,
}

/// Swift's `hull1 === hull2` for two live hulls: the same geometry allocation at the same offset.
#[inline(always)]
fn is_same_hull(a: &Hull, b: &Hull) -> bool {
    a.storage().base() == b.storage().base()
}

/// Compare two hulls by their serialized bytes, exactly as Swift's global `compareHullData` did.
///
/// The function has no Rust counterpart in `collision::hull.rs` (a recorded gap for the build
/// phase), so the byte-exact test lives here next to its only user. The identity test is the same
/// fast path Swift took, and the tail is a byte comparison over `byteCount` bytes of the zeroed,
/// fully serialized allocation, so serialized padding can never make two equal hulls differ.
pub(crate) fn compare_hull_data(hull1: &Hull, hull2: &Hull) -> bool {
    if is_same_hull(hull1, hull2) {
        return true;
    }

    if hull1.byte_count() != hull2.byte_count() {
        return false;
    }

    let byte_count = hull1.byte_count() as usize;

    // SAFETY: both bases address live, fully initialized geometry allocations of exactly
    // `byte_count` bytes (equal on the branch above, and each allocation was zeroed at creation and
    // never grown), so reading that many bytes stays inside each allocation.
    let bytes1 = unsafe { std::slice::from_raw_parts(hull1.storage().base(), byte_count) };
    // SAFETY: as above for the second hull's allocation.
    let bytes2 = unsafe { std::slice::from_raw_parts(hull2.storage().base(), byte_count) };

    // `==` on byte slices is the same content test as `memcmp(...) == 0`.
    bytes1 == bytes2
}

impl HullDatabase {
    /// Add a hull, cloning it on a miss. Returns the owned shared hull.
    pub fn add(&mut self, src: &Hull) -> Option<Hull> {
        let key = src.hash();
        if let Some(bucket) = self.entries.get_mut(&key) {
            for i in 0..bucket.len() {
                if compare_hull_data(&bucket[i].hull, src) {
                    bucket[i].ref_count += 1;
                    return Some(bucket[i].hull.clone());
                }
            }
        }

        // Swift's `Hull.clone()` also refuses an unbuilt hull (`version != B3.hullVersion`), so the
        // same guard is applied here: `deep_clone` alone does not test it.
        if src.version() != b3::HULL_VERSION {
            return None;
        }
        let clone = src.deep_clone()?;

        self.entries
            .entry(key)
            .or_default()
            .push(HullDatabaseEntry {
                hull: clone.clone(),
                ref_count: 1,
            });
        Some(clone)
    }

    /// Like `add` but takes ownership of a heap hull: inserted directly on a miss.
    pub fn add_owned(&mut self, owned: Hull) -> Hull {
        let key = owned.hash();
        if let Some(bucket) = self.entries.get_mut(&key) {
            for i in 0..bucket.len() {
                if compare_hull_data(&bucket[i].hull, &owned) {
                    bucket[i].ref_count += 1;
                    return bucket[i].hull.clone();
                }
            }
        }

        self.entries
            .entry(key)
            .or_default()
            .push(HullDatabaseEntry {
                hull: owned.clone(),
                ref_count: 1,
            });
        owned
    }

    /// Release a reference. The owned copy is freed when the count reaches zero.
    pub fn remove(&mut self, hull: Hull) {
        let key = hull.hash();
        let Some(bucket) = self.entries.get_mut(&key) else {
            return;
        };

        let mut released = false;
        for i in 0..bucket.len() {
            if is_same_hull(&bucket[i].hull, &hull) {
                bucket[i].ref_count -= 1;
                if bucket[i].ref_count <= 0 {
                    bucket.remove(i);
                }
                released = true;
                break;
            }
        }

        if released {
            if bucket.is_empty() {
                self.entries.remove(&key);
            }
        }
    }

    pub fn count(&self) -> usize {
        self.entries.values().map(|bucket| bucket.len()).sum()
    }

    /// Opaque copy of the reference counts, for world state snapshots. Hulls are shared, not copied.
    pub fn snapshot(&self) -> HullDatabaseSnapshot {
        HullDatabaseSnapshot {
            entries: self.entries.clone(),
        }
    }

    pub fn restore(&mut self, snapshot: HullDatabaseSnapshot) {
        self.entries = snapshot.entries;
    }
}

impl World {
    /// The per-shape AABB margin, a fraction of the shape extent capped at the global maximum.
    pub(crate) fn compute_shape_margin(&self, shape: &Shape) -> f32 {
        let mut margin: f32 = 0.0;

        match &shape.geometry {
            ShapeGeometry::Sphere(sphere) => {
                margin = sphere.radius;
            }

            ShapeGeometry::Capsule(capsule) => {
                margin = 0.5 * distance(capsule.center2, capsule.center1) + capsule.radius;
            }

            ShapeGeometry::Hull(hull) => {
                let points = hull.points();
                let mut max_extent_sqr: f32 = 0.0;
                for i in 0..hull.vertex_count() as usize {
                    let dist_sqr = distance_squared(points[i], hull.center());
                    max_extent_sqr = max_float(max_extent_sqr, dist_sqr);
                }
                margin = max_extent_sqr.sqrt();
            }

            ShapeGeometry::Mesh(_)
            | ShapeGeometry::HeightField(_)
            | ShapeGeometry::Compound(_) => {
                // Non-convex shapes use the maximum margin.
                return b3::max_aabb_margin();
            }
        }

        min_float(
            b3::max_aabb_margin(),
            b3::AABB_MARGIN_FRACTION * margin,
        )
    }

    /// Compute the shape AABB and its fat AABB.
    pub(crate) fn update_shape_aabbs(
        &self,
        shape: &mut Shape,
        fat_aabb: &mut AABB,
        transform: WorldTransform,
        proxy_type: BodyType,
    ) {
        // Compute a bounding box with a speculative margin
        let speculative_distance = b3::speculative_distance();
        let aabb_margin = shape.aabb_margin;

        let aabb = compute_fat_shape_aabb(shape, transform, speculative_distance);
        shape.aabb = aabb;

        // Smaller margin for static bodies. Cannot be zero due to TOI tolerance.
        let margin = if proxy_type == BodyType::StaticBody {
            speculative_distance
        } else {
            aabb_margin
        };
        *fat_aabb = aabb_inflate(aabb, margin);
    }

    /// Create a shape's broad-phase proxy.
    pub(crate) fn create_shape_proxy(
        &mut self,
        shape: &mut Shape,
        type_: BodyType,
        transform: WorldTransform,
        force_pair_creation: bool,
    ) {
        b3_assert(shape.proxy_key == b3::NULL_INDEX);

        let mut fat_aabb = AABB::default();
        self.update_shape_aabbs(shape, &mut fat_aabb, transform, type_);
        self.fat_aabbs[shape.id as usize] = fat_aabb;

        // Create proxies in the broad-phase.
        shape.proxy_key = self.broad_phase.create_proxy(
            fat_aabb,
            type_,
            shape.filter.category_bits,
            shape.id,
            force_pair_creation,
        );
        b3_assert((proxy_type_of(shape.proxy_key) as i32) < BodyType::COUNT);
    }

    // MARK: Creation

    #[allow(clippy::too_many_arguments)]
    fn create_shape_internal(
        &mut self,
        body_id: i32,
        body_transform: WorldTransform,
        def: &ShapeDef,
        geometry: ShapeGeometry,
        shape_type: ShapeType,
        shape_transform: Transform,
        scale: Vec3,
        have_shape_transform: bool,
    ) -> Option<i32> {
        let shape_id = self.shape_id_pool.alloc();

        if shape_id as usize == self.shapes.len() {
            self.shapes.push(Shape::default());
            self.fat_aabbs.push(AABB::default());
        } else {
            b3_assert(self.shapes[shape_id as usize].id == b3::NULL_INDEX);
        }

        b3_assert(self.fat_aabbs.len() == self.shapes.len());

        let mut shape = self.shapes[shape_id as usize].clone();
        shape.geometry = geometry;
        shape.type_ = shape_type;

        match shape_type {
            ShapeType::HullShape => {
                // The geometry read below borrows `shape`, so the interned hull is returned out of
                // the scope and written back after the read has ended.
                let mut shared: Option<Hull> = None;
                if let ShapeGeometry::Hull(hull) = &shape.geometry {
                    if have_shape_transform {
                        // The transform and non-uniform scale are baked into fresh data, then shared.
                        match clone_and_transform_hull(hull, shape_transform, scale) {
                            Some(baked) => {
                                shared = Some(self.hull_database.add_owned(baked));
                            }
                            None => {
                                self.shape_id_pool.free(shape_id);
                                let mut failed = self.shapes[shape_id as usize].clone();
                                failed.id = b3::NULL_INDEX;
                                self.shapes[shape_id as usize] = failed;
                                return None;
                            }
                        }
                    } else {
                        shared = self.hull_database.add(hull);
                    }
                }
                if let Some(shared) = shared {
                    shape.geometry = ShapeGeometry::Hull(shared);
                }
            }

            _ => {}
        }

        let mut body = self.bodies[body_id as usize].clone();

        shape.id = shape_id;
        shape.body_id = body.id;
        shape.density = def.density;
        shape.explosion_scale = def.explosion_scale;
        shape.filter = def.filter;
        shape.user_data = def.user_data;
        shape.user_shape = None;
        shape.flags = 0;
        shape.flags |= if def.enable_sensor_events {
            ShapeFlags::ENABLE_SENSOR_EVENTS.0
        } else {
            0
        };
        shape.flags |= if def.enable_contact_events {
            ShapeFlags::ENABLE_CONTACT_EVENTS.0
        } else {
            0
        };
        shape.flags |= if def.enable_custom_filtering {
            ShapeFlags::ENABLE_CUSTOM_FILTERING.0
        } else {
            0
        };
        shape.flags |= if def.enable_hit_events {
            ShapeFlags::ENABLE_HIT_EVENTS.0
        } else {
            0
        };
        shape.flags |= if def.enable_pre_solve_events {
            ShapeFlags::ENABLE_PRE_SOLVE_EVENTS.0
        } else {
            0
        };
        shape.flags |= if def.enable_speculative_contact {
            ShapeFlags::ENABLE_SPECULATIVE.0
        } else {
            0
        };
        shape.proxy_key = b3::NULL_INDEX;
        let local_centroid = get_shape_centroid(&shape);
        shape.local_centroid = local_centroid;
        let aabb_margin = self.compute_shape_margin(&shape);
        shape.aabb_margin = aabb_margin;
        shape.aabb = AABB::default();
        self.fat_aabbs[shape_id as usize] = AABB::default();
        shape.name_id = self.names.add(def.name.as_deref());
        shape.generation = shape.generation.wrapping_add(1);

        if let ShapeGeometry::Compound(compound) = &shape.geometry {
            shape.materials = compound.materials();
            self.compound_shape_count += 1;
        } else if def.materials.len() > 1 {
            // Per triangle materials need a heap array.
            shape.materials = def.materials.clone();
        } else {
            // The common case is one material, stored inline with no allocation.
            shape.materials = vec![def.materials.first().copied().unwrap_or(def.base_material)];
        }

        self.shapes[shape_id as usize] = shape.clone();

        if body.set_index != SetType::DisabledSet.raw_value() {
            let proxy_type = body.type_;
            let force_pair_creation =
                def.invoke_contact_creation && shape.type_ != ShapeType::CompoundShape;
            let mut mutable_shape = self.shapes[shape_id as usize].clone();
            self.create_shape_proxy(&mut mutable_shape, proxy_type, body_transform, force_pair_creation);
            self.shapes[shape_id as usize] = mutable_shape.clone();
            shape = mutable_shape;
        }

        // Add to shape doubly linked list
        if body.head_shape_id != b3::NULL_INDEX {
            let mut head_shape = self.shapes[body.head_shape_id as usize].clone();
            head_shape.prev_shape_id = shape_id;
            self.shapes[body.head_shape_id as usize] = head_shape;
        }

        shape.prev_shape_id = b3::NULL_INDEX;
        shape.next_shape_id = body.head_shape_id;
        self.shapes[shape_id as usize] = shape.clone();
        body.head_shape_id = shape_id;
        body.shape_count += 1;
        self.bodies[body_id as usize] = body;

        shape.sensor_index = b3::NULL_INDEX;
        self.shapes[shape_id as usize] = shape;

        if def.is_sensor {
            self.create_sensor(shape_id);
        }

        Some(shape_id)
    }

    #[allow(clippy::too_many_arguments)]
    fn create_shape(
        &mut self,
        body_id: BodyID,
        def: &ShapeDef,
        geometry: ShapeGeometry,
        shape_type: ShapeType,
        transform: Transform,
        scale: Vec3,
        have_shape_transform: bool,
    ) -> ShapeID {
        b3_assert(is_valid_float(def.density) && def.density >= 0.0);

        if !self.is_valid_body(body_id) {
            return ShapeID::default();
        }

        let index = self.body_index(body_id);
        let mut body = self.bodies[index as usize].clone();
        if body.type_ != BodyType::StaticBody {
            if shape_type == ShapeType::CompoundShape || shape_type == ShapeType::HeightShape {
                // Compound and height shapes must be on static bodies.
                return ShapeID::default();
            }
            if shape_type == ShapeType::MeshShape {
                // Meshes contribute no mass, so a dynamic body relying on one would get zero
                // inverse mass and silently never move. `create_dynamic_compound` already
                // rejects mesh children for the same reason. Use a hull/sphere/capsule for the
                // moving body, or attach the mesh to a static body.
                return ShapeID::default();
            }
        }

        let body_transform = self.body_sim(&body).transform;

        let Some(shape_id) = self.create_shape_internal(
            index,
            body_transform,
            def,
            geometry,
            shape_type,
            transform,
            scale,
            have_shape_transform,
        ) else {
            return ShapeID::default();
        };

        body = self.bodies[index as usize].clone();
        if def.update_body_mass {
            self.update_body_mass_data(index);
        } else if (body.flags & BodyFlags::DIRTY_MASS.0) == 0 {
            body.flags |= BodyFlags::DIRTY_MASS.0;
            self.bodies[index as usize] = body;
            self.sync_body_flags(index);
        }

        let shape = self.shapes[shape_id as usize].clone();
        ShapeID::new(shape.id + 1, body_id.world0, shape.generation)
    }

    /// Create a sphere shape.
    pub fn create_sphere_shape(
        &mut self,
        body_id: BodyID,
        def: &ShapeDef,
        sphere: Sphere,
    ) -> ShapeID {
        self.create_shape(
            body_id,
            def,
            ShapeGeometry::Sphere(sphere),
            ShapeType::SphereShape,
            Transform::IDENTITY,
            Vec3::new(1.0, 1.0, 1.0),
            false,
        )
    }

    /// Create a capsule shape. Very short capsules become spheres.
    pub fn create_capsule_shape(
        &mut self,
        body_id: BodyID,
        def: &ShapeDef,
        capsule: Capsule,
    ) -> ShapeID {
        b3_assert(is_valid_float(capsule.radius) && capsule.radius >= 0.0);

        if distance(capsule.center1, capsule.center2) < b3::min_capsule_length() {
            // A capsule that is too short becomes a sphere.
            let sphere = Sphere::new(
                0.5 * (capsule.center1 + capsule.center2),
                capsule.radius,
            );
            return self.create_shape(
                body_id,
                def,
                ShapeGeometry::Sphere(sphere),
                ShapeType::SphereShape,
                Transform::IDENTITY,
                Vec3::new(1.0, 1.0, 1.0),
                false,
            );
        }

        self.create_shape(
            body_id,
            def,
            ShapeGeometry::Capsule(capsule),
            ShapeType::CapsuleShape,
            Transform::IDENTITY,
            Vec3::new(1.0, 1.0, 1.0),
            false,
        )
    }

    /// Create a hull shape.
    pub fn create_hull_shape(&mut self, body_id: BodyID, def: &ShapeDef, hull: &Hull) -> ShapeID {
        self.create_shape(
            body_id,
            def,
            ShapeGeometry::Hull(hull.clone()),
            ShapeType::HullShape,
            Transform::IDENTITY,
            Vec3::new(1.0, 1.0, 1.0),
            false,
        )
    }

    /// Create a compound on a static body.
    pub fn create_compound_shape(
        &mut self,
        body_id: BodyID,
        def: &ShapeDef,
        compound: &Compound,
    ) -> ShapeID {
        self.create_shape(
            body_id,
            def,
            ShapeGeometry::Compound(compound.clone()),
            ShapeType::CompoundShape,
            Transform::IDENTITY,
            Vec3::ONE,
            false,
        )
    }

    /// Create a height field on a static body.
    pub fn create_height_field_shape(
        &mut self,
        body_id: BodyID,
        def: &ShapeDef,
        field: HeightField,
    ) -> ShapeID {
        self.create_shape(
            body_id,
            def,
            ShapeGeometry::HeightField(field),
            ShapeType::HeightShape,
            Transform::IDENTITY,
            Vec3::ONE,
            false,
        )
    }

    /// Create a mesh instance sharing immutable triangle geometry.
    /// Swift defaulted `scale` to `Vec3.one`; callers ported from `createMeshShape(bodyId, def, data)`
    /// pass `Vec3::ONE`.
    pub fn create_mesh_shape(
        &mut self,
        body_id: BodyID,
        def: &ShapeDef,
        data: TriangleMesh,
        scale: Vec3,
    ) -> ShapeID {
        self.create_shape(
            body_id,
            def,
            ShapeGeometry::Mesh(Mesh::new(Some(data), safe_scale(scale))),
            ShapeType::MeshShape,
            Transform::IDENTITY,
            scale,
            true,
        )
    }

    /// Create a hull shape baked with a local transform and scale.
    pub(crate) fn create_transformed_hull_shape(
        &mut self,
        body_id: BodyID,
        def: &ShapeDef,
        hull: &Hull,
        transform: Transform,
        scale: Vec3,
    ) -> ShapeID {
        self.create_shape(
            body_id,
            def,
            ShapeGeometry::Hull(hull.clone()),
            ShapeType::HullShape,
            transform,
            scale,
            true,
        )
    }

    /// Get a validated shape.
    pub(crate) fn shape(&self, id: ShapeID) -> Shape {
        b3_assert(self.shape_is_valid(id));
        self.shapes[(id.index1 - 1) as usize].clone()
    }

    /// True while the shape id refers to a live shape in this world.
    ///
    /// Shape creation returns a null `ShapeID` when the request is rejected (for example a mesh
    /// shape on a dynamic body), so check this before dereferencing the result.
    pub fn shape_is_valid(&self, id: ShapeID) -> bool {
        if id.world0 != self.world_id {
            return false;
        }
        if id.index1 < 1 || self.shapes.len() < id.index1 as usize {
            return false;
        }
        let shape = &self.shapes[(id.index1 - 1) as usize];
        if shape.id == b3::NULL_INDEX {
            return false;
        }
        shape.generation == id.generation
    }

    pub(crate) fn make_shape_id(&self, shape_id: i32) -> ShapeID {
        ShapeID::new(
            shape_id + 1,
            self.world_id,
            self.shapes[shape_id as usize].generation,
        )
    }
}

// MARK: - Destruction

impl World {
    /// Destroy a shape on a body. This does not need to be called when destroying a body.
    pub(crate) fn destroy_shape_internal(&mut self, shape_id: i32, wake_bodies: bool) {
        let mut shape = self.shapes[shape_id as usize].clone();
        let body_index = shape.body_id;
        let mut body = self.bodies[body_index as usize].clone();

        if shape.type_ == ShapeType::CompoundShape {
            b3_assert(self.compound_shape_count > 0);
            self.compound_shape_count -= 1;
        }

        // Remove the shape from the body's doubly linked list.
        if shape.prev_shape_id != b3::NULL_INDEX {
            let mut prev_shape = self.shapes[shape.prev_shape_id as usize].clone();
            prev_shape.next_shape_id = shape.next_shape_id;
            self.shapes[shape.prev_shape_id as usize] = prev_shape;
        }

        if shape.next_shape_id != b3::NULL_INDEX {
            let mut next_shape = self.shapes[shape.next_shape_id as usize].clone();
            next_shape.prev_shape_id = shape.prev_shape_id;
            self.shapes[shape.next_shape_id as usize] = next_shape;
        }

        if shape_id == body.head_shape_id {
            body.head_shape_id = shape.next_shape_id;
        }

        body.shape_count -= 1;

        // Remove from broad-phase.
        destroy_shape_proxy(&mut shape, &mut self.broad_phase);

        // Commit shape-list changes before contact destruction can update the body.
        let mut contact_key = body.head_contact_key;
        self.bodies[body_index as usize] = body;
        while contact_key != b3::NULL_INDEX {
            let contact_id = contact_key >> 1;
            let edge_index = (contact_key & 1) as usize;
            let contact = self.contacts[contact_id as usize];
            contact_key = contact.edges[edge_index].next_key;
            if contact.shape_id_a == shape_id || contact.shape_id_b == shape_id {
                self.destroy_contact(contact_id, wake_bodies);
            }
        }
        body = self.bodies[body_index as usize].clone();

        // Destroy the sensor and its touch events.
        self.destroy_sensor(shape_id);

        // Destroy every shape member from the allocation.
        destroy_shape_allocations(&mut self.hull_database, &mut shape);

        // Return shape to free list.
        self.shape_id_pool.free(shape_id);
        shape.id = b3::NULL_INDEX;
        shape.proxy_key = b3::NULL_INDEX;
        self.shapes[shape_id as usize] = shape;

        self.bodies[body_index as usize] = body;
    }

    /// Destroy a shape.
    pub(crate) fn destroy_shape(&mut self, id: ShapeID, update_body_mass: bool) {
        b3_assert(self.shape_is_valid(id));

        let shape_id = id.index1 - 1;
        let shape = self.shapes[shape_id as usize].clone();

        // need to wake bodies because this might be a static body
        let wake_bodies = true;

        let body_index = shape.body_id;
        self.destroy_shape_internal(shape_id, wake_bodies);

        if update_body_mass {
            self.update_body_mass_data(body_index);
        }
    }
}
