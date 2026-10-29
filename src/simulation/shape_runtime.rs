//! Runtime shape data and geometry helpers.
//!
//! Translated from `Sources/Natsel/Simulation/ShapeRuntime.swift`.

use std::ffi::c_void;

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::compound::compound::{Compound, compute_compound_aabb};
use crate::collision::distance::get_sweep_transform;
use crate::collision::height_field::cells::compute_height_field_aabb;
use crate::collision::height_field::quantization::HeightField;
use crate::collision::hull::Hull;
use crate::collision::hull_queries;
use crate::collision::internal_math::{ShapeExtent, farthest_point_on_aabb};
use crate::collision::proxy::ShapeProxyBuffer;
use crate::math::functions::*;
use crate::math::types::{AABB, Transform, Vec3, WorldTransform};
use crate::simulation::broad_phase::BroadPhase;
use crate::simulation::shape_creation::HullDatabase;
use crate::types::geometry::{Capsule, MassData, Mesh, Sphere};
use crate::types::queries::{CastOutput, RayCastInput, ShapeCastInput, ShapeProxy, Sweep};
use crate::types::shape_def::{Filter, QueryFilter, ShapeType, SurfaceMaterial};

use crate::collision::capsule::{
    compute_capsule_aabb, compute_capsule_mass, compute_swept_capsule_aabb, overlap_capsule,
    ray_cast_capsule, shape_cast_capsule,
};
use crate::collision::compound::child_queries::{
    overlap_compound, ray_cast_compound, shape_cast_compound,
};
use crate::collision::height_field::proxy_overlap::overlap_height_field;
use crate::collision::height_field::sweep_cast::{
    ray_cast_height_field, shape_cast_height_field,
};
use crate::collision::mesh::overlap::{overlap_mesh, ray_cast_mesh, shape_cast_mesh};
use crate::collision::mesh::storage::compute_mesh_aabb;
use crate::collision::sphere::{
    compute_sphere_aabb, compute_sphere_mass, compute_swept_sphere_aabb, overlap_sphere,
    ray_cast_sphere, shape_cast_sphere,
};

// MARK: - Shape flags

/// Shape flags.
///
/// Swift stored the raw value in a `UInt8` field; the Rust `Shape` keeps the same bit values in a
/// `u32` so every flag newtype in the simulation (`BodyFlags`, `ContactFlags`, `ShapeFlags`) shares
/// one representation and one set of operators. No raw-byte layout contract exists for `Shape` (it
/// owns a `Vec<SurfaceMaterial>` and class-backed geometry), so widening the field is behaviour
/// neutral: only the six low bits are ever set.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(transparent)]
pub struct ShapeFlags(pub u32);

impl ShapeFlags {
    pub const ENABLE_SENSOR_EVENTS: ShapeFlags = ShapeFlags(0x01);
    pub const ENABLE_CONTACT_EVENTS: ShapeFlags = ShapeFlags(0x02);
    pub const ENABLE_CUSTOM_FILTERING: ShapeFlags = ShapeFlags(0x04);
    pub const ENABLE_HIT_EVENTS: ShapeFlags = ShapeFlags(0x08);
    pub const ENABLE_PRE_SOLVE_EVENTS: ShapeFlags = ShapeFlags(0x10);
    pub const ENABLE_SPECULATIVE: ShapeFlags = ShapeFlags(0x40);

    /// Mirrors `OptionSet.contains`.
    #[inline(always)]
    pub fn contains(self, other: ShapeFlags) -> bool {
        self.0 & other.0 == other.0
    }

    /// Mirrors `OptionSet.insert`.
    #[inline(always)]
    pub fn insert(&mut self, other: ShapeFlags) {
        self.0 |= other.0;
    }

    /// Mirrors `OptionSet.remove`.
    #[inline(always)]
    pub fn remove(&mut self, other: ShapeFlags) {
        self.0 &= !other.0;
    }

    /// Mirrors `OptionSet.toggle`.
    #[inline(always)]
    pub fn toggle(&mut self, other: ShapeFlags) {
        self.0 ^= other.0;
    }
}

impl std::ops::BitOr for ShapeFlags {
    type Output = ShapeFlags;
    #[inline(always)]
    fn bitor(self, rhs: ShapeFlags) -> ShapeFlags {
        ShapeFlags(self.0 | rhs.0)
    }
}

impl std::ops::BitOrAssign for ShapeFlags {
    #[inline(always)]
    fn bitor_assign(&mut self, rhs: ShapeFlags) {
        self.0 |= rhs.0;
    }
}

impl std::ops::BitAnd for ShapeFlags {
    type Output = ShapeFlags;
    #[inline(always)]
    fn bitand(self, rhs: ShapeFlags) -> ShapeFlags {
        ShapeFlags(self.0 & rhs.0)
    }
}

impl std::ops::BitAndAssign for ShapeFlags {
    #[inline(always)]
    fn bitand_assign(&mut self, rhs: ShapeFlags) {
        self.0 &= rhs.0;
    }
}

impl std::ops::BitXor for ShapeFlags {
    type Output = ShapeFlags;
    #[inline(always)]
    fn bitxor(self, rhs: ShapeFlags) -> ShapeFlags {
        ShapeFlags(self.0 ^ rhs.0)
    }
}

impl std::ops::BitXorAssign for ShapeFlags {
    #[inline(always)]
    fn bitxor_assign(&mut self, rhs: ShapeFlags) {
        self.0 ^= rhs.0;
    }
}

impl std::ops::Not for ShapeFlags {
    type Output = ShapeFlags;
    #[inline(always)]
    fn not(self) -> ShapeFlags {
        ShapeFlags(!self.0)
    }
}

// MARK: - Geometry

/// The geometry carried by a shape.
#[derive(Clone, Debug)]
pub enum ShapeGeometry {
    Sphere(Sphere),
    Capsule(Capsule),
    Hull(Hull),
    /// Shared triangle geometry with instance scale.
    Mesh(Mesh),
    HeightField(HeightField),
    Compound(Compound),
}

// SAFETY: mirrors Swift's `@unchecked Sendable` on the geometry enum. The hull, mesh, and
// compound payloads are reference-counted, immutable-after-construction geometry shares; a shape
// is owned by exactly one `World` and is only handed to worker threads while that `World` is
// locked, so no thread ever mutates the shared storage concurrently.
unsafe impl Send for ShapeGeometry {}
// SAFETY: same reasoning as `Send`; the shared geometry is read-only during parallel phases.
unsafe impl Sync for ShapeGeometry {}

/// A shape is a piece of collision geometry attached to a body.
#[derive(Clone, Debug)]
pub struct Shape {
    pub(crate) id: i32,
    pub(crate) body_id: i32,
    pub(crate) prev_shape_id: i32,
    pub(crate) next_shape_id: i32,
    pub(crate) sensor_index: i32,
    pub(crate) proxy_key: i32,

    pub(crate) type_: ShapeType,
    pub(crate) density: f32,
    pub(crate) explosion_scale: f32,
    pub(crate) aabb_margin: f32,

    pub(crate) aabb: AABB,
    pub(crate) local_centroid: Vec3,

    pub(crate) materials: Vec<SurfaceMaterial>,
    pub(crate) filter: Filter,
    pub(crate) user_data: Option<*mut c_void>,
    pub(crate) user_shape: Option<*mut c_void>,

    pub(crate) name_id: u32,
    pub(crate) generation: u16,

    /// ShapeFlags.
    pub(crate) flags: u32,

    pub(crate) geometry: ShapeGeometry,
}

// SAFETY: mirrors Swift's `@unchecked Sendable` on `Shape`. `user_data` and `user_shape` are
// opaque pointers owned by the API client, exactly as in Swift, and `geometry` shares immutable
// geometry. A `Shape` is reachable only from its owning `World`, which the engine keeps locked for
// the whole duration of a step, so transferring one between threads preserves Swift's exclusivity.
unsafe impl Send for Shape {}

impl Default for Shape {
    #[inline]
    fn default() -> Self {
        Self {
            id: b3::NULL_INDEX,
            body_id: b3::NULL_INDEX,
            prev_shape_id: b3::NULL_INDEX,
            next_shape_id: b3::NULL_INDEX,
            sensor_index: b3::NULL_INDEX,
            proxy_key: b3::NULL_INDEX,
            type_: ShapeType::SphereShape,
            density: 0.0,
            explosion_scale: 1.0,
            aabb_margin: 0.0,
            aabb: AABB::default(),
            local_centroid: Vec3::ZERO,
            materials: Vec::new(),
            filter: Filter::default(),
            user_data: None,
            user_shape: None,
            name_id: 0,
            generation: 0,
            flags: 0,
            geometry: ShapeGeometry::Sphere(Sphere::default()),
        }
    }
}

impl Shape {
    // MARK: Geometry accessors

    pub fn sphere(&self) -> Sphere {
        match &self.geometry {
            ShapeGeometry::Sphere(sphere) => *sphere,
            _ => Sphere::default(),
        }
    }

    pub fn set_sphere(&mut self, value: Sphere) {
        self.geometry = ShapeGeometry::Sphere(value);
    }

    pub fn capsule(&self) -> Capsule {
        match &self.geometry {
            ShapeGeometry::Capsule(capsule) => *capsule,
            _ => Capsule::default(),
        }
    }

    pub fn set_capsule(&mut self, value: Capsule) {
        self.geometry = ShapeGeometry::Capsule(value);
    }

    /// Swift's optional setter ignored `nil`, so the setter only exists for a real hull.
    pub fn hull(&self) -> Option<&Hull> {
        match &self.geometry {
            ShapeGeometry::Hull(hull) => Some(hull),
            _ => None,
        }
    }

    pub fn set_hull(&mut self, value: &Hull) {
        self.geometry = ShapeGeometry::Hull(value.clone());
    }

    /// The material at an index. A single material shape presents its only material.
    pub fn material(&self, index: i32) -> SurfaceMaterial {
        let i = index as usize;
        if index >= 0 && i < self.materials.len() {
            return self.materials[i];
        }
        SurfaceMaterial::default()
    }

    pub fn material_count(&self) -> i32 {
        self.materials.len() as i32
    }
}

// MARK: - Filtering

/// Do two shape filters allow collision?
#[inline(always)]
pub fn should_shapes_collide(filter_a: &Filter, filter_b: &Filter) -> bool {
    if filter_a.group_index == filter_b.group_index && filter_a.group_index != 0 {
        return filter_a.group_index > 0;
    }

    (filter_a.mask_bits & filter_b.category_bits) != 0
        && (filter_a.category_bits & filter_b.mask_bits) != 0
}

/// Does a shape filter allow a query?
#[inline(always)]
pub fn should_query_collide(shape_filter: &Filter, query_filter: &QueryFilter) -> bool {
    (shape_filter.category_bits & query_filter.mask_bits) != 0
        && (shape_filter.mask_bits & query_filter.category_bits) != 0
}

/// Is this shape type convex?
#[inline(always)]
pub fn is_convex(type_: ShapeType) -> bool {
    type_ == ShapeType::SphereShape
        || type_ == ShapeType::CapsuleShape
        || type_ == ShapeType::HullShape
}

// MARK: - Hull geometry helpers

/// Compute the mass properties of a hull.
pub fn compute_hull_mass(hull: &Hull, density: f32) -> MassData {
    let mut out = MassData::default();
    out.mass = density * hull.volume();
    out.center = hull.center();

    // Inertia about the center of mass
    out.inertia = mul_sm(density, hull.central_inertia());
    out
}

/// Compute the AABB of a hull.
///
/// The implementation lives in `collision::hull_queries`, which the compound path shares; this is
/// the `ShapeRuntime` entry point for it.
#[inline(always)]
pub fn compute_hull_aabb(hull: &Hull, transform: Transform) -> AABB {
    hull_queries::compute_hull_aabb(hull, transform)
}

/// Compute the swept AABB of a hull.
#[inline(always)]
pub fn compute_swept_hull_aabb(hull: &Hull, xf1: Transform, xf2: Transform) -> AABB {
    hull_queries::compute_swept_hull_aabb(hull, xf1, xf2)
}

/// Do a hull and a point cloud proxy overlap?
#[inline(always)]
pub fn overlap_hull(hull: &Hull, shape_transform: Transform, proxy: &ShapeProxy) -> bool {
    hull_queries::overlap_hull(hull, shape_transform, proxy)
}

/// Ray cast a hull.
#[inline(always)]
pub fn ray_cast_hull(hull: &Hull, input: &RayCastInput) -> CastOutput {
    hull_queries::ray_cast_hull(hull, input)
}

/// Shape cast a hull.
#[inline(always)]
pub fn shape_cast_hull(hull: &Hull, input: &ShapeCastInput) -> CastOutput {
    hull_queries::shape_cast_hull(hull, input)
}

/// Compute the extent of a hull about a local origin.
#[inline(always)]
pub fn compute_hull_extent(hull: &Hull, origin: Vec3) -> ShapeExtent {
    hull_queries::compute_hull_extent(hull, origin)
}

/// Project the hull surface area onto a plane.
#[inline(always)]
pub fn compute_hull_projected_area(hull: &Hull, direction: Vec3) -> f32 {
    hull_queries::compute_hull_projected_area(hull, direction)
}

// MARK: - Shape queries

/// Compute the local-space AABB of a shape under a transform.
pub fn compute_shape_aabb(shape: &Shape, transform: Transform) -> AABB {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => compute_capsule_aabb(capsule, transform),

        ShapeGeometry::Compound(compound) => compute_compound_aabb(compound, transform),

        ShapeGeometry::HeightField(field) => compute_height_field_aabb(field, transform),

        ShapeGeometry::Hull(hull) => compute_hull_aabb(hull, transform),

        ShapeGeometry::Mesh(mesh) => match &mesh.data {
            Some(data) => compute_mesh_aabb(data, transform, mesh.scale),
            None => AABB::new(transform.p, transform.p),
        },

        ShapeGeometry::Sphere(sphere) => compute_sphere_aabb(sphere, transform),
    }
}

/// Bounds are conservatively rounded outward on the double-to-float boundary.
pub fn compute_shape_aabb_world(shape: &Shape, transform: WorldTransform) -> AABB {
    offset_aabb(
        compute_shape_aabb(shape, Transform::new(Vec3::ZERO, transform.q)),
        transform.p,
    )
}

/// Compute a world AABB for a shape inflated by an extra margin.
pub fn compute_fat_shape_aabb(
    shape: &Shape,
    transform: WorldTransform,
    extra: f32,
) -> AABB {
    let local = aabb_inflate(
        compute_shape_aabb(shape, Transform::new(Vec3::ZERO, transform.q)),
        extra,
    );
    offset_aabb(local, transform.p)
}

/// Compute the swept AABB of a shape.
pub fn compute_swept_shape_aabb(shape: &Shape, sweep: Sweep, time: f32) -> AABB {
    b3_assert(0.0 <= time && time <= 1.0);
    let xf1 = Transform::new(
        sweep.c1 - rotate_vector(sweep.q1, sweep.local_center),
        sweep.q1,
    );
    let xf2 = get_sweep_transform(&sweep, time);

    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => compute_swept_capsule_aabb(capsule, xf1, xf2),

        ShapeGeometry::Hull(hull) => compute_swept_hull_aabb(hull, xf1, xf2),

        ShapeGeometry::Sphere(sphere) => compute_swept_sphere_aabb(sphere, xf1, xf2),

        _ => {
            b3_assert(false);
            AABB::new(xf1.p, xf1.p)
        }
    }
}

/// The local centroid of a shape.
pub fn get_shape_centroid(shape: &Shape) -> Vec3 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => lerp(capsule.center1, capsule.center2, 0.5),
        ShapeGeometry::Sphere(sphere) => sphere.center,
        ShapeGeometry::Compound(compound) => {
            aabb_center(compute_compound_aabb(compound, Transform::IDENTITY))
        }
        ShapeGeometry::HeightField(field) => {
            aabb_center(compute_height_field_aabb(field, Transform::IDENTITY))
        }
        ShapeGeometry::Hull(hull) => hull.center(),
        ShapeGeometry::Mesh(mesh) => match &mesh.data {
            Some(data) => aabb_center(compute_mesh_aabb(data, Transform::IDENTITY, mesh.scale)),
            None => Vec3::ZERO,
        },
    }
}

/// The shape surface area.
pub fn get_shape_area(shape: &Shape) -> f32 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            2.0 * length(capsule.center1 - capsule.center2) + 2.0 * b3::PI * capsule.radius
        }
        ShapeGeometry::Hull(hull) => hull.surface_area(),
        ShapeGeometry::Sphere(sphere) => 2.0 * b3::PI * sphere.radius,
        _ => 0.0,
    }
}

/// The shape surface area projected onto a plane.
pub fn get_shape_projected_area(shape: &Shape, plane_normal: Vec3) -> f32 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            let radius = capsule.radius;
            let axis = capsule.center2 - capsule.center1;
            let projected_length = length(cross(axis, plane_normal));
            let cylinder_area = 2.0 * radius * projected_length;
            let sphere_area = b3::PI * radius * radius;
            sphere_area + cylinder_area
        }

        ShapeGeometry::Hull(hull) => compute_hull_projected_area(hull, plane_normal),

        ShapeGeometry::Sphere(sphere) => b3::PI * sphere.radius * sphere.radius,

        _ => 0.0,
    }
}

/// Compute the shape mass.
pub fn compute_shape_mass(shape: &Shape) -> MassData {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => compute_capsule_mass(capsule, shape.density),

        ShapeGeometry::Hull(hull) => compute_hull_mass(hull, shape.density),

        ShapeGeometry::Sphere(sphere) => compute_sphere_mass(sphere, shape.density),

        _ => MassData::default(),
    }
}

/// Compute the min and max extent of a shape about a local center.
pub fn compute_shape_extent(shape: &Shape, local_center: Vec3) -> ShapeExtent {
    let mut extent = ShapeExtent::default();

    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            let radius = capsule.radius;
            extent.min_extent = radius;
            let c1 = capsule.center1 - local_center;
            let c2 = capsule.center2 - local_center;
            let r = Vec3::new(radius, radius, radius);
            extent.max_extent = max_vec3(abs_vec3(c1), abs_vec3(c2)) + r;
        }

        ShapeGeometry::Sphere(sphere) => {
            let radius = sphere.radius;
            extent.min_extent = radius;
            let h = abs_vec3(sphere.center - local_center);
            let r = Vec3::new(radius, radius, radius);
            extent.max_extent = h + r;
        }

        ShapeGeometry::Hull(hull) => extent = compute_hull_extent(hull, local_center),

        ShapeGeometry::Compound(compound) => {
            let aabb = compute_compound_aabb(compound, Transform::IDENTITY);
            let r1 = length(aabb.lower_bound - local_center);
            let r2 = length(aabb.upper_bound - local_center);
            extent.min_extent = min_float(r1, r2);
            extent.max_extent = abs_vec3(farthest_point_on_aabb(aabb, local_center) - local_center);
        }

        ShapeGeometry::Mesh(mesh) => {
            if let Some(data) = &mesh.data {
                let aabb = compute_mesh_aabb(data, Transform::IDENTITY, mesh.scale);
                let r1 = length(aabb.lower_bound - local_center);
                let r2 = length(aabb.upper_bound - local_center);
                extent.min_extent = min_float(r1, r2);
                extent.max_extent =
                    abs_vec3(farthest_point_on_aabb(aabb, local_center) - local_center);
            }
        }

        _ => {}
    }

    extent
}

/// Compute the shape min extent.
pub fn compute_shape_min_extent(shape: &Shape, local_center: Vec3) -> f32 {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => capsule.radius,
        ShapeGeometry::Sphere(sphere) => sphere.radius,
        ShapeGeometry::Hull(hull) => hull.inner_radius(),
        _ => compute_shape_extent(shape, local_center).min_extent,
    }
}

/// Ray cast a shape.
pub fn ray_cast_shape(
    shape: &Shape,
    transform: Transform,
    input: &RayCastInput,
) -> CastOutput {
    let mut local_input = *input;
    local_input.origin = inv_transform_point(transform, input.origin);
    local_input.translation = inv_rotate_vector(transform.q, input.translation);

    let mut output = CastOutput::default();
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => output = ray_cast_capsule(capsule, &local_input),
        ShapeGeometry::Sphere(sphere) => output = ray_cast_sphere(sphere, &local_input),
        ShapeGeometry::Hull(hull) => output = ray_cast_hull(hull, &local_input),
        ShapeGeometry::Mesh(mesh) => output = ray_cast_mesh(mesh, local_input),
        ShapeGeometry::Compound(compound) => output = ray_cast_compound(compound, local_input),
        ShapeGeometry::HeightField(field) => {
            output = ray_cast_height_field(field, &local_input)
        }
    }

    output.point = transform_point(transform, output.point);
    output.normal = rotate_vector(transform.q, output.normal);
    output
}

/// Shape cast a shape.
pub fn shape_cast_shape(
    shape: &Shape,
    transform: Transform,
    input: &ShapeCastInput,
) -> CastOutput {
    let mut local_input = *input;
    // Swift allocated `B3.maxShapeCastPoints` scratch points once per cast; keeping the fixed
    // array keeps the copy loop, the bound, and the pointer handed to the narrow phase identical.
    let mut local_points = [Vec3::ZERO; b3::MAX_SHAPE_CAST_POINTS as usize];

    let count = min_int(input.proxy.count, b3::MAX_SHAPE_CAST_POINTS);
    let source = input.proxy.points();
    for i in 0..count as usize {
        local_points[i] = inv_transform_point(transform, source[i]);
    }

    let mut output = CastOutput::default();
    {
        // SAFETY: `local_points` is a fixed-size array whose storage cannot move while the cast
        // runs, and `count` never exceeds its length. This is the same invariant Swift's
        // `withUnsafeBufferPointer` gave the closure, which is why the raw pointer is kept instead
        // of redesigning the API.
        local_input.proxy = ShapeProxy::new(Some(local_points.as_ptr()), count, input.proxy.radius);
        local_input.translation = inv_rotate_vector(transform.q, input.translation);

        match &shape.geometry {
            ShapeGeometry::Capsule(capsule) => output = shape_cast_capsule(capsule, &local_input),
            ShapeGeometry::Hull(hull) => output = shape_cast_hull(hull, &local_input),
            ShapeGeometry::Sphere(sphere) => output = shape_cast_sphere(sphere, &local_input),
            ShapeGeometry::Mesh(mesh) => output = shape_cast_mesh(mesh, local_input),
            ShapeGeometry::Compound(compound) => {
                output = shape_cast_compound(compound, &local_input)
            }
            ShapeGeometry::HeightField(field) => {
                output = shape_cast_height_field(field, &local_input)
            }
        }
    }

    output.point = transform_point(transform, output.point);
    output.normal = rotate_vector(transform.q, output.normal);
    output
}

/// Does a shape overlap a point cloud proxy?
pub fn overlap_shape(shape: &Shape, transform: Transform, proxy: &ShapeProxy) -> bool {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => overlap_capsule(capsule, transform, proxy),
        ShapeGeometry::Hull(hull) => overlap_hull(hull, transform, proxy),
        ShapeGeometry::Sphere(sphere) => overlap_sphere(sphere, transform, proxy),
        ShapeGeometry::Mesh(mesh) => overlap_mesh(mesh, transform, *proxy),
        ShapeGeometry::Compound(compound) => overlap_compound(compound, transform, *proxy),
        ShapeGeometry::HeightField(field) => overlap_height_field(field, transform, *proxy),
    }
}

// MARK: - Proxies

/// Build a shape proxy.
pub fn make_shape_proxy(shape: &Shape) -> ShapeProxyBuffer {
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            ShapeProxyBuffer::segment(capsule.center1, capsule.center2, capsule.radius)
        }

        ShapeGeometry::Sphere(sphere) => ShapeProxyBuffer::point(sphere.center, sphere.radius),

        ShapeGeometry::Hull(hull) => ShapeProxyBuffer::from_hull(hull.clone()),

        _ => {
            b3_assert(false);
            ShapeProxyBuffer::point(Vec3::ZERO, 0.0)
        }
    }
}

// MARK: - Destruction

/// Destroy a shape's broad-phase proxy.
pub fn destroy_shape_proxy(shape: &mut Shape, broad_phase: &mut BroadPhase) {
    if shape.proxy_key != b3::NULL_INDEX {
        broad_phase.destroy_proxy(shape.proxy_key);
        shape.proxy_key = b3::NULL_INDEX;
    }
}

/// Destroy per-shape allocations. Hull data returns to the world database.
pub fn destroy_shape_allocations(hull_database: &mut HullDatabase, shape: &mut Shape) {
    if let ShapeGeometry::Hull(hull) = &shape.geometry {
        hull_database.remove(hull.clone());
    }
    shape.geometry = ShapeGeometry::Sphere(Sphere::default());

    shape.materials.clear();
}
