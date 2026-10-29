//! Soft body vs rigid shape collision detection. Each vertex retains up to four distinct surface
//! planes. XPBD projects all of them, including independent walls and floors at corners.
//!
//! Vertices are processed in spatial patches. A patch is tested against a shape's bounds before any
//! vertex is, and triangle geometry is fetched once per patch.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodyCollision.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::soft_body::soft_body_runtime::SoftBodyRecord` with, in Swift field order and snake_case
//!   names, `vertex_count()` (Swift's computed `var vertexCount: Int { positions.count }`),
//!   `colliders: Vec<SoftBodyCollider>`, `collider_indices: Vec<i32>`, `has_contact: Vec<bool>`,
//!   `vertex_plane_counts: Vec<u8>`, `vertex_planes: Vec<SoftVertexPlane>`, `edge_contacts:
//!   Vec<SoftEdgePlane>` (owned by `soft_body_edge_collision.rs`), `collide_with_rigid_bodies:
//!   bool`, `velocities: Vec<Vec3>`, `bounds: AABB`, `origin: Pos`, `filter: Filter`,
//!   `positions: Vec<Vec3>`, `vertex_radius: f32`, `enable_edge_collision: bool`, `mesh_edges:
//!   Vec<U32x2>` (see `soft_body_edge_collision.rs` for the `U32x2` note), `plane_normals:
//!   Vec<Vec3>`, `plane_offsets: Vec<f32>`, `inv_masses: Vec<f32>`, `friction: f32`, `restitution:
//!   f32`, `two_way_coupling: bool`, and the associated constant `PLANES_PER_VERTEX: usize` (Swift
//!   `static let planesPerVertex = 4`).
//! - `crate::soft_body::soft_body_runtime::{SoftBodyCollider, SoftVertexPlane}` with Swift's field
//!   shapes (`SoftBodyCollider` `Copy`-assignable with a `Default` matching Swift's memberwise
//!   defaults — notably `awake_index = B3.nullIndex` — and `SoftVertexPlane { normal, offset,
//!   separation, collider: i32 }` `Copy`).
//! - `crate::soft_body::soft_body_settings::SoftBodySettings::patches: Vec<Range<usize>>` (Swift
//!   `[Range<Int>]`).
//! - `soft_body_edge_collision.rs` provides `SoftEdgePlane`, `build_mesh_edges()`, and
//!   `detect_edge_planes()`.
//!
//! Borrowing adaptations (behaviour identical):
//! - Swift read `record.settings.patches` into a copy-on-write local snapshot; Rust clones the
//!   range list once (`settings` is never mutated in this pass, so the clone observes exactly the
//!   same values as Swift's copy).
//! - Swift wrapped read-only buffer views in `withUnsafeBufferPointer`; the Rust record stores
//!   owned vectors, so the same views are plain slice borrows with identical index order.
//! - Swift's `for (a, b, c) in scratch.triangles` iterated a snapshot while `consider` appended to
//!   `scratch.hits`; `collide_triangles` walks `triangles` by index under a snapshotted length —
//!   `consider` never touches `triangles`, so the visit order is unchanged.
//! - `guard ... else { continue }` chains became positive-form `if` bodies (or `if !(cond)` for
//!   single-condition guards) so NaN comparison behaviour and short-circuit order match Swift.
//! - Swift's `Hull.planes` / `Hull.points` optional pointers became the landed slice accessors;
//!   the `guard let` early-outs are kept as the equivalent empty-slice checks.

use core::ops::Range;

use crate::base::constants as b3;
use crate::collision::compound::child_queries::query_compound;
use crate::collision::compound::compound::CompoundChildGeometry;
use crate::collision::distance::shape_distance;
use crate::collision::geometry_queries::closest_point_on_triangle;
use crate::collision::hull::Hull;
use crate::collision::mesh::overlap::query_mesh;
use crate::collision::height_field::regions::query_height_field;
use crate::math::functions::{
    aabb_inflate, aabb_overlaps, aabb_transform, cross, dot, inv_transform_point, invert_transform,
    length, length_squared, max_float, max_vec3, min_vec3, mul_add, mul_sub, mul_transforms,
    normalize, offset_aabb, perp, point_to_segment_distance, rotate_vector, to_relative_transform,
    transform_point,
};
use crate::math::types::{AABB, Matrix3, Transform, Vec3};
use crate::simulation::shape_runtime::{ShapeGeometry, compute_shape_aabb, should_shapes_collide};
use crate::simulation::world_runtime::{SetType, World};
use crate::soft_body::soft_body_edge_collision::SoftEdgePlane;
use crate::soft_body::soft_body_runtime::{SoftBodyCollider, SoftBodyRecord, SoftVertexPlane};
use crate::types::body_def::BodyType;
use crate::types::collision_types::TriangleFeature;
use crate::types::geometry::{Capsule, Sphere};
use crate::types::queries::{DistanceInput, ShapeProxy, SimplexCache};

/// Closest surface feature of a shape to a point, in the shape's local frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
struct SurfaceHit {
    normal: Vec3,
    /// Signed distance from the surface along `normal`; negative inside.
    distance: f32,
    point: Vec3,
}

impl Default for SurfaceHit {
    /// Swift's memberwise defaults: zeroed `normal`/`point` and `distance =
    /// Float.greatestFiniteMagnitude`.
    #[inline]
    fn default() -> Self {
        Self {
            normal: Vec3::ZERO,
            distance: f32::MAX,
            point: Vec3::ZERO,
        }
    }
}

/// Per patch scratch reused across shapes.
#[derive(Clone, Debug)]
struct PatchScratch {
    local: Vec<Vec3>,
    triangles: Vec<(Vec3, Vec3, Vec3)>,
    /// Candidate features found for the patch, in the shape frame.
    hits: Vec<(usize, SurfaceHit)>,
}

impl Default for PatchScratch {
    #[inline]
    fn default() -> Self {
        Self {
            local: Vec::new(),
            triangles: Vec::new(),
            hits: Vec::new(),
        }
    }
}

impl PatchScratch {
    #[inline]
    fn consider(&mut self, vertex: usize, hit: SurfaceHit) {
        self.hits.push((vertex, hit));
    }
}

impl World {
    /// Collect nearby rigid shapes and choose a collision plane for each vertex.
    pub(crate) fn detect_soft_body_collisions(&self, record: &mut SoftBodyRecord, dt: f32) {
        let count = record.vertex_count();
        record.colliders.clear();
        record.collider_indices.fill(b3::NULL_INDEX);
        record.has_contact.fill(false);
        record.vertex_plane_counts.fill(0);
        record.edge_contacts.clear();
        if !record.collide_with_rigid_bodies || count == 0 {
            return;
        }

        // Speculative margin: a vertex can move at most this far in one step.
        let mut max_speed_squared: f32 = 0.0;
        {
            let v = &record.velocities;
            for i in 0..v.len() {
                max_speed_squared = max_float(max_speed_squared, length_squared(v[i]));
            }
        }
        let margin = max_speed_squared.sqrt() * dt + b3::speculative_distance()
            + record.vertex_radius;
        let query_bounds = offset_aabb(aabb_inflate(record.bounds, margin), record.origin);

        // Candidate shapes in a deterministic order.
        let mut candidates: Vec<i32> = Vec::new();
        {
            let trees = &self.broad_phase.trees;
            let shapes = &self.shapes;
            let bodies = &self.bodies;
            for tree in trees {
                tree.query(query_bounds, record.filter.mask_bits, false, |_, data| {
                    let shape_index = (data as i64) as i32;
                    let shape = &shapes[shape_index as usize];
                    if shape.sensor_index != b3::NULL_INDEX {
                        return true;
                    }
                    if should_shapes_collide(&record.filter, &shape.filter) == false {
                        return true;
                    }
                    if bodies[shape.body_id as usize].set_index
                        == SetType::DisabledSet.raw_value()
                    {
                        return true;
                    }
                    candidates.push(shape_index);
                    true
                });
            }
        }
        if candidates.is_empty() {
            return;
        }
        candidates.sort();

        // Patch bounds once per step.
        let patches: Vec<Range<usize>> = record.settings.patches.clone();
        let mut patch_bounds = vec![AABB::default(); patches.len()];
        {
            let p = &record.positions;
            for (k, range) in patches.iter().enumerate() {
                let mut lower = p[range.start];
                let mut upper = lower;
                for i in range.clone() {
                    lower = min_vec3(lower, p[i]);
                    upper = max_vec3(upper, p[i]);
                }
                patch_bounds[k] = aabb_inflate(AABB::new(lower, upper), margin);
            }
        }

        let mut best = vec![margin; count];
        let mut scratch = PatchScratch::default();
        let radius = record.vertex_radius;

        // Edge bounds for the edge planes.
        if record.enable_edge_collision {
            record.build_mesh_edges();
        }
        let edge_count = if record.enable_edge_collision {
            record.mesh_edges.len()
        } else {
            0
        };
        let mut edge_bounds = vec![AABB::default(); edge_count];
        for e in 0..edge_count {
            let edge = record.mesh_edges[e];
            let p0 = record.positions[edge.x as usize];
            let p1 = record.positions[edge.y as usize];
            edge_bounds[e] = aabb_inflate(AABB::new(min_vec3(p0, p1), max_vec3(p0, p1)), margin);
        }
        let mut edge_best = vec![margin; edge_count];
        let mut edge_planes: Vec<Option<SoftEdgePlane>> = vec![None; edge_count];

        for shape_index in candidates {
            let shape = &self.shapes[shape_index as usize];
            let body_index = shape.body_id;
            let body = &self.bodies[body_index as usize];
            let world_transform = self.body_transform_quick(body_index);
            let transform = to_relative_transform(world_transform, record.origin);
            let shape_bounds = aabb_inflate(compute_shape_aabb(shape, transform), margin);

            let mut collider = SoftBodyCollider::default();
            collider.shape_index = shape_index;
            collider.body_index = body_index;
            let material = shape.material(0);
            collider.friction =
                self.mix_friction(record.friction, 0, material.friction, material.user_material_id);
            collider.restitution = self.mix_restitution(
                record.restitution,
                0,
                material.restitution,
                material.user_material_id,
            );
            if body.set_index == SetType::AwakeSet.raw_value() {
                let set = SetType::AwakeSet.raw_value() as usize;
                let sim = self.solver_sets[set].body_sims[body.local_index as usize];
                let state = self.solver_sets[set].body_states[body.local_index as usize];
                collider.awake_index = body.local_index;
                collider.is_dynamic =
                    body.type_ == BodyType::DynamicBody && record.two_way_coupling;
                collider.inv_mass = if collider.is_dynamic { sim.inv_mass } else { 0.0 };
                collider.inv_inertia =
                    if collider.is_dynamic { sim.inv_inertia_world } else { Matrix3::default() };
                collider.center = sim.center;
                collider.linear_velocity = state.linear_velocity;
                collider.angular_velocity = state.angular_velocity;
            } else {
                collider.center = world_transform.p;
                // A sleeping dynamic body is treated as static until something wakes it.
                collider.is_dynamic = false;
            }
            collider.original_linear_velocity = collider.linear_velocity;
            collider.original_angular_velocity = collider.angular_velocity;

            let collider_index = record.colliders.len() as i32;
            let mut used = false;

            for (k, range) in patches.iter().enumerate() {
                if !aabb_overlaps(patch_bounds[k], shape_bounds) {
                    continue;
                }
                // Patch vertices in the shape frame.
                scratch.local.clear();
                scratch.hits.clear();
                for i in range.clone() {
                    scratch
                        .local
                        .push(inv_transform_point(transform, record.positions[i]));
                }
                let local_bounds = aabb_transform_inverse(transform, patch_bounds[k]);
                self.collide_geometry(
                    &shape.geometry,
                    Transform::IDENTITY,
                    range.clone(),
                    local_bounds,
                    margin + radius,
                    &mut scratch,
                );

                for (vertex, hit) in scratch.hits.iter() {
                    let vertex = *vertex;
                    let hit = *hit;
                    let separation = hit.distance - radius;
                    let n = rotate_vector(transform.q, hit.normal);
                    let point = transform_point(transform, hit.point);
                    if !(separation < margin) {
                        continue;
                    }
                    let offset = dot(n, point) + radius;
                    used = record.add_vertex_plane(
                        vertex,
                        SoftVertexPlane {
                            normal: n,
                            offset,
                            separation,
                            collider: collider_index,
                        },
                    ) || used;
                    // Keep the closest plane for the edge detector's flat-surface rejection.
                    if separation < best[vertex] {
                        best[vertex] = separation;
                        record.plane_normals[vertex] = n;
                        record.plane_offsets[vertex] = offset;
                        record.collider_indices[vertex] = collider_index;
                    }
                }
            }

            if edge_count > 0
                && self.detect_edge_planes(
                    record,
                    &shape.geometry,
                    transform,
                    shape_bounds,
                    &edge_bounds,
                    margin,
                    collider_index,
                    &mut edge_best,
                    &mut edge_planes,
                )
            {
                used = true;
            }

            if used {
                record.colliders.push(collider);
            }
        }
        for plane in edge_planes {
            if let Some(plane) = plane {
                record.edge_contacts.push(plane);
            }
        }
    }

    /// Dispatch on geometry. `child_transform` places the geometry in the shape frame.
    fn collide_geometry(
        &self,
        geometry: &ShapeGeometry,
        child_transform: Transform,
        range: Range<usize>,
        local_bounds: AABB,
        reach: f32,
        scratch: &mut PatchScratch,
    ) {
        match geometry {
            ShapeGeometry::Sphere(sphere) => {
                self.collide_sphere(*sphere, child_transform, range, reach, scratch)
            }
            ShapeGeometry::Capsule(capsule) => {
                self.collide_capsule(*capsule, child_transform, range, reach, scratch)
            }
            ShapeGeometry::Hull(hull) => {
                self.collide_hull(hull, child_transform, range, reach, scratch)
            }
            ShapeGeometry::Mesh(mesh) => {
                scratch.triangles.clear();
                let bounds = aabb_transform_inverse(child_transform, local_bounds);
                query_mesh(mesh, bounds, |a, b, c, _| {
                    scratch.triangles.push((
                        transform_point(child_transform, a),
                        transform_point(child_transform, b),
                        transform_point(child_transform, c),
                    ));
                    true
                });
                self.collide_triangles(range, reach, scratch);
            }
            ShapeGeometry::HeightField(field) => {
                scratch.triangles.clear();
                query_height_field(field, local_bounds, |a, b, c, _| {
                    scratch.triangles.push((a, b, c));
                    true
                });
                self.collide_triangles(range, reach, scratch);
            }
            ShapeGeometry::Compound(compound) => {
                let mut children: Vec<usize> = Vec::new();
                query_compound(compound, local_bounds, |child| {
                    children.push(child);
                    true
                });
                for c in children {
                    let child = compound.child(c);
                    let xf = mul_transforms(child_transform, child.transform);
                    match child.geometry {
                        CompoundChildGeometry::Sphere(s) => {
                            self.collide_sphere(s, xf, range.clone(), reach, scratch)
                        }
                        CompoundChildGeometry::Capsule(s) => {
                            self.collide_capsule(s, xf, range.clone(), reach, scratch)
                        }
                        CompoundChildGeometry::Hull(h) => {
                            self.collide_hull(&h, xf, range.clone(), reach, scratch)
                        }
                        CompoundChildGeometry::Mesh(m) => self.collide_geometry(
                            &ShapeGeometry::Mesh(m),
                            xf,
                            range.clone(),
                            local_bounds,
                            reach,
                            scratch,
                        ),
                    }
                }
            }
        }
    }

    fn collide_sphere(
        &self,
        sphere: Sphere,
        xf: Transform,
        range: Range<usize>,
        reach: f32,
        scratch: &mut PatchScratch,
    ) {
        let center = transform_point(xf, sphere.center);
        for (k, vertex) in range.enumerate() {
            let d = scratch.local[k] - center;
            let len = length(d);
            let distance = len - sphere.radius;
            if !(distance < reach) {
                continue;
            }
            let n = if len > 1.0e-6 {
                (1.0 / len) * d
            } else {
                Vec3::new(0.0, 1.0, 0.0)
            };
            scratch.consider(
                vertex,
                SurfaceHit {
                    normal: n,
                    distance,
                    point: mul_add(center, sphere.radius, n),
                },
            );
        }
    }

    fn collide_capsule(
        &self,
        capsule: Capsule,
        xf: Transform,
        range: Range<usize>,
        reach: f32,
        scratch: &mut PatchScratch,
    ) {
        let a = transform_point(xf, capsule.center1);
        let b = transform_point(xf, capsule.center2);
        for (k, vertex) in range.enumerate() {
            let p = scratch.local[k];
            let q = point_to_segment_distance(a, b, p);
            let d = p - q;
            let len = length(d);
            let distance = len - capsule.radius;
            if !(distance < reach) {
                continue;
            }
            let n = if len > 1.0e-6 {
                (1.0 / len) * d
            } else {
                perp(normalize(b - a))
            };
            scratch.consider(
                vertex,
                SurfaceHit {
                    normal: n,
                    distance,
                    point: mul_add(q, capsule.radius, n),
                },
            );
        }
    }

    fn collide_hull(
        &self,
        hull: &Hull,
        xf: Transform,
        range: Range<usize>,
        reach: f32,
        scratch: &mut PatchScratch,
    ) {
        // Swift's `hull.planes` / `hull.points` are optional pointers into the hull allocation;
        // the landed accessors return slices and the empty case is the same early-out.
        let planes = hull.planes();
        let points = hull.points();
        if planes.is_empty() || points.is_empty() {
            return;
        }
        let face_count = hull.face_count() as usize;
        let mut cache = SimplexCache::default();
        for (k, vertex) in range.enumerate() {
            // Hull frame.
            let p = inv_transform_point(xf, scratch.local[k]);

            // The largest face separation bounds the distance from below.
            let mut max_separation: f32 = -f32::MAX;
            let mut best_face = 0;
            for f in 0..face_count {
                let s = dot(planes[f].normal, p) - planes[f].offset;
                if s > max_separation {
                    max_separation = s;
                    best_face = f;
                }
            }
            if !(max_separation < reach) {
                continue;
            }

            let mut hit = SurfaceHit::default();
            if max_separation <= 0.0 {
                // Inside: push out through the closest face.
                let n = planes[best_face].normal;
                hit = SurfaceHit {
                    normal: n,
                    distance: max_separation,
                    point: mul_sub(p, max_separation, n),
                };
            } else {
                // Outside: the exact closest point may be on an edge or vertex.
                let point = p;
                let mut input = DistanceInput::default();
                input.proxy_a =
                    ShapeProxy::new(Some(points.as_ptr()), hull.vertex_count(), 0.0);
                // SAFETY: mirrors Swift's `withUnsafePointer(to: &point)`: `point` is a local
                // that stays alive for the whole `shape_distance` call, and the proxy is read
                // as exactly one `Vec3`, which `shape_distance` only borrows.
                input.proxy_b = ShapeProxy::new(
                    Some(core::ptr::from_ref::<Vec3>(&point)),
                    1,
                    0.0,
                );
                input.transform = Transform::IDENTITY;
                input.use_radii = false;
                cache.count = 0;
                let output = shape_distance(&input, &mut cache, None);
                if !(output.distance < reach) {
                    continue;
                }
                let n = if output.distance > 1.0e-6 {
                    output.normal
                } else {
                    planes[best_face].normal
                };
                hit = SurfaceHit {
                    normal: n,
                    distance: output.distance,
                    point: output.point_a,
                };
            }

            hit.normal = rotate_vector(xf.q, hit.normal);
            hit.point = transform_point(xf, hit.point);
            scratch.consider(vertex, hit);
        }
    }

    /// One sided triangles: vertices behind a triangle count as penetrating only when they are
    /// within reach of its face.
    fn collide_triangles(
        &self,
        range: Range<usize>,
        reach: f32,
        scratch: &mut PatchScratch,
    ) {
        if scratch.triangles.is_empty() == false {
            let reach_squared = reach * reach;
            // Swift's `for ... in scratch.triangles` iterated a value snapshot while `consider`
            // appended to `hits`; `consider` never touches `triangles`, so indexing under a
            // snapshotted length visits the same triangles in the same order.
            let triangle_count = scratch.triangles.len();
            for t in 0..triangle_count {
                let (a, b, c) = scratch.triangles[t];
                let normal_length = length(cross(b - a, c - a));
                if !(normal_length > 1.0e-12) {
                    continue;
                }
                let face_normal = (1.0 / normal_length) * cross(b - a, c - a);
                let lower = min_vec3(a, min_vec3(b, c)) - Vec3::new(reach, reach, reach);
                let upper = max_vec3(a, max_vec3(b, c)) + Vec3::new(reach, reach, reach);

                for (k, vertex) in range.clone().enumerate() {
                    let p = scratch.local[k];
                    if p.x < lower.x
                        || p.y < lower.y
                        || p.z < lower.z
                        || p.x > upper.x
                        || p.y > upper.y
                        || p.z > upper.z
                    {
                        continue;
                    }
                    let side = dot(p - a, face_normal);
                    if !(side > -reach) {
                        continue;
                    }
                    let closest = closest_point_on_triangle(a, b, c, p);
                    let d = p - closest.point;
                    let distance_squared = length_squared(d);
                    if side >= 0.0 {
                        if !(distance_squared < reach_squared) {
                            continue;
                        }
                        let len = distance_squared.sqrt();
                        let n = if len > 1.0e-6 {
                            (1.0 / len) * d
                        } else {
                            face_normal
                        };
                        scratch.consider(
                            vertex,
                            SurfaceHit {
                                normal: n,
                                distance: len,
                                point: closest.point,
                            },
                        );
                    } else if closest.feature == TriangleFeature::FeatureTriangleFace {
                        // Behind the face interior: penetration along the face normal.
                        scratch.consider(
                            vertex,
                            SurfaceHit {
                                normal: face_normal,
                                distance: side,
                                point: mul_sub(p, side, face_normal),
                            },
                        );
                    }
                }
            }
        }
    }
}

impl SoftBodyRecord {
    /// Merge parallel planes from one collider, then keep the deepest independent constraints.
    pub fn add_vertex_plane(&mut self, vertex: usize, plane: SoftVertexPlane) -> bool {
        let start = vertex * Self::PLANES_PER_VERTEX;
        let count = self.vertex_plane_counts[vertex] as usize;
        for k in 0..count {
            let old = self.vertex_planes[start + k];
            if old.collider == plane.collider && dot(old.normal, plane.normal) > 0.999 {
                if plane.separation < old.separation {
                    self.vertex_planes[start + k] = plane;
                }
                return true;
            }
        }
        if count < Self::PLANES_PER_VERTEX {
            self.vertex_planes[start + count] = plane;
            self.vertex_plane_counts[vertex] += 1;
            return true;
        }
        let mut worst = 0;
        for k in 1..count {
            if self.vertex_planes[start + k].separation > self.vertex_planes[start + worst].separation
            {
                worst = k;
            }
        }
        if plane.separation < self.vertex_planes[start + worst].separation {
            self.vertex_planes[start + worst] = plane;
            return true;
        }
        false
    }
}

/// Bounds of a world box in a local frame.
#[inline(always)]
pub(crate) fn aabb_transform_inverse(transform: Transform, box_: AABB) -> AABB {
    aabb_transform(invert_transform(transform), box_)
}
