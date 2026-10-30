//! Edges of soft bodies against rigid shapes. Vertex planes alone let a thin feature (a rod, a
//! blade, a hull edge, a mesh edge) slip between two vertices: neither vertex comes near it while the
//! edge between them passes through. Once per step every surface edge looks for the closest feature
//! of the nearby shapes; when that point lies strictly inside the edge (an end point is already the
//! vertex's job), the edge gets a contact plane through the feature. Each substep keeps the edge point
//! on the positive side of its plane, sharing the correction between the two vertices by their
//! weights and exchanging the impulse with dynamic bodies.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodyEdgeCollision.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::soft_body::soft_body_runtime::SoftBodyRecord`: `mesh_edges` and `edge_contacts` in
//!   addition to the fields recorded in `soft_body_collision.rs`. Swift's `meshEdges:
//!   [SIMD2<UInt32>]` is assumed to be `Vec<U32x2>` where `U32x2` is
//!   `crate::soft_body::soft_body_settings`' port of `SIMD2<UInt32>` — a `Copy` `#[repr(C)]` pair
//!   with `pub x: u32`, `pub y: u32` — matching the landed `world_debug_draw_extras.rs` assumption
//!   that an edge's pair is read through `.x` / `.y`.
//! - `crate::soft_body::soft_body_settings::SoftBodySettings`: `faces: Vec<SoftBodyFace>` where a
//!   face carries `vertex` with `.x/.y/.z: u32` and a `is_degenerate()` accessor, and `edges:
//!   Vec<SoftBodyEdge>` where an edge carries `vertex: U32x2`.
//! - `crate::soft_body::soft_body_runtime::SoftBodyCollider` fields `is_dynamic`, `displacement`,
//!   `rotation`, `linear_velocity`, `angular_velocity`, `center: Pos`, `inv_mass`, `inv_inertia:
//!   Matrix3`, `friction`, `updated`, `contact_count: i32`, and `point_velocity(r) = linear_velocity
//!   + cross(angular_velocity, r)`.
//! - `soft_body_collision.rs` provides `aabb_transform_inverse`.
//!
//! Borrowing adaptations (behaviour identical): Swift's buffer-pointer parameters keep their
//! declared mutability (`x`, `x0`, `contact`, `colliders`) as slices; `x0` is only read, exactly as
//! in Swift. `edgeContacts.withUnsafeBufferPointer` becomes a plain slice borrow. Swift's
//! `withUnsafePointer(to:) + withMemoryRebound(to: Vec3.self, ...)` over a `(Vec3, Vec3)` /
//! `(Vec3, Vec3, Vec3)` tuple is reproduced with `slice::from_raw_parts` over a pinned local; a
//! tuple of `#[repr(C)]` three-float vectors with no padding lays the points out contiguously in
//! the same order. The `guard ..., ..., else` chains became positive-form `if` bodies so the
//! short-circuit and NaN behaviour match Swift exactly.

use std::collections::HashSet;

use crate::collision::compound::child_queries::query_compound;
use crate::collision::compound::compound::CompoundChildGeometry;
use crate::collision::distance::shape_distance;
use crate::collision::height_field::regions::query_height_field;
use crate::collision::mesh::overlap::query_mesh;
use crate::math::functions::{
    aabb_overlaps, clamp_float, cross, dot, inv_transform_point, length, min_float, mul_add,
    mul_mv, mul_sub, mul_transforms, point_to_segment_distance, rotate_vector, segment_distance,
    sub_pos, transform_point,
};
use crate::math::types::{AABB, Transform, Vec3};
use crate::simulation::shape_runtime::ShapeGeometry;
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_collision::aabb_transform_inverse;
use crate::soft_body::soft_body_runtime::{SoftBodyCollider, SoftBodyRecord};
use crate::soft_body::soft_body_settings::U32x2;
use crate::types::queries::{DistanceInput, ShapeProxy, SimplexCache};

/// An edge point held on the positive side of a plane fixed to a rigid shape.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SoftEdgePlane {
    pub v0: i32,
    pub v1: i32,
    /// Position of the contact point along the edge, from v0 to v1.
    pub s: f32,
    pub normal: Vec3,
    pub offset: f32,
    pub collider: i32,
}

/// Closest feature of a shape to a segment, in the shape frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SegmentHit {
    /// Fraction along the segment.
    pub s: f32,
    pub normal: Vec3,
    pub point: Vec3,
    /// Distance from the surface; negative inside a rounded feature.
    pub distance: f32,
}

impl Default for SegmentHit {
    /// Swift's memberwise defaults: zeroed `s`/`normal`/`point` and `distance =
    /// Float.greatestFiniteMagnitude`.
    #[inline]
    fn default() -> Self {
        Self {
            s: 0.0,
            normal: Vec3::ZERO,
            point: Vec3::ZERO,
            distance: f32::MAX,
        }
    }
}

/// Edge contacts only where vertices cannot see the feature.
const INTERIOR_FRACTION: f32 = 0.02;

/// Vertex normals closer than about 8 degrees mean a flat surface under the edge.
const FLAT_COSINE: f32 = 0.99;

impl SoftBodyRecord {
    /// Surface edges: the edges of the faces, or the constraint edges of bodies without faces.
    pub fn build_mesh_edges(&mut self) {
        if self.mesh_edges.is_empty() == false {
            return;
        }
        if self.settings.faces.is_empty() {
            self.mesh_edges = self.settings.edges.iter().map(|e| e.vertex).collect();
            return;
        }
        let mut keys = HashSet::<u64>::new();
        let mut edges: Vec<U32x2> = Vec::new();
        for face in &self.settings.faces {
            if face.is_degenerate() == false {
                for (a, b) in [
                    (face.vertex.x, face.vertex.y),
                    (face.vertex.y, face.vertex.z),
                    (face.vertex.z, face.vertex.x),
                ] {
                    let lo = a.min(b);
                    let hi = a.max(b);
                    if keys.insert((lo as u64) << 32 | hi as u64) {
                        edges.push(U32x2 { x: lo, y: hi });
                    }
                }
            }
        }
        edges.sort_by(|a, b| if a.x != b.x { a.x.cmp(&b.x) } else { a.y.cmp(&b.y) });
        self.mesh_edges = edges;
    }

    /// Keep edge points above their rigid planes. Runs before the vertex collisions of a substep, so
    /// the velocities derived there include these corrections.
    pub fn apply_edge_collisions(
        &self,
        x: &mut [Vec3],
        x0: &mut [Vec3],
        w: &[f32],
        contact: &mut [bool],
        colliders: &mut [SoftBodyCollider],
        h: f32,
        last: bool,
    ) {
        if self.edge_contacts.is_empty() == false {
            let inv_h = 1.0 / h;
            // Dynamic bodies have moved by their tracked velocities; planes move with them.
            for i in 0..colliders.len() {
                if colliders[i].is_dynamic {
                    colliders[i].displacement =
                        mul_add(colliders[i].displacement, h, colliders[i].linear_velocity);
                    colliders[i].rotation =
                        mul_add(colliders[i].rotation, h, colliders[i].angular_velocity);
                }
            }
            for plane in self.edge_contacts.iter() {
                let i0 = plane.v0 as usize;
                let i1 = plane.v1 as usize;
                let s = plane.s;
                let a0 = 1.0 - s;
                let point = a0 * x[i0] + s * x[i1];
                let ci = plane.collider as usize;
                let r = point - sub_pos(colliders[ci].center, self.origin);
                let shift = colliders[ci].displacement + cross(colliders[ci].rotation, r);
                let penetration =
                    plane.offset + dot(plane.normal, shift) - dot(plane.normal, point);
                if !(penetration > 0.0) {
                    continue;
                }
                let w_edge = a0 * a0 * w[i0] + s * s * w[i1];
                if !(w_edge > 0.0) {
                    continue;
                }

                let mut w_body: f32 = 0.0;
                if colliders[ci].is_dynamic {
                    let rn = cross(r, plane.normal);
                    w_body = colliders[ci].inv_mass + dot(rn, mul_mv(colliders[ci].inv_inertia, rn));
                }
                let lambda = penetration / (w_edge + w_body);
                let mut correction = lambda * plane.normal;

                // Friction against the edge point's motion relative to the body over the substep.
                let start = a0 * x0[i0] + s * x0[i1];
                let motion = (point - start) - h * colliders[ci].point_velocity(r);
                let tangent = motion - dot(motion, plane.normal) * plane.normal;
                let tangent_length = length(tangent);
                if tangent_length > 1.0e-9 {
                    let k = min_float(
                        colliders[ci].friction * penetration / tangent_length,
                        1.0,
                    );
                    correction = correction - (k / (w_edge + w_body)) * tangent;
                }

                x[i0] = mul_add(x[i0], a0 * w[i0], correction);
                x[i1] = mul_add(x[i1], s * w[i1], correction);
                if colliders[ci].is_dynamic {
                    // The edge's momentum change, applied to the body with the opposite sign.
                    let impulse = inv_h * correction;
                    let dv = colliders[ci].inv_mass * impulse;
                    let dw = mul_mv(colliders[ci].inv_inertia, cross(r, impulse));
                    colliders[ci].linear_velocity = colliders[ci].linear_velocity - dv;
                    colliders[ci].angular_velocity = colliders[ci].angular_velocity - dw;
                    // The body moves away within this substep too, so later edges see it moved.
                    colliders[ci].displacement =
                        mul_sub(colliders[ci].displacement, h, dv);
                    colliders[ci].rotation = mul_sub(colliders[ci].rotation, h, dw);
                    colliders[ci].updated = true;
                }
                if last {
                    contact[i0] = true;
                    contact[i1] = true;
                    colliders[ci].contact_count += 1;
                }
            }
        }
    }
}

impl World {
    /// Closest feature of `geometry` (placed by `child_transform` in the shape frame) to the segment
    /// (a, b), both in the shape frame. Only hits closer than `reach` count.
    pub(crate) fn closest_segment_feature(
        &self,
        geometry: &ShapeGeometry,
        child_transform: Transform,
        a: Vec3,
        b: Vec3,
        local_bounds: AABB,
        reach: f32,
        best: &mut SegmentHit,
    ) {
        match geometry {
            ShapeGeometry::Sphere(sphere) => {
                let center = transform_point(child_transform, sphere.center);
                let q = point_to_segment_distance(a, b, center);
                self.consider(q, center, sphere.radius, a, b, reach, best);
            }
            ShapeGeometry::Capsule(capsule) => {
                let c1 = transform_point(child_transform, capsule.center1);
                let c2 = transform_point(child_transform, capsule.center2);
                let result = segment_distance(a, b, c1, c2);
                self.consider(result.point1, result.point2, capsule.radius, a, b, reach, best);
            }
            ShapeGeometry::Hull(hull) => {
                // Swift's `hull.points` optional pointer; the empty slice is the same early-out.
                let points = hull.points();
                if points.is_empty() {
                    return;
                }
                // Segment in the hull frame.
                let mut segment = (
                    inv_transform_point(child_transform, a),
                    inv_transform_point(child_transform, b),
                );
                self.segment_against_points(
                    points,
                    hull.vertex_count(),
                    &mut segment,
                    child_transform,
                    a,
                    b,
                    reach,
                    best,
                );
            }
            ShapeGeometry::Mesh(mesh) => {
                let bounds = aabb_transform_inverse(child_transform, local_bounds);
                query_mesh(mesh, bounds, |p, q, r, _| {
                    self.segment_against_triangle(
                        transform_point(child_transform, p),
                        transform_point(child_transform, q),
                        transform_point(child_transform, r),
                        a,
                        b,
                        reach,
                        best,
                    );
                    true
                });
            }
            ShapeGeometry::HeightField(field) => {
                query_height_field(field, local_bounds, |p, q, r, _| {
                    self.segment_against_triangle(p, q, r, a, b, reach, best);
                    true
                });
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
                        CompoundChildGeometry::Sphere(s) => self.closest_segment_feature(
                            &ShapeGeometry::Sphere(s),
                            xf,
                            a,
                            b,
                            local_bounds,
                            reach,
                            best,
                        ),
                        CompoundChildGeometry::Capsule(s) => self.closest_segment_feature(
                            &ShapeGeometry::Capsule(s),
                            xf,
                            a,
                            b,
                            local_bounds,
                            reach,
                            best,
                        ),
                        CompoundChildGeometry::Hull(h) => self.closest_segment_feature(
                            &ShapeGeometry::Hull(h),
                            xf,
                            a,
                            b,
                            local_bounds,
                            reach,
                            best,
                        ),
                        CompoundChildGeometry::Mesh(m) => self.closest_segment_feature(
                            &ShapeGeometry::Mesh(m),
                            xf,
                            a,
                            b,
                            local_bounds,
                            reach,
                            best,
                        ),
                    }
                }
            }
        }
    }

    /// A rounded feature: `on_segment` and `on_feature` are the closest points on the edge and on the
    /// feature's core, `radius` the rounding.
    #[inline(always)]
    fn consider(
        &self,
        on_segment: Vec3,
        on_feature: Vec3,
        radius: f32,
        a: Vec3,
        b: Vec3,
        reach: f32,
        best: &mut SegmentHit,
    ) {
        let d = on_segment - on_feature;
        let len = length(d);
        let distance = len - radius;
        // Inside the rounding the direction from the core still says which way is out; only an edge
        // through the core itself has none.
        if len > 1.0e-6 && distance < reach && distance < best.distance {
            let n = (1.0 / len) * d;
            *best = SegmentHit {
                s: segment_fraction(a, b, on_segment),
                normal: n,
                point: mul_add(on_feature, radius, n),
                distance,
            };
        }
    }

    fn segment_against_triangle(
        &self,
        p: Vec3,
        q: Vec3,
        r: Vec3,
        a: Vec3,
        b: Vec3,
        reach: f32,
        best: &mut SegmentHit,
    ) {
        let triangle = (p, q, r);
        // SAFETY: mirrors Swift's `withUnsafePointer(to: &triangle).withMemoryRebound(to: Vec3.self,
        // capacity: 3)`. `triangle` is a pinned local for the whole call, and a tuple of three
        // `#[repr(C)]` three-float vectors lays them out contiguously with no padding, so the
        // rebound exposes exactly the three points Swift's pointer did. GJK only reads them.
        let points = unsafe {
            core::slice::from_raw_parts(core::ptr::from_ref(&triangle).cast::<Vec3>(), 3)
        };
        let mut segment = (a, b);
        self.segment_against_points(
            points,
            3,
            &mut segment,
            Transform::IDENTITY,
            a,
            b,
            reach,
            best,
        );
    }

    /// GJK between a point cloud (in the frame placed by `transform`) and a segment given in that
    /// same frame; the hit is stored in the shape frame.
    fn segment_against_points(
        &self,
        points: &[Vec3],
        count: i32,
        segment: &mut (Vec3, Vec3),
        transform: Transform,
        a: Vec3,
        b: Vec3,
        reach: f32,
        best: &mut SegmentHit,
    ) {
        // SAFETY: mirrors Swift's `withUnsafePointer(to: &segment).withMemoryRebound(to: Vec3.self,
        // capacity: 2)`. `segment` is borrowed for the whole `shape_distance` call, and a tuple of
        // two `#[repr(C)]` three-float vectors lays them out contiguously with no padding, so the
        // rebound exposes exactly the two points Swift's pointer did. GJK only reads them.
        let segment_points = unsafe {
            core::slice::from_raw_parts(core::ptr::from_ref::<(Vec3, Vec3)>(segment).cast::<Vec3>(), 2)
        };
        let mut input = DistanceInput::default();
        input.proxy_a = ShapeProxy::new(Some(points.as_ptr()), count, 0.0);
        input.proxy_b = ShapeProxy::new(Some(segment_points.as_ptr()), 2, 0.0);
        input.transform = Transform::IDENTITY;
        input.use_radii = false;
        let mut cache = SimplexCache::default();
        let output = shape_distance(&input, &mut cache, None);
        // Zero distance: the edge is already through the feature; the vertices own that case.
        if output.distance > 1.0e-6 && output.distance < reach && output.distance < best.distance {
            let on_segment = transform_point(transform, output.point_b);
            *best = SegmentHit {
                s: segment_fraction(a, b, on_segment),
                normal: rotate_vector(transform.q, output.normal),
                point: transform_point(transform, output.point_a),
                distance: output.distance,
            };
        }
    }

    /// Detect edge planes against one shape. Called from the rigid detection for each candidate.
    pub(crate) fn detect_edge_planes(
        &self,
        record: &mut SoftBodyRecord,
        geometry: &ShapeGeometry,
        transform: Transform,
        shape_bounds: AABB,
        edge_bounds: &[AABB],
        reach: f32,
        collider_index: i32,
        best_distance: &mut [f32],
        best_plane: &mut [Option<SoftEdgePlane>],
    ) -> bool {
        let mut used = false;
        let radius = record.vertex_radius;
        for (e, edge) in record.mesh_edges.iter().enumerate() {
            if !aabb_overlaps(edge_bounds[e], shape_bounds) {
                continue;
            }
            let edge = *edge;
            let i0 = edge.x as usize;
            let i1 = edge.y as usize;
            if !(record.inv_masses[i0] > 0.0 || record.inv_masses[i1] > 0.0) {
                continue;
            }
            // Both ends already on this shape with nearly the same normal: the surface between them
            // is flat at this scale and nothing fits between the vertices.
            if record.collider_indices[i0] == collider_index
                && record.collider_indices[i1] == collider_index
                && dot(record.plane_normals[i0], record.plane_normals[i1]) > FLAT_COSINE
            {
                continue;
            }
            let a = inv_transform_point(transform, record.positions[i0]);
            let b = inv_transform_point(transform, record.positions[i1]);
            let local_bounds = aabb_transform_inverse(transform, edge_bounds[e]);
            let mut hit = SegmentHit::default();
            self.closest_segment_feature(
                geometry,
                Transform::IDENTITY,
                a,
                b,
                local_bounds,
                reach + radius,
                &mut hit,
            );
            let separation = hit.distance - radius;
            if hit.distance < f32::MAX
                && hit.s > INTERIOR_FRACTION
                && hit.s < 1.0 - INTERIOR_FRACTION
                && separation < best_distance[e]
            {
                best_distance[e] = separation;
                let n = rotate_vector(transform.q, hit.normal);
                let point = transform_point(transform, hit.point);
                best_plane[e] = Some(SoftEdgePlane {
                    v0: i0 as i32,
                    v1: i1 as i32,
                    s: hit.s,
                    normal: n,
                    offset: dot(n, point) + radius,
                    collider: collider_index,
                });
                used = true;
            }
        }
        used
    }
}

#[inline(always)]
fn segment_fraction(a: Vec3, b: Vec3, p: Vec3) -> f32 {
    let d = b - a;
    let length_squared = dot(d, d);
    if length_squared > 0.0 {
        clamp_float(dot(p - a, d) / length_squared, 0.0, 1.0)
    } else {
        0.0
    }
}
