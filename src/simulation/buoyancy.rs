//! Buoyancy and fluid drag. The submerged part of each shape is computed exactly for spheres,
//! hulls, and closed meshes (clipped against the fluid surface) and by quadrature for capsules.
//!
//! Translated from `Sources/Natsel/Simulation/Buoyancy.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::world_queries`: the `World` method
//!   `body_transform_quick(body_id: i32) -> WorldTransform` (Swift's `WorldQueries.swift`).
//! - `crate::simulation::world_runtime::World::shape_world_aabb` lands in `world_bounds.rs`
//!   (already translated).
//!
//! Determinism notes: every summation order, clamp, epsilon, and literal from the Swift original is
//! kept verbatim, including `4 / 3 * pi`, the Gauss-Legendre node/weight tables, the break-point
//! sort, and the per-interval piece subdivision. The Swift operand association and the
//! `mulAdd`/`mulSub` fused shapes are reproduced through the shared `crate::math::functions`
//! helpers so the scalar path agrees with any SIMD lane.

use crate::base::constants as b3;
use crate::collision::compound::compound::CompoundChildGeometry;
use crate::collision::hull::Hull;
use crate::collision::mesh::overlap::query_mesh;
use crate::math::functions::*;
use crate::math::types::{AABB, Plane, Pos, Transform, Vec3};
use crate::math::world_aabb::WorldAABB;
use crate::types::body_def::BodyType;
use crate::types::geometry::Mesh;
use crate::types::ids::{BodyID, ShapeID};
use crate::types::shape_def::QueryFilter;

use crate::simulation::shape_runtime::{ShapeGeometry, compute_shape_aabb, should_query_collide};
use crate::simulation::world_runtime::{Body, SetType, World};

/// Submerged volume of a body below a fluid surface.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SubmergedVolume {
    /// Volume of all shapes.
    pub total_volume: f32,
    /// Volume below the surface.
    pub submerged_volume: f32,
    /// Centroid of the submerged volume, in world space.
    pub center_of_buoyancy: Pos,
}

/// Accumulates volume and first moment.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct VolumeSum {
    total: f32,
    submerged: f32,
    moment: Vec3,
}

impl VolumeSum {
    #[inline(always)]
    fn add(&mut self, total: f32, submerged: f32, centroid: Vec3) {
        self.total += total;
        self.submerged += submerged;
        self.moment = mul_add(self.moment, submerged, centroid);
    }
}

impl World {
    /// Volume of the body below the plane
    /// `dot(surfaceNormal, x) = dot(surfaceNormal, surfacePosition)`. The normal points out of the
    /// fluid.
    pub fn body_submerged_volume(
        &self,
        body_id: BodyID,
        surface_position: Pos,
        surface_normal: Vec3,
    ) -> SubmergedVolume {
        let mut result = SubmergedVolume::default();
        // Swift's `guard lengthSquared(surfaceNormal) > 0` keeps NaN out of the fluid; the negated
        // form must be `!(... > 0.0)`, not `... <= 0.0`, so a NaN normal still returns early.
        if !self.is_valid_body(body_id) || !(length_squared(surface_normal) > 0.0) {
            return result;
        }
        let record = self.body(body_id);
        let transform = self.body_transform_quick(self.body_index(body_id));
        let n = normalize(surface_normal);
        // Work relative to the body origin to keep precision far from the world origin.
        let origin = transform.p;
        let plane = Plane::new(n, dot(n, sub_pos(surface_position, origin)));
        let local = Transform::new(Vec3::default(), transform.q);

        let mut sum = VolumeSum::default();
        let mut shape_id = record.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = &self.shapes[shape_id as usize];
            if shape.sensor_index == b3::NULL_INDEX {
                self.accumulate_submerged(&shape.geometry, local, plane, &mut sum);
            }
            shape_id = shape.next_shape_id;
        }

        result.total_volume = sum.total;
        result.submerged_volume = sum.submerged;
        result.center_of_buoyancy = if sum.submerged > 0.0 {
            offset_pos(origin, (1.0 / sum.submerged) * sum.moment)
        } else {
            transform.p
        };
        result
    }

    /// Apply buoyancy and drag for one time step (Jolt's `Body::ApplyBuoyancyImpulse`).
    /// - buoyancy: fluid density relative to the body's average density. 1 is neutral, more floats.
    /// - linearDrag: drag coefficient (about 0.5 for a sphere).
    /// - angularDrag: fraction of angular velocity removed per second when fully submerged.
    /// - fluidVelocity: velocity of the fluid, for currents.
    /// Returns false when the body is not in the fluid.
    ///
    /// Swift defaulted `linearDrag` to `0.5`, `angularDrag` to `0.01` and `fluidVelocity` to
    /// `Vec3()`; ported callers pass them explicitly.
    pub fn apply_buoyancy_impulse(
        &mut self,
        body_id: BodyID,
        surface_position: Pos,
        surface_normal: Vec3,
        buoyancy: f32,
        linear_drag: f32,
        angular_drag: f32,
        fluid_velocity: Vec3,
        time_step: f32,
    ) -> bool {
        // Every Swift `guard x > y` is negated as `!(x > y)` rather than `x <= y`, so a NaN
        // operand fails the guard exactly as in Swift instead of falling through.
        if !self.is_valid_body(body_id) || !(time_step > 0.0) {
            return false;
        }
        let record = self.body(body_id);
        if record.type_ != BodyType::DynamicBody
            || record.set_index == SetType::DisabledSet.raw_value()
        {
            return false;
        }

        let volume = self.body_submerged_volume(body_id, surface_position, surface_normal);
        if !(volume.submerged_volume > 1.0e-9 * b3::length_units_per_meter())
            || !(volume.total_volume > 0.0)
        {
            return false;
        }

        let sim = self.body_sim(&record);
        if !(sim.inv_mass > 0.0) {
            return false;
        }
        let mass = 1.0 / sim.inv_mass;
        let fluid_density = buoyancy * mass / volume.total_volume;
        let fraction = min_float(1.0, volume.submerged_volume / volume.total_volume);

        let mut linear_velocity = Vec3::default();
        let mut angular_velocity = Vec3::default();
        if let Some(state) = self.body_state(&record) {
            linear_velocity = state.linear_velocity;
            angular_velocity = state.angular_velocity;
        }

        // Archimedes: the displaced fluid's weight, acting at the center of buoyancy.
        let mut impulse = (-fluid_density * volume.submerged_volume * time_step) * self.gravity;

        // Quadratic drag on the velocity of the center of buoyancy relative to the fluid.
        let r = sub_pos(volume.center_of_buoyancy, sim.center);
        let relative = linear_velocity + cross(angular_velocity, r) - fluid_velocity;
        let speed = length(relative);
        if speed > 1.0e-6 {
            // Frontal area of the local bounds seen along the flow.
            let bounds = self.body_local_bounds(&record);
            let size = bounds.upper_bound - bounds.lower_bound;
            let local_flow = inv_rotate_vector(sim.transform.q, relative);
            let area = (abs_float(local_flow.x) * size.y * size.z
                + abs_float(local_flow.y) * size.x * size.z
                + abs_float(local_flow.z) * size.x * size.y)
                / speed;
            let mut drag = (-0.5 * fluid_density * linear_drag * area * fraction * time_step
                * speed)
                * relative;
            // Drag can stop the body relative to the fluid but not reverse it.
            let change = sim.inv_mass * length(drag);
            if change > speed {
                drag = (speed / change) * drag;
            }
            impulse += drag;
        }

        self.apply_body_linear_impulse(body_id, impulse, volume.center_of_buoyancy, true);

        // Angular drag removes a fraction of the spin, bounded so it never reverses it.
        let damping = min_float(1.0, angular_drag * fraction * time_step);
        if damping > 0.0 && length_squared(angular_velocity) > 0.0 {
            let inertia = invert_matrix(sim.inv_inertia_world);
            if is_valid_matrix3(inertia) {
                self.apply_body_angular_impulse(
                    body_id,
                    mul_mv(inertia, -damping * angular_velocity),
                    true,
                );
            }
        }
        true
    }

    /// Apply buoyancy to every dynamic body overlapping a box of fluid whose surface is the box top
    /// (along `surfaceNormal`). Returns the number of bodies affected.
    ///
    /// Swift defaulted `surfaceNormal` to `Vec3(0, 1, 0)`, `linearDrag` to `0.5`, `angularDrag` to
    /// `0.01`, `fluidVelocity` to `Vec3()` and `filter` to `.defaultFilter`; ported callers pass
    /// them explicitly.
    pub fn apply_buoyancy(
        &mut self,
        volume: WorldAABB,
        surface_normal: Vec3,
        buoyancy: f32,
        linear_drag: f32,
        angular_drag: f32,
        fluid_velocity: Vec3,
        time_step: f32,
        filter: QueryFilter,
    ) -> i32 {
        let n = normalize(surface_normal);
        let corner = Pos::new(
            if n.x >= 0.0 {
                volume.upper_bound.x
            } else {
                volume.lower_bound.x
            },
            if n.y >= 0.0 {
                volume.upper_bound.y
            } else {
                volume.lower_bound.y
            },
            if n.z >= 0.0 {
                volume.upper_bound.z
            } else {
                volume.lower_bound.z
            },
        );
        // Swift collected the body indices in a `Set<Int32>` and iterated `seen.sorted()`. A vector
        // sorted and deduplicated before the loop visits bodies yields exactly the same ascending
        // order of distinct indices, so the impulse application order is unchanged.
        let mut seen: Vec<i32> = Vec::new();
        let tree = &self.broad_phase.trees[BodyType::DynamicBody as usize];
        tree.query(volume.conservative_bounds(), filter.mask_bits, false, |_, data| {
            let shape = &self.shapes[data as usize];
            if should_query_collide(&shape.filter, &filter) {
                let id = ShapeID::new(shape.id + 1, self.world_id, shape.generation);
                if volume.overlaps(&self.shape_world_aabb(id)) {
                    seen.push(shape.body_id);
                }
            }
            true
        });

        let mut count = 0;
        seen.sort();
        seen.dedup();
        for body_index in seen {
            let id = self.make_body_id(body_index);
            if self.apply_buoyancy_impulse(
                id,
                corner,
                n,
                buoyancy,
                linear_drag,
                angular_drag,
                fluid_velocity,
                time_step,
            ) {
                count += 1;
            }
        }
        count
    }

    fn body_local_bounds(&self, record: &Body) -> AABB {
        let mut bounds = AABB::new(
            Vec3::new(f32::MAX, f32::MAX, f32::MAX),
            Vec3::new(-f32::MAX, -f32::MAX, -f32::MAX),
        );
        let mut shape_id = record.head_shape_id;
        while shape_id != b3::NULL_INDEX {
            let shape = &self.shapes[shape_id as usize];
            bounds = aabb_union(bounds, compute_shape_aabb(shape, Transform::IDENTITY));
            shape_id = shape.next_shape_id;
        }
        bounds
    }

    // MARK: Geometry

    fn accumulate_submerged(
        &self,
        geometry: &ShapeGeometry,
        xf: Transform,
        plane: Plane,
        sum: &mut VolumeSum,
    ) {
        match geometry {
            ShapeGeometry::Sphere(sphere) => {
                submerged_sphere(transform_point(xf, sphere.center), sphere.radius, plane, sum)
            }
            ShapeGeometry::Capsule(capsule) => submerged_capsule(
                transform_point(xf, capsule.center1),
                transform_point(xf, capsule.center2),
                capsule.radius,
                plane,
                sum,
            ),
            ShapeGeometry::Hull(hull) => submerged_hull(hull, xf, plane, sum),
            ShapeGeometry::Mesh(mesh) => submerged_mesh(mesh, xf, plane, sum),
            ShapeGeometry::Compound(compound) => {
                for i in 0..compound.child_count() {
                    let child = compound.child(i);
                    let child_xf = mul_transforms(xf, child.transform);
                    match &child.geometry {
                        CompoundChildGeometry::Sphere(sphere) => self.accumulate_submerged(
                            &ShapeGeometry::Sphere(*sphere),
                            child_xf,
                            plane,
                            sum,
                        ),
                        CompoundChildGeometry::Capsule(capsule) => self.accumulate_submerged(
                            &ShapeGeometry::Capsule(*capsule),
                            child_xf,
                            plane,
                            sum,
                        ),
                        CompoundChildGeometry::Hull(hull) => self.accumulate_submerged(
                            &ShapeGeometry::Hull(hull.clone()),
                            child_xf,
                            plane,
                            sum,
                        ),
                        CompoundChildGeometry::Mesh(mesh) => self.accumulate_submerged(
                            &ShapeGeometry::Mesh(mesh.clone()),
                            child_xf,
                            plane,
                            sum,
                        ),
                    }
                }
            }
            ShapeGeometry::HeightField(_) => {
                // Height fields are open surfaces without volume.
            }
        }
    }
}

// MARK: - Sphere

/// Spherical cap below the plane.
fn submerged_sphere(center: Vec3, radius: f32, plane: Plane, sum: &mut VolumeSum) {
    let total = 4.0 / 3.0 * b3::PI * radius * radius * radius;
    let height = clamp_float(
        radius - (dot(plane.normal, center) - plane.offset),
        0.0,
        2.0 * radius,
    );
    if !(height > 0.0) {
        sum.add(total, 0.0, center);
        return;
    }
    let volume = b3::PI * height * height * (3.0 * radius - height) / 3.0;
    // Centroid of a cap of height h, measured from the sphere center.
    let depth = 3.0 * (2.0 * radius - height) * (2.0 * radius - height)
        / (4.0 * (3.0 * radius - height));
    sum.add(total, volume, mul_sub(center, depth, plane.normal));
}

// MARK: - Capsule

/// Circular segment beyond a chord at signed distance `d` from the center of a disk of radius `r`:
/// area and centroid distance from the center.
#[inline(always)]
fn circular_segment(r: f32, d: f32) -> (f32, f32) {
    if d >= r {
        return (0.0, 0.0);
    }
    if d <= -r {
        return (b3::PI * r * r, 0.0);
    }
    let half = (r * r - d * d).sqrt();
    let angle = atan2_approx(half, d);
    let area = r * r * angle - d * half;
    if !(area > 0.0) {
        return (0.0, 0.0);
    }
    (area, 2.0 * half * half * half / (3.0 * area))
}

/// Integrate the disks perpendicular to the axis with Simpson's rule.
fn submerged_capsule(p1: Vec3, p2: Vec3, radius: f32, plane: Plane, sum: &mut VolumeSum) {
    let (axis_length, axis) = get_length_and_normalize(p2 - p1);
    let total = b3::PI * radius * radius * (axis_length + 4.0 / 3.0 * radius);
    let center = 0.5 * (p1 + p2);

    // Quick outs against the bounding sphere segment. Swift used the stdlib `min`/`max` here, not
    // `minFloat`/`maxFloat`; those are `y < x ? y : x` and `x < y ? y : x`, which propagate a NaN
    // first operand where `min_float`/`max_float` would hide it, so the comparison form is spelled
    // out instead of calling the helpers.
    let s1 = dot(plane.normal, p1) - plane.offset;
    let s2 = dot(plane.normal, p2) - plane.offset;
    let min_s = if s2 < s1 { s2 } else { s1 };
    if min_s >= radius {
        sum.add(total, 0.0, center);
        return;
    }
    let max_s = if s1 < s2 { s2 } else { s1 };
    if max_s <= -radius {
        sum.add(total, total, center);
        return;
    }
    if axis_length < 1.0e-6 {
        submerged_sphere(center, radius, plane, sum);
        return;
    }

    // In-disk direction of steepest descent into the fluid.
    let m = plane.normal - dot(plane.normal, axis) * axis;
    let m_length = length(m);
    let half = 0.5 * axis_length + radius;

    // Submerged disk area and centroid at axial position t.
    let slice = |t: f32| -> (f32, Vec3) {
        let beyond = abs_float(t) - 0.5 * axis_length;
        let r = if beyond > 0.0 {
            max_float(0.0, radius * radius - beyond * beyond).sqrt()
        } else {
            radius
        };
        let c = mul_add(center, t, axis);
        let s = dot(plane.normal, c) - plane.offset;
        if m_length < 1.0e-6 {
            return (if s < 0.0 { b3::PI * r * r } else { 0.0 }, c);
        }
        let segment = circular_segment(r, s / m_length);
        (segment.0, mul_sub(c, segment.1 / m_length, m))
    };

    // The integrand is smooth between the cylinder/cap joints and the points where the disks
    // start or stop touching the surface. Integrate each smooth piece with Gauss-Legendre.
    let mut breaks: Vec<f32> = vec![-half, -0.5 * axis_length, 0.5 * axis_length, half];
    let slope = dot(plane.normal, axis);
    if abs_float(slope) > 1.0e-6 {
        let s0 = dot(plane.normal, center) - plane.offset;
        for edge in [-radius * m_length, 0.0, radius * m_length] {
            let t = (edge - s0) / slope;
            if t > -half && t < half {
                breaks.push(t);
            }
        }
    }
    // Swift's `Array<Float>.sort()` is a total order on the finite break points produced above
    // (`|slope| > 1.0e-6` and a finite `s0` keep every `t` finite); ties compare equal in both
    // languages and equal neighbours are skipped by the width epsilon below, so the interval
    // sequence is identical.
    breaks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(core::cmp::Ordering::Equal));

    let nodes: [f32; 5] = [-0.906179846, -0.538469310, 0.0, 0.538469310, 0.906179846];
    let weights: [f32; 5] = [0.236926885, 0.478628670, 0.568888889, 0.478628670, 0.236926885];
    let mut volume: f32 = 0.0;
    let mut moment = Vec3::default();
    for k in 0..(breaks.len() - 1) {
        let a = breaks[k];
        let b = breaks[k + 1];
        if !(b - a > 1.0e-9) {
            continue;
        }
        let pieces: i32 = 4;
        let width = (b - a) / pieces as f32;
        for p in 0..pieces {
            let mid = a + (p as f32 + 0.5) * width;
            for g in 0..5 {
                let t = mid + 0.5 * width * nodes[g];
                let value = slice(t);
                let w = 0.5 * width * weights[g] * value.0;
                volume += w;
                moment = mul_add(moment, w, value.1);
            }
        }
    }
    sum.add(
        total,
        volume,
        if volume > 0.0 {
            (1.0 / volume) * moment
        } else {
            center
        },
    );
}

// MARK: - Polyhedra

/// Clip a convex polygon to the part below the plane and add the tetrahedra it forms with a point
/// on the plane. The cap that closes the clipped solid lies in the plane, so it adds nothing.
fn add_clipped_polygon(
    polygon: &[Vec3],
    plane: Plane,
    apex: Vec3,
    volume: &mut f32,
    moment: &mut Vec3,
) {
    let mut clipped: Vec<Vec3> = Vec::with_capacity(polygon.len() + 2);
    let count = polygon.len();
    for i in 0..count {
        let a = polygon[i];
        let b = polygon[(i + 1) % count];
        let sa = dot(plane.normal, a) - plane.offset;
        let sb = dot(plane.normal, b) - plane.offset;
        if sa <= 0.0 {
            clipped.push(a);
        }
        if (sa < 0.0) != (sb < 0.0) && sa != sb {
            let t = sa / (sa - sb);
            clipped.push(mul_add(a, t, b - a));
        }
    }
    if clipped.len() < 3 {
        return;
    }
    let p0 = clipped[0];
    for i in 1..(clipped.len() - 1) {
        let p1 = clipped[i];
        let p2 = clipped[i + 1];
        let v = dot(p0 - apex, cross(p1 - apex, p2 - apex)) / 6.0;
        *volume += v;
        *moment = mul_add(*moment, v * 0.25, apex + p0 + p1 + p2);
    }
}

fn submerged_hull(hull: &Hull, xf: Transform, plane: Plane, sum: &mut VolumeSum) {
    // Swift guarded `hull.points`, `hull.edges`, `hull.faces` for nil; the Rust `Hull` always owns
    // its baked allocation, so those slices exist for every constructed hull and no early return is
    // reachable. Deliberately no emptiness test is added: Swift fell through for a zero vertex or
    // face count and reached the `lowest >= 0` quick out, which the code below reproduces.
    let points = hull.points();
    let edges = hull.edges();
    let faces = hull.faces();
    let total = hull.volume();
    let center = transform_point(xf, hull.center());

    // Whole hull on one side?
    let mut lowest: f32 = f32::MAX;
    let mut highest: f32 = -f32::MAX;
    let vertex_count = hull.vertex_count() as usize;
    let mut world = vec![Vec3::default(); vertex_count];
    for i in 0..vertex_count {
        world[i] = transform_point(xf, points[i]);
        let s = dot(plane.normal, world[i]) - plane.offset;
        lowest = min_float(lowest, s);
        highest = max_float(highest, s);
    }
    if lowest >= 0.0 {
        sum.add(total, 0.0, center);
        return;
    }
    if highest <= 0.0 {
        sum.add(total, total, center);
        return;
    }

    let apex = mul_sub(center, dot(plane.normal, center) - plane.offset, plane.normal);
    let mut volume: f32 = 0.0;
    let mut moment = Vec3::default();
    let mut polygon: Vec<Vec3> = Vec::new();
    for f in 0..hull.face_count() as usize {
        // Swift's `removeAll(keepingCapacity: true)`; `clear` keeps the allocation.
        polygon.clear();
        let start = faces[f].edge as usize;
        let mut e = start;
        loop {
            polygon.push(world[edges[e].origin as usize]);
            e = edges[e].next as usize;
            if e == start || polygon.len() > vertex_count {
                break;
            }
        }
        add_clipped_polygon(&polygon, plane, apex, &mut volume, &mut moment);
    }
    sum.add(
        total,
        max_float(0.0, volume),
        if volume > 0.0 {
            (1.0 / volume) * moment
        } else {
            center
        },
    );
}

/// Closed meshes only: an open mesh has no inside.
fn submerged_mesh(mesh: &Mesh, xf: Transform, plane: Plane, sum: &mut VolumeSum) {
    let Some(data) = mesh.data.as_ref() else { return };
    let bounds = data.bounds();
    let reference = transform_point(xf, mesh.scale * aabb_center(bounds));
    let apex = mul_sub(reference, dot(plane.normal, reference) - plane.offset, plane.normal);
    let mut total: f32 = 0.0;
    let mut volume: f32 = 0.0;
    let mut moment = Vec3::default();
    let mut full_moment = Vec3::default();
    query_mesh(
        mesh,
        normalize_scaled_aabb(AABB::new(
            mesh.scale * bounds.lower_bound - Vec3::ONE,
            mesh.scale * bounds.upper_bound + Vec3::ONE,
        )),
        |a, b, c, _| {
            let wa = transform_point(xf, a);
            let wb = transform_point(xf, b);
            let wc = transform_point(xf, c);
            let v = dot(wa - reference, cross(wb - reference, wc - reference)) / 6.0;
            total += v;
            full_moment = mul_add(full_moment, v * 0.25, reference + wa + wb + wc);
            add_clipped_polygon(&[wa, wb, wc], plane, apex, &mut volume, &mut moment);
            true
        },
    );
    if !(total > 0.0) {
        return;
    }
    sum.add(
        total,
        max_float(0.0, volume),
        if volume > 0.0 {
            (1.0 / volume) * moment
        } else {
            (1.0 / total) * full_moment
        },
    );
}

/// Order the corners after a scale that may be negative.
///
/// Swift exposed this as `private extension AABB { var normalized: AABB }`; adding an inherent
/// method to the crate-wide `AABB` from a leaf module would put the name in scope for every other
/// module, so it stays a file-private helper.
#[inline]
fn normalize_scaled_aabb(a: AABB) -> AABB {
    AABB::new(
        min_vec3(a.lower_bound, a.upper_bound),
        max_vec3(a.lower_bound, a.upper_bound),
    )
}

impl World {
    /// Convenience overload for a fluid volume described by single precision world bounds.
    ///
    /// Swift overloaded `applyBuoyancy(in:)` on the bounds type; Rust has no overloading, so the
    /// `AABB` form keeps the type in its name.
    pub fn apply_buoyancy_in_aabb(
        &mut self,
        volume: AABB,
        surface_normal: Vec3,
        buoyancy: f32,
        linear_drag: f32,
        angular_drag: f32,
        fluid_velocity: Vec3,
        time_step: f32,
        filter: QueryFilter,
    ) -> i32 {
        self.apply_buoyancy(
            WorldAABB::new(Pos::from_vec3(volume.lower_bound), Pos::from_vec3(volume.upper_bound)),
            surface_normal,
            buoyancy,
            linear_drag,
            angular_drag,
            fluid_velocity,
            time_step,
            filter,
        )
    }
}
