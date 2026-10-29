//! Mesh queries: overlap, ray casts, shape casts, and mover collisions.

use crate::base::constants as b3;
use crate::collision::aabb::{bounds_overlap, bounds_triangle_overlap};
use crate::collision::internal_math::signed_volume;
use crate::collision::proxy::{compute_proxy_aabb, make_local_proxy};
use crate::math::functions::*;
use crate::math::types::{AABB, Vec3};
use crate::types::geometry::Mesh;
use crate::types::queries::*;

use super::access::{visit_mesh_candidates, MeshQueryBounds};

/// Do a mesh and a proxy overlap?
pub fn overlap_mesh(mesh: &Mesh, transform: crate::math::types::Transform, proxy: ShapeProxy) -> bool {
    let Some(data) = mesh.data.as_ref() else { return false };
    if proxy.count <= 0 {
        return false;
    }
    let mut local_points = Vec::new();
    let local_proxy = make_local_proxy(proxy, transform, &mut local_points);
    let bounds = MeshQueryBounds::new(compute_proxy_aabb(local_proxy), mesh.scale);
    let vertices = data.vertices();
    let mut touching = false;

    visit_mesh_candidates(data, bounds, |_, triangle| {
        let a = vertices[triangle.index1 as usize];
        let b = vertices[triangle.index2 as usize];
        let c = vertices[triangle.index3 as usize];
        if bounds_triangle_overlap(bounds.center(), bounds.extent(), a, b, c) {
            let scaled = [mesh.scale * a, mesh.scale * b, mesh.scale * c];
            let mut input = DistanceInput {
                proxy_a: ShapeProxy::new(Some(scaled.as_ptr()), 3, 0.0),
                proxy_b: local_proxy,
                transform: crate::math::types::Transform::IDENTITY,
                use_radii: true,
            };
            input.proxy_b = local_proxy;
            let mut cache = SimplexCache::default();
            if crate::collision::distance::shape_distance(input, &mut cache).distance < 0.1 * b3::linear_slop() {
                touching = true;
            }
        }
        !touching
    });
    touching
}

/// Query scaled mesh triangles overlapping a local-space box. Return false to stop.
pub fn query_mesh<F: FnMut(Vec3, Vec3, Vec3, i32) -> bool>(mesh: &Mesh, bounds: AABB, mut callback: F) {
    let Some(data) = mesh.data.as_ref() else { return };
    let local_bounds = MeshQueryBounds::new(bounds, mesh.scale);
    let vertices = data.vertices();
    let ccw = mesh.scale.x * mesh.scale.y * mesh.scale.z > 0.0;
    visit_mesh_candidates(data, local_bounds, |index, triangle| {
        let a = vertices[triangle.index1 as usize];
        let b = vertices[triangle.index2 as usize];
        let c = vertices[triangle.index3 as usize];
        if bounds_triangle_overlap(local_bounds.center(), local_bounds.extent(), a, b, c) {
            let _ = (ccw, index);
            return callback(
                mesh.scale * a,
                mesh.scale * if ccw { b } else { c },
                mesh.scale * if ccw { c } else { b },
                index as i32,
            );
        }
        true
    });
}

/// Triangle intersection fraction. One is the miss sentinel and endpoint.
pub fn intersect_mesh_ray_triangle(start: Vec3, delta: Vec3, a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let edge1 = c - b;
    let edge2 = a - c;
    let edge3 = b - a;
    let middle1 = 0.5 * (b + c);
    let middle2 = 0.5 * (c + a);
    let middle3 = 0.5 * (a + b);
    let normal1 = cross(edge1, middle1 - start);
    let normal2 = cross(edge2, middle2 - start);
    let normal3 = cross(edge3, middle3 - start);
    if dot(normal1, delta) < 0.0 || dot(normal2, delta) < 0.0 || dot(normal3, delta) < 0.0 {
        return 1.0;
    }
    let normal = cross(b - a, c - a);
    let denominator = dot(normal, delta);
    if denominator >= 0.0 {
        return 1.0;
    }
    let fraction = dot(normal, a - start) / denominator;
    if fraction <= 0.0 {
        return 1.0;
    }
    min_float(fraction, 1.0)
}

/// Ray cast against a mesh.
pub fn ray_cast_mesh(mesh: &Mesh, input: RayCastInput) -> CastOutput {
    let mut best = CastOutput::default();
    best.fraction = input.max_fraction;
    best.triangle_index = b3::NULL_INDEX;
    let Some(data) = mesh.data.as_ref() else { return best };

    let scale = mesh.scale;
    let inverse_scale = Vec3::new(1.0 / scale.x, 1.0 / scale.y, 1.0 / scale.z);
    let ccw = scale.x * scale.y * scale.z > 0.0;
    let start = inverse_scale * input.origin;
    let delta = inverse_scale * input.translation;
    let mut end = start + input.max_fraction * delta;
    let mut ray_min = min_vec3(start, end);
    let mut ray_max = max_vec3(start, end);

    let nodes = data.nodes();
    let triangles = data.triangles();
    let vertices = data.vertices();
    let materials = data.material_indices();

    let mut stack: Vec<usize> = Vec::new();
    let mut index = 0usize;
    loop {
        let node = nodes[index];
        let separation = max_vec3(ray_min - node.upper_bound, node.lower_bound - ray_max);
        let overlaps = separation.x <= 0.0 && separation.y <= 0.0 && separation.z <= 0.0;
        if overlaps
            && crate::collision::dynamic_tree::test_bounds_ray_overlap(
                node.lower_bound,
                node.upper_bound,
                start,
                delta,
            )
        {
            if node.is_leaf() {
                let offset = node.triangle_offset as usize;
                for triangle_index in offset..offset + node.leaf_triangle_count() {
                    let triangle = triangles[triangle_index];
                    let a = scale * vertices[triangle.index1 as usize];
                    let b = scale * vertices[if ccw { triangle.index2 } else { triangle.index3 } as usize];
                    let c = scale * vertices[if ccw { triangle.index3 } else { triangle.index2 } as usize];
                    let alpha = intersect_mesh_ray_triangle(input.origin, input.translation, a, b, c);
                    if alpha < best.fraction {
                        best.normal = normalize(cross(b - a, c - a));
                        best.point = input.origin + alpha * input.translation;
                        best.fraction = alpha;
                        best.triangle_index = triangle_index as i32;
                        best.material_index = materials[triangle_index] as i32;
                        best.hit = true;
                        end = start + alpha * delta;
                        ray_min = min_vec3(start, end);
                        ray_max = max_vec3(start, end);
                    }
                }
            } else {
                if get_by_index(delta, node.axis()) > 0.0 {
                    stack.push(index + node.child_offset());
                    index += 1;
                } else {
                    stack.push(index + 1);
                    index += node.child_offset();
                }
                continue;
            }
        }
        match stack.pop() {
            Some(next) => index = next,
            None => break,
        }
    }
    best
}

/// Shape cast against a mesh.
pub fn shape_cast_mesh(mesh: &Mesh, input: ShapeCastInput) -> CastOutput {
    let mut best = CastOutput::default();
    best.fraction = input.max_fraction;
    best.triangle_index = b3::NULL_INDEX;
    let Some(data) = mesh.data.as_ref() else { return best };

    let shape_bounds = crate::math::functions::make_aabb(input.proxy.points(), input.proxy.radius);
    let center = aabb_center(shape_bounds);
    let extent = aabb_extents(shape_bounds);
    let mut end = center + input.max_fraction * input.translation;
    let mut ray_min = min_vec3(center, end);
    let mut ray_max = max_vec3(center, end);

    let scale = mesh.scale;
    let inverse = Vec3::new(1.0 / scale.x, 1.0 / scale.y, 1.0 / scale.z);
    let ccw = scale.x * scale.y * scale.z > 0.0;
    let local_extent = abs_vec3(inverse) * extent;
    let nodes = data.nodes();
    let triangles = data.triangles();
    let vertices = data.vertices();
    let materials = data.material_indices();

    let mut stack: Vec<usize> = Vec::new();
    let mut index = 0usize;
    loop {
        let node = nodes[index];
        let node_min = node.lower_bound - local_extent;
        let node_max = node.upper_bound + local_extent;
        if bounds_overlap(node_min, node_max, ray_min, ray_max) {
            if node.is_leaf() {
                let offset = node.triangle_offset as usize;
                for triangle_index in offset..offset + node.leaf_triangle_count() {
                    let triangle = triangles[triangle_index];
                    let a = scale * vertices[triangle.index1 as usize];
                    let b = scale * vertices[if ccw { triangle.index2 } else { triangle.index3 } as usize];
                    let c = scale * vertices[if ccw { triangle.index3 } else { triangle.index2 } as usize];
                    let triangle_min = min_vec3(a, min_vec3(b, c)) - extent;
                    let triangle_max = max_vec3(a, max_vec3(b, c)) + extent;
                    if !bounds_overlap(triangle_min, triangle_max, ray_min, ray_max) {
                        continue;
                    }
                    if signed_volume(a, b, c, center) < 0.0 {
                        continue;
                    }
                    let triangle_points = [Vec3::ZERO, b - a, c - a];
                    let output = crate::collision::distance::shape_cast(ShapeCastPairInput {
                        proxy_a: ShapeProxy::new(Some(triangle_points.as_ptr()), 3, 0.0),
                        proxy_b: input.proxy,
                        transform: crate::math::types::Transform::new(-a, crate::math::types::Quat::IDENTITY),
                        translation_b: input.translation,
                        max_fraction: best.fraction,
                        can_encroach: input.can_encroach,
                    });
                    if output.hit {
                        best = output;
                        best.point = output.point + a;
                        best.triangle_index = triangle_index as i32;
                        best.material_index = materials[triangle_index] as i32;
                        end = center + output.fraction * input.translation;
                        ray_min = min_vec3(center, end);
                        ray_max = max_vec3(center, end);
                    }
                }
            } else {
                if get_by_index(input.translation, node.axis()) > 0.0 {
                    stack.push(index + node.child_offset());
                    index += 1;
                } else {
                    stack.push(index + 1);
                    index += node.child_offset();
                }
                continue;
            }
        }
        match stack.pop() {
            Some(next) => index = next,
            None => break,
        }
    }
    best
}

/// Collide a mover capsule with a mesh.
pub fn collide_mover_and_mesh(mesh: &Mesh, mover: crate::types::geometry::Capsule, capacity: usize) -> Vec<PlaneResult> {
    let mut results = Vec::new();
    let Some(data) = mesh.data.as_ref() else { return results };
    if capacity == 0 {
        return results;
    }
    let center = lerp(mover.center1, mover.center2, 0.5);
    let radius = Vec3::new(mover.radius, mover.radius, mover.radius);
    let bounds = MeshQueryBounds::new(
        AABB::new(
            min_vec3(mover.center1, mover.center2) - radius,
            max_vec3(mover.center1, mover.center2) + radius,
        ),
        mesh.scale,
    );
    let ccw = mesh.scale.x * mesh.scale.y * mesh.scale.z > 0.0;
    let vertices = data.vertices();
    let materials = data.material_indices();
    let mover_points = [mover.center1, mover.center2];

    visit_mesh_candidates(data, bounds, |index, triangle| {
        let a = vertices[triangle.index1 as usize];
        let b = vertices[if ccw { triangle.index2 } else { triangle.index3 } as usize];
        let c = vertices[if ccw { triangle.index3 } else { triangle.index2 } as usize];
        if !bounds_triangle_overlap(bounds.center(), bounds.extent(), a, b, c) {
            return true;
        }
        let scaled = [mesh.scale * a, mesh.scale * b, mesh.scale * c];
        if signed_volume(scaled[0], scaled[1], scaled[2], center) < 0.0 {
            return true;
        }
        let mut cache = SimplexCache::default();
        let output = crate::collision::distance::shape_distance(
            DistanceInput {
                proxy_a: ShapeProxy::new(Some(scaled.as_ptr()), 3, 0.0),
                proxy_b: ShapeProxy::new(Some(mover_points.as_ptr()), 2, 0.0),
                transform: crate::math::types::Transform::IDENTITY,
                use_radii: false,
            },
            &mut cache,
        );
        if output.distance != 0.0 && output.distance <= mover.radius {
            results.push(PlaneResult {
                plane: crate::math::types::Plane::new(output.normal, mover.radius - output.distance),
                point: output.point_a,
                triangle_index: index as i32,
                child_index: 0,
                material_index: materials[index] as i32,
            });
        }
        results.len() < capacity
    });

    results
}
