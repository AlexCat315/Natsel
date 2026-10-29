//! Hull bounds and shape queries shared by shapes and compounds.

use crate::base::constants as b3;
use crate::collision::distance::{shape_cast, shape_distance};
use crate::collision::hull::Hull;
use crate::collision::internal_math::ShapeExtent;
use crate::math::functions::*;
use crate::math::types::{AABB, Transform, Vec3};
use crate::types::queries::{
    CastOutput, DistanceInput, RayCastInput, ShapeCastInput, ShapeCastPairInput, ShapeProxy,
    SimplexCache,
};

/// Compute the AABB of a hull.
pub fn compute_hull_aabb(hull: &Hull, transform: Transform) -> AABB {
    aabb_transform(transform, hull.aabb())
}

/// Compute the swept AABB of a hull.
pub fn compute_swept_hull_aabb(hull: &Hull, xf1: Transform, xf2: Transform) -> AABB {
    aabb_union(compute_hull_aabb(hull, xf1), compute_hull_aabb(hull, xf2))
}

/// Do a hull and a point cloud proxy overlap?
pub fn overlap_hull(hull: &Hull, shape_transform: Transform, proxy: &ShapeProxy) -> bool {
    let input = DistanceInput {
        proxy_a: ShapeProxy::new(Some(hull.points().as_ptr()), hull.vertex_count(), 0.0),
        proxy_b: *proxy,
        transform: inv_mul_transforms(shape_transform, Transform::IDENTITY),
        use_radii: true,
    };
    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);
    output.distance < b3::overlap_slop()
}

/// Ray cast a hull.
pub fn ray_cast_hull(hull: &Hull, input: &RayCastInput) -> CastOutput {
    b3_assert(is_valid_ray(input));
    let mut output = CastOutput::default();

    let mut lower: f32 = 0.0;
    let mut upper = input.max_fraction;
    let mut best_face = b3::NULL_INDEX;

    let planes = hull.planes();
    for (face_index, plane) in planes.iter().enumerate().take(hull.face_count() as usize) {
        let distance = plane.offset - dot(plane.normal, input.origin);
        let denominator = dot(plane.normal, input.translation);

        if denominator == 0.0 {
            if distance < 0.0 {
                return output;
            }
        } else {
            let fraction = distance / denominator;

            if denominator < 0.0 {
                if fraction > lower {
                    best_face = face_index as i32;
                    lower = fraction;
                }
            } else if fraction < upper {
                upper = fraction;
            }

            if upper < lower {
                return output;
            }
        }
    }

    if best_face >= 0 {
        output.point = input.origin + lower * input.translation;
        output.normal = planes[best_face as usize].normal;
        output.fraction = lower;
        output.hit = true;
    } else {
        output.point = input.origin;
        output.hit = true;
    }

    output
}

/// Shape cast a hull.
pub fn shape_cast_hull(hull: &Hull, input: &ShapeCastInput) -> CastOutput {
    let pair_input = ShapeCastPairInput {
        proxy_a: ShapeProxy::new(Some(hull.points().as_ptr()), hull.vertex_count(), 0.0),
        proxy_b: input.proxy,
        transform: Transform::IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };
    shape_cast(pair_input)
}

/// Compute the extent of a hull about a local origin.
pub fn compute_hull_extent(hull: &Hull, origin: Vec3) -> crate::collision::internal_math::ShapeExtent {
    let mut extent = ShapeExtent {
        min_extent: hull.inner_radius(),
        max_extent: Vec3::ZERO,
    };
    for point in hull.points() {
        extent.max_extent = max_vec3(extent.max_extent, abs_vec3(*point - origin));
    }
    extent
}

/// Project the hull surface area onto a plane.
pub fn compute_hull_projected_area(hull: &Hull, direction: Vec3) -> f32 {
    let mut area: f32 = 0.0;

    let faces = hull.faces();
    let edges = hull.edges();
    let points = hull.points();

    for i in 0..hull.face_count() as usize {
        let base_edge = faces[i].edge as i32;
        let edge = edges[base_edge as usize];
        let p1 = points[edge.origin as usize];

        let mut edge_index = edge.next as i32;
        let next_edge = edges[edge_index as usize];
        let mut p2 = points[next_edge.origin as usize];

        edge_index = next_edge.next as i32;

        loop {
            let current = edges[edge_index as usize];
            let p3 = points[current.origin as usize];

            let n = cross(p2 - p1, p3 - p1);
            area += max_float(dot(n, direction), 0.0);

            p2 = p3;
            edge_index = current.next as i32;
            if edge_index == base_edge {
                break;
            }
        }
    }

    0.5 * area
}
