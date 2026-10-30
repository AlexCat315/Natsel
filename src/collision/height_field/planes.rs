//! Collision between a mover capsule and a height field.

use crate::collision::aabb::bounds_triangle_overlap;
use crate::collision::distance::shape_distance;
use crate::collision::internal_math::signed_volume;
use crate::math::functions::{lerp, max_vec3, min_vec3};
use crate::math::types::{AABB, Plane, Vec3};
use crate::types::geometry::Capsule;
use crate::types::queries::{DistanceInput, PlaneResult, ShapeProxy, SimplexCache};

use super::quantization::HeightField;
use super::regions::HeightFieldRegion;

/// Collide a mover capsule with a height field.
pub fn collide_mover_and_height_field(
    field: &HeightField,
    mover: &Capsule,
    capacity: usize,
) -> Vec<PlaneResult> {
    let mut planes: Vec<PlaneResult> = Vec::new();
    if capacity == 0 {
        return planes;
    }

    let center = lerp(mover.center1, mover.center2, 0.5);
    let r = Vec3::new(mover.radius, mover.radius, mover.radius);
    let bounds = AABB::new(
        min_vec3(mover.center1, mover.center2) - r,
        max_vec3(mover.center1, mover.center2) + r,
    );
    let bounds_center = 0.5 * (bounds.lower_bound + bounds.upper_bound);
    let bounds_extent = bounds.upper_bound - bounds_center;
    let region = HeightFieldRegion::new(field, bounds);
    let mover_points = [mover.center1, mover.center2];

    let mut input = DistanceInput {
        proxy_a: ShapeProxy::default(),
        proxy_b: ShapeProxy::new(Some(mover_points.as_ptr()), 2, 0.0),
        transform: crate::math::types::Transform::IDENTITY,
        use_radii: false,
    };
    let mut cache = SimplexCache::default();

    for row in region.rows {
        for column in region.columns.clone() {
            let cell = row * (field.column_count() - 1) + column;
            let material = field.material_indices()[cell];
            if material == 0xff {
                continue;
            }

            let mut v = field.corners(row, column);
            if field.header().clockwise != 0 {
                v.swap(1, 2);
            }

            for second in [false, true] {
                let sat = if second {
                    [v[2], v[3], v[1]]
                } else {
                    [v[0], v[2], v[1]]
                };
                if !bounds_triangle_overlap(bounds_center, bounds_extent, sat[0], sat[1], sat[2]) {
                    continue;
                }

                let points = if second {
                    [v[3], v[1], v[2]]
                } else {
                    [v[0], v[2], v[1]]
                };
                if signed_volume(points[0], points[1], points[2], center) < 0.0 {
                    continue;
                }

                input.proxy_a = ShapeProxy::new(Some(points.as_ptr()), 3, 0.0);
                cache.count = 0;
                let output = shape_distance(&input, &mut cache, None);

                if output.distance != 0.0 && output.distance <= mover.radius {
                    planes.push(PlaneResult {
                        plane: Plane::new(output.normal, mover.radius - output.distance),
                        point: output.point_a,
                        triangle_index: (2 * cell + if second { 1 } else { 0 }) as i32,
                        child_index: 0,
                        material_index: material as i32,
                    });
                    if planes.len() == capacity {
                        return planes;
                    }
                }
            }
        }
    }

    planes
}
