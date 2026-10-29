//! AABB ray casts and triangle overlap tests.

use crate::collision::internal_math::{get_by_index, modified_cross};
use crate::math::functions::*;
use crate::math::types::{AABB, Vec3};

/// Ray cast an AABB. Similar to Real-time Collision Detection, p179.
pub fn ray_cast_aabb(
    a: AABB,
    p1: Vec3,
    p2: Vec3,
    min_fraction: &mut f32,
    max_fraction: &mut f32,
) -> bool {
    let d = p2 - p1;
    let ray_length = length(d);

    if ray_length < f32::EPSILON {
        if p1.x >= a.lower_bound.x
            && p1.x <= a.upper_bound.x
            && p1.y >= a.lower_bound.y
            && p1.y <= a.upper_bound.y
            && p1.z >= a.lower_bound.z
            && p1.z <= a.upper_bound.z
        {
            *min_fraction = 0.0;
            *max_fraction = 0.0;
            return true;
        }
        return false;
    }

    let ray_dir = (1.0 / ray_length) * d;

    let mut t_min: f32 = 0.0;
    let mut t_max = ray_length;

    for axis in 0..3 {
        let ray_component = get_by_index(ray_dir, axis);
        let ray_start = get_by_index(p1, axis);
        let box_min = get_by_index(a.lower_bound, axis);
        let box_max = get_by_index(a.upper_bound, axis);

        if abs_float(ray_component) < f32::EPSILON {
            if ray_start < box_min || ray_start > box_max {
                return false;
            }
        } else {
            let mut t1 = (box_min - ray_start) / ray_component;
            let mut t2 = (box_max - ray_start) / ray_component;
            if t1 > t2 {
                std::mem::swap(&mut t1, &mut t2);
            }
            t_min = max_float(t_min, t1);
            t_max = min_float(t_max, t2);
            if t_min > t_max {
                return false;
            }
        }
    }

    if t_max < 0.0 {
        return false;
    }

    *min_fraction = clamp_float(t_min / ray_length, 0.0, 1.0);
    *max_fraction = clamp_float(t_max / ray_length, 0.0, 1.0);
    true
}

#[inline]
pub fn bounds_overlap(min1: Vec3, max1: Vec3, min2: Vec3, max2: Vec3) -> bool {
    let separation = max_vec3(min2 - max1, min1 - max2);
    separation.x <= 0.0 && separation.y <= 0.0 && separation.z <= 0.0
}

/// Scalar triangle/AABB separating-axis test.
pub fn bounds_triangle_overlap(center: Vec3, extent: Vec3, a: Vec3, b: Vec3, c: Vec3) -> bool {
    let v1 = a - center;
    let v2 = b - center;
    let v3 = c - center;
    let triangle_min = min_vec3(v1, min_vec3(v2, v3));
    let triangle_max = max_vec3(v1, max_vec3(v2, v3));
    let face = max_vec3(triangle_min - extent, -(triangle_max + extent));
    if face.x > 0.0 || face.y > 0.0 || face.z > 0.0 {
        return false;
    }
    let e1 = v2 - v1;
    let e2 = v3 - v2;
    let e3 = v1 - v3;
    let normal = cross(e1, e2);
    let triangle_separation = abs_float(dot(normal, v1)) - dot(abs_vec3(normal), extent);
    if triangle_separation > 0.0 {
        return false;
    }
    let separation1 =
        abs_vec3(cross(e1, v1 + v3)) - abs_vec3(cross(e1, e3)) - 2.0 * modified_cross(abs_vec3(e1), extent);
    if separation1.x > 0.0 || separation1.y > 0.0 || separation1.z > 0.0 {
        return false;
    }
    let separation2 =
        abs_vec3(cross(e2, v1 + v2)) - abs_vec3(cross(e2, e1)) - 2.0 * modified_cross(abs_vec3(e2), extent);
    if separation2.x > 0.0 || separation2.y > 0.0 || separation2.z > 0.0 {
        return false;
    }
    let separation3 =
        abs_vec3(cross(e3, v2 + v3)) - abs_vec3(cross(e3, e2)) - 2.0 * modified_cross(abs_vec3(e3), extent);
    if separation3.x > 0.0 || separation3.y > 0.0 || separation3.z > 0.0 {
        return false;
    }
    true
}
