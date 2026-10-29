//! Swept casts against height fields.

use crate::base::constants as b3;
use crate::collision::aabb::ray_cast_aabb;
use crate::collision::distance::shape_cast;
use crate::collision::internal_math::signed_volume;
use crate::collision::mesh::overlap::intersect_mesh_ray_triangle;
use crate::math::functions::*;
use crate::math::types::{AABB, Transform, Vec3};
use crate::types::queries::{
    BoxCastInput, CastOutput, RayCastInput, ShapeCastInput, ShapeCastPairInput, ShapeProxy,
};

use super::quantization::HeightField;
use super::regions::height_cell_bounds;

struct HeightGridAxis {
    head: i32,
    tail: i32,
    end: i32,
    step: i32,
    delta_alpha: f32,
    next_fraction: f32,
}

impl HeightGridAxis {
    fn new(start: f32, end: f32, tail: f32, absolute_delta: f32, scale: f32) -> Self {
        let head = (start / scale).floor() as i32;
        let end = (end / scale).floor() as i32;
        let tail = (tail / scale).floor() as i32;

        if head < end {
            Self {
                head,
                tail,
                end,
                step: 1,
                delta_alpha: scale / absolute_delta,
                next_fraction: (scale * (head + 1) as f32 - start) / absolute_delta,
            }
        } else if end < head {
            Self {
                head,
                tail,
                end,
                step: -1,
                delta_alpha: scale / absolute_delta,
                next_fraction: (start - scale * head as f32) / absolute_delta,
            }
        } else {
            Self {
                head,
                tail,
                end,
                step: 0,
                delta_alpha: 0.0,
                next_fraction: f32::MAX,
            }
        }
    }
}

/// Ray cast a height field.
pub fn ray_cast_height_field(field: &HeightField, input: &RayCastInput) -> CastOutput {
    let shape_input = ShapeCastInput {
        proxy: ShapeProxy::new(Some(&input.origin as *const Vec3), 1, 0.0),
        translation: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: false,
    };
    shape_cast_height_field(field, &shape_input)
}

/// Shape cast a height field.
pub fn shape_cast_height_field(field: &HeightField, input: &ShapeCastInput) -> CastOutput {
    let bounds = make_aabb(input.proxy.points(), input.proxy.radius);
    let translation = input.translation;
    let scale = field.scale();
    let start = aabb_center(bounds);
    let delta = input.max_fraction * translation;
    let end = start + delta;

    let mut result = CastOutput::default();

    let extent = aabb_extents(bounds);
    let margin = Vec3::new(
        b3::max_aabb_margin(),
        b3::max_aabb_margin(),
        b3::max_aabb_margin(),
    );
    let combined = AABB::new(
        (field.bounds().lower_bound - extent) - margin,
        (field.bounds().upper_bound + extent) + margin,
    );

    let mut minimum: f32 = 0.0;
    let mut maximum: f32 = 0.0;
    if !ray_cast_aabb(combined, start, end, &mut minimum, &mut maximum) {
        return result;
    }

    let mut clamped_start = mul_add(start, minimum, delta);
    let clamped_delta = (maximum - minimum) * delta;
    let mut clamped_end = clamped_start + clamped_delta;

    let center_start = clamped_start;
    let center_end = clamped_end;

    let sign_x: f32 = if translation.x >= 0.0 { 1.0 } else { -1.0 };
    let sign_z: f32 = if translation.z >= 0.0 { 1.0 } else { -1.0 };
    if translation.x >= 0.0 {
        clamped_start.x += extent.x;
    } else {
        clamped_start.x -= extent.x;
    }
    if translation.z >= 0.0 {
        clamped_start.z += extent.z;
    } else {
        clamped_start.z -= extent.z;
    }
    clamped_end = clamped_start + clamped_delta;

    let absolute = abs_vec3(clamped_delta);
    let mut x = HeightGridAxis::new(
        clamped_start.x,
        clamped_end.x,
        clamped_start.x - 2.0 * sign_x * extent.x,
        absolute.x,
        scale.x,
    );
    let mut z = HeightGridAxis::new(
        clamped_start.z,
        clamped_end.z,
        clamped_start.z - 2.0 * sign_z * extent.z,
        absolute.z,
        scale.z,
    );

    let mut best_fraction = input.max_fraction;
    let grid_scale = input.max_fraction * (maximum - minimum);
    let grid_offset = input.max_fraction * minimum;

    let mut pair = ShapeCastPairInput {
        proxy_a: ShapeProxy::default(),
        proxy_b: input.proxy,
        transform: Transform::IDENTITY,
        translation_b: translation,
        max_fraction: 0.0,
        can_encroach: input.can_encroach,
    };

    let cast_bounds = AABB::new(
        min_vec3(center_start, center_end) - extent,
        max_vec3(center_start, center_end) + extent,
    );

    loop {
        let row1 = 0.max(z.tail.min(z.head));
        let row2 = ((field.row_count() as i32) - 2).min(z.tail.max(z.head));
        let column1 = 0.max(x.tail.min(x.head));
        let column2 = ((field.column_count() as i32) - 2).min(x.tail.max(x.head));

        if row1 <= row2 && column1 <= column2 {
            for row in row1..=row2 {
                for column in column1..=column2 {
                    let cell = (row as usize) * (field.column_count() - 1) + column as usize;
                    let material = field.material_indices()[cell];
                    if material == 0xff {
                        continue;
                    }

                    let mut v = field.corners(row as usize, column as usize);
                    if field.header().clockwise != 0 {
                        v.swap(1, 2);
                    }
                    if !aabb_overlaps(cast_bounds, height_cell_bounds(&v)) {
                        continue;
                    }

                    for second in [false, true] {
                        let index = (2 * cell + if second { 1 } else { 0 }) as i32;
                        let origin = if second { v[2] } else { v[0] };
                        let b = if second { v[3] } else { v[2] };
                        let c = v[1];

                        if input.proxy.count == 1 && input.proxy.radius == 0.0 {
                            let alpha = if second {
                                intersect_mesh_ray_triangle(start, translation, v[3], v[1], v[2])
                            } else {
                                intersect_mesh_ray_triangle(start, translation, v[0], v[2], v[1])
                            };
                            if alpha < best_fraction {
                                result.point = mul_add(start, alpha, translation);
                                result.normal = normalize(cross(b - origin, c - origin));
                                result.fraction = alpha;
                                result.triangle_index = index;
                                result.material_index = material as i32;
                                result.hit = true;
                                best_fraction = alpha;
                            }
                        } else if signed_volume(origin, b, c, start) >= 0.0 {
                            let triangle = [Vec3::ZERO, b - origin, c - origin];
                            pair.proxy_a = ShapeProxy::new(Some(triangle.as_ptr()), 3, 0.0);
                            pair.max_fraction = best_fraction;
                            pair.transform.p = -origin;
                            let output = shape_cast(pair);
                            if output.hit {
                                best_fraction = output.fraction;
                                result = output;
                                result.point = result.point + origin;
                                result.triangle_index = index;
                                result.material_index = material as i32;
                            }
                        }
                    }
                }
            }
        }

        let sentinel = f32::MAX;
        let fraction_x = if x.next_fraction == sentinel {
            sentinel
        } else {
            grid_offset + x.next_fraction * grid_scale
        };
        let fraction_z = if z.next_fraction == sentinel {
            sentinel
        } else {
            grid_offset + z.next_fraction * grid_scale
        };

        if fraction_x > best_fraction && fraction_z > best_fraction {
            break;
        }

        if x.next_fraction <= z.next_fraction {
            if x.head == x.end {
                break;
            }
            x.head += x.step;
            x.tail = x.head;
            if extent.z == 0.0 {
                z.tail = z.head;
            } else {
                let intercept = clamped_start.z + x.next_fraction * clamped_delta.z;
                z.tail = ((intercept - 2.0 * sign_z * extent.z) / scale.z).floor() as i32;
            }
            x.next_fraction += x.delta_alpha;
        } else {
            if z.head == z.end {
                break;
            }
            z.head += z.step;
            z.tail = z.head;
            if extent.x == 0.0 {
                x.tail = x.head;
            } else {
                let intercept = clamped_start.x + z.next_fraction * clamped_delta.x;
                x.tail = ((intercept - 2.0 * sign_x * extent.x) / scale.x).floor() as i32;
            }
            z.next_fraction += z.delta_alpha;
        }
    }

    result
}

/// A box cast against a height field.
pub fn box_cast_height_field(field: &HeightField, input: &BoxCastInput) -> CastOutput {
    let proxy = ShapeProxy::new(Some(&input.box_.lower_bound as *const Vec3), 2, 0.0);
    let _ = proxy;
    let mut shape_input = ShapeCastInput::default();
    shape_input.max_fraction = input.max_fraction;
    shape_input.translation = input.translation;
    shape_cast_height_field(field, &shape_input)
}
