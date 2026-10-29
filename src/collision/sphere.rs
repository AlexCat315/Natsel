//! Sphere mass properties, bounds, ray casts, and mover collision.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::distance::shape_distance;
use crate::collision::internal_math::make_diagonal_matrix;
use crate::math::functions::*;
use crate::math::types::*;
use crate::types::geometry::{Capsule, MassData, Sphere};
use crate::types::queries::*;

/// Compute the mass properties of a sphere.
pub fn compute_sphere_mass(shape: &Sphere, density: f32) -> MassData {
    let radius = shape.radius;
    let volume = 4.0 / 3.0 * b3::PI * radius * radius * radius;
    let mass = volume * density;
    let ixx = 0.4 * mass * radius * radius;

    MassData {
        mass,
        center: shape.center,
        inertia: make_diagonal_matrix(ixx, ixx, ixx),
    }
}

/// Compute the AABB of a sphere.
pub fn compute_sphere_aabb(shape: &Sphere, transform: Transform) -> AABB {
    let center = transform_point(transform, shape.center);
    let radius = shape.radius;
    let extent = Vec3::new(radius, radius, radius);
    AABB::new(center - extent, center + extent)
}

/// Compute the swept AABB of a sphere.
pub fn compute_swept_sphere_aabb(shape: &Sphere, xf1: Transform, xf2: Transform) -> AABB {
    let r = Vec3::new(shape.radius, shape.radius, shape.radius);
    let center1 = transform_point(xf1, shape.center);
    let center2 = transform_point(xf2, shape.center);
    AABB::new(
        min_vec3(center1, center2) - r,
        max_vec3(center1, center2) + r,
    )
}

/// Do a sphere and a point cloud proxy overlap?
pub fn overlap_sphere(shape: &Sphere, shape_transform: Transform, proxy: &ShapeProxy) -> bool {
    let center = shape.center;
    let input = DistanceInput {
        proxy_a: ShapeProxy::new(Some(&center as *const Vec3), 1, shape.radius),
        proxy_b: *proxy,
        transform: inv_mul_transforms(shape_transform, Transform::IDENTITY),
        use_radii: true,
    };
    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);
    output.distance < b3::overlap_slop()
}

/// Ray cast a sphere.
pub fn ray_cast_sphere(shape: &Sphere, input: &RayCastInput) -> CastOutput {
    b3_assert(is_valid_ray(input));
    let mut output = CastOutput::default();
    let p = shape.center;
    let s = input.origin - p;
    let r = shape.radius;
    let rr = r * r;

    let (length, d) = get_length_and_normalize(input.translation);
    if length == 0.0 {
        if length_squared(s) < rr {
            output.point = input.origin;
            output.hit = true;
        }
        return output;
    }

    let t = -dot(s, d);
    let c = mul_add(s, t, d);
    let cc = dot(c, c);

    if cc > rr {
        return output;
    }

    let h = (rr - cc).sqrt();
    let fraction = t - h;

    if fraction < 0.0 || input.max_fraction * length < fraction {
        if length_squared(s) < rr {
            output.point = input.origin;
            output.hit = true;
        }
        return output;
    }

    let hit_point = mul_add(s, fraction, d);
    output.fraction = fraction / length;
    if output.fraction > input.max_fraction {
        crate::base::core::b3_log(&format!(
            "sphere input fraction = {}, output fraction = {}",
            input.max_fraction, output.fraction
        ));
        output.fraction = input.max_fraction;
    }
    output.normal = normalize(hit_point);
    output.point = mul_add(p, shape.radius, output.normal);
    output.hit = true;
    output
}

/// Ray cast a hollow sphere. This will do interior hits.
pub fn ray_cast_hollow_sphere(sphere: &Sphere, input: &RayCastInput) -> CastOutput {
    let mut output = CastOutput::default();
    let p = sphere.center;
    let s = input.origin - p;
    let d = normalize(input.translation);

    let t = -dot(s, d);
    let c = mul_add(s, t, d);
    let cc = dot(c, c);
    let r = sphere.radius;
    let rr = r * r;

    if cc > rr {
        return output;
    }

    let h = (rr - cc).sqrt();
    let mut fraction = t - h;
    if fraction < 0.0 {
        fraction = t + h;
    }
    if fraction < 0.0 || fraction > input.max_fraction {
        return output;
    }

    let hit_point = mul_add(s, fraction, d);
    output.fraction = fraction;
    output.normal = normalize(hit_point);
    output.point = mul_add(p, sphere.radius, output.normal);
    output.hit = true;
    output
}

/// Shape cast a sphere against a point cloud proxy.
pub fn shape_cast_sphere(sphere: &Sphere, input: &ShapeCastInput) -> CastOutput {
    let center = sphere.center;
    let pair_input = ShapeCastPairInput {
        proxy_a: ShapeProxy::new(Some(&center as *const Vec3), 1, sphere.radius),
        proxy_b: input.proxy,
        transform: Transform::IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };
    crate::collision::distance::shape_cast(&pair_input)
}

/// Collide a mover capsule with a sphere.
pub fn collide_mover_and_sphere(result: &mut PlaneResult, shape: &Sphere, mover: &Capsule) -> i32 {
    let total_radius = mover.radius + shape.radius;
    let closest = point_to_segment_distance(mover.center1, mover.center2, shape.center);
    let (distance, normal) = {
        let closest_shift = closest - shape.center;
        let (len, dir) = get_length_and_normalize(closest_shift);
        (len, dir)
    };

    let mut distance = distance;
    let mut normal = normal;

    if distance > total_radius {
        return 0;
    }

    let linear_slop = b3::linear_slop();
    if distance < linear_slop {
        let (length, axis) = get_length_and_normalize(mover.center2 - mover.center1);
        normal = if length > linear_slop { perp(axis) } else { Vec3::AXIS_Y };
        distance = 0.0;
    }

    result.plane = Plane::new(normal, total_radius - distance);
    result.point = shape.center;
    result.triangle_index = 0;
    result.child_index = 0;
    result.material_index = 0;
    1
}
