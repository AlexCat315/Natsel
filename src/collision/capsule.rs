//! Capsule mass properties, bounds, ray casts, and mover collision.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::distance::{shape_cast, shape_distance};
use crate::collision::internal_math::make_diagonal_matrix;
use crate::collision::sphere::ray_cast_sphere;
use crate::math::functions::*;
use crate::math::types::*;
use crate::types::geometry::{Capsule, MassData, Sphere};
use crate::types::queries::*;

/// The inertia tensor of a solid sphere about its center.
pub fn sphere_inertia(mass: f32, radius: f32) -> Matrix3 {
    let i = 0.4 * mass * radius * radius;
    make_diagonal_matrix(i, i, i)
}

/// The inertia tensor of a solid cylinder about its center.
pub fn cylinder_inertia(mass: f32, radius: f32, height: f32) -> Matrix3 {
    let ixx = mass * (3.0 * radius * radius + height * height) / 12.0;
    let iyy = 0.5 * mass * radius * radius;
    make_diagonal_matrix(ixx, iyy, ixx)
}

/// Compute the mass properties of a capsule.
pub fn compute_capsule_mass(shape: &Capsule, density: f32) -> MassData {
    let c1 = shape.center1;
    let c2 = shape.center2;
    let r = shape.radius;

    let cylinder_height = distance(c1, c2);
    let cylinder_volume = b3::PI * r * r * cylinder_height;
    let cylinder_mass = cylinder_volume * density;

    let sphere_volume = (4.0 / 3.0) * b3::PI * r * r * r;
    let sphere_mass = sphere_volume * density;

    let mut inertia = add_mm(
        cylinder_inertia(cylinder_mass, r, cylinder_height),
        sphere_inertia(sphere_mass, r),
    );

    let steiner = 0.125 * sphere_mass * (3.0 * r + 2.0 * cylinder_height) * cylinder_height;
    inertia.cx.x += steiner;
    inertia.cz.z += steiner;

    let mut rotation = Matrix3::IDENTITY;
    if cylinder_height * cylinder_height > 1000.0 * f32::MIN_POSITIVE {
        let direction = normalize(c2 - c1);
        let q = compute_quat_between_unit_vectors(Vec3::AXIS_Y, direction);
        rotation = make_matrix_from_quat(q);
    }

    let mass = sphere_mass + cylinder_mass;
    let center = 0.5 * (c1 + c2);

    MassData {
        mass,
        center,
        inertia: mul_mm(rotation, mul_mm(inertia, transpose(rotation))),
    }
}

/// Compute the AABB of a capsule.
pub fn compute_capsule_aabb(shape: &Capsule, transform: Transform) -> AABB {
    let r = shape.radius;
    let center1 = transform_point(transform, shape.center1);
    let center2 = transform_point(transform, shape.center2);
    let extent = Vec3::new(r, r, r);
    AABB::new(
        min_vec3(center1, center2) - extent,
        max_vec3(center1, center2) + extent,
    )
}

/// Compute the swept AABB of a capsule.
pub fn compute_swept_capsule_aabb(shape: &Capsule, xf1: Transform, xf2: Transform) -> AABB {
    let r = Vec3::new(shape.radius, shape.radius, shape.radius);
    let a = transform_point(xf1, shape.center1);
    let b = transform_point(xf1, shape.center2);
    let c = transform_point(xf2, shape.center1);
    let d = transform_point(xf2, shape.center2);
    AABB::new(
        min_vec3(min_vec3(a, b), min_vec3(c, d)) - r,
        max_vec3(max_vec3(a, b), max_vec3(c, d)) + r,
    )
}

/// Do a capsule and a point cloud proxy overlap?
pub fn overlap_capsule(shape: &Capsule, shape_transform: Transform, proxy: &ShapeProxy) -> bool {
    let points = [shape.center1, shape.center2];
    let input = DistanceInput {
        proxy_a: ShapeProxy::new(Some(points.as_ptr()), 2, shape.radius),
        proxy_b: *proxy,
        transform: inv_mul_transforms(shape_transform, Transform::IDENTITY),
        use_radii: true,
    };
    let mut cache = SimplexCache::default();
    let output = shape_distance(&input, &mut cache, None);
    output.distance < b3::overlap_slop()
}

/// Ray cast a capsule.
pub fn ray_cast_capsule(shape: &Capsule, input: &RayCastInput) -> CastOutput {
    b3_assert(is_valid_ray(input));

    let c1 = shape.center1;
    let c2 = shape.center2;
    let r = shape.radius;
    let mut output = CastOutput::default();

    let d = c2 - c1;
    let tol = 0.01 * b3::linear_slop();
    let axis_length_sq = length_squared(d);
    if axis_length_sq < tol * tol {
        let sphere = Sphere::new(0.5 * (shape.center1 + shape.center2), shape.radius);
        return ray_cast_sphere(&sphere, input);
    }

    let s = input.origin - c1;
    let length = axis_length_sq.sqrt();
    let axis = (1.0 / length) * d;

    let u = dot(s, axis);
    let c = u * axis;
    let sc = s - c;
    let sc2 = length_squared(sc);

    if sc2 < r * r {
        let u_clamped = clamp_float(u, 0.0, length);
        let cp = u_clamped * axis;
        let scp = s - cp;
        let scp2 = length_squared(scp);

        if scp2 < r * r {
            output.hit = true;
            output.point = input.origin;
            return output;
        }

        let sphere = Sphere::new(c1 + cp, r);
        return ray_cast_sphere(&sphere, input);
    }

    let dr = input.translation;
    let (ray_length, ray_axis) = get_length_and_normalize(dr);
    if ray_length == 0.0 {
        return output;
    }

    let v = u + input.max_fraction * dot(dr, axis);
    if (u < -r && v < -r) || (length + r < u && length + r < v) {
        return output;
    }

    let a1 = axis;
    let a2 = ray_axis;
    let a12 = dot(a1, a2);

    let tr: f32;
    let det = 1.0 - a12 * a12;
    if det < f32::EPSILON {
        let perp_v = mul_sub(a2, a12, a1);
        let perp2 = length_squared(perp_v);
        let beta = dot(sc, perp_v);
        let gamma = sc2 - r * r;
        let disc = beta * beta - perp2 * gamma;

        if beta >= 0.0 || disc < 0.0 {
            return output;
        }
        tr = gamma / (-beta + disc.sqrt());
    } else {
        let inv_det = 1.0 / det;
        let sa1 = u;
        let sa2 = dot(s, a2);
        let t1 = (sa1 - a12 * sa2) * inv_det;
        let t2 = (a12 * sa1 - sa2) * inv_det;

        let p1 = t1 * a1;
        let p2 = mul_add(s, t2, a2);
        let g = p2 - p1;
        let g2 = length_squared(g);
        if g2 > r * r {
            return output;
        }

        let h = ((r * r - g2) * inv_det).sqrt();
        tr = t2 - h;
    }

    if tr < 0.0 || input.max_fraction * ray_length < tr {
        return output;
    }

    let tc = u + tr * a12;
    if tc < 0.0 {
        let sphere = Sphere::new(c1, r);
        return ray_cast_sphere(&sphere, input);
    }
    if length < tc {
        let sphere = Sphere::new(c2, r);
        return ray_cast_sphere(&sphere, input);
    }

    let p = mul_add(s, tr, ray_axis);
    let normal = normalize(mul_sub(p, tc, axis));

    output.point = c1 + p;
    output.normal = normal;
    output.fraction = clamp_float(tr / ray_length, 0.0, input.max_fraction);
    output.hit = true;
    output
}

/// Shape cast a capsule against a point cloud proxy.
pub fn shape_cast_capsule(capsule: &Capsule, input: &ShapeCastInput) -> CastOutput {
    let points = [capsule.center1, capsule.center2];
    let pair_input = ShapeCastPairInput {
        proxy_a: ShapeProxy::new(Some(points.as_ptr()), 2, capsule.radius),
        proxy_b: input.proxy,
        transform: Transform::IDENTITY,
        translation_b: input.translation,
        max_fraction: input.max_fraction,
        can_encroach: input.can_encroach,
    };
    shape_cast(&pair_input)
}

/// Collide a mover capsule with a capsule shape.
pub fn collide_mover_and_capsule(
    result: &mut PlaneResult,
    shape: &Capsule,
    mover: &Capsule,
) -> i32 {
    let total_radius = mover.radius + shape.radius;
    let approach = segment_distance(shape.center1, shape.center2, mover.center1, mover.center2);
    let (distance, initial_normal) = get_length_and_normalize(approach.point2 - approach.point1);
    let mut distance = distance;
    let mut normal = initial_normal;

    if distance > total_radius {
        return 0;
    }

    let linear_slop = b3::linear_slop();
    if distance < linear_slop {
        let (mover_length, mover_axis) =
            get_length_and_normalize(mover.center2 - mover.center1);
        normal = if mover_length > linear_slop {
            perp(mover_axis)
        } else {
            Vec3::AXIS_Y
        };
        distance = 0.0;
    }

    result.plane = Plane::new(normal, total_radius - distance);
    result.point = approach.point1;
    result.triangle_index = 0;
    result.child_index = 0;
    result.material_index = 0;
    1
}
