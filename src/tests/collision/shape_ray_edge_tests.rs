//! Ported from Tests/NatselTests/Collision/ShapeRayEdgeTests.swift.

use crate::collision::capsule::ray_cast_capsule;
use crate::collision::sphere::ray_cast_sphere;
use crate::math::functions::{mul_add, point_to_segment_distance};
use crate::math::types::Vec3;
use crate::tests::world_position_assertions::{abs_float, distance};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::queries::{CastOutput, RayCastInput};

/// Capsule along x with radius 1.
fn ray_capsule() -> Capsule {
    Capsule::new(Vec3::new(-2.0, 0.0, 0.0), Vec3::new(2.0, 0.0, 0.0), 1.0)
}

fn check_hit(
    out: &CastOutput,
    origin: Vec3,
    translation: Vec3,
    point: Vec3,
    normal: Vec3,
    fraction: f32,
    tol: f32,
) -> Option<String> {
    if !out.hit {
        return Some("no hit".to_owned());
    }
    if abs_float((out.fraction - fraction) as f64) > tol as f64 {
        return Some(format!("fraction {}", out.fraction));
    }
    if abs_float((out.point.x - point.x) as f64) > tol as f64
        || abs_float((out.point.y - point.y) as f64) > tol as f64
        || abs_float((out.point.z - point.z) as f64) > tol as f64
    {
        return Some(format!("point {:?}", out.point));
    }
    if abs_float((out.normal.x - normal.x) as f64) > tol as f64
        || abs_float((out.normal.y - normal.y) as f64) > tol as f64
        || abs_float((out.normal.z - normal.z) as f64) > tol as f64
    {
        return Some(format!("normal {:?}", out.normal));
    }

    // The reported point must also sit on the ray.
    let on_ray = mul_add(origin, out.fraction, translation);
    if distance(out.point, on_ray) > tol {
        return Some("point not on ray".to_owned());
    }
    None
}

#[test]
fn degenerate() {
    // "coincident capsule centers collapse to a sphere"
    // Coincident centers collapse to a sphere.
    let sphere_like = Capsule::new(Vec3::ZERO, Vec3::ZERO, 1.0);
    let mut input = RayCastInput::default();
    input.origin = Vec3::new(-4.0, 0.0, 0.0);
    input.translation = Vec3::new(8.0, 0.0, 0.0);
    input.max_fraction = 1.0;

    let out = ray_cast_capsule(&sphere_like, &input);
    if let Some(error) = check_hit(
        &out,
        input.origin,
        input.translation,
        Vec3::new(-1.0, 0.0, 0.0),
        Vec3::new(-1.0, 0.0, 0.0),
        3.0 / 8.0,
        1e-5,
    ) {
        assert!(false, "{}", error);
    }
}

#[test]
fn clip() {
    // "max fraction straddles the side hit"
    let capsule = ray_capsule();

    // The side hit occurs at fraction 1/3.
    let mut miss = RayCastInput::default();
    miss.origin = Vec3::new(0.0, 3.0, 0.0);
    miss.translation = Vec3::new(0.0, -6.0, 0.0);
    miss.max_fraction = 0.3;
    assert!(!ray_cast_capsule(&capsule, &miss).hit);

    let mut hit = RayCastInput::default();
    hit.origin = Vec3::new(0.0, 3.0, 0.0);
    hit.translation = Vec3::new(0.0, -6.0, 0.0);
    hit.max_fraction = 0.5;
    let out = ray_cast_capsule(&capsule, &hit);
    assert!(out.hit);
    assert!(abs_float((out.fraction - 1.0 / 3.0) as f64) < 1e-5);
}

#[test]
fn parallel() {
    // "near-parallel rays converging on the axis still hit"
    // Capsule along y. A long ray almost parallel to the axis drifts inward from just outside
    // the cylinder and dips through the far endcap.
    let axis_y = Capsule::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 10.0, 0.0), 1.0);

    let mut first = RayCastInput::default();
    first.origin = Vec3::new(1.0001, 100.0, 0.0);
    first.translation = Vec3::new(-0.001, -200.0, 0.0);
    first.max_fraction = 1.0;

    let out_a = ray_cast_capsule(&axis_y, &first);
    assert!(out_a.hit);
    if out_a.hit {
        // The hit lands on the capsule surface and on the ray.
        let on_seg = point_to_segment_distance(axis_y.center1, axis_y.center2, out_a.point);
        assert!(abs_float((distance(out_a.point, on_seg) - axis_y.radius) as f64) < 1e-3);
        let on_ray = mul_add(first.origin, out_a.fraction, first.translation);
        assert!(distance(out_a.point, on_ray) < 1e-3);
    }

    // Near parallel ray converging onto the x-axis capsule from far away.
    let capsule = ray_capsule();
    let mut second = RayCastInput::default();
    second.origin = Vec3::new(-1000.0, 1.0001, 0.0);
    second.translation = Vec3::new(2000.0, -0.001, 0.0);
    second.max_fraction = 1.0;

    let out_b = ray_cast_capsule(&capsule, &second);
    assert!(out_b.hit);
    if out_b.hit {
        let on_seg = point_to_segment_distance(capsule.center1, capsule.center2, out_b.point);
        assert!(abs_float((distance(out_b.point, on_seg) - capsule.radius) as f64) < 1e-3);
    }

    // Exactly parallel and outside the cylinder still misses.
    let mut parallel = RayCastInput::default();
    parallel.origin = Vec3::new(0.0, 3.0, 0.0);
    parallel.translation = Vec3::new(8.0, 0.0, 0.0);
    parallel.max_fraction = 1.0;
    assert!(!ray_cast_capsule(&capsule, &parallel).hit);
}

#[test]
fn sphere_interior() {
    // "a ray starting inside the sphere hits at fraction zero"
    let sphere = Sphere::new(Vec3::ZERO, 1.0);

    let mut input = RayCastInput::default();
    input.origin = Vec3::ZERO;
    input.translation = Vec3::new(1.0, 0.0, 0.0);
    input.max_fraction = 1.0;

    let out = ray_cast_sphere(&sphere, &input);
    // Origin inside reports the origin with zero fraction.
    assert!(out.hit);
    assert_eq!(out.fraction, 0.0);
    assert!(distance(out.point, input.origin) < f32::EPSILON);

    // A zero length ray while inside still reports the origin.
    let mut still = RayCastInput::default();
    still.origin = Vec3::new(0.5, 0.0, 0.0);
    still.translation = Vec3::ZERO;
    still.max_fraction = 1.0;
    let still_out = ray_cast_sphere(&sphere, &still);
    assert!(still_out.hit);
    assert!(distance(still_out.point, still.origin) < f32::EPSILON);

    // A zero length ray outside misses.
    let mut outside = RayCastInput::default();
    outside.origin = Vec3::new(3.0, 0.0, 0.0);
    outside.translation = Vec3::ZERO;
    outside.max_fraction = 1.0;
    assert!(!ray_cast_sphere(&sphere, &outside).hit);
}

#[test]
fn sphere_graze() {
    // "a grazing ray reports a tangent hit"
    let sphere = Sphere::new(Vec3::ZERO, 1.0);

    // Travel along y = 1 exactly, so the ray is tangent to the sphere.
    let mut input = RayCastInput::default();
    input.origin = Vec3::new(-5.0, 1.0, 0.0);
    input.translation = Vec3::new(10.0, 0.0, 0.0);
    input.max_fraction = 1.0;

    let out = ray_cast_sphere(&sphere, &input);
    assert!(out.hit);
    if out.hit {
        // The hit is at the top of the sphere.
        assert!(abs_float(out.point.x as f64) < 1e-3);
        assert!(abs_float((out.point.y - 1.0) as f64) < 1e-3);
        assert!(abs_float((out.fraction - 0.5) as f64) < 1e-3);
    }
}
