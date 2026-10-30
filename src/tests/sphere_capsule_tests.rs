//! Ported from Tests/NatselTests/SphereCapsuleTests.swift.
//!
//! Sphere and capsule mass/AABB/ray-cast/shape-cast helpers must be deterministic and match
//! the analytic expectations.

use crate::base::constants::PI;
use crate::collision::capsule::{
    compute_capsule_aabb, compute_capsule_mass, ray_cast_capsule, shape_cast_capsule,
};
use crate::collision::sphere::{
    compute_sphere_aabb, compute_sphere_mass, ray_cast_hollow_sphere, ray_cast_sphere,
    shape_cast_sphere,
};
use crate::math::types::{Capsule, Sphere, Transform, Vec3};
use crate::types::queries::{CastOutput, RayCastInput, ShapeCastInput, ShapeProxy};

fn bits_f32(x: f32) -> u64 {
    x.to_bits() as u64
}

#[test]
fn sphere_mass() {
    // "sphere mass is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let shape = Sphere::new(Vec3::new(0.1, 0.2, -0.3), 0.5);
        let mass = compute_sphere_mass(&shape, 2.0);
        let expected_mass = 2.0 * (4.0 / 3.0) * PI * shape.radius * shape.radius * shape.radius;
        assert!((mass.mass - expected_mass).abs() < 0.000001);
        assert_eq!(mass.center, shape.center);
        assert!(
            (mass.inertia.cx.x - 0.4 * mass.mass * shape.radius * shape.radius).abs() < 0.000001
        );

        samples.push(bits_f32(mass.mass));
        samples.push(bits_f32(mass.center.x));
        samples.push(bits_f32(mass.inertia.cx.x));
        samples.push(bits_f32(mass.inertia.cy.y));
        samples.push(bits_f32(mass.inertia.cz.z));

        let aabb = compute_sphere_aabb(&shape, Transform::IDENTITY);
        samples.push(bits_f32(aabb.lower_bound.x));
        samples.push(bits_f32(aabb.upper_bound.y));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn capsule_mass() {
    // "capsule mass is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let shape = Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5);
        let mass = compute_capsule_mass(&shape, 2.0);
        let radius = shape.radius;
        // TODO(migration): confirm `Vec3::length` exists; otherwise use `math::functions::length`.
        let height = (shape.center2 - shape.center1).length();
        let expected_mass = 2.0 * PI * radius * radius * (height + (4.0 / 3.0) * radius);
        assert!((mass.mass - expected_mass).abs() < 0.000001);

        samples.push(bits_f32(mass.mass));
        assert_eq!(mass.center, Vec3::new(0.0, 0.0, 0.0));
        samples.push(bits_f32(mass.inertia.cx.x));
        samples.push(bits_f32(mass.inertia.cy.y));
        samples.push(bits_f32(mass.inertia.cz.z));

        let aabb = compute_capsule_aabb(&shape, Transform::IDENTITY);
        samples.push(bits_f32(aabb.lower_bound.x));
        samples.push(bits_f32(aabb.upper_bound.y));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn sphere_ray_cast() {
    // "sphere ray cast is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let shape = Sphere::new(Vec3::new(0.1, 0.2, -0.3), 0.5);
        let mut input = RayCastInput::new(Vec3::new(-3.0, 0.5, 0.0), Vec3::new(6.0, 0.0, 0.0), 1.0);

        let output = ray_cast_sphere(&shape, &input);
        assert!(output.hit);
        samples.push(bits_f32(output.fraction));
        samples.push(bits_f32(output.point.x));
        samples.push(bits_f32(output.point.y));
        samples.push(bits_f32(output.point.z));
        samples.push(bits_f32(output.normal.x));
        samples.push(bits_f32(output.normal.y));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn hollow_sphere() {
    // "hollow sphere only hits from inside within range"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let shape = Sphere::new(Vec3::new(0.1, 0.2, -0.3), 0.5);

        let mut outside = RayCastInput::new(Vec3::new(-3.0, 0.5, 0.0), Vec3::new(6.0, 0.0, 0.0), 1.0);
        assert!(!ray_cast_hollow_sphere(&shape, &outside).hit);

        let mut inside =
            RayCastInput::new(Vec3::new(0.1, 0.2, -0.3), Vec3::new(2.0, 0.5, 0.25), 1.0);
        let output = ray_cast_hollow_sphere(&shape, &inside);
        assert!(output.hit);
        samples.push(bits_f32(output.fraction));
        samples.push(bits_f32(output.point.x));
        samples.push(bits_f32(output.normal.x));
        samples.push(bits_f32(output.normal.z));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn capsule_ray_cast() {
    // "capsule ray cast is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let shape = Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5);
        let mut input = RayCastInput::new(Vec3::new(2.0, 0.2, 0.0), Vec3::new(-4.0, 0.0, 0.0), 1.0);

        let output = ray_cast_capsule(&shape, &input);
        assert!(output.hit);
        samples.push(bits_f32(output.fraction));
        samples.push(bits_f32(output.point.x));
        samples.push(bits_f32(output.point.y));
        samples.push(bits_f32(output.normal.x));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn shape_casts() {
    // "shape casts of sphere and capsule report initial overlap"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut input = ShapeCastInput::default();
        input.translation = Vec3::new(-5.0, 0.0, 0.0);
        input.max_fraction = 1.0;

        let sphere = Sphere::new(Vec3::new(0.1, 0.2, -0.3), 0.5);
        // Swift built the proxy from an 8-point cube buffer; replicate it here.
        let buffer: [Vec3; 8] = [
            Vec3::new(1.0, 1.0, 1.0),
            Vec3::new(-1.0, 1.0, 1.0),
            Vec3::new(-1.0, -1.0, 1.0),
            Vec3::new(1.0, -1.0, 1.0),
            Vec3::new(1.0, 1.0, -1.0),
            Vec3::new(-1.0, 1.0, -1.0),
            Vec3::new(-1.0, -1.0, -1.0),
            Vec3::new(1.0, -1.0, -1.0),
        ];
        input.proxy = ShapeProxy::new(Some(buffer.as_ptr()), 8, 0.0);
        let sphere_out = shape_cast_sphere(&sphere, &input);
        assert!(sphere_out.hit);
        assert_eq!(sphere_out.fraction.to_bits(), 0);
        samples.push(bits_f32(sphere_out.point.x));

        let capsule =
            Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5);
        input.proxy = ShapeProxy::new(Some(buffer.as_ptr()), 8, 0.0);
        let capsule_out = shape_cast_capsule(&capsule, &input);
        assert!(capsule_out.hit);
        assert_eq!(capsule_out.fraction.to_bits(), 0);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
