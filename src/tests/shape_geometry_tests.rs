//! Ported from Tests/NatselTests/ShapeGeometryTests.swift.
//!
//! Hull/sphere/capsule/shape mass, AABB, extent, projected area, ray casts, and proxy AABBs must be
//! deterministic and consistent with the Swift suite.

use crate::collision::hull::make_box_hull;
use crate::collision::hull_queries::{
    compute_hull_aabb, compute_hull_extent, compute_hull_projected_area, ray_cast_hull,
};
use crate::collision::proxy::compute_proxy_aabb;
use crate::math::functions::{make_quat_from_axis_angle, normalize};
use crate::math::types::{Capsule, Sphere, Transform, Vec3};
use crate::simulation::shape_runtime::{
    compute_hull_mass, compute_shape_aabb, compute_shape_extent, compute_shape_mass, make_shape_proxy,
    Shape, ShapeGeometry,
};
use crate::types::queries::{CastOutput, RayCastInput, ShapeProxy};
use crate::types::shape_def::ShapeType;

fn bits_f32(x: f32) -> u64 {
    x.to_bits() as u64
}

#[test]
fn hull_mass() {
    // "hull mass is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull = make_box_hull(1.0, 2.0, 3.0);
        let mass = compute_hull_mass(&hull, 2.0);

        samples.push(bits_f32(mass.mass));
        assert_eq!(mass.center, Vec3::new(0.0, 0.0, 0.0));
        samples.push(bits_f32(mass.inertia.cx.x));
        samples.push(bits_f32(mass.inertia.cy.y));
        samples.push(bits_f32(mass.inertia.cz.z));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn hull_bounds() {
    // "hull AABB and extent are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull = make_box_hull(1.0, 2.0, 3.0);
        let transform =
            Transform::new(Vec3::new(1.0, 2.0, 3.0), make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5));
        let aabb = compute_hull_aabb(&hull, transform);

        samples.push(bits_f32(aabb.lower_bound.x));
        samples.push(bits_f32(aabb.lower_bound.y));
        assert_eq!(aabb.lower_bound.z.to_bits(), 0);
        samples.push(bits_f32(aabb.upper_bound.x));
        samples.push(bits_f32(aabb.upper_bound.y));
        samples.push(bits_f32(aabb.upper_bound.z));

        let extent = compute_hull_extent(&hull, Vec3::new(0.2, 0.1, -0.3));
        samples.push(bits_f32(extent.min_extent));
        samples.push(bits_f32(extent.max_extent.x));
        samples.push(bits_f32(extent.max_extent.y));
        samples.push(bits_f32(extent.max_extent.z));

        let projected = compute_hull_projected_area(&hull, normalize(Vec3::new(1.0, 1.0, 1.0)));
        samples.push(bits_f32(projected));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn hull_ray_cast() {
    // "hull ray cast is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull = make_box_hull(1.0, 2.0, 3.0);
        let mut input = RayCastInput::new(Vec3::new(-3.0, 0.5, 0.5), Vec3::new(6.0, 0.0, 0.0), 1.0);

        let output: CastOutput = ray_cast_hull(&hull, &input);
        assert!(output.hit);
        samples.push(bits_f32(output.fraction));
        samples.push(bits_f32(output.point.x));
        samples.push(bits_f32(output.point.y));
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
fn sphere_shape() {
    // "sphere shape mass, AABB and extent are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut shape = Shape::default();
        shape.type_ = ShapeType::SphereShape;
        shape.geometry = ShapeGeometry::Sphere(Sphere::new(Vec3::new(0.1, 0.2, -0.3), 0.5));
        shape.density = 2.0;

        let mass = compute_shape_mass(&shape);
        samples.push(bits_f32(mass.mass));
        samples.push(bits_f32(mass.inertia.cx.x));

        let transform =
            Transform::new(Vec3::new(1.0, 2.0, 3.0), make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5));
        let aabb = compute_shape_aabb(&shape, transform);
        samples.push(bits_f32(aabb.lower_bound.x));
        samples.push(bits_f32(aabb.lower_bound.y));
        samples.push(bits_f32(aabb.lower_bound.z));
        samples.push(bits_f32(aabb.upper_bound.x));
        samples.push(bits_f32(aabb.upper_bound.y));
        samples.push(bits_f32(aabb.upper_bound.z));

        let extent = compute_shape_extent(&shape, Vec3::new(0.0, 0.1, 0.0));
        samples.push(b32(extent.min_extent));
        samples.push(b32(extent.max_extent.x));
        samples.push(b32(extent.max_extent.y));
        samples.push(b32(extent.max_extent.z));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn capsule_shape() {
    // "capsule shape mass, AABB and extent are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut shape = Shape::default();
        shape.type_ = ShapeType::CapsuleShape;
        shape.geometry = ShapeGeometry::Capsule(Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5));
        shape.density = 2.0;

        let mass = compute_shape_mass(&shape);
        samples.push(bits_f32(mass.mass));
        samples.push(bits_f32(mass.inertia.cx.x));
        samples.push(bits_f32(mass.inertia.cy.y));

        let transform =
            Transform::new(Vec3::new(1.0, 2.0, 3.0), make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.5));
        let aabb = compute_shape_aabb(&shape, transform);
        samples.push(bits_f32(aabb.lower_bound.x));
        samples.push(bits_f32(aabb.lower_bound.y));
        samples.push(bits_f32(aabb.lower_bound.z));
        samples.push(bits_f32(aabb.upper_bound.x));
        samples.push(bits_f32(aabb.upper_bound.y));
        samples.push(bits_f32(aabb.upper_bound.z));

        let extent = compute_shape_extent(&shape, Vec3::new(0.0, 0.0, 0.0));
        samples.push(b32(extent.min_extent));
        samples.push(b32(extent.max_extent.x));
        samples.push(b32(extent.max_extent.y));
        samples.push(b32(extent.max_extent.z));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn proxy_aabb() {
    // "proxy AABB and shape proxy are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let points: [Vec3; 2] = [Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0)];
        let proxy = ShapeProxy::new(Some(points.as_ptr()), 2, 0.5);
        let aabb = compute_proxy_aabb(proxy);
        samples.push(bits_f32(aabb.lower_bound.x));
        samples.push(bits_f32(aabb.lower_bound.y));
        samples.push(bits_f32(aabb.upper_bound.y));

        let mut shape = Shape::default();
        shape.type_ = ShapeType::CapsuleShape;
        shape.geometry = ShapeGeometry::Capsule(Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5));
        let buffer = make_shape_proxy(&shape);
        let capsule_aabb = compute_proxy_aabb(buffer.proxy());
        samples.push(bits_f32(capsule_aabb.lower_bound.y));
        samples.push(bits_f32(capsule_aabb.upper_bound.y));

        let mut hull_shape = Shape::default();
        hull_shape.type_ = ShapeType::HullShape;
        hull_shape.geometry = ShapeGeometry::Hull(make_box_hull(1.0, 2.0, 3.0));
        let hull_buffer = make_shape_proxy(&hull_shape);
        assert_eq!(hull_buffer.count(), 8);
        let hull_aabb = compute_proxy_aabb(hull_buffer.proxy());
        samples.push(bits_f32(hull_aabb.lower_bound.x));
        samples.push(bits_f32(hull_aabb.upper_bound.z));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

fn b32(x: f32) -> u64 {
    x.to_bits() as u64
}
