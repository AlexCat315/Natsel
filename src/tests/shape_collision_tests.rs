//! Ported from Tests/NatselTests/ShapeCollisionTests.swift.
//!
//! Primitive-vs-primitive contact manifolds must be byte-identical across two runs.

use crate::collision::convex_manifold_shapes::{
    collide_capsule_and_sphere, collide_capsules, collide_hull_and_capsule, collide_hull_and_sphere,
    collide_spheres,
};
use crate::collision::hull::make_box_hull;
use crate::math::types::{Capsule, Quat, Sphere, Transform, Vec3};
use crate::types::collision_types::{LocalManifold, LocalManifoldPoint};
use crate::types::queries::SimplexCache;

fn with_manifold(capacity: i32, body: impl FnOnce(&mut LocalManifold)) -> (Vec3, Vec<LocalManifoldPoint>) {
    let mut buffer = vec![LocalManifoldPoint::default(); capacity as usize];
    let mut manifold = LocalManifold::default();
    manifold.points = buffer.as_mut_ptr();
    body(&mut manifold);
    let count = manifold.point_count as usize;
    (manifold.normal, buffer[0..count].to_vec())
}

#[test]
fn spheres() {
    // "sphere-sphere manifold is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let (normal, points) = with_manifold(1, |manifold| {
            let a = Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0);
            let b = Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0);
            collide_spheres(manifold, 1, a, b, Transform::new(Vec3::new(1.5, 0.0, 0.0), Quat::IDENTITY));
        });
        samples.push(normal.x.to_bits() as u64);
        assert_eq!(normal.y.to_bits(), 0);
        assert_eq!(normal.z.to_bits(), 0);
        assert_eq!(points.len(), 1);
        samples.push(points[0].point.x.to_bits() as u64);
        assert_eq!(points[0].point.y.to_bits(), 0);
        samples.push(points[0].separation.to_bits() as u64);
        assert!(points[0].pair.owner1 == 0 && points[0].pair.index1 == 0);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn capsules() {
    // "capsule-capsule manifold is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let (normal, points) = with_manifold(2, |manifold| {
            let a = Capsule::new(Vec3::new(0.0, -1.0, 0.0), Vec3::new(0.0, 1.0, 0.0), 0.5);
            let b = a;
            collide_capsules(
                manifold,
                2,
                a,
                b,
                Transform::new(Vec3::new(0.8, 0.0, 0.0), Quat::IDENTITY),
            );
        });
        samples.push(normal.x.to_bits() as u64);
        samples.push(normal.y.to_bits() as u64);
        samples.push(normal.z.to_bits() as u64);
        assert_eq!(points.len(), 2);
        samples.push(points[0].point.x.to_bits() as u64);
        samples.push(points[0].point.y.to_bits() as u64);
        samples.push(points[0].point.z.to_bits() as u64);
        samples.push(points[0].separation.to_bits() as u64);
        samples.push(points[1].point.x.to_bits() as u64);
        samples.push(points[1].point.y.to_bits() as u64);
        samples.push(points[1].point.z.to_bits() as u64);
        samples.push(points[1].separation.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn capsule_sphere() {
    // "capsule-sphere manifold is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let (normal, points) = with_manifold(1, |manifold| {
            let a = Capsule::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0), 0.5);
            let b = Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0);
            collide_capsule_and_sphere(
                manifold,
                1,
                a,
                b,
                Transform::new(Vec3::new(1.2, 0.3, 0.0), Quat::IDENTITY),
            );
        });
        samples.push(normal.x.to_bits() as u64);
        samples.push(normal.y.to_bits() as u64);
        samples.push(normal.z.to_bits() as u64);
        assert_eq!(points.len(), 1);
        samples.push(points[0].point.x.to_bits() as u64);
        samples.push(points[0].point.y.to_bits() as u64);
        samples.push(points[0].point.z.to_bits() as u64);
        samples.push(points[0].separation.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn hull_sphere() {
    // "hull-sphere manifold is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull = make_box_hull(1.0, 1.0, 1.0);
        let mut cache = SimplexCache::default();
        let (normal, points) = with_manifold(1, |manifold| {
            let sphere = Sphere::new(Vec3::new(0.0, 0.0, 0.0), 1.0);
            collide_hull_and_sphere(
                manifold,
                1,
                &hull,
                sphere,
                Transform::new(Vec3::new(1.2, 0.5, 0.0), Quat::IDENTITY),
                &mut cache,
            );
        });
        samples.push(normal.x.to_bits() as u64);
        samples.push(normal.y.to_bits() as u64);
        samples.push(normal.z.to_bits() as u64);
        assert_eq!(points.len(), 1);
        samples.push(points[0].point.x.to_bits() as u64);
        samples.push(points[0].point.y.to_bits() as u64);
        samples.push(points[0].point.z.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn hull_capsule() {
    // "hull-capsule manifold is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull = make_box_hull(1.0, 1.0, 1.0);
        let mut cache = SimplexCache::default();
        let (normal, points) = with_manifold(2, |manifold| {
            let capsule = Capsule::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0), 0.25);
            collide_hull_and_capsule(
                manifold,
                2,
                &hull,
                capsule,
                Transform::new(Vec3::new(1.1, 0.0, 0.0), Quat::IDENTITY),
                &mut cache,
            );
        });
        samples.push(normal.x.to_bits() as u64);
        samples.push(normal.y.to_bits() as u64);
        samples.push(normal.z.to_bits() as u64);
        assert_eq!(points.len(), 2);
        samples.push(points[0].point.x.to_bits() as u64);
        samples.push(points[0].point.y.to_bits() as u64);
        samples.push(points[0].point.z.to_bits() as u64);
        samples.push(points[0].separation.to_bits() as u64);
        samples.push(points[1].point.x.to_bits() as u64);
        samples.push(points[1].point.y.to_bits() as u64);
        samples.push(points[1].point.z.to_bits() as u64);
        samples.push(points[1].separation.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
