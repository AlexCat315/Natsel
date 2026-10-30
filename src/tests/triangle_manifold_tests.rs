//! Ported from Tests/NatselTests/TriangleManifoldTests.swift.
//!
//! Triangle-vs-primitive contact manifolds must be byte-identical across two runs.

use crate::collision::hull::make_box_hull;
use crate::collision::triangle_manifold::{
    collide_triangle_and_capsule, collide_triangle_and_hull, collide_triangle_and_sphere,
};
use crate::math::types::{Quat, Transform, Vec3};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::collision_types::{LocalManifold, LocalManifoldPoint, SeparatingFeature, TriangleFeature};
use crate::types::collision_types::SATCache;
use crate::types::queries::SimplexCache;

fn with_manifold(
    capacity: i32,
    body: impl FnOnce(&mut LocalManifold),
) -> (Vec3, i32, Vec<LocalManifoldPoint>) {
    let mut buffer = vec![LocalManifoldPoint::default(); capacity as usize];
    let mut manifold = LocalManifold::default();
    manifold.points = buffer.as_mut_ptr();
    body(&mut manifold);
    let count = manifold.point_count as usize;
    (manifold.normal, manifold.feature as u8 as i32, buffer[0..count].to_vec())
}

#[test]
fn triangle_sphere() {
    // "triangle-sphere manifold is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ];
        let result = with_manifold(1, |manifold| {
            let sphere = Sphere::new(Vec3::new(0.2, 0.5, 0.2), 0.5);
            collide_triangle_and_sphere(manifold, 1, &tri, sphere);
        });
        samples.push(result.0.x.to_bits() as u64);
        samples.push(result.0.y.to_bits() as u64);
        samples.push(result.0.z.to_bits() as u64);
        assert_eq!(result.1, 1);
        assert_eq!(result.2.len(), 1);
        samples.push(result.2[0].point.x.to_bits() as u64);
        assert_eq!(result.2[0].point.y.to_bits(), 0);
        assert_eq!(result.2[0].separation.to_bits(), 0);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn triangle_capsule() {
    // "triangle-capsule manifold is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ];
        let result = with_manifold(2, |manifold| {
            let mut cache = SimplexCache::default();
            let capsule = Capsule::new(Vec3::new(0.2, 0.25, 0.2), Vec3::new(0.2, 0.25, 0.2), 0.3);
            collide_triangle_and_capsule(manifold, 2, &tri, capsule, &mut cache);
        });
        samples.push(result.0.y.to_bits() as u64);
        assert_eq!(result.1, 1);
        assert_eq!(result.2.len(), 2);
        samples.push(result.2[0].point.x.to_bits() as u64);
        samples.push(result.2[0].point.y.to_bits() as u64);
        samples.push(result.2[0].separation.to_bits() as u64);
        samples.push(result.2[1].point.x.to_bits() as u64);
        samples.push(result.2[1].point.y.to_bits() as u64);
        samples.push(result.2[1].separation.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn triangle_capsule2() {
    // "triangle-capsule (along-edge) manifold is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let tri = [
            Vec3::new(0.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
            Vec3::new(1.0, 0.0, 0.0),
        ];
        let result = with_manifold(2, |manifold| {
            let mut cache = SimplexCache::default();
            let capsule = ::new(Vec3::new(0.1, 0.1, 0.1), Vec3::new(0.6, 0.1, 0.1), 0.2);
            collide_triangle_and_capsule(manifold, 2, &tri, capsule, &mut cache);
        });
        assert_eq!(result.2.len(), 2);
        samples.push(result.2[0].point.x.to_bits() as u64);
        samples.push(result.2[0].point.y.to_bits() as u64);
        samples.push(result.2[0].separation.to_bits() as u64);
        samples.push(result.2[1].point.x.to_bits() as u64);
        samples.push(result.2[1].point.y.to_bits() as u64);
        samples.push(result.2[1].separation.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn triangle_hull_separated() {
    // "triangle-hull separation is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull = make_box_hull(1.0, 1.0, 1.0);
        let tri_h = [
            Vec3::new(-0.5, 1.2, -0.5),
            Vec3::new(0.5, 1.2, -0.5),
            Vec3::new(0.0, 1.2, 0.5),
        ];
        let mut cache = SATCache::default();
        let result = with_manifold(4, |manifold| {
            collide_triangle_and_hull(manifold, 4, tri_h[0], tri_h[1], tri_h[2], 0, &hull, &mut cache, true);
        });
        assert_eq!(result.2.len(), 0);
        assert_eq!(cache.type_, SeparatingFeature::FaceAxisA);
        samples.push(cache.separation.to_bits() as u64);
        assert_eq!(cache.hit, 0);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn triangle_hull_contact() {
    // "triangle-hull contact is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull = make_box_hull(1.0, 1.0, 1.0);
        let tri_h = [
            Vec3::new(-0.4, 1.05, -0.4),
            Vec3::new(0.4, 1.0, -0.3),
            Vec3::new(0.3, 1.15, 0.4),
        ];
        let mut cache = SATCache::default();
        let result = with_manifold(4, |manifold| {
            collide_triangle_and_hull(manifold, 4, tri_h[0], tri_h[1], tri_h[2], 0, &hull, &mut cache, true);
        });
        samples.push(result.0.x.to_bits() as u64);
        samples.push(result.0.y.to_bits() as u64);
        samples.push(result.0.z.to_bits() as u64);
        assert_eq!(result.1, 2);
        assert_eq!(result.2.len(), 3);
        samples.push(result.2[0].point.x.to_bits() as u64);
        samples.push(result.2[0].point.y.to_bits() as u64);
        samples.push(result.2[0].point.z.to_bits() as u64);
        samples.push(result.2[0].separation.to_bits() as u64);
        samples.push(result.2[1].point.x.to_bits() as u64);
        samples.push(result.2[1].point.y.to_bits() as u64);
        samples.push(result.2[1].point.z.to_bits() as u64);
        samples.push(result.2[1].separation.to_bits() as u64);
        samples.push(result.2[2].point.x.to_bits() as u64);
        samples.push(result.2[2].point.y.to_bits() as u64);
        samples.push(result.2[2].point.z.to_bits() as u64);
        samples.push(result.2[2].separation.to_bits() as u64);
        assert_eq!(cache.type_, SeparatingFeature::FaceAxisB);
        assert_eq!(cache.index_a, 1);
        assert_eq!(cache.index_b, 3);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
