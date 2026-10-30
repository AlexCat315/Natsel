//! Ported from Tests/NatselTests/ConvexManifoldTests.swift.
//!
//! Hull-vs-hull contact manifolds (via `collide_hulls` + `SATCache`) must be byte-identical across
//! two runs. The cache warm-start must also reproduce the contact on re-run.

use crate::collision::convex_manifold::collide_hulls;
use crate::collision::hull::{make_box_hull, Hull};
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Quat, Transform, Vec3};
use crate::types::collision_types::{LocalManifold, LocalManifoldPoint, SATCache, SeparatingFeature};

fn collide(
    hull_a: &Hull,
    hull_b: &Hull,
    transform: Transform,
    cache: &mut SATCache,
) -> Vec<LocalManifoldPoint> {
    let mut buffer = [LocalManifoldPoint::default(); 4];
    let mut manifold = LocalManifold::default();
    manifold.points = buffer.as_mut_ptr();
    collide_hulls(&mut manifold, 4, hull_a, hull_b, transform, cache);
    let count = manifold.point_count as usize;
    buffer[0..count].to_vec()
}

fn manifold_normal(hull_a: &Hull, hull_b: &Hull, transform: Transform, cache: &mut SATCache) -> Vec3 {
    let mut buffer = [LocalManifoldPoint::default(); 4];
    let mut manifold = LocalManifold::default();
    manifold.points = buffer.as_mut_ptr();
    collide_hulls(&mut manifold, 4, hull_a, hull_b, transform, cache);
    manifold.normal
}

#[test]
fn face_contact() {
    // "face contact is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull_a = make_box_hull(1.0, 1.0, 1.0);
        let hull_b = make_box_hull(1.0, 1.0, 1.0);
        let mut cache = SATCache::default();

        let normal = manifold_normal(
            &hull_a,
            &hull_b,
            Transform::new(Vec3::new(1.5, 0.0, 0.0), Quat::IDENTITY),
            &mut cache,
        );
        samples.push(normal.x.to_bits() as u64);
        samples.push(normal.y.to_bits() as u64);
        samples.push(normal.z.to_bits() as u64);

        assert_eq!(cache.type_, SeparatingFeature::FaceAxisB);
        samples.push(cache.separation.to_bits() as u64);
        assert_eq!(cache.index_a, 0);
        assert_eq!(cache.index_b, 0);
        assert_eq!(cache.hit, 0);

        let points = collide(
            &hull_a,
            &hull_b,
            Transform::new(Vec3::new(1.5, 0.0, 0.0), Quat::IDENTITY),
            &mut cache,
        );
        assert_eq!(cache.hit, 1);
        assert_eq!(points.len(), 4);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn rotated_contact() {
    // "rotated edge contact is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let hull_a = make_box_hull(1.0, 1.0, 1.0);
        let hull_b = make_box_hull(1.0, 1.0, 1.0);
        let q = make_quat_from_axis_angle(Vec3::new(0.0, 1.0, 0.0), 0.785398163);
        let xf = Transform::new(Vec3::new(1.2, 0.7, 0.3), q);

        let mut cache = SATCache::default();
        let normal = manifold_normal(&hull_a, &hull_b, xf, &mut cache);
        samples.push(normal.x.to_bits() as u64);
        assert_eq!(normal.y.to_bits(), 0);
        assert_eq!(normal.z.to_bits(), 0);

        assert_eq!(cache.type_, SeparatingFeature::FaceAxisA);
        samples.push(cache.separation.to_bits() as u64);
        assert_eq!(cache.index_a, 1);
        assert_eq!(cache.index_b, 5);

        let mut cache2 = SATCache::default();
        let points = collide(&hull_a, &hull_b, xf, &mut cache2);
        assert_eq!(points.len(), 4);

        samples.push(points[0].point.x.to_bits() as u64);
        samples.push(points[0].point.y.to_bits() as u64);
        samples.push(points[0].point.z.to_bits() as u64);
        samples.push(points[0].separation.to_bits() as u64);

        samples.push(points[1].point.x.to_bits() as u64);
        samples.push(points[1].point.y.to_bits() as u64);
        samples.push(points[1].separation.to_bits() as u64);

        samples.push(points[2].point.x.to_bits() as u64);
        samples.push(points[2].point.y.to_bits() as u64);
        samples.push(points[2].point.z.to_bits() as u64);
        samples.push(points[2].separation.to_bits() as u64);

        samples.push(points[3].point.x.to_bits() as u64);
        samples.push(points[3].point.y.to_bits() as u64);
        samples.push(points[3].point.z.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
