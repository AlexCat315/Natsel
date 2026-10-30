//! Ported from Tests/NatselTests/Collision/Mover/PrimitivePlaneTests.swift.
//!
//! The upstream primitive mover cases: sphere, capsule, and hull, each in separated, touching, and
//! deep-overlap (or crossing/parallel) configurations.

use crate::collision::capsule::collide_mover_and_capsule;
use crate::collision::hull::make_box_hull;
use crate::collision::hull_mover::collide_mover_and_hull;
use crate::collision::sphere::collide_mover_and_sphere;
use crate::math::functions::is_normalized;
use crate::math::types::Vec3;
use crate::types::geometry::{Capsule, Sphere};
use crate::types::queries::PlaneResult;

#[test]
fn sphere() {
    // "sphere separated, touching, and deep overlap"
    // Swift's `arguments: 0..<3` expands to one case per index.
    let movers = [
        Capsule::new(Vec3::new(4.0, 3.0, 0.0), Vec3::new(6.0, 3.0, 0.0), 0.2),
        Capsule::new(Vec3::new(-1.0, 0.6, 0.0), Vec3::new(1.0, 0.6, 0.0), 0.2),
        Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.2),
    ];
    for index in 0..3usize {
        let mut plane = PlaneResult::default();
        let count = collide_mover_and_sphere(
            &mut plane,
            &Sphere::new(Vec3::ZERO, 0.5),
            &movers[index],
        );
        assert_eq!(count, if index == 0 { 0 } else { 1 }, "index {index}");
        if index > 0 {
            assert!(is_normalized(plane.plane.normal), "index {index}");
            let expected = if index == 1 { 0.1 } else { 0.7 };
            assert!(
                (plane.plane.offset - expected).abs() < 0.00001,
                "index {index}"
            );
            if index == 1 {
                assert!(plane.plane.normal.y > 0.99);
            }
            if index == 2 {
                assert!(plane.plane.normal.x.abs() < 0.00001);
            }
        }
    }
}

#[test]
fn capsule() {
    // "capsule separated, touching, crossing, and parallel overlap"
    // Swift's `arguments: 0..<4` expands to one case per index.
    let movers = [
        Capsule::new(Vec3::new(-1.0, 5.0, 0.0), Vec3::new(1.0, 5.0, 0.0), 0.2),
        Capsule::new(Vec3::new(-1.0, 0.4, 0.0), Vec3::new(1.0, 0.4, 0.0), 0.2),
        Capsule::new(Vec3::new(0.0, 0.0, -1.0), Vec3::new(0.0, 0.0, 1.0), 0.2),
        Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.2),
    ];
    let shape = Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.3);
    for index in 0..4usize {
        let mut plane = PlaneResult::default();
        let count = collide_mover_and_capsule(&mut plane, &shape, &movers[index]);
        assert_eq!(count, if index == 0 { 0 } else { 1 }, "index {index}");
        if index > 0 {
            assert!(is_normalized(plane.plane.normal), "index {index}");
            let expected = if index == 1 { 0.1 } else { 0.5 };
            assert!(
                (plane.plane.offset - expected).abs() < 0.00001,
                "index {index}"
            );
            if index == 1 {
                assert!(plane.plane.normal.y > 0.99);
            }
            if index >= 2 {
                assert!(plane.plane.normal.x.abs() < 0.00001, "index {index}");
            }
            if index == 2 {
                assert!(plane.plane.normal.z.abs() < 0.00001);
            }
        }
    }
}

#[test]
fn hull() {
    // "hull separated, touching, and deep overlap"
    // Swift's `arguments: 0..<3` expands to one case per index.
    let movers = [
        Capsule::new(Vec3::new(-0.3, 5.0, 0.0), Vec3::new(0.3, 5.0, 0.0), 0.2),
        Capsule::new(Vec3::new(-0.3, 0.6, 0.0), Vec3::new(0.3, 0.6, 0.0), 0.2),
        Capsule::new(Vec3::new(-0.2, 0.0, 0.0), Vec3::new(0.2, 0.0, 0.0), 0.1),
    ];
    for index in 0..3usize {
        let mut plane = PlaneResult::default();
        let count =
            collide_mover_and_hull(&mut plane, &make_box_hull(0.5, 0.5, 0.5), &movers[index]);
        assert_eq!(count, if index == 1 { 1 } else { 0 }, "index {index}");
        if index == 1 {
            assert!(is_normalized(plane.plane.normal));
            assert!(plane.plane.normal.y > 0.99);
            assert!((plane.plane.offset - 0.1).abs() < 0.0001);
        }
    }
}
