//! Ported from Tests/NatselTests/HullTests.swift.
//!
//! Validates convex hull topology, properties, support queries, hashing, and cloning.

use crate::collision::hull::{
    find_hull_support_face, find_hull_support_vertex, is_valid_hull, make_box_hull,
};
// TODO(migration): `compare_hull_data` is currently private to `shape_creation`; make it
// `pub(crate)` during the final build-fix phase so this import resolves.
use crate::simulation::shape_creation::compare_hull_data;
use crate::math::types::Vec3;

#[test]
fn box_hull_properties() {
    // "box hull has the expected topology and properties"
    let hull = make_box_hull(1.0, 2.0, 3.0);

    assert_eq!(hull.vertex_count(), 8);
    assert_eq!(hull.half_edge_count(), 24);
    assert_eq!(hull.face_count(), 6);

    assert_eq!(hull.volume(), 48.0);
    assert_eq!(hull.surface_area(), 88.0);
    assert_eq!(hull.inner_radius(), 1.0);

    assert_eq!(hull.center(), Vec3::new(0.0, 0.0, 0.0));
    assert_eq!(hull.aabb().lower_bound, Vec3::new(-1.0, -2.0, -3.0));
    assert_eq!(hull.aabb().upper_bound, Vec3::new(1.0, 2.0, 3.0));

    assert!(is_valid_hull(&hull));
}

#[test]
fn box_hull_points() {
    // "box hull points use stable ordering"
    let hull = make_box_hull(1.0, 2.0, 3.0);
    let points = hull.points();

    assert_eq!(points.len(), 8);
    assert_eq!(points[0], Vec3::new(1.0, 2.0, 3.0));
    assert_eq!(points[1], Vec3::new(-1.0, 2.0, 3.0));
    assert_eq!(points[2], Vec3::new(-1.0, -2.0, 3.0));
    assert_eq!(points[6], Vec3::new(-1.0, -2.0, -3.0));
}

#[test]
fn support_vertex() {
    // "support vertex keeps the first maximum"
    let hull = make_box_hull(1.0, 2.0, 3.0);

    assert_eq!(find_hull_support_vertex(&hull, Vec3::new(1.0, 1.0, 1.0)), 0);
    assert_eq!(find_hull_support_vertex(&hull, Vec3::new(-1.0, -1.0, -1.0)), 6);
    assert_eq!(find_hull_support_vertex(&hull, Vec3::new(0.0, 0.0, -1.0)), 4);
}

#[test]
fn support_face() {
    // "support face picks the aligned plane"
    let hull = make_box_hull(1.0, 2.0, 3.0);

    assert_eq!(find_hull_support_face(&hull, Vec3::new(1.0, 0.0, 0.0)), 1);
    assert_eq!(find_hull_support_face(&hull, Vec3::new(0.0, -1.0, 0.0)), 2);
}

#[test]
fn hull_hash() {
    // "hull content hash is stable and clones compare equal"
    let a = make_box_hull(1.0, 2.0, 3.0);
    let b = make_box_hull(1.0, 2.0, 3.0);

    assert_ne!(a.hash(), 0);
    assert_eq!(a.hash(), b.hash());
    assert!(compare_hull_data(&a, &b));

    let clone = a.clone();
    assert_eq!(clone.hash(), a.hash());
    assert!(compare_hull_data(&a, &clone));
}
