//! Ported from Tests/NatselTests/Simulation/HullDatabaseTests.swift.

use crate::collision::hull::make_box_hull;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::shape_def::ShapeDef;

// "identical hulls share one interned copy"
#[test]
fn deduplication() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body_a = world.create_body(&def);
    let body_b = world.create_body(&def);
    let shape_def = ShapeDef::default();

    // Two shapes built from identical data share one owned copy.
    let shape_a = world.create_hull_shape(body_a, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    let shape_b = world.create_hull_shape(body_b, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    assert_eq!(world.hull_database_count(), 1);

    // A box built on an independent stack frame must de-duplicate to the same shared copy.
    // This only holds if content hashing sees deterministic padding bytes.
    let body_c = world.create_body(&def);
    let shape_c = world.create_hull_shape(body_c, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    assert_eq!(world.hull_database_count(), 1);

    world.destroy_shape(shape_c, false);
    assert_eq!(world.hull_database_count(), 1);

    // A different size is a separate entry.
    let body_d = world.create_body(&def);
    let shape_d = world.create_hull_shape(body_d, &shape_def, &make_box_hull(0.3, 0.3, 0.3));
    assert_eq!(world.hull_database_count(), 2);
    world.destroy_shape(shape_d, false);
    assert_eq!(world.hull_database_count(), 1);

    // Releasing one reference keeps the other alive.
    world.destroy_shape(shape_a, false);
    assert_eq!(world.hull_database_count(), 1);

    world.destroy_shape(shape_b, false);
    assert_eq!(world.hull_database_count(), 0);
}

// "destroying bodies drains the database"
#[test]
fn destroy_body_drains() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let bodies: Vec<_> = (0..4).map(|_| world.create_body(&def)).collect();

    let shape_def = ShapeDef::default();
    for body in &bodies {
        let _ = world.create_hull_shape(*body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    }
    assert_eq!(world.hull_database_count(), 1);

    for body in &bodies {
        world.destroy_body(*body);
    }
    assert_eq!(world.hull_database_count(), 0);
}

// "interning is content keyed across many distinct hulls"
#[test]
fn distinct_hulls() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let shape_def = ShapeDef::default();

    // Ten different sizes, each added twice.
    for i in 1..=10 {
        let half = i as f32 * 0.1;
        for _ in 0..2 {
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(half, half, half));
        }
    }

    assert_eq!(world.hull_database_count(), 10);
}
