//! Ported from Tests/NatselTests/Simulation/CompoundCountTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Transform, Vec3};
use crate::simulation::World;
use crate::collision::compound::compound::Compound;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{CompoundDef, CompoundHullDef};
use crate::types::shape_def::{ShapeDef, SurfaceMaterial};

fn make_compound() -> Option<Compound> {
    let mut definition = CompoundDef::default();
    let mut item = CompoundHullDef::default();
    item.hull = Some(make_box_hull(1.0, 1.0, 1.0));
    item.transform = Transform::IDENTITY;
    item.material = SurfaceMaterial::default_material();
    definition.hulls.push(item);
    Compound::new(&definition)
}

#[test]
fn destroy_shape_releases() {
    let mut world = World::default();
    assert_eq!(world.compound_shape_count_value(), 0);

    let compound = make_compound().expect("compound creation failed");
    let def = BodyDef::default();
    let body = world.create_body(&def);
    let shape_def = ShapeDef::default();

    let shape = world.create_compound_shape(body, &shape_def, &compound);
    assert_eq!(world.compound_shape_count_value(), 1);

    world.destroy_shape(shape, false);
    assert_eq!(world.compound_shape_count_value(), 0);

    world.destroy_body(body);
}

#[test]
fn destroy_body_releases() {
    let mut world = World::default();
    let compound = make_compound().expect("compound creation failed");
    let def = BodyDef::default();
    let body = world.create_body(&def);
    let shape_def = ShapeDef::default();

    let _ = world.create_compound_shape(body, &shape_def, &compound);
    assert_eq!(world.compound_shape_count_value(), 1);

    world.destroy_body(body);
    assert_eq!(world.compound_shape_count_value(), 0);
}

#[test]
fn multiple_compounds() {
    let mut world = World::default();
    let compound = make_compound().expect("compound creation failed");

    let mut bodies = Vec::new();
    for _ in 0..3 {
        let def = BodyDef::default();
        let body = world.create_body(&def);
        let _ = world.create_compound_shape(body, &ShapeDef::default(), &compound);
        bodies.push(body);
    }
    assert_eq!(world.compound_shape_count_value(), 3);

    world.destroy_body(bodies[1]);
    assert_eq!(world.compound_shape_count_value(), 2);

    world.destroy_body(bodies[0]);
    world.destroy_body(bodies[2]);
    assert_eq!(world.compound_shape_count_value(), 0);
}
