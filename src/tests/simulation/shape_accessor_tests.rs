//! Ported from Tests/NatselTests/Simulation/ShapeAccessorTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::types::Vec3;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::shape_def::{ShapeDef, ShapeType, SurfaceMaterial};

#[test]
fn type_and_owner() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);

    let shape_def = ShapeDef::default();
    let sphere_shape = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5));
    let capsule_shape = world.create_capsule_shape(
        body,
        &shape_def,
        Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.2),
    );
    let hull_shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    assert_eq!(world.shape_type(sphere_shape), Some(ShapeType::SphereShape));
    assert_eq!(world.shape_type(capsule_shape), Some(ShapeType::CapsuleShape));
    assert_eq!(world.shape_type(hull_shape), Some(ShapeType::HullShape));

    for shape in [sphere_shape, capsule_shape, hull_shape] {
        assert!(world.shape_is_valid(shape));
        assert_eq!(world.shape_body(shape).index1, body.index1);
    }
}

#[test]
fn geometry_access() {
    let mut world = World::default();
    world.enable_sleep = false;

    let def = BodyDef::default();
    let body = world.create_body(&def);
    let shape_def = ShapeDef::default();

    let sphere_shape = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(1.0, 2.0, 3.0), 0.75));
    assert_eq!(world.shape_sphere(sphere_shape).unwrap().radius, 0.75);
    assert!(world.shape_capsule(sphere_shape).is_none());
    assert!(world.shape_hull(sphere_shape).is_none());

    let capsule_shape = world.create_capsule_shape(
        body,
        &shape_def,
        Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), 0.2),
    );
    assert_eq!(world.shape_capsule(capsule_shape).unwrap().radius, 0.2);
    assert!(world.shape_sphere(capsule_shape).is_none());

    let hull_shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.4, 0.3));
    assert!(world.shape_hull(hull_shape).is_some());
    assert!(world.shape_capsule(hull_shape).is_none());
}

#[test]
fn material_editing() {
    let mut world = World::default();
    world.enable_sleep = false;

    let def = BodyDef::default();
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.base_material.friction = 0.3;
    shape_def.base_material.restitution = 0.1;
    shape_def.density = 500.0;
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    assert!((world.shape_friction(shape) - 0.3).abs() < 1e-6);
    assert!((world.shape_restitution(shape) - 0.1).abs() < 1e-6);
    assert!((world.shape_density(shape) - 500.0).abs() < 1e-6);

    world.set_shape_friction(shape, 0.85);
    world.set_shape_restitution(shape, 0.65);
    assert!((world.shape_friction(shape) - 0.85).abs() < 1e-6);
    assert!((world.shape_restitution(shape) - 0.65).abs() < 1e-6);

    let mut material = SurfaceMaterial::default_material();
    material.friction = 0.15;
    material.restitution = 0.9;
    material.rolling_resistance = 0.05;
    material.user_material_id = 42;
    world.set_shape_surface_material(shape, material);

    let back = world.shape_surface_material(shape);
    assert!((back.friction - 0.15).abs() < 1e-6);
    assert!((back.restitution - 0.9).abs() < 1e-6);
    assert!((back.rolling_resistance - 0.05).abs() < 1e-6);
    assert_eq!(back.user_material_id, 42);
}

#[test]
fn density_marks_dirty() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let mass_before = world.body_mass(body);
    world.set_shape_density(shape, 2000.0, true);

    world.step(1.0f32 / 60.0, 1);
    let mass_after = world.body_mass(body);
    assert!((mass_after - mass_before * 2.0).abs() < mass_after * 1e-3);
}

#[test]
fn event_flags() {
    let mut world = World::default();
    world.enable_sleep = false;

    let def = BodyDef::default();
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.enable_sensor_events = true;
    shape_def.enable_contact_events = true;
    shape_def.enable_hit_events = false;
    shape_def.enable_pre_solve_events = true;
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    assert!(world.shape_are_sensor_events_enabled(shape));
    assert!(world.shape_are_contact_events_enabled(shape));
    assert!(!world.shape_are_hit_events_enabled(shape));
    assert!(world.shape_are_pre_solve_events_enabled(shape));
}

#[test]
fn filter() {
    let mut world = World::default();
    world.enable_sleep = false;

    let def = BodyDef::default();
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.filter.category_bits = 0b0100;
    shape_def.filter.mask_bits = 0b1010;
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let filter = world.shape_filter(shape);
    assert_eq!(filter.category_bits, 0b0100);
    assert_eq!(filter.mask_bits, 0b1010);
}
