//! Ported from Tests/NatselTests/Simulation/ContactEventTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

#[test]
fn begin_and_end_events() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let mut ground_shape_def = ShapeDef::default();
    let ground_shape = world.create_hull_shape(ground, &ground_shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    let sphere_body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.enable_contact_events = true;
    shape_def.base_material.restitution = 0.6;
    let sphere_shape = world.create_sphere_shape(sphere_body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5));

    let mut begin_count = 0;
    let mut end_count = 0;
    let mut ids_checked = false;

    for _ in 0..120 {
        world.step(1.0f32 / 60.0, 4);
        let events = world.contact_events();

        if !events.begin_events.is_empty() && !ids_checked {
            let begin = &events.begin_events[0];
            let a_is_sphere = begin.shape_id_a.index1 == sphere_shape.index1
                && begin.shape_id_a.generation == sphere_shape.generation;
            let b_is_sphere = begin.shape_id_b.index1 == sphere_shape.index1
                && begin.shape_id_b.generation == sphere_shape.generation;
            let a_is_ground = begin.shape_id_a.index1 == ground_shape.index1;
            let b_is_ground = begin.shape_id_b.index1 == ground_shape.index1;
            assert!((a_is_sphere && b_is_ground) || (a_is_ground && b_is_sphere));
            ids_checked = true;
        }

        begin_count += events.begin_events.len();
        end_count += events.end_events.len();
    }

    assert!(ids_checked);
    assert!(begin_count >= 1);
    assert!(end_count >= 1);
}

#[test]
fn events_disabled() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 2.0, 0.0);
    let body = world.create_body(&def);
    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    shape_def.enable_contact_events = false;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let mut total = 0;
    for _ in 0..120 {
        world.step(1.0f32 / 60.0, 4);
        let events = world.contact_events();
        total += events.begin_events.len() + events.end_events.len();
    }
    assert_eq!(total, 0);
}

#[test]
fn hit_event() {
    let mut world = World::default();
    world.hit_event_threshold = 1.0;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let mut ground_shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &ground_shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.gravity_scale = 0.0;
    def.position = Pos::new(0.0, 2.0, 0.0);
    def.linear_velocity = Vec3::new(0.0, -30.0, 0.0);
    let sphere_body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    shape_def.enable_hit_events = true;
    shape_def.base_material.user_material_id = 7;
    let _ = world.create_sphere_shape(sphere_body, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5));

    let mut hit_count = 0;
    let mut captured_speed: f32 = 0.0;
    let mut captured_normal = Vec3::ZERO;
    let mut captured_material_a: u64 = 0;
    let mut captured_material_b: u64 = 0;

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
        let events = world.contact_events();
        if !events.hit_events.is_empty() && hit_count == 0 {
            let hit = &events.hit_events[0];
            captured_speed = hit.approach_speed;
            captured_normal = hit.normal;
            captured_material_a = hit.user_material_id_a;
            captured_material_b = hit.user_material_id_b;
        }
        hit_count += events.hit_events.len();
    }

    assert!(hit_count >= 1);
    assert!(captured_speed > 1.0);
    assert!(captured_normal.x.abs() < 0.01);
    assert!(captured_normal.z.abs() < 0.01);
    assert!(captured_material_a == 7 || captured_material_b == 7);
}

#[test]
fn no_hit_below_threshold() {
    let mut world = World::default();
    world.enable_sleep = false;
    world.hit_event_threshold = 100.0;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 2.0, 0.0);
    let body = world.create_body(&def);
    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    shape_def.enable_hit_events = true;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let mut total = 0;
    for _ in 0..120 {
        world.step(1.0f32 / 60.0, 4);
        total += world.contact_events().hit_events.len();
    }
    assert_eq!(total, 0);
}
