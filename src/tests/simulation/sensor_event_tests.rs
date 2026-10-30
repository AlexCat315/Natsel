//! Ported from Tests/NatselTests/Simulation/SensorEventTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

#[test]
fn bullet_through_sensor() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(1.5, 11.0, 0.0);
    let wall = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.enable_sensor_events = true;
    let _ = world.create_hull_shape(wall, &shape_def, &make_box_hull(0.5, 10.0, 1.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.is_bullet = true;
    def.gravity_scale = 0.0;
    def.position = Pos::new(7.39814, 4.0, 0.0);
    def.linear_velocity = Vec3::new(-20.0, 0.0, 0.0);
    let bullet = world.create_body(&def);

    shape_def = ShapeDef::default();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    let _ = world.create_sphere_shape(bullet, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.1));

    let mut begin_count = 0;
    let mut end_count = 0;

    loop {
        world.step(1.0f32 / 60.0, 4);
        let bullet_pos = world.body_transform(bullet).p;
        let events = world.sensor_events();
        if !events.begin.is_empty() {
            begin_count += 1;
        }
        if !events.end.is_empty() {
            end_count += 1;
        }
        if bullet_pos.x < -1.0 {
            break;
        }
    }

    assert_eq!(begin_count, 1);
    assert_eq!(end_count, 1);
}

#[test]
fn resting_inside_sensor() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let sensor_body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.is_sensor = true;
    shape_def.enable_sensor_events = true;
    let _ = world.create_hull_shape(sensor_body, &shape_def, &make_box_hull(2.0, 2.0, 2.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    let ball = world.create_body(&def);
    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    shape_def.enable_sensor_events = true;
    let _ = world.create_sphere_shape(ball, &shape_def, Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5));

    let mut begin_count = 0;
    let mut end_count = 0;
    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
        let events = world.sensor_events();
        begin_count += events.begin.len();
        end_count += events.end.len();
    }

    // Overlapping from the start: one begin, no end while it stays inside.
    assert_eq!(begin_count, 1);
    assert_eq!(end_count, 0);
}
