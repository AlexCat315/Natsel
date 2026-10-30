//! Ported from Tests/NatselTests/Simulation/WorldTuningTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::functions::length;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::tests::world_position_assertions::abs_float;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

// "gravity drives a falling body"
#[test]
fn gravity() {
    let mut world = World::default();
    world.enable_sleep = false;
    world.gravity = Vec3::new(0.0, -20.0, 0.0);

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 10.0, 0.0);
    let body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    world.step(1.0 / 60.0, 1);

    // After one step the velocity is g * dt.
    let v = world.body_linear_velocity(body);
    assert!(abs_float((v.y + 20.0 / 60.0) as f64) < 1e-4);
}

// "zero gravity leaves a free body still"
#[test]
fn zero_gravity() {
    let mut world = World::default();
    world.enable_sleep = false;
    world.gravity = Vec3::ZERO;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 10.0, 0.0);
    def.linear_velocity = Vec3::new(3.0, 0.0, 0.0);
    let body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    for _ in 0..30 {
        world.step(1.0 / 60.0, 1);
    }

    // Constant velocity, no acceleration.
    let v = world.body_linear_velocity(body);
    assert!(abs_float((v.x - 3.0) as f64) < 1e-4);
    assert!(abs_float(v.y as f64) < 1e-6);
}

// "contact tuning round trips"
#[test]
fn contact_tuning() {
    let mut world = World::default();
    world.set_contact_tuning(45.0, 6.0, 2.5);

    assert_eq!(world.contact_hertz, 45.0);
    assert_eq!(world.contact_damping_ratio, 6.0);
    assert_eq!(world.contact_speed, 2.5);

    // A negative contact speed clamps to zero.
    world.set_contact_tuning(30.0, 10.0, -5.0);
    assert_eq!(world.contact_speed, 0.0);
}

// "world toggles are readable back"
#[test]
fn toggles() {
    let mut world = World::default();

    assert!(world.is_sleeping_enabled());
    assert!(world.is_warm_starting_enabled());
    assert!(world.is_continuous_enabled());

    world.enable_sleeping(false);
    assert_eq!(world.is_sleeping_enabled(), false);

    world.enable_warm_starting(false);
    assert_eq!(world.is_warm_starting_enabled(), false);

    world.enable_continuous(false);
    assert_eq!(world.is_continuous_enabled(), false);

    // Continuous collision off lets a fast body tunnel.
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(5.0, 0.0, 0.0);
    let wall = world.create_body(&def);
    let _ = world.create_hull_shape(wall, &ShapeDef::default(), &make_box_hull(0.05, 5.0, 5.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.is_bullet = true;
    def.gravity_scale = 0.0;
    def.linear_velocity = Vec3::new(200.0, 0.0, 0.0);
    let bullet = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = world.create_sphere_shape(bullet, &shape_def, Sphere::new(Vec3::ZERO, 0.1));

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }
    assert!(world.body_transform(bullet).p.x > 5.0);
}

// "counters and profile reflect the step"
#[test]
fn counters_and_profile() {
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
    let box_body = world.create_body(&def);
    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    for _ in 0..30 {
        world.step(1.0 / 60.0, 4);
    }

    let counters = world.counters();
    assert_eq!(counters.body_count, 2);
    assert!(counters.contact_count > 0);
    // A single resting box on the ground is one leaf in each tree.
    assert_eq!(counters.shape_count, 2);

    assert_eq!(world.awake_body_count_public(), 1);

    let profile = world.profile_data();
    assert!(profile.step >= 0.0);
}

// "the maximum linear speed clamps velocity"
#[test]
fn max_linear_speed() {
    let mut world = World::default();
    world.enable_sleep = false;
    world.gravity = Vec3::ZERO;
    world.max_linear_speed = 50.0;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.linear_velocity = Vec3::new(500.0, 0.0, 0.0);
    let body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    // An impulse beyond the cap is clamped.
    world.apply_body_linear_impulse_to_center(body, Vec3::new(1000.0, 0.0, 0.0), true);
    let v = world.body_linear_velocity(body);
    assert!(length(v) <= 50.0 + 1.0e-3);
}
