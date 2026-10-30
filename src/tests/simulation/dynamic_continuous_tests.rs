//! Ported from Tests/NatselTests/Simulation/DynamicContinuousTests.swift.

use std::collections::HashSet;
use std::sync::Arc;

use crate::base::core::is_double_precision;
use crate::collision::hull::make_box_hull;
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Pos, Vec3};
use crate::simulation::world_runtime::BodyFlags;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;

/// Swift defaulted `type`, `customFilter`, `density` and `restitution`; Rust has no default
/// arguments, so every ported call site passes all of them explicitly.
fn ball(
    world: &mut World,
    x: f64,
    velocity: f32,
    bullet: bool,
    body_type: BodyType,
    custom_filter: bool,
    density: f32,
    restitution: f32,
) -> BodyID {
    let mut def = BodyDef::default();
    def.type_ = body_type;
    def.position = Pos::new(x, 0.0, 0.0);
    def.linear_velocity = Vec3::new(velocity, 0.0, 0.0);
    def.is_bullet = bullet;
    let id = world.create_body(&def);
    let mut shape = ShapeDef::default();
    shape.enable_custom_filtering = custom_filter;
    shape.density = density;
    shape.base_material.restitution = restitution;
    let _ = world.create_sphere_shape(id, &shape, Sphere::new(Vec3::ZERO, 0.1));
    id
}

/// The Swift test used the defaulted arguments for the common case.
fn plain_ball(world: &mut World, x: f64, velocity: f32, bullet: bool) -> BodyID {
    ball(
        world,
        x,
        velocity,
        bullet,
        BodyType::DynamicBody,
        false,
        1000.0,
        0.0,
    )
}

// "precision capability agrees with fractional world integration"
#[test]
fn precision_capability() {
    assert!(is_double_precision());
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    let id = plain_ball(&mut world, 1.0e12 + 0.125, 0.25, false);
    world.step(0.5, 1);
    assert!((world.body_position(id).x - (1.0e12 + 0.25)).abs() < 0.001);
}

// "crossing swept paths stop both movers"
#[test]
fn crossing() {
    for bullet in [false, true] {
        for reverse in [false, true] {
            let mut world = World::default();
            world.gravity = Vec3::ZERO;
            world.enable_sleep = false;
            world.enable_dynamic_continuous = true;
            let first = plain_ball(
                &mut world,
                if reverse { 2.0 } else { -2.0 },
                if reverse { -240.0 } else { 240.0 },
                bullet,
            );
            let second = plain_ball(
                &mut world,
                if reverse { -2.0 } else { 2.0 },
                if reverse { 240.0 } else { -240.0 },
                bullet,
            );
            let left = if reverse { second } else { first };
            let right = if reverse { first } else { second };
            for _ in 0..8 {
                world.step(1.0 / 60.0, 1);
                assert!(world.body_position(left).x < world.body_position(right).x);
            }
        }
    }
}

// "moving kinematic body is included in the sweep"
#[test]
fn kinematic() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_dynamic_continuous = true;
    let dynamic = plain_ball(&mut world, -2.0, 240.0, false);
    let platform = ball(
        &mut world,
        2.0,
        -240.0,
        false,
        BodyType::KinematicBody,
        false,
        1000.0,
        0.0,
    );
    world.step(1.0 / 60.0, 1);
    assert!(world.body_position(dynamic).x < world.body_position(platform).x);
    assert!((world.body_position(platform).x + 2.0).abs() < 0.001);
}

// "dynamic CCD respects filters and survives rollback"
#[test]
fn filtering_and_rollback() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_dynamic_continuous = true;
    let left = ball(&mut world, -2.0, 240.0, true, BodyType::DynamicBody, true, 1000.0, 0.0);
    let right = ball(&mut world, 2.0, -240.0, true, BodyType::DynamicBody, true, 1000.0, 0.0);
    world.set_custom_filter(Some(Arc::new(|_, _| false)));
    let state = world.save_state();
    world.step(1.0 / 60.0, 1);
    assert!(world.body_position(left).x > world.body_position(right).x);
    world.enable_dynamic_continuous = false;
    assert!(world.restore_state(&state));
    assert!(world.enable_dynamic_continuous);
}

// "CCD reports more than eight crossed sensors"
#[test]
fn many_sensors() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    for i in 0..12 {
        let mut def = BodyDef::default();
        def.position = Pos::new(-1.65 + i as f64 * 0.3, 0.0, 0.0);
        let sensor = world.create_body(&def);
        let mut shape = ShapeDef::default();
        shape.is_sensor = true;
        shape.enable_sensor_events = true;
        let _ = world.create_hull_shape(sensor, &shape, &make_box_hull(0.01, 1.0, 1.0));
    }
    let mover = plain_ball(&mut world, -2.0, 240.0, true);
    let shape = world.make_shape_id(world.body(mover).head_shape_id);
    world.enable_shape_sensor_events(shape, true);
    world.step(1.0 / 60.0, 1);
    let ids: HashSet<_> = world
        .sensor_event_buffers()
        .begin_events
        .iter()
        .map(|event| event.sensor_shape_id)
        .collect();
    assert_eq!(ids.len(), 12);
}

// "rotating thin hull queries the interior of its arc"
#[test]
fn rotation_arc() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_dynamic_continuous = true;
    let obstacle = world.create_body(&BodyDef::default());
    let _ = world.create_sphere_shape(
        obstacle,
        &ShapeDef::default(),
        Sphere::new(Vec3::new(0.0, 0.9, 0.0), 0.03),
    );
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Z, std::f32::consts::PI / 4.0);
    def.angular_velocity = Vec3::new(0.0, 0.0, 120.0);
    def.allow_fast_rotation = true;
    let rotor = world.create_body(&def);
    let _ = world.create_hull_shape(rotor, &ShapeDef::default(), &make_box_hull(1.0, 0.01, 0.01));
    world.step(1.0 / 60.0, 1);
    assert!((world.body_sim(&world.body(rotor)).flags & BodyFlags::HAD_TIME_OF_IMPACT.0) != 0);
}

// "impact conserves momentum and wakes a sleeping target"
#[test]
fn momentum() {
    for sleeping in [false, true] {
        let mut world = World::default();
        world.gravity = Vec3::ZERO;
        world.enable_dynamic_continuous = true;
        let left = ball(&mut world, -2.0, 240.0, true, BodyType::DynamicBody, false, 1000.0, 1.0);
        let right = ball(&mut world, 0.0, 0.0, true, BodyType::DynamicBody, false, 2000.0, 1.0);
        if sleeping {
            world.set_body_awake(right, false);
        }
        world.enable_shape_hit_events(world.make_shape_id(world.body(left).head_shape_id), true);
        let ma = world.body_mass(left);
        let mb = world.body_mass(right);
        let momentum = ma * 240.0;
        let energy = 0.5 * ma * 240.0 * 240.0;
        world.step(1.0 / 60.0, 1);
        assert!(world.is_body_awake_id(right));
        let va = world.body_linear_velocity(left).x;
        let vb = world.body_linear_velocity(right).x;
        assert!((ma * va + mb * vb - momentum).abs() < momentum * 0.0001);
        assert!((0.5 * (ma * va * va + mb * vb * vb) - energy).abs() < energy * 0.0001);
        assert!(va < 0.0 && vb > 0.0);
        assert_eq!(world.contact_events().hit_events.len(), 1);
    }
}

// "moving-pair results agree across workers at a distant origin"
#[test]
fn worker_parity() {
    fn run(workers: i32) -> Vec<Pos> {
        let mut world = World::default();
        world.gravity = Vec3::ZERO;
        world.worker_count = workers;
        world.enable_dynamic_continuous = true;
        let mut bodies: Vec<BodyID> = Vec::new();
        for i in 0..32 {
            let offset = 1.0e12 + i as f64 * 10.0;
            bodies.push(plain_ball(&mut world, offset - 2.0, 240.0, true));
            bodies.push(plain_ball(&mut world, offset + 2.0, -240.0, true));
        }
        for _ in 0..10 {
            world.step(1.0 / 60.0, 1);
        }
        bodies
            .iter()
            .map(|id| world.body_position(*id))
            .collect()
    }
    assert_eq!(run(1), run(4));
}

// "static CCD publishes transient impacts with contact flags and supports rollback"
#[test]
fn static_impact_events() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.hit_event_threshold = 1_000.0;
    let mover = plain_ball(&mut world, -3.0, 360.0, true);
    let target = ball(
        &mut world,
        0.0,
        0.0,
        false,
        BodyType::StaticBody,
        false,
        1000.0,
        0.0,
    );
    let mover_shape = world.make_shape_id(world.body(mover).head_shape_id);
    let target_shape = world.make_shape_id(world.body(target).head_shape_id);
    world.enable_shape_contact_events(mover_shape, true);
    let initial = world.save_state();
    world.step(1.0 / 60.0, 2);
    let impacts = world.continuous_impacts().to_vec();
    assert_eq!(impacts.len(), 1);
    assert_eq!(impacts[0].shape_id_a, target_shape);
    assert_eq!(impacts[0].shape_id_b, mover_shape);
    assert!(impacts[0].normal.x < -0.9);
    assert!(impacts[0].approach_speed > 350.0);
    assert!(world.body_position(mover).x < 0.0);
    assert!(world.contact_events().hit_events.is_empty());
    let with_impact = world.save_state();
    assert!(world.restore_state(&initial));
    assert!(world.continuous_impacts().is_empty());
    assert!(world.restore_state(&with_impact));
    assert_eq!(world.continuous_impacts(), impacts.as_slice());
    world.step(0.0, 4);
    assert!(world.continuous_impacts().is_empty());
    assert_eq!(impacts.len(), 1);
}
