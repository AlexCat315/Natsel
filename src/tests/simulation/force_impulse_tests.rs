//! Ported from Tests/NatselTests/Simulation/ForceImpulseTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::BodyID;
use crate::types::joint_defs::RevoluteJointDef;
use crate::types::shape_def::ShapeDef;

fn vec_len(v: Vec3) -> f32 {
    (v.x * v.x + v.y * v.y + v.z * v.z).sqrt()
}

fn make_box(world: &mut World, position: Pos) -> BodyID {
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = position;
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    body
}

#[test]
fn force_to_center() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_sleep = false;
    let body = make_box(&mut world, Pos::new(0.0, 0.0, 0.0));

    let mass = world.body_mass(body);
    world.apply_body_force_to_center(body, Vec3::new(0.0, mass * 10.0, 0.0), true);
    world.step(1.0f32 / 60.0, 1);

    let v = world.body_linear_velocity(body);
    assert!((v.y - 10.0f32 / 60.0).abs() < 1e-4);
    assert!(v.x.abs() < 1e-6);
}

#[test]
fn force_off_center() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_sleep = false;
    let body = make_box(&mut world, Pos::new(0.0, 0.0, 0.0));

    let mass = world.body_mass(body);
    world.apply_body_force(
        body,
        Vec3::new(mass * 10.0, 0.0, 0.0),
        Pos::new(0.0, 0.5, 0.0),
        true,
    );
    world.step(1.0f32 / 60.0, 1);

    let v = world.body_linear_velocity(body);
    let w = world.body_angular_velocity(body);
    assert!((v.x - 10.0f32 / 60.0).abs() < 1e-4);
    assert!(w.z < 0.0);
}

#[test]
fn torque() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_sleep = false;
    let body = make_box(&mut world, Pos::new(0.0, 0.0, 0.0));

    world.apply_body_torque(body, Vec3::new(0.0, 0.0, 50.0), true);
    world.step(1.0f32 / 60.0, 1);

    assert!(vec_len(world.body_linear_velocity(body)) < 1e-6);
    assert!(world.body_angular_velocity(body).z > 0.0);
}

#[test]
fn forces_clear() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_sleep = false;
    let body = make_box(&mut world, Pos::new(0.0, 0.0, 0.0));

    let mass = world.body_mass(body);
    world.apply_body_force_to_center(body, Vec3::new(0.0, mass * 6.0, 0.0), true);
    world.step(1.0f32 / 60.0, 1);
    let v1 = world.body_linear_velocity(body);

    world.step(1.0f32 / 60.0, 1);
    let v2 = world.body_linear_velocity(body);
    assert!((v2.y - v1.y).abs() < 1e-6);
}

#[test]
fn impulse_to_center() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_sleep = false;
    let body = make_box(&mut world, Pos::new(0.0, 0.0, 0.0));

    let mass = world.body_mass(body);
    world.apply_body_linear_impulse_to_center(body, Vec3::new(0.0, mass * 3.0, 0.0), true);

    let v = world.body_linear_velocity(body);
    assert!((v.y - 3.0).abs() < 1e-5);
    assert!(vec_len(world.body_angular_velocity(body)) < 1e-6);
}

#[test]
fn impulse_off_center() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_sleep = false;
    let body = make_box(&mut world, Pos::new(0.0, 0.0, 0.0));

    let mass = world.body_mass(body);
    world.apply_body_linear_impulse(
        body,
        Vec3::new(mass * 3.0, 0.0, 0.0),
        Pos::new(0.0, 0.5, 0.0),
        true,
    );

    let v = world.body_linear_velocity(body);
    assert!((v.x - 3.0).abs() < 1e-5);
    assert!(world.body_angular_velocity(body).z < 0.0);
}

#[test]
fn angular_impulse() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;
    world.enable_sleep = false;
    let body = make_box(&mut world, Pos::new(0.0, 0.0, 0.0));

    world.apply_body_angular_impulse(body, Vec3::new(0.0, 0.0, 5.0), true);

    assert!(vec_len(world.body_linear_velocity(body)) < 1e-6);
    assert!(world.body_angular_velocity(body).z > 0.0);
}

#[test]
fn wakes_body() {
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, -10.0, 0.0);

    let mut ground_def = BodyDef::default();
    ground_def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&ground_def);
    let mut ground_shape = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &ground_shape, &make_box_hull(10.0, 0.5, 10.0));

    let body = make_box(&mut world, Pos::new(0.0, 2.0, 0.0));

    for _ in 0..400 {
        world.step(1.0f32 / 60.0, 4);
    }
    assert!(!world.is_body_awake_id(body));

    world.apply_body_force_to_center(body, Vec3::new(0.0, 1000.0, 0.0), true);
    assert!(world.is_body_awake_id(body));
}

#[test]
fn body_associations() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.3, 0.3, 0.3));
    assert_eq!(world.body_shape_count(body), 2);
    assert!(world.body_joints(body).is_empty());

    def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);

    let mut joint_def = RevoluteJointDef::default();
    joint_def.base.body_id_a = ground;
    joint_def.base.body_id_b = body;
    let joint = world.create_revolute_joint(&joint_def);

    let joints = world.body_joints(body);
    assert_eq!(joints.len(), 1);
    assert_eq!(joints[0].index1, joint.index1);
}
