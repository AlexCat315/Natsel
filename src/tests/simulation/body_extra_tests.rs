//! Ported from Tests/NatselTests/Simulation/BodyExtraTests.swift.

use std::ffi::c_void;

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Quat, Vec3, WorldTransform};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

#[test]
fn contact_data() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -0.5, 0.0);
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.5, 0.0);
    let box_body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    // Before settling there is no touching contact.
    assert!(world.body_contact_data(box_body, usize::MAX).is_empty());

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }

    let contacts = world.body_contact_data(box_body, usize::MAX);
    assert_eq!(contacts.len(), 1);
    let first = contacts.first().unwrap();
    assert_ne!(first.contact_id.index1, 0);
    assert!(first.manifolds.len() > 0);

    // The capacity caps the result.
    assert!(world.body_contact_data(box_body, 0).is_empty());
}

#[test]
fn compute_aabb() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(-2.0, 0.0, 0.0), 0.5));
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(2.0, 0.0, 0.0), 0.5));

    world.step(1.0f32 / 60.0, 1);

    let aabb = world.body_compute_aabb(body);
    assert!(aabb.lower_bound.x <= -2.5 + 1e-3);
    assert!(aabb.upper_bound.x >= 2.5 - 1e-3);

    // A body with no shapes collapses to its origin.
    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(7.0, 8.0, 9.0);
    let empty = world.create_body(&def);
    let empty_aabb = world.body_compute_aabb(empty);
    assert!((empty_aabb.lower_bound.x - 7.0).abs() < 1e-5);
    assert!((empty_aabb.upper_bound.z - 9.0).abs() < 1e-5);
}

#[test]
fn apply_mass() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);

    // Add a shape with density but do not let creation compute the mass.
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    shape_def.update_body_mass = false;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    world.apply_body_mass_from_shapes(body);
    // A 1m cube at 1000 kg/m³ weighs 1000 kg.
    assert!((world.body_mass(body) - 1000.0).abs() < 1.0);
}

#[test]
fn closest_point() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(-3.0, 0.0, 0.0), 1.0));
    let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::new(3.0, 0.0, 0.0), 1.0));

    // The query runs core-on-core (useRadii is false), so a sphere contributes its center.
    // A target on the far right lands on the right sphere's center.
    let (right_point, right_distance) = world.body_closest_point(body, Vec3::new(10.0, 0.0, 0.0));
    assert!((right_point.x - 3.0).abs() < 1e-3);
    assert!((right_distance - 7.0).abs() < 1e-3);

    // A target on the far left lands on the left sphere's center.
    let (left_point, left_distance) = world.body_closest_point(body, Vec3::new(-10.0, 0.0, 0.0));
    assert!((left_point.x + 3.0).abs() < 1e-3);
    assert!((left_distance - 7.0).abs() < 1e-3);
}

#[test]
fn flags_and_user_data() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);

    assert!(!world.is_body_fast_rotation_allowed(body));
    world.allow_body_fast_rotation(body, true);
    assert!(world.is_body_fast_rotation_allowed(body));
    world.allow_body_fast_rotation(body, false);
    assert!(!world.is_body_fast_rotation_allowed(body));

    let mut token: i32 = 7;
    let pointer = (&mut token as *mut i32) as *mut c_void;
    world.set_body_user_data(body, Some(pointer));
    assert_eq!(world.body_user_data(body), Some(pointer));
    world.set_body_user_data(body, None);
    assert_eq!(world.body_user_data(body), None);
}

#[test]
fn target_transform() {
    let mut world = World::default();
    world.enable_sleep = false;
    world.gravity = Vec3::ZERO;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    // Ask the body to reach (6, 0, 0) in one second.
    world.set_body_target_transform(
        body,
        WorldTransform::new(Pos::new(6.0, 0.0, 0.0), Quat::IDENTITY),
        1.0,
        true,
    );

    // The velocity is the center difference over the time step.
    let v = world.body_linear_velocity(body);
    assert!((v.x - 6.0).abs() < 1e-3);
    assert!(v.y.abs() < 1e-5);

    // Stepping moves it toward the goal.
    world.step(1.0f32 / 60.0, 1);
    assert!(world.body_transform(body).p.x > 0.0);

    // A static body is ignored.
    let mut static_def = BodyDef::default();
    static_def.type_ = BodyType::StaticBody;
    static_def.position = Pos::new(20.0, 0.0, 0.0);
    let static_body = world.create_body(&static_def);
    world.set_body_target_transform(
        static_body,
        WorldTransform::new(Pos::new(0.0, 0.0, 0.0), Quat::IDENTITY),
        1.0,
        true,
    );
    assert_eq!(world.body_linear_velocity(static_body), Vec3::ZERO);
}
