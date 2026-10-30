//! Ported from Tests/NatselTests/Simulation/ApiCoverageTests.swift.

use crate::collision::compound::compound::Compound;
use crate::collision::hull::make_box_hull;
use crate::math::functions::make_quat_from_axis_angle;
use crate::math::types::{Pos, Quat, Transform, Vec3, WorldTransform};
use crate::simulation::World;
use crate::simulation::shape_runtime::should_shapes_collide;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{CompoundDef, CompoundHullDef};
use crate::types::ids::BodyID;
use crate::types::joint_defs::{
    DistanceJointDef, ParallelJointDef, PrismaticJointDef, RevoluteJointDef, SphericalJointDef,
    WheelJointDef,
};
use crate::types::shape_def::{Filter, ShapeDef, SurfaceMaterial};

fn make_body(world: &mut World, type_: BodyType) -> BodyID {
    let mut def = BodyDef::default();
    def.type_ = type_;
    world.create_body(&def)
}

#[test]
fn filters() {
    let mut a = Filter::default();
    a.category_bits = 0b0001;
    a.mask_bits = 0b0010;

    let mut b = Filter::default();
    b.category_bits = 0b0010;
    b.mask_bits = 0b0001;

    assert!(should_shapes_collide(&a, &b));

    let mut c = Filter::default();
    c.category_bits = 0b0010;
    c.mask_bits = 0;
    assert!(!should_shapes_collide(&a, &c));
}

#[test]
fn shape_filter() {
    let mut world = World::default();
    let body = make_body(&mut world, BodyType::DynamicBody);

    let mut shape_def = ShapeDef::default();
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let mut filter = Filter::default();
    filter.category_bits = 0b1000;
    filter.mask_bits = 0b0111;
    world.set_shape_filter(shape, filter);

    assert_eq!(world.shape_filter(shape).category_bits, 0b1000);
    assert_eq!(world.shape_filter(shape).mask_bits, 0b0111);
}

#[test]
fn geometry_accessors() {
    let mut world = World::default();
    world.enable_sleep = false;
    let body = make_body(&mut world, BodyType::DynamicBody);

    let shape_def = ShapeDef::default();
    let hull_shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    assert!(world.shape_hull(hull_shape).is_some());
    assert!(world.shape_mesh(hull_shape).is_none());
    assert!(world.shape_compound(hull_shape).is_none());
    assert!(world.shape_height_field(hull_shape).is_none());

    let mut definition = CompoundDef::default();
    let mut item = CompoundHullDef::default();
    item.hull = Some(make_box_hull(0.5, 0.5, 0.5));
    item.transform = Transform::IDENTITY;
    item.material = SurfaceMaterial::default_material();
    definition.hulls.push(item);
    let compound = Compound::new(&definition).expect("compound creation failed");

    // Compound shapes must live on a static body.
    let static_body = make_body(&mut world, BodyType::StaticBody);
    let compound_shape = world.create_compound_shape(static_body, &shape_def, &compound);
    assert_ne!(compound_shape.index1, 0);
    assert!(world.shape_compound(compound_shape).is_some());
    assert!(world.shape_hull(compound_shape).is_none());
}

#[test]
fn body_state() {
    let mut world = World::default();
    world.enable_sleep = false;
    let body = make_body(&mut world, BodyType::DynamicBody);

    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(1.0, 2.0, 3.0));

    // The smallest half dimension is the x extent.
    assert!((world.body_min_extent_value(body) - 1.0).abs() < 1e-4);
    assert!(world.is_body_enabled(body));

    world.disable_body(body);
    assert!(!world.is_body_enabled(body));
    world.enable_body(body);
    assert!(world.is_body_enabled(body));
}

#[test]
fn touching_contacts() {
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

    assert_eq!(world.touching_contact_count(), 0);
    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }
    assert!(world.touching_contact_count() > 0);
}

#[test]
fn joint_world_anchor() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(2.0, 3.0, 4.0);
    let anchor = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(2.0, 2.0, 4.0);
    let bob = world.create_body(&def);

    let mut joint_def = DistanceJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.length = 1.0;
    let joint = world.create_distance_joint(&joint_def);

    let world_anchor = world.joint_world_anchor_a(joint).unwrap();
    assert!((world_anchor.x - 2.0).abs() < 1e-4);
    assert!((world_anchor.y - 3.0).abs() < 1e-4);
    assert!((world_anchor.z - 4.0).abs() < 1e-4);
}

#[test]
fn wheel_limits() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let chassis = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, -1.0, 0.0);
    let wheel = world.create_body(&def);

    let mut joint_def = WheelJointDef::default();
    joint_def.base.body_id_a = chassis;
    joint_def.base.body_id_b = wheel;
    let joint = world.create_wheel_joint(&joint_def);

    world.set_wheel_joint_suspension_limits(joint, -0.2, 0.3);
    assert!((world.wheel_joint_lower_suspension_limit(joint).unwrap() + 0.2).abs() < 1e-6);
    assert!((world.wheel_joint_upper_suspension_limit(joint).unwrap() - 0.3).abs() < 1e-6);

    world.set_wheel_joint_steering_limits(joint, -0.4, 0.5);
    assert!((world.wheel_joint_lower_steering_limit(joint).unwrap() + 0.4).abs() < 1e-6);
    assert!((world.wheel_joint_upper_steering_limit(joint).unwrap() - 0.5).abs() < 1e-6);

    world.enable_wheel_joint_suspension(joint, true);
    assert!(world.is_wheel_joint_suspension_enabled(joint));
    world.enable_wheel_joint_suspension_limit(joint, true);
    assert!(world.is_wheel_joint_suspension_limit_enabled(joint));
    world.enable_wheel_joint_steering_limit(joint, true);
    assert!(world.is_wheel_joint_steering_limit_enabled(joint));

    assert!(world.wheel_joint_spin_torque_value(joint).is_some());
    assert!(world.wheel_joint_steering_torque(joint).is_some());
    assert!(world.wheel_joint_steering_angle(joint).is_some());
    assert!(world.wheel_joint_spin_speed_value(joint).is_some());
    assert!(world.wheel_joint_motor_torque(joint).is_some());
}

#[test]
fn spherical_extras() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let base = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, -1.0, 0.0);
    let bob = world.create_body(&def);

    let mut joint_def = SphericalJointDef::default();
    joint_def.base.body_id_a = base;
    joint_def.base.body_id_b = bob;
    let joint = world.create_spherical_joint(&joint_def);

    world.set_spherical_joint_spring(joint, 4.0, 0.4);
    let spring = world.spherical_joint_spring(joint).unwrap();
    assert!((spring.0 - 4.0).abs() < 1e-6);
    assert!((spring.1 - 0.4).abs() < 1e-6);

    let rotation = make_quat_from_axis_angle(Vec3::AXIS_Y, 0.5);
    world.set_spherical_joint_target_rotation(joint, rotation);
    assert!(world.spherical_joint_target_rotation(joint).is_some());

    world.enable_spherical_joint_twist_limit(joint, true);
    assert!(world.is_spherical_joint_twist_limit_enabled(joint));

    assert!(world.spherical_joint_twist_angle(joint).is_some());
    assert!(world.spherical_joint_motor_torque(joint).is_some());
}

#[test]
fn joint_reactions() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let base = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 0.0, 0.0);
    let other = world.create_body(&def);

    let mut prism_def = PrismaticJointDef::default();
    prism_def.base.body_id_a = base;
    prism_def.base.body_id_b = other;
    let prismatic = world.create_prismatic_joint(&prism_def);
    world.enable_prismatic_joint_motor(prismatic, true);
    assert!(world.prismatic_joint_motor_force(prismatic).is_some());

    let mut rev_def = RevoluteJointDef::default();
    rev_def.base.body_id_a = base;
    rev_def.base.body_id_b = other;
    let revolute = world.create_revolute_joint(&rev_def);
    world.enable_revolute_joint_motor(revolute, true);
    assert!(world.revolute_joint_motor_torque(revolute).is_some());
}

#[test]
fn parallel_aliases() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let base = world.create_body(&def);
    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let other = world.create_body(&def);

    let mut joint_def = ParallelJointDef::default();
    joint_def.base.body_id_a = base;
    joint_def.base.body_id_b = other;
    let joint = world.create_parallel_joint(&joint_def);

    world.set_parallel_joint_spring_hertz(joint, 7.0);
    assert!((world.parallel_joint_spring_hertz(joint).unwrap() - 7.0).abs() < 1e-6);
    assert_eq!(
        world.parallel_joint_hertz(joint),
        world.parallel_joint_spring_hertz(joint)
    );

    world.set_parallel_joint_spring_damping_ratio(joint, 0.6);
    assert!((world.parallel_joint_spring_damping_ratio(joint).unwrap() - 0.6).abs() < 1e-6);
}

#[test]
fn shape_name() {
    let mut world = World::default();
    let body = make_body(&mut world, BodyType::DynamicBody);
    let mut shape_def = ShapeDef::default();
    shape_def.name = Some("hull".to_string());
    let shape = world.create_hull_shape(body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    assert_eq!(world.shape_name(shape), Some("hull"));
    world.set_shape_name(shape, Some("renamed"));
    assert_eq!(world.shape_name(shape), Some("renamed"));
}
