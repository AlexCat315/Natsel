//! Ported from Tests/NatselTests/Simulation/JointInspectionTests.swift.

use crate::collision::hull::{make_box_hull, make_cube_hull};
use crate::math::functions::length;
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::World;
use crate::tests::world_position_assertions::abs_float;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::{BodyID, JointID};
use crate::types::joint_defs::{
    DistanceJointDef, JointType, MotorJointDef, ParallelJointDef, PrismaticJointDef,
    RevoluteJointDef, SphericalJointDef, WeldJointDef, WheelJointDef,
};
use crate::types::shape_def::ShapeDef;

/// Swift declared a `jointType:` parameter it never consulted; the pendulum is always a distance
/// joint.
fn make_pendulum(_joint_type: JointType) -> (World, JointID, BodyID, BodyID) {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    let anchor = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 5.0, 0.0);
    let bob = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(bob, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    let mut joint_def = DistanceJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.base.local_frame_a = Transform::new(Vec3::ZERO, Quat::IDENTITY);
    joint_def.base.local_frame_b = Transform::new(Vec3::new(-1.0, 0.0, 0.0), Quat::IDENTITY);
    joint_def.length = 1.0;
    let joint = world.create_distance_joint(&joint_def);

    (world, joint, anchor, bob)
}

// "collide connected round trips"
#[test]
fn collide_connected() {
    let (mut world, joint, _, _) = make_pendulum(JointType::DistanceJoint);

    assert!(!world.joint_collide_connected(joint));
    world.set_joint_collide_connected(joint, true);
    assert!(world.joint_collide_connected(joint));
    world.set_joint_collide_connected(joint, false);
    assert!(!world.joint_collide_connected(joint));
}

// "constraint tuning round trips"
#[test]
fn constraint_tuning() {
    let (mut world, joint, _, _) = make_pendulum(JointType::DistanceJoint);

    assert!(world.set_joint_constraint_tuning(joint, 12.0, 0.3));
    let tuning = world.joint_constraint_tuning(joint).unwrap();
    assert!(abs_float((tuning.0 - 12.0) as f64) < 1e-6);
    assert!(abs_float((tuning.1 - 0.3) as f64) < 1e-6);
}

// "force and torque thresholds round trip"
#[test]
fn thresholds() {
    let (mut world, joint, _, _) = make_pendulum(JointType::DistanceJoint);

    assert!(world.set_joint_force_threshold(joint, 250.0));
    assert!(abs_float((world.joint_force_threshold(joint).unwrap() - 250.0) as f64) < 1e-6);

    assert!(world.set_joint_torque_threshold(joint, 75.0));
    assert!(abs_float((world.joint_torque_threshold(joint).unwrap() - 75.0) as f64) < 1e-6);
}

// "local frames round trip"
#[test]
fn local_frames() {
    let (mut world, joint, _, _) = make_pendulum(JointType::DistanceJoint);

    let frame_a = Transform::new(Vec3::new(0.1, 0.2, 0.3), Quat::IDENTITY);
    let frame_b = Transform::new(Vec3::new(-0.4, 0.5, 0.6), Quat::IDENTITY);
    assert!(world.set_joint_local_frame_a(joint, frame_a));
    assert!(world.set_joint_local_frame_b(joint, frame_b));

    let back_a = world.joint_local_frame_a(joint).unwrap();
    let back_b = world.joint_local_frame_b(joint).unwrap();
    assert!(abs_float((back_a.p.x - 0.1) as f64) < 1e-6);
    assert!(abs_float((back_b.p.x + 0.4) as f64) < 1e-6);
}

// "a satisfied distance joint reports no linear separation"
#[test]
fn separation() {
    let (world, joint, _, _) = make_pendulum(JointType::DistanceJoint);

    // The anchors coincide when the joint is built.
    assert!(abs_float(world.joint_linear_separation(joint).unwrap() as f64) < 1e-5);
    assert!(abs_float(world.joint_angular_separation(joint).unwrap() as f64) < 1e-5);
}

// "a stretched joint reports a positive separation"
#[test]
fn stretched_separation() {
    let (mut world, joint, _, bob) = make_pendulum(JointType::DistanceJoint);

    // Move the bob away from the anchor; the joint is now violated.
    world.set_body_transform(bob, Pos::new(3.0, 5.0, 0.0), Quat::IDENTITY);
    let separation = world.joint_linear_separation(joint).unwrap();
    assert!(separation > 1.5);
}

// "a distance joint carries a constraint force under load"
#[test]
fn constraint_force() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    let anchor = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 4.0, 0.0);
    let bob = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(bob, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    let mut joint_def = DistanceJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.length = 1.0;
    let joint = world.create_distance_joint(&joint_def);

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }

    // The bob hangs from the joint, so the force must support its weight.
    let force = world.joint_constraint_force(joint).unwrap();
    let weight = world.body_mass(bob) * 10.0;
    assert!(force.y > weight * 0.5);

    // A distance joint cannot transmit torque.
    let torque = world.joint_constraint_torque(joint).unwrap();
    assert!(length(torque) < 1e-3);
}

// "waking the joint wakes its bodies"
#[test]
fn wake_bodies() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    let anchor = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 4.0, 0.0);
    let bob = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(bob, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    let mut joint_def = DistanceJointDef::default();
    joint_def.base.body_id_a = anchor;
    joint_def.base.body_id_b = bob;
    joint_def.length = 1.0;
    let joint = world.create_distance_joint(&joint_def);

    for _ in 0..400 {
        world.step(1.0f32 / 60.0, 4);
    }
    world.wake_joint_bodies(joint);
    assert!(world.is_body_awake_id(bob));
    assert!(world.joint_is_awake(joint));
}

// MARK: - Per-type scalar accessors

// "suspension and steering scalars round trip independently"
#[test]
fn wheel_scalars() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let chassis = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, -1.0, 0.0);
    let wheel = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(wheel, &shape_def, &make_box_hull(0.3, 0.3, 0.1));

    let mut joint_def = WheelJointDef::default();
    joint_def.base.body_id_a = chassis;
    joint_def.base.body_id_b = wheel;
    let joint = world.create_wheel_joint(&joint_def);

    world.set_wheel_joint_suspension_hertz(joint, 7.0);
    assert!(abs_float((world.wheel_joint_suspension_hertz(joint).unwrap() - 7.0) as f64) < 1e-6);

    world.set_wheel_joint_suspension_damping_ratio(joint, 0.45);
    assert!(
        abs_float((world.wheel_joint_suspension_damping_ratio(joint).unwrap() - 0.45) as f64)
            < 1e-6
    );

    world.set_wheel_joint_steering_hertz(joint, 3.0);
    assert!(abs_float((world.wheel_joint_steering_hertz(joint).unwrap() - 3.0) as f64) < 1e-6);

    world.set_wheel_joint_steering_damping_ratio(joint, 0.25);
    assert!(
        abs_float((world.wheel_joint_steering_damping_ratio(joint).unwrap() - 0.25) as f64) < 1e-6
    );

    world.set_wheel_joint_spin_motor_speed(joint, 12.0);
    assert!(abs_float((world.wheel_joint_spin_motor_speed(joint).unwrap() - 12.0) as f64) < 1e-6);

    // The two hertz values stay independent.
    assert!(world.wheel_joint_suspension_hertz(joint) != world.wheel_joint_steering_hertz(joint));
}

// "motor and weld hertz/damping pairs round trip"
#[test]
fn spring_pairs() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let base = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let other = world.create_body(&def);

    let mut motor_def = MotorJointDef::default();
    motor_def.base.body_id_a = base;
    motor_def.base.body_id_b = other;
    let motor = world.create_motor_joint(&motor_def);

    world.set_motor_joint_linear_hertz(motor, 4.0);
    world.set_motor_joint_linear_damping_ratio(motor, 0.6);
    world.set_motor_joint_angular_hertz(motor, 2.0);
    world.set_motor_joint_angular_damping_ratio(motor, 0.8);

    assert!(abs_float((world.motor_joint_linear_hertz(motor).unwrap() - 4.0) as f64) < 1e-6);
    assert!(
        abs_float((world.motor_joint_linear_damping_ratio(motor).unwrap() - 0.6) as f64) < 1e-6
    );
    assert!(abs_float((world.motor_joint_angular_hertz(motor).unwrap() - 2.0) as f64) < 1e-6);
    assert!(
        abs_float((world.motor_joint_angular_damping_ratio(motor).unwrap() - 0.8) as f64) < 1e-6
    );

    let mut weld_def = WeldJointDef::default();
    weld_def.base.body_id_a = base;
    weld_def.base.body_id_b = other;
    let weld = world.create_weld_joint(&weld_def);

    world.set_weld_joint_linear_hertz(weld, 9.0);
    world.set_weld_joint_linear_damping_ratio(weld, 0.1);
    world.set_weld_joint_angular_hertz(weld, 5.0);
    world.set_weld_joint_angular_damping_ratio(weld, 0.9);

    assert!(abs_float((world.weld_joint_linear_hertz(weld).unwrap() - 9.0) as f64) < 1e-6);
    assert!(abs_float((world.weld_joint_linear_damping_ratio(weld).unwrap() - 0.1) as f64) < 1e-6);
    assert!(abs_float((world.weld_joint_angular_hertz(weld).unwrap() - 5.0) as f64) < 1e-6);
    assert!(
        abs_float((world.weld_joint_angular_damping_ratio(weld).unwrap() - 0.9) as f64) < 1e-6
    );
}

// "limit accessors report the configured bounds"
#[test]
fn limit_accessors() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let base = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 0.0, 0.0);
    let other = world.create_body(&def);

    let mut rev_def = RevoluteJointDef::default();
    rev_def.base.body_id_a = base;
    rev_def.base.body_id_b = other;
    let revolute = world.create_revolute_joint(&rev_def);

    world.set_revolute_joint_limits(revolute, -0.4, 0.6);
    assert!(abs_float((world.revolute_joint_lower_limit(revolute).unwrap() + 0.4) as f64) < 1e-6);
    assert!(abs_float((world.revolute_joint_upper_limit(revolute).unwrap() - 0.6) as f64) < 1e-6);

    world.enable_revolute_joint_limit(revolute, true);
    assert!(world.is_revolute_joint_limit_enabled(revolute));

    world.set_revolute_joint_target_angle(revolute, 0.25);
    assert!(abs_float((world.revolute_joint_target_angle(revolute).unwrap() - 0.25) as f64) < 1e-6);

    let mut prism_def = PrismaticJointDef::default();
    prism_def.base.body_id_a = base;
    prism_def.base.body_id_b = other;
    let prismatic = world.create_prismatic_joint(&prism_def);

    world.set_prismatic_joint_limits(prismatic, -1.5, 2.5);
    assert!(
        abs_float((world.prismatic_joint_lower_limit(prismatic).unwrap() + 1.5) as f64) < 1e-6
    );
    assert!(
        abs_float((world.prismatic_joint_upper_limit(prismatic).unwrap() - 2.5) as f64) < 1e-6
    );

    world.enable_prismatic_joint_motor(prismatic, true);
    assert!(world.is_prismatic_joint_motor_enabled(prismatic));

    world.enable_prismatic_joint_spring(prismatic, true);
    assert!(world.is_prismatic_joint_spring_enabled(prismatic));
}

// "prismatic translation and speed track the body pose"
#[test]
fn prismatic_kinematics() {
    let mut world = World::default();
    world.enable_sleep = false;
    world.gravity = Vec3::ZERO;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let base = world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    let slider = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(slider, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    let mut joint_def = PrismaticJointDef::default();
    joint_def.base.body_id_a = base;
    joint_def.base.body_id_b = slider;
    let joint = world.create_prismatic_joint(&joint_def);

    // Resting at the origin: no translation.
    assert!(abs_float(world.prismatic_joint_translation_value(joint).unwrap() as f64) < 1e-5);

    // Drive it along +x and let it move.
    world.apply_body_force_to_center(slider, Vec3::ZERO, true);
    world.set_body_linear_velocity(slider, Vec3::new(2.0, 0.0, 0.0));
    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }

    let translation = world.prismatic_joint_translation_value(joint).unwrap();
    assert!(translation > 0.5);

    // The speed follows the linear velocity along the axis.
    let speed = world.prismatic_joint_speed_value(joint).unwrap();
    assert!(abs_float((speed - 2.0) as f64) < 1e-3);
}

// "spherical spring and limit scalars round trip"
#[test]
fn spherical_scalars() {
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

    world.set_spherical_joint_spring_hertz(joint, 6.0);
    world.set_spherical_joint_spring_damping_ratio(joint, 0.35);
    assert!(abs_float((world.spherical_joint_spring_hertz(joint).unwrap() - 6.0) as f64) < 1e-6);
    assert!(
        abs_float((world.spherical_joint_spring_damping_ratio(joint).unwrap() - 0.35) as f64)
            < 1e-6
    );

    world.enable_spherical_joint_spring(joint, true);
    assert!(world.is_spherical_joint_spring_enabled(joint));

    world.enable_spherical_joint_motor(joint, true);
    assert!(world.is_spherical_joint_motor_enabled(joint));

    world.set_spherical_joint_cone_limit(joint, 0.5);
    world.enable_spherical_joint_cone_limit(joint, true);
    assert!(abs_float((world.spherical_joint_cone_angle(joint).unwrap() - 0.5) as f64) < 1e-6);
    assert!(world.is_spherical_joint_cone_limit_enabled(joint));

    world.set_spherical_joint_twist_limits(joint, -0.3, 0.3);
    assert!(
        abs_float((world.spherical_joint_lower_twist_limit(joint).unwrap() + 0.3) as f64) < 1e-6
    );
    assert!(
        abs_float((world.spherical_joint_upper_twist_limit(joint).unwrap() - 0.3) as f64) < 1e-6
    );
}

// "parallel spring scalars round trip"
#[test]
fn parallel_scalars() {
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

    world.set_parallel_joint_hertz(joint, 8.0);
    world.set_parallel_joint_damping_ratio(joint, 0.55);
    assert!(abs_float((world.parallel_joint_hertz(joint).unwrap() - 8.0) as f64) < 1e-6);
    assert!(abs_float((world.parallel_joint_damping_ratio(joint).unwrap() - 0.55) as f64) < 1e-6);
}

// MARK: - Joint lifecycle interaction

/// Creating a joint with collideConnected off must drop contacts that already exist between the
/// attached bodies.
// "creating a non-colliding joint clears existing contacts"
#[test]
fn create_joint_clears_contacts() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    let body_a = world.create_body(&def);

    def.position = Pos::new(0.5, 0.0, 0.0);
    let body_b = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = world.create_hull_shape(body_a, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    let _ = world.create_hull_shape(body_b, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    world.step(1.0f32 / 60.0, 4);
    assert!(!world.body_contact_data(body_a, 32).is_empty());

    let mut joint_def = RevoluteJointDef::default();
    joint_def.base.body_id_a = body_a;
    joint_def.base.body_id_b = body_b;
    joint_def.base.collide_connected = false;
    let _ = world.create_revolute_joint(&joint_def);

    // The contact is gone immediately; the joint does not wait for the next step.
    assert!(world.body_contact_data(body_a, 32).is_empty());

    // Stepping keeps them apart.
    world.step(1.0f32 / 60.0, 4);
    assert!(world.body_contact_data(body_a, 32).is_empty());
}

/// A weld across a static and a kinematic body must survive type changes and a disable/enable
/// cycle on one of the bodies.
// "a non-dynamic joint survives type changes and disable cycles"
#[test]
fn non_dynamic_joint_survives() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    let body_a = world.create_body(&def);

    def.position = Pos::new(2.0, 0.0, 0.0);
    let body_b = world.create_body(&def);

    let shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body_a, &shape_def, &make_cube_hull(0.5));
    let _ = world.create_hull_shape(body_b, &shape_def, &make_cube_hull(0.5));

    let mut joint_def = WeldJointDef::default();
    joint_def.base.body_id_a = body_a;
    joint_def.base.body_id_b = body_b;
    let weld = world.create_weld_joint(&joint_def);

    world.set_body_type(body_a, BodyType::StaticBody);
    world.set_body_type(body_b, BodyType::KinematicBody);
    world.step(1.0f32 / 60.0, 4);

    world.disable_body(body_b);
    world.step(1.0f32 / 60.0, 4);

    world.enable_body(body_b);
    world.step(1.0f32 / 60.0, 4);

    assert!(world.joint_is_valid(weld));
    assert_eq!(world.joint_type(weld).unwrap(), JointType::WeldJoint);
}

/// Creating a colliding joint leaves the contact in place.
// "creating a colliding joint keeps existing contacts"
#[test]
fn create_joint_keeps_contacts() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 0.0, 0.0);
    let body_a = world.create_body(&def);

    def.position = Pos::new(0.5, 0.0, 0.0);
    let body_b = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = world.create_hull_shape(body_a, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    let _ = world.create_hull_shape(body_b, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    world.step(1.0f32 / 60.0, 4);
    assert!(!world.body_contact_data(body_a, 32).is_empty());

    let mut joint_def = RevoluteJointDef::default();
    joint_def.base.body_id_a = body_a;
    joint_def.base.body_id_b = body_b;
    joint_def.base.collide_connected = true;
    let _ = world.create_revolute_joint(&joint_def);

    assert!(!world.body_contact_data(body_a, 32).is_empty());
}
