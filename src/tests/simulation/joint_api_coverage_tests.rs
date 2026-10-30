//! Ported from Tests/NatselTests/Simulation/JointApiCoverageTests.swift.
//!
//! Round-trip checks for the per-type joint tuning APIs.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::{BodyID, JointID};
use crate::types::joint_defs::{
    JointType, MotorJointDef, ParallelJointDef, PrismaticJointDef, RevoluteJointDef,
    SphericalJointDef, WeldJointDef, WheelJointDef,
};
use crate::types::shape_def::ShapeDef;

struct Fixture {
    world: World,
    ground: BodyID,
    body: BodyID,
}

fn make_fixture() -> Fixture {
    let mut f = Fixture {
        world: World::default(),
        ground: BodyID::NULL,
        body: BodyID::NULL,
    };

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    f.ground = f.world.create_body(&def);

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(1.0, 0.0, 0.0);
    f.body = f.world.create_body(&def);

    let mut sd = ShapeDef::default();
    sd.density = 1.0;
    let _ = f
        .world
        .create_hull_shape(f.body, &sd, &make_box_hull(0.4, 0.4, 0.4));
    f
}

// "common joint accessors"
#[test]
fn common_accessors() {
    let mut f = make_fixture();

    let mut def = RevoluteJointDef::default();
    def.base.body_id_a = f.ground;
    def.base.body_id_b = f.body;
    let joint = f.world.create_revolute_joint(&def);

    assert!(f.world.joint_is_valid(joint));
    assert_eq!(f.world.joint_type(joint).unwrap(), JointType::RevoluteJoint);
    let bodies = f.world.joint_bodies(joint).unwrap();
    assert_eq!(bodies.0.index1, f.ground.index1);
    assert_eq!(bodies.1.index1, f.body.index1);

    f.world.destroy_joint(joint.index1 - 1, false);
    assert!(!f.world.joint_is_valid(joint));
}

// "revolute joint setters round trip"
#[test]
fn revolute_api() {
    let mut f = make_fixture();

    let mut def = RevoluteJointDef::default();
    def.base.body_id_a = f.ground;
    def.base.body_id_b = f.body;
    let joint = f.world.create_revolute_joint(&def);

    assert!(f.world.set_revolute_joint_limits(joint, -0.5, 0.75));
    let limits = f.world.revolute_joint_limits(joint).unwrap();
    assert!(limits.0 == -0.5 && limits.1 == 0.75);

    assert!(f.world.enable_revolute_joint_limit(joint, true));
    assert!(f.world.is_revolute_joint_limit_enabled(joint));

    assert!(f.world.enable_revolute_joint_motor(joint, true));
    assert!(f.world.is_revolute_joint_motor_enabled(joint));
    assert!(f.world.set_revolute_joint_motor_speed(joint, 2.5));
    assert_eq!(f.world.revolute_joint_motor_speed(joint).unwrap(), 2.5);
    assert!(f.world.set_revolute_joint_max_motor_torque(joint, 40.0));
    assert_eq!(f.world.revolute_joint_max_motor_torque(joint).unwrap(), 40.0);

    assert!(f.world.enable_revolute_joint_spring(joint, true));
    assert!(f.world.is_revolute_joint_spring_enabled(joint));
    assert!(f.world.set_revolute_joint_spring_hertz(joint, 3.0));
    assert_eq!(f.world.revolute_joint_spring_hertz(joint).unwrap(), 3.0);
    assert!(f.world.set_revolute_joint_spring_damping_ratio(joint, 0.7));
    assert_eq!(
        f.world.revolute_joint_spring_damping_ratio(joint).unwrap(),
        0.7
    );

    assert!(f.world.revolute_joint_angle(joint).is_some());
}

// "prismatic joint setters round trip"
#[test]
fn prismatic_api() {
    let mut f = make_fixture();

    let mut def = PrismaticJointDef::default();
    def.base.body_id_a = f.ground;
    def.base.body_id_b = f.body;
    let joint = f.world.create_prismatic_joint(&def);

    assert!(f.world.set_prismatic_joint_limits(joint, -1.0, 2.0));
    let limits = f.world.prismatic_joint_limits(joint).unwrap();
    assert!(limits.0 == -1.0 && limits.1 == 2.0);

    assert!(f.world.enable_prismatic_joint_limit(joint, true));
    assert!(f.world.is_prismatic_joint_limit_enabled(joint));

    assert!(f.world.enable_prismatic_joint_motor(joint, true));
    assert!(f.world.is_prismatic_joint_motor_enabled(joint));
    assert!(f.world.set_prismatic_joint_motor_speed(joint, 1.5));
    assert_eq!(f.world.prismatic_joint_motor_speed(joint).unwrap(), 1.5);
    assert!(f.world.set_prismatic_joint_max_motor_force(joint, 30.0));
    assert_eq!(f.world.prismatic_joint_max_motor_force(joint).unwrap(), 30.0);

    assert!(f.world.enable_prismatic_joint_spring(joint, true));
    assert!(f.world.is_prismatic_joint_spring_enabled(joint));
    assert!(f.world.set_prismatic_joint_spring_hertz(joint, 5.0));
    assert_eq!(f.world.prismatic_joint_spring_hertz(joint).unwrap(), 5.0);
    assert!(f.world.set_prismatic_joint_spring_damping_ratio(joint, 0.5));
    assert_eq!(
        f.world.prismatic_joint_spring_damping_ratio(joint).unwrap(),
        0.5
    );
    assert!(f.world.set_prismatic_joint_target_translation(joint, 0.8));
    assert_eq!(
        f.world.prismatic_joint_target_translation(joint).unwrap(),
        0.8
    );
}

// "wheel joint setters round trip"
#[test]
fn wheel_api() {
    let mut f = make_fixture();

    let mut def = WheelJointDef::default();
    def.base.body_id_a = f.ground;
    def.base.body_id_b = f.body;
    let joint = f.world.create_wheel_joint(&def);

    assert!(f.world.enable_wheel_joint_spin_motor(joint, true));
    assert!(f.world.is_wheel_joint_spin_motor_enabled(joint));
    assert!(f.world.set_wheel_joint_spin_speed(joint, 3.0));
    assert_eq!(f.world.wheel_joint_spin_speed(joint).unwrap(), 3.0);
    assert!(f.world.set_wheel_joint_max_spin_torque(joint, 25.0));
    assert_eq!(f.world.wheel_joint_max_spin_torque(joint).unwrap(), 25.0);

    assert!(f.world.set_wheel_joint_suspension_spring(joint, 4.0, 0.6));
    let suspension = f.world.wheel_joint_suspension_spring(joint).unwrap();
    assert!(suspension.0 == 4.0 && suspension.1 == 0.6);

    assert!(f.world.set_wheel_joint_suspension_limits(joint, -0.2, 0.2));
    let sus_limits = f.world.wheel_joint_suspension_limits(joint).unwrap();
    assert!(sus_limits.0 == -0.2 && sus_limits.1 == 0.2);

    assert!(f.world.enable_wheel_joint_steering(joint, true));
    assert!(f.world.is_wheel_joint_steering_enabled(joint));
    assert!(f.world.set_wheel_joint_target_steering_angle(joint, 0.3));
    assert_eq!(
        f.world.wheel_joint_target_steering_angle(joint).unwrap(),
        0.3
    );
    assert!(f.world.set_wheel_joint_max_steering_torque(joint, 15.0));
    assert_eq!(f.world.wheel_joint_max_steering_torque(joint).unwrap(), 15.0);

    assert!(f.world.set_wheel_joint_steering_spring(joint, 2.0, 0.4));
    let steering = f.world.wheel_joint_steering_spring(joint).unwrap();
    assert!(steering.0 == 2.0 && steering.1 == 0.4);

    assert!(f.world.set_wheel_joint_steering_limits(joint, -0.5, 0.5));
    let steer_limits = f.world.wheel_joint_steering_limits(joint).unwrap();
    assert!(steer_limits.0 == -0.5 && steer_limits.1 == 0.5);
}

// "spherical, motor, weld and parallel setters round trip"
#[test]
fn other_joint_apis() {
    let mut f = make_fixture();

    // Spherical
    let mut sdef = SphericalJointDef::default();
    sdef.base.body_id_a = f.ground;
    sdef.base.body_id_b = f.body;
    let spherical = f.world.create_spherical_joint(&sdef);

    assert!(f.world.set_spherical_joint_cone_limit(spherical, 0.6));
    assert!(f.world.enable_spherical_joint_cone_limit(spherical, true));
    let cone = f.world.spherical_joint_cone_limit(spherical).unwrap();
    assert!(cone.0 && cone.1 == 0.6);

    assert!(f.world.set_spherical_joint_twist_limits(spherical, -0.4, 0.4));
    let twist = f.world.spherical_joint_twist_limits(spherical).unwrap();
    assert!(twist.0 == -0.4 && twist.1 == 0.4);

    assert!(f.world.enable_spherical_joint_motor(spherical, true));
    assert!(f
        .world
        .set_spherical_joint_motor_velocity(spherical, Vec3::new(0.5, 0.0, 1.0)));
    assert_eq!(
        f.world.spherical_joint_motor_velocity(spherical).unwrap(),
        Vec3::new(0.5, 0.0, 1.0)
    );
    assert!(f
        .world
        .set_spherical_joint_max_motor_torque(spherical, 20.0));
    assert_eq!(
        f.world
            .spherical_joint_max_motor_torque(spherical)
            .unwrap(),
        20.0
    );

    // Motor
    let mut mdef = MotorJointDef::default();
    mdef.base.body_id_a = f.ground;
    mdef.base.body_id_b = f.body;
    let motor = f.world.create_motor_joint(&mdef);

    assert!(f
        .world
        .set_motor_joint_linear_velocity(motor, Vec3::new(1.0, 0.0, 0.0)));
    assert_eq!(
        f.world.motor_joint_linear_velocity(motor).unwrap(),
        Vec3::new(1.0, 0.0, 0.0)
    );
    assert!(f
        .world
        .set_motor_joint_angular_velocity(motor, Vec3::new(0.0, 2.0, 0.0)));
    assert_eq!(
        f.world.motor_joint_angular_velocity(motor).unwrap(),
        Vec3::new(0.0, 2.0, 0.0)
    );
    assert!(f.world.set_motor_joint_max_velocity_force(motor, 50.0));
    assert_eq!(
        f.world.motor_joint_max_velocity_force(motor).unwrap(),
        50.0
    );
    assert!(f.world.set_motor_joint_max_velocity_torque(motor, 60.0));
    assert_eq!(
        f.world.motor_joint_max_velocity_torque(motor).unwrap(),
        60.0
    );
    assert!(f.world.set_motor_joint_linear_spring(motor, 3.0, 0.5));
    let linear_spring = f.world.motor_joint_linear_spring(motor).unwrap();
    assert!(linear_spring.0 == 3.0 && linear_spring.1 == 0.5);
    assert!(f.world.set_motor_joint_angular_spring(motor, 2.0, 0.3));
    let angular_spring = f.world.motor_joint_angular_spring(motor).unwrap();
    assert!(angular_spring.0 == 2.0 && angular_spring.1 == 0.3);
    assert!(f.world.set_motor_joint_max_spring_force(motor, 80.0));
    assert_eq!(f.world.motor_joint_max_spring_force(motor).unwrap(), 80.0);
    assert!(f.world.set_motor_joint_max_spring_torque(motor, 90.0));
    assert_eq!(f.world.motor_joint_max_spring_torque(motor).unwrap(), 90.0);

    // Weld
    let mut wdef = WeldJointDef::default();
    wdef.base.body_id_a = f.ground;
    wdef.base.body_id_b = f.body;
    let weld = f.world.create_weld_joint(&wdef);

    assert!(f.world.set_weld_joint_linear_spring(weld, 6.0, 0.8));
    let weld_linear = f.world.weld_joint_linear_spring(weld).unwrap();
    assert!(weld_linear.0 == 6.0 && weld_linear.1 == 0.8);
    assert!(f.world.set_weld_joint_angular_spring(weld, 7.0, 0.9));
    let weld_angular = f.world.weld_joint_angular_spring(weld).unwrap();
    assert!(weld_angular.0 == 7.0 && weld_angular.1 == 0.9);

    // Parallel
    let mut pdef = ParallelJointDef::default();
    pdef.base.body_id_a = f.ground;
    pdef.base.body_id_b = f.body;
    let parallel = f.world.create_parallel_joint(&pdef);

    assert!(f.world.set_parallel_joint_spring(parallel, 9.0, 0.2));
    let parallel_spring = f.world.parallel_joint_spring(parallel).unwrap();
    assert!(parallel_spring.0 == 9.0 && parallel_spring.1 == 0.2);
    assert!(f.world.set_parallel_joint_max_torque(parallel, 100.0));
    assert_eq!(f.world.parallel_joint_max_torque(parallel).unwrap(), 100.0);
}

// "type mismatches are rejected"
#[test]
fn type_mismatch() {
    let mut f = make_fixture();

    let mut def = RevoluteJointDef::default();
    def.base.body_id_a = f.ground;
    def.base.body_id_b = f.body;
    let joint: JointID = f.world.create_revolute_joint(&def);

    // Prismatic setters must not touch a revolute joint.
    assert!(!f.world.set_prismatic_joint_motor_speed(joint, 5.0));
    assert!(f.world.prismatic_joint_motor_speed(joint).is_none());
    assert!(f.world.wheel_joint_spin_speed(joint).is_none());
}
