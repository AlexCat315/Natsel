//! Ported from Tests/NatselTests/SixDOFJointTests.swift.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::functions::{
    atan2_approx, compute_cos_sin, cross, dot, dot_quat, get_axis_angle, get_twist_angle, length,
    make_quat_from_axis_angle, rotate_vector, sub_pos,
};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::World;
use crate::simulation::six_dof_joint::{
    SixDOFAxis, SixDOFAxisSettings, SixDOFMotion, SixDOFMotorMode, SixDOFJointDef,
    SixDOFSwingLimit,
};
use crate::tests::world_position_assertions::distance;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::JointID;
use crate::types::joint_defs::JointType;
use crate::types::shape_def::ShapeDef;

// TODO(migration): `World::default()`, `World::gravity`, and `World::six_dof_pool` are not yet
// confirmed public; reconcile during the build-fix phase if the compiler complains.

struct Rig {
    world: World,
    ground: BodyID,
    body: BodyID,
}

impl Rig {
    fn new(gravity: Vec3, offset: Vec3) -> Self {
        let mut world = World::default();
        world.gravity = gravity;
        world.enable_sleep = false;
        let ground = world.create_body(&BodyDef::default());
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::from_vec3(offset);
        let body = world.create_body(&def);
        let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(0.2, 0.1, 0.1));
        Rig { world, ground, body }
    }

    fn joint(&mut self, configure: impl FnOnce(&mut SixDOFJointDef)) -> JointID {
        let mut def = SixDOFJointDef::default();
        def.base.body_id_a = self.ground;
        def.base.body_id_b = self.body;
        def.base.local_frame_a = Transform::IDENTITY;
        def.base.local_frame_b = Transform::new(
            sub_pos(Pos::ZERO, self.world.body_position(self.body)),
            Quat::IDENTITY,
        );
        configure(&mut def);
        self.world.create_six_dof_joint(def)
    }

    fn run(&mut self, steps: i32) {
        for _ in 0..steps {
            self.world.step(1.0 / 60.0, 4);
        }
    }
}

#[test]
fn weld() {
    // "all axes locked behaves like a weld"
    let mut rig = Rig::new(Vec3::new(0.0, -10.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let joint = rig.joint(|def| def.lock_all(&[]));
    assert_eq!(rig.world.joint_type(joint), Some(JointType::SixDOFJoint));
    rig.run(120);
    assert!(distance(rig.world.body_position(rig.body), Vec3::new(1.0, 0.0, 0.0)) < 0.01);
    let q = rig.world.body_transform(rig.body).q;
    assert!(q.s.abs() > 0.9995);
    assert!(rig
        .world
        .joint_constraint_force(joint)
        .map_or(false, |f| length(f) > 1.0));
}

#[test]
fn slider() {
    // "slider with limits"
    let mut rig = Rig::new(Vec3::new(10.0, -10.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let joint = rig.joint(|def| {
        def.lock_all(&[SixDOFAxis::TranslationX]);
        def[SixDOFAxis::TranslationX] =
            SixDOFAxisSettings::with_limits(SixDOFMotion::Limited, -0.5, 0.5);
    });
    rig.run(180);
    let pose = rig.world.six_dof_joint_pose(joint).unwrap();
    assert!((pose.p.x - 0.5).abs() < 0.01);
    assert!(pose.p.y.abs() < 0.01 && pose.p.z.abs() < 0.01);
    assert!(pose.q.s.abs() > 0.9995);
    // Free it and it slides away.
    let mut free = rig
        .world
        .six_dof_joint_axis(joint, SixDOFAxis::TranslationX)
        .unwrap();
    free.motion = SixDOFMotion::Free;
    rig.world
        .set_six_dof_joint_axis(joint, SixDOFAxis::TranslationX, free);
    rig.run(30);
    assert!(rig.world.six_dof_joint_pose(joint).unwrap().p.x > 1.0);
}

#[test]
fn hinge() {
    // "hinge with a velocity motor"
    let mut rig = Rig::new(Vec3::new(0.0, -10.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let _ = rig.joint(|def| {
        def.lock_all(&[SixDOFAxis::RotationX]);
        def[SixDOFAxis::RotationX].motor_mode = SixDOFMotorMode::Velocity;
        def[SixDOFAxis::RotationX].motor_target = 2.0;
        def[SixDOFAxis::RotationX].max_motor_force = 500.0;
    });
    rig.run(120);
    let w = rig.world.body_angular_velocity(rig.body);
    assert!((w.x - 2.0).abs() < 0.02);
    assert!(w.y.abs() < 0.02 && w.z.abs() < 0.02);
    assert!(distance(rig.world.body_position(rig.body), Vec3::new(1.0, 0.0, 0.0)) < 0.01);
}

#[test]
fn pyramid() {
    // "pyramid swing limit"
    let mut rig = Rig::new(Vec3::new(0.0, -10.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let limit: f32 = 30.0 * b3::DEG_TO_RAD;
    let joint = rig.joint(|def| {
        def.lock_all(&[SixDOFAxis::RotationZ]);
        def[SixDOFAxis::RotationZ] =
            SixDOFAxisSettings::with_limits(SixDOFMotion::Limited, -limit, limit);
    });
    rig.run(180);
    // The body hangs from the pivot toward -y and rests on the lower limit.
    let pose = rig.world.six_dof_joint_pose(joint).unwrap();
    let (angle, axis) = get_axis_angle(pose.q);
    assert!((angle.abs() - limit).abs() < 0.02);
    assert!((axis.z.abs() - 1.0).abs() < 1.0e-3);
}

#[test]
fn cone() {
    // "cone swing limit"
    let mut rig = Rig::new(Vec3::new(0.0, -10.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let angle: f32 = 20.0 * b3::DEG_TO_RAD;
    let _ = rig.joint(|def| {
        def.lock_all(&[
            SixDOFAxis::RotationX,
            SixDOFAxis::RotationY,
            SixDOFAxis::RotationZ,
        ]);
        def[SixDOFAxis::RotationY].motion = SixDOFMotion::Limited;
        def[SixDOFAxis::RotationZ].motion = SixDOFMotion::Limited;
        def.swing_limit = SixDOFSwingLimit::Cone(angle);
    });
    rig.world
        .set_body_angular_velocity(rig.body, Vec3::new(0.0, 3.0, 4.0));
    let mut max_angle: f32 = 0.0;
    for _ in 0..180 {
        rig.world.step(1.0 / 60.0, 4);
        let x = rotate_vector(rig.world.body_transform(rig.body).q, Vec3::new(1.0, 0.0, 0.0));
        max_angle = max_angle.max(atan2_approx(length(cross(Vec3::new(1.0, 0.0, 0.0), x)), x.x));
    }
    assert!(max_angle < angle + 0.04);
    assert!(max_angle > angle - 0.05);
}

#[test]
fn position_motors() {
    // "position motors reach their targets"
    // Anchor at the center of mass so the translation and rotation springs do not couple.
    let mut rig = Rig::new(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.0, 0.0));
    let target = make_quat_from_axis_angle(Vec3::new(0.0, 0.0, 1.0), 0.25 * b3::PI);
    let joint = rig.joint(|def| {
        for axis in SixDOFAxis::ALL {
            def[axis].motor_mode = SixDOFMotorMode::Position;
            def[axis].max_motor_force = 1.0e5;
            def[axis].motor_hertz = 4.0;
            def[axis].motor_damping_ratio = 1.0;
        }
        def[SixDOFAxis::TranslationX].motor_target = 0.3;
        def.target_rotation = target;
    });
    rig.run(240);
    let pose = rig.world.six_dof_joint_pose(joint).unwrap();
    assert!(distance(pose.p, Vec3::new(0.3, 0.0, 0.0)) < 0.01);
    assert!(dot_quat(pose.q, target).abs() > 0.9995);

    rig.world
        .set_six_dof_joint_motor_target(joint, SixDOFAxis::TranslationX, -0.2);
    rig.world
        .set_six_dof_joint_target_rotation(joint, Quat::IDENTITY);
    rig.run(240);
    let back = rig.world.six_dof_joint_pose(joint).unwrap();
    assert!(distance(back.p, Vec3::new(-0.2, 0.0, 0.0)) < 0.01);
    assert!(back.q.s.abs() > 0.9995);
}

#[test]
fn friction() {
    // "friction holds a free axis against a weaker force"
    let mass: f32 = 0.4 * 0.2 * 0.2 * 1000.0;
    for (friction, holds) in [
        (2.0 * mass * 10.0, true),
        (0.5 * mass * 10.0, false),
    ] {
        let mut rig = Rig::new(Vec3::new(10.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
        let joint = rig.joint(|def| {
            def.lock_all(&[SixDOFAxis::TranslationX]);
            def[SixDOFAxis::TranslationX].max_friction = friction;
        });
        rig.run(60);
        let x = rig.world.six_dof_joint_pose(joint).unwrap().p.x;
        if holds {
            assert!(x.abs() < 1.0e-3);
        } else {
            assert!(x > 0.5);
        }
    }
}

#[test]
fn lifecycle() {
    // "destroying six DOF joints recycles their state"
    let mut rig = Rig::new(Vec3::new(0.0, -10.0, 0.0), Vec3::new(1.0, 0.0, 0.0));
    let first = rig.joint(|def| def.lock_all(&[]));
    rig.run(2);
    rig.world.destroy_joint(first.index1 - 1, true);
    assert_eq!(rig.world.six_dof_pool.id_count(), 0);
    let second = rig.joint(|def| def.lock_all(&[SixDOFAxis::RotationX]));
    assert_eq!(
        rig.world
            .six_dof_joint_axis(second, SixDOFAxis::RotationX)
            .unwrap()
            .motion,
        SixDOFMotion::Free
    );
    assert!(rig
        .world
        .six_dof_joint_axis(first, SixDOFAxis::RotationX)
        .is_none());
    rig.world.destroy_body(rig.body);
    assert_eq!(rig.world.six_dof_pool.id_count(), 0);
}
