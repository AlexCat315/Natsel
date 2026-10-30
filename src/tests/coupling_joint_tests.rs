//! Ported from Tests/NatselTests/CouplingJointTests.swift.

use std::cell::RefCell;
use std::rc::Rc;
use std::sync::Arc;

use crate::collision::hull::make_box_hull;
use crate::math::functions::{get_twist_angle, length, sub_pos, unwind_angle};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::debug_draw::{DebugDraw, HexColor};
use crate::types::geometry::Capsule;
use crate::types::ids::{BodyID, JointID};
use crate::types::joint_defs::{
    GearJointDef, JointType, PrismaticJointDef, PulleyJointDef, RackAndPinionJointDef,
    RevoluteJointDef,
};
use crate::types::shape_def::ShapeDef;

// TODO(migration): `World::default()` and `World::gravity` are not yet confirmed public; reconcile
// during the build-fix phase if the compiler complains.

fn disk(world: &mut World, position: Vec3, radius: f32) -> BodyID {
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::from_vec3(position);
    let body = world.create_body(&def);
    let _ = world.create_capsule_shape(
        body,
        &ShapeDef::default(),
        Capsule::new(Vec3::new(0.0, 0.0, -0.05), Vec3::new(0.0, 0.0, 0.05), radius),
    );
    body
}

/// A revolute joint about world z at the body's position.
fn hinge(world: &mut World, ground: BodyID, body: BodyID, motor_speed: Option<f32>) -> JointID {
    let mut def = RevoluteJointDef::default();
    def.base.body_id_a = ground;
    def.base.body_id_b = body;
    def.base.local_frame_a = Transform::new(
        sub_pos(world.body_position(body), Pos::ZERO),
        Quat::IDENTITY,
    );
    def.base.local_frame_b = Transform::new(Vec3::new(0.0, 0.0, 0.0), Quat::IDENTITY);
    if let Some(speed) = motor_speed {
        def.enable_motor = true;
        def.motor_speed = speed;
        def.max_motor_torque = 1000.0;
    }
    world.create_revolute_joint(&def)
}

/// Unwrapped rotation about z.
struct AngleTracker {
    angle: f32,
    last: f32,
}

impl AngleTracker {
    fn new() -> Self {
        AngleTracker { angle: 0.0, last: 0.0 }
    }

    fn update(&mut self, q: Quat) {
        let twist = get_twist_angle(q);
        self.angle += unwind_angle(twist - self.last);
        self.last = twist;
    }
}

#[test]
fn gear() {
    // "gears keep their ratio over many revolutions"
    for use_references in [true, false] {
        let mut world = World::default();
        world.gravity = Vec3::new(0.0, 0.0, 0.0);
        let ground = world.create_body(&BodyDef::default());
        let a = disk(&mut world, Vec3::new(0.0, 0.0, 0.0), 0.5);
        let b = disk(&mut world, Vec3::new(1.5, 0.0, 0.0), 1.0);
        let hinge_a = hinge(&mut world, ground, a, Some(6.0));
        let hinge_b = hinge(&mut world, ground, b, None);

        let mut def = GearJointDef::default();
        def.base.body_id_a = a;
        def.base.body_id_b = b;
        def.ratio = 2.0;
        if use_references {
            def.reference_joint_a = hinge_a;
            def.reference_joint_b = hinge_b;
        }
        let gear = world.create_gear_joint(&def);
        assert_eq!(world.joint_type(gear), Some(JointType::GearJoint));
        assert_eq!(world.gear_joint_ratio(gear), Some(2.0));

        let mut angle_a = AngleTracker::new();
        let mut angle_b = AngleTracker::new();
        for _ in 0..600 {
            world.step(1.0 / 60.0, 4);
            angle_a.update(world.body_transform(a).q);
            angle_b.update(world.body_transform(b).q);
        }
        // A turned many times; B turned half as fast the other way.
        assert!(angle_a.angle > 50.0);
        assert!((angle_a.angle + 2.0 * angle_b.angle).abs() < 0.02);
        assert!(world
            .joint_constraint_torque(gear)
            .map_or(false, |t| length(t) > 0.0));
    }
}

#[test]
fn rack_and_pinion() {
    // "rack follows the pinion"
    let mut world = World::default();
    world.gravity = Vec3::new(0.0, 0.0, 0.0);
    let ground = world.create_body(&BodyDef::default());
    let pinion = disk(&mut world, Vec3::new(0.0, 0.0, 0.0), 0.25);
    let pinion_hinge = hinge(&mut world, ground, pinion, Some(2.0));

    let mut body_def = BodyDef::default();
    body_def.type_ = BodyType::DynamicBody;
    body_def.position = Pos::new(0.0, -0.4, 0.0);
    let rack = world.create_body(&body_def);
    let _ = world.create_hull_shape(
        rack,
        &ShapeDef::default(),
        &make_box_hull(2.0, 0.1, 0.1),
    );
    let mut slide = PrismaticJointDef::default();
    slide.base.body_id_a = ground;
    slide.base.body_id_b = rack;
    slide.base.local_frame_a = Transform::new(Vec3::new(0.0, -0.4, 0.0), Quat::IDENTITY);
    let rack_slide = world.create_prismatic_joint(&slide);

    let mut def = RackAndPinionJointDef::default();
    def.base.body_id_a = pinion;
    def.base.body_id_b = rack;
    def.ratio = 0.25;
    def.reference_pinion_joint = pinion_hinge;
    def.reference_rack_joint = rack_slide;
    let joint = world.create_rack_and_pinion_joint(&def);
    assert!(!joint.is_null());

    let mut angle = AngleTracker::new();
    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
        angle.update(world.body_transform(pinion).q);
    }
    let translation = world.body_position(rack).x;
    assert!(angle.angle > 3.0);
    assert!((translation - 0.25 * angle.angle).abs() < 0.01);
    assert!((world.body_position(rack).y + 0.4).abs() < 1.0e-3);
}

#[test]
fn pulley() {
    // "pulley keeps the rope length and the heavier side descends"
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    fn box_(world: &mut World, x: f32, density: f32) -> BodyID {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(x as f64, 0.0, 0.0);
        let body = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        shape_def.density = density;
        let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.2, 0.2, 0.2));
        body
    }
    let light = box_(&mut world, -1.0, 100.0);
    let heavy = box_(&mut world, 1.0, 200.0);

    let mut def = PulleyJointDef::default();
    def.base.body_id_a = light;
    def.base.body_id_b = heavy;
    def.base.local_frame_a = Transform::new(Vec3::new(0.0, 0.2, 0.0), Quat::IDENTITY);
    def.base.local_frame_b = Transform::new(Vec3::new(0.0, 0.2, 0.0), Quat::IDENTITY);
    def.ground_anchor_a = Pos::new(-1.0, 3.0, 0.0);
    def.ground_anchor_b = Pos::new(1.0, 3.0, 0.0);
    def.ratio = 1.0;
    def.base.collide_connected = true;
    let pulley = world.create_pulley_joint(&def);
    let range = world.pulley_joint_length_range(pulley).unwrap();
    assert!((range.0 - 5.6).abs() < 1.0e-4);
    let _ = ground;

    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }
    let lengths = world.pulley_joint_current_lengths(pulley).unwrap();
    assert!((lengths.0 + lengths.1 - 5.6).abs() < 0.01);
    assert!(world.body_position(heavy).y < -0.5);
    assert!(world.body_position(light).y > 0.5);
    // Expected acceleration (m2 - m1) / (m1 + m2) g = g / 3.
    let expected = 0.5 * (10.0 / 3.0) * 1.0;
    assert!((world.body_position(light).y - expected).abs() < 0.1);
    assert!(world
        .joint_constraint_force(pulley)
        .map_or(false, |f| length(f) > 0.0));

    // Slack rope: the light box falls freely until the rope tightens again.
    world.set_pulley_joint_length_range(pulley, 4.0, 8.0);
    world.set_body_linear_velocity(light, Vec3::new(0.0, 0.0, 0.0));
    world.set_body_linear_velocity(heavy, Vec3::new(0.0, 0.0, 0.0));
    for _ in 0..10 {
        world.step(1.0 / 60.0, 4);
    }
    assert!(world.body_linear_velocity(light).y < -1.0);
}

#[test]
fn generic_api() {
    // "coupling joints draw and report through the generic joint API"
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    let a = disk(&mut world, Vec3::new(0.0, 0.0, 0.0), 0.5);
    let b = disk(&mut world, Vec3::new(1.5, 0.0, 0.0), 1.0);
    let _ = hinge(&mut world, ground, a, None);
    let _ = hinge(&mut world, ground, b, None);
    let mut def = GearJointDef::default();
    def.base.body_id_a = a;
    def.base.body_id_b = b;
    let gear = world.create_gear_joint(&def);
    world.step(1.0 / 60.0, 4);
    let segments = Rc::new(RefCell::new(0usize));
    let seg = segments.clone();
    let mut options = DebugDraw::new();
    options.draw_joints = true;
    options.draw_segment = Some(Arc::new(move |_a: Pos, _b: Pos, _c: HexColor| {
        *seg.borrow_mut() += 1;
    }));
    world.draw(&options, u64::MAX);
    assert!(*segments.borrow() > 0);
    world.destroy_joint(gear.index1 - 1, true);
    assert!(!world.joint_is_valid(gear));
}

#[test]
fn stale_joint_ids() {
    // "a recycled joint slot invalidates the old id"
    let mut world = World::default();
    let ground = world.create_body(&BodyDef::default());
    let a = disk(&mut world, Vec3::new(0.0, 0.0, 0.0), 0.5);
    let first = hinge(&mut world, ground, a, None);
    world.destroy_joint(first.index1 - 1, false);
    let second = hinge(&mut world, ground, a, None);
    assert_eq!(second.index1, first.index1);
    assert_ne!(second.generation, first.generation);
    assert!(!world.joint_is_valid(first));
    assert!(world.joint_is_valid(second));
}
