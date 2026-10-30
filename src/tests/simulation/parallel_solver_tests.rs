//! Ported from Tests/NatselTests/Simulation/ParallelSolverTests.swift.
//!
//! Scenes that exercise every solver stage. Each must give the same bits for any worker count.

use crate::collision::hull::{make_box_hull, make_cube_hull};
use crate::math::functions::{compute_cos_sin, length};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::path_joint::{ConstraintPath, PathMotorMode, PathRotationConstraint};
use crate::simulation::six_dof_joint::{SixDOFAxis, SixDOFAxisSettings, SixDOFMotion};
use crate::simulation::World;
use crate::soft_body::soft_body_runtime::SoftBodyDef;
use crate::soft_body::soft_body_settings::{SoftBodyBendType, SoftBodySettings, SoftBodyVertexAttributes};
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::BodyID;
use crate::types::joint_defs::{DistanceJointDef, PathJointDef, RevoluteJointDef, SphericalJointDef};
use crate::types::shape_def::ShapeDef;
use crate::vehicle::vehicle_types::{
    VehicleControllerSettings, VehicleDef, VehicleDifferentialSettings,
    WheeledVehicleControllerSettings, WheelSettings,
};
use crate::simulation::six_dof_joint::SixDOFJointDef;
use crate::types::joint_defs::WeldJointDef;

use crate::tests::simulation::human::{create_human, Human};
use crate::tests::simulation::impulse_determinism_tests::make_mesh_mix;

/// Scenes that exercise every solver stage. Each must give the same bits for any worker count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ParallelScene {
    Pyramid,
    JointGrid,
    MeshMix,
    Restitution,
    MixedJoints,
    Ragdolls,
    FastAndSleepy,
    Paths,
}

impl ParallelScene {
    /// Swift's `CaseIterable.allCases`, in declaration order.
    pub const ALL: [ParallelScene; 8] = [
        ParallelScene::Pyramid,
        ParallelScene::JointGrid,
        ParallelScene::MeshMix,
        ParallelScene::Restitution,
        ParallelScene::MixedJoints,
        ParallelScene::Ragdolls,
        ParallelScene::FastAndSleepy,
        ParallelScene::Paths,
    ];

    /// The Swift raw value, used in assertion messages.
    pub fn name(self) -> &'static str {
        match self {
            ParallelScene::Pyramid => "pyramid",
            ParallelScene::JointGrid => "jointGrid",
            ParallelScene::MeshMix => "meshMix",
            ParallelScene::Restitution => "restitution",
            ParallelScene::MixedJoints => "mixedJoints",
            ParallelScene::Ragdolls => "ragdolls",
            ParallelScene::FastAndSleepy => "fastAndSleepy",
            ParallelScene::Paths => "paths",
        }
    }

    pub fn make(self, workers: i32) -> World {
        let mut world = match self {
            ParallelScene::MeshMix => make_mesh_mix(true).0,
            _ => World::default(),
        };
        world.worker_count = workers;
        match self {
            ParallelScene::Pyramid => build_pyramid(&mut world),
            ParallelScene::JointGrid => build_joint_grid(&mut world),
            ParallelScene::MeshMix => {}
            ParallelScene::Restitution => build_restitution(&mut world),
            ParallelScene::MixedJoints => build_mixed_joints(&mut world),
            ParallelScene::Ragdolls => build_ragdolls(&mut world),
            ParallelScene::FastAndSleepy => build_fast_and_sleepy(&mut world),
            ParallelScene::Paths => build_paths(&mut world),
        }
        world
    }
}

fn ground(world: &mut World) -> BodyID {
    let mut def = BodyDef::default();
    def.position = Pos::new(0.0, -1.0, 0.0);
    let body = world.create_body(&def);
    let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_box_hull(100.0, 1.0, 100.0));
    body
}

fn build_pyramid(world: &mut World) {
    world.enable_sleep = false;
    let _ = ground(world);
    let cube = make_cube_hull(0.5);
    let base = 24;
    for row in 0..base {
        for column in 0..(base - row) {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(
                (column as f32 - 0.5 * (base - row - 1) as f32) as f64,
                (0.5 + row as f32) as f64,
                0.0,
            );
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(body, &ShapeDef::default(), &cube);
        }
    }
}

fn build_joint_grid(world: &mut World) {
    world.gravity = Vec3::new(0.0, -10.0, 0.0);
    let n = 30;
    let mut bodies: Vec<BodyID> = Vec::new();
    for k in 0..n {
        for i in 0..n {
            let mut def = BodyDef::default();
            def.type_ = if i == 0 {
                BodyType::StaticBody
            } else {
                BodyType::DynamicBody
            };
            def.position = Pos::new(k as f64, (-i) as f64, 0.0);
            let body = world.create_body(&def);
            let _ = world.create_sphere_shape(
                body,
                &ShapeDef::default(),
                Sphere::new(Vec3::ZERO, 0.4),
            );
            let mut joint = SphericalJointDef::default();
            if i > 0 {
                joint.base.body_id_a = bodies[bodies.len() - 1];
                joint.base.body_id_b = body;
                joint.base.local_frame_a.p = Vec3::new(0.0, -0.5, 0.0);
                joint.base.local_frame_b.p = Vec3::new(0.0, 0.5, 0.0);
                let _ = world.create_spherical_joint(&joint);
            }
            if k > 0 {
                joint.base.body_id_a = bodies[bodies.len() - n as usize];
                joint.base.body_id_b = body;
                joint.base.local_frame_a.p = Vec3::new(0.5, 0.0, 0.0);
                joint.base.local_frame_b.p = Vec3::new(-0.5, 0.0, 0.0);
                let _ = world.create_spherical_joint(&joint);
            }
            bodies.push(body);
        }
    }
}

fn build_restitution(world: &mut World) {
    let _ = ground(world);
    let mut shape_def = ShapeDef::default();
    shape_def.base_material.restitution = 0.8;
    for i in 0..400 {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(
            ((i % 20) as f32 * 1.1 - 10.0) as f64,
            (1.0 + (i / 20) as f32 * 1.1) as f64,
            ((i % 7) as f32 * 0.05) as f64,
        );
        let body = world.create_body(&def);
        if i % 2 == 0 {
            let _ = world.create_sphere_shape(
                body,
                &shape_def,
                Sphere::new(Vec3::ZERO, 0.5),
            );
        } else {
            let _ = world.create_hull_shape(body, &shape_def, &make_cube_hull(0.45));
        }
    }
}

fn build_mixed_joints(world: &mut World) {
    let anchor = ground(world);
    for chain in 0..12 {
        let mut previous = anchor;
        for link in 0..12 {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(
                (chain as f32 * 2.0) as f64,
                (12.0 - link as f32) as f64,
                0.0,
            );
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(
                body,
                &ShapeDef::default(),
                &make_box_hull(0.4, 0.4, 0.4),
            );
            let pivot = Pos::new(
                (chain as f32 * 2.0) as f64,
                (12.5 - link as f32) as f64,
                0.0,
            );
            let frame_a = Transform::new(world.body_local_point(previous, pivot), Quat::IDENTITY);
            let frame_b = Transform::new(world.body_local_point(body, pivot), Quat::IDENTITY);
            match (chain + link) % 4 {
                0 => {
                    let mut joint = RevoluteJointDef::default();
                    joint.base.body_id_a = previous;
                    joint.base.body_id_b = body;
                    joint.base.local_frame_a = frame_a;
                    joint.base.local_frame_b = frame_b;
                    joint.enable_motor = true;
                    joint.motor_speed = 1.0;
                    joint.max_motor_torque = 50.0;
                    let _ = world.create_revolute_joint(&joint);
                }
                1 => {
                    let mut joint = SixDOFJointDef::default();
                    joint.base.body_id_a = previous;
                    joint.base.body_id_b = body;
                    joint.base.local_frame_a = frame_a;
                    joint.base.local_frame_b = frame_b;
                    joint.lock_all(&[SixDOFAxis::RotationX, SixDOFAxis::RotationZ]);
                    joint[SixDOFAxis::RotationZ] =
                        SixDOFAxisSettings::with_limits(SixDOFMotion::Limited, -0.5, 0.5);
                    let _ = world.create_six_dof_joint(joint);
                }
                2 => {
                    let mut joint = WeldJointDef::default();
                    joint.base.body_id_a = previous;
                    joint.base.body_id_b = body;
                    joint.base.local_frame_a = frame_a;
                    joint.base.local_frame_b = frame_b;
                    joint.linear_hertz = 5.0;
                    joint.angular_hertz = 5.0;
                    let _ = world.create_weld_joint(&joint);
                }
                _ => {
                    let mut joint = DistanceJointDef::default();
                    joint.base.body_id_a = previous;
                    joint.base.body_id_b = body;
                    joint.base.local_frame_a = frame_a;
                    joint.base.local_frame_b = frame_b;
                    joint.length = 0.1;
                    let _ = world.create_distance_joint(&joint);
                }
            }
            previous = body;
        }
    }
}

fn build_ragdolls(world: &mut World) {
    let _ = ground(world);
    for i in 0..16 {
        let mut human = Human::default();
        create_human(
            &mut human,
            world,
            Pos::new(
                ((i % 4) as f32 * 1.5) as f64,
                (2.0 + (i / 4) as f32 * 2.0) as f64,
                0.0,
            ),
            0.05,
            5.0,
            0.5,
            i + 1,
            None,
            false,
        );
    }
}

/// Enough awake bodies to finalize in parallel: a grid of boxes that settles and falls asleep, plus
/// fast bodies and bullets fired into static walls and the boxes, so the continuous pass, the bullet
/// list, island sleep, and proxy enlargement all run.
fn build_fast_and_sleepy(world: &mut World) {
    let _ = ground(world);
    let wall = make_box_hull(0.25, 4.0, 20.0);
    for x in [-16.0f32, 16.0] {
        let mut def = BodyDef::default();
        def.position = Pos::new(x as f64, 4.0, 0.0);
        let body = world.create_body(&def);
        let _ = world.create_hull_shape(body, &ShapeDef::default(), &wall);
    }
    let cube = make_cube_hull(0.4);
    for i in 0..640 {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(
            ((i % 16) as f32 - 7.5) as f64,
            (0.4 + (i / 256) as f32 * 0.8) as f64,
            (((i / 16) % 16) as f32 - 7.5) as f64,
        );
        let body = world.create_body(&def);
        let _ = world.create_hull_shape(body, &ShapeDef::default(), &cube);
    }
    for i in 0..24 {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.is_bullet = i % 3 == 0;
        def.position = Pos::new(
            (if i % 2 == 0 { -12.0 } else { 12.0 }) as f64,
            (1.0 + (i / 2) as f32 * 0.5) as f64,
            ((i % 5) as f32 - 2.0) as f64,
        );
        def.linear_velocity = Vec3::new(
            if i % 2 == 0 { 300.0 } else { -300.0 },
            0.0,
            (i % 3) as f32 - 1.0,
        );
        let body = world.create_body(&def);
        let _ = world.create_sphere_shape(
            body,
            &ShapeDef::default(),
            Sphere::new(Vec3::ZERO, 0.1),
        );
    }
}

/// Beads on tilted loops with every rotation mode and motor kind; neighbors collide.
fn build_paths(world: &mut World) {
    let anchor = ground(world);
    let modes: [PathRotationConstraint; 6] = [
        PathRotationConstraint::Free,
        PathRotationConstraint::ConstrainAroundTangent,
        PathRotationConstraint::ConstrainAroundNormal,
        PathRotationConstraint::ConstrainAroundBinormal,
        PathRotationConstraint::ConstrainToPath,
        PathRotationConstraint::FullyConstrained,
    ];
    for loop_index in 0..24 {
        let center = Vec3::new(
            (loop_index % 6) as f32 * 7.0 - 17.0,
            4.0,
            (loop_index / 6) as f32 * 7.0 - 10.0,
        );
        let positions: Vec<Vec3> = (0..10)
            .map(|i| {
                let cs = compute_cos_sin(2.0 * std::f32::consts::PI * i as f32 / 10.0);
                Vec3::new(2.5 * cs.cosine, 0.8 * cs.sine, 2.5 * cs.sine)
            })
            .collect();
        let path = std::sync::Arc::new(
            ConstraintPath::through(&positions, Vec3::AXIS_Y, true).expect("path"),
        );
        for bead in 0..8 {
            let at = path.point_at(bead as f32 * 1.25).0;
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::from_vec3(center + at);
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(
                body,
                &ShapeDef::default(),
                &make_cube_hull(0.3),
            );
            let mut joint = PathJointDef::default();
            joint.base.body_id_a = anchor;
            joint.base.body_id_b = body;
            joint.base.local_frame_a = Transform::new(center, Quat::IDENTITY);
            joint.base.collide_connected = true;
            joint.path = Some(std::sync::Arc::clone(&path));
            joint.path_fraction = bead as f32 * 1.25;
            joint.rotation_constraint = modes[((loop_index + bead) % modes.len() as i32) as usize];
            match bead % 3 {
                0 => joint.max_friction_force = 2.0,
                1 => {
                    joint.motor_mode = PathMotorMode::Velocity;
                    joint.target_velocity = (loop_index % 4) as f32 - 1.5;
                    joint.max_motor_force = 30.0;
                }
                _ => {
                    joint.motor_mode = PathMotorMode::Position;
                    joint.target_fraction = (loop_index % 10) as f32;
                    joint.max_motor_force = 20.0;
                }
            }
            let _ = world.create_path_joint(&joint);
        }
    }
}

// "bit identical for any worker count"
#[test]
fn identical() {
    for scene in ParallelScene::ALL {
        let mut serial = scene.make(1);
        let mut worlds = [scene.make(4), scene.make(8)];
        for step in 0..90 {
            serial.step(1.0 / 60.0, 4);
            let expected = serial.world_state_hash();
            for world in worlds.iter_mut() {
                world.step(1.0 / 60.0, 4);
                assert_eq!(
                    world.world_state_hash(),
                    expected,
                    "{} step {} workers {}",
                    scene.name(),
                    step,
                    world.worker_count
                );
            }
        }
        assert!(serial.awake_body_count() > 0);
    }
}

// "changing the worker count between steps keeps the result"
#[test]
fn change_workers() {
    let mut a = ParallelScene::Pyramid.make(1);
    let mut b = ParallelScene::Pyramid.make(1);
    let counts = [1, 3, 8, 2];
    for step in 0..60 {
        b.worker_count = counts[step % 4];
        a.step(1.0 / 60.0, 4);
        b.step(1.0 / 60.0, 4);
        assert_eq!(a.world_state_hash(), b.world_state_hash(), "step {}", step);
    }
}

// "vehicles, soft bodies, and joints with worker threads"
#[test]
fn systems() {
    fn run(workers: i32) -> u64 {
        let mut world = World::default();
        world.worker_count = workers;
        let _ = ground(&mut world);
        for k in 0..6 {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new((k as f32 * 4.0) as f64, 1.2, 0.0);
            let body = world.create_body(&def);
            let mut shape_def = ShapeDef::default();
            shape_def.density = 400.0;
            let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.9, 0.25, 2.0));
            let mut vehicle = VehicleDef::default();
            vehicle.body_id = body;
            for p in [
                Vec3::new(0.9, -0.1, 1.6),
                Vec3::new(-0.9, -0.1, 1.6),
                Vec3::new(0.9, -0.1, -1.6),
                Vec3::new(-0.9, -0.1, -1.6),
            ] {
                let mut wheel = WheelSettings::default();
                wheel.position = p;
                vehicle.wheels.push(wheel);
            }
            let mut controller = WheeledVehicleControllerSettings::default();
            controller.differentials = vec![VehicleDifferentialSettings {
                left_wheel: 0,
                right_wheel: 1,
                ..VehicleDifferentialSettings::default()
            }];
            vehicle.controller = VehicleControllerSettings::Wheeled(controller);
            let id = world.create_vehicle(&vehicle);
            world.set_vehicle_driver_input(
                id,
                1.0,
                if k % 2 == 0 { 0.3 } else { -0.3 },
                0.0,
                0.0,
            );
        }
        let cloth = SoftBodySettings::make_cloth(
            12,
            12,
            0.1,
            SoftBodyVertexAttributes::default(),
            SoftBodyBendType::Distance,
            |_x, _z| false,
        );
        let _ = world.create_soft_body(&SoftBodyDef::with_settings(
            cloth,
            Pos::new(0.0, 3.0, 4.0),
            Quat::IDENTITY,
        ));
        build_pyramid(&mut world);
        for _ in 0..60 {
            world.step(1.0 / 60.0, 4);
        }
        world.world_state_hash()
    }
    assert_eq!(run(1), run(6));
}
