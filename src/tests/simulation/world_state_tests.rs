//! Ported from Tests/NatselTests/Simulation/WorldStateTests.swift.
//!
//! A scene with every system: a sleeping pyramid, ragdolls, joints of every kind, a sensor,
//! vehicles, cloth, and a bullet.

use crate::character::character_virtual::{CharacterDef, CharacterVirtual};
use crate::collision::hull::{make_box_hull, make_cube_hull};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::simulation::World;
use crate::simulation::path_joint::{
    ConstraintPath, PathJointDef, PathMotorMode, PathRotationConstraint,
};
use crate::simulation::six_dof_joint::{SixDOFAxis, SixDOFJointDef};
use crate::simulation::world_state::WorldState;
use crate::soft_body::soft_body_runtime::{SoftBodyDef, SoftBodyID};
use crate::soft_body::soft_body_settings::{
    SoftBodyBendType, SoftBodySettings, SoftBodyVertexAttributes,
};
use crate::tests::simulation::human::{Human, create_human};
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::{BodyID, JointID};
use crate::types::joint_defs::{RevoluteJointDef, WeldJointDef};
use crate::types::shape_def::ShapeDef;
use crate::vehicle::vehicle_types::{
    VehicleControllerSettings, VehicleDef, VehicleDifferentialSettings, VehicleID, WheelSettings,
    WheeledVehicleControllerSettings,
};

/// The Swift scene held a `World` class reference and a `CharacterVirtual` that borrowed it. The
/// Rust character takes `&'w mut World`, so the world lives in a `Box` (stable address) and the
/// character holds the same allocation for `'static`.
///
/// SAFETY: `world` is boxed once in `StateScene::new` and never moved or replaced, so `world_ptr`
/// stays valid for the scene's whole life. No method holds a `&mut World` across a character call:
/// `step` and the character updates run one after the other, exactly as Swift's reference
/// interleaving did. `character` is declared before `world` so it is dropped first.
struct StateScene {
    character: CharacterVirtual<'static>,
    /// Owns the world allocation and keeps its address stable; every access goes through
    /// `world_ptr` so the borrow checker does not tie the character's lifetime to this field.
    #[allow(dead_code)]
    world: Box<World>,
    world_ptr: *mut World,
    vehicles: Vec<VehicleID>,
    soft_body: SoftBodyID,
    boxes: Vec<BodyID>,
    joints: Vec<JointID>,
    anchor: BodyID,
}

impl StateScene {
    fn new(workers: i32) -> Self {
        let mut world = Box::new(World::default());
        world.worker_count = workers;

        let mut ground_def = BodyDef::default();
        ground_def.position = Pos::new(0.0, -1.0, 0.0);
        let anchor = world.create_body(&ground_def);
        let _ = world.create_hull_shape(anchor, &ShapeDef::default(), &make_box_hull(60.0, 1.0, 60.0));

        // Sensor over the pyramid.
        let mut sensor_def = ShapeDef::default();
        sensor_def.is_sensor = true;
        sensor_def.enable_sensor_events = true;
        let _ = world.create_hull_shape(anchor, &sensor_def, &make_box_hull(3.0, 1.0, 1.0));

        let cube = make_cube_hull(0.4);
        let mut shape_def = ShapeDef::default();
        shape_def.enable_sensor_events = true;
        let mut boxes: Vec<BodyID> = Vec::new();
        for row in 0..8i32 {
            for column in 0..(8 - row) {
                let mut def = BodyDef::default();
                def.type_ = BodyType::DynamicBody;
                def.position = Pos::new(
                    column as f64 * 0.85 - 0.425 * (7 - row) as f64,
                    1.4 + row as f64 * 0.8,
                    0.0,
                );
                let body = world.create_body(&def);
                let _ = world.create_hull_shape(body, &shape_def, &cube);
                boxes.push(body);
            }
        }

        for i in 0..3 {
            let mut human = Human::new();
            create_human(
                &mut human,
                &mut world,
                Pos::new(i as f64 * 2.0 - 8.0, 2.0, 6.0),
                0.05,
                5.0,
                0.5,
                i + 1,
                None,
                false,
            );
        }

        // A chain mixing joint kinds, with a path joint and a six DOF joint at the end.
        let mut joints: Vec<JointID> = Vec::new();
        let mut previous = anchor;
        for link in 0..6i32 {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(10.0, 6.0 - link as f64, 0.0);
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(
                body,
                &ShapeDef::default(),
                &make_box_hull(0.3, 0.3, 0.3),
            );
            let pivot = Pos::new(10.0, 6.5 - link as f64, 0.0);
            if link % 2 == 0 {
                let mut joint = RevoluteJointDef::default();
                joint.base.body_id_a = previous;
                joint.base.body_id_b = body;
                joint.base.local_frame_a =
                    Transform::new(world.body_local_point(previous, pivot), Quat::IDENTITY);
                joint.base.local_frame_b =
                    Transform::new(world.body_local_point(body, pivot), Quat::IDENTITY);
                joints.push(world.create_revolute_joint(&joint));
            } else {
                let mut joint = SixDOFJointDef::default();
                joint.base.body_id_a = previous;
                joint.base.body_id_b = body;
                joint.base.local_frame_a =
                    Transform::new(world.body_local_point(previous, pivot), Quat::IDENTITY);
                joint.base.local_frame_b =
                    Transform::new(world.body_local_point(body, pivot), Quat::IDENTITY);
                joint.lock_all(&[SixDOFAxis::RotationX, SixDOFAxis::RotationY]);
                joint[SixDOFAxis::RotationX].max_friction = 2.0;
                joints.push(world.create_six_dof_joint(joint));
            }
            previous = body;
        }

        let mut bead = BodyDef::default();
        bead.type_ = BodyType::DynamicBody;
        bead.position = Pos::new(-10.0, 1.0, -5.0);
        let bead_body = world.create_body(&bead);
        let _ = world.create_sphere_shape(
            bead_body,
            &ShapeDef::default(),
            Sphere::new(Vec3::ZERO, 0.2),
        );
        let mut path = PathJointDef::default();
        path.base.body_id_a = anchor;
        path.base.body_id_b = bead_body;
        path.path = ConstraintPath::through(
            &[
                Vec3::new(-10.0, 2.0, -5.0),
                Vec3::new(-6.0, 3.0, -5.0),
                Vec3::new(-2.0, 2.0, -5.0),
                Vec3::new(2.0, 4.0, -5.0),
            ],
            Vec3::AXIS_Y,
            false,
        )
        .map(std::sync::Arc::new);
        path.motor_mode = PathMotorMode::Velocity;
        path.target_velocity = 2.0;
        path.max_motor_force = 50.0;
        path.rotation_constraint = PathRotationConstraint::ConstrainToPath;
        joints.push(world.create_path_joint(&path));

        let mut vehicles: Vec<VehicleID> = Vec::new();
        for k in 0..2 {
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(k as f64 * 5.0 - 20.0, 1.2, -12.0);
            let body = world.create_body(&def);
            let mut chassis = ShapeDef::default();
            chassis.density = 400.0;
            let _ = world.create_hull_shape(body, &chassis, &make_box_hull(0.9, 0.25, 2.0));
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
            let mut differential = VehicleDifferentialSettings::default();
            differential.left_wheel = 0;
            differential.right_wheel = 1;
            controller.differentials = vec![differential];
            vehicle.controller = VehicleControllerSettings::Wheeled(controller);
            let id = world.create_vehicle(&vehicle);
            world.set_vehicle_driver_input(id, 1.0, if k == 0 { 0.4 } else { -0.2 }, 0.0, 0.0);
            vehicles.push(id);
        }

        let mut soft_body_def = SoftBodyDef::default();
        soft_body_def.settings = SoftBodySettings::make_cloth(
            10,
            10,
            0.15,
            SoftBodyVertexAttributes::default(),
            SoftBodyBendType::Distance,
            |_, _| false,
        );
        soft_body_def.position = Pos::new(0.0, 6.0, 3.0);
        let soft_body = world.create_soft_body(&soft_body_def);

        let mut bullet = BodyDef::default();
        bullet.type_ = BodyType::DynamicBody;
        bullet.is_bullet = true;
        bullet.position = Pos::new(-25.0, 2.0, 0.0);
        bullet.linear_velocity = Vec3::new(250.0, 0.0, 0.0);
        let bullet_body = world.create_body(&bullet);
        let _ = world.create_sphere_shape(
            bullet_body,
            &ShapeDef::default(),
            Sphere::new(Vec3::ZERO, 0.1),
        );

        let mut character_def = CharacterDef::new();
        character_def.position = Pos::new(-4.0, 0.05, 0.0);

        // SAFETY: `world` is a `Box`, so its address is stable for the scene's lifetime and
        // `world_ptr` keeps pointing at the same allocation. The character is the only holder of
        // this borrow while it exists, and every user of the scene reaches the world through
        // `world_mut()` between character calls, never during one.
        let world_ptr: *mut World = &mut *world;
        let character =
            CharacterVirtual::new(unsafe { &mut *world_ptr }, &character_def)
                .expect("character creation failed");

        Self {
            character,
            world,
            world_ptr,
            vehicles,
            soft_body,
            boxes,
            joints,
            anchor,
        }
    }

    #[inline]
    fn world_mut(&mut self) -> &mut World {
        // SAFETY: see `StateScene`; the scene owns the single `Box<World>` behind this pointer.
        unsafe { &mut *self.world_ptr }
    }

    #[inline]
    fn world_ref(&self) -> &World {
        // SAFETY: see `StateScene`; the pointer is never invalidated.
        unsafe { &*self.world_ptr }
    }

    fn step(&mut self) {
        // Walk into the pyramid, pushing boxes.
        self.character.linear_velocity = Vec3::new(
            1.5,
            self.character.linear_velocity.y - 10.0 / 60.0,
            0.0,
        );
        self.character.update(1.0 / 60.0);
        self.world_mut().step(1.0 / 60.0, 4);
    }

    fn save(&self) -> (WorldState, crate::character::character_virtual::State) {
        (self.world_ref().save_state(), self.character.save_state())
    }

    fn restore(&mut self, state: &(WorldState, crate::character::character_virtual::State)) -> bool {
        self.character.restore_state(state.1.clone());
        self.world_mut().restore_state(&state.0)
    }

    /// Everything observable that should repeat after a restore.
    fn fingerprint(&self) -> Vec<u64> {
        let world = self.world_ref();
        let mut values: Vec<u64> =
            vec![world.world_state_hash(), world.awake_body_count() as u64];
        let mut hash: u64 = 14_695_981_039_346_656_037;
        let prime: u64 = 1_099_511_628_211;

        let mix_double = |hash: &mut u64, f: f64| {
            *hash = (*hash ^ f.to_bits()).wrapping_mul(prime);
        };
        let mix_float = |hash: &mut u64, f: f32| {
            *hash = (*hash ^ f.to_bits() as u64).wrapping_mul(prime);
        };

        for p in world.soft_body_vertex_positions(self.soft_body) {
            mix_double(&mut hash, p.x);
            mix_double(&mut hash, p.y);
            mix_double(&mut hash, p.z);
        }
        for vehicle in &self.vehicles {
            let state = world
                .vehicle_drivetrain_state(*vehicle)
                .expect("vehicle drivetrain state missing");
            mix_float(&mut hash, state.engine_rpm);
            mix_float(&mut hash, state.current_gear as f32);
            mix_float(&mut hash, state.clutch_friction);
        }
        for joint in &self.joints {
            if let Some(force) = world.joint_constraint_force(*joint) {
                mix_float(&mut hash, force.x);
                mix_float(&mut hash, force.y);
                mix_float(&mut hash, force.z);
            }
        }
        let p = self.character.position();
        mix_double(&mut hash, p.x);
        mix_double(&mut hash, p.y);
        mix_double(&mut hash, p.z);
        mix_float(&mut hash, self.character.linear_velocity.x);
        values.push(hash);

        let events = world.contact_events();
        values.push((events.begin_events.len() * 1000 + events.end_events.len()) as u64);
        let sensor_events = world.sensor_events();
        values.push((sensor_events.begin.len() * 1000 + sensor_events.end.len()) as u64);
        values
    }
}

// "restoring repeats the following steps bit for bit, any number of times"
#[test]
fn rollback() {
    let mut scene = StateScene::new(1);
    for _ in 0..40 {
        scene.step();
    }
    let state = scene.save();
    assert_eq!(state.0.step_index, scene.world_ref().step_index);
    assert!(state.0.byte_count() > 0);

    let mut expected: Vec<Vec<u64>> = Vec::new();
    for _ in 0..90 {
        scene.step();
        expected.push(scene.fingerprint());
    }
    for round in 0..2 {
        assert!(scene.restore(&state));
        for (i, values) in expected.iter().enumerate() {
            scene.step();
            assert!(
                scene.fingerprint() == *values,
                "round {round} step {i}"
            );
        }
    }
}

// "bodies, shapes, joints, vehicles, and soft bodies created or destroyed after the save are undone"
#[test]
fn structural_changes() {
    let mut scene = StateScene::new(1);
    for _ in 0..30 {
        scene.step();
    }
    let state = scene.save();
    let mut expected: Vec<Vec<u64>> = Vec::new();
    for _ in 0..60 {
        scene.step();
        expected.push(scene.fingerprint());
    }

    // Rewind, then change the world heavily.
    scene.restore(&state);
    let destroyed = scene.boxes[3];
    let joints_1 = scene.joints[1];
    let joints_last = *scene.joints.last().unwrap();
    let vehicles_0 = scene.vehicles[0];
    let boxes_0 = scene.boxes[0];
    let boxes_5 = scene.boxes[5];
    let anchor = scene.anchor;
    let soft_body = scene.soft_body;
    let added;
    {
        let world = scene.world_mut();
        world.destroy_body(destroyed);
        world.destroy_joint(joints_1.index1 - 1, true);
        world.destroy_joint(joints_last.index1 - 1, true);
        world.destroy_vehicle(vehicles_0);
        world.destroy_soft_body(soft_body);
        let head_shape_id = world.bodies[(boxes_5.index1 - 1) as usize].head_shape_id;
        world.destroy_shape_internal(head_shape_id, true);

        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, 8.0, 0.0);
        let added_body = world.create_body(&def);
        added = added_body;
        let _ = world.create_hull_shape(added_body, &ShapeDef::default(), &make_box_hull(1.0, 0.2, 0.7));

        let mut weld = WeldJointDef::default();
        weld.base.body_id_a = added_body;
        weld.base.body_id_b = boxes_0;
        let _ = world.create_weld_joint(&weld);

        let mut six = SixDOFJointDef::default();
        six.base.body_id_a = anchor;
        six.base.body_id_b = added_body;
        let _ = world.create_six_dof_joint(six);
    }
    for _ in 0..25 {
        scene.step();
    }
    assert_eq!(scene.world_ref().body_is_valid(destroyed), false);

    assert!(scene.restore(&state));
    assert!(scene.world_ref().body_is_valid(destroyed));
    assert_eq!(scene.world_ref().body_is_valid(added), false);
    assert!(scene.world_ref().vehicle_is_valid(vehicles_0));
    assert!(scene.world_ref().soft_body_is_valid(soft_body));
    for (i, values) in expected.iter().enumerate() {
        scene.step();
        assert!(scene.fingerprint() == *values, "step {i}");
    }
}

// "worker threads restore the same continuation"
#[test]
fn threads() {
    let mut serial = StateScene::new(1);
    let mut parallel = StateScene::new(4);
    for _ in 0..30 {
        serial.step();
        parallel.step();
    }
    let state = parallel.save();
    for _ in 0..30 {
        serial.step();
        parallel.step();
    }
    let expected = serial.fingerprint();
    assert!(parallel.fingerprint() == expected);
    parallel.restore(&state);
    for _ in 0..30 {
        parallel.step();
    }
    assert!(parallel.fingerprint() == expected);
}

// "a state only restores into its own world"
#[test]
fn ownership() {
    let mut a = World::default();
    let mut b = World::default();
    let state = a.save_state();
    assert_eq!(b.restore_state(&state), false);
    assert!(a.restore_state(&state));
}

// "saving and restoring many times keeps the world consistent"
#[test]
fn cycles() {
    let mut scene = StateScene::new(1);
    for _ in 0..10 {
        scene.step();
    }
    let saved = scene.save();
    scene.step();
    let expected = scene.fingerprint();
    for _ in 0..20 {
        let temporary = scene.save();
        scene.restore(&saved);
        scene.step();
        assert!(scene.fingerprint() == expected);
        scene.restore(&temporary);
    }
}
