//! Scenes for vehicles, cloth, and volumetric soft bodies.
//!
//! Ported from `Sources/Benchmarks/FeatureScenes.swift`.

use std::sync::atomic::{AtomicI32, Ordering};

use natsel::collision::hull::{make_box_hull, make_cube_hull, make_offset_box_hull};
use natsel::collision::mesh::procedural::create_wave_mesh;
use natsel::math::functions::{
    compute_cos_sin, make_quat_from_axis_angle, mul_quat,
};
use natsel::math::types::{Pos, Quat, Transform, Vec3};
use natsel::simulation::path_joint::{
    ConstraintPath, PathJointDef, PathMotorMode, PathRotationConstraint,
};
use natsel::skeleton::ragdoll::{
    ClosedRange, RagdollJointKind, RagdollPart, RagdollSettings, RagdollShape,
};
use natsel::skeleton::skeletal_animation::{Keyframe, SkeletalAnimation, Track};
use natsel::skeleton::skeleton::{Skeleton, SkeletonPose};
use natsel::simulation::world_state::WorldState;
use natsel::simulation::world_runtime::World;
use natsel::soft_body::soft_body_runtime::{SoftBodyDef, SoftBodyID};
use natsel::soft_body::soft_body_settings::{
    SoftBodyBendType, SoftBodyFace, SoftBodyLRAType, SoftBodySettings, SoftBodyVertex,
    SoftBodyVertexAttributes,
};
use natsel::types::body_def::{BodyDef, BodyType};
use natsel::types::geometry::{Capsule, Sphere};
use natsel::types::shape_def::ShapeDef;
use natsel::vehicle::vehicle_types::{
    VehicleAntiRollBar, VehicleCollisionTester, VehicleControllerSettings, VehicleDef,
    VehicleDifferentialSettings, VehicleID, WheelSettings, WheeledVehicleControllerSettings,
};

use crate::scenes::create_large_pyramid;

// MARK: - Shared storage

/// Swift's file-scope `nonisolated(unsafe) var` globals become `UnsafeCell` wrappers here.
/// Edition 2024 forbids taking `&mut` of a `static mut`, and these values are exactly the
/// single-threaded mutable globals Swift declared: the harness runs `create` / `step` / `destroy`
/// for one scenario at a time on the main thread.
struct BenchGlobal<T>(std::cell::UnsafeCell<T>);

// SAFETY: the benchmark harness is single threaded; every access happens on the main thread and
// never overlaps another borrow, which is the contract Swift's `nonisolated(unsafe)` asserted.
unsafe impl<T> Sync for BenchGlobal<T> {}

impl<T> BenchGlobal<T> {
    const fn new(value: T) -> Self {
        Self(std::cell::UnsafeCell::new(value))
    }

    /// SAFETY: the caller must not hold another reference to the same global at the same time.
    #[allow(clippy::mut_from_ref)]
    unsafe fn get_mut(&self) -> &mut T {
        unsafe { &mut *self.0.get() }
    }

    /// SAFETY: the caller must not hold a mutable reference to the same global at the same time.
    unsafe fn get_ref(&self) -> &T {
        unsafe { &*self.0.get() }
    }
}

/// Swift's `nonisolated(unsafe) var benchmarkVehicles: [VehicleID]`.
static BENCHMARK_VEHICLES: BenchGlobal<Vec<VehicleID>> = BenchGlobal::new(Vec::new());

fn vehicles() -> &'static mut Vec<VehicleID> {
    // SAFETY: see `BenchGlobal`; the harness touches one scenario at a time.
    unsafe { BENCHMARK_VEHICLES.get_mut() }
}

fn make_terrain(world: &mut World) {
    let ground = world.create_body(&BodyDef::default());
    if let Some(mesh) = create_wave_mesh(120, 120, 2.0, 0.4, 0.15, 0.15) {
        let _ = world.create_mesh_shape(ground, &ShapeDef::default(), mesh, Vec3::ONE);
    }
}

// MARK: - vehicles

/// 64 four wheel drive cars with anti-roll bars and cylinder wheel casts on wavy terrain.
pub fn create_vehicles(world: &mut World) {
    make_terrain(world);
    vehicles().clear();
    for row in 0..8i32 {
        for column in 0..8i32 {
            let mut body_def = BodyDef::default();
            body_def.type_ = BodyType::DynamicBody;
            body_def.position = Pos::new(
                ((column - 4) as f32 * 12.0) as f64,
                3.0,
                ((row - 4) as f32 * 12.0) as f64,
            );
            let body = world.create_body(&body_def);
            let mut shape_def = ShapeDef::default();
            shape_def.density = 1500.0 / (8.0 * 0.9 * 2.0 * 0.3);
            let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.9, 0.3, 2.0));

            let mut def = VehicleDef::default();
            def.body_id = body;
            def.collision_tester = VehicleCollisionTester::Cylinder(0.5);
            def.max_pitch_roll_angle = 60.0 * 0.0174533;
            let positions = [
                Vec3::new(0.9, -0.1, 1.5),
                Vec3::new(-0.9, -0.1, 1.5),
                Vec3::new(0.9, -0.1, -1.5),
                Vec3::new(-0.9, -0.1, -1.5),
            ];
            for (i, p) in positions.iter().enumerate() {
                let mut wheel = WheelSettings::default();
                wheel.position = *p;
                wheel.radius = 0.35;
                wheel.width = 0.25;
                wheel.suspension_min_length = 0.1;
                wheel.suspension_max_length = 0.4;
                wheel.max_steer_angle = if i < 2 { 0.5 } else { 0.0 };
                def.wheels.push(wheel);
            }
            def.anti_roll_bars = vec![
                VehicleAntiRollBar::with_wheels(0, 1, 1000.0),
                VehicleAntiRollBar::with_wheels(2, 3, 1000.0),
            ];
            let mut controller = WheeledVehicleControllerSettings::default();
            controller.differentials = vec![
                VehicleDifferentialSettings::with_wheels(0, 1, 0.5),
                VehicleDifferentialSettings::with_wheels(2, 3, 0.5),
            ];
            def.controller = VehicleControllerSettings::Wheeled(controller);
            let vehicle = world.create_vehicle(&def);
            vehicles().push(vehicle);
        }
    }
}

pub fn step_vehicles(world: &mut World, step_index: usize) {
    let list = vehicles();
    for (i, vehicle) in list.iter().enumerate() {
        let steer: f32 = if (step_index / 120 + i) % 2 == 0 {
            0.4
        } else {
            -0.4
        };
        world.set_vehicle_driver_input(*vehicle, 0.6, steer, 0.0, 0.0);
    }
}

pub fn destroy_vehicles() {
    vehicles().clear();
}

// MARK: - cloth

/// Eight 40x40 cloths falling over static spheres and a pile of dynamic boxes.
pub fn create_cloth(world: &mut World) {
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(50.0, 1.0, 50.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    let attributes = SoftBodyVertexAttributes::new(
        1.0e-6,
        1.0e-5,
        1.0e-3,
        SoftBodyLRAType::None,
        1.0,
    );
    let bend_type = if std::env::var("NATSEL_BEND").unwrap_or_default() == "distance" {
        SoftBodyBendType::Distance
    } else {
        SoftBodyBendType::Dihedral
    };
    let settings =
        SoftBodySettings::make_cloth(40, 40, 0.05, attributes, bend_type, |_, _| false);

    for k in 0..8i32 {
        let x = (k % 4) as f32 * 3.0 - 4.5;
        let z = (k / 4) as f32 * 3.0 - 1.5;
        let obstacle = world.create_body(&BodyDef::default());
        let _ = world.create_sphere_shape(
            obstacle,
            &ShapeDef::default(),
            Sphere::new(Vec3::new(x, 0.6, z), 0.6),
        );

        let mut body_def = BodyDef::default();
        body_def.type_ = BodyType::DynamicBody;
        body_def.position = Pos::new((x + 0.6) as f64, 1.6, z as f64);
        let box_body = world.create_body(&body_def);
        let _ = world.create_hull_shape(
            box_body,
            &ShapeDef::default(),
            &make_box_hull(0.15, 0.15, 0.15),
        );

        let mut def = SoftBodyDef::default();
        def.settings = settings.clone();
        def.position = Pos::new(x as f64, 2.0, z as f64);
        def.vertex_radius = 0.01;
        def.enable_sleep = false;
        let _ = world.create_soft_body(&def);
    }
}

// MARK: - soft_cubes

/// Swift's `nonisolated(unsafe) private var selfCollidingCloth64: (World, SoftBodyID)?`. The
/// world is owned by the benchmark loop, so only the id is kept here.
static SELF_COLLIDING_CLOTH_64_INDEX: AtomicI32 = AtomicI32::new(0);
static SELF_COLLIDING_CLOTH_64_WORLD: AtomicI32 = AtomicI32::new(0);
static SELF_COLLIDING_CLOTH_64_GENERATION: AtomicI32 = AtomicI32::new(0);

fn self_colliding_cloth_64_id() -> Option<SoftBodyID> {
    let index = SELF_COLLIDING_CLOTH_64_INDEX.load(Ordering::Relaxed);
    if index == 0 {
        return None;
    }
    Some(SoftBodyID::new(
        index,
        SELF_COLLIDING_CLOTH_64_WORLD.load(Ordering::Relaxed) as u16,
        SELF_COLLIDING_CLOTH_64_GENERATION.load(Ordering::Relaxed) as u16,
    ))
}

/// GuavaStudio's 64x64 cloth acceptance scene, with its authored vertex order and self collision.
pub fn create_self_colliding_cloth64(world: &mut World) {
    world.gravity = Vec3::new(0.0, -9.81, 0.0);
    let mut settings = SoftBodySettings::new();
    for z in 0..64i32 {
        for x in 0..64i32 {
            settings.vertices.push(SoftBodyVertex::new(
                Vec3::new(
                    (x as f32 - 31.5) * 0.05,
                    0.0,
                    (z as f32 - 31.5) * 0.05,
                ),
                Vec3::ZERO,
                if z == 0 { 0.0 } else { 20.0 },
            ));
        }
    }
    for z in 0..63i32 {
        for x in 0..63i32 {
            let a = (z * 64 + x) as u32;
            let b = a + 64;
            let c = b + 1;
            let d = a + 1;
            settings.faces.push(SoftBodyFace::new(a, b, c));
            settings.faces.push(SoftBodyFace::new(a, c, d));
        }
    }
    settings.create_constraints(
        &[SoftBodyVertexAttributes::new(
            1.0e-5,
            1.0e-5,
            1.0e-5,
            SoftBodyLRAType::None,
            1.0,
        )],
        SoftBodyBendType::Distance,
        0.13962634,
    );
    settings.calculate_rest_values();
    let mut def = SoftBodyDef::default();
    def.settings = settings;
    def.position = Pos::new(0.0, 8.0, 0.0);
    def.iteration_count = 5;
    def.linear_damping = 0.05;
    def.vertex_radius = 0.02;
    def.max_linear_velocity = 500.0;
    def.enable_sleep = false;
    def.enable_self_collision = true;
    def.faces_double_sided = true;
    let id = world.create_soft_body(&def);
    SELF_COLLIDING_CLOTH_64_INDEX.store(id.index1, Ordering::Relaxed);
    SELF_COLLIDING_CLOTH_64_WORLD.store(id.world0 as i32, Ordering::Relaxed);
    SELF_COLLIDING_CLOTH_64_GENERATION.store(id.generation as i32, Ordering::Relaxed);
}

/// `record_hash` is Swift's file-scope `recordHash` (the `NATSEL_HASH=1` diagnostic).
pub fn destroy_self_colliding_cloth64(world: &World, record_hash: bool) {
    if let Some(id) = self_colliding_cloth_64_id() {
        if record_hash {
            let mut hash: u64 = 0xcbf29ce484222325;
            for p in world.soft_body_vertex_positions(id) {
                for bits in [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()] {
                    hash = (hash ^ bits).wrapping_mul(0x100000001b3);
                }
            }
            println!("cloth_positions_hash: {:x}", hash);
        }
    }
    SELF_COLLIDING_CLOTH_64_INDEX.store(0, Ordering::Relaxed);
}

/// 27 volumetric cubes dropped in a pile.
pub fn create_soft_cubes(world: &mut World) {
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(50.0, 1.0, 50.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    let settings = SoftBodySettings::make_cube(6, 0.1, 1.0e-5, 0.0);
    for i in 0..27i32 {
        let p = Pos::new(
            ((i % 3) as f32 * 0.7 - 0.7) as f64,
            (1.0 + (i / 9) as f32 * 0.7) as f64,
            (((i / 3) % 3) as f32 * 0.7 - 0.7) as f64,
        );
        let mut def = SoftBodyDef::default();
        def.settings = settings.clone();
        def.position = p;
        def.vertex_radius = 0.01;
        def.enable_sleep = false;
        let _ = world.create_soft_body(&def);
    }
}

// MARK: - cloth_stack

/// Six 30x30 cloths dropped onto each other over a sphere: soft body contact and self collision.
pub fn create_cloth_stack(world: &mut World) {
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(50.0, 1.0, 50.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    let _ = world.create_sphere_shape(
        ground,
        &ShapeDef::default(),
        Sphere::new(Vec3::new(0.0, 0.5, 0.0), 0.5),
    );
    let attributes = SoftBodyVertexAttributes::new(
        1.0e-6,
        1.0e-5,
        1.0e-3,
        SoftBodyLRAType::None,
        1.0,
    );
    let settings = SoftBodySettings::make_cloth(
        30,
        30,
        0.05,
        attributes,
        SoftBodyBendType::Distance,
        |_, _| false,
    );
    for k in 0..6i32 {
        let mut def = SoftBodyDef::default();
        def.settings = settings.clone();
        def.position = Pos::new(
            (0.05 * k as f32) as f64,
            (1.4 + 0.25 * k as f32) as f64,
            0.0,
        );
        def.rotation = make_quat_from_axis_angle(Vec3::AXIS_Y, 0.3 * k as f32);
        def.enable_self_collision = true;
        def.enable_sleep = false;
        def.friction = 0.4;
        let _ = world.create_soft_body(&def);
    }
}

// MARK: - path_beads

/// 64 tilted loops with 16 beads each: path joints with motors, friction, and rotation
/// constraints.
pub fn create_path_beads(world: &mut World) {
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(80.0, 1.0, 80.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    let modes = [
        PathRotationConstraint::Free,
        PathRotationConstraint::ConstrainAroundTangent,
        PathRotationConstraint::ConstrainToPath,
        PathRotationConstraint::FullyConstrained,
    ];
    for loop_index in 0..64i32 {
        let center = Vec3::new(
            (loop_index % 8) as f32 * 7.0 - 24.0,
            4.0,
            (loop_index / 8) as f32 * 7.0 - 24.0,
        );
        let mut positions: Vec<Vec3> = Vec::with_capacity(12);
        for i in 0..12i32 {
            let cs = compute_cos_sin(2.0 * std::f32::consts::PI * i as f32 / 12.0);
            positions.push(Vec3::new(2.5 * cs.cosine, 0.8 * cs.sine, 2.5 * cs.sine));
        }
        let Some(path) = ConstraintPath::through(&positions, Vec3::AXIS_Y, true) else {
            continue;
        };
        let path = std::sync::Arc::new(path);
        for bead in 0..16i32 {
            let fraction = bead as f32 * 0.75;
            let mut def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            // Swift read `path.point(at:).position`; the Rust accessor returns the whole frame.
            let (point_position, _tangent, _normal, _binormal) = path.point_at(fraction);
            def.position = Pos::new(
                (center.x + point_position.x) as f64,
                (center.y + point_position.y) as f64,
                (center.z + point_position.z) as f64,
            );
            let body = world.create_body(&def);
            let _ = world.create_hull_shape(body, &ShapeDef::default(), &make_cube_hull(0.2));
            let mut joint = PathJointDef::default();
            joint.base.body_id_a = ground;
            joint.base.body_id_b = body;
            joint.base.local_frame_a = Transform::new(center, Quat::IDENTITY);
            joint.base.collide_connected = true;
            joint.path = Some(path.clone());
            joint.path_fraction = fraction;
            joint.rotation_constraint = modes[(bead % modes.len() as i32) as usize];
            if bead % 2 == 0 {
                joint.motor_mode = PathMotorMode::Velocity;
                joint.target_velocity = 2.0;
                joint.max_motor_force = 40.0;
            } else {
                joint.max_friction_force = 3.0;
            }
            let _ = world.create_path_joint(&joint);
        }
    }
}

// MARK: - ragdoll_motors

/// Swift's `nonisolated(unsafe) var benchmarkWalk`.
static BENCHMARK_WALK: BenchGlobal<Option<SkeletalAnimation>> = BenchGlobal::new(None);

/// Swift's `nonisolated(unsafe) var benchmarkRagdolls: [Ragdoll]`. The entries are raw pointers
/// because `Ragdoll<'w>` borrows the world; see the note in `create_ragdoll_motors`.
static BENCHMARK_RAGDOLLS: BenchGlobal<Vec<*mut natsel::skeleton::ragdoll::Ragdoll<'static>>> =
    BenchGlobal::new(Vec::new());

fn ragdoll_skeleton() -> Skeleton {
    let mut skeleton = Skeleton::new();
    skeleton.add_joint("pelvis", -1);
    skeleton.add_joint_with_parent_name("spine", "pelvis");
    skeleton.add_joint_with_parent_name("head", "spine");
    for side in ["L", "R"] {
        skeleton.add_joint_with_parent_name(&format!("upperArm{}", side), "spine");
        skeleton.add_joint_with_parent_name(
            &format!("lowerArm{}", side),
            &format!("upperArm{}", side),
        );
        skeleton.add_joint_with_parent_name(&format!("thigh{}", side), "pelvis");
        skeleton.add_joint_with_parent_name(
            &format!("calf{}", side),
            &format!("thigh{}", side),
        );
    }
    skeleton
}

/// 100 skeleton ragdolls that follow a looping walk cycle with joint motors.
pub fn create_ragdoll_motors(world: &mut World) {
    let ground = world.create_body(&BodyDef::default());
    let _ = world.create_hull_shape(
        ground,
        &ShapeDef::default(),
        &make_offset_box_hull(80.0, 1.0, 80.0, Vec3::new(0.0, -1.0, 0.0)),
    );
    let skeleton = ragdoll_skeleton();
    let down = make_quat_from_axis_angle(Vec3::AXIS_Z, -0.5 * std::f32::consts::PI);
    let up = make_quat_from_axis_angle(Vec3::AXIS_Z, 0.5 * std::f32::consts::PI);
    let mut bind = SkeletonPose::new(skeleton.clone());
    let set = |bind: &mut SkeletonPose, name: &str, p: Vec3, q: Quat| {
        let index = skeleton.joint_index(name).expect("joint present") as usize;
        bind.joint_matrices[index] = Transform::new(p, q);
    };
    set(&mut bind, "pelvis", Vec3::new(0.0, 1.0, 0.0), up);
    set(&mut bind, "spine", Vec3::new(0.0, 1.25, 0.0), up);
    set(&mut bind, "head", Vec3::new(0.0, 1.65, 0.0), up);
    for (side, x) in [("L", -1.0f32), ("R", 1.0f32)] {
        set(
            &mut bind,
            &format!("upperArm{}", side),
            Vec3::new(0.25 * x, 1.55, 0.0),
            down,
        );
        set(
            &mut bind,
            &format!("lowerArm{}", side),
            Vec3::new(0.25 * x, 1.25, 0.0),
            down,
        );
        set(
            &mut bind,
            &format!("thigh{}", side),
            Vec3::new(0.12 * x, 0.95, 0.0),
            down,
        );
        set(
            &mut bind,
            &format!("calf{}", side),
            Vec3::new(0.12 * x, 0.5, 0.0),
            down,
        );
    }
    bind.calculate_joint_states();

    let capsule = |length: f32, radius: f32| -> RagdollShape {
        RagdollShape::Capsule(Capsule::new(
            Vec3::new(radius, 0.0, 0.0),
            Vec3::new(length - radius, 0.0, 0.0),
            radius,
        ))
    };
    let hinge = Transform::new(
        Vec3::ZERO,
        make_quat_from_axis_angle(Vec3::AXIS_Z, 0.5 * std::f32::consts::PI),
    );
    let mut parts: Vec<RagdollPart> = Vec::new();
    for joint in skeleton.joints() {
        let mut part = RagdollPart::new();
        part.joint_friction = 0.5;
        let name = joint.name.clone();
        if name == "pelvis" {
            part.shapes = vec![RagdollShape::Box {
                half_extents: Vec3::new(0.12, 0.15, 0.1),
                transform: Transform::new(Vec3::new(0.1, 0.0, 0.0), Quat::IDENTITY),
            }];
        } else if name == "spine" {
            part.shapes = vec![capsule(0.38, 0.13)];
            part.joint = RagdollJointKind::SwingTwist {
                cone_angle: 0.4,
                twist_min: -0.3,
                twist_max: 0.3,
            };
        } else if name == "head" {
            part.shapes = vec![RagdollShape::Sphere(Sphere::new(
                Vec3::new(0.12, 0.0, 0.0),
                0.11,
            ))];
        } else if name.starts_with("upperArm") {
            part.shapes = vec![capsule(0.3, 0.05)];
            part.joint = RagdollJointKind::SwingTwist {
                cone_angle: 1.2,
                twist_min: -0.5,
                twist_max: 0.5,
            };
        } else if name.starts_with("lowerArm") {
            part.shapes = vec![capsule(0.3, 0.045)];
            part.joint = RagdollJointKind::Hinge {
                lower: -2.0,
                upper: 0.0,
            };
            part.joint_frame = hinge;
        } else if name.starts_with("thigh") {
            part.shapes = vec![capsule(0.45, 0.07)];
            part.joint = RagdollJointKind::Pyramid {
                twist: ClosedRange::new(-0.3, 0.3),
                swing_y: ClosedRange::new(-0.4, 0.4),
                swing_z: ClosedRange::new(-1.2, 0.4),
            };
        } else {
            part.shapes = vec![capsule(0.45, 0.055)];
            part.joint = RagdollJointKind::Hinge {
                lower: 0.0,
                upper: 2.0,
            };
            part.joint_frame = hinge;
        }
        parts.push(part);
    }
    let settings = RagdollSettings::new(skeleton.clone(), parts);

    // Walk cycle: thighs and knees swing out of phase.
    let swing = |angle: f32, axis: Vec3, base: Quat| -> Quat {
        mul_quat(base, make_quat_from_axis_angle(axis, angle))
    };
    let mut tracks: Vec<Track> = Vec::new();
    for (side, phase) in [("L", 0.0f32), ("R", std::f32::consts::PI)] {
        let thigh_index = skeleton
            .joint_index(&format!("thigh{}", side))
            .expect("thigh present") as usize;
        let calf_index = skeleton
            .joint_index(&format!("calf{}", side))
            .expect("calf present") as usize;
        let thigh = bind.joint_states[thigh_index];
        let calf = bind.joint_states[calf_index];
        let mut thigh_keys: Vec<Keyframe> = Vec::new();
        let mut calf_keys: Vec<Keyframe> = Vec::new();
        for k in 0..=8i32 {
            let t = k as f32 / 8.0;
            let s = compute_cos_sin(2.0 * std::f32::consts::PI * t + phase).sine;
            thigh_keys.push(Keyframe::new(
                t,
                thigh.translation,
                swing(0.5 * s, Vec3::AXIS_Z, thigh.rotation),
            ));
            calf_keys.push(Keyframe::new(
                t,
                calf.translation,
                swing(0.6 + 0.5 * s, Vec3::AXIS_Y, calf.rotation),
            ));
        }
        tracks.push(Track::new(format!("thigh{}", side), thigh_keys));
        tracks.push(Track::new(format!("calf{}", side), calf_keys));
    }
    // SAFETY: single threaded harness; written once per scene creation.
    unsafe {
        *BENCHMARK_WALK.get_mut() = Some(SkeletalAnimation::new(tracks, true));
    }

    // SAFETY: single threaded harness; the previous scene's ragdolls were already released.
    unsafe {
        let stale: Vec<_> = BENCHMARK_RAGDOLLS.get_mut().drain(..).collect();
        for pointer in stale {
            drop(Box::from_raw(pointer));
        }
    }
    // Swift kept 100 independent `Ragdoll` class references, each holding a shared `world`
    // reference, all alive at once and all driven every step. `Ragdoll<'w>` holds `&'w mut World`,
    // so 100 simultaneous live ragdolls over one world cannot be expressed safely. This is the
    // same adaptation `src/tests/ragdoll_tests.rs` makes (`Ragdoll::new(unsafe { &mut *world_ptr },
    // ...)`): the harness is strictly single threaded and never touches a second ragdoll while one
    // is borrowed, so the aliasing the engine already relies on for `world` is preserved.
    let world_ptr: *mut World = world;
    for i in 0..100i32 {
        let mut pose = bind.clone();
        pose.root_offset = Pos::new(
            ((i % 10) as f32 * 2.0 - 9.0) as f64,
            0.2,
            ((i / 10) as f32 * 2.0 - 9.0) as f64,
        );
        // SAFETY: see the note above. Each ragdoll re-borrows the same `World`; no two are ever
        // used at the same instant, and the borrow never outlives the scene.
        let ragdoll = natsel::skeleton::ragdoll::Ragdoll::new(
            unsafe { &mut *world_ptr },
            settings.clone(),
            &pose,
            None,
        );
        if let Some(ragdoll) = ragdoll {
            // SAFETY: `Box::into_raw` hands ownership to the table, which releases it in
            // `destroy_ragdoll_motors`. The lifetime is widened to `'static` because the table
            // outlives this call while the `World` itself is owned by the caller loop.
            let leaked: *mut natsel::skeleton::ragdoll::Ragdoll<'static> =
                Box::into_raw(Box::new(unsafe {
                    std::mem::transmute::<
                        natsel::skeleton::ragdoll::Ragdoll<'_>,
                        natsel::skeleton::ragdoll::Ragdoll<'static>,
                    >(ragdoll)
                }));
            // SAFETY: single threaded harness; the table owns `leaked` from here.
            unsafe {
                BENCHMARK_RAGDOLLS.get_mut().push(leaked);
            }
        }
    }
}

pub fn step_ragdoll_motors(_world: &mut World, step_index: usize) {
    // SAFETY: single threaded harness; the walk cycle and table are stable for the whole scene.
    let Some(walk) = (unsafe { BENCHMARK_WALK.get_ref().as_ref() }) else {
        return;
    };
    let Some(first) = (unsafe { BENCHMARK_RAGDOLLS.get_ref().first().copied() }) else {
        return;
    };
    // SAFETY: `first` came from `create_ragdoll_motors` and is live until the table is cleared.
    let first_ragdoll = unsafe { &*first };
    let mut pose = SkeletonPose::new(first_ragdoll.settings.skeleton.clone());
    let current = first_ragdoll.pose();
    pose.joint_states = current.joint_states;
    walk.sample(step_index as f32 / 60.0, &mut pose, 1.0);
    pose.calculate_joint_matrices();
    // SAFETY: every pointer in the table was produced by `create_ragdoll_motors`; they alias the
    // same `World` deliberately (see the note there) and are driven one at a time.
    for &ragdoll_ptr in unsafe { BENCHMARK_RAGDOLLS.get_ref() } {
        let ragdoll = unsafe { &mut *ragdoll_ptr };
        ragdoll.drive_to_pose_using_motors(&pose, 150.0, 6.0, 1.0);
    }
}

pub fn destroy_ragdoll_motors() {
    // Swift cleared the array without destroying the bodies; the world is discarded right after.
    // SAFETY: every pointer came from `Box::into_raw` in `create_ragdoll_motors` and is released
    // exactly once here.
    unsafe {
        for ragdoll_ptr in BENCHMARK_RAGDOLLS.get_mut().drain(..) {
            drop(Box::from_raw(ragdoll_ptr));
        }
        *BENCHMARK_WALK.get_mut() = None;
    }
}

// MARK: - rollback

/// Swift's `nonisolated(unsafe) var rollbackStates: [WorldState]`.
static ROLLBACK_STATES: BenchGlobal<Vec<WorldState>> = BenchGlobal::new(Vec::new());

fn rollback_states() -> &'static mut Vec<WorldState> {
    // SAFETY: single threaded harness, same contract as Swift's file-scope variable.
    unsafe { ROLLBACK_STATES.get_mut() }
}

/// A large pyramid saved every step; every 30 steps the last 8 are rolled back and replayed,
/// as a rollback network game does.
pub fn create_rollback(world: &mut World) {
    create_large_pyramid(world);
    rollback_states().clear();
}

pub fn step_rollback(world: &mut World, step_index: usize) {
    let states = rollback_states();
    states.push(world.save_state());
    if states.len() > 8 {
        states.remove(0);
    }
    if step_index % 30 == 29 {
        if let Some(oldest) = states.first() {
            world.restore_state(oldest);
            for _ in 0..7 {
                world.step(1.0 / 60.0, 4);
            }
            states.clear();
        }
    }
}

pub fn destroy_rollback() {
    rollback_states().clear();
}
