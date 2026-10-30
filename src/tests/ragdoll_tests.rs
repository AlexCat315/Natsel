//! Ported from Tests/NatselTests/RagdollTests.swift.
//!
//! Skeleton structure, pose-space conversion, animation sampling, skeleton mapping, and the
//! physical ragdoll: falling and settling, motor driving, kinematic driving, and mass
//! stabilization.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::functions::{
    atan2_approx, get_quat_angle, inv_mul_quat, inv_mul_transforms, make_quat_from_axis_angle,
    mul_quat, negate_quat, rotate_vector, transform_world_point,
};
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::skeleton::ragdoll::{
    ClosedRange, Ragdoll, RagdollJointKind, RagdollPart, RagdollSelfCollision, RagdollSettings,
    RagdollShape,
};
use crate::skeleton::skeletal_animation::{Keyframe, SkeletalAnimation, Track};
use crate::skeleton::skeleton::{Skeleton, SkeletonJointState, SkeletonPose};
use crate::skeleton::skeleton_mapper::SkeletonMapper;
use crate::simulation::six_dof_joint::{SixDOFAxis, SixDOFMotorMode};
use crate::simulation::World;
use crate::tests::world_position_assertions::{abs_float, distance};
use crate::tests::SWIFT_FLOAT_PI;
use crate::types::body_def::BodyDef;
use crate::types::geometry::{Capsule, Sphere};
use crate::types::shape_def::ShapeDef;

/// A small humanoid: pelvis, spine, head, two arms and two legs of two parts each.
fn human_skeleton() -> Skeleton {
    let mut skeleton = Skeleton::new();
    skeleton.add_joint("pelvis", -1);
    skeleton.add_joint_with_parent_name("spine", "pelvis");
    skeleton.add_joint_with_parent_name("head", "spine");
    for side in ["L", "R"] {
        skeleton.add_joint_with_parent_name(&format!("upperArm{side}"), "spine");
        skeleton.add_joint_with_parent_name(&format!("lowerArm{side}"), &format!("upperArm{side}"));
        skeleton.add_joint_with_parent_name(&format!("thigh{side}"), "pelvis");
        skeleton.add_joint_with_parent_name(&format!("calf{side}"), &format!("thigh{side}"));
    }
    skeleton
}

/// Standing bind pose, feet at y = 0 when the root offset is zero.
fn human_bind_pose(skeleton: &Skeleton, origin: Vec3) -> SkeletonPose {
    let mut pose = SkeletonPose::new(skeleton.clone());
    pose.root_offset = Pos::from_vec3(origin);
    let down = make_quat_from_axis_angle(Vec3::AXIS_Z, -0.5 * SWIFT_FLOAT_PI);
    let up = make_quat_from_axis_angle(Vec3::AXIS_Z, 0.5 * SWIFT_FLOAT_PI);
    let set = |pose: &mut SkeletonPose, name: &str, p: Vec3, q: Quat| {
        pose.joint_matrices[skeleton.joint_index(name).unwrap() as usize] = Transform::new(p, q);
    };
    set(&mut pose, "pelvis", Vec3::new(0.0, 1.0, 0.0), up);
    set(&mut pose, "spine", Vec3::new(0.0, 1.25, 0.0), up);
    set(&mut pose, "head", Vec3::new(0.0, 1.65, 0.0), up);
    for (side, x) in [("L", -1.0f32), ("R", 1.0f32)] {
        set(&mut pose, &format!("upperArm{side}"), Vec3::new(0.25 * x, 1.55, 0.0), down);
        set(&mut pose, &format!("lowerArm{side}"), Vec3::new(0.25 * x, 1.25, 0.0), down);
        set(&mut pose, &format!("thigh{side}"), Vec3::new(0.12 * x, 0.95, 0.0), down);
        set(&mut pose, &format!("calf{side}"), Vec3::new(0.12 * x, 0.5, 0.0), down);
    }
    pose.calculate_joint_states();
    pose
}

/// Parts along each joint's x axis (which points along the bone).
fn human_settings(skeleton: &Skeleton) -> RagdollSettings {
    fn capsule(length: f32, radius: f32) -> RagdollShape {
        RagdollShape::Capsule(Capsule::new(
            Vec3::new(radius, 0.0, 0.0),
            Vec3::new(length - radius, 0.0, 0.0),
            radius,
        ))
    }

    let mut parts: Vec<RagdollPart> = Vec::new();
    for joint in skeleton.joints() {
        let mut part = RagdollPart::new();
        part.joint_friction = 0.5;
        let name = joint.name.as_str();
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
            part.joint = RagdollJointKind::SwingTwist {
                cone_angle: 0.6,
                twist_min: -0.8,
                twist_max: 0.8,
            };
        } else if name.starts_with("upperArm") {
            part.shapes = vec![capsule(0.3, 0.05)];
            part.joint = RagdollJointKind::SwingTwist {
                cone_angle: 1.2,
                twist_min: -0.5,
                twist_max: 0.5,
            };
        } else if name.starts_with("lowerArm") {
            part.shapes = vec![capsule(0.3, 0.045)];
            // Hinge about the joint's y axis, which is world x in the bind pose: the elbow bends
            // forward.
            part.joint = RagdollJointKind::Hinge {
                lower: -2.0,
                upper: 0.0,
            };
            part.joint_frame = Transform::new(
                Vec3::ZERO,
                make_quat_from_axis_angle(Vec3::AXIS_Z, 0.5 * SWIFT_FLOAT_PI),
            );
        } else if name.starts_with("thigh") {
            part.shapes = vec![capsule(0.45, 0.07)];
            part.joint = RagdollJointKind::Pyramid {
                twist: ClosedRange::new(-0.3, 0.3),
                swing_y: ClosedRange::new(-0.4, 0.4),
                swing_z: ClosedRange::new(-1.2, 0.4),
            };
        } else {
            part.shapes = vec![capsule(0.45, 0.055)];
            // The knee bends backward.
            part.joint = RagdollJointKind::Hinge {
                lower: 0.0,
                upper: 2.0,
            };
            part.joint_frame = Transform::new(
                Vec3::ZERO,
                make_quat_from_axis_angle(Vec3::AXIS_Z, 0.5 * SWIFT_FLOAT_PI),
            );
        }
        parts.push(part);
    }
    RagdollSettings::new(skeleton.clone(), parts)
}

fn ground_world(gravity: Vec3) -> World {
    let mut world = World::default();
    world.gravity = gravity;
    let mut def = BodyDef::default();
    def.position = Pos::new(0.0, -1.0, 0.0);
    let body = world.create_body(&def);
    let shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(20.0, 1.0, 20.0));
    world
}

fn rotation_error(a: Quat, b: Quat) -> f32 {
    let relative = inv_mul_quat(a, b);
    get_quat_angle(if relative.s < 0.0 {
        negate_quat(relative)
    } else {
        relative
    })
}

#[test]
fn skeleton_structure_and_pose_spaces() {
    // "skeleton structure and pose spaces"
    let skeleton = human_skeleton();
    assert_eq!(skeleton.joint_count(), 11);
    let calf = skeleton.joint_index("calfL").unwrap();
    assert!(skeleton.is_ancestor(skeleton.joint_index("pelvis").unwrap(), calf));
    assert!(!skeleton.is_ancestor(skeleton.joint_index("spine").unwrap(), calf));

    let bind = human_bind_pose(&skeleton, Vec3::ZERO);
    let mut copy = SkeletonPose::new(skeleton.clone());
    copy.joint_states = bind.joint_states.clone();
    copy.calculate_joint_matrices();
    for i in 0..skeleton.joint_count() {
        assert!(distance(copy.joint_matrices[i].p, bind.joint_matrices[i].p) < 1.0e-5);
        assert!(rotation_error(copy.joint_matrices[i].q, bind.joint_matrices[i].q) < 1.0e-3);
    }
}

#[test]
fn animation_sampling_looping_and_blending() {
    // "animation sampling, looping, and blending"
    let mut skeleton = Skeleton::new();
    skeleton.add_joint("root", -1);
    skeleton.add_joint("arm", 0);
    let turn = make_quat_from_axis_angle(Vec3::AXIS_Z, 1.0);
    let animation = SkeletalAnimation::new(
        vec![
            Track::new(
                "arm".to_owned(),
                vec![
                    Keyframe::new(1.0, Vec3::new(2.0, 0.0, 0.0), turn),
                    Keyframe::new(0.0, Vec3::ZERO, Quat::IDENTITY),
                ],
            ),
            Track::new(
                "missing".to_owned(),
                vec![Keyframe::new(0.0, Vec3::ZERO, Quat::IDENTITY)],
            ),
        ],
        false,
    );
    assert_eq!(animation.duration(), 1.0);
    let mut pose = SkeletonPose::new(skeleton.clone());
    animation.sample(0.5, &mut pose, 1.0);
    assert!(distance(pose.joint_states[1].translation, Vec3::new(1.0, 0.0, 0.0)) < 1.0e-5);
    assert!((get_quat_angle(pose.joint_states[1].rotation) - 0.5).abs() < 2.0e-3);
    animation.sample(7.0, &mut pose, 1.0);
    assert!(distance(pose.joint_states[1].translation, Vec3::new(2.0, 0.0, 0.0)) < 1.0e-5);

    let mut animation = animation;
    animation.is_looping = true;
    animation.sample(2.25, &mut pose, 1.0);
    assert!(distance(pose.joint_states[1].translation, Vec3::new(0.5, 0.0, 0.0)) < 1.0e-5);

    // Half weight blends from the current state.
    animation.is_looping = false;
    pose.joint_states[1] =
        SkeletonJointState::new(Vec3::new(0.0, 4.0, 0.0), Quat::IDENTITY);
    animation.sample(1.0, &mut pose, 0.5);
    assert!(distance(pose.joint_states[1].translation, Vec3::new(1.0, 2.0, 0.0)) < 1.0e-5);
    pose.calculate_joint_matrices();
    assert!(distance(pose.joint_matrices[1].p, Vec3::new(1.0, 2.0, 0.0)) < 1.0e-5);
}

#[test]
fn mapper() {
    // "mapping between a detailed and a ragdoll skeleton"
    // Low: pelvis -> chest -> head. High: pelvis -> spine1 -> spine2 -> chest -> head -> jaw.
    let mut low = Skeleton::new();
    low.add_joint("pelvis", -1);
    low.add_joint_with_parent_name("chest", "pelvis");
    low.add_joint_with_parent_name("head", "chest");
    let mut high = Skeleton::new();
    high.add_joint("pelvis", -1);
    high.add_joint_with_parent_name("spine1", "pelvis");
    high.add_joint_with_parent_name("spine2", "spine1");
    high.add_joint_with_parent_name("chest", "spine2");
    high.add_joint_with_parent_name("head", "chest");
    high.add_joint_with_parent_name("jaw", "head");

    let low_neutral = [
        Transform::new(Vec3::new(0.0, 1.0, 0.0), Quat::IDENTITY),
        Transform::new(Vec3::new(0.0, 1.6, 0.0), Quat::IDENTITY),
        Transform::new(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY),
    ];
    let mut high_pose = SkeletonPose::new(high.clone());
    high_pose.joint_matrices = vec![
        Transform::new(Vec3::new(0.0, 1.0, 0.0), Quat::IDENTITY),
        Transform::new(Vec3::new(0.0, 1.2, 0.0), Quat::IDENTITY),
        Transform::new(Vec3::new(0.0, 1.4, 0.0), Quat::IDENTITY),
        Transform::new(Vec3::new(0.0, 1.6, 0.0), Quat::IDENTITY),
        Transform::new(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY),
        Transform::new(Vec3::new(0.0, 1.95, 0.08), Quat::IDENTITY),
    ];
    high_pose.calculate_joint_states();
    let mapper = SkeletonMapper::new(
        low.clone(),
        &low_neutral,
        high.clone(),
        &high_pose.joint_matrices,
    )
    .unwrap();
    assert_eq!(mapper.mappings().len(), 3);
    assert!(mapper.chains().len() == 1 && mapper.chains()[0].high_indices == vec![0, 1, 2, 3]);
    assert_eq!(mapper.unmapped(), &[5]);

    // Bend the ragdoll at the pelvis: the chest swings to the side.
    let mut low_pose = SkeletonPose::new(low.clone());
    let bend = make_quat_from_axis_angle(Vec3::AXIS_Z, 0.6);
    low_pose.joint_matrices = vec![
        Transform::new(Vec3::new(0.0, 1.0, 0.0), bend),
        Transform::new(
            Vec3::new(0.0, 1.0, 0.0) + rotate_vector(bend, Vec3::new(0.0, 0.6, 0.0)),
            bend,
        ),
        Transform::new(
            Vec3::new(0.0, 1.0, 0.0) + rotate_vector(bend, Vec3::new(0.0, 1.0, 0.0)),
            bend,
        ),
    ];
    low_pose.calculate_joint_states();
    let mut mapped = SkeletonPose::new(high.clone());
    mapper.map(&low_pose, &high_pose, &mut mapped);
    assert!(distance(mapped.joint_matrices[3].p, low_pose.joint_matrices[1].p) < 1.0e-5);
    // The chain inner joints lie on the bent line between pelvis and chest.
    let direction = rotate_vector(bend, Vec3::AXIS_Y);
    for (k, height) in [(1usize, 0.2f32), (2usize, 0.4f32)] {
        assert!(
            distance(
                mapped.joint_matrices[k].p,
                Vec3::new(0.0, 1.0, 0.0) + height * direction
            ) < 1.0e-4
        );
    }
    // The jaw keeps its animated offset from the head.
    let jaw_local = inv_mul_transforms(mapped.joint_matrices[4], mapped.joint_matrices[5]);
    assert!(distance(jaw_local.p, Vec3::new(0.0, -0.05, 0.08)) < 1.0e-4);

    let mut back = SkeletonPose::new(low.clone());
    mapper.map_reverse(&mapped, &mut back);
    for i in 0..3 {
        assert!(distance(back.joint_matrices[i].p, low_pose.joint_matrices[i].p) < 1.0e-5);
    }
}

#[test]
fn fall() {
    // "a ragdoll falls, settles, and keeps its joints together within limits"
    let mut world = ground_world(Vec3::new(0.0, -10.0, 0.0));
    // Swift's `ragdoll.world` and the local `world` were the same class reference, so the rig and
    // the caller interleaved freely. Rust's `Ragdoll` owns a `&mut World` for its whole lifetime,
    // so the tests reach the world through a raw pointer - the same aliasing discipline
    // `collide_pass.rs` uses, and safe here because no access overlaps another.
    let world_ptr: *mut World = &mut world;
    let skeleton = human_skeleton();
    let settings = human_settings(&skeleton);
    let bind = human_bind_pose(&skeleton, Vec3::new(0.0, 0.5, 0.0));
    let mut ragdoll =
        Ragdoll::new(unsafe { &mut *world_ptr }, settings.clone(), &bind, None).unwrap();
    assert_eq!(ragdoll.bodies().len(), 11);
    assert!(!ragdoll.filter_joints().is_empty());

    // The pose read back equals the creation pose.
    let read = ragdoll.pose();
    for i in 0..skeleton.joint_count() {
        assert!(distance(read.world_transform(i as i32).p, bind.world_transform(i as i32).p) < 1.0e-5);
    }

    ragdoll.apply_linear_impulse(Vec3::new(30.0, 0.0, 10.0));
    for _ in 0..300 {
        // SAFETY: no other borrow of the world is live at this point.
        unsafe { &mut *world_ptr }.step(1.0 / 60.0, 4);
    }
    let bounds = ragdoll.bounds();
    assert!(bounds.lower_bound.y > -0.05);
    assert!(bounds.upper_bound.y < 1.0);

    // Joints are connected: each child joint origin coincides with its anchor on the parent.
    let pose = ragdoll.pose();
    for (i, joint) in skeleton.joints().iter().enumerate() {
        if joint.parent_index < 0 {
            continue;
        }
        // SAFETY: read-only world access while the ragdoll's `&mut` is idle.
        let world = unsafe { &*world_ptr };
        let sim = world.joint_sim(ragdoll.joints()[i]).unwrap();
        let anchor_a = transform_world_point(
            world.body_transform(ragdoll.bodies()[joint.parent_index as usize]),
            sim.local_frame_a.p,
        );
        let anchor_b =
            transform_world_point(world.body_transform(ragdoll.bodies()[i]), sim.local_frame_b.p);
        assert!(distance(anchor_a, anchor_b) < 0.02, "{}", joint.name);
        // Hinges stay hinges.
        if let RagdollJointKind::Hinge { lower, upper } = settings.parts[i].joint {
            let rel = world.six_dof_joint_pose(ragdoll.joints()[i]).unwrap().q;
            assert!(
                abs_float(rel.v.y as f64) < 0.03 && abs_float(rel.v.z as f64) < 0.03,
                "{}",
                joint.name
            );
            let angle = 2.0 * atan2_approx(rel.v.x, rel.s);
            assert!(
                angle > lower - 0.08 && angle < upper + 0.08,
                "{} {}",
                joint.name,
                angle
            );
        }
    }
    let _ = pose;
    assert!(!ragdoll.is_awake());
}

#[test]
fn motors() {
    // "motors drive the ragdoll to an animated pose"
    let mut world = ground_world(Vec3::ZERO);
    // SAFETY: see `fall`; the pointer is the port of Swift's shared `world` reference.
    let world_ptr: *mut World = &mut world;
    let skeleton = human_skeleton();
    let bind = human_bind_pose(&skeleton, Vec3::new(0.0, 2.0, 0.0));
    let mut ragdoll =
        Ragdoll::new(unsafe { &mut *world_ptr }, human_settings(&skeleton), &bind, None).unwrap();

    // Target: elbow and knee bent inside their hinge ranges (about the joint's y axis), head
    // turned.
    let mut target = bind.clone();
    let elbow = skeleton.joint_index("lowerArmL").unwrap();
    let knee = skeleton.joint_index("calfR").unwrap();
    let head = skeleton.joint_index("head").unwrap();
    target.joint_states[elbow as usize].rotation = mul_quat(
        target.joint_states[elbow as usize].rotation,
        make_quat_from_axis_angle(Vec3::AXIS_Y, -1.0),
    );
    target.joint_states[knee as usize].rotation = mul_quat(
        target.joint_states[knee as usize].rotation,
        make_quat_from_axis_angle(Vec3::AXIS_Y, 1.0),
    );
    target.joint_states[head as usize].rotation = mul_quat(
        target.joint_states[head as usize].rotation,
        make_quat_from_axis_angle(Vec3::AXIS_X, 0.5),
    );
    target.calculate_joint_matrices();

    ragdoll.drive_to_pose_using_motors(&target, 200.0, 6.0, 1.0);
    for _ in 0..240 {
        // SAFETY: no other borrow of the world is live at this point.
        unsafe { &mut *world_ptr }.step(1.0 / 60.0, 4);
    }
    let pose = ragdoll.pose();
    for joint in [elbow, knee, head] {
        let actual = rotation_error(
            pose.joint_states[joint as usize].rotation,
            target.joint_states[joint as usize].rotation,
        );
        assert!(
            actual < 0.05,
            "{} {} {}",
            skeleton.joints()[joint as usize].name,
            actual,
            rotation_error(
                bind.joint_states[joint as usize].rotation,
                target.joint_states[joint as usize].rotation
            )
        );
    }

    ragdoll.disable_motors();
    // SAFETY: read-only world access while the ragdoll's `&mut` is idle.
    let axis = unsafe { &*world_ptr }
        .six_dof_joint_axis(ragdoll.joints()[elbow as usize], SixDOFAxis::RotationX)
        .unwrap();
    assert_eq!(axis.motor_mode, SixDOFMotorMode::Off);
}

#[test]
fn kinematic() {
    // "kinematic driving reaches the pose in one step"
    let mut world = ground_world(Vec3::ZERO);
    // SAFETY: see `fall`; the pointer is the port of Swift's shared `world` reference.
    let world_ptr: *mut World = &mut world;
    let skeleton = human_skeleton();
    let bind = human_bind_pose(&skeleton, Vec3::new(0.0, 2.0, 0.0));
    let mut ragdoll =
        Ragdoll::new(unsafe { &mut *world_ptr }, human_settings(&skeleton), &bind, None).unwrap();
    let mut target = bind.clone();
    target.root_offset = Pos::new(0.3, 2.1, -0.2);
    ragdoll.drive_to_pose_using_kinematics(&target, 1.0 / 60.0);
    world.step(1.0 / 60.0, 4);
    for i in 0..skeleton.joint_count() {
        assert!(
            distance(
                world.body_position(ragdoll.bodies()[i]),
                target.world_transform(i as i32).p
            ) < 2.0e-3
        );
    }
}

#[test]
fn mass_and_collision() {
    // "mass stabilization and self collision options"
    let skeleton = human_skeleton();
    let mut settings = human_settings(&skeleton);
    let hand = skeleton.joint_index("lowerArmL").unwrap();
    settings.parts[hand as usize].shape_def.density = 50_000.0;
    settings.max_mass_ratio = 0.0;

    // `Ragdoll` owns the world borrow for its whole lifetime, so Swift's `loose.world.bodyMass(...)`
    // reads go through a raw pointer to the world, the same aliasing `collide_pass.rs` uses for
    // Swift's shared class reference: the reads do not overlap any write.
    let mut loose_world = ground_world(Vec3::new(0.0, -10.0, 0.0));
    let loose_world_ptr: *const World = &loose_world;
    let loose_bind = human_bind_pose(&skeleton, Vec3::ZERO);
    let loose = Ragdoll::new(&mut loose_world, settings.clone(), &loose_bind, None).unwrap();
    let upper = skeleton.joint_index("upperArmL").unwrap();
    // SAFETY: read-only access to a world that is only otherwise accessed through the ragdoll's
    // immutable mass queries.
    let loose_view = unsafe { &*loose_world_ptr };
    let raw_ratio = loose_view.body_mass(loose.bodies()[hand as usize])
        / loose_view.body_mass(loose.bodies()[upper as usize]);
    assert!(raw_ratio > 10.0);

    settings.max_mass_ratio = 3.0;
    let mut stable_world = ground_world(Vec3::new(0.0, -10.0, 0.0));
    let stable_world_ptr: *const World = &stable_world;
    let bind = human_bind_pose(&skeleton, Vec3::ZERO);
    let mut stable = Ragdoll::new(&mut stable_world, settings.clone(), &bind, None).unwrap();
    for (i, joint) in skeleton.joints().iter().enumerate() {
        if joint.parent_index < 0 {
            continue;
        }
        // SAFETY: as above; `body_mass` only reads.
        let stable_view = unsafe { &*stable_world_ptr };
        let ratio = stable_view.body_mass(stable.bodies()[i])
            / stable_view.body_mass(stable.bodies()[joint.parent_index as usize]);
        assert!(ratio <= 3.001 && ratio >= 1.0 / 3.001, "{}", joint.name);
    }
    let loose_total = loose.total_mass();
    let stable_total = stable.total_mass();
    // Swift compared `Float`s here, so the tolerance stays single precision.
    assert!((stable_total - loose_total).abs() < 1.0e-3 * loose_total);
    drop(loose);

    settings.self_collision = RagdollSelfCollision::None;
    settings.collision_group = -7;
    let mut isolated_world = ground_world(Vec3::new(0.0, -10.0, 0.0));
    let isolated_world_ptr: *const World = &isolated_world;
    let isolated_bind = human_bind_pose(&skeleton, Vec3::ZERO);
    let mut isolated =
        Ragdoll::new(&mut isolated_world, settings.clone(), &isolated_bind, None).unwrap();
    assert!(isolated.filter_joints().is_empty());
    // SAFETY: `isolated_world_ptr` only reads; the shape lookup below is the same access Swift made
    // through `isolated.world`.
    let isolated_view = unsafe { &*isolated_world_ptr };
    let shape = isolated_view.bodies[(isolated.bodies()[0].index1 - 1) as usize].head_shape_id;
    assert_eq!(isolated_view.shapes[shape as usize].filter.group_index, -7);

    let count = isolated_view
        .bodies
        .iter()
        .filter(|b| b.id != b3::NULL_INDEX)
        .count();
    isolated.destroy();
    // SAFETY: `destroy` only removes bodies and joints; the pointer still addresses the same world.
    let isolated_view = unsafe { &*isolated_world_ptr };
    assert_eq!(
        isolated_view
            .bodies
            .iter()
            .filter(|b| b.id != b3::NULL_INDEX)
            .count(),
        count - 11
    );

    stable.destroy();
}
