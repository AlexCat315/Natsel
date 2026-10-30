//! Ported from Tests/NatselTests/Simulation/WideJointTests.swift.

use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::{BodyID, JointID};
use crate::types::joint_defs::SphericalJointDef;
use crate::types::shape_def::ShapeDef;

/// Swift's nested `makeWorld(_:)` captured the test's `mixed` argument; here it is an explicit
/// parameter so the two `World`s in one case share the same spring/motor predicate.
fn make_world(wide: bool, mixed: bool) -> (World, Vec<JointID>) {
    let mut world = World::default();
    world.enable_wide_solver = wide;
    world.enable_sleep = false;
    let width = 10;
    let height = 8;
    let mut bodies: Vec<BodyID> = Vec::new();
    let mut joints: Vec<JointID> = Vec::new();
    let mut shape = ShapeDef::default();
    shape.filter.category_bits = 2;
    shape.filter.mask_bits = !2u64;
    for x in 0..width {
        for y in 0..height {
            let mut body = BodyDef::default();
            body.type_ = if y == 0 {
                BodyType::StaticBody
            } else {
                BodyType::DynamicBody
            };
            body.position = Pos::new(x as f64, -(y as f64), 0.0);
            body.angular_velocity = Vec3::new(0.03 * x as f32, 0.01 * y as f32, -0.02);
            let id = world.create_body(&body);
            let _ = world.create_sphere_shape(id, &shape, Sphere::new(Vec3::ZERO, 0.4));

            let mut joint = SphericalJointDef::default();
            joint.base.body_id_b = id;
            joint.enable_spring = mixed && (x + y) % 11 == 0;
            joint.hertz = 2.0;
            joint.damping_ratio = 0.2;
            joint.enable_motor = mixed && (x + 2 * y) % 17 == 0;
            joint.max_motor_torque = 0.4;
            joint.motor_velocity = Vec3::new(0.2, 0.0, 0.1);
            if y > 0 {
                joint.base.body_id_a = *bodies.last().unwrap();
                joint.base.local_frame_a.p = Vec3::new(0.0, -0.5, 0.0);
                joint.base.local_frame_b.p = Vec3::new(0.0, 0.5, 0.0);
                joints.push(world.create_spherical_joint(&joint));
            }
            if x > 0 {
                joint.base.body_id_a = bodies[bodies.len() - height as usize];
                joint.base.local_frame_a.p = Vec3::new(0.5, 0.0, 0.0);
                joint.base.local_frame_b.p = Vec3::new(-0.5, 0.0, 0.0);
                joints.push(world.create_spherical_joint(&joint));
            }
            bodies.push(id);
        }
    }
    (world, joints)
}

// "point joints, angular fallbacks and disabled cached impulses agree", arguments: [false, true]
fn run_joint_grid(mixed: bool) {
    let (mut scalar, scalar_ids) = make_world(false, mixed);
    let (mut wide, wide_ids) = make_world(true, mixed);

    let mut saw_wide = false;
    let mut saw_partial = false;
    for step in 0..120 {
        if step == 40 {
            for (&id_a, &id_b) in scalar_ids.iter().zip(wide_ids.iter()) {
                let _ = scalar.enable_spherical_joint_spring(id_a, false);
                let _ = wide.enable_spherical_joint_spring(id_b, false);
                let _ = scalar.enable_spherical_joint_motor(id_a, false);
                let _ = wide.enable_spherical_joint_motor(id_b, false);
            }
        }
        scalar.enable_warm_starting = step % 23 != 0;
        wide.enable_warm_starting = scalar.enable_warm_starting;
        scalar.step(1.0f32 / 60.0, 4);
        wide.step(1.0f32 / 60.0, 4);
        assert!(
            scalar.world_state_hash() == wide.world_state_hash(),
            "step {step}, mixed {mixed}"
        );

        for (&id_a, &id_b) in scalar_ids.iter().zip(wide_ids.iter()) {
            let a = scalar.read_joint_sim(id_a.index1 - 1).spherical().linear_impulse;
            let b = wide.read_joint_sim(id_b.index1 - 1).spherical().linear_impulse;
            assert!(
                a.x.to_bits() == b.x.to_bits()
                    && a.y.to_bits() == b.y.to_bits()
                    && a.z.to_bits() == b.z.to_bits(),
                "joint impulse step {step}"
            );
        }

        for list in wide.solver_joints.batches.iter() {
            saw_wide = saw_wide || !list.is_empty();
            saw_partial = saw_partial || list.iter().any(|item| item.lane_count < 4);
        }
    }
    assert!(saw_wide);
    assert!(saw_partial);
}

#[test]
fn joint_grid_false() {
    run_joint_grid(false);
}

#[test]
fn joint_grid_true() {
    run_joint_grid(true);
}
