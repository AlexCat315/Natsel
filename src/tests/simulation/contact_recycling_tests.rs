//! Ported from Tests/NatselTests/Simulation/ContactRecyclingTests.swift.

use crate::base::constants as b3;
use crate::collision::dynamic_tree::{is_empty_node, is_node_moved, DynamicTree};
use crate::collision::hull::make_box_hull;
use crate::math::functions::{abs_float, min_float};
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType, MotionLocks};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::shape_def::ShapeDef;

// "recycling flag round trips and syncs to the sim"
#[test]
fn recycling_flag() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body_a = world.create_body(&def);
    assert!(world.is_body_contact_recycling_enabled(body_a));

    world.enable_body_contact_recycling(body_a, false);
    assert_eq!(world.is_body_contact_recycling_enabled(body_a), false);

    world.enable_body_contact_recycling(body_a, true);
    assert!(world.is_body_contact_recycling_enabled(body_a));

    // Per-def opt-out at creation
    def.enable_contact_recycling = false;
    let body_b = world.create_body(&def);
    assert_eq!(world.is_body_contact_recycling_enabled(body_b), false);

    // Stepping after toggling must not trip the flag-sync validator
    world.step(1.0f32 / 60.0, 4);
}

/// A slow sphere closes on a static round target from just outside contact. The step is too short
/// to make the sphere fast, so the pair is eligible for recycling. Without a speculative margin the
/// pair went empty and recycling then let the sphere sink through.
fn run_recycled_approach(capsule_target: bool) -> f32 {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let target = world.create_body(&def);

    let shape_def = ShapeDef::default();
    let target_radius: f32 = 0.5;
    if capsule_target {
        let _ = world.create_capsule_shape(
            target,
            &shape_def,
            &Capsule::new(Vec3::new(-1.0, 0.0, 0.0), Vec3::new(1.0, 0.0, 0.0), target_radius),
        );
    } else {
        let _ = world.create_sphere_shape(
            target,
            &shape_def,
            &Sphere::new(Vec3::ZERO, target_radius),
        );
    }

    let radius: f32 = 0.25;
    let total_radius = target_radius + radius;
    let time_step: f32 = 1.0 / 60.0;

    // Half the speculative distance per step
    let speed = 0.5 * b3::speculative_distance() / time_step;

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(
        0.0,
        f64::from(total_radius + 0.25 * b3::speculative_distance()),
        0.0,
    );
    def.linear_velocity = Vec3::new(0.0, -speed, 0.0);
    let body = world.create_body(&def);
    let _ = world.create_sphere_shape(body, &shape_def, &Sphere::new(Vec3::ZERO, radius));

    let mut min_separation = f32::MAX;
    for _ in 0..20 {
        world.step(time_step, 4);
        let p = world.body_transform(body).p;
        min_separation = min_float(min_separation, p.y as f32 - total_radius);
    }

    min_separation
}

// "recycling does not let a slow sphere sink into a sphere target"
#[test]
fn recycled_sphere_approach() {
    let min_separation = run_recycled_approach(false);
    assert!(min_separation > -b3::linear_slop());
    assert!(abs_float(min_separation) < 1e-4);
}

// "recycling does not let a slow sphere sink into a capsule target"
#[test]
fn recycled_capsule_approach() {
    let min_separation = run_recycled_approach(true);
    assert!(min_separation > -b3::linear_slop());
    assert!(abs_float(min_separation) < 1e-4);
}

// "a recycled pair keeps producing contacts while resting"
#[test]
fn resting_contact_persists() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 1.0, 0.0);
    let ball = world.create_body(&def);
    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_sphere_shape(ball, &shape_def, &Sphere::new(Vec3::ZERO, 0.5));

    for _ in 0..180 {
        world.step(1.0f32 / 60.0, 4);
    }

    // The ball rests on the ground: the contact survived recycling every step.
    assert!(abs_float((world.body_transform(ball).p.y - 1.0) as f32) < 0.01);
    assert!(world.contact_count() > 0);
}

// MARK: - Enlarged proxy lifetime

fn count_moved_nodes(tree: &DynamicTree) -> i32 {
    let mut count = 0;
    for i in 0..tree.node_end as usize {
        let node = tree.nodes[i];
        if !is_empty_node(node) && is_node_moved(node) {
            count += 1;
        }
    }
    count
}

// "a proxy enlarged then destroyed leaves the tree consistent"
#[test]
fn enlarged_proxy_destroyed() {
    let mut world = World::default();
    world.gravity = Vec3::ZERO;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;

    let shape_def = ShapeDef::default();
    let sphere = Sphere::new(Vec3::ZERO, 0.25);

    // Resting bodies keep the tree deep enough that the mover has an ancestor that outlives it
    for i in 0..8 {
        def.position = Pos::new(f64::from(2.0 * i as f32), 0.0, 0.0);
        def.linear_velocity = Vec3::ZERO;
        let resting = world.create_body(&def);
        let _ = world.create_sphere_shape(resting, &shape_def, &sphere);
    }

    // This body outruns its AABB margin every step
    def.position = Pos::new(7.0, 1.0, 0.0);
    def.linear_velocity = Vec3::new(6.0, 0.0, 0.0);
    let mover = world.create_body(&def);
    let _ = world.create_sphere_shape(mover, &shape_def, &sphere);

    let time_step: f32 = 1.0 / 60.0;
    world.step(time_step, 4);

    let tree_index = BodyType::DynamicBody as i32 as usize;
    assert!(count_moved_nodes(&world.broad_phase.trees[tree_index]) > 0);

    world.destroy_body(mover);

    // Removing the leaf re-derives the marks along its path, so the tree is already consistent
    // here and nothing is left marked to drive a rebuild.
    world.broad_phase.trees[tree_index].validate();
    assert_eq!(count_moved_nodes(&world.broad_phase.trees[tree_index]), 0);

    world.step(time_step, 4);

    assert_eq!(count_moved_nodes(&world.broad_phase.trees[tree_index]), 0);
    assert!(world.broad_phase.trees[tree_index].dfs_ordered);
}

// MARK: - Flag synchronization

// "sleep flag round trips and survives a step"
#[test]
fn sleep_flag_sync() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.enable_sleep = false;
    let body = world.create_body(&def);

    assert_eq!(world.is_body_sleep_enabled(body), false);

    world.enable_body_sleep(body, true);
    assert!(world.is_body_sleep_enabled(body));

    world.step(1.0f32 / 60.0, 4);
    assert!(world.is_body_sleep_enabled(body));
}

// "motion locks do not wipe the bullet flag"
#[test]
fn bullet_survives_motion_locks() {
    let mut world = World::default();

    // Reproducer A: create non-bullet, set bullet, then change motion locks.
    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.is_bullet = false;
    let body_a = world.create_body(&def);

    assert_eq!(world.is_body_bullet(body_a), false);
    world.set_body_bullet(body_a, true);
    assert!(world.is_body_bullet(body_a));

    let mut locks = MotionLocks::default();
    locks.linear_x = true;
    world.set_body_motion_locks(body_a, &locks);

    assert!(world.is_body_bullet(body_a));
    assert!(world.body_motion_locks(body_a).linear_x);

    // Reproducer B: create bullet, clear it, then change motion locks.
    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.is_bullet = true;
    let body_b = world.create_body(&def);

    assert!(world.is_body_bullet(body_b));
    world.set_body_bullet(body_b, false);
    assert_eq!(world.is_body_bullet(body_b), false);

    world.set_body_motion_locks(body_b, &locks);
    assert_eq!(world.is_body_bullet(body_b), false);
}

// "bullet flag survives a step"
#[test]
fn bullet_after_step() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 5.0, 0.0);
    def.linear_velocity = Vec3::new(20.0, 0.0, 0.0);
    let body = world.create_body(&def);

    let mut shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    world.set_body_bullet(body, true);
    for _ in 0..5 {
        world.step(1.0f32 / 60.0, 4);
    }

    assert!(world.is_body_bullet(body));
}
