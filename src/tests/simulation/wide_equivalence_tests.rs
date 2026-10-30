//! Ported from Tests/NatselTests/Simulation/WideEquivalenceTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::tests::simulation::determinism_scene::{create_mesh_drop, create_wave_pile};
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

fn scene(wide: bool) -> u64 {
    let mut world = World::default();
    world.enable_sleep = false;
    world.enable_wide_solver = wide;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    def.position = Pos::new(0.0, -1.0, 0.0);
    let ground = world.create_body(&def);
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(10.0, 1.0, 10.0));

    // A small grid so several contacts share a color.
    for x in 0..3 {
        for z in 0..3 {
            def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::new(
                x as f64 * 1.02 - 1.0,
                0.6 + z as f64 * 0.1,
                z as f64 * 1.02 - 1.0,
            );
            let box_body = world.create_body(&def);
            let mut shape_def = ShapeDef::default();
            shape_def.density = 1000.0;
            let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
        }
    }

    for _ in 0..60 {
        world.step(1.0 / 60.0, 4);
    }
    world.world_state_hash()
}

// "wide and scalar paths agree"
#[test]
fn wide_matches_scalar() {
    let scalar = scene(false);
    let wide = scene(true);
    assert!(
        scalar == wide,
        "scalar {scalar:x} wide {wide:x}"
    );
}

// "terrain and mesh batches agree at every step", arguments: [false, true]
fn run_multi_manifold(mesh: bool) {
    let mut scalar = World::default();
    let mut wide = World::default();
    scalar.enable_wide_solver = false;
    wide.enable_wide_solver = true;
    if mesh {
        let _ = create_mesh_drop(&mut scalar, Pos::ZERO);
        let _ = create_mesh_drop(&mut wide, Pos::ZERO);
    } else {
        let _ = create_wave_pile(&mut scalar);
        let _ = create_wave_pile(&mut wide);
    }
    for step in 0..300 {
        scalar.step(1.0f32 / 60.0, 4);
        wide.step(1.0f32 / 60.0, 4);
        assert!(
            scalar.world_state_hash() == wide.world_state_hash(),
            "step {step}, mesh {mesh}"
        );
    }
}

#[test]
fn multi_manifold_terrain() {
    run_multi_manifold(false);
}

#[test]
fn multi_manifold_mesh() {
    run_multi_manifold(true);
}

fn make_mixed_world(enable_wide: bool, propagation: bool) -> World {
    let mut world = World::default();
    world.enable_wide_solver = enable_wide;
    world.enable_sleep = false;
    world.enable_restitution_propagation = propagation;

    let mut ground_def = BodyDef::default();
    ground_def.type_ = BodyType::KinematicBody;
    ground_def.position = Pos::new(0.0, -1.0, 0.0);
    ground_def.linear_velocity = Vec3::new(0.15, 0.0, -0.1);
    let ground = world.create_body(&ground_def);
    let _ = world.create_hull_shape(ground, &ShapeDef::default(), &make_box_hull(20.0, 1.0, 20.0));

    for i in 0..11 {
        let mut def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(
            (i % 4) as f64 * 2.0 - 3.0,
            0.6 + (i % 3) as f64 * 0.05,
            (i / 4) as f64 * 2.0 - 2.0,
        );
        def.linear_velocity = Vec3::new(0.1, -3.0, -0.2);
        def.angular_velocity = Vec3::new(0.2, 0.1, -0.1);
        let body = world.create_body(&def);
        let mut shape = ShapeDef::default();
        shape.base_material.restitution = (i % 3) as f32 * 0.4;
        shape.base_material.rolling_resistance = if i % 2 == 0 { 0.2 } else { 0.0 };
        shape.base_material.tangent_velocity = Vec3::new(0.3, 0.0, -0.2);
        if i % 2 == 0 {
            let _ = world.create_sphere_shape(body, &shape, Sphere::new(Vec3::ZERO, 0.5));
        } else {
            let _ = world.create_hull_shape(body, &shape, &make_box_hull(0.5, 0.5, 0.5));
        }
    }
    world
}

// "mixed bounce, friction, warm starting, and partial batches agree", arguments: [false, true]
fn run_mixed_contacts(propagation: bool) {
    let mut scalar = make_mixed_world(false, propagation);
    let mut wide = make_mixed_world(true, propagation);
    let mut saw_wide = false;
    let mut saw_partial_batch = false;
    for step in 0..100 {
        scalar.enable_warm_starting = step % 15 != 0;
        wide.enable_warm_starting = scalar.enable_warm_starting;
        scalar.step(1.0f32 / 60.0, 4);
        wide.step(1.0f32 / 60.0, 4);

        for color in wide.solver_contacts.wide_constraints.iter() {
            saw_wide = saw_wide || !color.is_empty();
            saw_partial_batch =
                saw_partial_batch || color.iter().any(|item| item.contact_ids[3] < 0);
        }

        assert!(
            scalar.world_state_hash() == wide.world_state_hash(),
            "step {step}"
        );
        for id in 0..scalar.contacts.len() {
            assert!(
                scalar.contacts[id].manifolds() == wide.contacts[id].manifolds(),
                "step {step} contact {id}"
            );
        }
    }
    assert!(saw_wide);
    assert!(saw_partial_batch);
}

#[test]
fn mixed_contacts_bounce() {
    run_mixed_contacts(false);
}

#[test]
fn mixed_contacts_propagation() {
    run_mixed_contacts(true);
}
