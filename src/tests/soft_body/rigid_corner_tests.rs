//! Soft rigid corner contacts.

use crate::collision::hull::make_offset_box_hull;
use crate::math::types::Vec3;
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_runtime::{SoftBodyDef, SoftBodyID};
use crate::soft_body::soft_body_settings::SoftBodyVertex;
use crate::soft_body::soft_body_settings::SoftBodySettings;
use crate::types::body_def::BodyDef;
use crate::types::shape_def::ShapeDef;

fn scene(workers: i32) -> (World, SoftBodyID) {
    let mut world = World::default();
    world.gravity = Vec3::new(-10.0, -10.0, 0.0);
    world.worker_count = workers;
    world.enable_sleep = false;

    let floor = world.create_body(&BodyDef::default());
    let wall = world.create_body(&BodyDef::default());

    let mut shape = ShapeDef::default();
    shape.base_material.friction = 0.0;
    world.create_hull_shape(
        floor,
        &shape,
        &make_offset_box_hull(5.0, 0.1, 5.0, Vec3::new(0.0, -0.1, 0.0)),
    );
    world.create_hull_shape(
        wall,
        &shape,
        &make_offset_box_hull(0.1, 5.0, 5.0, Vec3::new(-0.1, 0.0, 0.0)),
    );

    let mut settings = SoftBodySettings::new();
    settings.vertices = vec![SoftBodyVertex::new(
        Vec3::new(0.005, 0.005, 0.0),
        Vec3::ZERO,
        1.0,
    )];
    let mut def = SoftBodyDef::default();
    def.settings = settings;
    def.vertex_radius = 0.02;
    def.iteration_count = 4;
    def.enable_sleep = false;
    let soft = world.create_soft_body(&def);
    (world, soft)
}

fn corner(workers: i32) {
    let (mut world, soft) = scene(workers);
    for _ in 0..120 {
        world.step(1.0 / 60.0, 4);
        let p = world.soft_body_vertex_positions(soft)[0];
        assert!(p.x >= 0.019, "vertex escaped the wall: x = {}", p.x);
        assert!(p.y >= 0.019, "vertex escaped the floor: y = {}", p.y);
    }
    assert_eq!(world.soft_body_contacts(soft).len(), 2);
}

#[test]
// "a vertex respects both a wall and a floor" (arguments: workers = 1)
fn corner_single_worker() {
    corner(1);
}

#[test]
// "a vertex respects both a wall and a floor" (arguments: workers = 4)
fn corner_four_workers() {
    corner(4);
}

#[test]
// "multi-plane contact state restores deterministically"
fn rollback() {
    let (mut world, soft) = scene(1);
    world.step(1.0 / 60.0, 4);
    let saved = world.save_state();
    for _ in 0..20 {
        world.step(1.0 / 60.0, 4);
    }
    let expected = world.soft_body_vertex_positions(soft);
    assert!(world.restore_state(&saved));
    world.worker_count = 4;
    for _ in 0..20 {
        world.step(1.0 / 60.0, 4);
    }
    assert_eq!(world.soft_body_vertex_positions(soft), expected);
}
