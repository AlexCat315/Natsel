//! Ported from Tests/NatselTests/Simulation/WorldStateHashTests.swift.
//!
//! `world_state_hash` uses FNV-1a over every live body's transform and velocities.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::shape_def::ShapeDef;

fn make_scene() -> World {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    for i in 0..3 {
        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(
            0.3 * i as f32,
            0.6 + i as f32 * 1.1,
            0.1 * i as f32,
        );
        let box_body = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    }

    world
}

// "independent scenes produce the same state hash at every step"
#[test]
fn state_hash() {
    let mut first = make_scene();
    let mut replay = make_scene();
    assert_eq!(first.world_state_hash(), replay.world_state_hash());
    for _ in 0..60 {
        first.step(1.0 / 60.0, 4);
        replay.step(1.0 / 60.0, 4);
        assert_eq!(first.world_state_hash(), replay.world_state_hash());
    }
}

// "state hash covers positions to double precision"
#[test]
fn state_hash_precision() {
    let mut world = make_scene();
    let before = world.world_state_hash();

    // A one-ULP change in a single body position must change the hash.
    let id = world.make_body_id(3);
    let transform = world.body_transform(id);
    let mut moved = transform.p;
    moved.x = moved.x.next_up();
    world.set_body_transform(id, moved, transform.q);

    assert!(world.world_state_hash() != before);
}

// "an empty world hashes to the FNV-1a seed"
#[test]
fn empty_world() {
    assert_eq!(World::default().world_state_hash(), 0xcbf2_9ce4_8422_2325);
}
