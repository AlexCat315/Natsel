//! Ported from Tests/NatselTests/Simulation/CallbackTests.swift.

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

use crate::collision::hull::make_box_hull;
use crate::math::types::Pos;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;

fn make_scene(world: &mut World, custom_filtering: bool) -> (BodyID, BodyID) {
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 1.0, 0.0);
    let box_body = world.create_body(&def);
    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    shape_def.enable_custom_filtering = custom_filtering;
    shape_def.enable_pre_solve_events = true;
    let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
    (ground, box_body)
}

#[test]
fn custom_filter() {
    let mut world = World::default();
    world.enable_sleep = false;
    let calls = Arc::new(AtomicUsize::new(0));
    let calls_clone = calls.clone();
    world.set_custom_filter(Some(Arc::new(move |_, _| {
        calls_clone.fetch_add(1, Ordering::SeqCst);
        false
    })));
    let (_, box_body) = make_scene(&mut world, true);

    for _ in 0..60 {
        world.step(1.0f32 / 60.0, 4);
    }

    assert!(calls.load(Ordering::SeqCst) > 0);
    // The filter removed all contacts, so the box falls through the ground.
    assert!(world.body_transform(box_body).p.y < -1.0);
    assert_eq!(world.contact_count(), 0);
}

#[test]
fn pre_solve() {
    let mut world = World::default();
    world.enable_sleep = false;
    let calls = Arc::new(AtomicUsize::new(0));
    let calls_clone = calls.clone();
    world.set_pre_solve_callback(Some(Arc::new(move |_, _, _, _| {
        calls_clone.fetch_add(1, Ordering::SeqCst);
        false
    })));
    let (_, box_body) = make_scene(&mut world, false);

    for _ in 0..60 {
        world.step(1.0f32 / 60.0, 4);
    }

    assert!(calls.load(Ordering::SeqCst) > 0);
    assert!(world.body_transform(box_body).p.y < -1.0);
}

#[test]
fn pre_solve_allows() {
    let mut world = World::default();
    world.enable_sleep = false;
    let calls = Arc::new(AtomicUsize::new(0));
    let calls_clone = calls.clone();
    world.set_pre_solve_callback(Some(Arc::new(move |_, _, _, _| {
        calls_clone.fetch_add(1, Ordering::SeqCst);
        true
    })));
    let (_, box_body) = make_scene(&mut world, false);

    for _ in 0..120 {
        world.step(1.0f32 / 60.0, 4);
    }

    assert!(calls.load(Ordering::SeqCst) > 0);
    // The box rests on the ground.
    assert!(world.body_transform(box_body).p.y > 0.9);
}
