//! Ported from Tests/NatselTests/Simulation/WorldCreateDestroyTests.swift.

use crate::collision::hull::{make_box_hull, make_cube_hull};
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::shape_def::ShapeDef;

#[test]
fn empty_world() {
    let mut world = World::default();
    for _ in 0..60 {
        world.step(1.0f32 / 60.0, 1);
    }
    assert_eq!(world.body_count(), 0);
    assert_eq!(world.contact_count(), 0);
}

#[test]
fn destroy_all_bodies() {
    let mut world = World::default();
    let body_count = 10;
    let cube = make_cube_hull(0.5);

    let mut body_ids: Vec<Option<_>> = vec![None; body_count as usize];
    let mut count = 0;
    let mut creating = true;

    for _ in 0..(2 * body_count + 10) {
        if creating {
            if count < body_count {
                let mut def = BodyDef::default();
                def.type_ = BodyType::DynamicBody;
                let body = world.create_body(&def);
                let mut shape_def = ShapeDef::default();
                let _ = world.create_hull_shape(body, &shape_def, &cube);
                body_ids[count] = Some(body);
                count += 1;
            } else {
                creating = false;
            }
        } else if count > 0 {
            world.destroy_body(body_ids[count - 1].unwrap());
            body_ids[count - 1] = None;
            count -= 1;
        }

        world.step(1.0f32 / 60.0, 3);
    }

    assert_eq!(world.body_count(), 0);
    assert_eq!(world.contact_count(), 0);
    assert_eq!(world.awake_body_count(), 0);

    for id in body_ids.iter().flatten() {
        assert!(!world.body_is_valid(*id));
    }
}

#[test]
fn destroyed_body_is_invalid() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let body = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(body, &shape_def, &make_cube_hull(0.5));

    assert!(world.body_is_valid(body));
    world.step(1.0f32 / 60.0, 1);

    world.destroy_body(body);
    assert!(!world.body_is_valid(body));
    assert_eq!(world.body_count(), 0);
}

#[test]
fn id_recycling() {
    let mut world = World::default();

    let mut def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    let first = world.create_body(&def);
    let second = world.create_body(&def);

    world.destroy_body(second);
    world.destroy_body(first);

    let recycled = world.create_body(&def);
    assert_eq!(recycled.index1, first.index1);

    let recycled2 = world.create_body(&def);
    assert_eq!(recycled2.index1, second.index1);
}

#[test]
fn contacts_released_on_destroy() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    let mut boxes: Vec<_> = Vec::new();
    for i in 0..3 {
        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, 0.6 + (i as f64) * 1.05, 0.0);
        let box_body = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
        boxes.push(box_body);
    }

    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
    }

    let contact_count = world.contact_count();
    assert!(contact_count > 0);

    for b in &boxes {
        world.destroy_body(*b);
    }

    assert_eq!(world.body_count(), 1);
    assert_eq!(world.contact_count(), 0);
}
