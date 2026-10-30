//! Ported from Tests/NatselTests/Simulation/ContinuousMoveEventTests.swift.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::shape_def::ShapeDef;

// "move events agree with the body pose across a CCD impact"
#[test]
fn move_event_matches_pose() {
    let mut world = World::default();

    // Thin static wall, near face at x = 0.1
    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let wall = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(wall, &shape_def, &make_box_hull(0.1, 5.0, 5.0));

    // Fast dynamic sphere fired at the wall.
    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.gravity_scale = 0.0;
    def.position = Pos::new(3.0, 0.0, 0.0);
    def.linear_velocity = Vec3::new(-30.0, 0.0, 0.0);
    let ball = world.create_body(&def);

    shape_def = ShapeDef::default();
    shape_def.density = 1.0;
    let _ = world.create_sphere_shape(ball, &shape_def, &Sphere::new(Vec3::ZERO, 0.25));

    let time_step: f32 = 1.0 / 60.0;
    let mut have_move = false;

    for _ in 0..30 {
        world.step(time_step, 4);

        let transform = world.body_transform(ball);

        for event in world.body_events() {
            if event.body_id != ball {
                continue;
            }

            have_move = true;

            // The move event must carry the same pose the body reports, CCD rewind included.
            assert_eq!(event.transform.p, transform.p);
            assert_eq!(event.transform.q, transform.q);
        }
    }

    assert!(have_move);

    // Tunnel check
    let final_pos = world.body_transform(ball).p;
    assert!(final_pos.x > 0.2 && final_pos.x < 0.8);
}

// "move events report the body ids and transforms of moving bodies"
#[test]
fn move_events_are_consistent() {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let ground = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(0.0, 3.0, 0.0);
    let box_body = world.create_body(&def);
    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

    let mut saw_box = false;
    for _ in 0..30 {
        world.step(1.0f32 / 60.0, 4);
        let transform = world.body_transform(box_body);
        for event in world.body_events() {
            if event.body_id != box_body {
                continue;
            }
            saw_box = true;
            assert_eq!(event.transform.p, transform.p);
            assert_eq!(event.transform.q, transform.q);
            assert_eq!(event.fell_asleep, false);
        }
    }
    assert!(saw_box);
}
