//! Ported from Tests/NatselTests/SleepTests.swift.
//!
//! A resting box must enter a sleeping set on a fixed schedule, and the resulting state
//! must be byte-identical across two independent runs.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::world_runtime::SetType;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::shape_def::ShapeDef;

fn fnv(mut h: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(1_099_511_628_211);
    }
    h
}

#[inline]
fn fnv_f32(h: u64, x: f32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[inline]
fn fnv_f64(h: u64, x: f64) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[test]
fn box_sleeps() {
    // "box sleep timing and state are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        // TODO(migration): add `impl Default for World` (or use `World::new`) in the build phase.
        let mut world = World::default();

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        let ground = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, 0.6, 0.0);
        let box_body = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

        for i in 0..180 {
            world.step(1.0 / 60.0, 4);
            if i == 59 {
                // After two seconds of resting the body should be in a sleeping set.
                assert!(
                    world.bodies[box_body.index1 as usize].set_index
                        >= SetType::FirstSleepingSet.raw_value()
                );
                assert!(world.body_move_events.is_empty());
            }
        }

        let b = &world.bodies[box_body.index1 as usize];
        let sim = &world.solver_sets[b.set_index as usize].body_sims[b.local_index as usize];
        let v = world.body_linear_velocity(box_body);

        let mut h: u64 = 14_695_981_039_346_656_037;
        let p = sim.transform.p;
        let q = sim.transform.q;
        let vv = v;
        let flags = b.flags;
        let sleep_time = b.sleep_time;
        h = fnv_f64(h, p.x);
        h = fnv_f64(h, p.y);
        h = fnv_f64(h, p.z);
        h = fnv_f32(h, q.v.x);
        h = fnv_f32(h, q.v.y);
        h = fnv_f32(h, q.v.z);
        h = fnv_f32(h, q.s);
        h = fnv_f32(h, vv.x);
        h = fnv_f32(h, vv.y);
        h = fnv_f32(h, vv.z);
        h = fnv(h, &flags.to_ne_bytes());
        h = fnv_f32(h, sleep_time);

        assert_eq!(b.set_index, 3);
        assert_eq!(b.local_index, 0);
        samples.push(b.sleep_time.to_bits() as u64);
        samples.push(h);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
