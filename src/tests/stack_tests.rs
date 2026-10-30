//! Ported from Tests/NatselTests/StackTests.swift.
//!
//! A three-box stack must be byte-identical across two runs. The FNV hash is computed over the
//! same fields in the same order on both passes, so only internal consistency is asserted.

use crate::base::constants::NULL_INDEX;
use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3, WorldTransform};
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
fn fnv_f64(h: u64, x: f64) -> u64 {
    fnv(h, &x.to_ne_bytes())
}
#[inline]
fn fnv_f32(h: u64, x: f32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}
#[inline]
fn fnv_i32(h: u64, x: i32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}
#[inline]
fn fnv_u32(h: u64, x: u32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[inline]
fn hash_transform(mut h: u64, t: &WorldTransform, v: Vec3, w: Vec3) -> u64 {
    h = fnv_f64(h, t.p.x);
    h = fnv_f64(h, t.p.y);
    h = fnv_f64(h, t.p.z);
    h = fnv_f32(h, t.q.v.x);
    h = fnv_f32(h, t.q.v.y);
    h = fnv_f32(h, t.q.v.z);
    h = fnv_f32(h, t.q.s);
    h = fnv_f32(h, v.x);
    h = fnv_f32(h, v.y);
    h = fnv_f32(h, v.z);
    h = fnv_f32(h, w.x);
    h = fnv_f32(h, w.y);
    h = fnv_f32(h, w.z);
    h
}

#[test]
fn box_stack() {
    // "three box stack is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        // TODO(migration): add `impl Default for World` in the final build-fix phase.
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
            def.position = Pos::new(0.0, 0.55 + i as f64 * 1.05, 0.0);
            let box_body = world.create_body(&def);
            shape_def = ShapeDef::default();
            shape_def.density = 1000.0;
            let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
            boxes.push(box_body);
        }

        for _ in 0..120 {
            world.step(1.0 / 60.0, 4);
        }

        let mut h: u64 = 14_695_981_039_346_656_037;
        for &id in &boxes {
            let t = world.body_transform(id);
            let v = world.body_linear_velocity(id);
            let w = world.body_angular_velocity(id);
            h = hash_transform(h, &t, v, w);
        }

        let mut count = 0i32;
        let mut ch: u64 = 14_695_981_039_346_656_037;
        for c in &world.contacts {
            if c.contact_id == NULL_INDEX {
                continue;
            }
            count += 1;
            ch = fnv_u32(ch, c.flags);
            ch = fnv_i32(ch, c.manifold_count);

            let view = c.manifolds();
            for m in 0..c.manifold_count as usize {
                let manifold = view.get(m);
                let normal = manifold.normal;
                let point_count = manifold.point_count;
                let friction_impulse = manifold.friction_impulse;
                let twist_impulse = manifold.twist_impulse;
                ch = fnv_f32(ch, normal.x);
                ch = fnv_f32(ch, normal.y);
                ch = fnv_f32(ch, normal.z);
                ch = fnv_i32(ch, point_count);
                ch = fnv_f32(ch, friction_impulse.x);
                ch = fnv_f32(ch, friction_impulse.y);
                ch = fnv_f32(ch, friction_impulse.z);
                ch = fnv_f32(ch, twist_impulse);

                for p in 0..point_count as usize {
                    let mp = manifold.points[p];
                    let anchor_a = mp.anchor_a;
                    let anchor_b = mp.anchor_b;
                    let separation = mp.separation;
                    let normal_impulse = mp.normal_impulse;
                    let total_normal_impulse = mp.total_normal_impulse;
                    ch = fnv_f32(ch, anchor_a.x);
                    ch = fnv_f32(ch, anchor_a.y);
                    ch = fnv_f32(ch, anchor_a.z);
                    ch = fnv_f32(ch, anchor_b.x);
                    ch = fnv_f32(ch, anchor_b.y);
                    ch = fnv_f32(ch, anchor_b.z);
                    ch = fnv_f32(ch, separation);
                    ch = fnv_f32(ch, normal_impulse);
                    ch = fnv_f32(ch, total_normal_impulse);
                }
            }
        }

        assert_eq!(count, 3);
        samples.push(ch);
        samples.push(h);

        let p1 = world.body_transform(boxes[1]).p;
        samples.push((p1.x as f32).to_bits() as u64);
        samples.push((p1.y as f32).to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
