//! Ported from Tests/NatselTests/BounceTests.swift.
//!
//! A bouncing ball must produce a byte-identical state across two independent runs
//! (determinism contract). The FNV hash is computed over the same scalars in the same
//! order on both passes, so only internal consistency is asserted.

use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
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
fn bouncing_ball() {
    // "bouncing ball is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        // TODO(migration): add `impl Default for World` (or use `World::new`) in the build phase.
        let mut world = World::default();
        world.enable_sleep = false;

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        let ground = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        shape_def.base_material.restitution = 0.8;
        let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(0.0, 3.0, 0.0);
        let ball = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        shape_def.base_material.restitution = 0.8;
        let _ = world.create_sphere_shape(
            ball,
            &shape_def,
            Sphere::new(Vec3::new(0.0, 0.0, 0.0), 0.5),
        );

        for _ in 0..240 {
            world.step(1.0 / 60.0, 4);
        }

        let transform = world.body_transform(ball);
        let v = world.body_linear_velocity(ball);
        let w = world.body_angular_velocity(ball);

        let mut h: u64 = 14_695_981_039_346_656_037;
        h = fnv_f64(h, transform.p.x);
        h = fnv_f64(h, transform.p.y);
        h = fnv_f64(h, transform.p.z);
        h = fnv_f32(h, transform.q.v.x);
        h = fnv_f32(h, transform.q.v.y);
        h = fnv_f32(h, transform.q.v.z);
        h = fnv_f32(h, transform.q.s);
        h = fnv_f32(h, v.x);
        h = fnv_f32(h, v.y);
        h = fnv_f32(h, v.z);
        h = fnv_f32(h, w.x);
        h = fnv_f32(h, w.y);
        h = fnv_f32(h, w.z);

        samples.push(h);
        samples.push((transform.p.y as f32).to_bits() as u64);
        samples.push(v.y.to_bits() as u64);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
