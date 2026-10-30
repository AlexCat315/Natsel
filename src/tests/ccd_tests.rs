//! Ported from Tests/NatselTests/CcdTests.swift.
//!
//! A body moving 200 m/s at a thin wall must not tunnel through it, with or without the bullet
//! flag, and the resulting state must be identical across two runs.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
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

/// `fnv` over the first `n` bytes of a value, matching Swift's pointer-plus-length hashing.
#[inline]
fn fnv_raw<T>(h: u64, value: &T, n: usize) -> u64 {
    // SAFETY: `value` is a live `T` and the callers pass a length no larger than `size_of::<T>()`,
    // so the slice stays inside the object exactly as Swift's raw-pointer walk did.
    let bytes = unsafe { std::slice::from_raw_parts(value as *const T as *const u8, n) };
    fnv(h, bytes)
}

/// Runs the scene and returns the state hash plus the number of live contacts.
fn run(bullet: bool) -> (u64, usize) {
    let mut world = World::default();
    world.enable_sleep = false;

    let mut def = BodyDef::default();
    def.type_ = BodyType::StaticBody;
    let wall = world.create_body(&def);
    let mut shape_def = ShapeDef::default();
    let _ = world.create_hull_shape(wall, &shape_def, &make_box_hull(0.1, 2.0, 2.0));

    def = BodyDef::default();
    def.type_ = BodyType::DynamicBody;
    def.position = Pos::new(-5.0, 0.0, 0.0);
    def.linear_velocity = Vec3::new(200.0, 0.0, 0.0);
    def.is_bullet = bullet;
    let body = world.create_body(&def);
    shape_def = ShapeDef::default();
    shape_def.density = 1000.0;
    let _ = world.create_hull_shape(body, &shape_def, &make_box_hull(0.2, 0.2, 0.2));

    for _ in 0..30 {
        world.step(1.0 / 60.0, 1);
    }

    let transform = world.body_transform(body);
    let v = world.body_linear_velocity(body);
    let av = world.body_angular_velocity(body);
    let flags = world.body(body).flags;
    assert!(
        transform.p.x < 0.0,
        "the fast body must remain on the approach side of the wall"
    );
    assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite());
    assert!(av.x.is_finite() && av.y.is_finite() && av.z.is_finite());

    let mut h: u64 = 14_695_981_039_346_656_037;
    // `MemoryLayout<Pos>.size` is 24 for the three `f64` components; `Quat` and the two `Vec3`s
    // are 16 and 12 bytes.
    h = fnv_raw(h, &transform.p, 24);
    h = fnv_raw(h, &transform.q, 16);
    h = fnv_raw(h, &v, 12);
    h = fnv_raw(h, &av, 12);
    h = fnv_raw(h, &flags, 4);

    let mut count = 0;
    for c in world.contacts.iter() {
        if c.contact_id != b3::NULL_INDEX {
            count += 1;
        }
    }
    (h, count)
}

#[test]
fn bullet() {
    // "bullet does not tunnel"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let (h, count) = run(true);
        assert_eq!(count, 0);
        samples.push(h);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn fast_body() {
    // "non-bullet fast body does not tunnel"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let (h, count) = run(false);
        assert_eq!(count, 0);
        samples.push(h);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
