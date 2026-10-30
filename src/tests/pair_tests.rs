//! Ported from Tests/NatselTests/PairTests.swift.
//!
//! Broad-phase pair generation must produce the same contacts, the same body contact lists, and
//! no duplicates when it runs twice, across two runs.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::containers::hash_set::shape_pair_key;
use crate::math::types::{Pos, Vec3};
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::Sphere;
use crate::types::ids::BodyID;
use crate::types::shape_def::ShapeDef;

fn fnv(mut h: u64, bytes: &[u8]) -> u64 {
    for &b in bytes {
        h ^= b as u64;
        h = h.wrapping_mul(1_099_511_628_211);
    }
    h
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
fn fnv_f32(h: u64, x: f32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[inline]
fn fnv_u64(h: u64, x: u64) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

#[inline]
fn fnv_raw<T>(h: u64, value: &T, n: usize) -> u64 {
    // SAFETY: `value` is a live `T` and the callers pass a length no larger than `size_of::<T>()`,
    // so the slice stays inside the object exactly as Swift's raw-pointer walk did.
    let bytes = unsafe { std::slice::from_raw_parts(value as *const T as *const u8, n) };
    fnv(h, bytes)
}

/// Hash every live contact's identity, filtering, and pair key. Returns `(hash, count)`.
fn contact_hash(world: &World) -> (u64, i32) {
    let mut h: u64 = 14_695_981_039_346_656_037;
    let mut count = 0;
    for c in world.contacts.iter() {
        if c.contact_id == b3::NULL_INDEX {
            continue;
        }
        count += 1;
        let pair_key = shape_pair_key(c.shape_id_a, c.shape_id_b, c.child_index);

        h = fnv_i32(h, c.contact_id);
        h = fnv_i32(h, c.shape_id_a);
        h = fnv_i32(h, c.shape_id_b);
        h = fnv_i32(h, c.child_index);
        h = fnv_i32(h, c.set_index);
        h = fnv_u32(h, c.flags);
        h = fnv_i32(h, c.encoded_body_sim_a);
        h = fnv_i32(h, c.encoded_body_sim_b);
        h = fnv_f32(h, c.rolling_resistance);
        h = fnv_u64(h, pair_key);
    }
    (h, count)
}

/// Hash each live body's contact list head and count.
fn body_hash(world: &World) -> u64 {
    let mut h: u64 = 14_695_981_039_346_656_037;
    for b in world.bodies.iter() {
        if b.id == b3::NULL_INDEX {
            continue;
        }
        h = fnv_i32(h, b.id);
        h = fnv_i32(h, b.head_contact_key);
        h = fnv_i32(h, b.contact_count);
    }
    h
}

#[test]
fn generate_pairs() {
    // "pair generation is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        let ground = world.create_body(&def);

        let mut shape_def = ShapeDef::default();
        shape_def.density = 0.0;
        let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

        let positions = [
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.5, 1.0, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ];
        let mut dynamic_bodies: Vec<BodyID> = Vec::new();
        for p in positions {
            def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::from_vec3(p);
            let body = world.create_body(&def);
            dynamic_bodies.push(body);

            shape_def = ShapeDef::default();
            shape_def.density = 1000.0;
            let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::ZERO, 0.5));
        }

        world.update_broad_phase_pairs();

        let (mut h, count) = contact_hash(&world);
        assert_eq!(count, 5);
        samples.push(h);
        samples.push(body_hash(&world));

        for (i, body_id) in dynamic_bodies.iter().enumerate() {
            let b = world.body(*body_id);
            match i {
                0 => {
                    assert_eq!(b.head_contact_key, 6);
                    assert_eq!(b.contact_count, 3);
                }
                1 => {
                    assert_eq!(b.head_contact_key, 8);
                    assert_eq!(b.contact_count, 3);
                }
                _ => {
                    assert_eq!(b.head_contact_key, 9);
                    assert_eq!(b.contact_count, 2);
                }
            }
        }

        assert_eq!(world.solver_sets[2].contact_indices.len(), 5);
        assert_eq!(world.solver_sets[1].contact_indices.len(), 0);

        // A second update must not create duplicate contacts.
        world.update_broad_phase_pairs();
        let (next_h, count) = contact_hash(&world);
        h = next_h;
        assert_eq!(count, 5);
        samples.push(h);
        samples.push(body_hash(&world));
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
