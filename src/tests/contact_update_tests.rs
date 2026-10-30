//! Ported from Tests/NatselTests/ContactUpdateTests.swift.
//!
//! Narrow-phase manifolds, island linking, and impulse recycling by feature id must be
//! byte-identical across two runs.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::types::{Pos, Vec3};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::contact_store::ContactStore;
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
fn fnv_u8(h: u64, x: u8) -> u64 {
    fnv(h, &[x])
}

/// `fnv` over the first `n` bytes of a value, matching Swift's pointer-plus-length hashing.
#[inline]
fn fnv_raw<T>(h: u64, value: &T, n: usize) -> u64 {
    // SAFETY: `value` is a live `T` and the callers pass a length no larger than `size_of::<T>()`,
    // so the slice stays inside the object exactly as Swift's raw-pointer walk did.
    let bytes = unsafe { std::slice::from_raw_parts(value as *const T as *const u8, n) };
    fnv(h, bytes)
}

/// Hash every live contact's scalars and manifold points. Returns `(hash, touching, count)`.
fn hash_manifolds(world: &World) -> (u64, i32, i32) {
    let mut h: u64 = 14_695_981_039_346_656_037;
    let mut touch_count = 0;
    let mut contact_count = 0;

    for c in world.contacts.iter() {
        if c.contact_id == b3::NULL_INDEX {
            continue;
        }
        contact_count += 1;

        h = fnv_u32(h, c.flags);
        h = fnv_i32(h, c.manifold_count);
        h = fnv_f32(h, c.friction);
        h = fnv_f32(h, c.restitution);
        h = fnv_f32(h, c.rolling_resistance);
        h = fnv_raw(h, &c.tangent_velocity, 12);

        let manifolds = c.manifolds();
        for m in 0..c.manifold_count as usize {
            let manifold = manifolds.get(m);
            h = fnv_raw(h, &manifold.normal, 12);
            h = fnv_i32(h, manifold.point_count);

            for p in 0..manifold.point_count as usize {
                let mp = manifold.points[p];
                h = fnv_raw(h, &mp.anchor_a, 12);
                h = fnv_raw(h, &mp.anchor_b, 12);
                h = fnv_f32(h, mp.separation);
                h = fnv_u32(h, mp.feature_id);
                h = fnv_f32(h, mp.normal_impulse);
                h = fnv_u8(h, if mp.persisted { 1 } else { 0 });
            }
        }

        if (c.flags & ContactFlags::SIM_TOUCHING.0) != 0 {
            touch_count += 1;
        }
    }

    (h, touch_count, contact_count)
}

/// Update every contact in the awake set and count the touching ones.
fn run_updates(world: &mut World) -> i32 {
    // Swift wrote through `inout ContactStore(&world.contacts)` while calling `updateContact`,
    // relying on unchecked exclusivity. The store is rebuilt over the live contact buffer through
    // a raw pointer, exactly as `collide_pass.rs` does, so the borrow checker sees the same
    // discipline Swift had.
    let contact_ptr = world.contacts.as_mut_ptr();
    let contact_count = world.contacts.len();
    let contact_ids = world.solver_sets[2].contact_indices.clone();

    let mut store = ContactStore::from_buffer(
        // SAFETY: `contacts` is not resized by the narrow phase and does not move while the store
        // lives, which is the invariant Swift's `withUnsafeMutableBufferPointer` gave it.
        unsafe { std::slice::from_raw_parts_mut(contact_ptr, contact_count) },
    );

    let mut touch_count = 0;
    for cid in contact_ids {
        let contact = store.get(cid);
        let shape_a = &world.shapes[contact.shape_id_a as usize];
        let shape_b = &world.shapes[contact.shape_id_b as usize];
        let body_a = world.make_body_id(shape_a.body_id);
        let body_b = world.make_body_id(shape_b.body_id);
        let _ = world.body_index(body_a);
        let _ = world.body_index(body_b);

        let sim_a = world.body_sim_for(body_a);
        let sim_b = world.body_sim_for(body_b);

        let touching = world.update_contact_default_scratch(
            cid,
            sim_a.local_center,
            sim_a.transform,
            sim_b.local_center,
            sim_b.transform,
            false,
            &mut store,
        );
        if touching {
            touch_count += 1;
        }
    }
    touch_count
}

#[test]
fn manifolds() {
    // "convex manifolds are repeatable"
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
            Vec3::new(0.0, 0.9, 0.0),
            Vec3::new(0.6, 0.9, 0.0),
            Vec3::new(0.0, 2.0, 0.0),
        ];
        for p in positions {
            def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            def.position = Pos::from_vec3(p);
            let body = world.create_body(&def);

            shape_def = ShapeDef::default();
            shape_def.density = 1000.0;
            let _ = world.create_sphere_shape(body, &shape_def, Sphere::new(Vec3::ZERO, 0.5));
        }

        // Dynamic hull box sitting on the ground and touching sphere A.
        def = BodyDef::default();
        def.type_ = BodyType::DynamicBody;
        def.position = Pos::new(-0.9, 0.9, 0.0);
        let box_body = world.create_body(&def);
        shape_def = ShapeDef::default();
        shape_def.density = 1000.0;
        let _ = world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));

        world.update_broad_phase_pairs();

        let touching = run_updates(&mut world);
        assert_eq!(touching, 5);

        let (mut h, touch_count, contact_count) = hash_manifolds(&world);
        assert_eq!(touch_count, 5);
        assert_eq!(contact_count, 8);
        samples.push(h);

        // Link touching contacts into islands in awake set order.
        let contact_ids = world.solver_sets[2].contact_indices.clone();
        for cid in contact_ids {
            let flags = world.contacts[cid as usize].flags;
            if (flags & ContactFlags::SIM_TOUCHING.0) != 0 {
                world.link_contact(cid);
            }
        }

        let mut ih: u64 = 14_695_981_039_346_656_037;
        for island in world.islands.iter() {
            if island.island_id == b3::NULL_INDEX {
                continue;
            }
            ih = fnv_i32(ih, island.island_id);
            ih = fnv_i32(ih, island.set_index);
            for &body_id in island.bodies.iter() {
                ih = fnv_i32(ih, body_id);
            }
            for link in island.contacts.iter() {
                ih = fnv_i32(ih, link.contact_id);
            }
        }
        for b in world.bodies.iter() {
            if b.id == b3::NULL_INDEX {
                continue;
            }
            ih = fnv_i32(ih, b.island_id);
            ih = fnv_i32(ih, b.island_index);
        }
        samples.push(ih);

        // Inject impulses to test recycling by feature id.
        for i in 0..world.contacts.len() {
            let c = world.contacts[i];
            if c.contact_id == b3::NULL_INDEX {
                continue;
            }
            let manifolds = c.manifolds();
            for m in 0..c.manifold_count as usize {
                let manifold = manifolds.get_mut(m);
                for p in 0..manifold.point_count as usize {
                    manifold.points[p].normal_impulse = 10.0 + p as f32;
                }
            }
            world.contacts[i] = c;
        }

        let touching = run_updates(&mut world);
        assert_eq!(touching, 5);
        let (next_h, _, _) = hash_manifolds(&world);
        h = next_h;
        samples.push(h);

        // A further update carries the recycled impulses.
        let touching = run_updates(&mut world);
        assert_eq!(touching, 5);
        let (next_h, _, _) = hash_manifolds(&world);
        h = next_h;
        samples.push(h);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
