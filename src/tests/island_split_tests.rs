//! Ported from Tests/NatselTests/IslandSplitTests.swift.
//!
//! Splitting an island after unlinking a contact must relabel the bodies identically across two
//! runs.

use crate::base::constants as b3;
use crate::collision::hull::make_box_hull;
use crate::math::types::Pos;
use crate::simulation::World;
use crate::types::body_def::{BodyDef, BodyType};
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

/// Hash every live island's id, set, body ids, and contact ids.
fn island_hash(world: &World) -> u64 {
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
    ih
}

#[test]
fn split_island() {
    // "union-find split is repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut world = World::default();
        world.enable_sleep = false;

        let mut def = BodyDef::default();
        def.type_ = BodyType::StaticBody;
        let ground = world.create_body(&def);
        let mut shape_def = ShapeDef::default();
        let _ = world.create_hull_shape(ground, &shape_def, &make_box_hull(10.0, 0.5, 10.0));

        let mut boxes: Vec<BodyID> = Vec::new();
        for i in 0..2 {
            def = BodyDef::default();
            def.type_ = BodyType::DynamicBody;
            // Swift's `Pos(0, 0.55 + Float(i) * 1.01, 0)` computes in `Float` and widens to `Double`.
            def.position = Pos::new(0.0, (0.55 + i as f64 * 1.01) as f64, 0.0);
            let box_body = world.create_body(&def);
            shape_def = ShapeDef::default();
            shape_def.density = 1000.0;
            let _ =
                world.create_hull_shape(box_body, &shape_def, &make_box_hull(0.5, 0.5, 0.5));
            boxes.push(box_body);
        }

        for _ in 0..30 {
            world.step(1.0 / 60.0, 4);
        }

        assert_eq!(world.islands.len(), 2);
        let before = island_hash(&world);
        samples.push(before);

        // Unlink the first touching contact from its island.
        let mut target_contact: i32 = b3::NULL_INDEX;
        let mut target_island: i32 = b3::NULL_INDEX;
        for c in world.contacts.iter() {
            if c.contact_id != b3::NULL_INDEX {
                if c.island_id != b3::NULL_INDEX {
                    target_contact = c.contact_id;
                    target_island = c.island_id;
                    break;
                }
            }
        }
        assert_eq!(target_contact, 0);
        assert_eq!(target_island, 0);

        world.unlink_contact(target_contact);
        world.split_island(target_island);

        let mut h: u64 = island_hash(&world);
        for b in world.bodies.iter() {
            if b.id == b3::NULL_INDEX {
                continue;
            }
            h = fnv_i32(h, b.island_id);
            h = fnv_i32(h, b.island_index);
        }

        samples.push(h);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
