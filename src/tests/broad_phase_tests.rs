//! Ported from Tests/NatselTests/BroadPhaseTests.swift.
//!
//! Creating, moving, and destroying proxies must produce identical tree state and identical query
//! results across two runs of the same sequence.

use crate::math::types::{AABB, Vec3};
use crate::simulation::broad_phase::BroadPhase;
use crate::types::body_def::BodyType;
use crate::types::world_def::Capacity;

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
fn fnv_u8(h: u64, x: u8) -> u64 {
    fnv(h, &[x])
}

#[inline]
fn fnv_u64(h: u64, x: u64) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

/// Hash the whole tree: the pool counters, then every live node, parent, and proxy record, in
/// the same order and at the same widths Swift hashed them.
fn tree_hash(tree: &crate::collision::dynamic_tree::DynamicTree) -> u64 {
    let mut h: u64 = 14_695_981_039_346_656_037;
    h = fnv_i32(h, tree.node_end);
    h = fnv_i32(h, tree.node_capacity);
    h = fnv_i32(h, tree.proxy_count);
    h = fnv_i32(h, tree.proxy_capacity);
    h = fnv_i32(h, tree.pair_free_list);
    h = fnv_i32(h, tree.proxy_free_list);
    h = fnv_u8(h, if tree.dfs_ordered { 1 } else { 0 });

    // Swift hashed 32 bytes per `TreeNode` and 4 bytes per parent straight out of the backing
    // buffers; the Rust structs are `#[repr(C)]` with the same layout, so their byte images match.
    for i in 0..tree.node_end as usize {
        h = fnv_raw(h, &tree.nodes[i], 32);
    }
    for i in 0..tree.node_end as usize {
        h = fnv_i32(h, tree.parents[i]);
    }
    for i in 0..tree.proxy_capacity as usize {
        h = fnv_raw(h, &tree.proxies[i], 24);
    }
    h
}

/// `fnv` over the first `n` bytes of a value, matching Swift's pointer-plus-length hashing.
#[inline]
fn fnv_raw<T>(h: u64, value: &T, n: usize) -> u64 {
    // SAFETY: `value` is a live `T`; the caller passes `n` no larger than `size_of::<T>()`, so the
    // slice stays inside the object exactly as Swift's `buffer.baseAddress + i` walk did.
    let bytes = unsafe { std::slice::from_raw_parts(value as *const T as *const u8, n) };
    fnv(h, bytes)
}

#[test]
fn proxies() {
    // "proxies are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut capacity = Capacity::default();
        capacity.static_shape_count = 8;
        capacity.dynamic_shape_count = 8;
        capacity.contact_count = 16;

        let mut bp = BroadPhase::new(capacity);

        let mut keys: Vec<i32> = Vec::new();
        for i in 0..4 {
            let box_ = AABB::new(
                Vec3::new(i as f32, 0.0, 0.0),
                Vec3::new(i as f32 + 0.5, 1.0, 1.0),
            );
            keys.push(bp.create_proxy(box_, BodyType::StaticBody, 1, i as i32, false));
        }
        for i in 0..2 {
            let box_ = AABB::new(
                Vec3::new(i as f32, 2.0, 0.0),
                Vec3::new(i as f32 + 0.5, 3.0, 1.0),
            );
            keys.push(bp.create_proxy(
                box_,
                BodyType::KinematicBody,
                1,
                10 + i as i32,
                false,
            ));
        }
        for i in 0..6 {
            let box_ = AABB::new(
                Vec3::new(i as f32 * 0.5, 4.0, 0.0),
                Vec3::new(i as f32 * 0.5 + 0.4, 4.4, 1.0),
            );
            keys.push(bp.create_proxy(
                box_,
                BodyType::DynamicBody,
                2,
                20 + i as i32,
                false,
            ));
        }

        assert_eq!(keys, vec![0, 4, 8, 12, 1, 5, 2, 6, 10, 14, 18, 22]);
        assert_eq!(bp.get_shape_index(keys[0]), 0);
        assert_eq!(bp.get_shape_index(keys[6]), 20);
        assert_eq!(bp.get_shape_index(keys[11]), 25);

        assert!(!bp.test_overlap(keys[0], keys[1]));
        assert!(!bp.test_overlap(keys[0], keys[6]));
        assert!(!bp.test_overlap(keys[6], keys[7]));

        bp.move_proxy(
            keys[8],
            AABB::new(Vec3::new(0.0, 4.0, 0.0), Vec3::new(1.0, 5.0, 1.0)),
        );
        bp.destroy_proxy(keys[7]);

        assert!(bp.test_overlap(keys[8], keys[6]));
        assert!(!bp.test_overlap(keys[8], keys[9]));

        samples.push(tree_hash(&bp.trees[0]));
        samples.push(tree_hash(&bp.trees[1]));
        samples.push(tree_hash(&bp.trees[2]));

        let query_box = AABB::new(Vec3::new(0.0, 3.8, 0.0), Vec3::new(2.0, 5.0, 1.0));
        let mut h: u64 = 14_695_981_039_346_656_037;
        let mut n = 0;
        let stats = bp.trees[2].query(query_box, u64::MAX, false, |proxy_id, user_data| {
            h = fnv_i32(h, proxy_id);
            h = fnv_u64(h, user_data);
            n += 1;
            true
        });
        samples.push(h);
        assert_eq!(n, 4);
        assert_eq!(stats.node_visits, 4);
        assert_eq!(stats.leaf_visits, 4);

        bp.validate();
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
