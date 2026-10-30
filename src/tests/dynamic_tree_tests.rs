//! Ported from Tests/NatselTests/DynamicTreeTests.swift.
//!
//! Tree shape, query visit counts, and save/load round trips must be identical across two runs.

use std::fs;

use crate::collision::dynamic_tree::DynamicTree;
use crate::math::types::{AABB, Vec3};
use crate::types::queries::{BoxCastInput, RayCastInput};

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

#[inline]
fn fnv_f32(h: u64, x: f32) -> u64 {
    fnv(h, &x.to_ne_bytes())
}

/// `fnv` over the first `n` bytes of a value, matching Swift's pointer-plus-length hashing.
#[inline]
fn fnv_raw<T>(h: u64, value: &T, n: usize) -> u64 {
    // SAFETY: `value` is a live `T` and the callers pass a length no larger than `size_of::<T>()`,
    // so the slice stays inside the object exactly as Swift's raw-pointer walk did.
    let bytes = unsafe { std::slice::from_raw_parts(value as *const T as *const u8, n) };
    fnv(h, bytes)
}

/// Hash the whole tree: pool counters, then every live node, parent, and proxy record.
fn tree_hash(tree: &DynamicTree) -> u64 {
    let mut h: u64 = 14_695_981_039_346_656_037;
    h = fnv_i32(h, tree.node_end);
    h = fnv_i32(h, tree.node_capacity);
    h = fnv_i32(h, tree.proxy_count);
    h = fnv_i32(h, tree.proxy_capacity);
    h = fnv_i32(h, tree.pair_free_list);
    h = fnv_i32(h, tree.proxy_free_list);
    h = fnv_u8(h, if tree.dfs_ordered { 1 } else { 0 });

    // Swift hashed 32 bytes per `TreeNode`, 4 per parent, and 24 per proxy straight out of the
    // backing buffers; the Rust structs are `#[repr(C)]` with the same layout.
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

#[test]
fn tree_repeatability() {
    // "tree operations are repeatable"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let mut tree = DynamicTree::new(16);
        for i in 0..20 {
            let x = (i % 5) as f32;
            let y = (i / 5) as f32;
            let box_ = AABB::new(
                Vec3::new(x, y, 0.0),
                Vec3::new(x + 0.9, y + 0.9, 1.0),
            );
            let category: u64 = if (i & 1) != 0 { 1 } else { 2 };
            let _ = tree.create_proxy(box_, category, (100 + i) as u64);
        }

        samples.push(tree_hash(&tree));
        assert_eq!(tree.height(), 5);
        samples.push(tree.area_ratio().to_bits() as u64);
        assert_eq!(tree.proxy_count, 20);

        let query_box = AABB::new(
            Vec3::new(0.5, 0.5, 0.5),
            Vec3::new(2.5, 1.5, 0.5),
        );

        fn run_query(
            tree: &DynamicTree,
            query_box: AABB,
            mask: u64,
            require_all: bool,
        ) -> (u64, i32, crate::collision::dynamic_tree::TreeStats) {
            let mut h: u64 = 14_695_981_039_346_656_037;
            let mut n = 0;
            let stats = tree.query(query_box, mask, require_all, |proxy_id, user_data| {
                h = fnv_i32(h, proxy_id);
                h = fnv_u64(h, user_data);
                n += 1;
                true
            });
            (h, n, stats)
        }

        let (mut qh, qn, q_stats) = run_query(&tree, query_box, u64::MAX, false);
        samples.push(qh);
        assert_eq!(qn, 6);
        assert_eq!(q_stats.node_visits, 10);
        assert_eq!(q_stats.leaf_visits, 6);

        let (qh_next, qn_next, _) = run_query(&tree, query_box, 1, false);
        qh = qh_next;
        samples.push(qh);
        assert_eq!(qn_next, 3);

        let (qh_next, qn_next, _) = run_query(&tree, query_box, 1, true);
        qh = qh_next;
        samples.push(qh);
        assert_eq!(qn_next, 3);

        // Ray cast
        let ray_input = RayCastInput::new(Vec3::new(-1.0, 0.45, 0.5), Vec3::new(10.0, 0.0, 0.0), 1.0);

        let mut h: u64 = 14_695_981_039_346_656_037;
        let mut n = 0;
        let ray_stats = tree.ray_cast(ray_input, u64::MAX, false, |input, proxy_id, user_data| {
            h = fnv_i32(h, proxy_id);
            h = fnv_u64(h, user_data);
            h = fnv_f32(h, input.max_fraction);
            n += 1;
            input.max_fraction
        });
        samples.push(h);
        assert_eq!(n, 5);
        assert_eq!(ray_stats.node_visits, 6);
        assert_eq!(ray_stats.leaf_visits, 5);

        // Box cast
        let box_input = BoxCastInput {
            box_: AABB::new(Vec3::new(-0.1, -0.1, 0.4), Vec3::new(0.1, 0.1, 0.6)),
            translation: Vec3::new(10.0, 0.0, 0.0),
            max_fraction: 1.0,
        };

        h = 14_695_981_039_346_656_037;
        n = 0;
        let box_stats = tree.box_cast(box_input, u64::MAX, false, |input, proxy_id, user_data| {
            h = fnv_i32(h, proxy_id);
            h = fnv_u64(h, user_data);
            h = fnv_f32(h, input.max_fraction);
            n += 1;
            input.max_fraction
        });
        samples.push(h);
        assert_eq!(n, 5);
        assert_eq!(box_stats.node_visits, 6);

        // Closest query
        h = 14_695_981_039_346_656_037;
        n = 0;
        let (closest_stats, closest_min_dist_sqr) = tree.query_closest(
            Vec3::new(0.45, 0.45, 0.5),
            u64::MAX,
            false,
            1e30,
            |min_sqr, proxy_id, user_data| {
                h = fnv_i32(h, proxy_id);
                h = fnv_u64(h, user_data);
                h = fnv_f32(h, min_sqr);
                n += 1;
                min_sqr
            },
        );
        samples.push(h);
        assert_eq!(n, 20);
        assert_eq!(closest_stats.node_visits, 38);
        assert_eq!(closest_stats.leaf_visits, 20);
        samples.push(closest_min_dist_sqr.to_bits() as u64);

        // Edits
        tree.move_proxy(3, AABB::new(Vec3::new(10.0, 10.0, 0.0), Vec3::new(11.0, 11.0, 1.0)));
        tree.destroy_proxy(5);
        let _ = tree.create_proxy(
            AABB::new(Vec3::new(5.0, 5.0, 0.0), Vec3::new(5.5, 5.5, 1.0)),
            4,
            999,
        );
        samples.push(tree_hash(&tree));

        let rebuilt = tree.rebuild(false);
        assert_eq!(rebuilt, 1);
        samples.push(tree_hash(&tree));
        assert_eq!(tree.height(), 7);

        // Save and load
        let path = std::env::temp_dir().join("natsel_tree.bin");
        let path = path.to_string_lossy().into_owned();
        tree.save(&path);
        let loaded = DynamicTree::load(&path, 1.0).unwrap();
        samples.push(tree_hash(&loaded));
        let _ = fs::remove_file(&path);
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}
