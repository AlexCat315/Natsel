//! Ported from Tests/NatselTests/Simulation/SpatialPairsTests.swift.

use crate::math::spatial_pairs::{BoundsPair, overlapping_bounds_pairs};
use crate::math::types::Pos;
use crate::math::world_aabb::WorldAABB;

// "sweep agrees with exhaustive overlap at distant coordinates"
#[test]
fn oracle() {
    let mut seed: u64 = 17;
    let mut random = move || -> f64 {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 40) as f64 / (1u64 << 24) as f64
    };

    let mut bounds: Vec<WorldAABB> = Vec::with_capacity(400);
    for _ in 0..400 {
        let p = Pos::new(
            1.0e12 + random() * 20.0,
            -1.0e12 + random() * 20.0,
            random() * 20.0,
        );
        let size = random() * 3.0;
        bounds.push(WorldAABB::new(
            p,
            Pos::new(p.x + size, p.y + size, p.z + size),
        ));
    }

    let mut expected: Vec<BoundsPair> = Vec::new();
    for a in 0..bounds.len() {
        for b in (a + 1)..bounds.len() {
            if bounds[a].overlaps(&bounds[b]) {
                expected.push(BoundsPair { a, b });
            }
        }
    }

    assert_eq!(overlapping_bounds_pairs(&bounds).pairs, expected);
}

// "separated objects avoid quadratic candidate work"
fn sparse_axis(axis: i32) {
    let mut bounds: Vec<WorldAABB> = Vec::with_capacity(5000);
    for i in 0..5000 {
        let d = i as f64 * 4.0;
        let p = Pos::new(
            1.0e12 + if axis == 0 { d } else { 0.0 },
            if axis == 1 { d } else { 0.0 },
            if axis == 2 { d } else { 0.0 },
        );
        bounds.push(WorldAABB::new(
            p,
            Pos::new(p.x + 1.0, p.y + 1.0, p.z + 1.0),
        ));
    }
    let result = overlapping_bounds_pairs(&bounds);
    assert!(result.pairs.is_empty());
    assert_eq!(result.candidate_count, 0);
}

#[test]
fn sparse_0() {
    sparse_axis(0);
}

#[test]
fn sparse_1() {
    sparse_axis(1);
}

#[test]
fn sparse_2() {
    sparse_axis(2);
}

// "touching endpoints and equal sort keys retain every pair"
#[test]
fn ties() {
    let bounds = [
        WorldAABB::new(Pos::ZERO, Pos::new(1.0, 1.0, 1.0)),
        WorldAABB::new(Pos::new(1.0, 0.0, 0.0), Pos::new(2.0, 1.0, 1.0)),
        WorldAABB::new(Pos::ZERO, Pos::new(1.0, 1.0, 1.0)),
    ];
    assert_eq!(
        overlapping_bounds_pairs(&bounds).pairs,
        vec![
            BoundsPair { a: 0, b: 1 },
            BoundsPair { a: 0, b: 2 },
            BoundsPair { a: 1, b: 2 },
        ]
    );
    assert!(overlapping_bounds_pairs(&[]).pairs.is_empty());
}
