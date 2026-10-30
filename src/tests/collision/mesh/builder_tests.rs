//! Ported from Tests/NatselTests/Collision/Mesh/BuilderTests.swift.
//!
//! SAH, median, and degenerate centroid fallback splits must produce repeatable trees and
//! permutations, and welding must preserve negative cells, neighbour priority, tolerance, and
//! non-transitive duplicates.

use crate::collision::mesh::tree_builder::{MeshPrimitive, MeshTreeBuild};
use crate::collision::mesh::welding::weld_mesh_vertices;
use crate::math::types::{AABB, Vec3};

/// Swift's `mix` helper: fold a `UInt32` into the FNV digest byte by byte.
fn mix(hash: &mut u64, word: u32) {
    for i in 0..4 {
        *hash ^= ((word >> (8 * i)) & 255) as u64;
        *hash = hash.wrapping_mul(1099511628211);
    }
}

#[test]
fn tree_repeatability() {
    // "SAH, median, and degenerate centroid fallback produce repeatable trees and permutations"
    // Swift's `arguments: 0..<4` expands to one case per scenario.
    for scenario in 0..4 {
        fn snapshot(scenario: usize) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let counts = [37usize, 37, 7, 17];
            let node_counts = [21usize, 25, 1, 5];
            let heights = [5usize, 5, 0, 3];
            let mut primitives: Vec<MeshPrimitive> = (0..counts[scenario])
                .map(|i| {
                    let center = if scenario < 2 {
                        Vec3::new(
                            ((i * 17) % 19) as f32 * 0.25 - 2.0,
                            ((i * 7) % 13) as f32 * 0.5,
                            ((i * 3) % 11) as f32 * 0.125,
                        )
                    } else {
                        Vec3::ZERO
                    };
                    let extent = Vec3::new(0.125, 0.25, 0.0625);
                    MeshPrimitive {
                        bounds: AABB::new(center - extent, center + extent),
                        center,
                        triangle_index: i,
                    }
                })
                .collect();
            // Swift's `MeshTreeBuild(primitives:useMedianSplit: scenario == 1)`; the Rust builder
            // takes only the primitives, so the median-split variant has no call-site spelling yet.
            // The node-count and height assertions below still pin scenario 1's result, so the gap
            // surfaces as a failure rather than being silently dropped.
            let tree = MeshTreeBuild::new(&mut primitives);
            assert_eq!(tree.nodes.len(), node_counts[scenario]);
            assert_eq!(tree.height, heights[scenario]);
            let mut digest: u64 = 14695981039346656037;
            for primitive in primitives.iter() {
                mix(&mut digest, primitive.triangle_index as u32);
            }
            for node in tree.nodes.iter() {
                mix(&mut digest, node.lower_bound.x.to_bits());
                mix(&mut digest, node.lower_bound.y.to_bits());
                mix(&mut digest, node.lower_bound.z.to_bits());
                mix(&mut digest, node.data);
                mix(&mut digest, node.upper_bound.x.to_bits());
                mix(&mut digest, node.upper_bound.y.to_bits());
                mix(&mut digest, node.upper_bound.z.to_bits());
                mix(&mut digest, node.triangle_offset);
            }
            samples.push(digest);
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}

#[test]
fn welding() {
    // "welding preserves negative cells, neighbor priority, tolerance, and non-transitive duplicates"
    let source = [
        Vec3::new(-0.375, 0.0, 0.0),
        Vec3::new(0.375, 0.0, 0.0),
        Vec3::ZERO,
        Vec3::new(-0.75, 0.0, 0.0),
        Vec3::new(0.75, 0.0, 0.0),
        Vec3::new(0.125, 0.5, -0.5),
        Vec3::new(0.25, 0.625, -0.625),
        Vec3::new(0.375, 0.75, -0.75),
    ];
    let indices: Vec<i32> = (0..8).collect();
    let (vertices, welded_indices) = weld_mesh_vertices(&source, &indices, 0.5);
    assert_eq!(vertices, vec![source[0], source[1], source[6]]);
    assert_eq!(welded_indices, vec![0, 1, 0, 0, 1, 0, 2, 2]);
}
