//! Ported from Tests/NatselTests/Collision/Mesh/ReductionTests.swift.
//!
//! Deepest, collinear, and area-based manifold-point selection must preserve a stable order.

use std::collections::HashSet;

use crate::math::types::Vec2;
use crate::simulation::mesh_contact::reduction::{cull_mesh_points, MeshPatchPoint};

#[test]
fn reduction() {
    // "deepest, collinear, and area-based selection preserve stable order"
    // Swift's `arguments: 0..<3` expands to one case per scenario.
    for scenario in 0..3 {
        fn snapshot(scenario: usize) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let mut hash: u64 = 14695981039346656037;
            let mut word = |value: u32| {
                for byte in 0..4 {
                    hash ^= ((value >> (8 * byte)) & 255) as u64;
                    hash = hash.wrapping_mul(1099511628211);
                }
            };
            for trial in 0..100usize {
                let points: Vec<MeshPatchPoint> = (0..(trial % 24 + 1))
                    .map(|i| {
                        let mut x = ((i * 17 + trial * 3) % 19) as f32 - 9.0;
                        let mut y = ((i * 7 + trial * 5) % 13) as f32 - 6.0;
                        if scenario == 0 {
                            x *= 0.00001;
                            y *= 0.00001;
                        }
                        if scenario == 1 {
                            y = 0.0;
                        }
                        MeshPatchPoint {
                            p: Vec2::new(x, y),
                            separation: ((i * 3 + trial) % 11) as f32 * -0.002,
                            original_index: i,
                        }
                    })
                    .collect();
                let selected = cull_mesh_points(&points);
                assert!(!selected.is_empty() && selected.len() <= 4);
                let unique: HashSet<usize> = selected.iter().map(|p| p.original_index).collect();
                assert_eq!(unique.len(), selected.len());
                let selected_min = selected
                    .iter()
                    .map(|p| p.separation)
                    .fold(f32::INFINITY, f32::min);
                let all_min = points
                    .iter()
                    .map(|p| p.separation)
                    .fold(f32::INFINITY, f32::min);
                assert_eq!(selected_min, all_min);
                word(selected.len() as u32);
                for point in selected.iter() {
                    word(point.p.x.to_bits());
                    word(point.p.y.to_bits());
                    word(point.separation.to_bits());
                    word(point.original_index as u32);
                }
            }
            samples.push(hash);
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}
