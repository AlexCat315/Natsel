//! Ported from Tests/NatselTests/Collision/Mesh/GeneratorTests.swift.
//!
//! Box, hollow box, and platform generators must produce repeatable storage hashes.

use crate::collision::mesh::procedural::{create_box_mesh, create_hollow_box_mesh, create_platform_mesh};
use crate::math::types::Vec3;

#[test]
fn primitives() {
    // "box, hollow box, and platform produce repeatable storage hashes"
    // Swift's `arguments: 0..<3` expands to one case per scenario.
    for scenario in 0..3 {
        fn snapshot(scenario: usize) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let center = Vec3::new(0.125, -0.25, 0.5);
            let extent = Vec3::new(1.25, 0.75, 2.0);
            let geometry = match scenario {
                0 => create_box_mesh(center, extent, true),
                1 => create_hollow_box_mesh(center, extent),
                _ => create_platform_mesh(center, 1.5, 2.0, 3.0),
            };
            let mesh = geometry.expect("mesh creation failed");
            samples.push(mesh.hash());
            assert_eq!(mesh.byte_count(), 656);
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}
