//! Ported from Tests/NatselTests/Collision/Mesh/RayTests.swift.
//!
//! Grid storage and 240 scaled ray results must be repeatable.

use crate::collision::mesh::overlap::ray_cast_mesh;
use crate::collision::mesh::procedural::create_grid_mesh;
use crate::math::types::Vec3;
use crate::types::geometry::Mesh;
use crate::types::queries::RayCastInput;

/// Swift's inline FNV loop over a list of words.
fn mix(hash: &mut u64, word: u32) {
    for byte in 0..4 {
        *hash ^= ((word >> (8 * byte)) & 255) as u64;
        *hash = hash.wrapping_mul(1099511628211);
    }
}

#[test]
fn ray_repeatability() {
    // "grid storage and 240 scaled ray results are repeatable"
    // Swift's `arguments: 0..<3` expands to one case per scenario.
    for scenario in 0..3 {
        fn snapshot(scenario: usize) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let data = create_grid_mesh(5, 4, 0.75, 3, true).expect("mesh creation failed");
            samples.push(data.hash());
            assert_eq!(data.byte_count(), 1624);
            let scales = [
                Vec3::ONE,
                Vec3::new(-2.0, 0.5, 1.5),
                Vec3::new(1.0, -1.0, 1.0),
            ];
            let mesh = Mesh::new(Some(data), scales[scenario]);
            let mut hash: u64 = 14695981039346656037;
            for i in 0..80i32 {
                let ray = RayCastInput::new(
                    Vec3::new(
                        (i % 10) as f32 * 0.5 - 2.25,
                        if i < 40 { 2.0 } else { -2.0 },
                        (i % 8) as f32 * 0.5 - 1.75,
                    ),
                    Vec3::new(0.125, if i < 40 { -4.0 } else { 4.0 }, -0.25),
                    if i % 3 == 0 { 0.5 } else { 1.0 },
                );
                let out = ray_cast_mesh(&mesh, ray);
                for word in [
                    out.normal.x.to_bits(),
                    out.normal.y.to_bits(),
                    out.normal.z.to_bits(),
                    out.point.x.to_bits(),
                    out.point.y.to_bits(),
                    out.point.z.to_bits(),
                    out.fraction.to_bits(),
                    out.triangle_index as u32,
                    out.material_index as u32,
                    if out.hit { 1 } else { 0 },
                ] {
                    mix(&mut hash, word);
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
