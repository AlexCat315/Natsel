//! Ported from Tests/NatselTests/Collision/HeightField/EncodingTests.swift.
//!
//! The shared height-field fixtures plus the storage-repeatability cases.

use crate::collision::height_field::factories::{create_grid_height_field, create_wave_height_field};
use crate::collision::height_field::quantization::HeightField;
use crate::math::types::Vec3;
use crate::tests::collision::collision_digest::CollisionDigest;
use crate::types::geometry::HeightFieldDef;

/// Swift's `makeTestHeightField(_ scenario:)`.
///
/// Scenarios 0-1 are flat grids (1 has holes), 2-3 are waves (3 has holes), and 4-5 come from an
/// explicit definition with the winding flipped in 5.
pub fn make_test_height_field(scenario: i32) -> Option<HeightField> {
    let scale = Vec3::new(0.75, 1.25, 0.5);
    if scenario < 2 {
        return create_grid_height_field(6, 7, scale, scenario == 1);
    }
    if scenario < 4 {
        return create_wave_height_field(6, 7, scale, 0.2, 0.3, scenario == 3);
    }
    HeightField::new(&make_test_height_definition(scenario == 5))
}

/// Swift's `makeTestHeightDefinition(clockwise:)`.
pub fn make_test_height_definition(clockwise: bool) -> HeightFieldDef {
    let scale = Vec3::new(0.75, 1.25, 0.5);
    let mut definition = HeightFieldDef::default();
    definition.heights = (0..42)
        .map(|i: i32| ((i * 17) % 29) as f32 * 0.125 - 1.5)
        .collect();
    definition.material_indices = (0..30)
        .map(|i: i32| if i % 11 == 0 { 255 } else { (i % 3) as u8 })
        .collect();
    definition.scale = scale;
    definition.count_x = 7;
    definition.count_z = 6;
    definition.global_minimum_height = -1.0;
    definition.global_maximum_height = 1.0;
    definition.clockwise_winding = clockwise;
    definition
}

#[test]
fn encoding() {
    // "quantization, holes, adjacency, winding, and blob hashes are repeatable"
    // Swift's `arguments: 0..<6` expands to one case per scenario; each is asserted separately so a
    // failure names the scenario that broke.
    for scenario in 0..6 {
        fn snapshot(scenario: i32) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let field = make_test_height_field(scenario).unwrap();
            assert_eq!(field.byte_count(), 280);
            samples.push(field.hash());

            let mut digest = CollisionDigest::new();
            for i in 0..60i32 {
                if field.material(i) == 255 {
                    continue;
                }
                let triangle = field.triangle(i);
                for j in 0..3 {
                    digest.add_vec3(triangle.vertices[j]);
                }
                digest.add_u32(triangle.i1 as u32);
                digest.add_u32(triangle.i2 as u32);
                digest.add_u32(triangle.i3 as u32);
                digest.add_u32(triangle.flags as u32);
            }
            samples.push(digest.value);
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}
