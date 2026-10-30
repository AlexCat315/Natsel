//! Ported from Tests/NatselTests/Collision/Mesh/BlobTests.swift.
//!
//! Complete mesh blobs must hash repeatably, and degenerate or invalid input must be rejected.

use crate::collision::mesh::build::MeshBuildOptions;
use crate::collision::mesh::storage::TriangleMesh;
use crate::math::types::Vec3;

#[test]
fn storage_repeatability() {
    // "complete zeroed mesh blob hashes are repeatable"
    // Swift's `arguments: 0..<3` expands to one case per scenario.
    for scenario in 0..3 {
        fn snapshot(scenario: usize) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let vertices = [
                Vec3::ZERO,
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 0.0, 1.0),
                Vec3::new(1.0, 0.0, 1.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::ZERO,
                Vec3::new(2.0, 0.0, 0.0),
            ];
            let indices: [i32; 21] = [
                0, 2, 1, 1, 2, 3, 0, 1, 4, 5, 2, 1, 0, 0, 1, 0, 1, 5, 0, 1, 6,
            ];
            let mut options = MeshBuildOptions::default();
            options.identify_edges = scenario > 0;
            options.weld_tolerance = if scenario == 2 { 0.001 } else { 0.0 };
            options.clockwise_winding = scenario == 2;
            let materials: [u8; 7] = [0, 1, 2, 3, 4, 5, 6];
            let mesh = TriangleMesh::new(&vertices, &indices, Some(&materials), options)
                .expect("mesh creation failed");
            samples.push(mesh.hash());
            assert_eq!(mesh.byte_count(), if scenario == 2 { 264 } else { 280 });
            assert_eq!(mesh.triangle_count(), 4);
            assert_eq!(mesh.vertex_count(), if scenario == 2 { 6 } else { 7 });
            assert_eq!(mesh.surface_area().to_bits(), 2.0f32.to_bits());
            assert_eq!(
                mesh.degenerate_triangle_count(),
                if scenario == 2 { 1 } else { 2 }
            );
            assert_eq!(mesh.header().material_count, 4);
            let expected_flags: [[u8; 4]; 3] = [[0, 0, 0, 0], [4, 0, 1, 0], [0x44, 0, 0, 0x44]];
            assert_eq!(mesh.edge_flags(), &expected_flags[scenario]);
            assert_eq!(mesh.material_indices(), &[0u8, 1, 2, 3]);
            let again = TriangleMesh::new(&vertices, &indices, Some(&materials), options)
                .expect("mesh creation failed");
            assert_eq!(mesh.hash(), again.hash());
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}

#[test]
fn rejected_inputs() {
    // "invalid and fully degenerate input does not produce geometry"
    let vertices = [
        Vec3::ZERO,
        Vec3::new(1.0, 0.0, 0.0),
        Vec3::new(2.0, 0.0, 0.0),
    ];
    let options = MeshBuildOptions::default();
    assert!(TriangleMesh::new(&vertices, &[0, 1, 2], None, options).is_none());
    assert!(TriangleMesh::new(&vertices, &[0, 1, 3], None, options).is_none());
    assert!(TriangleMesh::new(&vertices, &[-1, 1, 2], None, options).is_none());
    assert!(TriangleMesh::new(&vertices, &[0, 1], None, options).is_none());
}
