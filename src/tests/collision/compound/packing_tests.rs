//! Ported from Tests/NatselTests/Collision/Compound/PackingTests.swift.
//!
//! Packing, geometry sharing, materials, and the rebuilt tree must produce repeatable storage, and
//! invalid definitions or damaged headers must be rejected.

use crate::base::hash::hash64_non_zero;
use crate::collision::compound::compound::{Compound, CompoundChildGeometry};
use crate::collision::compound::packing::PackedHullInstance;
use crate::tests::collision::compound::fixture::make_test_compound;
use crate::types::geometry::CompoundDef;

#[test]
fn packing() {
    // "packing, sharing, materials, and rebuilt tree produce repeatable storage"
    // Swift's `arguments: 0..<6` expands to one case per scenario.
    for scenario in 0..6usize {
        fn snapshot(scenario: usize) -> Vec<u64> {
            let mut samples: Vec<u64> = Vec::new();
            let compound = make_test_compound(scenario as i32).unwrap();
            let sizes = [224usize, 368, 1072, 1000, 2344, 4480];
            let bytes = compound.serialized_bytes();
            assert_eq!(bytes.len(), sizes[scenario]);
            let hash = hash64_non_zero(&bytes);
            samples.push(hash);
            let header = compound.header();
            assert_eq!(header.tree_height, [0, 1, 1, 1, 6, 7][scenario]);
            assert_eq!(header.shared_hull_count, [0, 0, 1, 0, 1, 2][scenario]);
            assert_eq!(header.shared_mesh_count, [0, 0, 0, 1, 1, 1][scenario]);
            let loaded = Compound::from_serialized_bytes(&bytes).unwrap();
            assert_eq!(loaded.serialized_bytes(), bytes);
            assert_eq!(loaded.child_count(), compound.child_count());
            assert_eq!(loaded.materials(), compound.materials());
            samples
        }

        let first = snapshot(scenario);
        let replay = snapshot(scenario);
        assert!(!first.is_empty(), "scenario {scenario}");
        assert_eq!(first, replay, "scenario {scenario}");
    }
}

#[test]
fn lifetime() {
    // "embedded geometry outlives its compound without retaining the compound object"
    // The Rust `Compound` is an owned value rather than a class, so there is no weak reference to
    // observe: the equivalent guarantee is that a cloned `Hull` child keeps its own vertex storage
    // after the compound is dropped.
    let compound = make_test_compound(2).unwrap();
    let child = compound.child(0);
    let hull = match &child.geometry {
        CompoundChildGeometry::Hull(hull) => hull.clone(),
        _ => panic!("expected hull"),
    };
    let expected = hull.vertex_positions();
    drop(compound);
    assert_eq!(hull.vertex_positions(), expected);
}

#[test]
fn rejected() {
    // "invalid definitions and damaged compound headers are rejected"
    assert!(Compound::new(&CompoundDef::default()).is_none());
    let compound = make_test_compound(4).unwrap();
    let mut bytes = compound.serialized_bytes();
    bytes[0] ^= 1;
    assert!(Compound::from_serialized_bytes(&bytes).is_none());
    assert!(Compound::from_serialized_bytes(&bytes[..8]).is_none());

    let mut damaged = compound.serialized_bytes();
    let hull_offset = compound.header().hull_offset as usize
        + core::mem::offset_of!(PackedHullInstance, hull_offset);
    for i in 0..4 {
        damaged[hull_offset + i] = 255;
    }
    assert!(Compound::from_serialized_bytes(&damaged).is_none());

    let mut definition = CompoundDef::default();
    definition.hulls = vec![Default::default()];
    assert!(Compound::new(&definition).is_none());
}
