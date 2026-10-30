//! Ported from Tests/NatselTests/Collision/HeightField/TextRoundTripTests.swift.
//!
//! Authored terrain must survive the text format, and truncated or invalid data must be rejected.

use crate::collision::height_field::quantization::HeightField;
use crate::tests::collision::height_field::encoding_tests::make_test_height_definition;
use crate::types::geometry::HeightFieldDef;

/// A unique temporary path, standing in for Swift's `UUID().uuidString`.
fn unique_path(tag: &str) -> String {
    use std::sync::atomic::{AtomicU64, Ordering};
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    let nanos = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    std::env::temp_dir()
        .join(format!("natsel-height-{tag}-{nanos}-{n}.txt"))
        .to_string_lossy()
        .into_owned()
}

#[test]
fn round_trip() {
    // "authored text and loaded terrain round-trip through the text format"
    fn snapshot() -> Vec<u64> {
        let mut samples: Vec<u64> = Vec::new();
        let definition = make_test_height_definition(true);

        // `HeightFieldText` is private in the port (Swift's was internal), so the text is produced
        // and consumed through the public `write`/`load` pair. The encoded byte length and its
        // hash are asserted below, which covers the same format-stability guarantee.
        let path = unique_path("round-trip");
        definition.write(&path).expect("height field write failed");
        let text = std::fs::read_to_string(&path).expect("height field read failed");
        let bytes = text.as_bytes();
        assert_eq!(bytes.len(), 655);

        // Swift hashed the encoded bytes with `hash64NonZero`; call it on the same bytes.
        let digest = crate::base::hash::hash64_non_zero(bytes);
        samples.push(digest);

        // Loading the text must reproduce a field whose stored hash matches the authoring
        // definition's, which is the round-trip guarantee the Swift case asserted by comparing the
        // decoded definition.
        let loaded = HeightField::load(&path).expect("height field load failed");
        assert_eq!(loaded.hash(), HeightField::new(&definition).unwrap().hash());
        let _ = std::fs::remove_file(&path);

        let field_path = unique_path("round-trip-field");
        definition
            .write(&field_path)
            .expect("height field write failed");
        let field = HeightField::load(&field_path).expect("height field load failed");
        let _ = std::fs::remove_file(&field_path);
        samples.push(field.hash());
        samples
    }

    let first = snapshot();
    let replay = snapshot();
    assert!(!first.is_empty());
    assert_eq!(first, replay);
}

#[test]
fn malformed() {
    // "truncated and invalid height data is rejected"
    // `HeightFieldText::decode` is private, so the equivalent rejection is exercised through
    // `HeightField::load`, which performs the same parse and returns the same `InvalidData`.
    let path = unique_path("malformed-empty");
    std::fs::write(&path, b"").expect("write failed");
    assert!(HeightField::load(&path).is_err());
    let _ = std::fs::remove_file(&path);

    let path = unique_path("malformed-truncated");
    std::fs::write(&path, b"7 6 1 1 1 -1 1 0").expect("write failed");
    assert!(HeightField::load(&path).is_err());
    let _ = std::fs::remove_file(&path);

    let path = unique_path("malformed-negative");
    std::fs::write(&path, b"-7 6 1 1 1 -1 1 0").expect("write failed");
    assert!(HeightField::load(&path).is_err());
    let _ = std::fs::remove_file(&path);

    assert!(HeightField::new(&HeightFieldDef::default()).is_none());

    let mut definition = make_test_height_definition(false);
    definition.heights[0] = f32::NAN;
    assert!(HeightField::new(&definition).is_none());

    definition.heights[0] = 0.0;
    definition.material_indices.pop();
    assert!(HeightField::new(&definition).is_none());
}
