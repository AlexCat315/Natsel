//! Soft contact feature tree.

use crate::math::functions::{aabb_inflate, aabb_overlaps};
use crate::math::types::{AABB, Vec3};
use crate::soft_body::soft_contact_tree::SoftContactTree;

fn inflate(point: Vec3, margin: f32) -> AABB {
    aabb_inflate(AABB::new(point, point), margin)
}

fn no_reference() -> Vec<AABB> {
    Vec::new()
}

#[test]
// "queries match brute-force overlap without duplicate features"
fn complete_candidates() {
    let boxes: Vec<AABB> = (0..48)
        .map(|index: i32| {
            let center = Vec3::new(
                ((index * 7) % 8) as f32 - 3.0,
                (index / 24) as f32,
                (index / 8) as f32 - 2.0,
            );
            inflate(center, 0.25)
        })
        .collect();

    let mut tree = SoftContactTree::default();
    tree.refit(&boxes, no_reference);

    for x in -5..=5 {
        let center = Vec3::new(x as f32, 0.0, 0.0);
        let query = inflate(center, 1.25);

        let mut found: Vec<i32> = Vec::new();
        tree.visit(query, -1, None, |feature| found.push(feature));
        let expected: Vec<i32> = boxes
            .iter()
            .enumerate()
            .filter(|(_, box_)| aabb_overlaps(query, **box_))
            .map(|(index, _)| index as i32)
            .collect();
        let mut found_sorted = found.clone();
        found_sorted.sort();
        assert_eq!(found_sorted, expected);

        let mut deduped = found.clone();
        deduped.sort();
        deduped.dedup();
        assert_eq!(deduped.len(), found.len(), "a feature was visited twice");

        let mut later: Vec<i32> = Vec::new();
        tree.visit(query, 23, None, |feature| later.push(feature));
        later.sort();
        let expected_later: Vec<i32> = expected.iter().copied().filter(|f| *f > 23).collect();
        assert_eq!(later, expected_later);
    }
}

#[test]
// "refitting moving features updates ancestor bounds and rebuilding changes feature count"
fn refit_and_rebuild() {
    let mut boxes: Vec<AABB> = (0..16)
        .map(|index: i32| {
            let p = Vec3::new(index as f32, 0.0, 0.0);
            inflate(p, 0.1)
        })
        .collect();

    let mut tree = SoftContactTree::default();
    tree.refit(&boxes, no_reference);

    let distant = Vec3::new(-100.0, 50.0, 12.0);
    boxes[3] = AABB::new(distant, distant);
    tree.refit(&boxes, no_reference);

    let mut remote: Vec<i32> = Vec::new();
    tree.visit(
        AABB::new(distant, distant),
        -1,
        None,
        |feature| remote.push(feature),
    );
    assert_eq!(remote, [3]);

    let mut old: Vec<i32> = Vec::new();
    tree.visit(
        AABB::new(Vec3::new(3.0, 0.0, 0.0), Vec3::new(3.0, 0.0, 0.0)),
        -1,
        None,
        |feature| old.push(feature),
    );
    assert!(old.is_empty());

    tree.refit(&[AABB::new(Vec3::ZERO, Vec3::ZERO)], no_reference);
    let mut rebuilt: Vec<i32> = Vec::new();
    tree.visit(
        AABB::new(Vec3::ZERO, Vec3::ZERO),
        -1,
        None,
        |feature| rebuilt.push(feature),
    );
    assert_eq!(rebuilt, [0]);

    tree.refit(&[], no_reference);
    let mut empty: Vec<i32> = Vec::new();
    tree.visit(
        AABB::new(Vec3::ZERO, Vec3::ZERO),
        -1,
        None,
        |feature| empty.push(feature),
    );
    assert!(empty.is_empty());
}

#[test]
// "coincident features are each visited once"
fn coincident_features() {
    let mut tree = SoftContactTree::default();
    let boxes = vec![AABB::new(Vec3::ZERO, Vec3::ZERO); 20];
    tree.refit(&boxes, no_reference);

    let mut found: Vec<i32> = Vec::new();
    tree.visit(
        AABB::new(Vec3::ZERO, Vec3::ZERO),
        -1,
        None,
        |feature| found.push(feature),
    );
    found.sort();
    let expected: Vec<i32> = (0..20i32).collect();
    assert_eq!(found, expected);
}

#[test]
// "rest exclusion prunes neighborhoods and preserves distant rest features folded together"
fn reference_exclusion() {
    let current = vec![AABB::new(Vec3::ZERO, Vec3::ZERO); 32];
    let near = Vec3::new(0.1, 0.0, 0.0);
    let far = Vec3::new(10.0, 0.0, 0.0);
    let reference: Vec<AABB> = (0..32)
        .map(|index: i32| {
            let p = if index < 16 { near } else { far };
            AABB::new(p, p)
        })
        .collect();

    let mut tree = SoftContactTree::default();
    let reference_for_refit = reference.clone();
    tree.refit(&current, || reference_for_refit);

    let mut found: Vec<i32> = Vec::new();
    tree.visit(
        AABB::new(Vec3::ZERO, Vec3::ZERO),
        -1,
        Some((Vec3::ZERO, 1.0)),
        |feature| found.push(feature),
    );
    found.sort();
    let expected: Vec<i32> = (16..32i32).collect();
    assert_eq!(found, expected);
}
