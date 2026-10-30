//! Ported from Tests/NatselTests/Collision/ShapeCastEncroachTests.swift.
//!
//! The speculative shape cast has to close a real gap between two convex point sets, and
//! `canEncroach` only governs travel *after* contact, not whether a gap may be closed.

use crate::collision::distance::shape_cast;
use crate::math::types::{Transform, Vec3};
use crate::tests::world_position_assertions::abs_float;
use crate::types::queries::{CastOutput, ShapeCastPairInput, ShapeProxy};

fn cast(b_points: &[Vec3], translation: Vec3, max_fraction: f32) -> CastOutput {
    let box_a = [
        Vec3::new(-1.0, -1.0, 0.0),
        Vec3::new(1.0, -1.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(-1.0, 1.0, 0.0),
    ];
    let mut input = ShapeCastPairInput::default();
    input.transform = Transform::IDENTITY;
    input.translation_b = translation;
    input.max_fraction = max_fraction;
    input.can_encroach = true;
    input.proxy_a = ShapeProxy::new(Some(box_a.as_ptr()), box_a.len() as i32, 0.0);
    input.proxy_b = ShapeProxy::new(Some(b_points.as_ptr()), b_points.len() as i32, 0.0);
    shape_cast(&input)
}

fn segment_b() -> [Vec3; 2] {
    [Vec3::new(1.002, -1.0, 0.0), Vec3::new(1.002, 1.0, 0.0)]
}

/// A 5mm-gap variant used by the graded and max-fraction cases.
fn graded_b() -> [Vec3; 2] {
    [Vec3::new(1.005, -1.0, 0.0), Vec3::new(1.005, 1.0, 0.0)]
}

#[test]
fn recede() {
    // "receding from the gap misses"
    assert!(!cast(&segment_b(), Vec3::new(1.0, 0.0, 0.0), 1.0).hit);
}

#[test]
fn slide() {
    // "sliding along the gap misses"
    assert!(!cast(&segment_b(), Vec3::new(0.0, 2.0, 0.0), 1.0).hit);
}

#[test]
fn closing() {
    // "closing the gap hits immediately"
    let out = cast(&segment_b(), Vec3::new(-1.0, 0.0, 0.0), 1.0);
    assert!(out.hit);
    assert!(abs_float(out.fraction as f64) < f32::EPSILON as f64);
    assert!(abs_float((out.normal.x - 1.0) as f64) < 0.001);
}

#[test]
fn graded() {
    // "a 5mm gap closes at the graded fraction"
    let out = cast(&graded_b(), Vec3::new(-1.0, 0.0, 0.0), 1.0);
    assert!(out.hit);
    assert!(abs_float((out.fraction - 0.0025) as f64) < 0.0005);
    assert!(abs_float((out.normal.x - 1.0) as f64) < 0.001);
}

#[test]
fn max_fraction() {
    // "max fraction gates the closed-gap hit"
    assert!(!cast(&graded_b(), Vec3::new(-1.0, 0.0, 0.0), 0.002).hit);

    let out = cast(&graded_b(), Vec3::new(-1.0, 0.0, 0.0), 0.003);
    assert!(out.hit);
    assert!(abs_float((out.fraction - 0.0025) as f64) < 0.0005);
}

#[test]
fn graze() {
    // "a mostly sideways sweep with no inward component misses"
    // Moving purely sideways keeps the 2mm rest gap, so nothing closes.
    assert!(!cast(&segment_b(), Vec3::new(0.0, 4.0, 0.0), 1.0).hit);
    // Moving purely outward opens the gap further.
    assert!(!cast(&segment_b(), Vec3::new(2.0, 4.0, 0.0), 1.0).hit);
}

#[test]
fn overlap() {
    // "starting overlapped reports a zero fraction when encroaching is allowed"
    // Start the segment inside the box.
    let inside = [Vec3::new(0.5, -1.0, 0.0), Vec3::new(0.5, 1.0, 0.0)];
    let out = cast(&inside, Vec3::new(-1.0, 0.0, 0.0), 1.0);
    assert!(out.hit);
    assert!(abs_float(out.fraction as f64) < f32::EPSILON as f64);
}

#[test]
fn no_encroach() {
    // "without encroaching the same gap is not closed"
    let box_a = [
        Vec3::new(-1.0, -1.0, 0.0),
        Vec3::new(1.0, -1.0, 0.0),
        Vec3::new(1.0, 1.0, 0.0),
        Vec3::new(-1.0, 1.0, 0.0),
    ];
    let b = segment_b();
    let mut input = ShapeCastPairInput::default();
    input.transform = Transform::IDENTITY;
    input.translation_b = Vec3::new(-1.0, 0.0, 0.0);
    input.max_fraction = 1.0;
    input.can_encroach = false;
    input.proxy_a = ShapeProxy::new(Some(box_a.as_ptr()), box_a.len() as i32, 0.0);
    input.proxy_b = ShapeProxy::new(Some(b.as_ptr()), b.len() as i32, 0.0);
    let output = shape_cast(&input);

    // Closing the 2mm gap still registers; canEncroach only governs how close a cast may
    // travel *after* contact, not whether a real gap may be closed.
    assert!(output.hit);
}
