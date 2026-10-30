//! Ported from Tests/NatselTests/MathTests.swift.
//!
//! Exercises scalar math helpers. Keep tolerances identical to the Swift suite.

use crate::base::constants::PI;
use crate::math::functions::{atan2_approx, compute_cos_sin};

#[test]
fn atan2_accuracy() {
    // "atan2 approximation stays within tolerance"
    assert_eq!(atan2_approx(0.0, 0.0), 0.0);
    assert!((atan2_approx(1.0, 1.0) - 0.785398163).abs() < 1e-4);
    assert!((atan2_approx(-1.0, -1.0) + 2.35619449).abs() < 1e-4);
}

#[test]
fn cos_sin() {
    // "cosine and sine pair is normalized"
    let cs = compute_cos_sin(0.5);
    assert!((cs.cosine * cs.cosine + cs.sine * cs.sine - 1.0).abs() < 1e-6);
}

#[test]
fn pi_rounding() {
    // "pi rounds to the nearest float"
    assert_eq!(PI.to_bits(), 0x4049_0FDB);
}
