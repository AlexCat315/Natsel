//! Shared helpers for the ported suite, mirroring `Tests/NatselTests/WorldPositionAssertions.swift`.
//!
//! Existing tolerances remain in local units; don't narrow double world coordinates for assertions.

use crate::math::functions::distance_pos;
use crate::math::types::{Pos, Vec3};

/// A value usable where a double precision world position is expected.
pub trait IntoPos {
    fn into_pos(self) -> Pos;
}

impl IntoPos for Pos {
    #[inline]
    fn into_pos(self) -> Pos {
        self
    }
}

impl IntoPos for Vec3 {
    #[inline]
    fn into_pos(self) -> Pos {
        Pos::from_vec3(self)
    }
}

#[inline]
pub fn abs_float(value: f64) -> f64 {
    value.abs()
}

/// `Natsel.distance(a, Pos(b))`, widened so either operand may be `Pos` or `Vec3`.
#[inline]
pub fn distance<A: IntoPos, B: IntoPos>(a: A, b: B) -> f32 {
    distance_pos(a.into_pos(), b.into_pos())
}
