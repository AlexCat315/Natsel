//! Precise world bounds. `AABB` remains the compact, conservative broad-phase representation.

use crate::math::functions::{offset_pos, round_down_float, round_up_float};
use crate::math::types::{AABB, Pos, Vec3};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WorldAABB {
    pub lower_bound: Pos,
    pub upper_bound: Pos,
}

impl WorldAABB {
    #[inline]
    pub const fn new(lower_bound: Pos, upper_bound: Pos) -> Self {
        Self {
            lower_bound,
            upper_bound,
        }
    }

    #[inline]
    pub fn from_local_bounds(local_bounds: AABB, origin: Pos) -> Self {
        Self {
            lower_bound: offset_pos(origin, local_bounds.lower_bound),
            upper_bound: offset_pos(origin, local_bounds.upper_bound),
        }
    }

    /// Outward rounding preserves all candidates when querying the compact spatial tree.
    #[inline]
    pub fn conservative_bounds(&self) -> AABB {
        AABB::new(
            Vec3::new(
                round_down_float(self.lower_bound.x),
                round_down_float(self.lower_bound.y),
                round_down_float(self.lower_bound.z),
            ),
            Vec3::new(
                round_up_float(self.upper_bound.x),
                round_up_float(self.upper_bound.y),
                round_up_float(self.upper_bound.z),
            ),
        )
    }

    #[inline]
    pub fn contains(&self, point: Pos) -> bool {
        self.lower_bound.x <= point.x
            && point.x <= self.upper_bound.x
            && self.lower_bound.y <= point.y
            && point.y <= self.upper_bound.y
            && self.lower_bound.z <= point.z
            && point.z <= self.upper_bound.z
    }

    #[inline]
    pub fn overlaps(&self, other: &WorldAABB) -> bool {
        self.lower_bound.x <= other.upper_bound.x
            && other.lower_bound.x <= self.upper_bound.x
            && self.lower_bound.y <= other.upper_bound.y
            && other.lower_bound.y <= self.upper_bound.y
            && self.lower_bound.z <= other.upper_bound.z
            && other.lower_bound.z <= self.upper_bound.z
    }
}
