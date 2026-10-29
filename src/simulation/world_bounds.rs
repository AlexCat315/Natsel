//! Precise world bounds and a world-bounds overlap query.
//!
//! Translated from `Sources/Natsel/Simulation/WorldBounds.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::world_queries`: the `World` methods `body_transform_quick(body_id: i32)
//!   -> WorldTransform` and `overlap_aabb(aabb: AABB, filter: QueryFilter, callback) -> TreeStats`
//!   (Swift's `WorldQueries.swift`, same argument order).
//! - `crate::soft_body::soft_body_runtime`: the `SoftBodyID` handle, the `World` method
//!   `soft_body_index(id: SoftBodyID) -> Option<usize>`, and `SoftBodyRecord::{bounds, origin}`.

use crate::base::constants as b3;
use crate::collision::dynamic_tree::TreeStats;
use crate::math::functions::aabb_union;
use crate::math::types::{AABB, Transform, Vec3};
use crate::math::world_aabb::WorldAABB;
use crate::soft_body::soft_body_runtime::SoftBodyID;
use crate::simulation::shape_runtime::compute_shape_aabb;
use crate::simulation::world_runtime::World;
use crate::types::ids::{BodyID, ShapeID};
use crate::types::shape_def::QueryFilter;

impl World {
    /// Tight world bounds without narrowing the absolute coordinates to float.
    pub fn shape_world_aabb(&self, id: ShapeID) -> WorldAABB {
        let shape = &self.shapes[(id.index1 - 1) as usize];
        let transform = self.body_transform_quick(shape.body_id);
        WorldAABB::from_local_bounds(
            compute_shape_aabb(shape, Transform::new(Vec3::ZERO, transform.q)),
            transform.p,
        )
    }

    pub fn body_world_aabb(&self, id: BodyID) -> WorldAABB {
        let record = self.body(id);
        let transform = self.body_transform_quick(self.body_index(id));
        let mut local_bounds = AABB::new(Vec3::ZERO, Vec3::ZERO);
        let mut shape_id = record.head_shape_id;
        let mut first = true;
        while shape_id != b3::NULL_INDEX {
            let shape = &self.shapes[shape_id as usize];
            let bounds = compute_shape_aabb(shape, Transform::new(Vec3::ZERO, transform.q));
            local_bounds = if first {
                bounds
            } else {
                aabb_union(local_bounds, bounds)
            };
            first = false;
            shape_id = shape.next_shape_id;
        }
        WorldAABB::from_local_bounds(local_bounds, transform.p)
    }

    pub fn soft_body_world_aabb(&self, id: SoftBodyID) -> Option<WorldAABB> {
        match self.soft_body_index(id) {
            None => None,
            Some(i) => Some(WorldAABB::from_local_bounds(
                self.soft_bodies[i].bounds,
                self.soft_bodies[i].origin,
            )),
        }
    }
}

impl World {
    /// Query precise world bounds, rejecting conservative float-tree false positives.
    ///
    /// Swift declared this as a second `overlapAABB` overload next to the `AABB` version in
    /// `WorldQueries.swift`; Rust has no overloading, so this keeps the semantics and takes the
    /// type-specific name `overlap_world_aabb`. Swift defaulted `filter` to `.defaultFilter`, so
    /// ported callers pass `QueryFilter::default_filter()` explicitly.
    pub fn overlap_world_aabb<F>(
        &self,
        bounds: WorldAABB,
        filter: QueryFilter,
        mut callback: F,
    ) -> TreeStats
    where
        F: FnMut(ShapeID) -> bool,
    {
        self.overlap_aabb(bounds.conservative_bounds(), filter, |id| {
            if bounds.overlaps(&self.shape_world_aabb(id)) == false {
                return true;
            }
            callback(id)
        })
    }
}
