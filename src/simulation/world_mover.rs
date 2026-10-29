//! Mover and query-shape casts against the world.
//!
//! Translated from `Sources/Natsel/Simulation/WorldMover.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::world_queries`: the `World` method
//!   `body_transform_quick(body_id: i32) -> WorldTransform` (Swift's `WorldQueries.swift`).
//!
//! Swift's file-local `aabbOffset(_:_:)` (declared in `Continuous.swift` as an
//! `@inline(__always)` alias of `Math/Functions.swift`'s `offsetAABB`) is called through
//! `offset_aabb` directly here, so this file does not depend on the name the concurrent
//! `continuous` port gives that alias.

use crate::math::functions::{
    make_aabb, max_vec3, min_vec3, offset_aabb, offset_pos, to_relative_transform,
};
use crate::math::types::{AABB, Pos, Vec3};
use crate::simulation::shape_planes::collide_mover;
use crate::simulation::shape_runtime::{shape_cast_shape, should_query_collide};
use crate::simulation::world_runtime::World;
use crate::types::geometry::Capsule;
use crate::types::ids::ShapeID;
use crate::types::queries::{BoxCastInput, PlaneResult, ShapeCastInput, ShapeProxy};
use crate::types::shape_def::QueryFilter;

impl World {
    /// Collide a mover capsule (relative to `origin`) with the world.
    ///
    /// Swift defaulted `filter` to `.defaultFilter`; ported callers pass it explicitly.
    pub fn collide_mover<F>(
        &self,
        origin: Pos,
        mover: &Capsule,
        filter: &QueryFilter,
        mut callback: F,
    ) where
        F: FnMut(ShapeID, &[PlaneResult]) -> bool,
    {
        let radius = Vec3::new(mover.radius, mover.radius, mover.radius);
        let relative = AABB::new(
            min_vec3(mover.center1, mover.center2) - radius,
            max_vec3(mover.center1, mover.center2) + radius,
        );
        let bounds = offset_aabb(relative, origin);
        for tree in &self.broad_phase.trees {
            tree.query(bounds, filter.mask_bits, false, |_, data| {
                let shape = &self.shapes[data as usize];
                if !should_query_collide(&shape.filter, filter) {
                    return true;
                }
                let transform =
                    to_relative_transform(self.body_transform_quick(shape.body_id), origin);
                let planes = collide_mover(shape, transform, mover, 64);
                if planes.is_empty() {
                    return true;
                }
                let id = ShapeID::new(shape.id + 1, self.world_id, shape.generation);
                callback(id, &planes)
            });
        }
    }

    /// Cast a mover capsule through the world, returning the first hit fraction.
    ///
    /// Swift defaulted `filter` to `.defaultFilter` and `shouldCollide` to `nil`; ported callers
    /// pass them explicitly.
    pub fn cast_mover(
        &self,
        origin: Pos,
        mover: &Capsule,
        translation: Vec3,
        filter: &QueryFilter,
        should_collide: Option<&dyn Fn(ShapeID) -> bool>,
    ) -> f32 {
        let centers = [mover.center1, mover.center2];
        // SAFETY: the two element array is a local of this function and is neither moved nor
        // mutated before it returns, and the proxy below never escapes it - every consumer
        // (`shape_cast_shape`) reads exactly the two points through the pointer while this frame is
        // live. This mirrors Swift's `centers.withUnsafeBufferPointer { points in ... }`, whose
        // `points.baseAddress` had the same scoped-lifetime guarantee.
        let mut shape_input = ShapeCastInput::default();
        shape_input.proxy = ShapeProxy::new(Some(centers.as_ptr()), 2, mover.radius);
        shape_input.translation = translation;
        shape_input.max_fraction = 1.0;
        shape_input.can_encroach = mover.radius > 0.0;
        let mut input = BoxCastInput::default();
        input.box_ = offset_aabb(make_aabb(&centers, mover.radius), origin);
        input.translation = translation;
        input.max_fraction = 1.0;
        let mut fraction: f32 = 1.0;
        for tree in &self.broad_phase.trees {
            tree.box_cast(input, filter.mask_bits, false, |advancing, _, data| {
                let shape = &self.shapes[data as usize];
                if !should_query_collide(&shape.filter, filter) {
                    return fraction;
                }
                let id = ShapeID::new(shape.id + 1, self.world_id, shape.generation);
                if let Some(should_collide) = should_collide {
                    if !should_collide(id) {
                        return fraction;
                    }
                }
                let mut local = shape_input;
                local.max_fraction = advancing.max_fraction;
                let transform =
                    to_relative_transform(self.body_transform_quick(shape.body_id), origin);
                let output = shape_cast_shape(shape, transform, &local);
                if output.fraction == 0.0 {
                    return fraction;
                }
                fraction = output.fraction;
                fraction
            });
            if fraction == 0.0 {
                break;
            }
            input.max_fraction = fraction;
        }
        fraction
    }

    /// Cast a generic query shape through the world. The proxy is expressed in the query frame
    /// relative to `origin`.
    ///
    /// Swift defaulted `filter` to `.defaultFilter` and `hit` to `nil`; ported callers pass them
    /// explicitly.
    pub fn cast_shape(
        &self,
        origin: Pos,
        proxy: ShapeProxy,
        translation: Vec3,
        filter: &QueryFilter,
        hit: Option<&dyn Fn(ShapeID, Pos, Vec3, f32) -> f32>,
    ) -> f32 {
        if proxy.points.is_none() || proxy.count <= 0 {
            return 1.0;
        }
        let points = proxy.points();

        let mut shape_input = ShapeCastInput::default();
        shape_input.proxy = proxy;
        shape_input.translation = translation;
        shape_input.max_fraction = 1.0;
        shape_input.can_encroach = proxy.radius > 0.0;

        let mut input = BoxCastInput::default();
        input.box_ = offset_aabb(make_aabb(points, proxy.radius), origin);
        input.translation = translation;
        input.max_fraction = 1.0;

        let mut fraction: f32 = 1.0;
        for tree in &self.broad_phase.trees {
            tree.box_cast(input, filter.mask_bits, false, |advancing, _, data| {
                let shape = &self.shapes[data as usize];
                if !should_query_collide(&shape.filter, filter) {
                    return fraction;
                }
                let mut local = shape_input;
                local.max_fraction = advancing.max_fraction;
                let transform =
                    to_relative_transform(self.body_transform_quick(shape.body_id), origin);
                let output = shape_cast_shape(shape, transform, &local);
                if output.fraction == 0.0 {
                    return fraction;
                }

                let id = ShapeID::new(shape.id + 1, self.world_id, shape.generation);
                if let Some(hit) = hit {
                    let point = offset_pos(origin, output.point);
                    let next = hit(id, point, output.normal, output.fraction);
                    fraction = next;
                    return next;
                }
                fraction = output.fraction;
                fraction
            });
            if fraction == 0.0 {
                break;
            }
            input.max_fraction = fraction;
        }
        fraction
    }
}
