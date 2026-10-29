//! Closest-hit queries that skip one body and all sensors, and report the hit shape index and
//! material. Vehicles, characters, and soft bodies use these to look at the world around
//! themselves.
//!
//! Translated from `Sources/Natsel/Simulation/FilteredQueries.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::world_queries`: the `World` method
//!   `body_transform_quick(body_id: i32) -> WorldTransform` (Swift's `WorldQueries.swift`).
//!
//! Swift's file-local `aabbOffset(_:_:)` (declared in `Continuous.swift` as an `@inline(__always)`
//! alias of `Math/Functions.swift`'s `offsetAABB`) is called through `offset_aabb` directly here,
//! so this file does not depend on the name the concurrent `continuous` port gives that alias. The
//! rounding behaviour is the same function.

use crate::base::constants as b3;
use crate::collision::proxy::compute_proxy_aabb;
use crate::math::functions::{
    clamp_int, cross, make_aabb, max_vec3, min_vec3, offset_aabb, offset_pos, sub_pos,
    to_relative_transform,
};
use crate::math::types::{AABB, Pos, Vec3};
use crate::simulation::shape_planes::collide_mover;
use crate::simulation::shape_runtime::{
    overlap_shape, ray_cast_shape, shape_cast_shape, should_query_collide, Shape,
};
use crate::simulation::world_runtime::{SetType, World};
use crate::types::geometry::Capsule;
use crate::types::queries::{BoxCastInput, PlaneResult, RayCastInput, ShapeCastInput, ShapeProxy};
use crate::types::shape_def::QueryFilter;

/// Closest hit of an internal query.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct QueryHit {
    pub(crate) hit: bool,
    pub(crate) fraction: f32,
    pub(crate) point: Pos,
    pub(crate) normal: Vec3,
    pub(crate) shape_index: i32,
    pub(crate) material_index: i32,
}

impl Default for QueryHit {
    /// Swift's property initializers: `fraction = 1`, `shapeIndex = B3.nullIndex`,
    /// `materialIndex = 0`. A derived `Default` would zero the fraction and the shape index, so the
    /// defaults are spelled out to keep an unset hit distinguishable exactly as in Swift.
    #[inline]
    fn default() -> Self {
        Self {
            hit: false,
            fraction: 1.0,
            point: Pos::default(),
            normal: Vec3::default(),
            shape_index: b3::NULL_INDEX,
            material_index: 0,
        }
    }
}

impl World {
    /// True when an internal query should look at this shape.
    #[inline(always)]
    pub(crate) fn query_accepts_shape(
        &self,
        shape: &Shape,
        filter: &QueryFilter,
        ignore_body: i32,
    ) -> bool {
        shape.body_id != ignore_body
            && shape.sensor_index == b3::NULL_INDEX
            && should_query_collide(&shape.filter, filter)
    }

    /// Closest ray hit from `origin` along `translation`. `accept` may reject a hit by normal.
    ///
    /// Swift defaulted `accept` to `{ _ in true }`; ported callers pass it explicitly.
    pub(crate) fn closest_ray_hit<A>(
        &self,
        origin: Pos,
        translation: Vec3,
        filter: &QueryFilter,
        ignore_body: i32,
        accept: A,
    ) -> QueryHit
    where
        A: Fn(Vec3) -> bool,
    {
        let mut result = QueryHit::default();
        let mut input = BoxCastInput::default();
        input.box_ = offset_aabb(AABB::new(Vec3::default(), Vec3::default()), origin);
        input.translation = translation;
        input.max_fraction = 1.0;

        for tree in &self.broad_phase.trees {
            tree.box_cast(input, filter.mask_bits, false, |query_input, _, user_data| {
                let shape_index = user_data as i64 as i32;
                let shape = &self.shapes[shape_index as usize];
                if !self.query_accepts_shape(shape, filter, ignore_body) {
                    return query_input.max_fraction;
                }

                let mut local_input = RayCastInput::default();
                local_input.translation = translation;
                local_input.max_fraction = query_input.max_fraction;
                let transform = to_relative_transform(
                    self.body_transform_quick(shape.body_id),
                    origin,
                );
                let output = ray_cast_shape(shape, transform, &local_input);
                if !(output.hit && output.fraction < result.fraction && accept(output.normal)) {
                    return query_input.max_fraction;
                }

                result.hit = true;
                result.fraction = output.fraction;
                result.point = offset_pos(origin, output.point);
                result.normal = output.normal;
                result.shape_index = shape_index;
                result.material_index =
                    clamp_int(output.material_index, 0, shape.material_count() - 1);
                output.fraction
            });
            input.max_fraction = result.fraction;
            if result.fraction == 0.0 {
                break;
            }
        }
        result
    }

    /// Closest shape cast hit. The proxy is relative to `origin`. Initially overlapping shapes
    /// (fraction zero) are skipped, as in `shapeCastClosest`.
    ///
    /// Swift defaulted `canEncroach` to `false` and `accept` to `{ _ in true }`; ported
    /// callers pass them explicitly.
    pub(crate) fn closest_shape_cast_hit<A>(
        &self,
        origin: Pos,
        proxy: ShapeProxy,
        translation: Vec3,
        filter: &QueryFilter,
        ignore_body: i32,
        can_encroach: bool,
        accept: A,
    ) -> QueryHit
    where
        A: Fn(Vec3) -> bool,
    {
        let mut result = QueryHit::default();
        if proxy.points.is_none() || proxy.count <= 0 {
            return result;
        }
        let points = proxy.points();

        let mut shape_input = ShapeCastInput::default();
        shape_input.proxy = proxy;
        shape_input.translation = translation;
        shape_input.max_fraction = 1.0;
        shape_input.can_encroach = can_encroach;

        let mut box_input = BoxCastInput::default();
        box_input.box_ = offset_aabb(make_aabb(points, proxy.radius), origin);
        box_input.translation = translation;
        box_input.max_fraction = 1.0;

        for tree in &self.broad_phase.trees {
            tree.box_cast(box_input, filter.mask_bits, false, |advancing, _, user_data| {
                let shape_index = user_data as i64 as i32;
                let shape = &self.shapes[shape_index as usize];
                if !self.query_accepts_shape(shape, filter, ignore_body) {
                    return advancing.max_fraction;
                }

                let mut local = shape_input;
                local.max_fraction = advancing.max_fraction;
                let transform = to_relative_transform(
                    self.body_transform_quick(shape.body_id),
                    origin,
                );
                let output = shape_cast_shape(shape, transform, &local);
                if !(output.hit
                    && output.fraction > 0.0
                    && output.fraction < result.fraction
                    && accept(output.normal))
                {
                    return advancing.max_fraction;
                }

                result.hit = true;
                result.fraction = output.fraction;
                result.point = offset_pos(origin, output.point);
                result.normal = output.normal;
                result.shape_index = shape_index;
                result.material_index =
                    clamp_int(output.material_index, 0, shape.material_count() - 1);
                output.fraction
            });
            box_input.max_fraction = result.fraction;
            if result.fraction == 0.0 {
                break;
            }
        }
        result
    }

    /// Collision planes of a mover capsule (relative to `origin`) against the world, skipping one
    /// body and all sensors. Plane normals point toward the mover; offsets are penetration depths.
    pub(crate) fn collide_mover_ignoring<F>(
        &self,
        origin: Pos,
        mover: &Capsule,
        filter: &QueryFilter,
        ignore_body: i32,
        mut callback: F,
    ) where
        F: FnMut(i32, &[PlaneResult]),
    {
        let radius = Vec3::new(mover.radius, mover.radius, mover.radius);
        let relative = AABB::new(
            min_vec3(mover.center1, mover.center2) - radius,
            max_vec3(mover.center1, mover.center2) + radius,
        );
        let bounds = offset_aabb(relative, origin);
        let mut candidates: Vec<i32> = Vec::new();
        for tree in &self.broad_phase.trees {
            tree.query(bounds, filter.mask_bits, false, |_, data| {
                let shape_index = data as i64 as i32;
                if self.query_accepts_shape(&self.shapes[shape_index as usize], filter, ignore_body)
                {
                    candidates.push(shape_index);
                }
                true
            });
        }
        // Shape order keeps results independent of tree layout.
        candidates.sort();
        for shape_index in candidates {
            let shape = &self.shapes[shape_index as usize];
            let transform = to_relative_transform(self.body_transform_quick(shape.body_id), origin);
            let planes = collide_mover(shape, transform, mover, 64);
            if planes.is_empty() == false {
                callback(shape_index, &planes);
            }
        }
    }

    /// True when the proxy (relative to `origin`) overlaps any accepted shape.
    ///
    /// Swift defaulted `accept` to `{ _ in true }`; ported callers pass it explicitly.
    pub(crate) fn overlaps_ignoring<A>(
        &self,
        origin: Pos,
        proxy: ShapeProxy,
        filter: &QueryFilter,
        ignore_body: i32,
        accept: A,
    ) -> bool
    where
        A: Fn(i32) -> bool,
    {
        let bounds = offset_aabb(compute_proxy_aabb(proxy), origin);
        let mut found = false;
        for tree in &self.broad_phase.trees {
            tree.query(bounds, filter.mask_bits, false, |_, data| {
                let shape_index = data as i64 as i32;
                let shape = &self.shapes[shape_index as usize];
                if !self.query_accepts_shape(shape, filter, ignore_body) || !accept(shape_index) {
                    return true;
                }
                let transform =
                    to_relative_transform(self.body_transform_quick(shape.body_id), origin);
                found = overlap_shape(shape, transform, &proxy);
                !found
            });
            if found {
                break;
            }
        }
        found
    }

    /// World space velocity of a point on a body. Static and sleeping bodies report zero.
    #[inline(always)]
    pub(crate) fn point_velocity(&self, body_index: i32, point: Pos) -> Vec3 {
        let record = &self.bodies[body_index as usize];
        if record.set_index != SetType::AwakeSet.raw_value() {
            return Vec3::default();
        }
        let set = SetType::AwakeSet.raw_value() as usize;
        let state = self.solver_sets[set].body_states[record.local_index as usize];
        let center = self.solver_sets[set].body_sims[record.local_index as usize].center;
        state.linear_velocity + cross(state.angular_velocity, sub_pos(point, center))
    }
}
