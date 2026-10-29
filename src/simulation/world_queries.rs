//! World queries: overlap AABB, overlap shape, ray cast, shape cast.
//!
//! Translated from `Sources/Natsel/Simulation/WorldQueries.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"): the query helpers and runtime types already landed
//! (`shape_runtime`, `world_runtime`, `broad_phase`, `collision::dynamic_tree`,
//! `collision::proxy`, `types::{queries, shape_def, ids, body_def}`). `world_bounds.rs` and the
//! joint modules call `body_transform_quick(body_id: i32) -> WorldTransform` and
//! `overlap_aabb(aabb, filter, callback) -> TreeStats` with the signatures defined here.
//!
//! Swift's query methods read `self.shapes` while a user callback runs against the whole
//! reference-aliased `World` class; the Swift compiler accepted that because class exclusivity is
//! unchecked through `borrowing` reads in a closure. Rust cannot express the alias, so the tree
//! callbacks read shape storage through raw pointer copies of the `Vec` buffers (see
//! `shape_at`), exactly as `body_transform_quick` reads through `Vec` indexing. The traversal
//! order, filter checks, id construction, and fraction updates are unchanged.

use crate::base::core::b3_assert;
use crate::collision::proxy::compute_proxy_aabb;
use crate::collision::dynamic_tree::TreeStats;
use crate::math::functions::{clamp_int, offset_aabb, offset_pos, to_relative_transform};
use crate::math::types::{AABB, Pos, Transform, Vec3, WorldTransform};
use crate::simulation::shape_runtime::{
    ray_cast_shape, should_query_collide, shape_cast_shape, Shape,
};
use crate::simulation::world_runtime::World;
use crate::types::body_def::BodyType;
use crate::types::ids::ShapeID;
use crate::types::queries::{
    BoxCastInput, RayCastInput, RayResult, ShapeCastInput, ShapeProxy,
};
use crate::types::shape_def::QueryFilter;

/// Result of the closest shape cast.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ShapeCastResult {
    pub shape_id: ShapeID,
    pub point: Pos,
    pub normal: Vec3,
    pub fraction: f32,
    pub user_material_id: u64,
    pub triangle_index: i32,
    pub child_index: i32,
    pub hit: bool,
}

impl World {
    /// Fast body transform lookup without locking.
    pub fn body_transform_quick(&self, body_id: i32) -> WorldTransform {
        let body = &self.bodies[body_id as usize];
        self.solver_sets[body.set_index as usize].body_sims[body.local_index as usize].transform
    }

    #[inline(always)]
    fn query_shape_transform(&self, shape: &Shape, origin: Pos) -> Transform {
        to_relative_transform(self.body_transform_quick(shape.body_id), origin)
    }

    /// Raw handle to the shape storage for tree callbacks.
    ///
    /// SAFETY: returns a pointer to the heap buffer of `self.shapes`. The pointer is valid for
    /// as long as `self` is alive and the vector is not mutated; Swift relied on the same
    /// aliasing (`self.shapes[...]` read inside the traversal closure while the user callback
    /// held the reference-aliased world). The tree traversal itself never grows `shapes`, so
    /// reading `(*ptr)[index]` inside a callback observes exactly the element Swift observed.
    #[inline(always)]
    fn shapes_ptr(&self) -> *const Vec<Shape> {
        &self.shapes
    }

    /// Borrow a shape through the raw storage pointer used by the tree callbacks.
    ///
    /// SAFETY: `ptr` must come from `shapes_ptr` of the same `World` for the duration of the
    /// traversal, and `index` must be in bounds - the same contract Swift's array subscript
    /// gave `self.shapes[Int(shapeId)]`.
    #[inline(always)]
    unsafe fn shape_at(&self, ptr: *const Vec<Shape>, index: i32) -> &Shape {
        // SAFETY: see `shapes_ptr`; the buffer is alive and `index` is in bounds, so the
        // derived element reference is valid for the traversal callback body.
        unsafe { &(*ptr)[index as usize] }
    }

    /// Query the world for all shapes whose AABB overlaps the given box.
    pub fn overlap_aabb<F>(&self, aabb: AABB, filter: QueryFilter, mut callback: F) -> TreeStats
    where
        F: FnMut(ShapeID) -> bool,
    {
        let mut tree_stats = TreeStats::default();

        let shapes_ptr = self.shapes_ptr();
        let world_id = self.world_id;

        for i in 0..BodyType::COUNT as usize {
            let tree_result = self.broad_phase.trees[i].query(
                aabb,
                filter.mask_bits,
                false,
                |_proxy_id, user_data| {
                    let shape_id = user_data as i64 as i32;
                    // SAFETY: pointer and index satisfy the `shape_at` contract (see above).
                    let shape = unsafe { self.shape_at(shapes_ptr, shape_id) };

                    if !should_query_collide(&shape.filter, &filter) {
                        return true;
                    }

                    let id = ShapeID::new(shape.id + 1, world_id, shape.generation);
                    callback(id)
                },
            );

            tree_stats.node_visits += tree_result.node_visits;
            tree_stats.leaf_visits += tree_result.leaf_visits;
        }

        tree_stats
    }

    /// Query the world for all shapes that overlap the given shape proxy placed at `origin`.
    pub fn overlap_shape<F>(
        &self,
        origin: Pos,
        proxy: ShapeProxy,
        filter: QueryFilter,
        mut callback: F,
    ) -> TreeStats
    where
        F: FnMut(ShapeID) -> bool,
    {
        let mut tree_stats = TreeStats::default();

        let aabb = offset_aabb(compute_proxy_aabb(proxy), origin);

        let shapes_ptr = self.shapes_ptr();
        let world_id = self.world_id;

        for i in 0..BodyType::COUNT as usize {
            let tree_result = self.broad_phase.trees[i].query(
                aabb,
                filter.mask_bits,
                false,
                |_proxy_id, user_data| {
                    let shape_id = user_data as i64 as i32;
                    // SAFETY: pointer and index satisfy the `shape_at` contract (see above).
                    let shape = unsafe { self.shape_at(shapes_ptr, shape_id) };

                    if !should_query_collide(&shape.filter, &filter) {
                        return true;
                    }

                    let transform = self.query_shape_transform(shape, origin);
                    if !crate::simulation::shape_runtime::overlap_shape(shape, transform, &proxy) {
                        return true;
                    }

                    let id = ShapeID::new(shape.id + 1, world_id, shape.generation);
                    callback(id)
                },
            );

            tree_stats.node_visits += tree_result.node_visits;
            tree_stats.leaf_visits += tree_result.leaf_visits;
        }

        tree_stats
    }

    /// Cast a ray through the world. The callback returns the new fraction, or -1 to skip the shape.
    pub fn cast_ray<F>(
        &self,
        origin: Pos,
        translation: Vec3,
        filter: QueryFilter,
        mut callback: F,
    ) -> TreeStats
    where
        F: FnMut(ShapeID, Pos, Vec3, f32, u64, i32, i32) -> f32,
    {
        let mut tree_stats = TreeStats::default();
        let mut fraction: f32 = 1.0;
        let mut input = BoxCastInput::default();
        input.box_ = offset_aabb(AABB::new(Vec3::ZERO, Vec3::ZERO), origin);
        input.translation = translation;
        input.max_fraction = 1.0;

        let shapes_ptr = self.shapes_ptr();
        let world_id = self.world_id;

        for i in 0..BodyType::COUNT as usize {
            let tree_result = self.broad_phase.trees[i].box_cast(
                input,
                filter.mask_bits,
                false,
                |query_input, _proxy_id, user_data| {
                    let shape_id = user_data as i64 as i32;
                    // SAFETY: pointer and index satisfy the `shape_at` contract (see above).
                    let shape = unsafe { self.shape_at(shapes_ptr, shape_id) };

                    if !should_query_collide(&shape.filter, &filter) {
                        return query_input.max_fraction;
                    }

                    let transform = self.query_shape_transform(shape, origin);

                    let mut local_input = RayCastInput::default();
                    local_input.translation = translation;
                    local_input.max_fraction = query_input.max_fraction;
                    let output = ray_cast_shape(shape, transform, &local_input);

                    if output.hit {
                        b3_assert(output.fraction <= query_input.max_fraction);

                        let id = ShapeID::new(shape.id + 1, world_id, shape.generation);
                        let point = offset_pos(origin, output.point);
                        let material_index =
                            clamp_int(output.material_index, 0, shape.material_count() - 1);
                        let user_material_id = shape.material(material_index).user_material_id;

                        let new_fraction = callback(
                            id,
                            point,
                            output.normal,
                            output.fraction,
                            user_material_id,
                            output.triangle_index,
                            output.child_index,
                        );

                        // The user may return -1 to skip this shape
                        if 0.0 <= new_fraction && new_fraction <= 1.0 {
                            fraction = new_fraction;
                        }

                        return new_fraction;
                    }

                    query_input.max_fraction
                },
            );

            tree_stats.node_visits += tree_result.node_visits;
            tree_stats.leaf_visits += tree_result.leaf_visits;

            if fraction == 0.0 {
                break;
            }

            input.max_fraction = fraction;
        }

        tree_stats
    }

    /// Cast a ray and return the closest hit.
    pub fn cast_ray_closest(&self, origin: Pos, translation: Vec3, filter: QueryFilter) -> RayResult {
        let mut result = RayResult::default();

        {
            let mut visitor = |shape_id: ShapeID,
                               point: Pos,
                               normal: Vec3,
                               fraction: f32,
                               user_material_id: u64,
                               triangle_index: i32,
                               child_index: i32| {
                // Ignore initial overlap
                if fraction == 0.0 {
                    return -1.0;
                }

                result.shape_id = shape_id;
                result.point = point;
                result.normal = normal;
                result.fraction = fraction;
                result.user_material_id = user_material_id;
                result.triangle_index = triangle_index;
                result.child_index = child_index;
                result.hit = true;
                fraction
            };

            let _ = self.cast_ray(origin, translation, filter, &mut visitor);
        }

        result
    }

    /// Cast a shape through the world.
    pub fn shape_cast<F>(
        &self,
        origin: Pos,
        input: ShapeCastInput,
        filter: QueryFilter,
        mut callback: F,
    ) -> TreeStats
    where
        F: FnMut(ShapeID, Pos, Vec3, f32, u64, i32, i32) -> f32,
    {
        let mut tree_stats = TreeStats::default();
        let mut fraction: f32 = 1.0;

        let aabb = offset_aabb(compute_proxy_aabb(input.proxy), origin);

        let shapes_ptr = self.shapes_ptr();
        let world_id = self.world_id;

        for i in 0..BodyType::COUNT as usize {
            let mut box_cast = BoxCastInput::default();
            box_cast.box_ = aabb;
            box_cast.translation = input.translation;
            box_cast.max_fraction = fraction;

            let tree_result = self.broad_phase.trees[i].box_cast(
                box_cast,
                filter.mask_bits,
                false,
                |box_input, _proxy_id, user_data| {
                    let shape_id = user_data as i64 as i32;
                    // SAFETY: pointer and index satisfy the `shape_at` contract (see above).
                    let shape = unsafe { self.shape_at(shapes_ptr, shape_id) };

                    if !should_query_collide(&shape.filter, &filter) {
                        return box_input.max_fraction;
                    }

                    let mut local_input = input;
                    local_input.max_fraction = box_input.max_fraction;

                    let transform = self.query_shape_transform(shape, origin);
                    let output = shape_cast_shape(shape, transform, &local_input);

                    if output.hit {
                        let id = ShapeID::new(shape.id + 1, world_id, shape.generation);
                        let material_index =
                            clamp_int(output.material_index, 0, shape.material_count() - 1);
                        let user_material_id = shape.material(material_index).user_material_id;

                        let new_fraction = callback(
                            id,
                            offset_pos(origin, output.point),
                            output.normal,
                            output.fraction,
                            user_material_id,
                            output.triangle_index,
                            output.child_index,
                        );

                        // The user may return -1 to skip this shape
                        if 0.0 <= new_fraction && new_fraction <= 1.0 {
                            fraction = new_fraction;
                        }

                        return new_fraction;
                    }

                    box_input.max_fraction
                },
            );

            tree_stats.node_visits += tree_result.node_visits;
            tree_stats.leaf_visits += tree_result.leaf_visits;

            if fraction == 0.0 {
                break;
            }
        }

        tree_stats
    }

    /// Cast a shape and return the closest hit.
    pub fn shape_cast_closest(
        &self,
        origin: Pos,
        input: ShapeCastInput,
        filter: QueryFilter,
    ) -> ShapeCastResult {
        let mut result = ShapeCastResult::default();

        {
            let mut visitor = |shape_id: ShapeID,
                               point: Pos,
                               normal: Vec3,
                               fraction: f32,
                               user_material_id: u64,
                               triangle_index: i32,
                               child_index: i32| {
                // Ignore initial overlap
                if fraction == 0.0 {
                    return -1.0;
                }

                result.shape_id = shape_id;
                result.point = point;
                result.normal = normal;
                result.fraction = fraction;
                result.user_material_id = user_material_id;
                result.triangle_index = triangle_index;
                result.child_index = child_index;
                result.hit = true;
                fraction
            };

            let _ = self.shape_cast(origin, input, filter, &mut visitor);
        }

        result
    }
}
