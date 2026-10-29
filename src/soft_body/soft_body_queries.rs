//! Ray casts against soft body surfaces.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodyQueries.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::soft_body::soft_body_runtime`: the `SoftBodyID` handle (`SoftBodyID::new(index1,
//!   world0, generation)`, Swift's `SoftBodyID(index1:world0:generation:)`), `SoftBodyRayResult`
//!   with fields `soft_body_id: SoftBodyID`, `point: Pos`, `normal: Vec3`, `fraction: f32`,
//!   `face_index: i32`, `hit: bool`, the `World` method `soft_body_index(id) -> Option<usize>`, and
//!   `SoftBodyRecord::{id, generation, spatial_revision: u64, origin: Pos, bounds: AABB,
//!   filter: Filter, faces_double_sided: bool, positions: Vec<Vec3>, settings.faces}`.
//!   `SoftBodyRayResult::default()` must reproduce Swift's member defaults - notably
//!   `fraction = 1` and `face_index = -1` - the way the hand-written `impl Default` used for the
//!   other `*Def`/`*Result` types does; the un-hit result returned by `cast_ray_closest_soft_body`
//!   starts at fraction 1.0.
//! - `crate::soft_body::soft_body_ray_cache::SoftBodyRayCache` (this batch).
//!
//! Borrowing adaptation (behaviour identical): Swift passed `SoftBodyRecord` - a struct of
//! copy-on-write arrays - by value into the private helper, so the helper held a snapshot while it
//! inserted into `softBodyRayCaches`. Rust cannot borrow `self.soft_bodies` across a `&mut self`
//! call, so the helper takes the record's index and reborrows the record from the world; nothing
//! here writes the record's arrays between the snapshot and the reads, so it observes exactly the
//! values Swift's copy did. `cast_ray_closest_soft_body` copies the two scalars it still needs
//! after the call (`id`, `generation`) into locals, matching the same snapshot. The face pointer
//! view Swift built with `positions.withUnsafeBufferPointer` is a plain slice borrow, so no unsafe
//! code is needed in this file.
//!
//! `ray_cast_aabb` here is Swift's soft-body slab test; it is a different function from
//! `crate::collision::aabb::ray_cast_aabb` and is not a re-export of it.

use crate::base::constants as b3;
use crate::collision::dynamic_tree::DynamicTree;
use crate::math::functions::{
    abs_float, cross, dot, max_float, min_float, normalize, pos_mul_add, sub_pos,
};
use crate::math::types::{AABB, Pos, Vec3};
use crate::simulation::shape_runtime::should_query_collide;
use crate::simulation::world_runtime::World;
use crate::soft_body::soft_body_ray_cache::SoftBodyRayCache;
use crate::soft_body::soft_body_runtime::{SoftBodyID, SoftBodyRayResult};
use crate::types::queries::RayCastInput;
use crate::types::shape_def::QueryFilter;

impl World {
    /// Closest hit of a ray on one soft body's faces.
    pub fn cast_ray_soft_body(
        &mut self,
        soft_body_id: SoftBodyID,
        origin: Pos,
        translation: Vec3,
    ) -> SoftBodyRayResult {
        let Some(index) = self.soft_body_index(soft_body_id) else {
            return SoftBodyRayResult::default();
        };
        let mut result = self.ray_cast_soft_body_record(index, origin, translation, 1.0);
        if result.hit {
            result.soft_body_id = soft_body_id;
        }
        result
    }

    /// Closest hit of a ray on any soft body whose filter accepts the query.
    ///
    /// Swift defaulted `filter` to `.defaultFilter`; ported callers pass
    /// `QueryFilter::default_filter()` explicitly.
    pub fn cast_ray_closest_soft_body(
        &mut self,
        origin: Pos,
        translation: Vec3,
        filter: QueryFilter,
    ) -> SoftBodyRayResult {
        let mut best = SoftBodyRayResult::default();
        let mut input = RayCastInput::default();
        input.translation = translation;
        input.max_fraction = 1.0;
        for i in 0..self.soft_bodies.len() {
            if self.soft_bodies[i].id == b3::NULL_INDEX {
                continue;
            }
            let record = &self.soft_bodies[i];
            if !should_query_collide(&record.filter, &filter) {
                continue;
            }
            input.max_fraction = best.fraction;
            input.origin = sub_pos(origin, record.origin);
            if !ray_cast_aabb(record.bounds, input) {
                continue;
            }
            let record_id = record.id;
            let record_generation = record.generation;
            let hit = self.ray_cast_soft_body_record(i, origin, translation, best.fraction);
            if hit.hit && hit.fraction < best.fraction {
                best = hit;
                best.soft_body_id =
                    SoftBodyID::new(record_id + 1, self.world_id, record_generation);
            }
        }
        best
    }

    fn ray_cast_soft_body_record(
        &mut self,
        index: usize,
        origin: Pos,
        translation: Vec3,
        max_fraction: f32,
    ) -> SoftBodyRayResult {
        let mut result = SoftBodyRayResult::default();
        result.fraction = max_fraction;
        // Disjoint field borrows: `record` reads the body storage while the face BVH below is
        // built and inserted into the cache map, exactly as Swift's by-value record copy let the
        // dictionary mutation happen alongside it.
        let Self {
            soft_bodies,
            soft_body_ray_caches,
            enable_soft_body_query_acceleration,
            ..
        } = self;
        let record = &soft_bodies[index];
        let double_sided = record.faces_double_sided;
        let local_origin = sub_pos(origin, record.origin);
        let mut tree: Option<&DynamicTree> = None;
        if *enable_soft_body_query_acceleration && record.settings.faces.len() >= 32 {
            // Swift reused `cached.tree` when both stamps matched and otherwise rebuilt in place;
            // the stamp test is hoisted into a flag so no borrow of the map survives the rebuild.
            let reuse = match soft_body_ray_caches.get(&record.id) {
                Some(cached) => {
                    cached.generation == record.generation
                        && cached.revision == record.spatial_revision
                }
                None => false,
            };
            if !reuse {
                let cache = SoftBodyRayCache::new(record);
                soft_body_ray_caches.insert(record.id, cache);
            }
            tree = soft_body_ray_caches.get(&record.id).map(|cached| &cached.tree);
        }
        let faces = &record.settings.faces;
        let x = &record.positions;
        let mut test_face = move |f: usize, result: &mut SoftBodyRayResult| {
            let face = &faces[f];
            let a = x[face.vertex.x as usize];
            let b = x[face.vertex.y as usize];
            let c = x[face.vertex.z as usize];
            if let Some((t, normal)) = ray_triangle(local_origin, translation, a, b, c, double_sided)
            {
                if t < result.fraction
                    || (result.hit && t == result.fraction && (f as i32) < result.face_index)
                {
                    result.hit = true;
                    result.fraction = t;
                    result.face_index = f as i32;
                    result.normal = normal;
                    result.point = pos_mul_add(origin, t, translation);
                }
            }
        };
        if let Some(tree) = tree {
            let input = RayCastInput::new(local_origin, translation, max_fraction);
            let _ = tree.ray_cast(input, u64::MAX, false, |_sub_input, _proxy_id, data| {
                test_face(data as usize, &mut result);
                result.fraction
            });
        } else {
            for f in 0..faces.len() {
                test_face(f, &mut result);
            }
        }
        result
    }
}

/// Slab test of a ray segment against a box.
pub(crate) fn ray_cast_aabb(box_: AABB, input: RayCastInput) -> bool {
    let mut t_min: f32 = 0.0;
    let mut t_max = input.max_fraction;
    let p = input.origin;
    let d = input.translation;
    for axis in 0..3 {
        let pa = if axis == 0 { p.x } else if axis == 1 { p.y } else { p.z };
        let da = if axis == 0 { d.x } else if axis == 1 { d.y } else { d.z };
        let lo = if axis == 0 {
            box_.lower_bound.x
        } else if axis == 1 {
            box_.lower_bound.y
        } else {
            box_.lower_bound.z
        };
        let hi = if axis == 0 {
            box_.upper_bound.x
        } else if axis == 1 {
            box_.upper_bound.y
        } else {
            box_.upper_bound.z
        };
        if abs_float(da) < 1.0e-12 {
            if pa < lo || pa > hi {
                return false;
            }
            continue;
        }
        let inv = 1.0 / da;
        let mut t1 = (lo - pa) * inv;
        let mut t2 = (hi - pa) * inv;
        if t1 > t2 {
            core::mem::swap(&mut t1, &mut t2);
        }
        t_min = max_float(t_min, t1);
        t_max = min_float(t_max, t2);
        if t_min > t_max {
            return false;
        }
    }
    true
}

/// Möller–Trumbore. Returns the fraction along `translation` and the face normal facing the ray.
#[inline(always)]
pub(crate) fn ray_triangle(
    origin: Vec3,
    translation: Vec3,
    a: Vec3,
    b: Vec3,
    c: Vec3,
    double_sided: bool,
) -> Option<(f32, Vec3)> {
    let e1 = b - a;
    let e2 = c - a;
    let normal = cross(e1, e2);
    let facing = dot(normal, translation);
    if !double_sided && facing >= 0.0 {
        return None;
    }
    let p = cross(translation, e2);
    let determinant = dot(e1, p);
    if abs_float(determinant) <= 1.0e-12 {
        return None;
    }
    let inv_det = 1.0 / determinant;
    let s = origin - a;
    let u = dot(s, p) * inv_det;
    if !(u >= 0.0 && u <= 1.0) {
        return None;
    }
    let q = cross(s, e1);
    let v = dot(translation, q) * inv_det;
    if !(v >= 0.0 && u + v <= 1.0) {
        return None;
    }
    let t = dot(e2, q) * inv_det;
    if !(t >= 0.0 && t <= 1.0) {
        return None;
    }
    let mut n = normalize(normal);
    if facing > 0.0 {
        n = -n;
    }
    Some((t, n))
}
