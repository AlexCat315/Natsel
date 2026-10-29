//! Triangle-level mesh collision: survivor scan and the per-triangle contact kernels.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::collision::internal_math::{make_normal_from_points, Triangle};
use crate::collision::triangle_manifold::{
    collide_triangle_and_capsule, collide_triangle_and_hull, collide_triangle_and_sphere,
};
use crate::math::functions::{
    inv_mul_world_transforms, make_matrix_from_quat, max_vec3, min_vec3, mul_mv,
};
use crate::math::types::{Vec3, WorldTransform};
use crate::simulation::body_api::ContactFlags;
use crate::simulation::contact_payload::MeshContactScratch;
use crate::simulation::shape_runtime::{Shape, ShapeGeometry};
use crate::simulation::triangle_surface::TriangleSurface;
use crate::simulation::world_runtime::{Contact, ContactCache};
use crate::types::collision_types::{
    LocalManifold, LocalManifoldPoint, SATCache, SeparatingFeature, TriangleFeature,
};
use crate::types::queries::SimplexCache;

/// A borrowed run of `[start, start + len)` triangle-manifold points inside the worker's point
/// arena. The Rust form of Swift's `ArraySlice<LocalManifoldPoint>`: `Copy` like the Swift slice
/// header, so manifolds can store their point views without reference counting.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct MeshPointSlice {
    base: *const LocalManifoldPoint,
    len: usize,
}

// SAFETY: mirrors `unsafe impl Send for LocalManifold`; the slice only ever addresses the
// `MeshContactScratch` point arena of the worker that owns the manifold records.
unsafe impl Send for MeshPointSlice {}

impl Default for MeshPointSlice {
    #[inline]
    fn default() -> Self {
        Self {
            base: std::ptr::null(),
            len: 0,
        }
    }
}

impl MeshPointSlice {
    /// The empty slice (`points: []` in the Swift initializer).
    pub const EMPTY: Self = Self {
        base: std::ptr::null(),
        len: 0,
    };

    #[inline]
    pub fn new(slice: &[LocalManifoldPoint]) -> Self {
        Self {
            base: slice.as_ptr(),
            len: slice.len(),
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.len
    }

    #[inline]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Rebuild the arena view. The empty case returns a real empty slice so no null pointer is
    /// ever passed to `from_raw_parts`.
    #[inline]
    pub fn as_slice(&self) -> &[LocalManifoldPoint] {
        if self.len == 0 {
            return &[];
        }
        // SAFETY: a non-empty slice was built by `new` from a live arena range that outlives the
        // manifold records referencing it (the arena only grows between refreshes, and every
        // producer clears the manifold lists before rewriting the arena, exactly as Swift's
        // slice-sharing rules required).
        unsafe { core::slice::from_raw_parts(self.base, self.len) }
    }

    #[inline]
    pub fn iter(&self) -> core::slice::Iter<'_, LocalManifoldPoint> {
        self.as_slice().iter()
    }
}

impl core::ops::Index<usize> for MeshPointSlice {
    type Output = LocalManifoldPoint;

    #[inline]
    fn index(&self, index: usize) -> &LocalManifoldPoint {
        &self.as_slice()[index]
    }
}

/// One triangle's local manifold plus the mesh identity needed for selection and clustering.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct MeshTriangleManifold {
    pub normal: Vec3,
    pub triangle_normal: Vec3,
    pub points: MeshPointSlice,
    pub triangle_index: i32,
    pub vertices: [i32; 3],
    pub squared_distance: f32,
    pub feature: TriangleFeature,
    pub flags: i32,
    pub point_range: core::ops::Range<usize>,
}

/// A triangle that survived the hull-bounds cull, with its transformed vertices.
#[derive(Clone, Copy, Debug)]
pub struct MeshSurvivor {
    pub cache_index: usize,
    pub triangle: Triangle,
    pub points: [Vec3; 3],
}

impl crate::simulation::world_runtime::World {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn collide_mesh_triangles(
        &self,
        contact: &mut Contact,
        surface: &TriangleSurface,
        transform_a: WorldTransform,
        shape_b: &Shape,
        transform_b: WorldTransform,
        is_fast: bool,
        scratch: &mut MeshContactScratch,
    ) -> Vec<MeshTriangleManifold> {
        let transform = inv_mul_world_transforms(transform_b, transform_a);
        let matrix = make_matrix_from_quat(transform.q);
        let speculative =
            contact.flags & ContactFlags::ENABLE_SPECULATIVE_POINTS.0 != 0;
        let hull = shape_b.hull();
        let distance: f32 = if speculative {
            b3::speculative_distance()
        } else {
            0.0
        };
        let margin = Vec3::new(distance, distance, distance);
        scratch.survivors.clear();
        // Release last contact's manifold slices before overwriting the point arena. Keeping them
        // alive here would make the arena multiply referenced and copy on the first point write.
        scratch.selected_manifolds.clear();
        let entry_count = contact.mesh_contact().triangles.len();
        for index in 0..entry_count {
            let entry = contact.mesh_contact().triangles[index];
            let triangle = surface.triangle(entry.triangle_index);
            // Each scalar lane uses exactly the nested adds in the four-wide transform.
            let points: [Vec3; 3] = [
                mul_mv(matrix, triangle.vertices[0]) + transform.p,
                mul_mv(matrix, triangle.vertices[1]) + transform.p,
                mul_mv(matrix, triangle.vertices[2]) + transform.p,
            ];
            if let Some(hull) = hull {
                let lower = min_vec3(points[0], min_vec3(points[1], points[2])) - margin;
                let upper = max_vec3(points[0], max_vec3(points[1], points[2])) + margin;
                let bounds = hull.aabb();
                if upper.x < bounds.lower_bound.x
                    || lower.x > bounds.upper_bound.x
                    || upper.y < bounds.lower_bound.y
                    || lower.y > bounds.upper_bound.y
                    || upper.z < bounds.lower_bound.z
                    || lower.z > bounds.upper_bound.z
                {
                    contact.mesh_contact_mut().triangles[index].cache =
                        ContactCache::Sat(SATCache::default());
                    continue;
                }
            }
            scratch.survivors.push(MeshSurvivor {
                cache_index: index,
                triangle,
                points,
            });
        }
        let triangle_count = contact.mesh_contact().triangles.len();
        let capacity = 32 * triangle_count;
        let mut total_point_count = 0usize;
        scratch.manifolds.clear();
        if scratch.points.len() < capacity {
            scratch.points.resize(capacity, LocalManifoldPoint::default());
        }
        // SAFETY: the arena is grown to `capacity` above and never resized or pushed to again in
        // this call, so the base pointer stays valid for the whole loop. The loop breaks before
        // `total_point_count + 3` reaches `capacity`, and every kernel writes at most its
        // advertised `available` points starting at `manifold.points`, so the pointer arithmetic
        // below stays inside the allocation — the same contract Swift's buffer pointer had.
        let point_base: *mut LocalManifoldPoint = scratch.points.as_mut_ptr();
        {
            for survivor_index in 0..scratch.survivors.len() {
                let survivor = scratch.survivors[survivor_index];
                if total_point_count + 3 >= capacity {
                    break;
                }
                let index = survivor.cache_index;
                let triangle = survivor.triangle;
                let triangle_index =
                    contact.mesh_contact().triangles[index].triangle_index;
                let mut manifold = LocalManifold::default();
                manifold.points = point_base.add(total_point_count);
                manifold.triangle_flags = triangle.flags;
                let available = (capacity - total_point_count) as i32;
                match &shape_b.geometry {
                    ShapeGeometry::Capsule(capsule) => {
                        let mut cache = SimplexCache::default();
                        if let ContactCache::Simplex(saved) =
                            contact.mesh_contact().triangles[index].cache
                        {
                            cache = saved;
                        }
                        collide_triangle_and_capsule(
                            &mut manifold,
                            available,
                            &survivor.points,
                            *capsule,
                            &mut cache,
                        );
                        contact.mesh_contact_mut().triangles[index].cache =
                            ContactCache::Simplex(cache);
                    }
                    ShapeGeometry::Hull(hull) => {
                        let mut cache = SATCache::default();
                        if let ContactCache::Sat(saved) =
                            contact.mesh_contact().triangles[index].cache
                        {
                            cache = saved;
                        }
                        if is_fast && cache.type_ == SeparatingFeature::EdgePairAxis {
                            cache = SATCache::default();
                        }
                        collide_triangle_and_hull(
                            &mut manifold,
                            available,
                            survivor.points[0],
                            survivor.points[1],
                            survivor.points[2],
                            triangle.flags,
                            hull,
                            &mut cache,
                            speculative,
                        );
                        scratch.sat_calls += 1;
                        scratch.sat_cache_hits += cache.hit as i32;
                        contact.mesh_contact_mut().triangles[index].cache =
                            ContactCache::Sat(cache);
                    }
                    ShapeGeometry::Sphere(sphere) => {
                        collide_triangle_and_sphere(
                            &mut manifold,
                            available,
                            &survivor.points,
                            *sphere,
                        );
                    }
                    _ => {
                        b3_assert(false);
                        continue;
                    }
                }
                if manifold.point_count > 0 {
                    // Triangle kernels fill point/separation/features. Mesh clustering supplies
                    // the triangle id; initialize it explicitly without clearing the unused tail.
                    for i in 0..manifold.point_count as usize {
                        // SAFETY: the kernel reported `point_count` points, all inside the
                        // `available` window below `capacity` of the arena the base was taken from.
                        unsafe {
                            (*point_base.add(total_point_count + i)).triangle_index = 0;
                        }
                    }
                    scratch.manifolds.push(MeshTriangleManifold {
                        normal: manifold.normal,
                        triangle_normal: make_normal_from_points(
                            survivor.points[0],
                            survivor.points[1],
                            survivor.points[2],
                        ),
                        points: MeshPointSlice::EMPTY,
                        triangle_index,
                        vertices: [triangle.i1, triangle.i2, triangle.i3],
                        squared_distance: manifold.squared_distance,
                        feature: manifold.feature,
                        flags: triangle.flags,
                        point_range: total_point_count
                            ..total_point_count + manifold.point_count as usize,
                    });
                    total_point_count += manifold.point_count as usize;
                }
            }
        }
        // Form shared, immutable slices only after writing the whole arena. Taking a slice
        // inside the collide loop would make subsequent point writes copy the arena.
        for i in 0..scratch.manifolds.len() {
            let range = scratch.manifolds[i].point_range.clone();
            let slice = &scratch.points[range];
            scratch.manifolds[i].points = MeshPointSlice::new(slice);
        }
        std::mem::take(&mut scratch.manifolds)
    }
}
