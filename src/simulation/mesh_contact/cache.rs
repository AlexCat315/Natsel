//! Transient triangle caches and the world-space bounds that own their validity.

use crate::base::constants as b3;
use crate::math::functions::{aabb_contains, aabb_transform, invert_transform, to_relative_transform};
use crate::math::types::{AABB, Pos, Vec3, WorldTransform};
use crate::simulation::triangle_surface::TriangleSurface;
use crate::simulation::world_runtime::TriangleCache;

/// Reusable per-contact triangle cache state.
#[derive(Clone, Debug, Default)]
#[repr(C)]
pub struct MeshContactState {
    pub triangles: Vec<TriangleCache>,
    pub query_bounds: AABB,
    /// Reused query scratch. Keeping the old buffer as a second array lets the next refresh read
    /// cached triangles and fill the other buffer without copying the array.
    pub triangle_scratch: Vec<TriangleCache>,
    pub index_scratch: Vec<i32>,
}

impl MeshContactState {
    /// Re-query the surface unless `bounds` is already inside the cached query bounds.
    pub fn refresh(&mut self, surface: &TriangleSurface, transform: WorldTransform, bounds: AABB) {
        if aabb_contains(self.query_bounds, bounds) {
            return;
        }

        let radius = b3::max_aabb_margin() + b3::speculative_distance();
        let extension_vector = Vec3::new(radius, radius, radius);
        self.query_bounds = AABB::new(
            bounds.lower_bound - extension_vector,
            bounds.upper_bound + extension_vector,
        );

        let mesh_transform = to_relative_transform(transform, Pos::ZERO);
        let local_bounds = aabb_transform(invert_transform(mesh_transform), self.query_bounds);

        self.index_scratch.clear();
        let limit = b3::MAX_MESH_CONTACT_TRIANGLES as usize;
        {
            let index_scratch = &mut self.index_scratch;
            surface.query(local_bounds, |_, _, _, index| {
                if index_scratch.len() == limit {
                    return false;
                }
                index_scratch.push(index);
                index_scratch.len() < limit
            });
        }

        self.triangle_scratch.clear();
        let mut old_index = 0usize;
        for index in self.index_scratch.iter().copied() {
            while old_index < self.triangles.len()
                && self.triangles[old_index].triangle_index < index
            {
                old_index += 1;
            }
            let mut entry = TriangleCache::default();
            entry.triangle_index = index;
            if old_index < self.triangles.len()
                && self.triangles[old_index].triangle_index == index
            {
                entry.cache = self.triangles[old_index].cache;
            }
            self.triangle_scratch.push(entry);
        }

        std::mem::swap(&mut self.triangles, &mut self.triangle_scratch);
    }
}
