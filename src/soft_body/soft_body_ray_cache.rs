//! A local-frame face BVH, built only when a queried soft body has enough faces to benefit.
//! It retains bounds and face indices, never copies or retains vertex storage.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodyRayCache.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::soft_body::soft_body_runtime::SoftBodyRecord`: fields `generation: u16`,
//!   `spatial_revision: u64`, `settings.faces: Vec<SoftBodyFace>` with `face.vertex.x` /
//!   `.y` / `.z` carrying the Swift `SIMD3<UInt32>` triple, and `positions: Vec<Vec3>`.
//!
//! `DynamicTree` is a Swift class, so Swift's `tree = cached.tree` shared one allocation. The
//! landed Rust port made it an owned value (`Vec`-backed, `Clone`), so the cache owns its tree and
//! `soft_body_queries.rs` borrows `&cache.tree` instead of copying or reference-counting it; the
//! proxies, their insertion order, and the rebuild are bit-for-bit the same either way.

use crate::collision::dynamic_tree::DynamicTree;
use crate::math::functions::{max_vec3, min_vec3};
use crate::math::types::AABB;
use crate::soft_body::soft_body_runtime::SoftBodyRecord;

/// A face BVH in the body's local frame, keyed by soft body id in `World::soft_body_ray_caches`.
#[derive(Clone, Debug)]
pub struct SoftBodyRayCache {
    pub generation: u16,
    pub revision: u64,
    pub tree: DynamicTree,
}

impl SoftBodyRayCache {
    pub fn new(record: &SoftBodyRecord) -> Self {
        let generation = record.generation;
        let revision = record.spatial_revision;
        let mut tree = DynamicTree::new(record.settings.faces.len() as i32);
        for (index, face) in record.settings.faces.iter().enumerate() {
            let a = record.positions[face.vertex.x as usize];
            let b = record.positions[face.vertex.y as usize];
            let c = record.positions[face.vertex.z as usize];
            let bounds = AABB::new(min_vec3(a, min_vec3(b, c)), max_vec3(a, max_vec3(b, c)));
            let _ = tree.create_proxy(bounds, u64::MAX, index as u64);
        }
        let _ = tree.rebuild(true);
        Self {
            generation,
            revision,
            tree,
        }
    }
}
