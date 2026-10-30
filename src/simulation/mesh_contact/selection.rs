//! Mesh manifold selection by feature keys and clustering by normal.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::containers::quick_sort::quick_sort;
use crate::math::functions::{dot, max_int, min_float, min_int};
use crate::math::types::Vec3;
use crate::simulation::contact_payload::MeshContactScratch;
use crate::types::collision_types::{LocalManifoldPoint, TriangleFeature};

use super::reduction::reduce_mesh_cluster;
use super::triangles::MeshTriangleManifold;

/// Distinct edges or vertices already claimed by an accepted manifold. The capacity is the mesh
/// feature budget; once it is full a new key still reports as unseen so the manifold is kept.
#[derive(Clone, Debug)]
struct MeshFeatureKeys {
    keys: [u64; 64],
    count: usize,
}

impl Default for MeshFeatureKeys {
    fn default() -> Self {
        Self {
            keys: [0; 64],
            count: 0,
        }
    }
}

impl MeshFeatureKeys {
    /// Add `key`, returning true when it was not already present.
    fn add(&mut self, key: u64) -> bool {
        for i in 0..self.count {
            if self.keys[i] == key {
                return false;
            }
        }
        if self.count == 64 {
            return true;
        }
        self.keys[self.count] = key;
        self.count += 1;
        true
    }

    fn contains(&self, key: u64) -> bool {
        for i in 0..self.count {
            if self.keys[i] == key {
                return true;
            }
        }
        false
    }
}

/// Pack an unordered vertex pair into one 64-bit key.
#[inline]
fn mesh_edge_key(a: i32, b: i32) -> u64 {
    let lower = min_int(a, b) as u32;
    let upper = max_int(a, b) as u32;
    (lower as u64) << 32 | (upper as u64)
}

/// Keep the manifolds that touch a new feature, dropping duplicates of features an accepted
/// manifold already owns. The result is stored in the worker's reusable scratch buffer.
pub fn select_mesh_manifolds<'s>(
    manifolds: &[MeshTriangleManifold],
    sphere: bool,
    scratch: &'s mut MeshContactScratch,
) -> &'s [MeshTriangleManifold] {
    let mut edges = MeshFeatureKeys::default();
    let mut vertices = MeshFeatureKeys::default();
    let mut accepted = [0i32; 512];
    let mut tentative = [0i32; 512];
    let mut accepted_count = 0usize;
    let mut tentative_count = 0usize;

    for (index, manifold) in manifolds.iter().enumerate() {
        let mut accept = manifold.feature == TriangleFeature::FeatureTriangleFace;
        if manifold.feature == TriangleFeature::FeatureHullFace {
            if dot(manifold.triangle_normal, manifold.normal) > 0.5 {
                accept = true;
            } else {
                let mut separation = manifold.points[0].separation;
                for point in manifold.points.iter().skip(1) {
                    separation = min_float(separation, point.separation);
                }
                accept = separation < -2.0 * b3::linear_slop();
            }
        }
        if accept {
            let v = manifold.vertices;
            let _ = edges.add(mesh_edge_key(v[0], v[1]));
            let _ = edges.add(mesh_edge_key(v[1], v[2]));
            let _ = edges.add(mesh_edge_key(v[2], v[0]));
            let _ = vertices.add(v[0] as u64);
            let _ = vertices.add(v[1] as u64);
            let _ = vertices.add(v[2] as u64);
            accepted[accepted_count] = index as i32;
            accepted_count += 1;
        } else {
            tentative[tentative_count] = index as i32;
            tentative_count += 1;
        }
    }

    if sphere {
        // SAFETY: the comparator reads `tentative` while the swapper reorders it, exactly as the
        // two Swift closures captured the one inline buffer. The sort is synchronous, the two
        // closures only ever touch this local array, and no other borrow of it exists while the
        // sort runs, so the raw pointer aliasing is the same aliasing Swift relied on.
        let tentative_ptr = tentative.as_mut_ptr();
        quick_sort(
            tentative_count as i32,
            |i, j| unsafe {
                let a = *tentative_ptr.add(i) as usize;
                let b = *tentative_ptr.add(j) as usize;
                manifolds[a].squared_distance < manifolds[b].squared_distance
            },
            |i, j| unsafe {
                if i != j {
                    std::ptr::swap(tentative_ptr.add(i), tentative_ptr.add(j));
                }
            },
        );

        for tentative_index in 0..tentative_count {
            let index = tentative[tentative_index] as usize;
            let m = &manifolds[index];
            let v = m.vertices;
            let edge1 = edges.add(mesh_edge_key(v[0], v[1]));
            let edge2 = edges.add(mesh_edge_key(v[1], v[2]));
            let edge3 = edges.add(mesh_edge_key(v[2], v[0]));
            let vertex1 = vertices.add(v[0] as u64);
            let vertex2 = vertices.add(v[1] as u64);
            let vertex3 = vertices.add(v[2] as u64);
            let collide = match m.feature {
                TriangleFeature::FeatureEdge1 => edge1,
                TriangleFeature::FeatureEdge2 => edge2,
                TriangleFeature::FeatureEdge3 => edge3,
                TriangleFeature::FeatureVertex1 => vertex1,
                TriangleFeature::FeatureVertex2 => vertex2,
                TriangleFeature::FeatureVertex3 => vertex3,
                _ => {
                    b3_assert(false);
                    false
                }
            };
            if collide {
                accepted[accepted_count] = index as i32;
                accepted_count += 1;
            }
        }
    } else {
        for tentative_index in 0..tentative_count {
            let index = tentative[tentative_index] as usize;
            let m = &manifolds[index];
            if m.flags & 0x77 == 0x77 {
                continue;
            }
            let v = m.vertices;
            if m.flags & 0x11 == 0x11 && edges.contains(mesh_edge_key(v[0], v[1])) {
                continue;
            }
            if m.flags & 0x22 == 0x22 && edges.contains(mesh_edge_key(v[1], v[2])) {
                continue;
            }
            if m.flags & 0x44 == 0x44 && edges.contains(mesh_edge_key(v[2], v[0])) {
                continue;
            }
            accepted[accepted_count] = index as i32;
            accepted_count += 1;
        }
    }

    scratch.selected_manifolds = Vec::new();
    scratch
        .selected_manifolds
        .reserve(accepted_count);
    for i in 0..accepted_count {
        let index = accepted[i] as usize;
        // Swift's `MeshTriangleManifold` is a value type, so this is a copy there. It cannot be
        // `Copy` in Rust because `point_range` is a `Range<usize>`.
        scratch.selected_manifolds.push(manifolds[index].clone());
    }
    &scratch.selected_manifolds
}

/// Manifolds that share a contact normal, reduced to at most four points each.
#[derive(Clone, Debug, Default)]
pub struct MeshManifoldCluster {
    pub normal: Vec3,
    pub triangle_normal: Vec3,
    pub points: Vec<LocalManifoldPoint>,
}

pub fn cluster_mesh_manifolds<'s>(
    manifolds: &[MeshTriangleManifold],
    scratch: &'s mut MeshContactScratch,
) -> &'s [MeshManifoldCluster] {
    let mut cluster_count = 0usize;

    for i in 0..scratch.clusters.len() {
        scratch.clusters[i].points.clear();
    }

    for manifold in manifolds {
        let mut selected: Option<usize> = None;
        for index in 0..cluster_count {
            let manifold_angle = dot(scratch.clusters[index].normal, manifold.normal);
            let triangle_angle =
                dot(scratch.clusters[index].triangle_normal, manifold.triangle_normal);
            if manifold_angle <= 0.996 || triangle_angle <= 0.996 {
                continue;
            }
            selected = Some(index);
            break;
        }

        let index = match selected {
            Some(index) => index,
            None => {
                let index = cluster_count;
                if cluster_count == scratch.clusters.len() {
                    scratch.clusters.push(MeshManifoldCluster::default());
                }
                scratch.clusters[index].normal = manifold.normal;
                scratch.clusters[index].triangle_normal = manifold.triangle_normal;
                cluster_count += 1;
                index
            }
        };

        for point in manifold.points.iter() {
            let mut point = *point;
            point.triangle_index = manifold.triangle_index;
            scratch.clusters[index].points.push(point);
        }
    }

    for i in 0..cluster_count {
        let normal = scratch.clusters[i].triangle_normal;
        reduce_mesh_cluster(
            &mut scratch.clusters[i].points,
            normal,
            &mut scratch.projected_points,
        );
    }

    &scratch.clusters[..cluster_count]
}
