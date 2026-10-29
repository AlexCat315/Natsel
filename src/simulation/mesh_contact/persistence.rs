//! Warm-starting: match new clusters against the previous step's manifolds and points.
//!
//! Translated from `Sources/Natsel/Simulation/MeshContact/Persistence.swift`.

use crate::base::constants as b3;
use crate::collision::manifold::make_feature_id;
use crate::math::functions::{dot, make_matrix_from_quat, mul_mv, sub_pos};
use crate::math::types::WorldTransform;
use crate::simulation::contact_payload::{ContactManifolds, MeshContactScratch};
use crate::simulation::mesh_contact::selection::MeshManifoldCluster;
use crate::types::collision_types::{Manifold, ManifoldPoint};

/// Rebuild the contact's manifold list from the new clusters, carrying impulses over from the
/// previous manifolds by normal similarity and, per point, by feature id plus triangle index.
pub fn persist_mesh_manifolds(
    clusters: &[MeshManifoldCluster],
    previous: &ContactManifolds,
    transform_a: WorldTransform,
    transform_b: WorldTransform,
    scratch: &mut MeshContactScratch,
) -> ContactManifolds {
    // Swift's `var old = previous` shared storage until the first point-consumption write; the
    // eager clone is the same value semantics without the copy-on-write trigger point.
    let mut old = previous.clone();
    if scratch.consumed_manifolds.len() < old.count() {
        scratch
            .consumed_manifolds
            .resize(old.count(), false);
    }
    for i in 0..old.count() {
        scratch.consumed_manifolds[i] = false;
    }
    let matrix = make_matrix_from_quat(transform_b.q);
    let offset = sub_pos(transform_b.p, transform_a.p);
    let mut output = ContactManifolds::new();
    for cluster in clusters {
        let mut manifold = Manifold::default();
        manifold.point_count = cluster.points.len() as i32;
        manifold.normal = mul_mv(matrix, cluster.normal);
        let cluster_normal = mul_mv(matrix, cluster.normal);
        let mut best_dot: f32 = 0.995;
        let mut best_index: Option<usize> = None;
        for j in 0..old.count() {
            if scratch.consumed_manifolds[j] {
                continue;
            }
            let d = dot(old.get(j).normal, cluster_normal);
            if d > best_dot {
                best_index = Some(j);
                best_dot = d;
            }
        }
        if let Some(best_index) = best_index {
            let previous_manifold = old.get(best_index);
            manifold.friction_impulse = previous_manifold.friction_impulse;
            manifold.rolling_impulse = previous_manifold.rolling_impulse;
            manifold.twist_impulse = previous_manifold.twist_impulse;
            scratch.consumed_manifolds[best_index] = true;
        }
        for (j, source) in cluster.points.iter().enumerate() {
            let mut target = ManifoldPoint::default();
            target.anchor_b = mul_mv(matrix, source.point);
            target.anchor_a = target.anchor_b + offset;
            target.separation = source.separation - b3::mesh_rest_offset();
            target.feature_id = make_feature_id(&source.pair);
            target.triangle_index = source.triangle_index;
            if let Some(best_index) = best_index {
                let mut best_manifold = old.get(best_index);
                for k in 0..best_manifold.point_count as usize {
                    let point = best_manifold.points[k];
                    if target.feature_id == point.feature_id
                        && target.triangle_index == point.triangle_index
                    {
                        target.normal_impulse = point.normal_impulse;
                        target.persisted = true;
                        best_manifold.points[k].triangle_index = b3::NULL_INDEX;
                        old.set(best_index, best_manifold);
                        break;
                    }
                }
            }
            manifold.points[j] = target;
        }
        output.append(manifold);
    }
    output
}
