//! Mesh patch point projection and the four-point reduction.

use crate::base::constants as b3;
use crate::collision::internal_math::{cross2, distance_squared2};
use crate::math::functions::{abs_float, cross, dot, max_float, perp};
use crate::math::types::{Vec2, Vec3};
use crate::types::collision_types::LocalManifoldPoint;

/// A cluster point projected onto the contact plane basis, remembering where it came from.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct MeshPatchPoint {
    pub p: Vec2,
    pub separation: f32,
    pub original_index: usize,
}

/// Is `score`/`separation` a better candidate than the current best? Distance dominates within its
/// tolerance, separation breaks the tie.
#[inline(always)]
fn better_mesh_cull_candidate(
    score: f32,
    separation: f32,
    best_score: f32,
    best_separation: f32,
    score_tolerance: f32,
    separation_tolerance: f32,
) -> bool {
    if score > best_score + score_tolerance {
        return true;
    }
    if score < best_score - score_tolerance {
        return false;
    }
    separation < best_separation - separation_tolerance
}

/// Four-point reduction, including tie tolerances and swap removal order.
///
/// `points` is the working pool; its length is the working count, exactly like the
/// `UnsafeMutableBufferPointer` the Swift version took, and removals move the last live slot into
/// the removed one and shorten the live range `[0, count)` the same way `points[i] = points[count -
/// 1]; count -= 1` did.
#[inline(always)]
fn cull_mesh_points_pool(
    points: &mut [MeshPatchPoint],
    selected: &mut [MeshPatchPoint; 4],
) -> usize {
    if points.len() <= 1 {
        if points.len() == 1 {
            selected[0] = points[0];
        }
        return points.len();
    }

    let tolerance: f32 = 0.25 * b3::linear_slop();
    let tolerance_squared = tolerance * tolerance;
    let separation_tolerance = b3::linear_slop();
    let mut count = points.len();
    let mut best_score: f32 = 0.0;
    let mut best_separation = f32::MAX;
    // Swift's `-1` sentinels. They are only ever read after the `best_score < tolerance_squared`
    // guard below, which returns first when no pair was accepted.
    let mut best_index1 = usize::MAX;
    let mut best_index2 = usize::MAX;

    for i in 0..count {
        let p1 = points[i].p;
        for j in (i + 1)..count {
            let score = distance_squared2(p1, points[j].p);
            let separation = points[i].separation + points[j].separation;
            if better_mesh_cull_candidate(
                score,
                separation,
                best_score,
                best_separation,
                tolerance_squared,
                separation_tolerance,
            ) {
                best_index1 = i;
                best_index2 = j;
                best_score = score;
                best_separation = separation;
            }
        }
    }

    if best_score < tolerance_squared {
        let mut deepest = 0usize;
        for i in 1..count {
            if points[i].separation < points[deepest].separation {
                deepest = i;
            }
        }
        selected[0] = points[deepest];
        return 1;
    }

    selected[0] = points[best_index1];
    selected[1] = points[best_index2];
    points[best_index2] = points[count - 1];
    points[best_index1] = points[count - 2];
    count -= 2;
    if count == 0 {
        return 2;
    }

    let a = selected[0].p;
    let mut b = selected[1].p;
    let mut ba = Vec2::new(b.x - a.x, b.y - a.y);
    best_score = 0.0;
    best_separation = f32::MAX;
    let mut best_index = usize::MAX;
    let mut best_signed_area: f32 = 0.0;

    for i in 0..count {
        let p = points[i].p;
        let signed_area = cross2(ba, Vec2::new(p.x - a.x, p.y - a.y));
        let score = abs_float(signed_area);
        if better_mesh_cull_candidate(
            score,
            points[i].separation,
            best_score,
            best_separation,
            tolerance_squared,
            separation_tolerance,
        ) {
            best_signed_area = signed_area;
            best_score = score;
            best_separation = points[i].separation;
            best_index = i;
        }
    }

    if best_index == usize::MAX {
        return 2;
    }

    selected[2] = points[best_index];
    if count == 1 {
        return 3;
    }

    points[best_index] = points[count - 1];
    count -= 1;

    let mut c = selected[2].p;
    if best_signed_area < 0.0 {
        std::mem::swap(&mut b, &mut c);
        ba = Vec2::new(b.x - a.x, b.y - a.y);
    }
    let cb = Vec2::new(c.x - b.x, c.y - b.y);
    let ac = Vec2::new(a.x - c.x, a.y - c.y);
    best_score = 0.0;
    best_separation = f32::MAX;
    best_index = usize::MAX;

    for i in 0..count {
        let p = points[i].p;
        let u1 = cross2(Vec2::new(p.x - a.x, p.y - a.y), ba);
        let u2 = cross2(Vec2::new(p.x - b.x, p.y - b.y), cb);
        let u3 = cross2(Vec2::new(p.x - c.x, p.y - c.y), ac);
        let score = max_float(u1, max_float(u2, u3));
        if better_mesh_cull_candidate(
            score,
            points[i].separation,
            best_score,
            best_separation,
            tolerance_squared,
            separation_tolerance,
        ) {
            best_score = score;
            best_separation = points[i].separation;
            best_index = i;
        }
    }

    if best_index != usize::MAX {
        selected[3] = points[best_index];
        return 4;
    }
    3
}

/// Standalone/reference-test entry point. Production projects into the worker's reusable buffer.
pub fn cull_mesh_points(source: &[MeshPatchPoint]) -> Vec<MeshPatchPoint> {
    let mut points: Vec<MeshPatchPoint> = source.to_vec();
    let mut selected = [MeshPatchPoint::default(); 4];
    let count = cull_mesh_points_pool(&mut points, &mut selected);
    let mut output: Vec<MeshPatchPoint> = Vec::with_capacity(count);
    for i in 0..count {
        output.push(selected[i]);
    }
    output
}

/// Reduce in place, preserving each cluster's allocation for the next contact. The projected
/// scratch has no shared slices, so neither projection nor swap removal needs an Array copy.
pub fn reduce_mesh_cluster(
    points: &mut Vec<LocalManifoldPoint>,
    normal: Vec3,
    projected: &mut Vec<MeshPatchPoint>,
) {
    let count = points.len();
    if count <= 1 {
        return;
    }

    let u = perp(normal);
    let v = cross(normal, u);
    let origin = points[0].point;

    if projected.len() < count {
        projected.resize(count, MeshPatchPoint::default());
    }

    let mut selected = [MeshPatchPoint::default(); 4];
    let selected_count = {
        let scratch = &mut projected[..count];
        for i in 0..count {
            let point = points[i];
            let d = point.point - origin;
            scratch[i] = MeshPatchPoint {
                p: Vec2::new(dot(d, u), dot(d, v)),
                separation: point.separation,
                original_index: i,
            };
        }
        cull_mesh_points_pool(scratch, &mut selected)
    };

    // Fetch all chosen points before overwriting the source prefix: selected indices may alias it.
    let mut output = [LocalManifoldPoint::default(); 4];
    for i in 0..selected_count {
        output[i] = points[selected[i].original_index];
    }
    for i in 0..selected_count {
        points[i] = output[i];
    }
    points.truncate(selected_count);
}
