//! Ported from Tests/NatselTests/Collision/SeparatingAxisOracleTests.swift.
//!
//! Brute-force separating-axis oracle. Every face axis and every edge pair is tried both ways and
//! the winner must agree with `compute_separating_axis` to within the Swift tolerances.

use crate::base::constants as b3;
use crate::collision::convex_manifold::compute_separating_axis_temp;
use crate::collision::hull::{
    make_box_hull, make_offset_box_hull, make_transformed_box_hull, Hull,
};
use crate::collision::manifold::get_best_axis;
use crate::math::functions::{
    cross, dot, length, make_quat_from_axis_angle, max_float, min_float, mul_add, normalize,
    rotate_vector, transform_point,
};
use crate::math::types::{Quat, Transform, Vec3};
use crate::tests::world_position_assertions::abs_float;
use crate::types::collision_types::SeparatingFeature;

struct Random {
    seed: u32,
}

impl Random {
    fn new(seed: u32) -> Self {
        Self { seed }
    }

    fn next(&mut self) -> u32 {
        self.seed ^= self.seed << 13;
        self.seed ^= self.seed >> 17;
        self.seed ^= self.seed << 5;
        self.seed
    }

    fn float(&mut self, lower: f32, upper: f32) -> f32 {
        let unit = (self.next() >> 8) as f32 / 16_777_216.0;
        lower + (upper - lower) * unit
    }

    fn vector(&mut self, lower: f32, upper: f32) -> Vec3 {
        Vec3::new(
            self.float(lower, upper),
            self.float(lower, upper),
            self.float(lower, upper),
        )
    }

    fn direction(&mut self) -> Vec3 {
        loop {
            let v = self.vector(-1.0, 1.0);
            let len = length(v);
            if 0.1 < len && len < 1.0 {
                return (1.0 / len) * v;
            }
        }
    }

    fn quat(&mut self) -> Quat {
        let direction = self.direction();
        let radians = self.float(0.0, b3::PI);
        make_quat_from_axis_angle(direction, radians)
    }
}

fn exact_quat(axis: Vec3, radians: f32) -> Quat {
    let half = 0.5 * radians;
    Quat::new(half.sin() * axis, half.cos())
}

fn hull_points(hull: &Hull) -> Vec<Vec3> {
    if hull.points().is_empty() {
        return Vec::new();
    }
    (0..hull.vertex_count() as usize)
        .map(|i| hull.points()[i])
        .collect()
}

/// Projection gap: min over B minus max over A along the given normal.
fn separation_along(hull_a: &Hull, hull_b: &Hull, xf_b: Transform, n: Vec3) -> f32 {
    let mut max_a = -f32::MAX;
    for p in hull_points(hull_a) {
        max_a = max_float(max_a, dot(n, p));
    }
    let mut min_b = f32::MAX;
    for p in hull_points(hull_b) {
        min_b = min_float(min_b, dot(n, transform_point(xf_b, p)));
    }
    min_b - max_a
}

/// The true maximum separation over every face axis and edge pair. Undirected axes are tried
/// both ways so the winner is oriented from A to B.
fn oracle_separation(hull_a: &Hull, hull_b: &Hull, xf_b: Transform) -> (f32, Vec3) {
    let mut best = -f32::MAX;
    let mut best_normal = Vec3::ZERO;

    let mut try_axis = |n: Vec3, best: &mut f32, best_normal: &mut Vec3| {
        let s = separation_along(hull_a, hull_b, xf_b, n);
        if s > *best {
            *best = s;
            *best_normal = n;
        }
    };

    // Face axes of A.
    let planes_a = hull_a.planes();
    for i in 0..hull_a.face_count() as usize {
        let n = planes_a[i].normal;
        try_axis(n, &mut best, &mut best_normal);
        try_axis(-n, &mut best, &mut best_normal);
    }
    // Face axes of B, rotated into A's frame.
    let planes_b = hull_b.planes();
    for i in 0..hull_b.face_count() as usize {
        let n = rotate_vector(xf_b.q, planes_b[i].normal);
        try_axis(n, &mut best, &mut best_normal);
        try_axis(-n, &mut best, &mut best_normal);
    }
    // Edge pairs. Half edges store adjacent twins, so the even index and its successor bound
    // one edge.
    let edges_a = hull_a.edges();
    let edges_b = hull_b.edges();
    let p_a = hull_points(hull_a);
    let p_b = hull_points(hull_b);
    let mut i = 0usize;
    while i < hull_a.half_edge_count() as usize {
        let dir_a = p_a[edges_a[i + 1].origin as usize] - p_a[edges_a[i].origin as usize];
        let mut j = 0usize;
        while j < hull_b.half_edge_count() as usize {
            let dir_b = rotate_vector(
                xf_b.q,
                p_b[edges_b[j + 1].origin as usize] - p_b[edges_b[j].origin as usize],
            );
            let c = cross(dir_a, dir_b);
            // Near parallel edges give no useful axis and cannot be the true maximum.
            if dot(c, c) >= 1e-10 {
                try_axis(normalize(c), &mut best, &mut best_normal);
            }
            j += 2;
        }
        i += 2;
    }
    (best, best_normal)
}

#[test]
fn axis_oracle() {
    // "the query never overstates the true separation"
    let mut random = Random::new(987_654_321);

    let mut separated = 0;
    let mut penetrating = 0;
    let mut edge_wins = 0;

    for i in 0..4000 {
        let half_a: Vec3;
        let half_b: Vec3;
        let reach: f32;

        if i & 1 != 0 {
            // Thin crossed beams driven into deep overlap.
            half_a = Vec3::new(
                random.float(1.0, 1.6),
                random.float(0.08, 0.15),
                random.float(0.08, 0.15),
            );
            half_b = Vec3::new(
                random.float(1.0, 1.6),
                random.float(0.08, 0.15),
                random.float(0.08, 0.15),
            );
            reach = random.float(0.0, 0.4);
        } else {
            // Chunky boxes across the whole separated to overlapping range.
            half_a = random.vector(0.3, 0.8);
            half_b = random.vector(0.3, 0.8);
            reach = random.float(0.0, 1.7);
        }

        // Rotate A in place so it stays centered on the origin, which keeps the support bias
        // valid.
        let dir_a = random.direction();
        let angle_a = random.float(0.0, b3::PI);
        let hull_a = make_transformed_box_hull(
            half_a.x,
            half_a.y,
            half_a.z,
            Transform::new(Vec3::ZERO, exact_quat(dir_a, angle_a)),
        );
        let hull_b = make_box_hull(half_b.x, half_b.y, half_b.z);

        // Argument order is part of the PRNG stream: the position draws before the axis
        // direction, which draws before the angle.
        let position_dir = random.direction();
        let axis_dir = random.direction();
        let angle_b = random.float(0.0, b3::PI);
        let xf_b = Transform::new(reach * position_dir, exact_quat(axis_dir, angle_b));

        let query = compute_separating_axis_temp(&hull_a, &hull_b, xf_b, true);
        let q = get_best_axis(&query);

        let oracle = oracle_separation(&hull_a, &hull_b, xf_b);

        assert!(
            abs_float((length(q.normal) - 1.0) as f64) < 1e-3,
            "sample {} normal not unit",
            i
        );
        assert!(
            abs_float(
                (separation_along(&hull_a, &hull_b, xf_b, q.normal) - q.separation) as f64
            ) < 2e-3,
            "sample {} separation inconsistent",
            i
        );

        // Never overstate the true maximum: the reported axis is real.
        assert!(
            q.separation as f64 <= oracle.0 as f64 + 2e-3,
            "sample {} overstated",
            i
        );

        // Inside the speculative band the scan runs exhaustively, so the result is the maximum.
        if q.separation <= b3::speculative_distance() {
            assert!(
                q.separation as f64 >= oracle.0 as f64 - 4e-3,
                "sample {} shortfall {}",
                i,
                oracle.0 - q.separation
            );
        }

        if oracle.0 > 0.0 {
            separated += 1;
        } else {
            penetrating += 1;
        }
        if q.kind == SeparatingFeature::EdgePairAxis {
            edge_wins += 1;
        }
    }

    // The sweep must cover both regimes and actually drive the edge path.
    assert!(separated > 100);
    assert!(penetrating > 100);
    assert!(edge_wins > 20);
}

// MARK: - Edge pair sweep

/// Crossed beam edges: the axis of minimum penetration is the edge pair across the whole
/// separated-to-overlap range, and it must match the brute-force oracle.
#[test]
fn edge_pair_sweep() {
    // "crossed beams keep the edge pair axis across the whole sweep"
    let hull_a = make_transformed_box_hull(
        0.5,
        0.5,
        0.5,
        Transform::new(
            Vec3::ZERO,
            exact_quat(Vec3::AXIS_Y, 0.25 * b3::PI),
        ),
    );
    let hull_b = make_transformed_box_hull(
        0.5,
        0.5,
        0.5,
        Transform::new(
            Vec3::ZERO,
            exact_quat(Vec3::AXIS_Z, 0.25 * b3::PI),
        ),
    );

    let root2 = (2.0f32).sqrt();

    for d in [1.43f32, 1.42, root2, 1.40, 1.38, 1.35, 1.30] {
        let expected = d - root2;
        let xf_b = Transform::new(Vec3::new(d, 0.0, 0.0), Quat::IDENTITY);

        let query = compute_separating_axis_temp(&hull_a, &hull_b, xf_b, true);
        let q = get_best_axis(&query);

        assert!(
            q.kind == SeparatingFeature::EdgePairAxis,
            "d {} type {:?}",
            d,
            q.kind
        );
        assert!(
            abs_float((q.separation - expected) as f64) < 1e-4,
            "d {} separation {}",
            d,
            q.separation
        );
        assert!(
            abs_float((q.normal.x - 1.0) as f64) < 1e-4,
            "d {} normal {:?}",
            d,
            q.normal
        );
        assert!(abs_float(q.normal.y as f64) < 1e-4);
        assert!(abs_float(q.normal.z as f64) < 1e-4);

        // Half edge indices are the even twin of each pair.
        assert_eq!(q.index_a & 1, 0);
        assert_eq!(q.index_b & 1, 0);

        let oracle = oracle_separation(&hull_a, &hull_b, xf_b);
        assert!(
            abs_float((q.separation - oracle.0) as f64) < 1e-4,
            "d {} vs oracle",
            d
        );
        assert!(
            abs_float(
                (separation_along(&hull_a, &hull_b, xf_b, q.normal) - q.separation) as f64
            ) < 1e-4,
            "d {} projection mismatch",
            d
        );
    }
}

// MARK: - Face axis selection

#[test]
fn face_axis_a() {
    // "a separated pair picks body A's face axis"
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);
    let xf_b = Transform::new(Vec3::new(1.2, 0.0, 0.0), Quat::IDENTITY);

    let query = compute_separating_axis_temp(&hull_a, &hull_b, xf_b, true);
    let q = get_best_axis(&query);

    assert_eq!(q.kind, SeparatingFeature::FaceAxisA);
    assert!(abs_float((q.separation - 0.2) as f64) < 1e-5);
    assert!(abs_float((q.normal.x - 1.0) as f64) < 1e-6);
    assert!(abs_float(q.normal.y as f64) < 1e-6);
    assert!(abs_float(q.normal.z as f64) < 1e-6);

    let planes_a = hull_a.planes();
    assert!(abs_float((planes_a[q.index_a as usize].normal.x - 1.0) as f64) < 1e-6);
    let points_b = hull_b.points();
    assert!(abs_float((points_b[q.index_b as usize].x + 0.5) as f64) < 1e-6);
    assert!(
        abs_float((separation_along(&hull_a, &hull_b, xf_b, q.normal) - q.separation) as f64)
            < 1e-5
    );
}

#[test]
fn face_axis_b() {
    // "a rotated body A hands the axis to body B"
    let hull_a = make_transformed_box_hull(
        0.5,
        0.5,
        0.5,
        Transform::new(
            Vec3::ZERO,
            exact_quat(Vec3::AXIS_Z, 0.25 * b3::PI),
        ),
    );
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let root2 = (2.0f32).sqrt();
    let a_extent = 0.5 * root2;
    let gap: f32 = 0.2;
    let d = a_extent + 0.5 + gap;
    let xf_b = Transform::new(Vec3::new(d, 0.0, 0.0), Quat::IDENTITY);

    let query = compute_separating_axis_temp(&hull_a, &hull_b, xf_b, true);
    let q = get_best_axis(&query);

    assert_eq!(q.kind, SeparatingFeature::FaceAxisB);
    assert!(abs_float((q.separation - gap) as f64) < 1e-5);
    assert!(abs_float((q.normal.x - 1.0) as f64) < 1e-5);
    assert!(abs_float(q.normal.y as f64) < 1e-5);
    assert!(abs_float(q.normal.z as f64) < 1e-5);

    let planes_b = hull_b.planes();
    assert!(abs_float((planes_b[q.index_b as usize].normal.x + 1.0) as f64) < 1e-5);
    let points_a = hull_a.points();
    assert!(abs_float((points_a[q.index_a as usize].x - a_extent) as f64) < 1e-4);
    assert!(
        abs_float((separation_along(&hull_a, &hull_b, xf_b, q.normal) - q.separation) as f64)
            < 1e-4
    );
}

#[test]
fn face_far_separated() {
    // "a far separated pair still reports the exact gap"
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);
    let xf_b = Transform::new(Vec3::new(3.0, 0.0, 0.0), Quat::IDENTITY);

    let query = compute_separating_axis_temp(&hull_a, &hull_b, xf_b, true);
    let q = get_best_axis(&query);

    assert_eq!(q.kind, SeparatingFeature::FaceAxisA);
    assert!(abs_float((q.separation - 2.0) as f64) < 1e-5);
    assert!(abs_float((q.normal.x - 1.0) as f64) < 1e-6);

    let oracle = oracle_separation(&hull_a, &hull_b, xf_b);
    assert!(abs_float((q.separation - oracle.0) as f64) < 1e-5);
}

/// Body A carries its origin far from the world center, so the face axis lookup has to survive
/// the offset rather than reporting the world origin.
#[test]
fn offset_face_axis_b() {
    // "an offset body A keeps the face axis lookup exact"
    let cx: f32 = 5.0;
    let place_a = Transform::new(
        Vec3::new(cx, 0.0, 0.0),
        exact_quat(Vec3::AXIS_Z, 0.25 * b3::PI),
    );
    let hull_a = make_transformed_box_hull(0.5, 0.5, 0.5, place_a);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let root2 = (2.0f32).sqrt();
    let a_extent = cx + 0.5 * root2;
    let gap: f32 = 0.2;
    let d = a_extent + 0.5 + gap;
    let xf_b = Transform::new(Vec3::new(d, 0.0, 0.0), Quat::IDENTITY);

    let query = compute_separating_axis_temp(&hull_a, &hull_b, xf_b, true);
    let q = get_best_axis(&query);

    assert_eq!(q.kind, SeparatingFeature::FaceAxisB);
    assert!(abs_float((q.separation - gap) as f64) < 1e-4);
    assert!(abs_float((q.normal.x - 1.0) as f64) < 1e-5);
    assert!(abs_float(q.normal.y as f64) < 1e-5);
    assert!(abs_float(q.normal.z as f64) < 1e-5);

    let planes_b = hull_b.planes();
    assert!(abs_float((planes_b[q.index_b as usize].normal.x + 1.0) as f64) < 1e-5);
    let points_a = hull_a.points();
    assert!(abs_float((points_a[q.index_a as usize].x - a_extent) as f64) < 1e-4);
}

/// A hull whose origin sits far from its own geometry: the support pick must not be biased by
/// the offset, which shows up as the returned normal disagreeing with its own separation.
#[test]
fn offset_hull_oracle() {
    // "offset hulls keep the reported axis consistent with its separation"
    let mut random = Random::new(24_681_012);

    let mut separated = 0;
    let mut penetrating = 0;
    let mut edge_wins = 0;

    for i in 0..3000 {
        let half_a = random.vector(0.3, 0.8);
        let half_b = random.vector(0.3, 0.8);

        let offset = random.float(2.0, 6.0) * random.direction();
        let reach = random.float(0.0, 1.7);
        let dir = random.direction();

        let hull_a: Hull;
        let hull_b: Hull;
        let xf_b: Transform;

        if i & 1 != 0 {
            // A carries the offset in its own frame.
            let dir_place = random.direction();
            let angle_place = random.float(0.0, b3::PI);
            let place_a = Transform::new(offset, exact_quat(dir_place, angle_place));
            hull_a = make_transformed_box_hull(half_a.x, half_a.y, half_a.z, place_a);
            hull_b = make_box_hull(half_b.x, half_b.y, half_b.z);
            let dir_q = random.direction();
            let angle_q = random.float(0.0, b3::PI);
            xf_b = Transform::new(mul_add(offset, reach, dir), exact_quat(dir_q, angle_q));
        } else {
            // B carries the offset in its own frame, placed so its world center lands near the
            // origin.
            let dir_q_b = random.direction();
            let angle_q_b = random.float(0.0, b3::PI);
            let q_b = exact_quat(dir_q_b, angle_q_b);
            hull_a = make_box_hull(half_a.x, half_a.y, half_a.z);
            hull_b = make_offset_box_hull(half_b.x, half_b.y, half_b.z, offset);
            xf_b = Transform::new(reach * dir - rotate_vector(q_b, offset), q_b);
        }

        let query = compute_separating_axis_temp(&hull_a, &hull_b, xf_b, true);
        let q = get_best_axis(&query);
        let oracle = oracle_separation(&hull_a, &hull_b, xf_b);

        // Consistency is the sharp bias check: a wrong support pick on an offset hull throws the
        // normal off its own separation by a vertex spacing, not a noise floor.
        let consistency =
            abs_float((separation_along(&hull_a, &hull_b, xf_b, q.normal) - q.separation) as f64);

        assert!(
            abs_float((length(q.normal) - 1.0) as f64) < 1e-3,
            "sample {} normal not unit",
            i
        );
        assert!(consistency < 1e-3, "sample {} consistency {}", i, consistency);
        assert!(
            q.separation as f64 <= oracle.0 as f64 + 3e-3,
            "sample {} overstated",
            i
        );

        // Only the in-band result feeds a manifold, and only there does the scan run to
        // completion. Offset coordinates cost a few more digits.
        if q.separation <= b3::speculative_distance() {
            assert!(
                q.separation as f64 >= oracle.0 as f64 - 8e-3,
                "sample {} shortfall {}",
                i,
                oracle.0 - q.separation
            );
        }

        if oracle.0 > 0.0 {
            separated += 1;
        } else {
            penetrating += 1;
        }
        if q.kind == SeparatingFeature::EdgePairAxis {
            edge_wins += 1;
        }
    }

    assert!(separated > 100);
    assert!(penetrating > 100);
    assert!(edge_wins > 20);
}
