//! Ported from Tests/NatselTests/Collision/ManifoldRobustnessTests.swift.
//!
//! The numerically most delicate file in the suite: overlapping hulls must always report a
//! penetrating point, edge-pair axes must match a cross-product oracle, ridge crossings must keep
//! face contact shallow and switch to edges when deep, caches must reuse and then report
//! separation, and every routine must report pairs inside the speculative distance.

use crate::base::constants as b3;
use crate::collision::convex_manifold::collide_hulls;
use crate::collision::convex_manifold_shapes::{
    collide_capsule_and_sphere, collide_capsules, collide_hull_and_capsule, collide_hull_and_sphere,
    collide_spheres,
};
use crate::collision::hull::{make_box_hull, make_transformed_box_hull, Hull};
use crate::collision::triangle_manifold::{
    collide_triangle_and_capsule, collide_triangle_and_hull, collide_triangle_and_sphere,
};
use crate::math::functions::{
    cross, dot, length, line_distance, mul_add, mul_sub, normalize, transform_point,
};
use crate::math::types::{Quat, Transform, Vec3};
use crate::tests::world_position_assertions::abs_float;
use crate::types::collision_types::{LocalManifold, LocalManifoldPoint, SATCache, SeparatingFeature};
use crate::types::geometry::{Capsule, Sphere};
use crate::types::queries::SimplexCache;

/// Swift's `Random`: a 32-bit LCG so the sample sequence is identical everywhere.
struct Random {
    seed: u32,
}

impl Random {
    fn new() -> Self {
        Self { seed: 12345 }
    }

    fn next_float(&mut self, lower: f32, upper: f32) -> f32 {
        self.seed = 1664525u32
            .wrapping_mul(self.seed)
            .wrapping_add(1013904223);
        let t = (self.seed >> 8) as f32 / (1u32 << 24) as f32;
        lower + t * (upper - lower)
    }

    fn next_direction(&mut self) -> Vec3 {
        let v = Vec3::new(
            self.next_float(-1.0, 1.0),
            self.next_float(-1.0, 1.0),
            self.next_float(-1.0, 1.0),
        );
        normalize(v)
    }
}

/// Swift's `exactQuat`: a half-angle quaternion built from `sinf`/`cosf` directly so it does not go
/// through the engine's approximation helpers.
fn exact_quat(axis: Vec3, radians: f32) -> Quat {
    let half = 0.5 * radians;
    let s = half.sin();
    Quat::new(s * axis, half.cos())
}

/// Scratch buffer plus manifold for one collision call, mirroring Swift's
/// `withUnsafeMutableBufferPointer` setup.
struct Scratch {
    /// Heap buffer that `manifold.points` addresses. It must not move after `manifold.points` is
    /// set, so it is boxed rather than stored inline: a `Vec`/array field would be relocated when
    /// `Scratch` is returned from `new`, leaving `manifold.points` dangling and letting the collide
    /// call write through a stale pointer.
    points: Vec<LocalManifoldPoint>,
    manifold: LocalManifold,
}

impl Scratch {
    fn new(capacity: i32) -> Self {
        let mut points = vec![LocalManifoldPoint::default(); 8];
        let mut manifold = LocalManifold::default();
        manifold.points = points.as_mut_ptr();
        let _ = capacity;
        Self { points, manifold }
    }

    /// The clipped points the call produced, in manifold order.
    fn clipped(&self) -> Vec<LocalManifoldPoint> {
        let count = self.manifold.point_count as usize;
        self.points[..count].to_vec()
    }
}

fn collide(
    hull_a: &Hull,
    hull_b: &Hull,
    transform: Transform,
    capacity: i32,
) -> (Vec<LocalManifoldPoint>, Vec3) {
    let mut scratch = Scratch::new(capacity);
    let mut cache = SATCache::default();
    collide_hulls(
        &mut scratch.manifold,
        capacity,
        hull_a,
        hull_b,
        transform,
        &mut cache,
    );
    (scratch.clipped(), scratch.manifold.normal)
}

#[test]
fn overlap_never_empty() {
    // "overlapping hulls always produce at least one penetrating point"
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.4, 0.6, 0.5);

    let mut random = Random::new();

    for i in 0..2000 {
        let axis = random.next_direction();

        // Half the samples are nearly aligned, where the edge cross products are smallest.
        let angle = if (i & 1) != 0 {
            random.next_float(-0.01, 0.01)
        } else {
            random.next_float(-b3::PI, b3::PI)
        };

        // Shorter than the smallest half width, so the center of B stays inside A.
        let offset = 0.4 * random.next_direction();

        let transform = Transform::new(offset, exact_quat(axis, angle));
        let (points, normal) = collide(&hull_a, &hull_b, transform, 8);

        assert!(!points.is_empty(), "sample {i} produced no points");
        assert!(abs_float((length(normal) - 1.0) as f64) < 1e-5, "sample {i} normal not unit");

        // Clipping keeps points that are separated, but the deepest one must penetrate.
        let min_separation = points
            .iter()
            .map(|p| p.separation)
            .fold(f32::INFINITY, f32::min);
        assert!(min_separation < 0.0, "sample {i} min separation {min_separation}");
    }
}

#[test]
fn max_point_count() {
    // "manifold point storage matches the published limit"
    // The portable build fixes four points per manifold; the wide build can hold more.
    assert!(b3::MAX_MANIFOLD_POINTS >= 4);
    assert!(core::mem::size_of::<LocalManifoldPoint>() > 0);

    // Collision never writes past the capacity it was given.
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);
    let transform = Transform::new(Vec3::new(0.0, 0.99, 0.0), Quat::IDENTITY);

    for capacity in [1i32, 2, 3, 4] {
        let (points, _) = collide(&hull_a, &hull_b, transform, capacity);
        assert!(points.len() <= capacity as usize);
    }
}

#[test]
fn deep_overlap_samples() {
    // "deep overlap keeps points penetrating across many orientations"
    let hull_a = make_box_hull(1.0, 1.0, 1.0);
    let hull_b = make_box_hull(0.3, 0.3, 0.3);

    let mut random = Random::new();

    for _ in 0..500 {
        let axis = random.next_direction();
        let angle = random.next_float(-b3::PI, b3::PI);
        // Well inside A on every axis.
        let offset = 0.2 * random.next_direction();

        let transform = Transform::new(offset, exact_quat(axis, angle));
        let (points, _) = collide(&hull_a, &hull_b, transform, 8);

        assert!(!points.is_empty());
        let min_separation = points
            .iter()
            .map(|p| p.separation)
            .fold(f32::INFINITY, f32::min);
        assert!(min_separation < 0.0);
    }
}

// MARK: - Crossed edge geometry

/// Cube A yawed 45 degrees presents an edge along y at x = +h*root2. Cube B rolled 45 degrees
/// presents an edge along z at x = -h*root2. Sliding B along x makes those two edges the closest
/// features, so the axis of minimum penetration is x.
fn crossed_edge_hulls(half_width: f32) -> (Hull, Hull) {
    let a = make_transformed_box_hull(
        half_width,
        half_width,
        half_width,
        Transform::new(Vec3::ZERO, exact_quat(Vec3::AXIS_Y, 0.25 * b3::PI)),
    );
    let b = make_transformed_box_hull(
        half_width,
        half_width,
        half_width,
        Transform::new(Vec3::ZERO, exact_quat(Vec3::AXIS_Z, 0.25 * b3::PI)),
    );
    (a, b)
}

#[test]
fn crossed_edge() {
    // "crossed edges pick the edge pair axis and the analytic point"
    let (hull_a, hull_b) = crossed_edge_hulls(0.5);
    let root2 = 2.0f32.sqrt();

    for d in [1.42f32, root2, 1.41, 1.3] {
        let expected = d - root2;
        let transform = Transform::new(Vec3::new(d, 0.0, 0.0), Quat::IDENTITY);

        let mut scratch = Scratch::new(8);
        let mut cache = SATCache::default();
        collide_hulls(&mut scratch.manifold, 8, &hull_a, &hull_b, transform, &mut cache);

        assert_eq!(scratch.manifold.point_count, 1);
        assert_eq!(cache.type_, SeparatingFeature::EdgePairAxis);
        assert!(abs_float((scratch.manifold.normal.x - 1.0) as f64) < 1e-6);
        assert!(abs_float(scratch.manifold.normal.y as f64) < 1e-6);
        assert!(abs_float(scratch.manifold.normal.z as f64) < 1e-6);

        let point = scratch.points[0];
        assert!(abs_float((point.separation - expected) as f64) < 1e-5);
        assert!(abs_float((point.point.x - 0.5 * d) as f64) < 1e-5);
        assert!(abs_float(point.point.y as f64) < 1e-5);
        assert!(abs_float(point.point.z as f64) < 1e-5);

        // The forced edge query must agree with what the full solver chose.
        let mut manual = Scratch::new(8);
        let mut manual_cache = SATCache::default();
        manual_cache.type_ = SeparatingFeature::ManualEdgePairAxis;
        collide_hulls(
            &mut manual.manifold,
            8,
            &hull_a,
            &hull_b,
            transform,
            &mut manual_cache,
        );
        assert_eq!(manual.manifold.point_count, 1);
        assert!(abs_float((manual.points[0].separation - expected) as f64) < 1e-5);
    }
}

// MARK: - Ridge crossing policy

/// A roof face of one hull, so the normal is 45 degrees off the vertical.
fn roof_contact(
    manifold: &LocalManifold,
    points: &[LocalManifoldPoint],
    cache: &SATCache,
    overlap: f32,
    half_root2: f32,
    root2: f32,
) -> Option<String> {
    if manifold.point_count < 4 || manifold.point_count > b3::MAX_MANIFOLD_POINTS {
        return Some(format!("point count {}", manifold.point_count));
    }
    if cache.type_ != SeparatingFeature::FaceAxisA && cache.type_ != SeparatingFeature::FaceAxisB {
        return Some(format!("cache type {:?}", cache.type_));
    }
    if abs_float((manifold.normal.y - half_root2) as f64) > 1e-4 {
        return Some(format!("normal.y {}", manifold.normal.y));
    }

    let min_separation = points
        .iter()
        .map(|p| p.separation)
        .fold(0.0f32, f32::min);
    if min_separation >= -half_root2 * overlap + 1e-4 {
        return Some(format!("separation too shallow {min_separation}"));
    }
    if min_separation <= -root2 * overlap - 1e-4 {
        return Some(format!("separation too deep {min_separation}"));
    }
    None
}

#[test]
fn ridge_crossing() {
    // "ridge crossing keeps face contact shallow and switches to edges when deep"
    let hull_a = make_transformed_box_hull(
        1.5,
        0.1,
        0.1,
        Transform::new(Vec3::ZERO, exact_quat(Vec3::AXIS_X, 0.25 * b3::PI)),
    );
    let hull_b = make_transformed_box_hull(
        1.5,
        0.1,
        0.1,
        Transform::new(Vec3::ZERO, exact_quat(Vec3::AXIS_X, 0.25 * b3::PI)),
    );

    let root2 = 2.0f32.sqrt();
    let half_root2: f32 = 0.70710678;
    let ridge_y: f32 = 0.1 * root2;

    let run = |overlap: f32, angle: f32| -> (Scratch, SATCache) {
        let lift = 2.0 * ridge_y - overlap;
        let transform = Transform::new(Vec3::new(0.0, lift, 0.0), exact_quat(Vec3::AXIS_Y, angle));

        let mut scratch = Scratch::new(8);
        let mut cache = SATCache::default();
        collide_hulls(&mut scratch.manifold, 8, &hull_a, &hull_b, transform, &mut cache);
        (scratch, cache)
    };

    // Shallow overlap: the edge axis is better by only (root2 - 1) * overlap, inside the slop, so
    // the four point face contact carries the crossing at every angle.
    let overlap: f32 = 0.01;
    for angle in [0.0f32, 1e-3, 0.02, 0.1, 0.5] {
        let (scratch, cache) = run(overlap, angle);
        let points = scratch.clipped();
        if let Some(error) = roof_contact(
            &scratch.manifold,
            &points,
            &cache,
            overlap,
            half_root2,
            root2,
        ) {
            println!("shallow angle {angle}: {error}");
        }
        // The recorded issue is followed by the same evaluation the Swift case kept.
        assert!(
            roof_contact(&scratch.manifold, &points, &cache, overlap, half_root2, root2).is_none(),
            "shallow angle {angle}"
        );
    }

    // Deep overlap at a clear crossing. The edge axis now beats the clipped face separation by more
    // than the slop, so the edge contact replaces the face contact.
    let deep: f32 = 0.05;
    for angle in [0.05f32, 0.1, 0.2, 0.5] {
        let (scratch, cache) = run(deep, angle);
        let points = scratch.clipped();
        assert_eq!(scratch.manifold.point_count, 1, "deep angle {angle}");
        assert_eq!(cache.type_, SeparatingFeature::EdgePairAxis, "deep angle {angle}");
        assert!(abs_float(scratch.manifold.normal.x as f64) < 1e-4);
        assert!(abs_float((scratch.manifold.normal.y - 1.0) as f64) < 1e-4);
        assert!(abs_float(scratch.manifold.normal.z as f64) < 1e-4);
        assert!(abs_float((points[0].separation + deep) as f64) < 1e-4);
        assert!(abs_float((points[0].point.y - (ridge_y - 0.5 * deep)) as f64) < 1e-4);
        // Only has to land near the crossing, not at the end of a three meter beam.
        assert!(abs_float(points[0].point.x as f64) < 0.01);
        assert!(abs_float(points[0].point.z as f64) < 0.01);
    }

    // Deep overlap near parallel: a one point edge contact off a parallel pair would have a normal
    // built from noise, so the roof faces keep the contact.
    for angle in [0.0f32, 1e-4, 1e-3, 0.003] {
        let (scratch, cache) = run(deep, angle);
        let points = scratch.clipped();
        if let Some(error) = roof_contact(&scratch.manifold, &points, &cache, deep, half_root2, root2)
        {
            println!("parallel angle {angle}: {error}");
        }
        assert!(
            roof_contact(&scratch.manifold, &points, &cache, deep, half_root2, root2).is_none(),
            "parallel angle {angle}"
        );
    }
}

// MARK: - Edge axis oracle

/// Rebuild the half-edge segment a cached edge index refers to.
fn hull_edge_segment(hull: &Hull, edge_index: u8, transform: Transform) -> (Vec3, Vec3) {
    let edges = hull.edges();
    let points = hull.points();
    let e = edges[edge_index as usize];
    let twin = edges[e.twin as usize];
    let tail = transform_point(transform, points[e.origin as usize]);
    let head = transform_point(transform, points[twin.origin as usize]);
    (tail, head - tail)
}

/// Verify the recovered axis, separation and point against a cross-product oracle built from the
/// edges the solver actually latched onto.
#[allow(clippy::too_many_arguments)]
fn check_edge_contact(
    manifold: &LocalManifold,
    points: &[LocalManifoldPoint],
    p1: Vec3,
    e1: Vec3,
    p2: Vec3,
    e2: Vec3,
    orient_ref: Vec3,
    radius: f32,
    normal_tol: f32,
    sep_tol: f32,
    point_tol: f32,
) -> Option<String> {
    let mut axis = normalize(cross(e1, e2));
    if dot(axis, orient_ref) < 0.0 {
        axis = -axis;
    }

    let n = manifold.normal;
    if abs_float((n.x - axis.x) as f64) > normal_tol as f64
        || abs_float((n.y - axis.y) as f64) > normal_tol as f64
        || abs_float((n.z - axis.z) as f64) > normal_tol as f64
    {
        return Some(format!("normal {n:?} vs axis {axis:?}"));
    }
    if abs_float(dot(n, normalize(e1)) as f64) > normal_tol as f64 {
        return Some("normal not perpendicular to e1".to_owned());
    }
    if abs_float(dot(n, normalize(e2)) as f64) > normal_tol as f64 {
        return Some("normal not perpendicular to e2".to_owned());
    }

    let closest = line_distance(p1, e1, p2, e2);

    // Signed gap between the edge lines along the axis, less the capsule radius.
    let expected_separation = dot(axis, closest.point2 - closest.point1) - radius;
    if abs_float((points[0].separation - expected_separation) as f64) > sep_tol as f64 {
        return Some(format!(
            "separation {} vs {expected_separation}",
            points[0].separation
        ));
    }

    // Midpoint of the closest approach, pulling the first point in by the radius.
    let expected_point = 0.5 * (mul_sub(closest.point1, radius, axis) + closest.point2);
    if abs_float((points[0].point.x - expected_point.x) as f64) > point_tol as f64
        || abs_float((points[0].point.y - expected_point.y) as f64) > point_tol as f64
        || abs_float((points[0].point.z - expected_point.z) as f64) > point_tol as f64
    {
        return Some(format!(
            "point {:?} vs {expected_point:?}",
            points[0].point
        ));
    }
    None
}

#[test]
fn edge_axis_oracle() {
    // "edge pair axis matches the cross product oracle"
    let hull_a = make_transformed_box_hull(
        0.5,
        0.5,
        0.5,
        Transform::new(Vec3::ZERO, exact_quat(Vec3::AXIS_Y, 0.25 * b3::PI)),
    );

    let root2 = 2.0f32.sqrt();
    let mut edge_contacts = 0;
    let mut failures: Vec<String> = Vec::new();

    for roll in [0.18 * b3::PI, 0.25 * b3::PI, 0.32 * b3::PI] {
        let hull_b = make_transformed_box_hull(
            0.5,
            0.5,
            0.5,
            Transform::new(Vec3::ZERO, exact_quat(Vec3::AXIS_Z, roll)),
        );

        for yaw in [-0.35f32, -0.15, 0.0, 0.15, 0.35] {
            for distance in [1.38f32, 1.40, root2, 1.44] {
                let transform = Transform::new(
                    Vec3::new(distance, 0.0, 0.0),
                    exact_quat(Vec3::AXIS_Y, yaw),
                );

                let mut scratch = Scratch::new(8);
                let mut cache = SATCache::default();
                collide_hulls(&mut scratch.manifold, 8, &hull_a, &hull_b, transform, &mut cache);

                if cache.type_ != SeparatingFeature::EdgePairAxis
                    || scratch.manifold.point_count != 1
                {
                    continue;
                }

                let (p1, e1) =
                    hull_edge_segment(&hull_a, cache.index_a, Transform::new(Vec3::ZERO, Quat::IDENTITY));
                let (p2, e2) = hull_edge_segment(&hull_b, cache.index_b, transform);

                let center_a = hull_a.center();
                let center_b = transform_point(transform, hull_b.center());
                let orient_ref = center_b - center_a;

                if let Some(error) = check_edge_contact(
                    &scratch.manifold,
                    &scratch.clipped(),
                    p1,
                    e1,
                    p2,
                    e2,
                    orient_ref,
                    0.0,
                    2e-4,
                    2e-4,
                    2e-3,
                ) {
                    failures.push(format!("roll {roll} yaw {yaw} d {distance}: {error}"));
                }

                edge_contacts += 1;
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{:?}",
        &failures[..failures.len().min(3)]
    );
    // The sweep is only meaningful if it actually drove the edge path.
    assert!(edge_contacts >= 15);
}

// MARK: - Deep edge overlap

#[test]
fn hull_capsule_edge_deep() {
    // "hull and capsule keep a single negative point under deep edge overlap"
    let hull = make_box_hull(0.5, 0.5, 0.5);

    // The +x +y edge runs along z between the +x and +y faces.
    let edge_point = Vec3::new(0.5, 0.5, 0.0);
    let edge_dir = Vec3::new(0.0, 0.0, 1.0);
    let outward = normalize(Vec3::new(1.0, 1.0, 0.0));

    let mut count = 0;
    let mut failures: Vec<String> = Vec::new();

    for depth in [0.12f32, 0.18, 0.25] {
        for radius in [0.05f32, 0.1, 0.2] {
            for tilt in [0.0f32, 0.25, -0.25] {
                let capsule_dir = normalize(Vec3::new(1.0, -1.0, tilt));
                let mid = mul_add(edge_point, -depth, outward);
                let c1 = mul_add(mid, -0.5, capsule_dir);
                let c2 = mul_add(mid, 0.5, capsule_dir);
                let capsule = Capsule::new(c1, c2, radius);

                let mut scratch = Scratch::new(8);
                let mut cache = SimplexCache::default();
                collide_hull_and_capsule(
                    &mut scratch.manifold,
                    8,
                    &hull,
                    capsule,
                    Transform::new(Vec3::ZERO, Quat::IDENTITY),
                    &mut cache,
                );

                let label = format!("d {depth} r {radius} tilt {tilt}");
                if scratch.manifold.point_count != 1 {
                    failures.push(format!("{label}: pointCount {}", scratch.manifold.point_count));
                    continue;
                }
                let sep = scratch.points[0].separation;
                if sep >= 0.0 {
                    failures.push(format!("{label}: separation {sep}"));
                    continue;
                }

                // Hull edge is e1, capsule axis is e2, and the normal points out of the hull.
                if let Some(error) = check_edge_contact(
                    &scratch.manifold,
                    &[scratch.points[0]],
                    edge_point,
                    edge_dir,
                    c1,
                    c2 - c1,
                    outward,
                    radius,
                    1e-4,
                    1e-4,
                    1e-3,
                ) {
                    failures.push(format!("{label}: {error}"));
                    continue;
                }

                count += 1;
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{:?}",
        &failures[..failures.len().min(3)]
    );
    assert_eq!(count, 27);
}

// MARK: - Edge cache reuse

#[test]
fn edge_cache() {
    // "edge cache reuses features across small motion then reports separation"
    let (hull_a, hull_b) = crossed_edge_hulls(0.5);
    let root2 = 2.0f32.sqrt();

    let mut scratch = Scratch::new(8);
    let mut cache = SATCache::default();

    let run = |scratch: &mut Scratch, cache: &mut SATCache, distance: f32| -> i32 {
        let transform = Transform::new(Vec3::new(distance, 0.0, 0.0), Quat::IDENTITY);
        collide_hulls(&mut scratch.manifold, 8, &hull_a, &hull_b, transform, cache);
        scratch.manifold.point_count
    };

    assert_eq!(run(&mut scratch, &mut cache, 1.41), 1);
    assert_eq!(cache.type_, SeparatingFeature::EdgePairAxis);
    // Cached edges are the even half of each twin pair.
    assert!(
        (cache.index_a & 1) == 0 && (cache.index_a as i32) < hull_a.half_edge_count(),
        "cache.indexA {}",
        cache.index_a
    );
    assert!(
        (cache.index_b & 1) == 0 && (cache.index_b as i32) < hull_b.half_edge_count(),
        "cache.indexB {}",
        cache.index_b
    );

    let seeded_separation = cache.separation;
    assert!(abs_float((seeded_separation - (1.41 - root2)) as f64) < 1e-5);

    // Small motion: the cached features still describe the contact.
    assert_eq!(run(&mut scratch, &mut cache, 1.4105), 1);
    assert_eq!(cache.separation, seeded_separation);
    assert!(abs_float((scratch.points[0].separation - (1.4105 - root2)) as f64) < 1e-5);
    assert!(abs_float((scratch.manifold.normal.x - 1.0) as f64) < 1e-6);

    // Jump past the speculative distance: the cached axis alone reports the separation.
    assert_eq!(run(&mut scratch, &mut cache, 1.5), 0);
    assert_eq!(cache.separation, seeded_separation);
}

// MARK: - Edge endpoints

fn collide_manual_edge(hull_a: &Hull, hull_b: &Hull, transform: Transform) -> (i32, SATCache, Vec<LocalManifoldPoint>) {
    let mut scratch = Scratch::new(8);
    let mut cache = SATCache::default();
    cache.type_ = SeparatingFeature::ManualEdgePairAxis;
    collide_hulls(&mut scratch.manifold, 8, hull_a, hull_b, transform, &mut cache);
    let count = scratch.manifold.point_count;
    (count, cache, scratch.clipped())
}

#[test]
fn edge_endpoint() {
    // "an edge pair beyond its endpoint is rejected and clears the cache"
    let (hull_a, hull_b) = crossed_edge_hulls(0.5);
    let root2 = 2.0f32.sqrt();
    let d: f32 = 1.41;
    let expected = d - root2;

    // Just inside the end of edge A.
    let (count, _, points) = collide_manual_edge(
        &hull_a,
        &hull_b,
        Transform::new(Vec3::new(d, 0.49, 0.0), Quat::IDENTITY),
    );
    assert_eq!(count, 1);
    assert!(abs_float((points[0].separation - expected) as f64) < 1e-5);
    assert!(abs_float((points[0].point.y - 0.49) as f64) < 1e-5);

    // Off the end.
    let (count, cache, _) = collide_manual_edge(
        &hull_a,
        &hull_b,
        Transform::new(Vec3::new(d, 0.55, 0.0), Quat::IDENTITY),
    );
    assert_eq!(count, 0);
    assert_eq!(cache.type_, SeparatingFeature::InvalidAxis);

    // The true gap is a vertex to edge distance well past the speculative distance.
    let mut scratch = Scratch::new(8);
    let mut fresh_cache = SATCache::default();
    collide_hulls(
        &mut scratch.manifold,
        8,
        &hull_a,
        &hull_b,
        Transform::new(Vec3::new(d, 0.55, 0.0), Quat::IDENTITY),
        &mut fresh_cache,
    );
    assert_eq!(scratch.manifold.point_count, 0);
    assert!(fresh_cache.separation > 0.0);
}

// MARK: - Parallel edges

#[test]
fn parallel_edge() {
    // "tiny tilts keep the face normal on parallel boxes"
    let hull_a = make_box_hull(0.5, 0.5, 0.5);
    let hull_b = make_box_hull(0.5, 0.5, 0.5);

    let tilt_axes = [
        Vec3::new(0.57735027, 0.57735027, 0.57735027),
        Vec3::new(0.70710678, 0.0, 0.70710678),
        Vec3::new(0.26726124, 0.53452248, 0.80178373),
        Vec3::new(-0.48507125, 0.72760688, -0.48507125),
    ];
    // Angles that straddle the 0.005 rejection threshold.
    let tilt_angles: [f32; 11] = [
        0.0, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3, 0.004, 0.005, 0.006, 0.01, 0.05,
    ];
    let overlap: f32 = 0.01;
    // A cube corner is root3/2 from the center of rotation.
    let half_diagonal: f32 = 0.87;

    let mut failures: Vec<String> = Vec::new();

    for axis in tilt_axes {
        for angle in tilt_angles {
            let transform = Transform::new(
                Vec3::new(0.0, 1.0 - overlap, 0.0),
                exact_quat(axis, angle),
            );

            let mut scratch = Scratch::new(8);
            let mut cache = SATCache::default();
            collide_hulls(&mut scratch.manifold, 8, &hull_a, &hull_b, transform, &mut cache);

            let label = format!("axis {axis:?} angle {angle}");
            if scratch.manifold.point_count < 4
                || scratch.manifold.point_count > b3::MAX_MANIFOLD_POINTS
            {
                failures.push(format!("{label}: count {}", scratch.manifold.point_count));
                continue;
            }
            if cache.type_ != SeparatingFeature::FaceAxisA
                && cache.type_ != SeparatingFeature::FaceAxisB
            {
                failures.push(format!("{label}: cache {:?}", cache.type_));
                continue;
            }
            if dot(scratch.manifold.normal, Vec3::AXIS_Y) <= 0.998 {
                failures.push(format!("{label}: normal {:?}", scratch.manifold.normal));
                continue;
            }

            // The tilt can only lift or sink a face point by the arc it sweeps.
            let bound = half_diagonal * angle + 1e-5;
            for k in 0..scratch.manifold.point_count as usize {
                if abs_float((scratch.points[k].separation + overlap) as f64) > bound as f64 {
                    failures.push(format!(
                        "{label}: point {k} separation {}",
                        scratch.points[k].separation
                    ));
                    break;
                }
            }
        }
    }

    assert!(
        failures.is_empty(),
        "{:?}",
        &failures[..failures.len().min(3)]
    );
}

// MARK: - Sphere hull seam

#[test]
fn sphere_hull_seam() {
    // "sphere crossing a hull face keeps separation and normal continuous"
    let hull = make_box_hull(0.5, 0.5, 0.5);
    let radius: f32 = 0.15;

    let y_start = 0.5 + radius + 0.4 * b3::speculative_distance();
    let y_end: f32 = 0.1;
    let steps = 400;
    let dy = (y_start - y_end) / steps as f32;

    let mut previous: f32 = 0.0;
    let mut shallow_samples = 0;
    let mut deep_samples = 0;
    let mut failures: Vec<String> = Vec::new();

    for i in 0..=steps {
        let y = y_start - i as f32 * dy;
        let sphere = Sphere::new(Vec3::new(0.0, y, 0.0), radius);

        let mut scratch = Scratch::new(8);
        let mut cache = SimplexCache::default();
        collide_hull_and_sphere(
            &mut scratch.manifold,
            8,
            &hull,
            sphere,
            Transform::new(Vec3::ZERO, Quat::IDENTITY),
            &mut cache,
        );

        if scratch.manifold.point_count != 1 {
            failures.push(format!("step {i}: count {}", scratch.manifold.point_count));
            break;
        }

        let separation = scratch.points[0].separation;
        let expected = (y - 0.5) - radius;

        // Separation is the analytic gap on both sides of the seam.
        if abs_float((separation - expected) as f64) > 1e-5 {
            failures.push(format!("step {i}: separation {separation} vs {expected}"));
            break;
        }
        // Normal holds the face direction with no flip.
        let n = scratch.manifold.normal;
        if abs_float(n.x as f64) > 1e-5
            || abs_float((n.y - 1.0) as f64) > 1e-5
            || abs_float(n.z as f64) > 1e-5
        {
            failures.push(format!("step {i}: normal {n:?}"));
            break;
        }
        // No jump across the seam: consecutive separations track the step.
        if i > 0 && abs_float(((previous - separation) - dy) as f64) > 1e-5 {
            failures.push(format!("step {i}: jump {} vs {dy}", previous - separation));
            break;
        }
        previous = separation;

        if y > 0.5 {
            shallow_samples += 1;
        } else {
            deep_samples += 1;
        }
    }

    assert!(
        failures.is_empty(),
        "{:?}",
        &failures[..failures.len().min(2)]
    );
    // The sweep must straddle the surface so both the GJK and the SAT branch run.
    assert!(shallow_samples > 0);
    assert!(deep_samples > 0);
}

// MARK: - Capsule hull seam

#[test]
fn capsule_hull_seam() {
    // "capsule crossing a hull face keeps every point at the analytic gap"
    let hull = make_box_hull(0.5, 0.5, 0.5);
    let radius: f32 = 0.15;
    let half_length: f32 = 0.3;

    let y_start = 0.5 + radius + 0.4 * b3::speculative_distance();
    let y_end: f32 = 0.1;
    let steps = 400;
    let dy = (y_start - y_end) / steps as f32;

    let mut previous: f32 = 0.0;
    let mut shallow_samples = 0;
    let mut deep_samples = 0;
    let mut failures: Vec<String> = Vec::new();

    for i in 0..=steps {
        let y = y_start - i as f32 * dy;
        let capsule = Capsule::new(
            Vec3::new(-half_length, y, 0.0),
            Vec3::new(half_length, y, 0.0),
            radius,
        );

        let mut scratch = Scratch::new(8);
        let mut cache = SimplexCache::default();
        collide_hull_and_capsule(
            &mut scratch.manifold,
            8,
            &hull,
            capsule,
            Transform::new(Vec3::ZERO, Quat::IDENTITY),
            &mut cache,
        );

        let count = scratch.manifold.point_count as usize;
        if count < 1 {
            failures.push(format!("step {i}: no points"));
            break;
        }

        let expected = (y - 0.5) - radius;
        let mut bad_point: Option<f32> = None;
        for k in 0..count {
            if abs_float((scratch.points[k].separation - expected) as f64) > 1e-5 {
                bad_point = Some(scratch.points[k].separation);
                break;
            }
        }
        if let Some(bad_point) = bad_point {
            failures.push(format!("step {i}: separation {bad_point} vs {expected}"));
            break;
        }

        let n = scratch.manifold.normal;
        if abs_float(n.x as f64) > 1e-5
            || abs_float((n.y - 1.0) as f64) > 1e-5
            || abs_float(n.z as f64) > 1e-5
        {
            failures.push(format!("step {i}: normal {n:?}"));
            break;
        }

        let min_separation = scratch.points[..count]
            .iter()
            .map(|p| p.separation)
            .fold(f32::INFINITY, f32::min);
        if i > 0 && abs_float(((previous - min_separation) - dy) as f64) > 1e-5 {
            failures.push(format!(
                "step {i}: jump {} vs {dy}",
                previous - min_separation
            ));
            break;
        }
        previous = min_separation;

        if y > 0.5 {
            shallow_samples += 1;
        } else {
            deep_samples += 1;
        }
    }

    assert!(
        failures.is_empty(),
        "{:?}",
        &failures[..failures.len().min(2)]
    );
    assert!(shallow_samples > 0);
    assert!(deep_samples > 0);
}

// MARK: - Triangle edge pair

#[test]
fn triangle_edge() {
    // "triangle and hull recover the edge pair axis and analytic point"
    let half_root2: f32 = 0.70710678;
    let hull = make_transformed_box_hull(
        0.5,
        0.5,
        0.5,
        Transform::new(Vec3::ZERO, exact_quat(Vec3::AXIS_X, 0.25 * b3::PI)),
    );

    let beta = 20.0 * b3::PI / 180.0;
    let gamma = 30.0 * b3::PI / 180.0;

    let tri_normal = Vec3::new(beta.sin(), beta.cos(), 0.0);
    let tri_edge = Vec3::new(
        gamma.sin() * beta.cos(),
        -gamma.sin() * beta.sin(),
        gamma.cos(),
    );

    // Perpendicular to both edges and pointing out of the hull.
    let axis = normalize(cross(Vec3::AXIS_X, tri_edge));
    let hull_point = Vec3::new(0.0, -half_root2, 0.0);

    let mut failures: Vec<String> = Vec::new();

    for gap in [0.03f32, 0.01, 0.0, -0.01, -0.1] {
        let triangle_point = mul_add(hull_point, gap, axis);

        let v1 = mul_add(triangle_point, -1.0, tri_edge);
        let v2 = mul_add(triangle_point, 1.0, tri_edge);
        let v3 = mul_add(v1, 1.5, cross(tri_normal, tri_edge));

        let mut scratch = Scratch::new(8);
        let mut cache = SATCache::default();
        cache.type_ = SeparatingFeature::ManualEdgePairAxis;
        collide_triangle_and_hull(
            &mut scratch.manifold,
            8,
            v1,
            v2,
            v3,
            0,
            &hull,
            &mut cache,
            true,
        );

        let label = format!("gap {gap}");
        if scratch.manifold.point_count != 1 {
            failures.push(format!("{label}: count {}", scratch.manifold.point_count));
            continue;
        }
        if cache.type_ != SeparatingFeature::EdgePairAxis {
            failures.push(format!("{label}: cache {:?}", cache.type_));
            continue;
        }

        let expected_normal = -axis;
        let expected_point = mul_add(hull_point, 0.5 * gap, axis);
        let n = scratch.manifold.normal;
        let p = scratch.points[0];

        if abs_float((n.x - expected_normal.x) as f64) > 1e-5
            || abs_float((n.y - expected_normal.y) as f64) > 1e-5
            || abs_float((n.z - expected_normal.z) as f64) > 1e-5
        {
            failures.push(format!("{label}: normal {n:?} vs {expected_normal:?}"));
            continue;
        }
        if abs_float((p.separation - gap) as f64) > 1e-5 {
            failures.push(format!("{label}: separation {}", p.separation));
            continue;
        }
        if abs_float((p.point.x - expected_point.x) as f64) > 1e-5
            || abs_float((p.point.y - expected_point.y) as f64) > 1e-5
            || abs_float((p.point.z - expected_point.z) as f64) > 1e-5
        {
            failures.push(format!("{label}: point {:?} vs {expected_point:?}", p.point));
            continue;
        }
    }

    assert!(
        failures.is_empty(),
        "{:?}",
        &failures[..failures.len().min(2)]
    );

    // Far separated: no contact, but the cached edge separation must still report the gap.
    let far_gap: f32 = 0.2;
    let triangle_point = mul_add(hull_point, far_gap, axis);
    let v1 = mul_add(triangle_point, -1.0, tri_edge);
    let v2 = mul_add(triangle_point, 1.0, tri_edge);
    let v3 = mul_add(v1, 1.5, cross(tri_normal, tri_edge));

    let mut scratch = Scratch::new(8);
    let mut cache = SATCache::default();
    collide_triangle_and_hull(
        &mut scratch.manifold,
        8,
        v1,
        v2,
        v3,
        0,
        &hull,
        &mut cache,
        true,
    );
    assert_eq!(scratch.manifold.point_count, 0);
    assert_eq!(cache.type_, SeparatingFeature::EdgePairAxis);
    assert!(abs_float((cache.separation - far_gap) as f64) < 1e-5);
}

// MARK: - Speculative margin

#[test]
fn speculative_margin() {
    // "every manifold routine reports pairs inside the speculative distance"
    let max_points = b3::MAX_MANIFOLD_POINTS;

    let sphere = Sphere::new(Vec3::ZERO, 0.25);
    let big_sphere = Sphere::new(Vec3::ZERO, 0.5);
    let capsule_x = Capsule::new(Vec3::new(-0.5, 0.0, 0.0), Vec3::new(0.5, 0.0, 0.0), 0.25);
    let capsule_z = Capsule::new(Vec3::new(0.0, 0.0, -0.5), Vec3::new(0.0, 0.0, 0.5), 0.25);
    let box_ = make_box_hull(0.5, 0.5, 0.5);

    // Triangle with a +y normal.
    let triangle = [
        Vec3::new(-2.0, 0.0, -2.0),
        Vec3::new(0.0, 0.0, 2.0),
        Vec3::new(2.0, 0.0, -2.0),
    ];

    // Every routine must produce points whose minimum separation is the requested gap.
    let check = |label: &str, gap: f32, points: &[LocalManifoldPoint]| {
        if points.is_empty() {
            println!("{label} gap {gap}: no points");
            assert!(!points.is_empty(), "{label} gap {gap}: no points");
            return;
        }
        let min_separation = points
            .iter()
            .map(|p| p.separation)
            .fold(f32::INFINITY, f32::min);
        if abs_float((min_separation - gap) as f64) > 1e-4 {
            println!("{label} gap {gap}: separation {min_separation}");
        }
        assert!(
            abs_float((min_separation - gap) as f64) <= 1e-4,
            "{label} gap {gap}: separation {min_separation}"
        );
    };

    let collide = |body: &dyn Fn(&mut LocalManifold)| -> Vec<LocalManifoldPoint> {
        let mut scratch = Scratch::new(max_points);
        body(&mut scratch.manifold);
        scratch.clipped()
    };

    for gap in [
        0.0f32,
        0.25 * b3::speculative_distance(),
        0.75 * b3::speculative_distance(),
    ] {
        let slide_y75 = Transform::new(Vec3::new(0.0, 0.75 + gap, 0.0), Quat::IDENTITY);
        let slide_y50 = Transform::new(Vec3::new(0.0, 0.5 + gap, 0.0), Quat::IDENTITY);
        let slide_y100 = Transform::new(Vec3::new(0.0, 1.0 + gap, 0.0), Quat::IDENTITY);

        check(
            "sphere-sphere",
            gap,
            &collide(&|m| collide_spheres(m, max_points, big_sphere, sphere, slide_y75)),
        );

        check(
            "capsule-sphere",
            gap,
            &collide(&|m| collide_capsule_and_sphere(m, max_points, capsule_x, sphere, slide_y50)),
        );

        check(
            "capsule-capsule",
            gap,
            &collide(&|m| collide_capsules(m, max_points, capsule_x, capsule_z, slide_y50)),
        );

        check(
            "hull-sphere",
            gap,
            &collide(&|m| {
                let mut cache = SimplexCache::default();
                collide_hull_and_sphere(m, max_points, &box_, sphere, slide_y75, &mut cache)
            }),
        );

        check(
            "hull-capsule",
            gap,
            &collide(&|m| {
                let mut cache = SimplexCache::default();
                collide_hull_and_capsule(m, max_points, &box_, capsule_x, slide_y75, &mut cache)
            }),
        );

        check(
            "hull-hull",
            gap,
            &collide(&|m| {
                let mut cache = SATCache::default();
                collide_hulls(m, max_points, &box_, &box_, slide_y100, &mut cache)
            }),
        );

        check(
            "triangle-sphere",
            gap,
            &collide(&|m| {
                let raised = Sphere::new(Vec3::new(0.0, 0.25 + gap, 0.0), 0.25);
                collide_triangle_and_sphere(m, max_points, &triangle, raised)
            }),
        );

        check(
            "triangle-capsule",
            gap,
            &collide(&|m| {
                let raised = Capsule::new(
                    Vec3::new(-0.5, 0.25 + gap, 0.0),
                    Vec3::new(0.5, 0.25 + gap, 0.0),
                    0.25,
                );
                let mut cache = SimplexCache::default();
                collide_triangle_and_capsule(m, max_points, &triangle, raised, &mut cache)
            }),
        );
    }

    // Beyond the speculative distance the closed form routines report nothing.
    let far_gap = 1.5 * b3::speculative_distance();

    let far_spheres = collide(&|m| {
        collide_spheres(
            m,
            max_points,
            big_sphere,
            sphere,
            Transform::new(Vec3::new(0.0, 0.75 + far_gap, 0.0), Quat::IDENTITY),
        )
    });
    assert!(far_spheres.is_empty());

    let far_capsule = collide(&|m| {
        collide_capsule_and_sphere(
            m,
            max_points,
            capsule_x,
            sphere,
            Transform::new(Vec3::new(0.0, 0.5 + far_gap, 0.0), Quat::IDENTITY),
        )
    });
    assert!(far_capsule.is_empty());
}
