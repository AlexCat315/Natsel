//! Separating axis test and hull contact generation.
//!
//! The SIMD implementation reduces edge candidates in ascending index order, so the scalar loops
//! below produce the same winner. The wide support query includes the mantissa index embedding so
//! near ties resolve to the same vertex.

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::collision::hull::{find_hull_support_vertex, hull_soa_stride, Hull};
use crate::collision::internal_math::{
    arbitrary_perp, make_plane_from_normal_and_point, plane_separation,
};
use crate::collision::manifold::{
    clip_polygon, find_incident_face, flip_pair, make_feature_pair, validate_polygon, AxisQuery,
    ClipVertex, FeatureOwner, SeparatingAxis,
};
use crate::math::functions::*;
use crate::math::types::{Plane, SegmentDistanceResult, Transform, Vec3};
use crate::types::collision_types::{LocalManifold, LocalManifoldPoint, SATCache, SeparatingFeature};

// MARK: - Constants

/// Edge candidates narrower than this cosine are treated as parallel.
pub const PARALLEL_TOL: f32 = 1e-4;
/// Minimum number of edge probes before the probe culling is worth doing.
pub const EDGE_PROBE_MIN_TESTS: i32 = 10;
/// The number of bits reserved for the vertex index in the wide support query.
pub const HULL_BIT_COUNT: u32 = 7;
/// The slot count of each SAT scratch array.
pub const SAT_ARRAY_LEN: usize = 132;

// MARK: - Scratch

fn sat_floats() -> Vec<f32> {
    vec![0.0; SAT_ARRAY_LEN]
}

fn sat_indices() -> Vec<i32> {
    vec![0; SAT_ARRAY_LEN]
}

/// Reusable separating axis scratch arrays.
#[derive(Clone, Debug)]
pub struct SatScratch {
    pub dot_a: Vec<f32>,
    pub dot_b: Vec<f32>,
    pub plane_dot_a: Vec<f32>,
    pub plane_dot_b: Vec<f32>,
    pub edge_indices_a: Vec<i32>,
    pub edge_indices_b: Vec<i32>,
    pub b_fn_x: Vec<f32>,
    pub b_fn_y: Vec<f32>,
    pub b_fn_z: Vec<f32>,
    pub b_w_x: Vec<f32>,
    pub b_w_y: Vec<f32>,
    pub b_w_z: Vec<f32>,
    pub a_n0_x: Vec<f32>,
    pub a_n0_y: Vec<f32>,
    pub a_n0_z: Vec<f32>,
    pub a_n1_x: Vec<f32>,
    pub a_n1_y: Vec<f32>,
    pub a_n1_z: Vec<f32>,
    pub a_d_x: Vec<f32>,
    pub a_d_y: Vec<f32>,
    pub a_d_z: Vec<f32>,
    pub a_v0_x: Vec<f32>,
    pub a_v0_y: Vec<f32>,
    pub a_v0_z: Vec<f32>,
    pub a_tol: Vec<f32>,
}

impl SatScratch {
    pub fn new() -> Self {
        Self {
            dot_a: sat_floats(),
            dot_b: sat_floats(),
            plane_dot_a: sat_floats(),
            plane_dot_b: sat_floats(),
            edge_indices_a: sat_indices(),
            edge_indices_b: sat_indices(),
            b_fn_x: sat_floats(),
            b_fn_y: sat_floats(),
            b_fn_z: sat_floats(),
            b_w_x: sat_floats(),
            b_w_y: sat_floats(),
            b_w_z: sat_floats(),
            a_n0_x: sat_floats(),
            a_n0_y: sat_floats(),
            a_n0_z: sat_floats(),
            a_n1_x: sat_floats(),
            a_n1_y: sat_floats(),
            a_n1_z: sat_floats(),
            a_d_x: sat_floats(),
            a_d_y: sat_floats(),
            a_d_z: sat_floats(),
            a_v0_x: sat_floats(),
            a_v0_y: sat_floats(),
            a_v0_z: sat_floats(),
            a_tol: sat_floats(),
        }
    }
}

impl Default for SatScratch {
    fn default() -> Self {
        Self::new()
    }
}

/// Reusable face clipping buffers.
#[derive(Clone, Debug)]
pub struct FaceScratch {
    pub polygon1: Vec<ClipVertex>,
    pub polygon2: Vec<ClipVertex>,
    pub points: Vec<LocalManifoldPoint>,
}

impl FaceScratch {
    pub fn new() -> Self {
        Self {
            polygon1: vec![ClipVertex::default(); 64],
            polygon2: vec![ClipVertex::default(); 64],
            points: vec![LocalManifoldPoint::default(); 64],
        }
    }
}

impl Default for FaceScratch {
    fn default() -> Self {
        Self::new()
    }
}

// MARK: - Segment clipping

/// Clip a segment against a plane.
pub fn clip_segment(segment: &mut [ClipVertex], plane: Plane) -> i32 {
    let mut vertex_count: i32 = 0;
    let vertex1 = segment[0];
    let vertex2 = segment[1];

    let distance1 = plane_separation(plane, vertex1.position);
    let distance2 = plane_separation(plane, vertex2.position);

    // If the points are behind the plane
    if distance1 <= 0.0 {
        segment[vertex_count as usize] = vertex1;
        vertex_count += 1;
    }
    if distance2 <= 0.0 {
        segment[vertex_count as usize] = vertex2;
        vertex_count += 1;
    }

    // If the points are on different sides of the plane
    if distance1 * distance2 < 0.0 {
        // Find intersection point of edge and plane
        let t = distance1 / (distance1 - distance2);
        segment[vertex_count as usize].position =
            (1.0 - t) * vertex1.position + t * vertex2.position;
        segment[vertex_count as usize].pair = if distance1 > 0.0 {
            vertex1.pair
        } else {
            vertex2.pair
        };
        vertex_count += 1;
    }

    vertex_count
}

/// Clip a segment against the side planes of a hull face.
pub fn clip_segment_to_hull_face(segment: &mut [ClipVertex], hull: &Hull, ref_face: i32) -> i32 {
    let faces = hull.faces();
    let planes = hull.planes();
    let edges = hull.edges();
    let points = hull.points();

    let ref_plane = planes[ref_face as usize];
    let face = faces[ref_face as usize];

    let start_edge = face.edge as i32;
    let mut edge_index = start_edge;

    loop {
        let edge = edges[edge_index as usize];
        let next_edge_index = edge.next as i32;
        let next = edges[next_edge_index as usize];

        let vertex1 = points[edge.origin as usize];
        let vertex2 = points[next.origin as usize];
        let tangent = normalize(vertex2 - vertex1);
        let binormal = cross(tangent, ref_plane.normal);

        let point_count = clip_segment(segment, make_plane_from_normal_and_point(binormal, vertex1));
        if point_count < 2 {
            return 0;
        }

        edge_index = next_edge_index;
        if edge_index == start_edge {
            break;
        }
    }

    2
}

// MARK: - Edge candidate test

/// Inscribed sphere edge test.
pub fn test_edge_candidate_sorted(a1: f32, a2: f32, c: f32, bound: f32) -> bool {
    let hi = max_float(a1, a2);
    let lo = min_float(a1, a2);
    let exterior = hi >= bound;

    let u = lo - c * hi;
    let max_is_interior = u >= 0.0;

    let s = 1.0 - c * c;
    let bound_term = (bound - hi) * (bound + hi);
    let lhs = u * u;
    let rhs = bound_term * s;
    let max_beats_bound = lhs >= rhs;

    let nearly_parallel = s < PARALLEL_TOL;

    let interior = max_is_interior && (max_beats_bound || nearly_parallel);

    exterior || interior
}

// MARK: - Wide helpers

/// Round a count up to the SIMD width (four lanes).
pub fn round_up_to_simd_width(count: i32) -> i32 {
    (count + 4 - 1) & !(4 - 1)
}

/// Dot all face normals with `d`, into `dots`.
pub fn get_face_dots(hull: &Hull, d: Vec3, dots: &mut [f32]) {
    let (nx, ny, nz) = hull.soa_normals();
    let face_count = hull.face_count() as usize;
    for i in 0..face_count {
        dots[i] = d.x * nx[i] + d.y * ny[i] + d.z * nz[i];
    }
}

/// `dot(n, otherCenter) - offset` for all faces of the hull.
pub fn get_face_plane_separations(hull: &Hull, other_center: Vec3, separations: &mut [f32]) {
    get_face_dots(hull, other_center, separations);

    let planes = hull.planes();
    let face_count = hull.face_count() as usize;
    for i in 0..face_count {
        separations[i] -= planes[i].offset;
    }
}

/// Re-test edge candidates against the plane separations of a probe point on the other hull.
pub fn filter_edge_candidates(
    hull: &Hull,
    plane_separations: &[f32],
    bound: f32,
    edge_indices: &mut [i32],
    count: i32,
) -> i32 {
    let half_edges = hull.edges();
    let cosines = hull.edge_cosines();
    let mut kept_count: i32 = 0;

    for k in 0..count as usize {
        let i = edge_indices[k];
        let i1 = half_edges[i as usize].face as usize;
        let i2 = half_edges[i as usize + 1].face as usize;
        let c = cosines[(i as usize) >> 1];
        edge_indices[kept_count as usize] = i;
        if test_edge_candidate_sorted(plane_separations[i1], plane_separations[i2], c, bound) {
            kept_count += 1;
        }
    }

    kept_count
}

/// Four-wide support query, preserving the embedded vertex index bits.
///
/// The support query minimizes `bias - dot(normal, vertex)` with the vertex index embedded in the
/// low mantissa bits and then performs a horizontal minimum. Because the values are positive, that
/// is a plain numeric minimum over all lanes, including the padded tail (which repeats vertex 0
/// with a higher embedded index, so it never wins a tie). The exact support is recomputed
/// afterwards without the index bits.
pub fn get_support_wide(
    normal: Vec3,
    vx: &[f32],
    vy: &[f32],
    vz: &[f32],
    n: i32,
    bias: f32,
) -> (f32, i32) {
    let mask: u32 = (1u32 << HULL_BIT_COUNT) - 1;
    let mut min_value = f32::INFINITY;

    for i in 0..n as usize {
        let d = normal.x * vx[i] + normal.y * vy[i] + normal.z * vz[i];
        let value = bias - d;
        let bits = (value.to_bits() & !mask) | ((i as u32) & mask);
        let augmented = f32::from_bits(bits);
        if augmented < min_value {
            min_value = augmented;
        }
    }

    // One horizontal min, the winning lane's value and index bits ride through.
    let vertex_index = (min_value.to_bits() & mask) as i32;

    // Exact support for the chosen vertex.
    let vi = vertex_index as usize;
    let support = normal.x * vx[vi] + normal.y * vy[vi] + normal.z * vz[vi];

    (support, vertex_index)
}

/// Separation of a face plane against the support vertex of the other hull.
#[allow(clippy::too_many_arguments)]
pub fn get_face_separation(
    direction: Vec3,
    plane_separation: f32,
    vx: &[f32],
    vy: &[f32],
    vz: &[f32],
    n: i32,
    center: Vec3,
    extents: Vec3,
    vertex_index: &mut i32,
) -> f32 {
    let bias = dot(direction, center) + 1.0625 * dot(abs_vec3(direction), extents);
    let (support, vi) = get_support_wide(direction, vx, vy, vz, n, bias);
    *vertex_index = vi;
    plane_separation - support
}

/// Transform a SoA point/normal stream by `out = -(R*v (+t))`.
#[allow(clippy::too_many_arguments)]
pub fn negative_transform_from_soa(
    r: crate::math::types::Matrix3,
    p: Vec3,
    in_x: &[f32],
    in_y: &[f32],
    in_z: &[f32],
    n: i32,
    out_x: &mut [f32],
    out_y: &mut [f32],
    out_z: &mut [f32],
    is_point: bool,
) {
    // row-column
    let r0 = Vec3::new(r.cx.x, r.cy.x, r.cz.x);
    let r1 = Vec3::new(r.cx.y, r.cy.y, r.cz.y);
    let r2 = Vec3::new(r.cx.z, r.cy.z, r.cz.z);

    for i in 0..n as usize {
        let v = Vec3::new(in_x[i], in_y[i], in_z[i]);

        // Rotate one vector per lane
        let mut out = Vec3::new(dot(r0, v), dot(r1, v), dot(r2, v));

        if is_point {
            out = out + p;
        }

        out_x[i] = -out.x;
        out_y[i] = -out.y;
        out_z[i] = -out.z;
    }
}

// MARK: - Separating axis

/// Test/standalone entry point.
pub fn compute_separating_axis_temp(
    hull_a: &Hull,
    hull_b: &Hull,
    xf_b: Transform,
    early_return: bool,
) -> AxisQuery {
    let mut scratch = SatScratch::new();
    compute_separating_axis(hull_a, hull_b, xf_b, early_return, &mut scratch)
}

/// Compute the best face and edge separating axes between two hulls, with B in A's frame.
pub fn compute_separating_axis(
    hull_a: &Hull,
    hull_b: &Hull,
    xf_b: Transform,
    early_return: bool,
    scratch: &mut SatScratch,
) -> AxisQuery {
    // Every consumed scratch element is written before it is read.
    b3_assert(
        b3::MAX_HULL_VERTICES <= SAT_ARRAY_LEN as i32
            && b3::MAX_HULL_FACES <= SAT_ARRAY_LEN as i32
            && b3::MAX_HULL_EDGES <= SAT_ARRAY_LEN as i32,
    );

    let r = make_matrix_from_quat(xf_b.q);
    let inv_r = transpose(r);

    let speculative_distance = b3::speculative_distance();

    let mut res = AxisQuery::default();
    res.face_a.normal = Vec3::ZERO;
    res.face_a.separation = -f32::INFINITY;
    res.face_a.index_a = b3::NULL_INDEX;
    res.face_a.index_b = b3::NULL_INDEX;
    res.face_a.kind = SeparatingFeature::FaceAxisA;

    res.face_b.normal = Vec3::ZERO;
    res.face_b.separation = -f32::INFINITY;
    res.face_b.index_a = b3::NULL_INDEX;
    res.face_b.index_b = b3::NULL_INDEX;
    res.face_b.kind = SeparatingFeature::FaceAxisB;

    res.edge.normal = Vec3::ZERO;
    res.edge.separation = -f32::INFINITY;
    res.edge.index_a = b3::NULL_INDEX;
    res.edge.index_b = b3::NULL_INDEX;
    res.edge.kind = SeparatingFeature::EdgePairAxis;

    res.separated_feature = SeparatingFeature::InvalidAxis;

    let face_count_a = hull_a.face_count();
    let planes_a = hull_a.planes();

    let (vx_b, vy_b, vz_b) = hull_b.soa_vertices();
    let wide_vertex_count_b = round_up_to_simd_width(hull_b.vertex_count());

    let c_b = aabb_center(hull_b.aabb());
    let h_b = aabb_extents(hull_b.aabb());

    let delta_center = mul_mv(r, hull_b.center()) + xf_b.p - hull_a.center();
    let center_distance = length(delta_center);
    let radius = hull_a.inner_radius() + hull_b.inner_radius();

    // Adjust the radius to ensure the best axis isn't skipped.
    let radius_bound = radius - (b3::linear_slop() + 0.001 * (center_distance + radius));

    get_face_dots(hull_a, delta_center, &mut scratch.dot_a);

    // Find the face of hullA that most aligns with centerDelta.
    let mut seed_index_a: i32 = 0;
    let mut max_dot_a = scratch.dot_a[0];
    for i in 1..face_count_a as usize {
        if scratch.dot_a[i] > max_dot_a {
            max_dot_a = scratch.dot_a[i];
            seed_index_a = i as i32;
        }
    }

    // Use the seed to get a lower bound on the separation for the faces of hullA.
    let mut floor_a = -f32::INFINITY;
    let mut seed_separation_a = -f32::INFINITY;
    let mut seed_vertex_b: i32 = 0;
    if early_return {
        let plane = planes_a[seed_index_a as usize];
        let direction = -mul_mv(inv_r, plane.normal);
        let plane_sep = dot(plane.normal, xf_b.p) - plane.offset;
        seed_separation_a = get_face_separation(
            direction,
            plane_sep,
            vx_b,
            vy_b,
            vz_b,
            wide_vertex_count_b,
            c_b,
            h_b,
            &mut seed_vertex_b,
        );
        floor_a = min_float(seed_separation_a, speculative_distance);
    }

    // Test A's face planes against B's vertices.
    for i in 0..face_count_a as usize {
        // The bound offset ensures the seed will be evaluated.
        if scratch.dot_a[i] - radius_bound < max_float(floor_a, res.face_a.separation) {
            continue;
        }

        let plane = planes_a[i];
        let mut vertex_index = seed_vertex_b;
        let mut separation = seed_separation_a;

        // Avoid recomputing the seed face separation.
        if !early_return || i != seed_index_a as usize {
            let direction = -mul_mv(inv_r, plane.normal);
            let plane_sep = dot(plane.normal, xf_b.p) - plane.offset;
            separation = get_face_separation(
                direction,
                plane_sep,
                vx_b,
                vy_b,
                vz_b,
                wide_vertex_count_b,
                c_b,
                h_b,
                &mut vertex_index,
            );
        }

        if separation > res.face_a.separation {
            res.face_a.normal = plane.normal;
            res.face_a.separation = separation;
            res.face_a.index_a = i as i32;
            res.face_a.index_b = vertex_index;
            if separation > speculative_distance && early_return {
                res.separated_feature = SeparatingFeature::FaceAxisA;
                return res;
            }
        }
    }

    b3_validate(res.face_a.index_a != b3::NULL_INDEX);

    let face_count_b = hull_b.face_count();
    let planes_b = hull_b.planes();

    let (vx_a, vy_a, vz_a) = hull_a.soa_vertices();
    let wide_vertex_count_a = round_up_to_simd_width(hull_a.vertex_count());

    let c_a = aabb_center(hull_a.aabb());
    let h_a = aabb_extents(hull_a.aabb());

    get_face_dots(hull_b, -mul_mv(inv_r, delta_center), &mut scratch.dot_b);

    // Find the face of hullB that most aligns with the vector pointing from centerB to centerA.
    let mut seed_index_b: i32 = 0;
    let mut max_dot_b = scratch.dot_b[0];
    for i in 1..face_count_b as usize {
        if scratch.dot_b[i] > max_dot_b {
            max_dot_b = scratch.dot_b[i];
            seed_index_b = i as i32;
        }
    }

    // Get a lower bound on the separation for the faces of hullB.
    let mut floor_b = -f32::INFINITY;
    let mut seed_separation_b = -f32::INFINITY;
    let mut seed_vertex_a: i32 = 0;
    if early_return {
        let plane = planes_b[seed_index_b as usize];
        let direction = -mul_mv(r, plane.normal);
        let plane_sep = dot(direction, xf_b.p) - plane.offset;
        seed_separation_b = get_face_separation(
            direction,
            plane_sep,
            vx_a,
            vy_a,
            vz_a,
            wide_vertex_count_a,
            c_a,
            h_a,
            &mut seed_vertex_a,
        );

        // Include the floor set by hull A faces.
        floor_b = max_float(seed_separation_b, res.face_a.separation);
        floor_b = min_float(floor_b, speculative_distance);
    }

    // Test B's face planes against A's vertices.
    for i in 0..face_count_b as usize {
        if scratch.dot_b[i] - radius_bound < max_float(floor_b, res.face_b.separation) {
            continue;
        }

        let plane = planes_b[i];
        let direction = -mul_mv(r, plane.normal);
        let mut vertex_index = seed_vertex_a;
        let mut separation = seed_separation_b;

        // Avoid recomputing the seed face separation.
        if !early_return || i != seed_index_b as usize {
            let plane_sep = dot(direction, xf_b.p) - plane.offset;
            separation = get_face_separation(
                direction,
                plane_sep,
                vx_a,
                vy_a,
                vz_a,
                wide_vertex_count_a,
                c_a,
                h_a,
                &mut vertex_index,
            );
        }

        if separation > res.face_b.separation {
            res.face_b.normal = direction;
            res.face_b.separation = separation;
            res.face_b.index_a = vertex_index;
            res.face_b.index_b = i as i32;
            if separation > speculative_distance && early_return {
                res.separated_feature = SeparatingFeature::FaceAxisB;
                return res;
            }
        }
    }

    // Gather edges of A that can feasibly create a separating axis that beats the maximum face
    // separation.
    let half_edge_count_a = hull_a.half_edge_count();
    let half_edges_a = hull_a.edges();
    let mut na: i32 = 0;

    let half_edge_count_b = hull_b.half_edge_count();
    let half_edges_b = hull_b.edges();
    let mut nb: i32 = 0;

    let max_face_separation = max_float(res.face_a.separation, res.face_b.separation);
    let bound_slack = radius - radius_bound;
    let threshold_a = if early_return {
        max_face_separation + hull_b.inner_radius() - bound_slack
    } else {
        -f32::INFINITY
    };
    let threshold_b = if early_return {
        max_face_separation + hull_a.inner_radius() - bound_slack
    } else {
        -f32::INFINITY
    };

    let center_b_in_a = mul_mv(r, hull_b.center()) + xf_b.p;
    let center_a_in_b = mul_mv(inv_r, hull_a.center() - xf_b.p);

    get_face_plane_separations(hull_a, center_b_in_a, &mut scratch.plane_dot_a);
    get_face_plane_separations(hull_b, center_a_in_b, &mut scratch.plane_dot_b);

    let cosines_a = hull_a.edge_cosines();
    let cosines_b = hull_b.edge_cosines();

    let mut i = 0usize;
    while i < half_edge_count_a as usize {
        let i1 = half_edges_a[i].face as usize;
        let i2 = half_edges_a[i + 1].face as usize;
        let c = cosines_a[i >> 1];
        scratch.edge_indices_a[na as usize] = i as i32;
        if test_edge_candidate_sorted(
            scratch.plane_dot_a[i1],
            scratch.plane_dot_a[i2],
            c,
            threshold_a,
        ) {
            na += 1;
        }
        i += 2;
    }

    let mut i = 0usize;
    while i < half_edge_count_b as usize {
        let i1 = half_edges_b[i].face as usize;
        let i2 = half_edges_b[i + 1].face as usize;
        let c = cosines_b[i >> 1];
        scratch.edge_indices_b[nb as usize] = i as i32;
        if test_edge_candidate_sorted(
            scratch.plane_dot_b[i1],
            scratch.plane_dot_b[i2],
            c,
            threshold_b,
        ) {
            nb += 1;
        }
        i += 2;
    }

    // Apply additional culling using the support points for the best face axes.
    if early_return && nb * ((na + 3) >> 2) >= EDGE_PROBE_MIN_TESTS {
        let probe_bound = max_face_separation
            - (0.1 * b3::linear_slop() + 0.001 * abs_float(center_distance + radius));

        let vertex_b = if res.face_a.index_b != b3::NULL_INDEX {
            res.face_a.index_b
        } else {
            seed_vertex_b
        };
        let probe_b = Vec3::new(
            vx_b[vertex_b as usize],
            vy_b[vertex_b as usize],
            vz_b[vertex_b as usize],
        );
        get_face_plane_separations(
            hull_a,
            mul_mv(r, probe_b) + xf_b.p,
            &mut scratch.plane_dot_a,
        );
        na = filter_edge_candidates(
            hull_a,
            &scratch.plane_dot_a,
            probe_bound,
            &mut scratch.edge_indices_a,
            na,
        );

        let vertex_a = if res.face_b.index_a != b3::NULL_INDEX {
            res.face_b.index_a
        } else {
            seed_vertex_a
        };
        let probe_a = Vec3::new(
            vx_a[vertex_a as usize],
            vy_a[vertex_a as usize],
            vz_a[vertex_a as usize],
        );
        get_face_plane_separations(
            hull_b,
            mul_mv(inv_r, probe_a - xf_b.p),
            &mut scratch.plane_dot_b,
        );
        nb = filter_edge_candidates(
            hull_b,
            &scratch.plane_dot_b,
            probe_bound,
            &mut scratch.edge_indices_b,
            nb,
        );
    }

    if na == 0 || nb == 0 {
        // No edge candidates found.
        return res;
    }

    // B face normals in A space, negated. B vertices in A space, negated.

    let (nx_b, ny_b, nz_b) = hull_b.soa_normals();
    let wide_face_count_b = round_up_to_simd_width(face_count_b);

    negative_transform_from_soa(
        r,
        xf_b.p,
        nx_b,
        ny_b,
        nz_b,
        wide_face_count_b,
        &mut scratch.b_fn_x,
        &mut scratch.b_fn_y,
        &mut scratch.b_fn_z,
        false,
    );
    negative_transform_from_soa(
        r,
        xf_b.p,
        vx_b,
        vy_b,
        vz_b,
        wide_vertex_count_b,
        &mut scratch.b_w_x,
        &mut scratch.b_w_y,
        &mut scratch.b_w_z,
        true,
    );

    // Per A edge data. n0 and n1 are the two face normals, d the edge vector, v0 the first vertex.
    // tol is the parallel edge tolerance, scaled by the edge length.

    let squared_tol = b3::PARALLEL_EDGE_TOLERANCE * b3::PARALLEL_EDGE_TOLERANCE;
    for k in 0..na as usize {
        let edge = half_edges_a[scratch.edge_indices_a[k] as usize];
        let twin = half_edges_a[scratch.edge_indices_a[k] as usize + 1];

        let a = planes_a[edge.face as usize].normal;
        let b = planes_a[twin.face as usize].normal;
        scratch.a_n0_x[k] = a.x;
        scratch.a_n0_y[k] = a.y;
        scratch.a_n0_z[k] = a.z;
        scratch.a_n1_x[k] = b.x;
        scratch.a_n1_y[k] = b.y;
        scratch.a_n1_z[k] = b.z;

        let v0 = edge.origin as usize;
        let v1 = twin.origin as usize;

        scratch.a_d_x[k] = vx_a[v1] - vx_a[v0];
        scratch.a_d_y[k] = vy_a[v1] - vy_a[v0];
        scratch.a_d_z[k] = vz_a[v1] - vz_a[v0];
        scratch.a_v0_x[k] = vx_a[v0];
        scratch.a_v0_y[k] = vy_a[v0];
        scratch.a_v0_z[k] = vz_a[v0];

        scratch.a_tol[k] = squared_tol
            * (scratch.a_d_x[k] * scratch.a_d_x[k]
                + scratch.a_d_y[k] * scratch.a_d_y[k]
                + scratch.a_d_z[k] * scratch.a_d_z[k]);
    }

    let linear_slop = b3::linear_slop();
    let eps = -linear_slop * linear_slop;

    // Scalar edge/edge test. The scalar fallback must agree with the SIMD path for cross platform
    // determinism.
    for j in 0..nb as usize {
        let edge = half_edges_b[scratch.edge_indices_b[j] as usize];
        let twin = half_edges_b[scratch.edge_indices_b[j] as usize + 1];
        let f0 = edge.face as usize;
        let f1 = twin.face as usize;
        let v0 = edge.origin as usize;
        let v1 = twin.origin as usize;

        let c = Vec3::new(
            scratch.b_fn_x[f0],
            scratch.b_fn_y[f0],
            scratch.b_fn_z[f0],
        );
        let d_normal = Vec3::new(
            scratch.b_fn_x[f1],
            scratch.b_fn_y[f1],
            scratch.b_fn_z[f1],
        );
        let bv0 = Vec3::new(
            scratch.b_w_x[v0],
            scratch.b_w_y[v0],
            scratch.b_w_z[v0],
        );
        let bv1 = Vec3::new(
            scratch.b_w_x[v1],
            scratch.b_w_y[v1],
            scratch.b_w_z[v1],
        );
        let dc = bv1 - bv0;

        for i in 0..na as usize {
            let d = Vec3::new(scratch.a_d_x[i], scratch.a_d_y[i], scratch.a_d_z[i]);

            // CBA = C.dir, DBA = D.dir, where dir = B_x_A
            let cba = dot(c, d);
            let dba = dot(d_normal, d);
            if cba * dba >= eps {
                continue;
            }

            let n0 = Vec3::new(
                scratch.a_n0_x[i],
                scratch.a_n0_y[i],
                scratch.a_n0_z[i],
            );
            let n1 = Vec3::new(
                scratch.a_n1_x[i],
                scratch.a_n1_y[i],
                scratch.a_n1_z[i],
            );

            // ADC = n0.DC, BDC = n1.DC, where DC = D_x_C
            let adc = dot(n0, dc);
            let bdc = dot(n1, dc);
            if adc * bdc >= eps || cba * bdc >= eps {
                continue;
            }

            // Reject near parallel edges
            let max_cd = max_float(cba * cba, dba * dba);
            if max_cd <= scratch.a_tol[i] {
                continue;
            }

            // t = -CBA / (DBA - CBA)
            let t = -cba / (dba - cba);

            // normal = lerp(t, C, D) = C + (D-C)*t
            let mut n = mul_add(c, t, d_normal - c);
            let len2 = dot(n, n);

            let inv = 1.0 / len2.sqrt();
            n = inv * n;

            // separation = -dot(normal, av0 + bv0)
            let av0 = Vec3::new(
                scratch.a_v0_x[i],
                scratch.a_v0_y[i],
                scratch.a_v0_z[i],
            );
            let separation = -dot(av0 + bv0, n);

            if separation > res.edge.separation {
                res.edge.normal = n;
                res.edge.separation = separation;

                // Half edge index
                res.edge.index_a = scratch.edge_indices_a[i];
                res.edge.index_b = scratch.edge_indices_b[j];

                if separation > speculative_distance && early_return {
                    res.separated_feature = SeparatingFeature::EdgePairAxis;
                    return res;
                }
            }
        }
    }

    res
}

// MARK: - Manifold point reduction

/// Reduce the manifold points to a maximum of 4 points. This modifies the input point array to
/// improve performance.
pub fn reduce_manifold_points(
    manifold: &mut LocalManifold,
    capacity: i32,
    points: &mut [LocalManifoldPoint],
    count: i32,
) {
    if capacity < 4 {
        return;
    }

    let mut count = count;

    if count <= 4 {
        for i in 0..count as usize {
            unsafe {
                *manifold.points.add(i) = points[i];
            }
        }

        manifold.point_count = count;
        return;
    }

    let normal = manifold.normal;
    let speculative_distance = b3::speculative_distance();
    let tol_sqr = speculative_distance * speculative_distance;

    // This bias is very important for contact point consistency across time steps.
    // It creates a pecking order to avoid flickering between candidates with similar scores.
    let bias: f32 = 0.95;

    // Step 1: find extreme point that is touching
    let mut best_index = b3::NULL_INDEX;
    let mut best_score = -f32::MAX;

    let search_direction = arbitrary_perp(normal);
    for index in 0..count as usize {
        let pt = &points[index];

        if pt.separation > speculative_distance {
            continue;
        }

        // The deeper the better
        let score = -pt.separation + dot(search_direction, pt.point);
        if bias * score > best_score {
            best_index = index as i32;
            best_score = score;
        }
    }

    b3_validate(0 <= best_index && best_index < count);
    if best_index == b3::NULL_INDEX {
        manifold.point_count = 0;
        return;
    }

    unsafe {
        *manifold.points = points[best_index as usize];
    }
    manifold.point_count = 1;

    // Remove best point from array
    points[best_index as usize] = points[count as usize - 1];
    count -= 1;

    let a = points_of(manifold)[0].point;

    // Step 2: Find farthest point in 2D
    best_score = 0.0;
    best_index = b3::NULL_INDEX;

    for index in 0..count as usize {
        let p = points[index].point;
        let d = p - a;
        let v = mul_sub(d, dot(d, normal), normal);
        let distance_squared = length_squared(v);
        let separation = max_float(0.0, -points[index].separation);
        let score = distance_squared + 4.0 * separation * separation;
        if bias * score > best_score {
            best_score = score;
            best_index = index as i32;
        }
    }

    if best_score < tol_sqr {
        return;
    }

    b3_assert(0 <= best_index && best_index < count);
    unsafe {
        *manifold.points.add(1) = points[best_index as usize];
    }
    manifold.point_count = 2;

    // Remove best point from array
    points[best_index as usize] = points[count as usize - 1];
    count -= 1;

    let b = points_of(manifold)[1].point;

    // Step 3: Find the point with the maximum triangular area
    best_score = tol_sqr;
    best_index = b3::NULL_INDEX;
    let mut best_signed_area: f32 = 0.0;
    let ba = b - a;
    for index in 0..count as usize {
        let p = points[index].point;
        let signed_area = dot(normal, cross(ba, p - a));
        let score = abs_float(signed_area);
        if bias * score >= best_score {
            best_score = score;
            best_index = index as i32;
            best_signed_area = signed_area;
        }
    }

    if best_index == b3::NULL_INDEX {
        return;
    }

    b3_assert(best_index != b3::NULL_INDEX);

    unsafe {
        *manifold.points.add(2) = points[best_index as usize];
    }
    manifold.point_count = 3;
    points[best_index as usize] = points[count as usize - 1];
    count -= 1;

    let c = points_of(manifold)[2].point;

    // Step 4: get the point that adds the most area outside the current triangle
    best_score = tol_sqr;
    best_index = b3::NULL_INDEX;
    let sign: f32 = if best_signed_area < 0.0 { -1.0 } else { 1.0 };
    for index in 0..count as usize {
        let p = points[index].point;
        let u1 = sign * dot(normal, cross(p - a, ba));
        let u2 = sign * dot(normal, cross(p - b, c - b));
        let u3 = sign * dot(normal, cross(p - c, a - c));
        let score = max_float(u1, max_float(u2, u3));

        if bias * score > best_score {
            best_score = score;
            best_index = index as i32;
        }
    }

    if best_index != b3::NULL_INDEX {
        unsafe {
            *manifold
                .points
                .add(manifold.point_count as usize) = points[best_index as usize];
        }
        manifold.point_count += 1;
    }
}

fn points_of(manifold: &LocalManifold) -> &[LocalManifoldPoint] {
    unsafe { std::slice::from_raw_parts(manifold.points, 4) }
}

// MARK: - Contact building

/// Build the incident face polygon in frame A.
pub fn build_polygon(
    out: &mut [ClipVertex],
    transform: Transform,
    hull: &Hull,
    inc_face: i32,
    ref_plane: Plane,
) -> i32 {
    let faces = hull.faces();
    let edges = hull.edges();
    let points = hull.points();

    let face = faces[inc_face as usize];
    let mut edge_index = face.edge as i32;
    b3_assert(edges[edge_index as usize].face as i32 == inc_face);

    let mut out_count: i32 = 0;

    let matrix = make_matrix_from_quat(transform.q);

    loop {
        let edge = edges[edge_index as usize];

        let next_edge_index = edge.next as i32;
        let next = edges[next_edge_index as usize];

        let mut vertex = ClipVertex::default();
        vertex.position = mul_mv(matrix, points[next.origin as usize]) + transform.p;
        vertex.separation = plane_separation(ref_plane, vertex.position);
        vertex.pair = make_feature_pair(
            FeatureOwner::ShapeB,
            edge_index,
            FeatureOwner::ShapeB,
            next_edge_index,
        );

        out[out_count as usize] = vertex;
        out_count += 1;

        edge_index = next_edge_index;
        if edge_index == face.edge as i32 || out_count >= crate::collision::manifold::MAX_CLIP_POINTS
        {
            break;
        }
    }

    b3_validate(validate_polygon(&out[..out_count as usize]));

    out_count
}

fn collect_face_points(
    input: &[ClipVertex],
    points: &mut [LocalManifoldPoint],
    point_count: i32,
    normal: Vec3,
) -> f32 {
    let mut min_separation = f32::MAX;

    for i in 0..point_count as usize {
        let clip_point = input[i];
        let pt = LocalManifoldPoint {
            // Using the half-way point keeps the points in the same position when swapping
            // reference face from A to B.
            point: mul_sub(clip_point.position, 0.5 * clip_point.separation, normal),
            separation: clip_point.separation,
            pair: clip_point.pair,
            triangle_index: 0,
        };

        min_separation = min_float(min_separation, clip_point.separation);

        points[i] = pt;
    }

    min_separation
}

/// Build a manifold for a face-A reference axis.
#[allow(clippy::too_many_arguments)]
pub fn build_face_a_contact(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &Hull,
    hull_b: &Hull,
    transform_b_to_a: Transform,
    query: SeparatingAxis,
    scratch: &mut FaceScratch,
    cache: &mut SATCache,
) -> bool {
    b3_assert(capacity > 0);
    b3_validate(query.kind == SeparatingFeature::FaceAxisA);
    b3_validate(0 <= query.index_a && query.index_a < hull_a.face_count());
    b3_validate(0 <= query.index_b && query.index_b < hull_b.vertex_count());

    let faces_a = hull_a.faces();
    let edges_a = hull_a.edges();
    let planes_a = hull_a.planes();
    let points_a = hull_a.points();

    // Reference face
    let ref_face = query.index_a;
    let ref_plane = planes_a[ref_face as usize];

    // Find incident face
    let ref_normal_in_b = inv_rotate_vector(transform_b_to_a.q, ref_plane.normal);
    let inc_face = find_incident_face(hull_b, ref_normal_in_b, query.index_b);

    // Build the clip polygons and reduced points in one uninitialized temporary slab.
    let mut point_count = build_polygon(
        &mut scratch.polygon1,
        transform_b_to_a,
        hull_b,
        inc_face,
        ref_plane,
    );

    // Clip incident face against side planes of reference face
    let ref_face_data = faces_a[ref_face as usize];
    let mut edge_index = ref_face_data.edge as i32;

    let mut use_buffer1_as_input = true;

    loop {
        let edge = edges_a[edge_index as usize];
        let next_edge_index = edge.next as i32;
        let next = edges_a[next_edge_index as usize];
        let vertex1 = points_a[edge.origin as usize];
        let vertex2 = points_a[next.origin as usize];
        let tangent = normalize(vertex2 - vertex1);
        let binormal = cross(tangent, ref_plane.normal);

        let clip_plane = make_plane_from_normal_and_point(binormal, vertex1);

        if use_buffer1_as_input {
            point_count = clip_polygon(
                &mut scratch.polygon2,
                &scratch.polygon1,
                point_count,
                clip_plane,
                edge_index,
                ref_plane,
            );
        } else {
            point_count = clip_polygon(
                &mut scratch.polygon1,
                &scratch.polygon2,
                point_count,
                clip_plane,
                edge_index,
                ref_plane,
            );
        }
        b3_assert(point_count <= crate::collision::manifold::MAX_CLIP_POINTS);

        use_buffer1_as_input = !use_buffer1_as_input;

        if point_count < 3 {
            *cache = SATCache::default();
            return false;
        }

        edge_index = next_edge_index;
        if edge_index == ref_face_data.edge as i32 {
            break;
        }
    }

    point_count = min_int(point_count, crate::collision::manifold::MAX_CLIP_POINTS);

    manifold.normal = ref_plane.normal;

    let min_separation = if use_buffer1_as_input {
        collect_face_points(
            &scratch.polygon1,
            &mut scratch.points,
            point_count,
            ref_plane.normal,
        )
    } else {
        collect_face_points(
            &scratch.polygon2,
            &mut scratch.points,
            point_count,
            ref_plane.normal,
        )
    };

    if min_separation >= b3::speculative_distance() {
        *cache = SATCache::default();
        return false;
    }

    reduce_manifold_points(manifold, capacity, &mut scratch.points, point_count);

    // Save cache
    cache.separation = min_separation;
    cache.type_ = SeparatingFeature::FaceAxisA;
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;

    true
}

/// Build a manifold for a face-B reference axis.
#[allow(clippy::too_many_arguments)]
pub fn build_face_b_contact(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &Hull,
    hull_b: &Hull,
    transform_b_to_a: Transform,
    query: SeparatingAxis,
    scratch: &mut FaceScratch,
    cache: &mut SATCache,
) -> bool {
    b3_validate(query.kind == SeparatingFeature::FaceAxisB);

    let transform_a_to_b = invert_transform(transform_b_to_a);

    let flipped_query = SeparatingAxis {
        normal: -query.normal,
        separation: query.separation,
        index_a: query.index_b,
        index_b: query.index_a,
        kind: SeparatingFeature::FaceAxisA,
    };

    let touching = build_face_a_contact(
        manifold,
        capacity,
        hull_b,
        hull_a,
        transform_a_to_b,
        flipped_query,
        scratch,
        cache,
    );
    if !touching {
        *cache = SATCache::default();
        return false;
    }

    // Results are in frame B, need to transform them into frame A
    let matrix = make_matrix_from_quat(transform_b_to_a.q);

    // Transform and flip normal so it points from A to B, even though B has the reference face.
    manifold.normal = -mul_mv(matrix, manifold.normal);

    // Transform points from frame B to frame A. Also flip the pairs to ensure correct matches.
    let point_count = manifold.point_count;
    let dest = manifold.points;
    b3_assert(!dest.is_null());
    for i in 0..point_count as usize {
        unsafe {
            let pt = &mut *dest.add(i);
            pt.point = mul_mv(matrix, pt.point) + transform_b_to_a.p;
            pt.pair = flip_pair(&pt.pair);
        }
    }

    cache.type_ = SeparatingFeature::FaceAxisB;
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;

    true
}

/// Build a single point manifold for an edge pair axis.
#[allow(clippy::too_many_arguments)]
pub fn build_edge_contact(
    manifold: &mut LocalManifold,
    hull_a: &Hull,
    hull_b: &Hull,
    transform_b_to_a: Transform,
    query: SeparatingAxis,
    cache: &mut SATCache,
) -> bool {
    b3_validate(query.kind == SeparatingFeature::EdgePairAxis);
    b3_validate(0 <= query.index_a && query.index_a < hull_a.half_edge_count());
    b3_validate(0 <= query.index_b && query.index_b < hull_b.half_edge_count());

    // Work in shapeA coordinates
    let edges_a = hull_a.edges();
    let points_a = hull_a.points();

    let edges_b = hull_b.edges();
    let points_b = hull_b.points();

    let edge_a = edges_a[query.index_a as usize];
    let twin_a = edges_a[edge_a.twin as usize];
    let p_a = points_a[edge_a.origin as usize];
    let q_a = points_a[twin_a.origin as usize];
    let e_a = q_a - p_a;

    let edge_b = edges_b[query.index_b as usize];
    let twin_b = edges_b[edge_b.twin as usize];
    let p_b = transform_point(transform_b_to_a, points_b[edge_b.origin as usize]);
    let q_b = transform_point(transform_b_to_a, points_b[twin_b.origin as usize]);
    let e_b = q_b - p_b;

    let normal = query.normal;
    let result = line_distance(p_a, e_a, p_b, e_b);

    if !is_within_segments(&result) {
        *cache = SATCache::default();
        return false;
    }

    // This can slide off the end from caching
    let separation = dot(normal, result.point2 - result.point1);
    let point = 0.5 * (result.point1 + result.point2);

    // Result in frame A
    manifold.normal = normal;
    manifold.point_count = 1;

    let pt = manifold.points;
    b3_assert(!pt.is_null());
    unsafe {
        (*pt).point = point;
        (*pt).separation = separation;
        (*pt).pair = make_feature_pair(
            FeatureOwner::ShapeA,
            query.index_a,
            FeatureOwner::ShapeB,
            query.index_b,
        );
    }

    // Save cache
    cache.separation = separation;
    cache.type_ = SeparatingFeature::EdgePairAxis;
    cache.index_a = query.index_a as u8;
    cache.index_b = query.index_b as u8;

    true
}

/// Is the segment distance result inside both segments?
#[inline]
pub fn is_within_segments(result: &SegmentDistanceResult) -> bool {
    (0.0 <= result.fraction1 && result.fraction1 <= 1.0)
        && (0.0 <= result.fraction2 && result.fraction2 <= 1.0)
}

// MARK: - Hull collision

/// Test/standalone entry point.
pub fn collide_hulls(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &Hull,
    hull_b: &Hull,
    transform_b_to_a: Transform,
    cache: &mut SATCache,
) {
    // Swift's `collideHulls` overload allocated a scratch per call; the Rust caller reuses its
    // worker scratch instead, so this entry point owns one for the duration of the call.
    let mut sat = SatScratch::new();
    let mut face = FaceScratch::new();
    collide_hulls_with_scratch(
        manifold,
        capacity,
        hull_a,
        hull_b,
        transform_b_to_a,
        &mut sat,
        &mut face,
        cache,
    );
}

/// Compute the contact manifold between two hulls. The transform maps B into A's frame.
#[allow(clippy::too_many_arguments)]
pub fn collide_hulls_with_scratch(
    manifold: &mut LocalManifold,
    capacity: i32,
    hull_a: &Hull,
    hull_b: &Hull,
    transform_b_to_a: Transform,
    sat: &mut SatScratch,
    face: &mut FaceScratch,
    cache: &mut SATCache,
) {
    manifold.point_count = 0;

    if capacity < 4 {
        return;
    }

    // Work in shapeA coordinates
    let speculative_distance = b3::speculative_distance();
    let linear_slop = b3::linear_slop();

    let edges_a = hull_a.edges();
    let planes_a = hull_a.planes();
    let points_a = hull_a.points();

    let edges_b = hull_b.edges();
    let planes_b = hull_b.planes();
    let points_b = hull_b.points();

    cache.hit = 0;

    // Attempt to use the cache to speed up collision
    match cache.type_ {
        SeparatingFeature::InvalidAxis => {}

        SeparatingFeature::FaceAxisA => {
            b3_assert((cache.index_a as i32) < hull_a.face_count());

            // Check for separation using cached face
            let plane = planes_a[cache.index_a as usize];
            let search_direction_in_b = -inv_rotate_vector(transform_b_to_a.q, plane.normal);

            let vertex_index = find_hull_support_vertex(hull_b, search_direction_in_b);
            let support = transform_point(transform_b_to_a, points_b[vertex_index as usize]);
            let separation = plane_separation(plane, support);

            if separation >= speculative_distance {
                // Cache hit, shapes are separated
                cache.hit = 1;
                return;
            }

            // Attempt face contact using cached feature
            let face_query = SeparatingAxis {
                normal: plane.normal,
                separation: 0.0,
                index_a: cache.index_a as i32,
                index_b: vertex_index,
                kind: SeparatingFeature::FaceAxisA,
            };

            let mut local_cache = SATCache::default();
            let touching = build_face_a_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                face_query,
                face,
                &mut local_cache,
            );
            if touching && abs_float(cache.separation - local_cache.separation) < linear_slop {
                // Cache hit, contact points generated
                cache.hit = 1;
                return;
            }
        }

        SeparatingFeature::FaceAxisB => {
            b3_assert((cache.index_b as i32) < hull_b.face_count());

            // Check for separation using cached face
            let plane = planes_b[cache.index_b as usize];
            let search_direction_in_a = -rotate_vector(transform_b_to_a.q, plane.normal);

            let vertex_index = find_hull_support_vertex(hull_a, search_direction_in_a);
            let support = inv_transform_point(transform_b_to_a, points_a[vertex_index as usize]);
            let separation = plane_separation(plane, support);

            if separation >= speculative_distance {
                // Cache hit, shapes are separated
                cache.hit = 1;
                return;
            }

            // Attempt face contact using cached feature
            let face_query = SeparatingAxis {
                normal: -plane.normal,
                separation: 0.0,
                index_a: vertex_index,
                index_b: cache.index_b as i32,
                kind: SeparatingFeature::FaceAxisB,
            };

            let mut local_cache = SATCache::default();
            let touching = build_face_b_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                face_query,
                face,
                &mut local_cache,
            );
            if touching && abs_float(cache.separation - local_cache.separation) < linear_slop {
                // Cache hit, contact points generated
                cache.hit = 1;
                return;
            }
        }

        SeparatingFeature::EdgePairAxis => {
            let index_a = cache.index_a as i32;
            let edge1 = edges_a[index_a as usize];
            let twin1 = edges_a[index_a as usize + 1];
            b3_assert(edge1.twin as i32 == index_a + 1 && twin1.twin as i32 == index_a);

            let p_a = points_a[edge1.origin as usize];
            let q_a = points_a[twin1.origin as usize];
            let e_a = q_a - p_a;

            let u_a = planes_a[edge1.face as usize].normal;
            let v_a = planes_a[twin1.face as usize].normal;

            let index_b = cache.index_b as i32;
            let edge2 = edges_b[index_b as usize];
            let twin2 = edges_b[index_b as usize + 1];
            b3_assert(edge2.twin as i32 == index_b + 1 && twin2.twin as i32 == index_b);

            let p_b = transform_point(transform_b_to_a, points_b[edge2.origin as usize]);
            let q_b = transform_point(transform_b_to_a, points_b[twin2.origin as usize]);
            let e_b = q_b - p_b;

            let u_b = rotate_vector(transform_b_to_a.q, planes_b[edge2.face as usize].normal);
            let v_b = rotate_vector(transform_b_to_a.q, planes_b[twin2.face as usize].normal);

            // flipping the signs of u2 and v2
            // cross(v2, u2) == cross(-v2, -u2)
            // so we still use -e2
            // but we can also use e1 = cross(u1, v1) and e2 = cross(u2, v2)
            let cba = dot(u_b, e_a);
            let dba = dot(v_b, e_a);
            let adc = -dot(u_a, e_b);
            let bdc = -dot(v_a, e_b);

            if cba * dba < 0.0 && adc * bdc < 0.0 && cba * bdc > 0.0 {
                // Avoid nearly parallel edges that may lead to invalid separation values at the
                // noise floor.
                let squared_tolerance = b3::PARALLEL_EDGE_TOLERANCE * b3::PARALLEL_EDGE_TOLERANCE;
                if max_float(cba * cba, dba * dba) >= squared_tolerance * length_squared(e_a) {
                    let t = cba / (cba - dba);
                    let mut axis = lerp(u_b, v_b, t);
                    b3_validate(length_squared(axis) > 1000.0 * f32::MIN_POSITIVE);
                    axis = normalize(axis);
                    let separation = dot(axis, q_a - q_b);

                    if separation > speculative_distance {
                        // Cache hit, shapes are separated
                        cache.hit = 1;
                        return;
                    }

                    // Try to rebuild contact from last features
                    let edge_query = SeparatingAxis {
                        normal: -axis,
                        separation: 0.0,
                        index_a: cache.index_a as i32,
                        index_b: cache.index_b as i32,
                        kind: SeparatingFeature::EdgePairAxis,
                    };

                    let mut local_cache = SATCache::default();
                    let touching = build_edge_contact(
                        manifold,
                        hull_a,
                        hull_b,
                        transform_b_to_a,
                        edge_query,
                        &mut local_cache,
                    );

                    // This separation tolerance may have a big impact on performance in some
                    // benchmarks.
                    if touching && abs_float(cache.separation - local_cache.separation) < linear_slop
                    {
                        // Cache hit, contact point generated
                        cache.hit = 1;
                        return;
                    }
                }
            }
        }

        SeparatingFeature::ManualFaceAxisA => {
            let axis_query = compute_separating_axis(
                hull_a,
                hull_b,
                transform_b_to_a,
                false,
                sat,
            );
            let _ = build_face_a_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                axis_query.face_a,
                face,
                cache,
            );
            return;
        }

        SeparatingFeature::ManualFaceAxisB => {
            let axis_query = compute_separating_axis(
                hull_a,
                hull_b,
                transform_b_to_a,
                false,
                sat,
            );
            let _ = build_face_b_contact(
                manifold,
                capacity,
                hull_a,
                hull_b,
                transform_b_to_a,
                axis_query.face_b,
                face,
                cache,
            );
            return;
        }

        SeparatingFeature::ManualEdgePairAxis => {
            let axis_query = compute_separating_axis(
                hull_a,
                hull_b,
                transform_b_to_a,
                false,
                sat,
            );
            let edge_query = axis_query.edge;
            if edge_query.index_a != b3::NULL_INDEX {
                let _ = build_edge_contact(
                    manifold,
                    hull_a,
                    hull_b,
                    transform_b_to_a,
                    edge_query,
                    cache,
                );
            }
            return;
        }

        SeparatingFeature::BacksideAxis | SeparatingFeature::ClosestPointsAxis => {
            b3_assert(false);
        }
    }

    manifold.point_count = 0;
    *cache = SATCache::default();

    let axis_query =
        compute_separating_axis(hull_a, hull_b, transform_b_to_a, true, sat);

    if axis_query.separated_feature != SeparatingFeature::InvalidAxis {
        // We found a separating axis
        cache.type_ = axis_query.separated_feature;

        if axis_query.separated_feature == SeparatingFeature::FaceAxisA {
            b3_validate(axis_query.face_a.separation > speculative_distance);
            cache.separation = axis_query.face_a.separation;
            cache.index_a = axis_query.face_a.index_a as u8;
            cache.index_b = axis_query.face_a.index_b as u8;
        } else if axis_query.separated_feature == SeparatingFeature::FaceAxisB {
            b3_validate(axis_query.face_b.separation > speculative_distance);
            cache.separation = axis_query.face_b.separation;
            cache.index_a = axis_query.face_b.index_a as u8;
            cache.index_b = axis_query.face_b.index_b as u8;
        } else {
            b3_assert(axis_query.separated_feature == SeparatingFeature::EdgePairAxis);
            b3_validate(axis_query.edge.separation > speculative_distance);
            cache.separation = axis_query.edge.separation;
            cache.index_a = axis_query.edge.index_a as u8;
            cache.index_b = axis_query.edge.index_b as u8;
        }
        return;
    }

    b3_validate(
        axis_query.face_a.separation <= speculative_distance
            || axis_query.face_b.separation <= speculative_distance
            || axis_query.edge.separation <= speculative_distance,
    );

    if axis_query.face_a.separation > axis_query.face_b.separation {
        let face_query = axis_query.face_a;
        b3_validate(0 <= face_query.index_a && face_query.index_a < hull_a.face_count());
        b3_validate(0 <= face_query.index_b && face_query.index_b < hull_b.vertex_count());

        // Face contact A
        let _ = build_face_a_contact(
            manifold,
            capacity,
            hull_a,
            hull_b,
            transform_b_to_a,
            face_query,
            face,
            cache,
        );

        b3_validate((cache.index_a as i32) < hull_a.face_count());
        b3_validate((cache.index_b as i32) < hull_b.vertex_count());
    } else {
        let face_query = axis_query.face_b;
        b3_validate(0 <= face_query.index_a && face_query.index_a < hull_a.vertex_count());
        b3_validate(0 <= face_query.index_b && face_query.index_b < hull_b.face_count());

        // Face contact B
        let _ = build_face_b_contact(
            manifold,
            capacity,
            hull_a,
            hull_b,
            transform_b_to_a,
            face_query,
            face,
            cache,
        );

        b3_validate((cache.index_a as i32) < hull_a.vertex_count());
        b3_validate((cache.index_b as i32) < hull_b.face_count());
    }

    let edge_query = axis_query.edge;

    if edge_query.index_a == b3::NULL_INDEX {
        // There are no valid edge pairs (all edges parallel)
        return;
    }

    let face_separation = max_float(axis_query.face_a.separation, axis_query.face_b.separation);
    let clip_separation = cache.separation;
    let edge_tol = linear_slop;

    // Face contact can be empty if it is not the axis of maximum separation. It can also be empty
    // in narrow cases in the speculative region. If that case was important then a GJK fallback
    // would be used. So far it doesn't seem important.
    // Create edge contact if face contact fails or edge contact is significantly better.
    if (manifold.point_count == 0 && edge_query.separation > face_separation)
        || edge_query.separation > clip_separation + edge_tol
    {
        b3_assert(0 <= edge_query.index_a && edge_query.index_a < hull_a.half_edge_count());
        b3_assert(0 <= edge_query.index_b && edge_query.index_b < hull_b.half_edge_count());

        // Edge contact
        let mut edge_point = LocalManifoldPoint::default();
        let mut edge_manifold = LocalManifold {
            points: &mut edge_point as *mut LocalManifoldPoint,
            ..Default::default()
        };

        let mut edge_cache = SATCache::default();
        let _ = build_edge_contact(
            &mut edge_manifold,
            hull_a,
            hull_b,
            transform_b_to_a,
            edge_query,
            &mut edge_cache,
        );

        // It is possible with speculation to have vertex-vertex collision that is missed by SAT,
        // so edge contact yields no points. In that case perhaps the face contact has some points.
        if edge_manifold.point_count == 1 {
            // Copy edge manifold out, being careful to preserve manifold point buffer.
            let points = manifold.points;
            *manifold = edge_manifold;
            manifold.points = points;
            unsafe {
                *manifold.points = edge_point;
            }
            *cache = edge_cache;
        }
    }
}
