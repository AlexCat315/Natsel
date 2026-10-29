//! GJK distance and conservative-advancement shape casting.

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_log, b3_validate};
use crate::collision::internal_math::{blend3, scalar_triple_product};
use crate::math::functions::*;
use crate::math::types::{Transform, Vec3};
use crate::types::queries::*;

pub const MAX_SIMPLEX_VERTICES: i32 = 4;
pub const MAX_GJK_ITERATIONS: i32 = 32;

// MARK: - Support

/// Get the support vertex of a proxy along an axis. The first vertex is moved to the origin for
/// improved precision, so `axis` is expected in the proxy's frame.
pub fn get_proxy_support(proxy: &ShapeProxy, axis: Vec3) -> i32 {
    let points = proxy.points();
    b3_assert(proxy.count > 0);
    b3_assert(!points.is_empty());

    let origin = points[0];
    let mut max_index: i32 = 0;
    let mut max_projection: f32 = 0.0;

    let mut index: i32 = 1;
    while index < proxy.count {
        let projection = dot(axis, points[index as usize] - origin);
        if projection > max_projection {
            max_index = index;
            max_projection = projection;
        }
        index += 1;
    }
    max_index
}

/// Get the support vertex of a point cloud along an axis, with the first point shifted to the origin.
pub fn get_point_support(points: &[Vec3], axis: Vec3) -> i32 {
    b3_assert(!points.is_empty());
    let origin = points[0];
    let mut max_index: i32 = 0;
    let mut max_projection: f32 = 0.0;

    let mut index: usize = 1;
    while index < points.len() {
        let projection = dot(axis, points[index] - origin);
        if projection > max_projection {
            max_index = index as i32;
            max_projection = projection;
        }
        index += 1;
    }
    max_index
}

// MARK: - Barycentric coordinates

/// Barycentric coordinates of the closest point on the segment `a-b` to the origin.
pub fn barycentric_coords_edge(a: Vec3, b: Vec3) -> (f32, f32, f32) {
    let ab = b - a;
    let divisor = dot(ab, ab);
    let u = dot(b, ab);
    let v = -dot(a, ab);
    (u, v, divisor)
}

/// Barycentric coordinates of the closest point on the triangle `a-b-c` to the origin.
pub fn barycentric_coords_tri(a: Vec3, b: Vec3, c: Vec3) -> (f32, f32, f32, f32) {
    let ab = b - a;
    let ac = c - a;
    let b_xc = cross(b, c);
    let c_xa = cross(c, a);
    let a_xb = cross(a, b);
    let ab_xac = cross(ab, ac);
    let divisor = dot(ab_xac, ab_xac);
    let u = dot(b_xc, ab_xac);
    let v = dot(c_xa, ab_xac);
    let w = dot(a_xb, ab_xac);
    (u, v, w, divisor)
}

/// Barycentric coordinates of the closest point on the tetrahedron `a-b-c-d` to the origin.
pub fn barycentric_coords_tet(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> (f32, f32, f32, f32, f32) {
    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let divisor = scalar_triple_product(ab, ac, ad);
    let sign: f32 = if divisor < 0.0 { -1.0 } else { 1.0 };
    let u = sign * scalar_triple_product(b, c, d);
    let v = sign * scalar_triple_product(a, d, c);
    let w = sign * scalar_triple_product(a, b, d);
    let x = sign * scalar_triple_product(a, c, b);
    (u, v, w, x, sign * divisor)
}

// MARK: - Simplex helpers

/// The metric of a simplex: length, area, or volume.
pub fn get_metric(simplex: &Simplex) -> f32 {
    let count = simplex.count;
    b3_assert(1 <= count && count <= 4);
    let vertices = &simplex.vertices;

    match count {
        1 => 0.0,
        2 => distance(vertices[0].w, vertices[1].w),
        3 => length(cross(vertices[1].w - vertices[0].w, vertices[2].w - vertices[0].w)) / 2.0,
        4 => {
            scalar_triple_product(
                vertices[1].w - vertices[0].w,
                vertices[2].w - vertices[0].w,
                vertices[3].w - vertices[0].w,
            ) / 6.0
        }
        _ => {
            b3_assert(false);
            0.0
        }
    }
}

/// Store the simplex in the cache for warm starting.
pub fn write_cache(cache: &mut SimplexCache, simplex: &Simplex) {
    let count = simplex.count;
    cache.metric = get_metric(simplex);
    cache.count = count as u16;
    for index in 0..count as usize {
        cache.index_a[index] = simplex.vertices[index].index_a as u8;
        cache.index_b[index] = simplex.vertices[index].index_b as u8;
    }
}

/// Solve the closest point on a 2-simplex. Returns false if there was no progress.
pub fn solve_simplex2(simplex: &mut Simplex) -> bool {
    b3_assert(simplex.count == 2);
    let a = simplex.vertices[0].w;
    let b = simplex.vertices[1].w;
    let ab = b - a;
    let divisor = dot(ab, ab);
    let u = dot(b, ab);
    let v = -dot(a, ab);

    if v <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    if u <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = simplex.vertices[1];
        simplex.vertices[0].a = 1.0;
        return true;
    }

    if divisor <= 0.0 {
        return false;
    }

    let denominator = 1.0 / divisor;
    simplex.vertices[0].a = denominator * u;
    simplex.vertices[1].a = denominator * v;
    true
}

/// Solve the closest point on a 3-simplex. Returns false if there was no progress.
pub fn solve_simplex3(simplex: &mut Simplex) -> bool {
    b3_assert(simplex.count == 3);
    let v1 = simplex.vertices[0];
    let v2 = simplex.vertices[1];
    let v3 = simplex.vertices[2];

    let w_ab = barycentric_coords_edge(v1.w, v2.w);
    let w_bc = barycentric_coords_edge(v2.w, v3.w);
    let w_ca = barycentric_coords_edge(v3.w, v1.w);

    if w_ab.1 <= 0.0 && w_ca.0 <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = v1;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    if w_bc.1 <= 0.0 && w_ab.0 <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = v2;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    if w_ca.1 <= 0.0 && w_bc.0 <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = v3;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    let w_abc = barycentric_coords_tri(v1.w, v2.w, v3.w);

    if w_abc.2 <= 0.0 && w_ab.0 > 0.0 && w_ab.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = v1;
        simplex.vertices[1] = v2;
        let divisor = w_ab.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_ab.0 / divisor;
        simplex.vertices[1].a = w_ab.1 / divisor;
        return true;
    }

    if w_abc.0 <= 0.0 && w_bc.0 > 0.0 && w_bc.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = v2;
        simplex.vertices[1] = v3;
        let divisor = w_bc.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_bc.0 / divisor;
        simplex.vertices[1].a = w_bc.1 / divisor;
        return true;
    }

    if w_abc.1 <= 0.0 && w_ca.0 > 0.0 && w_ca.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = v3;
        simplex.vertices[1] = v1;
        let divisor = w_ca.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_ca.0 / divisor;
        simplex.vertices[1].a = w_ca.1 / divisor;
        return true;
    }

    let divisor = w_abc.3;
    if divisor <= 0.0 {
        return false;
    }

    simplex.vertices[0].a = w_abc.0 / divisor;
    simplex.vertices[1].a = w_abc.1 / divisor;
    simplex.vertices[2].a = w_abc.2 / divisor;
    true
}

/// Solve the closest point on a 4-simplex. Returns false if there was no progress.
pub fn solve_simplex4(simplex: &mut Simplex) -> bool {
    b3_assert(simplex.count == 4);
    let vertex_a = simplex.vertices[0];
    let vertex_b = simplex.vertices[1];
    let vertex_c = simplex.vertices[2];
    let vertex_d = simplex.vertices[3];

    let w_ab = barycentric_coords_edge(vertex_a.w, vertex_b.w);
    let w_ac = barycentric_coords_edge(vertex_a.w, vertex_c.w);
    let w_ad = barycentric_coords_edge(vertex_a.w, vertex_d.w);
    let w_bc = barycentric_coords_edge(vertex_b.w, vertex_c.w);
    let w_cd = barycentric_coords_edge(vertex_c.w, vertex_d.w);
    let w_db = barycentric_coords_edge(vertex_d.w, vertex_b.w);

    if w_ab.1 <= 0.0 && w_ac.1 <= 0.0 && w_ad.1 <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    if w_ab.0 <= 0.0 && w_db.0 <= 0.0 && w_bc.1 <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = vertex_b;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    if w_ac.0 <= 0.0 && w_bc.0 <= 0.0 && w_cd.1 <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = vertex_c;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    if w_ad.0 <= 0.0 && w_cd.0 <= 0.0 && w_db.1 <= 0.0 {
        simplex.count = 1;
        simplex.vertices[0] = vertex_d;
        simplex.vertices[0].a = 1.0;
        return true;
    }

    let w_acb = barycentric_coords_tri(vertex_a.w, vertex_c.w, vertex_b.w);
    let w_abd = barycentric_coords_tri(vertex_a.w, vertex_b.w, vertex_d.w);
    let w_adc = barycentric_coords_tri(vertex_a.w, vertex_d.w, vertex_c.w);
    let w_bcd = barycentric_coords_tri(vertex_b.w, vertex_c.w, vertex_d.w);

    if w_abd.2 <= 0.0 && w_acb.1 <= 0.0 && w_ab.0 > 0.0 && w_ab.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_b;
        let divisor = w_ab.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_ab.0 / divisor;
        simplex.vertices[1].a = w_ab.1 / divisor;
        return true;
    }

    if w_acb.2 <= 0.0 && w_adc.1 <= 0.0 && w_ac.0 > 0.0 && w_ac.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_c;
        let divisor = w_ac.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_ac.0 / divisor;
        simplex.vertices[1].a = w_ac.1 / divisor;
        return true;
    }

    if w_adc.2 <= 0.0 && w_abd.1 <= 0.0 && w_ad.0 > 0.0 && w_ad.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_d;
        let divisor = w_ad.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_ad.0 / divisor;
        simplex.vertices[1].a = w_ad.1 / divisor;
        return true;
    }

    if w_acb.0 <= 0.0 && w_bcd.2 <= 0.0 && w_bc.0 > 0.0 && w_bc.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_b;
        simplex.vertices[1] = vertex_c;
        let divisor = w_bc.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_bc.0 / divisor;
        simplex.vertices[1].a = w_bc.1 / divisor;
        return true;
    }

    if w_adc.0 <= 0.0 && w_bcd.0 <= 0.0 && w_cd.0 > 0.0 && w_cd.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_c;
        simplex.vertices[1] = vertex_d;
        let divisor = w_cd.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_cd.0 / divisor;
        simplex.vertices[1].a = w_cd.1 / divisor;
        return true;
    }

    if w_abd.0 <= 0.0 && w_bcd.1 <= 0.0 && w_db.0 > 0.0 && w_db.1 > 0.0 {
        simplex.count = 2;
        simplex.vertices[0] = vertex_d;
        simplex.vertices[1] = vertex_b;
        let divisor = w_db.2;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_db.0 / divisor;
        simplex.vertices[1].a = w_db.1 / divisor;
        return true;
    }

    let w_abcd = barycentric_coords_tet(vertex_a.w, vertex_b.w, vertex_c.w, vertex_d.w);

    if w_abcd.3 < 0.0 && w_acb.0 > 0.0 && w_acb.1 > 0.0 && w_acb.2 > 0.0 {
        simplex.count = 3;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_c;
        simplex.vertices[2] = vertex_b;
        let divisor = w_acb.3;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_acb.0 / divisor;
        simplex.vertices[1].a = w_acb.1 / divisor;
        simplex.vertices[2].a = w_acb.2 / divisor;
        return true;
    }

    if w_abcd.2 < 0.0 && w_abd.0 > 0.0 && w_abd.1 > 0.0 && w_abd.2 > 0.0 {
        simplex.count = 3;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_b;
        simplex.vertices[2] = vertex_d;
        let divisor = w_abd.3;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_abd.0 / divisor;
        simplex.vertices[1].a = w_abd.1 / divisor;
        simplex.vertices[2].a = w_abd.2 / divisor;
        return true;
    }

    if w_abcd.1 < 0.0 && w_adc.0 > 0.0 && w_adc.1 > 0.0 && w_adc.2 > 0.0 {
        simplex.count = 3;
        simplex.vertices[0] = vertex_a;
        simplex.vertices[1] = vertex_d;
        simplex.vertices[2] = vertex_c;
        let divisor = w_adc.3;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_adc.0 / divisor;
        simplex.vertices[1].a = w_adc.1 / divisor;
        simplex.vertices[2].a = w_adc.2 / divisor;
        return true;
    }

    if w_abcd.0 < 0.0 && w_bcd.0 > 0.0 && w_bcd.1 > 0.0 && w_bcd.2 > 0.0 {
        simplex.count = 3;
        simplex.vertices[0] = vertex_b;
        simplex.vertices[1] = vertex_c;
        simplex.vertices[2] = vertex_d;
        let divisor = w_bcd.3;
        if divisor <= 0.0 {
            return false;
        }
        simplex.vertices[0].a = w_bcd.0 / divisor;
        simplex.vertices[1].a = w_bcd.1 / divisor;
        simplex.vertices[2].a = w_bcd.2 / divisor;
        return true;
    }

    let divisor = w_abcd.4;
    if divisor <= 0.0 {
        return false;
    }

    simplex.vertices[0].a = w_abcd.0 / divisor;
    simplex.vertices[1].a = w_abcd.1 / divisor;
    simplex.vertices[2].a = w_abcd.2 / divisor;
    simplex.vertices[3].a = w_abcd.3 / divisor;
    true
}

/// The witness points on each shape, in frame A.
pub fn compute_witness_points(simplex: &Simplex, vertex_a: &mut Vec3, vertex_b: &mut Vec3) {
    let vs = &simplex.vertices;
    let count = simplex.count;
    b3_assert(1 <= count && count <= 4);

    match count {
        1 => {
            *vertex_a = vs[0].wa;
            *vertex_b = vs[0].wb;
        }
        2 => {
            *vertex_a = blend2(vs[0].a, vs[0].wa, vs[1].a, vs[1].wa);
            *vertex_b = blend2(vs[0].a, vs[0].wb, vs[1].a, vs[1].wb);
        }
        3 => {
            *vertex_a = blend3(vs[0].a, vs[0].wa, vs[1].a, vs[1].wa, vs[2].a, vs[2].wa);
            *vertex_b = blend3(vs[0].a, vs[0].wb, vs[1].a, vs[1].wb, vs[2].a, vs[2].wb);
        }
        4 => {
            let sum = blend2(vs[0].a, vs[0].wa, vs[1].a, vs[1].wa)
                + blend2(vs[2].a, vs[2].wa, vs[3].a, vs[3].wa);
            *vertex_a = sum;
            *vertex_b = sum;
        }
        _ => b3_assert(false),
    }
}

/// The closest point on the current simplex.
pub fn closest_point_on_simplex(simplex: &Simplex) -> Vec3 {
    let vs = &simplex.vertices;
    match simplex.count {
        1 => vs[0].w,
        2 => blend2(vs[0].a, vs[0].w, vs[1].a, vs[1].w),
        3 => blend3(vs[0].a, vs[0].w, vs[1].a, vs[1].w, vs[2].a, vs[2].w),
        4 => blend2(vs[0].a, vs[0].w, vs[1].a, vs[1].w)
            + blend2(vs[2].a, vs[2].w, vs[3].a, vs[3].w),
        _ => {
            b3_assert(false);
            Vec3::default()
        }
    }
}

// MARK: - GJK distance

/// Compute the closest points between two convex shapes represented as point clouds. The query
/// runs in frame A and returns witness points and the normal in frame A.
pub fn shape_distance(
    input: &DistanceInput,
    cache: &mut SimplexCache,
    simplexes: Option<&mut [Simplex]>,
) -> DistanceOutput {
    let xf = input.transform;
    let m = make_matrix_from_quat(xf.q);
    let mt = transpose(m);

    let proxy_a = &input.proxy_a;
    let proxy_b = &input.proxy_b;
    let points_a = proxy_a.points();
    let points_b = proxy_b.points();

    b3_assert(cache.count as i32 <= MAX_SIMPLEX_VERTICES);

    let mut simplex = Simplex::default();
    simplex.count = cache.count as i32;
    for i in 0..cache.count as usize {
        let index1 = cache.index_a[i] as i32;
        let index2 = cache.index_b[i] as i32;
        b3_assert(0 <= index1 && index1 < proxy_a.count);
        b3_assert(0 <= index2 && index2 < proxy_b.count);

        let vertex1 = points_a[index1 as usize];
        let vertex2 = mul_mv(m, points_b[index2 as usize]) + xf.p;
        simplex.vertices[i].index_a = index1;
        simplex.vertices[i].index_b = index2;
        simplex.vertices[i].wa = vertex1;
        simplex.vertices[i].wb = vertex2;
        simplex.vertices[i].w = vertex2 - vertex1;
        simplex.vertices[i].a = 0.0;
    }

    if simplex.count > 0 {
        let metric1 = cache.metric;
        let metric2 = get_metric(&simplex);
        if 2.0 * metric1 < metric2 || metric2 < 0.5 * metric1 || metric2 < f32::EPSILON {
            simplex.count = 0;
        }
    }

    if simplex.count == 0 {
        let vertex1 = points_a[0];
        let vertex2 = mul_mv(m, points_b[0]) + xf.p;
        simplex.count = 1;
        simplex.vertices[0].index_a = 0;
        simplex.vertices[0].index_b = 0;
        simplex.vertices[0].wa = vertex1;
        simplex.vertices[0].wb = vertex2;
        simplex.vertices[0].w = vertex2 - vertex1;
        simplex.vertices[0].a = 0.0;
    }

    let mut backup = Simplex::default();

    let mut simplex_index = 0usize;
    if let Some(output) = simplexes {
        if simplex_index < output.len() {
            output[simplex_index] = simplex;
            simplex_index += 1;
        }
    }

    let mut distance_output = DistanceOutput::default();
    let mut distance_sq = f32::MAX;
    let mut normal = Vec3::default();

    let mut iteration: i32 = 0;
    while iteration < MAX_GJK_ITERATIONS {
        let solved = match simplex.count {
            1 => {
                simplex.vertices[0].a = 1.0;
                true
            }
            2 => solve_simplex2(&mut simplex),
            3 => solve_simplex3(&mut simplex),
            4 => solve_simplex4(&mut simplex),
            _ => {
                b3_assert(false);
                false
            }
        };

        if !solved {
            b3_assert(backup.count != 0);
            simplex = backup;
            break;
        }

        if simplex.count == MAX_SIMPLEX_VERTICES {
            let mut local_point_a = Vec3::default();
            let mut local_point_b = Vec3::default();
            compute_witness_points(&simplex, &mut local_point_a, &mut local_point_b);
            distance_output.point_a = local_point_a;
            distance_output.point_b = local_point_b;
            distance_output.normal = Vec3::default();
            distance_output.distance = 0.0;
            distance_output.iterations = iteration;
            return distance_output;
        }

        let old_distance_sq = distance_sq;
        let closest_point = closest_point_on_simplex(&simplex);
        distance_sq = dot(closest_point, closest_point);

        if distance_sq >= old_distance_sq {
            b3_assert(backup.count != 0);
            simplex = backup;
            break;
        }

        let search_direction = match simplex.count {
            1 => -simplex.vertices[0].w,
            2 => {
                let a = simplex.vertices[0].w;
                let b = simplex.vertices[1].w;
                let ab = b - a;
                cross(cross(ab, -a), ab)
            }
            3 => {
                let a = simplex.vertices[0].w;
                let b = simplex.vertices[1].w;
                let c = simplex.vertices[2].w;
                let ab = b - a;
                let ac = c - a;
                let n = cross(ab, ac);
                if dot(n, a) < 0.0 { n } else { -n }
            }
            _ => {
                b3_assert(false);
                Vec3::default()
            }
        };

        if length_squared(search_direction) < 1000.0 * f32::MIN_POSITIVE {
            let mut local_point_a = Vec3::default();
            let mut local_point_b = Vec3::default();
            compute_witness_points(&simplex, &mut local_point_a, &mut local_point_b);
            distance_output.point_a = local_point_a;
            distance_output.point_b = local_point_b;
            distance_output.normal = Vec3::default();
            distance_output.distance = 0.0;
            distance_output.iterations = iteration;
            b3_validate(distance(local_point_a, local_point_b) < f32::EPSILON);
            return distance_output;
        }

        normal = -search_direction;

        let index_a = get_proxy_support(proxy_a, -search_direction);
        let support_a = points_a[index_a as usize];
        let search_direction2 = mul_mv(mt, search_direction);
        let index_b = get_proxy_support(proxy_b, search_direction2);
        let support_b = mul_mv(m, points_b[index_b as usize]) + xf.p;

        backup = simplex;

        let mut duplicate = false;
        for i in 0..simplex.count as usize {
            if simplex.vertices[i].index_a == index_a && simplex.vertices[i].index_b == index_b {
                duplicate = true;
                break;
            }
        }

        if duplicate {
            break;
        }

        let slot = simplex.count as usize;
        simplex.vertices[slot].index_a = index_a;
        simplex.vertices[slot].index_b = index_b;
        simplex.vertices[slot].wa = support_a;
        simplex.vertices[slot].wb = support_b;
        simplex.vertices[slot].w = support_b - support_a;
        simplex.count += 1;

        iteration += 1;
    }

    let mut local_point_a = Vec3::default();
    let mut local_point_b = Vec3::default();
    compute_witness_points(&simplex, &mut local_point_a, &mut local_point_b);

    distance_output.point_a = local_point_a;
    distance_output.point_b = local_point_b;
    distance_output.iterations = iteration;

    normal = normalize(normal);
    if !is_normalized(normal) {
        distance_output.distance = 0.0;
        distance_output.normal = Vec3::default();
        return distance_output;
    }

    distance_output.distance = distance(local_point_a, local_point_b);
    distance_output.normal = normal;

    if input.use_radii {
        let r_a = proxy_a.radius;
        let r_b = proxy_b.radius;
        distance_output.distance = max_float(0.0, distance_output.distance - r_a - r_b);
        distance_output.point_a = distance_output.point_a + r_a * normal;
        distance_output.point_b = distance_output.point_b - r_b * normal;
    }

    write_cache(cache, &simplex);
    distance_output
}

// MARK: - Shape cast

/// Perform a linear shape cast of shape B moving and shape A fixed.
pub fn shape_cast(input: &ShapeCastPairInput) -> CastOutput {
    let linear_slop = b3::linear_slop();
    let total_radius = input.proxy_a.radius + input.proxy_b.radius;
    let mut target = max_float(linear_slop, total_radius - linear_slop);
    let tolerance = 0.25 * linear_slop;

    b3_assert(target > tolerance);

    let mut cache = SimplexCache::default();
    let mut alpha: f32 = 0.0;

    let mut distance_input = DistanceInput {
        proxy_a: input.proxy_a,
        proxy_b: input.proxy_b,
        transform: input.transform,
        use_radii: false,
    };

    let delta2 = input.translation_b;
    let mut distance_output = DistanceOutput::default();
    let mut output = CastOutput {
        triangle_index: b3::NULL_INDEX,
        ..Default::default()
    };

    let mut iteration = 0;
    let max_iterations = 20;

    while iteration < max_iterations {
        output.iterations += 1;
        distance_output = shape_distance(&distance_input, &mut cache, None);

        if distance_output.distance < target + tolerance {
            if iteration == 0 {
                if input.can_encroach && distance_output.distance > 2.0 * linear_slop {
                    target = distance_output.distance - linear_slop;
                } else if input.can_encroach && distance_output.distance > 0.0 {
                    let retained = 0.5 * linear_slop;
                    let needed = distance_output.distance - retained;
                    let approach = -dot(delta2, distance_output.normal);
                    if approach <= max_float(needed, 0.0) {
                        return output;
                    }
                    let fraction = if needed > 0.0 { needed / approach } else { 0.0 };
                    if fraction >= input.max_fraction {
                        return output;
                    }
                    output.fraction = fraction;
                    output.point = mul_add(distance_output.point_a, input.proxy_a.radius, distance_output.normal);
                    output.normal = distance_output.normal;
                    output.hit = true;
                    return output;
                } else {
                    output.hit = true;
                    let c1 = mul_add(distance_output.point_a, input.proxy_a.radius, distance_output.normal);
                    let c2 = mul_add(distance_output.point_b, -input.proxy_b.radius, distance_output.normal);
                    output.point = lerp(c1, c2, 0.5);
                    return output;
                }
            } else {
                if distance_output.distance > 0.0 && !is_normalized(distance_output.normal) {
                    let pa = input.proxy_a.points();
                    for (i, p) in pa.iter().enumerate() {
                        b3_log(&format!("pointA[{i}] = {{{}, {}, {}}}", p.x, p.y, p.z));
                    }
                    b3_log(&format!("radiusA = {}", input.proxy_a.radius));
                    let pb = input.proxy_b.points();
                    for (i, p) in pb.iter().enumerate() {
                        b3_log(&format!("pointB[{i}] = {{{}, {}, {}}}", p.x, p.y, p.z));
                    }
                    b3_log(&format!("radiusB = {}", input.proxy_b.radius));
                    return output;
                }

                output.fraction = alpha;
                output.point = mul_add(distance_output.point_a, input.proxy_a.radius, distance_output.normal);
                output.normal = distance_output.normal;
                output.hit = true;
                return output;
            }
        }

        b3_assert(distance_output.distance > 0.0);
        b3_assert(is_normalized(distance_output.normal));

        let denominator = dot(delta2, distance_output.normal);
        if denominator >= 0.0 {
            return output;
        }

        alpha += (target - distance_output.distance) / denominator;
        if alpha >= input.max_fraction {
            return output;
        }

        distance_input.transform.p = mul_add(input.transform.p, alpha, delta2);
        iteration += 1;
    }

    output
}

/// Evaluate the transform sweep at a specific time.
pub fn get_sweep_transform(sweep: &Sweep, time: f32) -> Transform {
    let q = nlerp(sweep.q1, sweep.q2, time);
    Transform {
        p: lerp(sweep.c1, sweep.c2, time) - rotate_vector(q, sweep.local_center),
        q,
    }
}

/// Evaluate the transform sweep at its final time.
pub fn get_final_sweep_transform(sweep: &Sweep) -> Transform {
    let q = sweep.q2;
    Transform {
        p: sweep.c2 - rotate_vector(q, sweep.local_center),
        q,
    }
}
