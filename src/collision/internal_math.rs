//! Scalar geometry helpers shared by the narrow phase.

use crate::base::core::{b3_assert, b3_validate};
use crate::base::constants as b3;
use crate::math::functions::*;
use crate::math::types::*;
use crate::types::collision_types::TriangleFeature;

/// A 2x2 matrix stored as two column vectors.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Matrix2 {
    pub cx: Vec2,
    pub cy: Vec2,
}

impl Matrix2 {
    pub fn new(cx: Vec2, cy: Vec2) -> Self {
        Self { cx, cy }
    }
}

/// A triangle with vertex indices and edge flags.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Triangle {
    pub vertices: [Vec3; 3],
    pub i1: i32,
    pub i2: i32,
    pub i3: i32,
    pub flags: i32,
}

impl Triangle {
    pub fn new(i1: i32, i2: i32, i3: i32, a: Vec3, b: Vec3, c: Vec3) -> Self {
        Self {
            vertices: [a, b, c],
            i1,
            i2,
            i3,
            flags: 0,
        }
    }
}

/// A point on a triangle along with the feature it belongs to.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct TrianglePoint {
    pub point: Vec3,
    pub feature: TriangleFeature,
}

/// Minimum extent and the axis along which it occurs.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ShapeExtent {
    pub min_extent: f32,
    pub max_extent: Vec3,
}

/// The empty bounds sentinel.
#[inline]
pub fn bounds3_empty() -> AABB {
    AABB::new(
        Vec3::new(f32::MAX, f32::MAX, f32::MAX),
        Vec3::new(-f32::MAX, -f32::MAX, -f32::MAX),
    )
}

/// Align a byte count up to 8 bytes.
#[inline]
pub fn align_up8(x: usize) -> usize {
    (x + 7) & !7
}

/// Integer ceiling division. The denominator must be positive.
#[inline]
pub fn ceiling_int(numerator: i32, denominator: i32) -> i32 {
    b3_validate(denominator > 0);
    (numerator + denominator - 1) / denominator
}

/// Integer ceiling division where the denominator is a power of two.
#[inline]
pub fn ceiling_pow2(numerator: i32, denominator: i32, exponent: i32) -> i32 {
    b3_validate(exponent > 0 && denominator == 1 << exponent);
    (numerator + denominator - 1) >> exponent
}

// MARK: - Vec2 helpers

#[inline]
pub fn dot2(v1: Vec2, v2: Vec2) -> f32 {
    v1.x * v2.x + v1.y * v2.y
}

#[inline]
pub fn length2(v: Vec2) -> f32 {
    dot2(v, v).sqrt()
}

#[inline]
pub fn length_squared2(v: Vec2) -> f32 {
    dot2(v, v)
}

#[inline]
pub fn min_vec2(v1: Vec2, v2: Vec2) -> Vec2 {
    Vec2::new(min_float(v1.x, v2.x), min_float(v1.y, v2.y))
}

#[inline]
pub fn max_vec2(v1: Vec2, v2: Vec2) -> Vec2 {
    Vec2::new(max_float(v1.x, v2.x), max_float(v1.y, v2.y))
}

#[inline]
pub fn add2(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x + b.x, a.y + b.y)
}

#[inline]
pub fn sub2(a: Vec2, b: Vec2) -> Vec2 {
    Vec2::new(a.x - b.x, a.y - b.y)
}

#[inline]
pub fn neg2(v: Vec2) -> Vec2 {
    Vec2::new(-v.x, -v.y)
}

#[inline]
pub fn mul_sv2(s: f32, v: Vec2) -> Vec2 {
    Vec2::new(s * v.x, s * v.y)
}

/// a + s * b
#[inline]
pub fn mul_add2(a: Vec2, s: f32, b: Vec2) -> Vec2 {
    Vec2::new(a.x + s * b.x, a.y + s * b.y)
}

/// a - s * b
#[inline]
pub fn mul_sub2(a: Vec2, s: f32, b: Vec2) -> Vec2 {
    Vec2::new(a.x - s * b.x, a.y - s * b.y)
}

#[inline]
pub fn cross2(a: Vec2, b: Vec2) -> f32 {
    a.x * b.y - a.y * b.x
}

#[inline]
pub fn distance_squared2(a: Vec2, b: Vec2) -> f32 {
    let dx = b.x - a.x;
    let dy = b.y - a.y;
    dx * dx + dy * dy
}

#[inline]
pub fn mul_mv2(m: Matrix2, a: Vec2) -> Vec2 {
    Vec2::new(m.cx.x * a.x + m.cy.x * a.y, m.cx.y * a.x + m.cy.y * a.y)
}

#[inline]
pub fn mul_mm2(m1: Matrix2, m2: Matrix2) -> Matrix2 {
    Matrix2::new(mul_mv2(m1, m2.cx), mul_mv2(m1, m2.cy))
}

#[inline]
pub fn det2(m: Matrix2) -> f32 {
    m.cx.x * m.cy.y - m.cx.y * m.cy.x
}

pub fn invert2(m: Matrix2) -> Matrix2 {
    let det = det2(m);
    if abs_float(det) > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / det;
        Matrix2::new(
            Vec2::new(inv_det * m.cy.y, -inv_det * m.cx.y),
            Vec2::new(-inv_det * m.cy.x, inv_det * m.cx.x),
        )
    } else {
        Matrix2::new(Vec2::ZERO, Vec2::ZERO)
    }
}

/// Solve a positive semi-definite 2x2 system.
pub fn solve2(m: Matrix2, b: Vec2) -> Vec2 {
    let det = det2(m);
    if det > 1000.0 * f32::MIN_POSITIVE {
        let inv_det = 1.0 / det;
        Vec2::new(
            inv_det * m.cy.y * b.x - inv_det * m.cy.x * b.y,
            -inv_det * m.cx.y * b.x + inv_det * m.cx.x * b.y,
        )
    } else {
        Vec2::ZERO
    }
}

// MARK: - Vec3 helpers

/// Get a component by index.
#[inline]
pub fn get_by_index(v: Vec3, index: usize) -> f32 {
    b3_validate(index < 3);
    match index {
        0 => v.x,
        1 => v.y,
        _ => v.z,
    }
}

/// The index of the largest component.
#[inline]
pub fn major_axis(v: Vec3) -> i32 {
    if v.x < v.y {
        if v.y < v.z { 2 } else { 1 }
    } else if v.x < v.z {
        2
    } else {
        0
    }
}

#[inline]
pub fn min_element(v: Vec3) -> f32 {
    min_float(v.x, min_float(v.y, v.z))
}

#[inline]
pub fn max_element(v: Vec3) -> f32 {
    max_float(v.x, max_float(v.y, v.z))
}

#[inline]
pub fn max_element_index(v: Vec3) -> i32 {
    major_axis(v)
}

/// Convenience function: s * a + t * b + u * c
#[inline]
pub fn blend3(s: f32, a: Vec3, t: f32, b: Vec3, u: f32, c: Vec3) -> Vec3 {
    Vec3::new(
        s * a.x + t * b.x + u * c.x,
        s * a.y + t * b.y + u * c.y,
        s * a.z + t * b.z + u * c.z,
    )
}

/// Cross product that adds equal components instead of subtracting them.
#[inline]
pub fn modified_cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z + a.z * b.y,
        a.z * b.x + a.x * b.z,
        a.x * b.y + a.y * b.x,
    )
}

#[inline]
pub fn make_diagonal_matrix(a: f32, b: f32, c: f32) -> Matrix3 {
    Matrix3::new(Vec3::new(a, 0.0, 0.0), Vec3::new(0.0, b, 0.0), Vec3::new(0.0, 0.0, c))
}

/// Clamp the length of a vector while keeping its direction.
pub fn clamp_length(v: Vec3, max_length: f32) -> Vec3 {
    let length_sq = length_squared(v);
    if length_sq <= max_length * max_length {
        return v;
    }
    let len = length_sq.sqrt();
    (max_length / len) * v
}

/// Compute a unit vector perpendicular to the given unit vector.
pub fn arbitrary_perp(v: Vec3) -> Vec3 {
    let p = if v.x < -0.5 || v.x > 0.5 {
        let a: f32 = 0.67;
        let b: f32 = -0.42;
        Vec3::new(a * v.y + b * v.z, -a * v.x, -b * v.x)
    } else if v.y < -0.5 || v.y > 0.5 {
        let a: f32 = 0.67;
        let c: f32 = -0.42;
        Vec3::new(a * v.y, -a * v.x + c * v.z, -c * v.y)
    } else {
        let a: f32 = 0.67;
        let b: f32 = -0.42;
        Vec3::new(a * v.z, b * v.z, -a * v.x - b * v.y)
    };
    b3_validate(length_squared(p) > 0.1);
    normalize(p)
}

#[inline]
pub fn scalar_triple_product(a: Vec3, b: Vec3, c: Vec3) -> f32 {
    let d = Vec3::new(
        b.y * c.z - b.z * c.y,
        b.z * c.x - b.x * c.z,
        b.x * c.y - b.y * c.x,
    );
    a.x * d.x + a.y * d.y + a.z * d.z
}

// MARK: - Rotation helpers

/// Exponential map (Grassia).
pub fn quat_from_exponential_map(v: Vec3) -> Quat {
    let threshold: f32 = 0.018581361;
    let angle = length(v);
    if angle < threshold {
        let cs = compute_cos_sin(0.5 * angle);
        return Quat::new((0.5 + angle * angle / 48.0) * v, cs.cosine);
    }
    make_quat_from_axis_angle((1.0 / angle) * v, angle)
}

/// Integrate rotation from angular velocity.
pub fn integrate_rotation(q1: Quat, delta_rotation: Vec3) -> Quat {
    let qd = mul_quat(Quat::new(0.5 * delta_rotation, 0.0), q1);
    normalize_quat(Quat::new(q1.v + qd.v, qd.s + q1.s))
}

/// Pseudo angular velocity from a quaternion target.
pub fn delta_quat_to_rotation(q: Quat, target: Quat) -> Vec3 {
    let s = if dot_quat(q, target) < 0.0 { negate_quat(q) } else { q };
    let diff = Quat::new(target.v - s.v, target.s - s.s);
    let product = mul_quat(diff, conjugate(s));
    2.0 * product.v
}

// MARK: - Plane helpers

pub fn normalize_plane(plane: Plane) -> Plane {
    let inv_length = 1.0 / length(plane.normal);
    Plane::new(inv_length * plane.normal, inv_length * plane.offset)
}

pub fn make_plane_from_normal_and_point(normal: Vec3, point: Vec3) -> Plane {
    Plane::new(normal, dot(normal, point))
}

pub fn make_plane_from_points(point1: Vec3, point2: Vec3, point3: Vec3) -> Plane {
    let normal = normalize(cross(point2 - point1, point3 - point1));
    Plane::new(normal, dot(normal, point1))
}

pub fn make_normal_from_points(point1: Vec3, point2: Vec3, point3: Vec3) -> Vec3 {
    normalize(cross(point2 - point1, point3 - point1))
}

/// normal2 = q * normal1, offset2 = dot(normal2, p) + offset1
pub fn transform_plane(transform: Transform, plane: Plane) -> Plane {
    let normal = rotate_vector(transform.q, plane.normal);
    Plane::new(normal, plane.offset + dot(normal, transform.p))
}

#[inline]
pub fn plane_separation(plane: Plane, point: Vec3) -> f32 {
    dot(plane.normal, point) - plane.offset
}

/// Negative if p is below the triangle v1-v2-v3.
pub fn signed_volume(v1: Vec3, v2: Vec3, v3: Vec3, p: Vec3) -> f32 {
    let e1 = v2 - v1;
    let e2 = v3 - v1;
    let n = cross(e1, e2);
    dot(n, p - v1)
}

// MARK: - Inertia helpers

pub fn rotate_inertia(q: Quat, central_inertia: Matrix3) -> Matrix3 {
    let rotation_matrix = make_matrix_from_quat(q);
    mul_mm(rotation_matrix, mul_mm(central_inertia, transpose(rotation_matrix)))
}

pub fn transform_inertia(transform: Transform, central_inertia: Matrix3, mass: f32) -> Matrix3 {
    add_mm(rotate_inertia(transform.q, central_inertia), steiner(mass, transform.p))
}

// MARK: - AABB helpers

/// Get the surface area (perimeter) of an AABB.
pub fn perimeter(a: AABB) -> f32 {
    let wx = a.upper_bound.x - a.lower_bound.x;
    let wy = a.upper_bound.y - a.lower_bound.y;
    let wz = a.upper_bound.z - a.lower_bound.z;
    2.0 * (wx * wz + wy * wx + wz * wy)
}

/// Enlarge `a` to contain `b`. Returns true if the AABB grew.
pub fn enlarge_aabb(a: &mut AABB, b: AABB) -> bool {
    let mut changed = false;
    if b.lower_bound.x < a.lower_bound.x {
        a.lower_bound.x = b.lower_bound.x;
        changed = true;
    }
    if b.lower_bound.y < a.lower_bound.y {
        a.lower_bound.y = b.lower_bound.y;
        changed = true;
    }
    if b.lower_bound.z < a.lower_bound.z {
        a.lower_bound.z = b.lower_bound.z;
        changed = true;
    }
    if a.upper_bound.x < b.upper_bound.x {
        a.upper_bound.x = b.upper_bound.x;
        changed = true;
    }
    if a.upper_bound.y < b.upper_bound.y {
        a.upper_bound.y = b.upper_bound.y;
        changed = true;
    }
    if a.upper_bound.z < b.upper_bound.z {
        a.upper_bound.z = b.upper_bound.z;
        changed = true;
    }
    changed
}

/// Get the farthest point on the AABB in the direction of `p`.
pub fn farthest_point_on_aabb(b: AABB, p: Vec3) -> Vec3 {
    Vec3::new(
        if (p.x - b.lower_bound.x) > (b.upper_bound.x - p.x) { b.lower_bound.x } else { b.upper_bound.x },
        if (p.y - b.lower_bound.y) > (b.upper_bound.y - p.y) { b.lower_bound.y } else { b.upper_bound.y },
        if (p.z - b.lower_bound.z) > (b.upper_bound.z - p.z) { b.lower_bound.z } else { b.upper_bound.z },
    )
}

/// Add a point to an AABB.
#[inline]
pub fn aabb_add_point(a: AABB, point: Vec3) -> AABB {
    AABB::new(min_vec3(a.lower_bound, point), max_vec3(a.upper_bound, point))
}

/// Alias used by the original header.
#[inline]
pub fn bounds3_empty_alias() -> AABB {
    bounds3_empty()
}

/// Sanity for a fully built hull.
pub fn is_valid_hull_geometry(vertex_count: i32, edge_count: i32, face_count: i32) -> bool {
    b3_assert(vertex_count >= 4);
    b3_assert(edge_count >= 6);
    b3_assert(face_count >= 4);
    vertex_count <= b3::MAX_HULL_VERTICES
        && edge_count <= b3::MAX_HULL_EDGES
        && face_count <= b3::MAX_HULL_FACES
}
