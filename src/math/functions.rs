//! Scalar, vector, quaternion, transform, matrix, and AABB helpers.
//!
//! Scalar helpers preserve operand order. Ties return the other operand, which preserves signed
//! zeros and NaN propagation.

use core::ops::{
    Add, AddAssign, Div, Mul, MulAssign, Neg, Sub, SubAssign,
};

use crate::base::constants as b3;
use crate::math::types::*;

// MARK: - Scalar

#[inline]
pub fn min_int(a: i32, b: i32) -> i32 {
    if a < b { a } else { b }
}

#[inline]
pub fn max_int(a: i32, b: i32) -> i32 {
    if a > b { a } else { b }
}

#[inline]
pub fn clamp_int(a: i32, lower: i32, upper: i32) -> i32 {
    if a < lower {
        lower
    } else if upper < a {
        upper
    } else {
        a
    }
}

#[inline]
pub fn abs_float(a: f32) -> f32 {
    if a < 0.0 { -a } else { a }
}

#[inline]
pub fn min_float(a: f32, b: f32) -> f32 {
    if a < b { a } else { b }
}

#[inline]
pub fn max_float(a: f32, b: f32) -> f32 {
    if a > b { a } else { b }
}

#[inline]
pub fn clamp_float(a: f32, lower: f32, upper: f32) -> f32 {
    if a < lower {
        lower
    } else if upper < a {
        upper
    } else {
        a
    }
}

#[inline]
pub fn lerp_float(a: f32, b: f32, alpha: f32) -> f32 {
    (1.0 - alpha) * a + alpha * b
}

/// Compute an approximate arctangent in the range `[-pi, pi]`.
pub fn atan2_approx(y: f32, x: f32) -> f32 {
    if x == 0.0 && y == 0.0 {
        return 0.0;
    }

    let ax = abs_float(x);
    let ay = abs_float(y);
    let mx = max_float(ay, ax);
    let mn = min_float(ay, ax);
    let a = mn / mx;

    let s = a * a;
    let c = s * a;
    let q = s * s;
    let mut r: f32 = 0.024840285 * q + 0.18681418;
    let t: f32 = -0.094097948 * q - 0.33213072;
    r = r * s + t;
    r = r * c + a;

    if ay > ax {
        r = 1.57079637 - r;
    }
    if x < 0.0 {
        r = 3.14159274 - r;
    }
    if y < 0.0 {
        r = -r;
    }

    r
}

/// Compute the cosine and sine of an angle in radians. Deterministic across platforms.
pub fn compute_cos_sin(radians: f32) -> CosSin {
    let x = unwind_angle(radians);
    let pi2 = b3::PI * b3::PI;

    let c: f32;
    if x < -0.5 * b3::PI {
        let y = x + b3::PI;
        let y2 = y * y;
        c = -(pi2 - 4.0 * y2) / (pi2 + y2);
    } else if x > 0.5 * b3::PI {
        let y = x - b3::PI;
        let y2 = y * y;
        c = -(pi2 - 4.0 * y2) / (pi2 + y2);
    } else {
        let y2 = x * x;
        c = (pi2 - 4.0 * y2) / (pi2 + y2);
    }

    let s: f32;
    if x < 0.0 {
        let y = x + b3::PI;
        s = -16.0 * y * (b3::PI - y) / (5.0 * pi2 - 4.0 * y * (b3::PI - y));
    } else {
        s = 16.0 * x * (b3::PI - x) / (5.0 * pi2 - 4.0 * x * (b3::PI - x));
    }

    let mag = (s * s + c * c).sqrt();
    let inv_mag: f32 = if mag > 0.0 { 1.0 / mag } else { 0.0 };
    CosSin::new(c * inv_mag, s * inv_mag)
}

/// Convert any angle into the range `[-pi, pi]` using the IEEE remainder.
#[inline]
pub fn unwind_angle(radians: f32) -> f32 {
    let period = 2.0 * b3::PI;
    radians - (radians / period).round_ties_even() * period
}

// MARK: - Vector operators

impl Add for Vec3 {
    type Output = Vec3;
    #[inline]
    fn add(self, b: Vec3) -> Vec3 {
        Vec3::new(self.x + b.x, self.y + b.y, self.z + b.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;
    #[inline]
    fn sub(self, b: Vec3) -> Vec3 {
        Vec3::new(self.x - b.x, self.y - b.y, self.z - b.z)
    }
}

/// Component-wise multiplication.
impl Mul for Vec3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, b: Vec3) -> Vec3 {
        Vec3::new(self.x * b.x, self.y * b.y, self.z * b.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(s * self.x, s * self.y, s * self.z)
    }
}

impl Mul<Vec3> for f32 {
    type Output = Vec3;
    #[inline]
    fn mul(self, a: Vec3) -> Vec3 {
        Vec3::new(self * a.x, self * a.y, self * a.z)
    }
}

impl Div<f32> for Vec3 {
    type Output = Vec3;
    #[inline]
    fn div(self, s: f32) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;
    #[inline]
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl AddAssign for Vec3 {
    #[inline]
    fn add_assign(&mut self, b: Vec3) {
        *self = *self + b;
    }
}

impl SubAssign for Vec3 {
    #[inline]
    fn sub_assign(&mut self, b: Vec3) {
        *self = *self - b;
    }
}

impl MulAssign<f32> for Vec3 {
    #[inline]
    fn mul_assign(&mut self, s: f32) {
        *self = s * *self;
    }
}

/// Vector dot product.
#[inline]
pub fn dot(a: Vec3, b: Vec3) -> f32 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

/// Vector length.
#[inline]
pub fn length(v: Vec3) -> f32 {
    dot(v, v).sqrt()
}

/// Vector length squared.
#[inline]
pub fn length_squared(a: Vec3) -> f32 {
    a.x * a.x + a.y * a.y + a.z * a.z
}

/// Distance between two points.
#[inline]
pub fn distance(a: Vec3, b: Vec3) -> f32 {
    length(Vec3::new(b.x - a.x, b.y - a.y, b.z - a.z))
}

/// Squared distance between two points.
#[inline]
pub fn distance_squared(a: Vec3, b: Vec3) -> f32 {
    let dv = Vec3::new(b.x - a.x, b.y - a.y, b.z - a.z);
    dv.x * dv.x + dv.y * dv.y + dv.z * dv.z
}

/// Normalize a vector. Returns a zero vector if the input vector is very small.
#[inline(always)]
pub fn normalize(a: Vec3) -> Vec3 {
    let ls = a.x * a.x + a.y * a.y + a.z * a.z;
    if ls > 1000.0 * f32::MIN_POSITIVE {
        let s = 1.0 / ls.sqrt();
        Vec3::new(s * a.x, s * a.y, s * a.z)
    } else {
        Vec3::ZERO
    }
}

/// Normalize a vector and return the length. Returns a zero vector if the input is very small.
#[inline]
pub fn get_length_and_normalize(a: Vec3) -> (f32, Vec3) {
    let ls = a.x * a.x + a.y * a.y + a.z * a.z;
    if ls > 1000.0 * f32::MIN_POSITIVE {
        let l = ls.sqrt();
        let s = 1.0 / l;
        (l, Vec3::new(s * a.x, s * a.y, s * a.z))
    } else {
        (0.0, Vec3::ZERO)
    }
}

/// Get a unit vector that is perpendicular to the supplied vector.
#[inline]
pub fn perp(a: Vec3) -> Vec3 {
    let p = if a.x < -0.5 || 0.5 < a.x {
        Vec3::new(a.y, -a.x, 0.0)
    } else {
        Vec3::new(0.0, a.z, -a.y)
    };
    normalize(p)
}

/// Is a vector normalized?
#[inline]
pub fn is_normalized(a: Vec3) -> bool {
    abs_float(1.0 - dot(a, a)) < 100.0 * f32::EPSILON
}

/// `a + s * b`
#[inline]
pub fn mul_add(a: Vec3, s: f32, b: Vec3) -> Vec3 {
    Vec3::new(a.x + s * b.x, a.y + s * b.y, a.z + s * b.z)
}

/// `a - s * b`
#[inline]
pub fn mul_sub(a: Vec3, s: f32, b: Vec3) -> Vec3 {
    Vec3::new(a.x - s * b.x, a.y - s * b.y, a.z - s * b.z)
}

#[inline]
pub fn cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

#[inline]
pub fn lerp(a: Vec3, b: Vec3, alpha: f32) -> Vec3 {
    crate::base::core::b3_assert(0.0 <= alpha && alpha <= 1.0);
    Vec3::new(
        (1.0 - alpha) * a.x + alpha * b.x,
        (1.0 - alpha) * a.y + alpha * b.y,
        (1.0 - alpha) * a.z + alpha * b.z,
    )
}

/// Blend two vectors: `s * a + t * b`.
#[inline]
pub fn blend2(s: f32, a: Vec3, t: f32, b: Vec3) -> Vec3 {
    Vec3::new(s * a.x + t * b.x, s * a.y + t * b.y, s * a.z + t * b.z)
}

#[inline]
pub fn abs_vec3(a: Vec3) -> Vec3 {
    Vec3::new(abs_float(a.x), abs_float(a.y), abs_float(a.z))
}

/// Component-wise -1 or 1 (1 if zero).
#[inline]
pub fn sign_vec3(a: Vec3) -> Vec3 {
    Vec3::new(
        if a.x >= 0.0 { 1.0 } else { -1.0 },
        if a.y >= 0.0 { 1.0 } else { -1.0 },
        if a.z >= 0.0 { 1.0 } else { -1.0 },
    )
}

#[inline]
pub fn min_vec3(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        min_float(a.x, b.x),
        min_float(a.y, b.y),
        min_float(a.z, b.z),
    )
}

#[inline]
pub fn max_vec3(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(
        max_float(a.x, b.x),
        max_float(a.y, b.y),
        max_float(a.z, b.z),
    )
}

#[inline]
pub fn clamp_vec3(a: Vec3, lower: Vec3, upper: Vec3) -> Vec3 {
    Vec3::new(
        clamp_float(a.x, lower.x, upper.x),
        clamp_float(a.y, lower.y, upper.y),
        clamp_float(a.z, lower.z, upper.z),
    )
}

/// Create a safe scaling value for scaling collision. Allows negative scale, but keeps scale
/// sufficiently far from zero.
#[inline]
pub fn safe_scale(a: Vec3) -> Vec3 {
    let abs_scale = abs_vec3(a);
    let min_scale = Vec3::new(b3::MIN_SCALE, b3::MIN_SCALE, b3::MIN_SCALE);
    sign_vec3(a) * max_vec3(abs_scale, min_scale)
}

// MARK: - Quaternion

#[inline]
pub fn is_normalized_quat(q: Quat) -> bool {
    let qq = q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z + q.s * q.s;
    1.0 - 20.0 * f32::EPSILON < qq && qq < 1.0 + 20.0 * f32::EPSILON
}

#[inline(always)]
pub fn rotate_vector(q: Quat, v: Vec3) -> Vec3 {
    let t1 = cross(q.v, v);
    let t2 = mul_add(t1, q.s, v);
    let t3 = cross(q.v, t2);
    mul_add(v, 2.0, t3)
}

#[inline(always)]
pub fn inv_rotate_vector(q: Quat, v: Vec3) -> Vec3 {
    let t1 = cross(q.v, v);
    let t2 = mul_sub(t1, q.s, v);
    let t3 = cross(q.v, t2);
    mul_add(v, 2.0, t3)
}

#[inline]
pub fn dot_quat(a: Quat, b: Quat) -> f32 {
    a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s
}

#[inline(always)]
pub fn mul_quat(q1: Quat, q2: Quat) -> Quat {
    let t1 = cross(q1.v, q2.v);
    let t2 = mul_add(t1, q1.s, q2.v);
    let t3 = mul_add(t2, q2.s, q1.v);
    Quat::new(t3, q1.s * q2.s - dot(q1.v, q2.v))
}

/// Compute a relative quaternion: `inv(q1) * q2`.
#[inline(always)]
pub fn inv_mul_quat(q1: Quat, q2: Quat) -> Quat {
    let t1 = cross(q2.v, q1.v);
    let t2 = mul_add(t1, q1.s, q2.v);
    let t3 = mul_sub(t2, q2.s, q1.v);
    Quat::new(t3, q1.s * q2.s + dot(q1.v, q2.v))
}

#[inline]
pub fn conjugate(q: Quat) -> Quat {
    Quat::new(Vec3::new(-q.v.x, -q.v.y, -q.v.z), q.s)
}

#[inline]
pub fn negate_quat(q: Quat) -> Quat {
    Quat::new(Vec3::new(-q.v.x, -q.v.y, -q.v.z), -q.s)
}

#[inline]
pub fn normalize_quat(q: Quat) -> Quat {
    let ls = dot_quat(q, q);
    if ls > 1000.0 * f32::MIN_POSITIVE {
        let s = 1.0 / ls.sqrt();
        Quat::new(Vec3::new(s * q.v.x, s * q.v.y, s * q.v.z), s * q.s)
    } else {
        Quat::IDENTITY
    }
}

#[inline]
pub fn make_quat_from_axis_angle(axis: Vec3, radians: f32) -> Quat {
    crate::base::core::b3_assert(is_normalized(axis));
    let cs = compute_cos_sin(0.5 * radians);
    Quat::new(
        Vec3::new(cs.sine * axis.x, cs.sine * axis.y, cs.sine * axis.z),
        cs.cosine,
    )
}

/// Get the axis and angle from a quaternion. Assumes the quaternion is normalized.
#[inline]
pub fn get_axis_angle(q: Quat) -> (f32, Vec3) {
    let l = (q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z).sqrt();
    let radians = 2.0 * atan2_approx(l, q.s);
    if l > 0.0 {
        let inv = 1.0 / l;
        (radians, Vec3::new(inv * q.v.x, inv * q.v.y, inv * q.v.z))
    } else {
        (radians, Vec3::ZERO)
    }
}

#[inline]
pub fn get_quat_angle(q: Quat) -> f32 {
    let l = (q.v.x * q.v.x + q.v.y * q.v.y + q.v.z * q.v.z).sqrt();
    2.0 * atan2_approx(l, q.s)
}

#[inline]
pub fn get_twist_angle(q: Quat) -> f32 {
    let mut twist = if q.s < 0.0 {
        atan2_approx(-q.v.z, -q.s)
    } else {
        atan2_approx(q.v.z, q.s)
    };
    twist *= 2.0;
    crate::base::core::b3_assert(-b3::PI <= twist && twist <= b3::PI);
    twist
}

#[inline(always)]
pub fn get_swing_angle(q: Quat) -> f32 {
    let x = (q.v.z * q.v.z + q.s * q.s).sqrt();
    let y = (q.v.x * q.v.x + q.v.y * q.v.y).sqrt();
    let swing = 2.0 * atan2_approx(y, x);
    crate::base::core::b3_assert(0.0 <= swing && swing <= b3::PI);
    swing
}

#[inline]
pub fn nlerp(q1: Quat, q2: Quat, alpha: f32) -> Quat {
    let mut q1 = q1;
    if dot_quat(q1, q2) < 0.0 {
        q1 = Quat::new(Vec3::new(-q1.v.x, -q1.v.y, -q1.v.z), -q1.s);
    }

    let v = lerp(q1.v, q2.v, alpha);
    let s = (1.0 - alpha) * q1.s + alpha * q2.s;
    normalize_quat(Quat::new(v, s))
}

// MARK: - Transform

#[inline]
pub fn mul_transforms(a: Transform, b: Transform) -> Transform {
    Transform::new(
        rotate_vector(a.q, b.p) + a.p,
        mul_quat(a.q, b.q),
    )
}

#[inline(always)]
pub fn inv_mul_transforms(a: Transform, b: Transform) -> Transform {
    Transform::new(
        inv_rotate_vector(a.q, b.p - a.p),
        inv_mul_quat(a.q, b.q),
    )
}

#[inline]
pub fn invert_transform(t: Transform) -> Transform {
    Transform::new(inv_rotate_vector(t.q, -t.p), conjugate(t.q))
}

#[inline(always)]
pub fn transform_point(t: Transform, v: Vec3) -> Vec3 {
    rotate_vector(t.q, v) + t.p
}

#[inline]
pub fn inv_transform_point(t: Transform, v: Vec3) -> Vec3 {
    inv_rotate_vector(t.q, v - t.p)
}

// MARK: - World position boundary

#[inline]
pub fn to_pos(v: Vec3) -> Pos {
    Pos::from_vec3(v)
}

/// Lossy absolute conversion.
#[inline]
pub fn to_vec3(p: Pos) -> Vec3 {
    Vec3::new(p.x as f32, p.y as f32, p.z as f32)
}

pub trait RoundDownFloat {
    fn round_down_float(self) -> f32;
}

impl RoundDownFloat for f32 {
    #[inline]
    fn round_down_float(self) -> f32 {
        self
    }
}

impl RoundDownFloat for f64 {
    #[inline]
    fn round_down_float(self) -> f32 {
        let f = self as f32;
        if f as f64 > self { f.next_down() } else { f }
    }
}

pub trait RoundUpFloat {
    fn round_up_float(self) -> f32;
}

impl RoundUpFloat for f32 {
    #[inline]
    fn round_up_float(self) -> f32 {
        self
    }
}

impl RoundUpFloat for f64 {
    #[inline]
    fn round_up_float(self) -> f32 {
        let f = self as f32;
        if (f as f64) < self { f.next_up() } else { f }
    }
}

#[inline]
pub fn round_down_float<T: RoundDownFloat>(x: T) -> f32 {
    x.round_down_float()
}

#[inline]
pub fn round_up_float<T: RoundUpFloat>(x: T) -> f32 {
    x.round_up_float()
}

/// Subtract in double precision before narrowing the relative displacement.
#[inline]
pub fn sub_pos(a: Pos, b: Pos) -> Vec3 {
    Vec3::new(
        (a.x - b.x) as f32,
        (a.y - b.y) as f32,
        (a.z - b.z) as f32,
    )
}

#[inline]
pub fn offset_pos(p: Pos, d: Vec3) -> Pos {
    Pos::new(
        p.x + d.x as f64,
        p.y + d.y as f64,
        p.z + d.z as f64,
    )
}

#[inline]
pub fn lerp_position(a: Pos, b: Pos, t: f32) -> Pos {
    let u = t as f64;
    Pos::new(
        a.x + u * (b.x - a.x),
        a.y + u * (b.y - a.y),
        a.z + u * (b.z - a.z),
    )
}

#[inline]
pub fn transform_world_point(t: WorldTransform, p: Vec3) -> Pos {
    offset_pos(t.p, rotate_vector(t.q, p))
}

#[inline]
pub fn inv_transform_world_point(t: WorldTransform, p: Pos) -> Vec3 {
    inv_rotate_vector(t.q, sub_pos(p, t.p))
}

#[inline]
pub fn mul_world_transforms(a: WorldTransform, b: Transform) -> WorldTransform {
    WorldTransform::new(transform_world_point(a, b.p), mul_quat(a.q, b.q))
}

#[inline]
pub fn inv_mul_world_transforms(a: WorldTransform, b: WorldTransform) -> Transform {
    Transform::new(inv_transform_world_point(a, b.p), inv_mul_quat(a.q, b.q))
}

#[inline]
pub fn to_relative_transform(t: WorldTransform, base: Pos) -> Transform {
    Transform::new(sub_pos(t.p, base), t.q)
}

#[inline]
pub fn make_world_transform(t: Transform) -> WorldTransform {
    WorldTransform::new(Pos::from_vec3(t.p), t.q)
}

/// Conservative world bounds: inflate locally before rounding outwards here.
#[inline]
pub fn offset_aabb(local_box: AABB, origin: Pos) -> AABB {
    AABB::new(
        Vec3::new(
            round_down_float(origin.x + local_box.lower_bound.x as f64),
            round_down_float(origin.y + local_box.lower_bound.y as f64),
            round_down_float(origin.z + local_box.lower_bound.z as f64),
        ),
        Vec3::new(
            round_up_float(origin.x + local_box.upper_bound.x as f64),
            round_up_float(origin.y + local_box.upper_bound.y as f64),
            round_up_float(origin.z + local_box.upper_bound.z as f64),
        ),
    )
}

impl Add<Vec3> for Pos {
    type Output = Pos;
    #[inline]
    fn add(self, d: Vec3) -> Pos {
        offset_pos(self, d)
    }
}

impl Sub<Vec3> for Pos {
    type Output = Pos;
    #[inline]
    fn sub(self, d: Vec3) -> Pos {
        offset_pos(self, -d)
    }
}

impl Sub for Pos {
    type Output = Vec3;
    #[inline]
    fn sub(self, b: Pos) -> Vec3 {
        sub_pos(self, b)
    }
}

// MARK: - Matrix

#[inline]
pub fn det(m: Matrix3) -> f32 {
    dot(m.cx, cross(m.cy, m.cz))
}

#[inline]
pub fn mul_mv(m: Matrix3, a: Vec3) -> Vec3 {
    Vec3::new(
        m.cx.x * a.x + m.cy.x * a.y + m.cz.x * a.z,
        m.cx.y * a.x + m.cy.y * a.y + m.cz.y * a.z,
        m.cx.z * a.x + m.cy.z * a.y + m.cz.z * a.z,
    )
}

#[inline]
pub fn negate_mat3(a: Matrix3) -> Matrix3 {
    Matrix3::new(-a.cx, -a.cy, -a.cz)
}

#[inline]
pub fn add_mm(a: Matrix3, b: Matrix3) -> Matrix3 {
    Matrix3::new(a.cx + b.cx, a.cy + b.cy, a.cz + b.cz)
}

#[inline]
pub fn sub_mm(a: Matrix3, b: Matrix3) -> Matrix3 {
    Matrix3::new(a.cx - b.cx, a.cy - b.cy, a.cz - b.cz)
}

#[inline]
pub fn mul_sm(s: f32, a: Matrix3) -> Matrix3 {
    Matrix3::new(s * a.cx, s * a.cy, s * a.cz)
}

#[inline(always)]
pub fn mul_mm(a: Matrix3, b: Matrix3) -> Matrix3 {
    Matrix3::new(mul_mv(a, b.cx), mul_mv(a, b.cy), mul_mv(a, b.cz))
}

#[inline]
pub fn transpose(m: Matrix3) -> Matrix3 {
    Matrix3::new(
        Vec3::new(m.cx.x, m.cy.x, m.cz.x),
        Vec3::new(m.cx.y, m.cy.y, m.cz.y),
        Vec3::new(m.cx.z, m.cy.z, m.cz.z),
    )
}

#[inline]
pub fn invert_matrix(m: Matrix3) -> Matrix3 {
    let d = det(m);
    if abs_float(d) > 1000.0 * f32::MIN_POSITIVE {
        let inv_d = 1.0 / d;
        let out = Matrix3::new(
            inv_d * cross(m.cy, m.cz),
            inv_d * cross(m.cz, m.cx),
            inv_d * cross(m.cx, m.cy),
        );
        transpose(out)
    } else {
        Matrix3::ZERO
    }
}

#[inline(always)]
pub fn solve3(m: Matrix3, a: Vec3) -> Vec3 {
    let d = det(m);
    if abs_float(d) > 1000.0 * f32::MIN_POSITIVE {
        let inv_d = 1.0 / d;
        let sx = cross(m.cy, m.cz);
        let sy = cross(m.cz, m.cx);
        let sz = cross(m.cx, m.cy);
        Vec3::new(
            inv_d * dot(sx, a),
            inv_d * dot(sy, a),
            inv_d * dot(sz, a),
        )
    } else {
        Vec3::ZERO
    }
}

#[inline]
pub fn invert_t(m: Matrix3) -> Matrix3 {
    let d = det(m);
    if abs_float(d) > 1000.0 * f32::MIN_POSITIVE {
        let inv_d = 1.0 / d;
        Matrix3::new(
            inv_d * cross(m.cy, m.cz),
            inv_d * cross(m.cz, m.cx),
            inv_d * cross(m.cx, m.cy),
        )
    } else {
        Matrix3::ZERO
    }
}

#[inline]
pub fn abs_matrix3(m: Matrix3) -> Matrix3 {
    Matrix3::new(abs_vec3(m.cx), abs_vec3(m.cy), abs_vec3(m.cz))
}

#[inline(always)]
pub fn make_matrix_from_quat(q: Quat) -> Matrix3 {
    let xx = q.v.x * q.v.x;
    let yy = q.v.y * q.v.y;
    let zz = q.v.z * q.v.z;
    let xy = q.v.x * q.v.y;
    let xz = q.v.x * q.v.z;
    let xw = q.v.x * q.s;
    let yz = q.v.y * q.v.z;
    let yw = q.v.y * q.s;
    let zw = q.v.z * q.s;

    Matrix3::new(
        Vec3::new(1.0 - 2.0 * (yy + zz), 2.0 * (xy + zw), 2.0 * (xz - yw)),
        Vec3::new(2.0 * (xy - zw), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + xw)),
        Vec3::new(2.0 * (xz + yw), 2.0 * (yz - xw), 1.0 - 2.0 * (xx + yy)),
    )
}

// MARK: - AABB

/// Get the AABB of a point cloud.
#[inline]
pub fn make_aabb(points: &[Vec3], radius: f32) -> AABB {
    crate::base::core::b3_assert(!points.is_empty());
    let mut a = AABB::new(points[0], points[0]);
    for p in &points[1..] {
        a.lower_bound = min_vec3(a.lower_bound, *p);
        a.upper_bound = max_vec3(a.upper_bound, *p);
    }
    let r = Vec3::new(radius, radius, radius);
    a.lower_bound = a.lower_bound - r;
    a.upper_bound = a.upper_bound + r;
    a
}

/// Does `a` fully contain `b`?
#[inline]
pub fn aabb_contains(a: AABB, b: AABB) -> bool {
    if a.lower_bound.x > b.lower_bound.x || b.upper_bound.x > a.upper_bound.x {
        return false;
    }
    if a.lower_bound.y > b.lower_bound.y || b.upper_bound.y > a.upper_bound.y {
        return false;
    }
    if a.lower_bound.z > b.lower_bound.z || b.upper_bound.z > a.upper_bound.z {
        return false;
    }
    true
}

#[inline]
pub fn aabb_area(a: AABB) -> f32 {
    let delta = a.upper_bound - a.lower_bound;
    2.0 * (delta.x * delta.y + delta.y * delta.z + delta.z * delta.x)
}

#[inline]
pub fn aabb_center(a: AABB) -> Vec3 {
    0.5 * (a.upper_bound + a.lower_bound)
}

#[inline]
pub fn aabb_extents(a: AABB) -> Vec3 {
    0.5 * (a.upper_bound - a.lower_bound)
}

#[inline]
pub fn aabb_union(a: AABB, b: AABB) -> AABB {
    AABB::new(
        min_vec3(a.lower_bound, b.lower_bound),
        max_vec3(a.upper_bound, b.upper_bound),
    )
}

/// Add uniform padding to an axis-aligned bounding box.
#[inline]
pub fn aabb_inflate(a: AABB, extension: f32) -> AABB {
    let radius = Vec3::new(extension, extension, extension);
    AABB::new(a.lower_bound - radius, a.upper_bound + radius)
}

/// Do two axis-aligned boxes overlap?
#[inline]
pub fn aabb_overlaps(a: AABB, b: AABB) -> bool {
    if a.upper_bound.x < b.lower_bound.x || a.lower_bound.x > b.upper_bound.x {
        return false;
    }
    if a.upper_bound.y < b.lower_bound.y || a.lower_bound.y > b.upper_bound.y {
        return false;
    }
    if a.upper_bound.z < b.lower_bound.z || a.lower_bound.z > b.upper_bound.z {
        return false;
    }
    true
}

#[inline(always)]
pub fn aabb_transform(transform: Transform, a: AABB) -> AABB {
    let center = transform_point(transform, aabb_center(a));
    let m = make_matrix_from_quat(transform.q);
    let extent = mul_mv(abs_matrix3(m), aabb_extents(a));
    AABB::new(center - extent, center + extent)
}

#[inline]
pub fn closest_point_to_aabb(point: Vec3, a: AABB) -> Vec3 {
    clamp_vec3(point, a.lower_bound, a.upper_bound)
}

/// Is this a valid number? Not NaN or infinity.
#[inline]
pub fn is_valid_float(a: f32) -> bool {
    a.is_finite()
}

#[inline]
pub fn is_valid_vec3(a: Vec3) -> bool {
    a.x.is_finite() && a.y.is_finite() && a.z.is_finite()
}

pub fn is_valid_quat(a: Quat) -> bool {
    if !(a.v.x.is_finite() && a.v.y.is_finite() && a.v.z.is_finite() && a.s.is_finite()) {
        return false;
    }
    is_normalized_quat(a)
}

pub fn is_valid_transform(a: Transform) -> bool {
    is_valid_vec3(a.p) && is_valid_quat(a.q)
}

pub fn is_valid_matrix3(a: Matrix3) -> bool {
    is_valid_vec3(a.cx) && is_valid_vec3(a.cy) && is_valid_vec3(a.cz)
}

pub fn is_valid_aabb(a: AABB) -> bool {
    if !is_valid_vec3(a.lower_bound) {
        return false;
    }
    if !is_valid_vec3(a.upper_bound) {
        return false;
    }
    if a.lower_bound.x > a.upper_bound.x {
        return false;
    }
    if a.lower_bound.y > a.upper_bound.y {
        return false;
    }
    if a.lower_bound.z > a.upper_bound.z {
        return false;
    }
    true
}

pub fn is_bounded_aabb(a: AABB) -> bool {
    let huge = b3::huge();
    if a.lower_bound.x < -huge || a.lower_bound.y < -huge || a.lower_bound.z < -huge {
        return false;
    }
    if a.upper_bound.x > huge || a.upper_bound.y > huge || a.upper_bound.z > huge {
        return false;
    }
    true
}

pub fn is_sane_aabb(a: AABB) -> bool {
    is_valid_aabb(a) && is_bounded_aabb(a)
}

pub fn is_valid_plane(a: Plane) -> bool {
    if !is_valid_vec3(a.normal) {
        return false;
    }
    if !is_normalized(a.normal) {
        return false;
    }
    is_valid_float(a.offset)
}

pub fn is_valid_position(p: Pos) -> bool {
    p.x.is_finite() && p.y.is_finite() && p.z.is_finite()
}

pub fn is_valid_world_transform(t: WorldTransform) -> bool {
    is_valid_position(t.p) && is_valid_quat(t.q)
}

pub fn make_quat_from_matrix(m: Matrix3) -> Quat {
    let c1 = m.cx;
    let c2 = m.cy;
    let c3 = m.cz;
    let mut q = Quat::default();

    let trace = m.cx.x + m.cy.y + m.cz.z;
    if trace >= 0.0 {
        q.v.x = c2.z - c3.y;
        q.v.y = c3.x - c1.z;
        q.v.z = c1.y - c2.x;
        q.s = trace + 1.0;
    } else if c1.x > c2.y && c1.x > c3.z {
        q.v.x = c1.x - c2.y - c3.z + 1.0;
        q.v.y = c2.x + c1.y;
        q.v.z = c3.x + c1.z;
        q.s = c2.z - c3.y;
    } else if c2.y > c3.z {
        q.v.x = c1.y + c2.x;
        q.v.y = c2.y - c3.z - c1.x + 1.0;
        q.v.z = c3.y + c2.z;
        q.s = c3.x - c1.z;
    } else {
        q.v.x = c1.z + c3.x;
        q.v.y = c2.z + c3.y;
        q.v.z = c3.z - c1.x - c2.y + 1.0;
        q.s = c1.y - c2.x;
    }

    normalize_quat(q)
}

/// Find a quaternion that rotates one unit vector to another.
pub fn compute_quat_between_unit_vectors(v1: Vec3, v2: Vec3) -> Quat {
    crate::base::core::b3_assert(is_normalized(v1));
    crate::base::core::b3_assert(is_normalized(v2));

    let mut out = Quat::default();
    let m = lerp(v1, v2, 0.5);
    let tolerance = 100.0 * f32::EPSILON;
    if length_squared(m) > tolerance * tolerance {
        out.v = cross(v1, m);
        out.s = dot(v1, m);
    } else {
        if abs_float(v1.x) > 0.5 {
            out.v.x = v1.y;
            out.v.y = -v1.x;
            out.v.z = 0.0;
        } else {
            out.v.x = 0.0;
            out.v.y = v1.z;
            out.v.z = -v1.y;
        }
        out.s = 0.0;
    }

    normalize_quat(out)
}

/// Compute the closest points on two infinite lines.
pub fn line_distance(p1: Vec3, d1: Vec3, p2: Vec3, d2: Vec3) -> SegmentDistanceResult {
    let mut result = SegmentDistanceResult::default();

    let a11 = dot(d1, d1);
    let a12 = -dot(d1, d2);
    let a21 = dot(d2, d1);
    let a22 = -dot(d2, d2);

    let w = p1 - p2;
    let b1 = -dot(d1, w);
    let b2 = -dot(d2, w);

    let d = a11 * a22 - a12 * a21;
    if d * d < 1000.0 * f32::MIN_POSITIVE {
        let s1 = dot(p2 - p1, d1) / dot(d1, d1);
        let s2: f32 = 0.0;

        result.point1 = mul_add(p1, s1, d1);
        result.fraction1 = s1;
        result.point2 = p2;
        result.fraction2 = s2;
        return result;
    }

    let s1 = (a22 * b1 - a12 * b2) / d;
    let s2 = (a11 * b2 - a21 * b1) / d;

    result.point1 = mul_add(p1, s1, d1);
    result.fraction1 = s1;
    result.point2 = mul_add(p2, s2, d2);
    result.fraction2 = s2;
    result
}

/// Compute the closest points on two line segments.
pub fn segment_distance(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3) -> SegmentDistanceResult {
    let mut result = SegmentDistanceResult::default();

    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;

    let a = dot(d1, d1);
    let b = dot(d1, d2);
    let c = dot(d1, r);
    let e = dot(d2, d2);
    let f = dot(d2, r);

    if a < 100.0 * f32::EPSILON && e < 100.0 * f32::EPSILON {
        result.point1 = p1;
        result.fraction1 = 0.0;
        result.point2 = p2;
        result.fraction2 = 0.0;
        return result;
    }

    if a < 100.0 * f32::EPSILON {
        let s2 = clamp_float(f / e, 0.0, 1.0);
        result.point1 = p1;
        result.fraction1 = 0.0;
        result.point2 = mul_add(p2, s2, d2);
        result.fraction2 = s2;
        return result;
    }

    if e < 100.0 * f32::EPSILON {
        let s1 = clamp_float(-c / a, 0.0, 1.0);
        result.point1 = mul_add(p1, s1, d1);
        result.fraction1 = s1;
        result.point2 = p2;
        result.fraction2 = 0.0;
        return result;
    }

    let denom = a * e - b * b;
    let mut s1: f32 = if denom > 1000.0 * f32::MIN_POSITIVE {
        clamp_float((b * f - c * e) / denom, 0.0, 1.0)
    } else {
        0.0
    };
    let mut s2 = (b * s1 + f) / e;

    if s2 < 0.0 {
        s1 = clamp_float(-c / a, 0.0, 1.0);
        s2 = 0.0;
    } else if s2 > 1.0 {
        s1 = clamp_float((b - c) / a, 0.0, 1.0);
        s2 = 1.0;
    }

    result.point1 = mul_add(p1, s1, d1);
    result.fraction1 = s1;
    result.point2 = mul_add(p2, s2, d2);
    result.fraction2 = s2;
    result
}

/// Compute the closest point on the segment `a-b` to the target `q`.
pub fn point_to_segment_distance(a: Vec3, b: Vec3, q: Vec3) -> Vec3 {
    let ab = b - a;
    let aq = q - a;

    let mut alpha = dot(ab, aq);

    if alpha <= 0.0 {
        a
    } else {
        let denominator = dot(ab, ab);
        if alpha > denominator {
            b
        } else {
            alpha /= denominator;
            mul_add(a, alpha, ab)
        }
    }
}

/// Get the inertia tensor of an offset point.
pub fn steiner(mass: f32, origin: Vec3) -> Matrix3 {
    let ixx = mass * (origin.y * origin.y + origin.z * origin.z);
    let iyy = mass * (origin.x * origin.x + origin.z * origin.z);
    let izz = mass * (origin.x * origin.x + origin.y * origin.y);
    let ixy = -mass * origin.x * origin.y;
    let ixz = -mass * origin.x * origin.z;
    let iyz = -mass * origin.y * origin.z;

    let mut out = Matrix3::default();
    out.cx.x = ixx;
    out.cy.x = ixy;
    out.cz.x = ixz;
    out.cx.y = ixy;
    out.cy.y = iyy;
    out.cz.y = iyz;
    out.cx.z = ixz;
    out.cy.z = iyz;
    out.cz.z = izz;
    out
}

/// Is this a valid ray?
pub fn is_valid_ray(input: &crate::types::queries::RayCastInput) -> bool {
    is_valid_vec3(input.origin)
        && is_valid_vec3(input.translation)
        && is_valid_float(input.max_fraction)
        && 0.0 <= input.max_fraction
        && input.max_fraction < b3::huge()
}

#[inline]
pub fn pos_mul_add(a: Pos, s: f32, b: Vec3) -> Pos {
    offset_pos(a, s * b)
}

#[inline]
pub fn pos_mul_sub(a: Pos, s: f32, b: Vec3) -> Pos {
    offset_pos(a, -(s * b))
}

#[inline]
pub fn distance_pos(a: Pos, b: Pos) -> f32 {
    length(sub_pos(a, b))
}

#[inline]
pub fn abs_pos(a: Pos) -> Pos {
    Pos::new(abs_float(a.x as f32) as f64, abs_float(a.y as f32) as f64, abs_float(a.z as f32) as f64)
}
