//! Four-wide solver types. Each lane preserves the scalar arithmetic order.
//!
//! Swift lowered `Builtin.Vec4xFPIEEE32` arithmetic to a single NEON instruction per lane group.
//! Here the lane vector is an explicit fixed four-element array and every helper writes the four
//! per-lane IEEE operations in lane order, so no result depends on how the backend chooses to
//! vectorise (or refuse to vectorise) a loop.

use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

use crate::collision::internal_math::Matrix2;
use crate::math::types::{Vec2, Vec3};
use crate::simulation::contact_solver::SymMatrix3;

/// Flip the IEEE-754 sign bit. `Builtin.fneg` and scalar `Float` negation both do this, so
/// `+0`, `-0` and NaN payloads keep their sign; subtracting from zero would not.
#[inline(always)]
fn flip_sign(value: f32) -> f32 {
    f32::from_bits(value.to_bits() ^ 0x8000_0000)
}

/// Four floats, one per contact lane.
///
/// `storage` mirrors Swift's `Builtin.Vec4xFPIEEE32`: four IEEE `f32` lanes with 16-byte alignment.
/// The type is a plain value, so unlike the Swift original it needs no `@unchecked Sendable`;
/// lanes are copied, never aliased.
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FloatW {
    /// The four lanes in order: `x`, `y`, `z`, `w`.
    pub storage: [f32; 4],
}

impl FloatW {
    /// `init(_ x:_ y:_ z:_ w:)`
    #[inline(always)]
    pub const fn new(x: f32, y: f32, z: f32, w: f32) -> Self {
        Self { storage: [x, y, z, w] }
    }

    /// `init()`
    #[inline(always)]
    pub const fn zero() -> Self {
        Self { storage: [0.0; 4] }
    }

    /// `init(repeating:)`
    #[inline(always)]
    pub const fn splat(value: f32) -> Self {
        Self {
            storage: [value, value, value, value],
        }
    }

    /// `init(storage:)`
    #[inline(always)]
    pub const fn from_storage(storage: [f32; 4]) -> Self {
        Self { storage }
    }

    /// `init(_ a: SIMD4<Float>)` - the lane vector is already a four-element array here.
    #[inline(always)]
    pub const fn from_lanes(lanes: [f32; 4]) -> Self {
        Self {
            storage: Self::load(lanes),
        }
    }

    /// `load(_:)` - `Builtin.loadRaw` of a 16-byte lane vector is a bitwise copy of the lanes.
    #[inline(always)]
    const fn load(lanes: [f32; 4]) -> [f32; 4] {
        lanes
    }

    /// `simd` - the lane vector.
    #[inline(always)]
    pub const fn lanes(&self) -> [f32; 4] {
        self.storage
    }

    /// The lanes reinterpreted as raw IEEE bit patterns, for bitwise selection.
    #[inline(always)]
    pub const fn to_bits(&self) -> [u32; 4] {
        [
            self.storage[0].to_bits(),
            self.storage[1].to_bits(),
            self.storage[2].to_bits(),
            self.storage[3].to_bits(),
        ]
    }

    /// Lanes rebuilt from raw IEEE bit patterns.
    #[inline(always)]
    pub const fn from_bits(bits: [u32; 4]) -> Self {
        Self {
            storage: [
                f32::from_bits(bits[0]),
                f32::from_bits(bits[1]),
                f32::from_bits(bits[2]),
                f32::from_bits(bits[3]),
            ],
        }
    }

    #[inline(always)]
    pub fn x(&self) -> f32 {
        self.storage[0]
    }

    #[inline(always)]
    pub fn set_x(&mut self, value: f32) {
        self.storage[0] = value;
    }

    #[inline(always)]
    pub fn y(&self) -> f32 {
        self.storage[1]
    }

    #[inline(always)]
    pub fn set_y(&mut self, value: f32) {
        self.storage[1] = value;
    }

    #[inline(always)]
    pub fn z(&self) -> f32 {
        self.storage[2]
    }

    #[inline(always)]
    pub fn set_z(&mut self, value: f32) {
        self.storage[2] = value;
    }

    #[inline(always)]
    pub fn w(&self) -> f32 {
        self.storage[3]
    }

    #[inline(always)]
    pub fn set_w(&mut self, value: f32) {
        self.storage[3] = value;
    }

    /// Lane `i` of the vector.
    #[inline(always)]
    pub fn lane(&self, i: usize) -> f32 {
        self.storage[i]
    }

    #[inline(always)]
    pub fn set_lane(&mut self, i: usize, value: f32) {
        self.storage[i] = value;
    }

    /// Per-lane `squareRoot`, in lane order.
    #[inline(always)]
    pub fn square_root(&self) -> FloatW {
        FloatW::from_storage([
            self.storage[0].sqrt(),
            self.storage[1].sqrt(),
            self.storage[2].sqrt(),
            self.storage[3].sqrt(),
        ])
    }

    pub const ZERO: FloatW = FloatW::splat(0.0);
    pub const ONE: FloatW = FloatW::splat(1.0);
}

impl core::ops::Index<usize> for FloatW {
    type Output = f32;
    #[inline(always)]
    fn index(&self, i: usize) -> &f32 {
        &self.storage[i]
    }
}

impl core::ops::IndexMut<usize> for FloatW {
    #[inline(always)]
    fn index_mut(&mut self, i: usize) -> &mut f32 {
        &mut self.storage[i]
    }
}

/// The only place the engine touches lane arithmetic.
///
/// Each helper applies the scalar IEEE operation to lane 0, then 1, 2, 3; the results are
/// independent, so the order of evaluation cannot change any value.
#[inline(always)]
pub fn wide_add(a: FloatW, b: FloatW) -> FloatW {
    FloatW::from_storage([
        a.storage[0] + b.storage[0],
        a.storage[1] + b.storage[1],
        a.storage[2] + b.storage[2],
        a.storage[3] + b.storage[3],
    ])
}

#[inline(always)]
pub fn wide_sub(a: FloatW, b: FloatW) -> FloatW {
    FloatW::from_storage([
        a.storage[0] - b.storage[0],
        a.storage[1] - b.storage[1],
        a.storage[2] - b.storage[2],
        a.storage[3] - b.storage[3],
    ])
}

#[inline(always)]
pub fn wide_mul(a: FloatW, b: FloatW) -> FloatW {
    FloatW::from_storage([
        a.storage[0] * b.storage[0],
        a.storage[1] * b.storage[1],
        a.storage[2] * b.storage[2],
        a.storage[3] * b.storage[3],
    ])
}

#[inline(always)]
pub fn wide_div(a: FloatW, b: FloatW) -> FloatW {
    FloatW::from_storage([
        a.storage[0] / b.storage[0],
        a.storage[1] / b.storage[1],
        a.storage[2] / b.storage[2],
        a.storage[3] / b.storage[3],
    ])
}

#[inline(always)]
pub fn wide_neg(a: FloatW) -> FloatW {
    FloatW::from_storage([
        flip_sign(a.storage[0]),
        flip_sign(a.storage[1]),
        flip_sign(a.storage[2]),
        flip_sign(a.storage[3]),
    ])
}

// MARK: - Lane comparisons

/// `SIMDMask<SIMD4<Int32>>`: a lane is true when all of its bits are set, false when clear. That
/// is the bit pattern Swift's operator-syntax comparisons produce, which is what lets `select_w`
/// below be a bitwise blend.
#[repr(C, align(16))]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MaskW {
    /// One bit pattern per lane: `u32::MAX` when the lane's comparison holds, else `0`.
    pub bits: [u32; 4],
}

impl MaskW {
    #[inline(always)]
    pub const fn from_bits(bits: [u32; 4]) -> Self {
        Self { bits }
    }

    /// Is lane `i` true?
    #[inline(always)]
    pub const fn lane(&self, i: usize) -> bool {
        self.bits[i] == u32::MAX
    }
}

const TRUE: u32 = u32::MAX;
const FALSE: u32 = 0;

#[inline(always)]
const fn mask_from_lanes(a: bool, b: bool, c: bool, d: bool) -> MaskW {
    MaskW {
        bits: [
            if a { TRUE } else { FALSE },
            if b { TRUE } else { FALSE },
            if c { TRUE } else { FALSE },
            if d { TRUE } else { FALSE },
        ],
    }
}

#[inline(always)]
pub fn wide_gt(a: FloatW, b: FloatW) -> MaskW {
    mask_from_lanes(
        a.storage[0] > b.storage[0],
        a.storage[1] > b.storage[1],
        a.storage[2] > b.storage[2],
        a.storage[3] > b.storage[3],
    )
}

#[inline(always)]
pub fn wide_lt(a: FloatW, b: FloatW) -> MaskW {
    mask_from_lanes(
        a.storage[0] < b.storage[0],
        a.storage[1] < b.storage[1],
        a.storage[2] < b.storage[2],
        a.storage[3] < b.storage[3],
    )
}

#[inline(always)]
pub fn wide_ge(a: FloatW, b: FloatW) -> MaskW {
    mask_from_lanes(
        a.storage[0] >= b.storage[0],
        a.storage[1] >= b.storage[1],
        a.storage[2] >= b.storage[2],
        a.storage[3] >= b.storage[3],
    )
}

#[inline(always)]
pub fn wide_le(a: FloatW, b: FloatW) -> MaskW {
    mask_from_lanes(
        a.storage[0] <= b.storage[0],
        a.storage[1] <= b.storage[1],
        a.storage[2] <= b.storage[2],
        a.storage[3] <= b.storage[3],
    )
}

#[inline(always)]
pub fn wide_eq(a: FloatW, b: FloatW) -> MaskW {
    mask_from_lanes(
        a.storage[0] == b.storage[0],
        a.storage[1] == b.storage[1],
        a.storage[2] == b.storage[2],
        a.storage[3] == b.storage[3],
    )
}

#[inline(always)]
pub fn wide_ne(a: FloatW, b: FloatW) -> MaskW {
    mask_from_lanes(
        a.storage[0] != b.storage[0],
        a.storage[1] != b.storage[1],
        a.storage[2] != b.storage[2],
        a.storage[3] != b.storage[3],
    )
}

#[inline(always)]
pub fn wide_gt_scalar(a: FloatW, b: f32) -> MaskW {
    wide_gt(a, FloatW::splat(b))
}

#[inline(always)]
pub fn wide_lt_scalar(a: FloatW, b: f32) -> MaskW {
    wide_lt(a, FloatW::splat(b))
}

#[inline(always)]
pub fn wide_ge_scalar(a: FloatW, b: f32) -> MaskW {
    wide_ge(a, FloatW::splat(b))
}

#[inline(always)]
pub fn wide_le_scalar(a: FloatW, b: f32) -> MaskW {
    wide_le(a, FloatW::splat(b))
}

#[inline(always)]
pub fn wide_scalar_gt(a: f32, b: FloatW) -> MaskW {
    wide_gt(FloatW::splat(a), b)
}

#[inline(always)]
pub fn wide_scalar_lt(a: f32, b: FloatW) -> MaskW {
    wide_lt(FloatW::splat(a), b)
}

// MARK: - Lane operators

impl Add<FloatW> for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn add(self, b: FloatW) -> FloatW {
        wide_add(self, b)
    }
}

impl Sub<FloatW> for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn sub(self, b: FloatW) -> FloatW {
        wide_sub(self, b)
    }
}

impl Mul<FloatW> for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn mul(self, b: FloatW) -> FloatW {
        wide_mul(self, b)
    }
}

impl Div<FloatW> for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn div(self, b: FloatW) -> FloatW {
        wide_div(self, b)
    }
}

impl Neg for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn neg(self) -> FloatW {
        wide_neg(self)
    }
}

impl Add<f32> for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn add(self, b: f32) -> FloatW {
        wide_add(self, FloatW::splat(b))
    }
}

impl Add<FloatW> for f32 {
    type Output = FloatW;
    #[inline(always)]
    fn add(self, b: FloatW) -> FloatW {
        wide_add(FloatW::splat(self), b)
    }
}

impl Sub<f32> for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn sub(self, b: f32) -> FloatW {
        wide_sub(self, FloatW::splat(b))
    }
}

impl Sub<FloatW> for f32 {
    type Output = FloatW;
    #[inline(always)]
    fn sub(self, b: FloatW) -> FloatW {
        wide_sub(FloatW::splat(self), b)
    }
}

impl Mul<f32> for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn mul(self, s: f32) -> FloatW {
        wide_mul(self, FloatW::splat(s))
    }
}

impl Mul<FloatW> for f32 {
    type Output = FloatW;
    #[inline(always)]
    fn mul(self, a: FloatW) -> FloatW {
        wide_mul(FloatW::splat(self), a)
    }
}

impl Div<f32> for FloatW {
    type Output = FloatW;
    #[inline(always)]
    fn div(self, s: f32) -> FloatW {
        wide_div(self, FloatW::splat(s))
    }
}

impl Div<FloatW> for f32 {
    type Output = FloatW;
    #[inline(always)]
    fn div(self, a: FloatW) -> FloatW {
        wide_div(FloatW::splat(self), a)
    }
}

impl AddAssign<FloatW> for FloatW {
    #[inline(always)]
    fn add_assign(&mut self, b: FloatW) {
        *self = wide_add(*self, b);
    }
}

impl SubAssign<FloatW> for FloatW {
    #[inline(always)]
    fn sub_assign(&mut self, b: FloatW) {
        *self = wide_sub(*self, b);
    }
}

impl MulAssign<FloatW> for FloatW {
    #[inline(always)]
    fn mul_assign(&mut self, b: FloatW) {
        *self = wide_mul(*self, b);
    }
}

impl DivAssign<FloatW> for FloatW {
    #[inline(always)]
    fn div_assign(&mut self, b: FloatW) {
        *self = wide_div(*self, b);
    }
}

/// Transpose four rows into four lane columns. Building the columns from scalar lanes makes LLVM
/// split the downstream SIMD math into partial vectors.
///
/// Apple's `simd_transpose(simd_float4x4(a, b, c, d))` yields exactly these columns, so the lane
/// gather below is the portable form of the same operation on every target.
#[inline(always)]
pub fn transpose4(a: FloatW, b: FloatW, c: FloatW, d: FloatW) -> (FloatW, FloatW, FloatW, FloatW) {
    portable_transpose4(a, b, c, d)
}

/// Lane-only transpose for platforms without Apple's simd matrix module.
#[inline(always)]
pub fn portable_transpose4(
    a: FloatW,
    b: FloatW,
    c: FloatW,
    d: FloatW,
) -> (FloatW, FloatW, FloatW, FloatW) {
    (
        FloatW::new(a.x(), b.x(), c.x(), d.x()),
        FloatW::new(a.y(), b.y(), c.y(), d.y()),
        FloatW::new(a.z(), b.z(), c.z(), d.z()),
        FloatW::new(a.w(), b.w(), c.w(), d.w()),
    )
}

// MARK: - Wide aggregate types

/// Four 3-vectors, one per lane. `x`, `y`, and `z` each hold four lanes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Vec3W {
    pub x: FloatW,
    pub y: FloatW,
    pub z: FloatW,
}

impl Vec3W {
    /// `init(x:y:z:)` and the positional `init(_:_:_:)`.
    #[inline(always)]
    pub const fn new(x: FloatW, y: FloatW, z: FloatW) -> Self {
        Self { x, y, z }
    }

    /// `init()`
    #[inline(always)]
    pub const fn zero() -> Self {
        Self {
            x: FloatW::ZERO,
            y: FloatW::ZERO,
            z: FloatW::ZERO,
        }
    }

    /// Replicate one vector across every lane.
    pub fn splat(v: Vec3) -> Self {
        Self {
            x: FloatW::splat(v.x),
            y: FloatW::splat(v.y),
            z: FloatW::splat(v.z),
        }
    }

    /// Gather one vector per lane from four sources.
    pub fn gather(a: Vec3, b: Vec3, c: Vec3, d: Vec3) -> Self {
        Self {
            x: FloatW::new(a.x, b.x, c.x, d.x),
            y: FloatW::new(a.y, b.y, c.y, d.y),
            z: FloatW::new(a.z, b.z, c.z, d.z),
        }
    }

    /// Lane `i` of the wide vector.
    #[inline(always)]
    pub fn lane(&self, i: usize) -> Vec3 {
        Vec3::new(self.x[i], self.y[i], self.z[i])
    }

    #[inline(always)]
    pub fn set_lane(&mut self, i: usize, value: Vec3) {
        self.x[i] = value.x;
        self.y[i] = value.y;
        self.z[i] = value.z;
    }
}

/// Four 2-vectors, one per lane.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Vec2W {
    pub x: FloatW,
    pub y: FloatW,
}

impl Vec2W {
    #[inline(always)]
    pub const fn new(x: FloatW, y: FloatW) -> Self {
        Self { x, y }
    }

    /// `init()`
    #[inline(always)]
    pub const fn zero() -> Self {
        Self {
            x: FloatW::ZERO,
            y: FloatW::ZERO,
        }
    }

    pub fn gather(a: Vec2, b: Vec2, c: Vec2, d: Vec2) -> Self {
        Self {
            x: FloatW::new(a.x, b.x, c.x, d.x),
            y: FloatW::new(a.y, b.y, c.y, d.y),
        }
    }

    #[inline(always)]
    pub fn lane(&self, i: usize) -> Vec2 {
        Vec2::new(self.x[i], self.y[i])
    }

    #[inline(always)]
    pub fn set_lane(&mut self, i: usize, value: Vec2) {
        self.x[i] = value.x;
        self.y[i] = value.y;
    }
}

/// Four quaternions, one per lane.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct QuatW {
    pub v: Vec3W,
    pub s: FloatW,
}

impl QuatW {
    #[inline(always)]
    pub const fn new(v: Vec3W, s: FloatW) -> Self {
        Self { v, s }
    }

    /// `init()`
    #[inline(always)]
    pub const fn zero() -> Self {
        Self {
            v: Vec3W::zero(),
            s: FloatW::ZERO,
        }
    }
}

/// A symmetric 3x3 matrix, four lanes. Only the six unique entries are stored.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SymMatrix3W {
    pub xx: FloatW,
    pub yy: FloatW,
    pub zz: FloatW,
    pub xy: FloatW,
    pub xz: FloatW,
    pub yz: FloatW,
}

impl SymMatrix3W {
    /// `init(_:_:_:_:)` - gather one matrix per lane.
    pub fn gather(a: SymMatrix3, b: SymMatrix3, c: SymMatrix3, d: SymMatrix3) -> Self {
        Self {
            xx: FloatW::new(a.cxx, b.cxx, c.cxx, d.cxx),
            yy: FloatW::new(a.cyy, b.cyy, c.cyy, d.cyy),
            zz: FloatW::new(a.czz, b.czz, c.czz, d.czz),
            xy: FloatW::new(a.cxy, b.cxy, c.cxy, d.cxy),
            xz: FloatW::new(a.cxz, b.cxz, c.cxz, d.cxz),
            yz: FloatW::new(a.cyz, b.cyz, c.cyz, d.cyz),
        }
    }
}

/// A full 3x3 matrix, four lanes, stored as columns like Swift's `Matrix3W`.
///
/// Wide joint batches use this rather than [`SymMatrix3W`]: `invInertiaWorld` is only
/// approximately symmetric after the `R * I * Rᵀ` product, and reading the upper triangle for
/// both halves changes the last bits of the solve.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct Matrix3W {
    pub cx: Vec3W,
    pub cy: Vec3W,
    pub cz: Vec3W,
}

impl Matrix3W {
    /// Store one matrix in lane `i`, keeping every column exactly.
    #[inline(always)]
    pub fn set_lane(&mut self, i: usize, m: crate::math::types::Matrix3) {
        self.cx.set_lane(i, m.cx);
        self.cy.set_lane(i, m.cy);
        self.cz.set_lane(i, m.cz);
    }
}

/// A symmetric 2x2 matrix, four lanes.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct SymMatrix2W {
    pub xx: FloatW,
    pub yy: FloatW,
    pub xy: FloatW,
}

impl SymMatrix2W {
    /// `init(_:_:_:_:)` - gather one matrix per lane.
    pub fn gather(a: Matrix2, b: Matrix2, c: Matrix2, d: Matrix2) -> Self {
        Self {
            xx: FloatW::new(a.cx.x, b.cx.x, c.cx.x, d.cx.x),
            yy: FloatW::new(a.cy.y, b.cy.y, c.cy.y, d.cy.y),
            xy: FloatW::new(a.cx.y, b.cx.y, c.cx.y, d.cx.y),
        }
    }
}

// MARK: - Lane-wise operations

#[inline(always)]
pub fn add_w(a: Vec3W, b: Vec3W) -> Vec3W {
    Vec3W::new(a.x + b.x, a.y + b.y, a.z + b.z)
}

#[inline(always)]
pub fn sub_w(a: Vec3W, b: Vec3W) -> Vec3W {
    Vec3W::new(a.x - b.x, a.y - b.y, a.z - b.z)
}

#[inline(always)]
pub fn dot_w(a: Vec3W, b: Vec3W) -> FloatW {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[inline(always)]
pub fn cross_w(a: Vec3W, b: Vec3W) -> Vec3W {
    Vec3W::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

#[inline(always)]
pub fn mul_add_w(a: Vec3W, b: FloatW, c: Vec3W) -> Vec3W {
    Vec3W::new(a.x + b * c.x, a.y + b * c.y, a.z + b * c.z)
}

#[inline(always)]
pub fn mul_mv_w(m: SymMatrix3W, a: Vec3W) -> Vec3W {
    Vec3W::new(
        m.xx * a.x + (m.xy * a.y + m.xz * a.z),
        m.xy * a.x + (m.yy * a.y + m.yz * a.z),
        m.xz * a.x + (m.yz * a.y + m.zz * a.z),
    )
}

#[inline(always)]
pub fn mul_mv2_w(m: SymMatrix2W, a: Vec2W) -> Vec2W {
    Vec2W::new(m.xx * a.x + m.xy * a.y, m.xy * a.x + m.yy * a.y)
}

#[inline(always)]
pub fn neg_vec3_w(a: Vec3W) -> Vec3W {
    Vec3W::new(neg_w(a.x), neg_w(a.y), neg_w(a.z))
}

// MARK: - Body state lanes

/// Four body states, gathered for the wide solver. Lane `i` mirrors `states[index[i] - 1]`, or the
/// identity state when the index is zero (static).
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct BodyStateW {
    pub v: Vec3W,
    pub w: Vec3W,
    pub dp: Vec3W,
    pub dq: QuatW,
}

impl BodyStateW {
    /// Build a gathered state directly. The default initializer can be outlined into a call
    /// that clears all thirteen vectors before the gather overwrites every one of them.
    #[inline(always)]
    pub const fn new(v: Vec3W, w: Vec3W, dp: Vec3W, dq: QuatW) -> Self {
        Self { v, w, dp, dq }
    }
}

impl crate::simulation::world_runtime::World {
    /// Inverse-rotate four vectors by four quaternions, matching the wide scalar helper
    /// `invRotateVectorWide`. That helper is deliberately not bit-equivalent to the generic
    /// `invRotateVector`, so the lane math has to follow it exactly.
    #[inline(always)]
    pub(crate) fn inv_rotate_vec_w(&self, q: QuatW, a: Vec3W) -> Vec3W {
        let mut t = cross_w(q.v, a);
        t = Vec3W::new(t.x + t.x, t.y + t.y, t.z + t.z);
        let u = cross_w(q.v, t);
        Vec3W::new(
            (a.x - q.s * t.x) + u.x,
            (a.y - q.s * t.y) + u.y,
            (a.z - q.s * t.z) + u.z,
        )
    }
}

/// Selection preserves the scalar operand order, including signed zeros and NaNs. A bitwise blend
/// (NEON `bsl`): the generic `replacing(with:where:)` is lowered lane by lane.
#[inline(always)]
pub fn select_w(mask: MaskW, a: FloatW, b: FloatW) -> FloatW {
    let m = mask.bits;
    let bits_a = a.to_bits();
    let bits_b = b.to_bits();
    FloatW::from_bits([
        (bits_a[0] & m[0]) | (bits_b[0] & !m[0]),
        (bits_a[1] & m[1]) | (bits_b[1] & !m[1]),
        (bits_a[2] & m[2]) | (bits_b[2] & !m[2]),
        (bits_a[3] & m[3]) | (bits_b[3] & !m[3]),
    ])
}

#[inline(always)]
pub fn select_vec3_w(mask: MaskW, a: Vec3W, b: Vec3W) -> Vec3W {
    Vec3W::new(
        select_w(mask, a.x, b.x),
        select_w(mask, a.y, b.y),
        select_w(mask, a.z, b.z),
    )
}

#[inline(always)]
pub fn max_w(a: FloatW, b: FloatW) -> FloatW {
    select_w(wide_gt(a, b), a, b)
}

#[inline(always)]
pub fn min_w(a: FloatW, b: FloatW) -> FloatW {
    select_w(wide_lt(a, b), a, b)
}

#[inline(always)]
pub fn mul_svw(s: FloatW, a: Vec3W) -> Vec3W {
    Vec3W::new(s * a.x, s * a.y, s * a.z)
}

#[inline(always)]
pub fn mul_w(s: FloatW, a: Vec3W) -> Vec3W {
    mul_svw(s, a)
}

/// SIMD's generic unary minus uses zero subtraction, which loses the sign of +0.
/// Flip the IEEE sign bit to match scalar Float negation, including zero and NaNs.
#[inline(always)]
pub fn neg_w(a: FloatW) -> FloatW {
    wide_neg(a)
}
