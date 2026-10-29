//! Four wide constraint batches. Constraints in one batch share no vertex, so the four lanes are
//! solved together with SIMD and the result equals solving them one after another. Batches are
//! built greedily in the settings' constraint order with a short look-ahead window, which keeps the
//! anchor-outward order of the edges (see `SoftBodySettings.optimize`) nearly intact while breaking
//! the dependency chain between consecutive constraints.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftBodyBatches.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::soft_body::soft_body_runtime::SoftBodyRecord`: fields `edge_batches: Vec<EdgeBatch>`,
//!   `bend_batches: Vec<BendBatch>`, `volume_batches: Vec<VolumeBatch>` (declared in
//!   `SoftBodyRuntime.swift` right after the contact members), and `settings` with
//!   `vertices: Vec<SoftBodyVertex>`, `edges: Vec<SoftBodyEdge>` (`edge.vertex.x` / `edge.vertex.y`
//!   carrying the Swift `SIMD2<UInt32>` pair, plus `rest_length: f32`, `compliance: f32`),
//!   `dihedral_bends: Vec<SoftBodyDihedralBend>` (`bend.vertex.x/.y/.z/.w` carrying the Swift
//!   `SIMD4<UInt32>` quad, plus `initial_angle: f32`, `compliance: f32`) and
//!   `volumes: Vec<SoftBodyVolume>` (`volume.vertex.x/.y/.z/.w`, `six_rest_volume: f32`,
//!   `compliance: f32`), matching `SoftBodySettings.swift`.
//!
//! Lane types: Swift's `SIMD4<Float>` / `SIMD4<Bool>` / `SIMD4<Int32>` become the local [`FloatX4`],
//! [`BoolX4`] and plain `[i32; 4]` arrays. Every operation runs per lane in the same order the
//! Swift vector operation did, which is the scalar lane structure the migration rules require;
//! `Int(index[k])` indexing becomes `index[k] as usize` (Swift's `Int(SIMD4<Int32>)` lane is
//! always a non-negative vertex index here).
//!
//! Buffer-pointer adaptation (behaviour identical): Swift's
//! `UnsafeMutableBufferPointer<Vec3>` / `UnsafeBufferPointer<Float>` parameters become
//! `&mut [Vec3]` / `&[f32]` slices. The gather/scatter writes go through the same caller-owned
//! buffers in the same per-batch, per-lane order; no reallocation or aliasing change occurs, so no
//! `unsafe` is needed in this file.

use crate::base::constants as b3;
use crate::math::types::Vec3;
use crate::soft_body::soft_body_runtime::SoftBodyRecord;

// MARK: - Lane value types

/// Swift's `typealias FloatX4 = SIMD4<Float>`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct FloatX4 {
    pub lanes: [f32; 4],
}

impl FloatX4 {
    #[inline(always)]
    pub fn new(a: f32, b: f32, c: f32, d: f32) -> Self {
        Self { lanes: [a, b, c, d] }
    }

    /// Swift's splat form used on the right of a vector op (`compliance * invH2`, `x .> 0`, ...).
    #[inline(always)]
    pub fn splat(s: f32) -> Self {
        Self { lanes: [s, s, s, s] }
    }

    /// Swift's per-lane `squareRoot()`.
    #[inline(always)]
    pub fn square_root(self) -> FloatX4 {
        FloatX4 {
            lanes: [
                self.lanes[0].sqrt(),
                self.lanes[1].sqrt(),
                self.lanes[2].sqrt(),
                self.lanes[3].sqrt(),
            ],
        }
    }

    /// Swift's `a .> b` (vector-vector) and `a .> s` (vector-splat).
    #[inline(always)]
    pub fn gt(self, b: FloatX4) -> BoolX4 {
        BoolX4 {
            lanes: [
                self.lanes[0] > b.lanes[0],
                self.lanes[1] > b.lanes[1],
                self.lanes[2] > b.lanes[2],
                self.lanes[3] > b.lanes[3],
            ],
        }
    }

    #[inline(always)]
    pub fn gt_splat(self, s: f32) -> BoolX4 {
        self.gt(FloatX4::splat(s))
    }

    /// Swift's `a .< b` / `a .< s`.
    #[inline(always)]
    pub fn lt(self, b: FloatX4) -> BoolX4 {
        BoolX4 {
            lanes: [
                self.lanes[0] < b.lanes[0],
                self.lanes[1] < b.lanes[1],
                self.lanes[2] < b.lanes[2],
                self.lanes[3] < b.lanes[3],
            ],
        }
    }

    #[inline(always)]
    pub fn lt_splat(self, s: f32) -> BoolX4 {
        self.lt(FloatX4::splat(s))
    }

    /// Swift's `a .== b` / `a .== s`.
    #[inline(always)]
    pub fn eq(self, b: FloatX4) -> BoolX4 {
        BoolX4 {
            lanes: [
                self.lanes[0] == b.lanes[0],
                self.lanes[1] == b.lanes[1],
                self.lanes[2] == b.lanes[2],
                self.lanes[3] == b.lanes[3],
            ],
        }
    }

    #[inline(always)]
    pub fn eq_splat(self, s: f32) -> BoolX4 {
        self.eq(FloatX4::splat(s))
    }

    /// Swift's `a.replacing(with: b, where: mask)`; the `where:` label is a Rust keyword, so the
    /// mask is the third positional parameter. A scalar `with:` value is splatted through
    /// `impl Into<FloatX4>`, matching Swift's scalar overload.
    #[inline(always)]
    pub fn replacing(self, with: impl Into<FloatX4>, mask: BoolX4) -> FloatX4 {
        let with = with.into();
        FloatX4 {
            lanes: [
                if mask.lanes[0] { with.lanes[0] } else { self.lanes[0] },
                if mask.lanes[1] { with.lanes[1] } else { self.lanes[1] },
                if mask.lanes[2] { with.lanes[2] } else { self.lanes[2] },
                if mask.lanes[3] { with.lanes[3] } else { self.lanes[3] },
            ],
        }
    }
}

impl From<f32> for FloatX4 {
    #[inline(always)]
    fn from(s: f32) -> Self {
        FloatX4::splat(s)
    }
}

impl std::ops::Index<usize> for FloatX4 {
    type Output = f32;
    #[inline(always)]
    fn index(&self, k: usize) -> &f32 {
        &self.lanes[k]
    }
}

impl std::ops::IndexMut<usize> for FloatX4 {
    #[inline(always)]
    fn index_mut(&mut self, k: usize) -> &mut f32 {
        &mut self.lanes[k]
    }
}

/// Swift's per-lane `+` (vector-vector).
impl std::ops::Add for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn add(self, b: FloatX4) -> FloatX4 {
        FloatX4 {
            lanes: [
                self.lanes[0] + b.lanes[0],
                self.lanes[1] + b.lanes[1],
                self.lanes[2] + b.lanes[2],
                self.lanes[3] + b.lanes[3],
            ],
        }
    }
}

/// Swift's per-lane `+` (vector-splat).
impl std::ops::Add<f32> for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn add(self, s: f32) -> FloatX4 {
        self + FloatX4::splat(s)
    }
}

/// Swift's per-lane `+` (splat-vector).
impl std::ops::Add<FloatX4> for f32 {
    type Output = FloatX4;
    #[inline(always)]
    fn add(self, a: FloatX4) -> FloatX4 {
        FloatX4::splat(self) + a
    }
}

/// Swift's per-lane `-` (vector-vector).
impl std::ops::Sub for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn sub(self, b: FloatX4) -> FloatX4 {
        FloatX4 {
            lanes: [
                self.lanes[0] - b.lanes[0],
                self.lanes[1] - b.lanes[1],
                self.lanes[2] - b.lanes[2],
                self.lanes[3] - b.lanes[3],
            ],
        }
    }
}

/// Swift's per-lane `-` (vector-splat), e.g. `c - B3.twoPi`.
impl std::ops::Sub<f32> for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn sub(self, s: f32) -> FloatX4 {
        self - FloatX4::splat(s)
    }
}

/// Swift's per-lane `-` (splat-vector), e.g. `1.57079637 - r`.
impl std::ops::Sub<FloatX4> for f32 {
    type Output = FloatX4;
    #[inline(always)]
    fn sub(self, a: FloatX4) -> FloatX4 {
        FloatX4::splat(self) - a
    }
}

/// Swift's per-lane `*` (vector-vector).
impl std::ops::Mul for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn mul(self, b: FloatX4) -> FloatX4 {
        FloatX4 {
            lanes: [
                self.lanes[0] * b.lanes[0],
                self.lanes[1] * b.lanes[1],
                self.lanes[2] * b.lanes[2],
                self.lanes[3] * b.lanes[3],
            ],
        }
    }
}

/// Swift's per-lane `*` (vector-splat), e.g. `batch.compliance * invH2`.
impl std::ops::Mul<f32> for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn mul(self, s: f32) -> FloatX4 {
        self * FloatX4::splat(s)
    }
}

/// Swift's per-lane `*` (splat-vector), e.g. `0.024840285 * q`.
impl std::ops::Mul<FloatX4> for f32 {
    type Output = FloatX4;
    #[inline(always)]
    fn mul(self, a: FloatX4) -> FloatX4 {
        FloatX4::splat(self) * a
    }
}

/// Swift's per-lane `/` (vector-vector).
impl std::ops::Div for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn div(self, b: FloatX4) -> FloatX4 {
        FloatX4 {
            lanes: [
                self.lanes[0] / b.lanes[0],
                self.lanes[1] / b.lanes[1],
                self.lanes[2] / b.lanes[2],
                self.lanes[3] / b.lanes[3],
            ],
        }
    }
}

/// Swift's per-lane `/` (vector-splat).
impl std::ops::Div<f32> for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn div(self, s: f32) -> FloatX4 {
        self / FloatX4::splat(s)
    }
}

/// Swift's per-lane `/` (splat-vector), e.g. `1 / safeLen`.
impl std::ops::Div<FloatX4> for f32 {
    type Output = FloatX4;
    #[inline(always)]
    fn div(self, a: FloatX4) -> FloatX4 {
        FloatX4::splat(self) / a
    }
}

/// Swift's per-lane prefix `-`.
impl std::ops::Neg for FloatX4 {
    type Output = FloatX4;
    #[inline(always)]
    fn neg(self) -> FloatX4 {
        FloatX4 {
            lanes: [-self.lanes[0], -self.lanes[1], -self.lanes[2], -self.lanes[3]],
        }
    }
}

/// Swift's `SIMD4<Bool>` produced by the `.>`, `.<`, `.==` operators and combined with `.&` / `.!`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct BoolX4 {
    pub lanes: [bool; 4],
}

/// Swift's `a .& b`.
impl std::ops::BitAnd for BoolX4 {
    type Output = BoolX4;
    #[inline(always)]
    fn bitand(self, b: BoolX4) -> BoolX4 {
        BoolX4 {
            lanes: [
                self.lanes[0] & b.lanes[0],
                self.lanes[1] & b.lanes[1],
                self.lanes[2] & b.lanes[2],
                self.lanes[3] & b.lanes[3],
            ],
        }
    }
}

/// Swift's prefix `.!a`.
impl std::ops::Not for BoolX4 {
    type Output = BoolX4;
    #[inline(always)]
    fn not(self) -> BoolX4 {
        BoolX4 {
            lanes: [
                !self.lanes[0],
                !self.lanes[1],
                !self.lanes[2],
                !self.lanes[3],
            ],
        }
    }
}

// MARK: - Batch records

/// Swift's `EdgeBatch: BitwiseCopyable`; `SIMD4<Int32>` becomes `[i32; 4]`, `FloatX4()` the
/// all-zero [`FloatX4`] default.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct EdgeBatch {
    pub v0: [i32; 4],
    pub v1: [i32; 4],
    pub rest_length: FloatX4,
    pub compliance: FloatX4,
    pub lane_count: i32,
}

/// Swift's `BendBatch: BitwiseCopyable`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct BendBatch {
    pub v0: [i32; 4],
    pub v1: [i32; 4],
    pub v2: [i32; 4],
    pub v3: [i32; 4],
    pub initial_angle: FloatX4,
    pub compliance: FloatX4,
    pub lane_count: i32,
}

/// Swift's `VolumeBatch: BitwiseCopyable`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct VolumeBatch {
    pub v0: [i32; 4],
    pub v1: [i32; 4],
    pub v2: [i32; 4],
    pub v3: [i32; 4],
    pub six_rest_volume: FloatX4,
    pub compliance: FloatX4,
    pub lane_count: i32,
}

/// Group constraints into batches of up to four vertex-disjoint constraints, in order.
/// `vertices(i)` gives the vertices of constraint i and how many of the four are used.
///
/// Swift's default `window: Int = 64` is passed explicitly by ported callers.
pub fn build_constraint_batches(
    count: usize,
    vertex_count: usize,
    window: usize,
    vertices: impl Fn(usize) -> ([i32; 4], usize),
) -> Vec<Vec<usize>> {
    let mut batches: Vec<Vec<usize>> = Vec::new();
    batches.reserve(count / 4 + 1);
    let mut assigned = vec![false; count];
    let mut mark = vec![-1i64; vertex_count];
    let mut first = 0;
    let mut batch_index = 0i64;
    while first < count {
        while first < count && assigned[first] {
            first += 1;
        }
        if first >= count {
            break;
        }
        let mut batch: Vec<usize> = Vec::new();
        let mut i = first;
        let end = std::cmp::min(count, first + window);
        while i < end && batch.len() < 4 {
            if assigned[i] == false {
                let (vs, used) = vertices(i);
                let mut free = true;
                for k in 0..used {
                    if mark[vs[k] as usize] == batch_index {
                        free = false;
                    }
                }
                if free {
                    for k in 0..used {
                        mark[vs[k] as usize] = batch_index;
                    }
                    batch.push(i);
                    assigned[i] = true;
                }
            }
            i += 1;
        }
        batches.push(batch);
        batch_index += 1;
    }
    batches
}

impl SoftBodyRecord {
    pub fn build_batches(&mut self) {
        let n = self.settings.vertices.len();

        let edges = &self.settings.edges;
        let edge_batches = build_constraint_batches(edges.len(), n, 64, |i| {
            (
                [edges[i].vertex.x as i32, edges[i].vertex.y as i32, 0, 0],
                2,
            )
        })
        .into_iter()
        .map(|lanes| {
            let mut batch = EdgeBatch::default();
            batch.lane_count = lanes.len() as i32;
            for (k, e) in lanes.iter().enumerate() {
                batch.v0[k] = edges[*e].vertex.x as i32;
                batch.v1[k] = edges[*e].vertex.y as i32;
                batch.rest_length[k] = edges[*e].rest_length;
                batch.compliance[k] = edges[*e].compliance;
            }
            batch
        })
        .collect();
        self.edge_batches = edge_batches;

        let bends = &self.settings.dihedral_bends;
        let bend_batches = build_constraint_batches(bends.len(), n, 64, |i| {
            (
                [
                    bends[i].vertex.x as i32,
                    bends[i].vertex.y as i32,
                    bends[i].vertex.z as i32,
                    bends[i].vertex.w as i32,
                ],
                4,
            )
        })
        .into_iter()
        .map(|lanes| {
            let mut batch = BendBatch::default();
            batch.lane_count = lanes.len() as i32;
            for (k, b) in lanes.iter().enumerate() {
                let v = bends[*b].vertex;
                batch.v0[k] = v.x as i32;
                batch.v1[k] = v.y as i32;
                batch.v2[k] = v.z as i32;
                batch.v3[k] = v.w as i32;
                batch.initial_angle[k] = bends[*b].initial_angle;
                batch.compliance[k] = bends[*b].compliance;
            }
            batch
        })
        .collect();
        self.bend_batches = bend_batches;

        let volumes = &self.settings.volumes;
        let volume_batches = build_constraint_batches(volumes.len(), n, 64, |i| {
            (
                [
                    volumes[i].vertex.x as i32,
                    volumes[i].vertex.y as i32,
                    volumes[i].vertex.z as i32,
                    volumes[i].vertex.w as i32,
                ],
                4,
            )
        })
        .into_iter()
        .map(|lanes| {
            let mut batch = VolumeBatch::default();
            batch.lane_count = lanes.len() as i32;
            for (k, t) in lanes.iter().enumerate() {
                let v = volumes[*t].vertex;
                batch.v0[k] = v.x as i32;
                batch.v1[k] = v.y as i32;
                batch.v2[k] = v.z as i32;
                batch.v3[k] = v.w as i32;
                batch.six_rest_volume[k] = volumes[*t].six_rest_volume;
                batch.compliance[k] = volumes[*t].compliance;
            }
            batch
        })
        .collect();
        self.volume_batches = volume_batches;
    }
}

// MARK: - Lane helpers

/// Positions of four vertices as three component vectors. Unused lanes repeat lane zero.
#[derive(Clone, Copy, Debug)]
pub struct Vec3X4 {
    pub x: FloatX4,
    pub y: FloatX4,
    pub z: FloatX4,
}

impl Vec3X4 {
    #[inline(always)]
    pub fn new(x: FloatX4, y: FloatX4, z: FloatX4) -> Self {
        Self { x, y, z }
    }

    /// Swift's `init(gather p:_ index:)`.
    #[inline(always)]
    pub fn gather(p: &[Vec3], index: [i32; 4]) -> Self {
        let a = p[index[0] as usize];
        let b = p[index[1] as usize];
        let c = p[index[2] as usize];
        let d = p[index[3] as usize];
        Self {
            x: FloatX4::new(a.x, b.x, c.x, d.x),
            y: FloatX4::new(a.y, b.y, c.y, d.y),
            z: FloatX4::new(a.z, b.z, c.z, d.z),
        }
    }

    #[inline(always)]
    pub fn scatter(self, p: &mut [Vec3], index: [i32; 4], lanes: i32) {
        if lanes == 4 {
            // Full batch: straight line stores.
            p[index[0] as usize] = Vec3::new(self.x[0], self.y[0], self.z[0]);
            p[index[1] as usize] = Vec3::new(self.x[1], self.y[1], self.z[1]);
            p[index[2] as usize] = Vec3::new(self.x[2], self.y[2], self.z[2]);
            p[index[3] as usize] = Vec3::new(self.x[3], self.y[3], self.z[3]);
            return;
        }
        for k in 0..lanes as usize {
            p[index[k] as usize] = Vec3::new(self.x[k], self.y[k], self.z[k]);
        }
    }
}

/// Swift's `static func + (a: Vec3X4, b: Vec3X4)`.
impl std::ops::Add for Vec3X4 {
    type Output = Vec3X4;
    #[inline(always)]
    fn add(self, b: Vec3X4) -> Vec3X4 {
        Vec3X4::new(self.x + b.x, self.y + b.y, self.z + b.z)
    }
}

/// Swift's `static func - (a: Vec3X4, b: Vec3X4)`.
impl std::ops::Sub for Vec3X4 {
    type Output = Vec3X4;
    #[inline(always)]
    fn sub(self, b: Vec3X4) -> Vec3X4 {
        Vec3X4::new(self.x - b.x, self.y - b.y, self.z - b.z)
    }
}

/// Swift's `static func * (s: FloatX4, a: Vec3X4)`.
impl std::ops::Mul<Vec3X4> for FloatX4 {
    type Output = Vec3X4;
    #[inline(always)]
    fn mul(self, a: Vec3X4) -> Vec3X4 {
        Vec3X4::new(self * a.x, self * a.y, self * a.z)
    }
}

/// Swift's `static prefix func - (a: Vec3X4)`.
impl std::ops::Neg for Vec3X4 {
    type Output = Vec3X4;
    #[inline(always)]
    fn neg(self) -> Vec3X4 {
        Vec3X4::new(-self.x, -self.y, -self.z)
    }
}

#[inline(always)]
fn dot_x4(a: Vec3X4, b: Vec3X4) -> FloatX4 {
    a.x * b.x + a.y * b.y + a.z * b.z
}

#[inline(always)]
fn cross_x4(a: Vec3X4, b: Vec3X4) -> Vec3X4 {
    Vec3X4::new(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    )
}

#[inline(always)]
fn gather_x4(w: &[f32], index: [i32; 4]) -> FloatX4 {
    FloatX4::new(
        w[index[0] as usize],
        w[index[1] as usize],
        w[index[2] as usize],
        w[index[3] as usize],
    )
}

#[inline(always)]
fn abs_x4(a: FloatX4) -> FloatX4 {
    a.replacing(-a, a.lt_splat(0.0))
}

/// Swift's stdlib SIMD `pointwiseMax(_:_:)`, lane-wise `max`.
#[inline(always)]
pub fn pointwise_max(a: FloatX4, b: FloatX4) -> FloatX4 {
    FloatX4 {
        lanes: [
            a.lanes[0].max(b.lanes[0]),
            a.lanes[1].max(b.lanes[1]),
            a.lanes[2].max(b.lanes[2]),
            a.lanes[3].max(b.lanes[3]),
        ],
    }
}

/// Swift's stdlib SIMD `pointwiseMin(_:_:)`, lane-wise `min`.
#[inline(always)]
pub fn pointwise_min(a: FloatX4, b: FloatX4) -> FloatX4 {
    FloatX4 {
        lanes: [
            a.lanes[0].min(b.lanes[0]),
            a.lanes[1].min(b.lanes[1]),
            a.lanes[2].min(b.lanes[2]),
            a.lanes[3].min(b.lanes[3]),
        ],
    }
}

/// Lane-wise `atan2Approx`, the same polynomial and branches.
#[inline(always)]
pub fn atan2_approx_x4(y: FloatX4, x: FloatX4) -> FloatX4 {
    let ax = abs_x4(x);
    let ay = abs_x4(y);
    let mx = pointwise_max(ay, ax);
    let mn = pointwise_min(ay, ax);
    let a = mn / mx.replacing(1.0, mx.eq_splat(0.0));
    let s = a * a;
    let c = s * a;
    let q = s * s;
    let mut r = 0.024840285 * q + 0.18681418;
    let t = -0.094097948 * q - 0.33213072;
    r = r * s + t;
    r = r * c + a;
    r = r.replacing(1.57079637 - r, ay.gt(ax));
    r = r.replacing(3.14159274 - r, x.lt_splat(0.0));
    r = r.replacing(-r, y.lt_splat(0.0));
    r.replacing(0.0, x.eq_splat(0.0) & y.eq_splat(0.0))
}

// MARK: - Batched solvers

impl SoftBodyRecord {
    pub fn solve_edge_batches(&self, x: &mut [Vec3], w: &[f32], inv_h2: f32) {
        for batch in self.edge_batches.iter() {
            let a = Vec3X4::gather(x, batch.v0);
            let b = Vec3X4::gather(x, batch.v1);
            let w0 = gather_x4(w, batch.v0);
            let w1 = gather_x4(w, batch.v1);
            let d = b - a;
            let len = dot_x4(d, d).square_root();
            let denominator = w0 + w1 + batch.compliance * inv_h2;
            let valid = len.gt_splat(1.0e-9) & denominator.gt_splat(0.0);
            let safe_len = len.replacing(1.0, (!valid));
            let safe_denominator = denominator.replacing(1.0, (!valid));
            let mut scale = -(len - batch.rest_length) / safe_denominator / safe_len;
            scale = scale.replacing(0.0, (!valid));
            let correction = scale * d;
            (a - w0 * correction).scatter(x, batch.v0, batch.lane_count);
            (b + w1 * correction).scatter(x, batch.v1, batch.lane_count);
        }
    }

    pub fn solve_bend_batches(&self, x: &mut [Vec3], w: &[f32], inv_h2: f32) {
        for batch in self.bend_batches.iter() {
            let p0 = Vec3X4::gather(x, batch.v0);
            let p1 = Vec3X4::gather(x, batch.v1);
            let p2 = Vec3X4::gather(x, batch.v2);
            let p3 = Vec3X4::gather(x, batch.v3);
            let w0 = gather_x4(w, batch.v0);
            let w1 = gather_x4(w, batch.v1);
            let w2 = gather_x4(w, batch.v2);
            let w3 = gather_x4(w, batch.v3);

            let e = p1 - p0;
            let len = dot_x4(e, e).square_root();
            let n1 = cross_x4(e, p2 - p0);
            let n2 = cross_x4(p3 - p0, e);
            let n1_squared = dot_x4(n1, n1);
            let n2_squared = dot_x4(n2, n2);
            let valid = len.gt_splat(1.0e-9)
                & n1_squared.gt_splat(1.0e-18)
                & n2_squared.gt_splat(1.0e-18)
                & ((w0 + w1 + w2 + w3).gt_splat(0.0));
            let safe_len = len.replacing(1.0, (!valid));
            let inv_len = 1.0 / safe_len;
            let m1 = (1.0 / n1_squared.replacing(1.0, (!valid))) * n1;
            let m2 = (1.0 / n2_squared.replacing(1.0, (!valid))) * n2;

            let angle = atan2_approx_x4(
                dot_x4(cross_x4(n1, n2), e) * inv_len,
                dot_x4(n1, n2),
            );
            let mut c = angle - batch.initial_angle;
            c = c.replacing(c - b3::TWO_PI, c.gt_splat(b3::PI));
            c = c.replacing(c + b3::TWO_PI, c.lt_splat(-b3::PI));

            let g2 = -(safe_len * m1);
            let g3 = -(safe_len * m2);
            let g0 = -((dot_x4(p2 - p1, e) * inv_len) * m1
                + (dot_x4(p3 - p1, e) * inv_len) * m2);
            let g1 = (dot_x4(p2 - p0, e) * inv_len) * m1
                + (dot_x4(p3 - p0, e) * inv_len) * m2;

            let denominator = w0 * dot_x4(g0, g0)
                + w1 * dot_x4(g1, g1)
                + w2 * dot_x4(g2, g2)
                + w3 * dot_x4(g3, g3)
                + batch.compliance * inv_h2;
            let solvable = valid & denominator.gt_splat(1.0e-12);
            let mut lambda = -c / denominator.replacing(1.0, (!solvable));
            lambda = lambda.replacing(0.0, (!solvable));

            (p0 + (w0 * lambda) * g0).scatter(x, batch.v0, batch.lane_count);
            (p1 + (w1 * lambda) * g1).scatter(x, batch.v1, batch.lane_count);
            (p2 + (w2 * lambda) * g2).scatter(x, batch.v2, batch.lane_count);
            (p3 + (w3 * lambda) * g3).scatter(x, batch.v3, batch.lane_count);
        }
    }

    pub fn solve_volume_batches(&self, x: &mut [Vec3], w: &[f32], inv_h2: f32) {
        for batch in self.volume_batches.iter() {
            let x0 = Vec3X4::gather(x, batch.v0);
            let x1 = Vec3X4::gather(x, batch.v1);
            let x2 = Vec3X4::gather(x, batch.v2);
            let x3 = Vec3X4::gather(x, batch.v3);
            let w0 = gather_x4(w, batch.v0);
            let w1 = gather_x4(w, batch.v1);
            let w2 = gather_x4(w, batch.v2);
            let w3 = gather_x4(w, batch.v3);
            let e1 = x1 - x0;
            let e2 = x2 - x0;
            let e3 = x3 - x0;
            let g1 = cross_x4(e2, e3);
            let g2 = cross_x4(e3, e1);
            let g3 = cross_x4(e1, e2);
            let g0 = -(g1 + g2 + g3);
            let c = dot_x4(e1, g1) - batch.six_rest_volume;
            let denominator = w0 * dot_x4(g0, g0)
                + w1 * dot_x4(g1, g1)
                + w2 * dot_x4(g2, g2)
                + w3 * dot_x4(g3, g3)
                + batch.compliance * inv_h2;
            let solvable = denominator.gt_splat(1.0e-12);
            let mut lambda = -c / denominator.replacing(1.0, (!solvable));
            lambda = lambda.replacing(0.0, (!solvable));
            (x0 + (w0 * lambda) * g0).scatter(x, batch.v0, batch.lane_count);
            (x1 + (w1 * lambda) * g1).scatter(x, batch.v1, batch.lane_count);
            (x2 + (w2 * lambda) * g2).scatter(x, batch.v2, batch.lane_count);
            (x3 + (w3 * lambda) * g3).scatter(x, batch.v3, batch.lane_count);
        }
    }
}
