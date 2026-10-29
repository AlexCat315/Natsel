//! Path joint (Jolt's PathConstraint). A point on body B slides along a Hermite spline fixed in
//! body A. Perpendicular to the path the point is held rigidly; along it the joint can be free,
//! carry friction, a velocity motor, or a position motor that drives the point to a target
//! fraction. At the ends of an open path the point stops. The rotation of body B can be free,
//! hinged about one of the path axes, aligned with the path frame, or fixed relative to body A.
//!
//! The path frame at a fraction is (tangent, normal, binormal) = (x, y, z). The path lives in body
//! A's `local_frame_a`; the attachment point on body B is `local_frame_b.p`. Rotation constraints
//! take the current orientation of body B relative to the path frame as their reference.
//!
//! Translated from `Sources/Natsel/Simulation/PathJoint.swift`.

use std::sync::Arc;

use crate::base::constants as b3;
use crate::collision::internal_math::{Matrix2, solve2};
use crate::math::functions::{
    abs_float, add_mm, clamp_float, cross, distance_squared, dot, get_length_and_normalize,
    inv_mul_quat, inv_rotate_vector, is_valid_vec3, lerp, length, length_squared, make_matrix_from_quat,
    make_quat_from_matrix, max_float, min_float, min_vec3, max_vec3, mul_add, mul_mv, mul_quat,
    mul_world_transforms, negate_quat, normalize, normalize_quat, rotate_vector, solve3, sub_pos,
    transform_world_point, inv_transform_world_point, offset_pos,
};
use crate::math::types::{Matrix3, Pos, Quat, Transform, Vec2, Vec3, WorldTransform};
use crate::simulation::six_dof_joint::quaternion_jacobian;
use crate::simulation::solver_integration::{Softness, StepContext, make_soft};
use crate::simulation::world_runtime::{
    Body, BodyFlags, BodySim, BodyState, JointSim, SetType, World,
};
use crate::types::debug_draw::{HexColor, debug_colors};
use crate::types::ids::JointID;
use crate::types::joint_defs::{JointDef, JointType};

// MARK: - Path data

/// One path control point: a position, a Hermite tangent, and an up direction.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct PathPoint {
    pub position: Vec3,
    /// Hermite tangent. Its length sets how far the curve leaves the point in this direction.
    pub tangent: Vec3,
    /// Up direction of the path frame at this point; made perpendicular to the tangent.
    pub normal: Vec3,
}

impl PathPoint {
    #[inline]
    pub fn new(position: Vec3, tangent: Vec3, normal: Vec3) -> Self {
        Self {
            position,
            tangent,
            normal,
        }
    }
}

/// One cubic segment, `p(t) = a + t (b + t (c + t d))` for `t` in `[0, 1]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub(crate) struct PathSegment {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub d: Vec3,
    pub normal0: Vec3,
    pub normal1: Vec3,
    /// Bounds of the Bezier control polygon, which contains the curve.
    pub lower: Vec3,
    pub upper: Vec3,
    /// Arc length before this segment, and of this segment.
    pub start_length: f32,
    pub segment_length: f32,
}

impl PathSegment {
    #[inline(always)]
    fn position(&self, t: f32) -> Vec3 {
        self.a + t * (self.b + t * (self.c + t * self.d))
    }

    #[inline(always)]
    fn derivative(&self, t: f32) -> Vec3 {
        self.b + t * (2.0 * self.c + (3.0 * t) * self.d)
    }

    #[inline(always)]
    fn second_derivative(&self, t: f32) -> Vec3 {
        2.0 * self.c + (6.0 * t) * self.d
    }

    /// Arc length from 0 to t, five point Gauss-Legendre.
    fn arc_length(&self, t: f32) -> f32 {
        let nodes: [f32; 5] = [0.0, -0.5384693101, 0.5384693101, -0.9061798459, 0.9061798459];
        let weights: [f32; 5] =
            [0.5688888889, 0.4786286705, 0.4786286705, 0.2369268851, 0.2369268851];
        let half = 0.5 * t;
        let mut sum = weights[0] * length(self.derivative(half + half * nodes[0]));
        sum += weights[1] * length(self.derivative(half + half * nodes[1]));
        sum += weights[2] * length(self.derivative(half + half * nodes[2]));
        sum += weights[3] * length(self.derivative(half + half * nodes[3]));
        sum += weights[4] * length(self.derivative(half + half * nodes[4]));
        half * sum
    }

    #[inline(always)]
    fn bounds_distance_squared(&self, p: Vec3) -> f32 {
        let dx = max_float(max_float(self.lower.x - p.x, p.x - self.upper.x), 0.0);
        let dy = max_float(max_float(self.lower.y - p.y, p.y - self.upper.y), 0.0);
        let dz = max_float(max_float(self.lower.z - p.z, p.z - self.upper.z), 0.0);
        dx * dx + dy * dy + dz * dz
    }

    /// Closest parameter to `p`: the best of nine samples, refined by Newton steps on the squared
    /// distance inside the neighboring sample intervals.
    fn closest(&self, p: Vec3) -> (f32, f32) {
        let sample_count: usize = 8;
        let mut best_t: f32 = 0.0;
        let mut best = distance_squared(self.position(0.0), p);
        for k in 1..=sample_count {
            let t = k as f32 / sample_count as f32;
            let d = distance_squared(self.position(t), p);
            if d < best {
                best = d;
                best_t = t;
            }
        }
        let lower_t = max_float(best_t - 1.0 / sample_count as f32, 0.0);
        let upper_t = min_float(best_t + 1.0 / sample_count as f32, 1.0);
        let mut t = best_t;
        for _ in 0..4 {
            let e = self.position(t) - p;
            let d1 = self.derivative(t);
            let g = dot(e, d1);
            let hessian = dot(d1, d1) + dot(e, self.second_derivative(t));
            if hessian <= 1.0e-12 {
                break;
            }
            t = clamp_float(t - g / hessian, lower_t, upper_t);
        }
        let refined = distance_squared(self.position(t), p);
        if refined < best { (t, refined) } else { (best_t, best) }
    }
}

/// Plain path data the solver reads on any thread. Points into a `ConstraintPath`'s storage, which
/// the world keeps alive while a joint uses it.
///
/// The `segments` pointer refers into the [`ConstraintPath`] that built this geometry. Every
/// path-joint slot's `PathJointData` copy is kept in `World::path_joint_store` alongside an
/// `Arc` clone of the owning path in `World::path_joint_paths`, so the pointee outlives every
/// copy that references it, mirroring Swift's class-reference lifetime.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct PathGeometry {
    pub segments: Option<*const PathSegment>,
    pub count: i32,
    pub looping: bool,
    pub total_length: f32,
}

// SAFETY: mirrors Swift's `@unchecked Sendable` on `ConstraintPath`. The `segments` pointer is
// read-only after construction and the `World` holds an `Arc` to the owning `ConstraintPath` for
// as long as any stored `PathJointData` references it, so sharing it across solver threads is
// sound exactly as in the Swift original.
unsafe impl Send for PathGeometry {}
unsafe impl Sync for PathGeometry {}

impl Default for PathGeometry {
    /// Swift `init() {}`: every field keeps its declared default.
    #[inline]
    fn default() -> Self {
        Self {
            segments: None,
            count: 0,
            looping: false,
            total_length: 0.0,
        }
    }
}

impl PathGeometry {
    #[inline(always)]
    fn max_fraction(&self) -> f32 {
        self.count as f32
    }

    /// Wrap (looping) or clamp (open) a fraction into `[0, max_fraction]`.
    #[inline(always)]
    fn wrap_fraction(&self, fraction: f32) -> f32 {
        let max_f = self.max_fraction();
        if self.looping {
            // Swift `truncatingRemainder(dividingBy:)`: IEEE remainder, sign of the dividend.
            let mut f = fraction % max_f;
            if f < 0.0 {
                f += max_f;
            }
            if f < max_f { f } else { 0.0 }
        } else {
            clamp_float(fraction, 0.0, max_f)
        }
    }

    #[inline(always)]
    fn locate(&self, fraction: f32) -> (usize, f32) {
        let f = self.wrap_fraction(fraction);
        let index = crate::math::functions::min_int(f as i32, self.count - 1) as usize;
        (index, f - index as f32)
    }

    /// Position and orthonormal frame (tangent, normal, binormal) at a fraction.
    fn evaluate(&self, fraction: f32) -> (Vec3, Vec3, Vec3, Vec3) {
        let segments = match self.segments {
            Some(segments) if self.count > 0 => segments,
            _ => {
                return (Vec3::default(), Vec3::AXIS_X, Vec3::AXIS_Y, Vec3::AXIS_Z);
            }
        };
        let (index, t) = self.locate(fraction);
        // SAFETY: `index` is in `[0, count)` by `locate`, and the pointer refers to the
        // read-only storage of the owning `ConstraintPath` (see the type-level safety comment).
        let segment = unsafe { *segments.add(index) };
        let position = segment.position(t);
        let tangent = segment.derivative(t);
        let (mut len, mut tangent) = get_length_and_normalize(tangent);
        if len < 1.0e-9 {
            // Zero tangent at a cusp: use the chord of the segment.
            tangent = normalize(segment.position(1.0) - segment.position(0.0));
            if length_squared(tangent) == 0.0 {
                tangent = Vec3::AXIS_X;
            }
        }
        let up = lerp(segment.normal0, segment.normal1, t);
        let (len_b, binormal_b) = get_length_and_normalize(cross(tangent, up));
        len = len_b;
        let mut binormal = binormal_b;
        if len < 1.0e-6 {
            binormal = normalize(cross(tangent, perpendicular(tangent)));
        }
        let normal = cross(binormal, tangent);
        (position, tangent, normal, binormal)
    }

    /// Arc length from the start of the path to a fraction.
    fn arc_length(&self, fraction: f32) -> f32 {
        let segments = match self.segments {
            Some(segments) if self.count > 0 => segments,
            _ => return 0.0,
        };
        let (index, t) = self.locate(fraction);
        // SAFETY: as in [`Self::evaluate`].
        let segment = unsafe { *segments.add(index) };
        segment.start_length + segment.arc_length(t)
    }

    /// The fraction of the closest point on the path. Starts at the hint's segment so the bounds of
    /// most other segments reject them, and keeps the hint's segment on ties.
    fn closest_fraction(&self, p: Vec3, hint: f32) -> f32 {
        let segments = match self.segments {
            Some(segments) if self.count > 0 => segments,
            _ => return 0.0,
        };
        let n = self.count as usize;
        let hint_index = self.locate(hint).0;
        // SAFETY: `hint_index` is in `[0, count)` by `locate`.
        let (mut best_t, mut best) = unsafe { (*segments.add(hint_index)).closest(p) };
        let mut best_index = hint_index;
        for k in 1..std::cmp::max(n, 1) {
            let mut index = hint_index + k;
            if index >= n {
                index -= n;
            }
            // SAFETY: `index` is in `[0, count)` by the wrap above.
            let segment = unsafe { *segments.add(index) };
            if !(segment.bounds_distance_squared(p) < best) {
                continue;
            }
            let (t, d) = segment.closest(p);
            if d < best {
                best = d;
                best_t = t;
                best_index = index;
            }
        }
        self.wrap_fraction(best_index as f32 + best_t)
    }
}

#[inline(always)]
fn perpendicular(v: Vec3) -> Vec3 {
    if abs_float(v.x) < 0.57735 { Vec3::new(0.0, v.z, -v.y) } else { Vec3::new(v.y, -v.x, 0.0) }
}

/// A cubic Hermite path (Jolt's PathConstraintPathHermite). Fractions run from 0 at the first
/// point to `max_fraction`: one unit per segment. A looping path joins the last point back to the
/// first.
///
/// Swift `final class`; multiple path-joint slots share one path through
/// `Arc<ConstraintPath>` (see `World::path_joint_paths`). The path is immutable after
/// construction, so no interior mutability is required — Swift never mutated it in place either.
#[derive(Debug)]
pub struct ConstraintPath {
    pub points: Vec<PathPoint>,
    pub is_looping: bool,
    /// Swift's `UnsafeMutablePointer<PathSegment>` storage. Owned as a `Vec` that is never
    /// mutated or reallocated after `new`, so `geometry.segments` stays valid for the life of
    /// this object (moving the `Vec` header does not move the heap buffer).
    segments: Vec<PathSegment>,
    pub(crate) geometry: PathGeometry,
}

impl ConstraintPath {
    /// Needs two points, or three for a loop. Swift `init?` returning nil becomes `Option`.
    pub fn new(points: Vec<PathPoint>, is_looping: bool) -> Option<ConstraintPath> {
        if points.len() < if is_looping { 3 } else { 2 } {
            return None;
        }
        if !points.iter().all(|point| {
            is_valid_vec3(point.position)
                && is_valid_vec3(point.tangent)
                && is_valid_vec3(point.normal)
        }) {
            return None;
        }
        let count = if is_looping { points.len() } else { points.len() - 1 };
        let mut segments: Vec<PathSegment> = Vec::with_capacity(count);
        let mut start_length: f32 = 0.0;
        for i in 0..count {
            let p0 = points[i];
            let p1 = points[(i + 1) % points.len()];
            let mut segment = PathSegment::default();
            segment.a = p0.position;
            segment.b = p0.tangent;
            segment.c = 3.0 * (p1.position - p0.position) - 2.0 * p0.tangent - p1.tangent;
            segment.d = 2.0 * (p0.position - p1.position) + p0.tangent + p1.tangent;
            segment.normal0 = p0.normal;
            segment.normal1 = p1.normal;
            let control1 = p0.position + (1.0 / 3.0) * p0.tangent;
            let control2 = p1.position - (1.0 / 3.0) * p1.tangent;
            segment.lower =
                min_vec3(min_vec3(p0.position, p1.position), min_vec3(control1, control2));
            segment.upper =
                max_vec3(max_vec3(p0.position, p1.position), max_vec3(control1, control2));
            segment.start_length = start_length;
            segment.segment_length = segment.arc_length(1.0);
            start_length += segment.segment_length;
            // Swift `storage.advanced(by: i).initialize(to: segment)` on the reserved buffer.
            segments.push(segment);
        }
        let mut geometry = PathGeometry::default();
        geometry.segments = Some(segments.as_ptr());
        geometry.count = count as i32;
        geometry.looping = is_looping;
        geometry.total_length = start_length;
        Some(ConstraintPath {
            points,
            is_looping,
            segments,
            geometry,
        })
    }

    /// A smooth path through `positions` with Catmull-Rom tangents and a constant up direction.
    /// Swift `convenience init?(through:up:isLooping:)` with defaults `up = .axisY`,
    /// `isLooping = false` — Rust has no default arguments, pass them explicitly.
    pub fn through(positions: &[Vec3], up: Vec3, is_looping: bool) -> Option<ConstraintPath> {
        let n = positions.len();
        if n < 2 {
            return None;
        }
        let mut points: Vec<PathPoint> = Vec::with_capacity(n);
        for i in 0..n {
            let previous = if is_looping {
                positions[(i + n - 1) % n]
            } else {
                positions[if i > 0 { i - 1 } else { 0 }]
            };
            let next = if is_looping {
                positions[(i + 1) % n]
            } else {
                positions[if i + 1 < n { i + 1 } else { n - 1 }]
            };
            let scale: f32 =
                if is_looping || (i > 0 && i < n - 1) { 0.5 } else { 1.0 };
            points.push(PathPoint::new(
                positions[i],
                scale * (next - previous),
                up,
            ));
        }
        Self::new(points, is_looping)
    }

    // Swift `deinit` (deinitialize + deallocate of the raw storage) is handled by `Vec` drop.

    /// Largest fraction; the number of segments.
    #[inline]
    pub fn max_fraction(&self) -> f32 {
        self.geometry.max_fraction()
    }

    /// Total arc length.
    #[inline]
    pub fn length(&self) -> f32 {
        self.geometry.total_length
    }

    /// Position and frame at a fraction, in the path's space.
    #[inline]
    pub fn point_at(&self, fraction: f32) -> (Vec3, Vec3, Vec3, Vec3) {
        self.geometry.evaluate(fraction)
    }

    /// Fraction of the closest point on the path to `position`. Swift default `hint = 0`.
    #[inline]
    pub fn closest_fraction_to(&self, position: Vec3, hint: f32) -> f32 {
        self.geometry.closest_fraction(position, hint)
    }

    /// Arc length from the start to a fraction.
    #[inline]
    pub fn arc_length_at(&self, fraction: f32) -> f32 {
        self.geometry.arc_length(fraction)
    }
}

/// Rotation mode of a path joint.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PathRotationConstraint {
    /// Body B rotates freely.
    #[default]
    Free,
    /// Body B only rotates about the path tangent.
    ConstrainAroundTangent,
    /// Body B only rotates about the path normal.
    ConstrainAroundNormal,
    /// Body B only rotates about the path binormal.
    ConstrainAroundBinormal,
    /// Body B follows the path frame, like a roller coaster car.
    ConstrainToPath,
    /// The rotation of body B relative to body A is fixed.
    FullyConstrained,
}

/// Motor mode along the path.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum PathMotorMode {
    #[default]
    Off,
    /// Drive the speed along the path to `target_velocity` (m/s, positive toward larger
    /// fractions).
    Velocity,
    /// Spring the point toward `target_fraction`, measured by arc length.
    Position,
}
/// Definition of a path joint.
#[derive(Clone, Debug)]
pub struct PathJointDef {
    /// `local_frame_a` places the path in body A; `local_frame_b.p` is the point on body B that
    /// follows it. The rotation of `local_frame_b` is ignored: the current orientation is the
    /// reference.
    pub base: JointDef,
    pub path: Option<Arc<ConstraintPath>>,
    /// Start hint for the closest point search.
    pub path_fraction: f32,
    pub rotation_constraint: PathRotationConstraint,
    /// Force (N) resisting motion along the path while the motor is off.
    pub max_friction_force: f32,
    pub motor_mode: PathMotorMode,
    pub target_velocity: f32,
    pub target_fraction: f32,
    /// Largest motor force (N).
    pub max_motor_force: f32,
    /// Spring of the position motor.
    pub motor_hertz: f32,
    pub motor_damping_ratio: f32,
}

impl Default for PathJointDef {
    /// Swift `public init() {}` with the declared field defaults.
    #[inline]
    fn default() -> Self {
        Self {
            base: JointDef::default(),
            path: None,
            path_fraction: 0.0,
            rotation_constraint: PathRotationConstraint::Free,
            max_friction_force: 0.0,
            motor_mode: PathMotorMode::Off,
            target_velocity: 0.0,
            target_fraction: 0.0,
            max_motor_force: f32::MAX,
            motor_hertz: 2.0,
            motor_damping_ratio: 1.0,
        }
    }
}

impl PathJointDef {
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// Solver data kept in the joint record.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub(crate) struct PathJointSim {
    pub slot: i32,
    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,
}

impl Default for PathJointSim {
    /// Swift `init() {}`: `slot`/`indexA`/`indexB` start at `B3.nullIndex`.
    #[inline]
    fn default() -> Self {
        Self {
            slot: b3::NULL_INDEX,
            index_a: b3::NULL_INDEX,
            index_b: b3::NULL_INDEX,
            frame_a: Transform::default(),
            frame_b: Transform::default(),
            delta_center: Vec3::default(),
        }
    }
}

/// Persistent joint state, stored in `World::path_joint_store` by slot.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub(crate) struct PathJointData {
    pub geometry: PathGeometry,
    pub rotation: PathRotationConstraint,
    /// Fraction of the closest point, updated by every solve.
    pub fraction: f32,
    /// Path frame at the reference pose, relative to the path space (fully constrained).
    pub target_rotation: Quat,

    pub motor_mode: PathMotorMode,
    pub target_velocity: f32,
    pub target_fraction: f32,
    pub max_motor_force: f32,
    pub motor_hertz: f32,
    pub motor_damping_ratio: f32,
    pub max_friction: f32,
    pub motor_softness: Softness,

    pub perpendicular_impulse: Vec2,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub motor_impulse: f32,
    pub friction_impulse: f32,
    pub angular_impulse: Vec3,

    /// World impulses of the last solve, for reactions and events.
    pub linear_impulse_world: Vec3,
    pub angular_impulse_world: Vec3,
}

impl PathJointData {
    /// Swift `init() {}`: `targetRotation = Quat.identity`, `motorHertz = 2`,
    /// `motorDampingRatio = 1`; everything else zero / default.
    #[inline]
    fn new_init() -> Self {
        Self {
            geometry: PathGeometry::default(),
            rotation: PathRotationConstraint::Free,
            fraction: 0.0,
            target_rotation: Quat::IDENTITY,
            motor_mode: PathMotorMode::Off,
            target_velocity: 0.0,
            target_fraction: 0.0,
            max_motor_force: 0.0,
            motor_hertz: 2.0,
            motor_damping_ratio: 1.0,
            max_friction: 0.0,
            motor_softness: Softness::new(),
            perpendicular_impulse: Vec2::default(),
            lower_impulse: 0.0,
            upper_impulse: 0.0,
            motor_impulse: 0.0,
            friction_impulse: 0.0,
            angular_impulse: Vec3::default(),
            linear_impulse_world: Vec3::default(),
            angular_impulse_world: Vec3::default(),
        }
    }
}

impl Default for PathJointData {
    /// The `init() {}` defaults are *not* all-zero, so `Default` must match `new_init`, exactly
    /// as Swift's `PathJointData()` does at slot release.
    #[inline]
    fn default() -> Self {
        Self::new_init()
    }
}

/// Frame of a path joint at the current substep.
#[derive(Clone, Copy)]
struct PathFrame {
    quat_a: Quat,
    quat_b: Quat,
    /// Path point relative to body A's center, and attachment point relative to body B's center.
    r_a: Vec3,
    r_b: Vec3,
    /// From the path point to the attachment point.
    u: Vec3,
    tangent: Vec3,
    normal: Vec3,
    binormal: Vec3,
    fraction: f32,
}

#[inline(always)]
fn path_axis(f: PathFrame, i: usize) -> Vec3 {
    if i == 0 { f.tangent } else if i == 1 { f.normal } else { f.binormal }
}

impl World {
    // MARK: Storage

    fn allocate_path_data(
        &mut self,
        data: PathJointData,
        path: Arc<ConstraintPath>,
    ) -> i32 {
        let slot = self.path_joint_pool.alloc();
        if slot as usize == self.path_joint_store.count() {
            self.path_joint_store.append(data);
            self.path_joint_paths.push(Some(path));
        } else {
            self.path_joint_store.set(slot as usize, data);
            self.path_joint_paths[slot as usize] = Some(path);
        }
        slot
    }

    pub(crate) fn release_path_data(&mut self, slot: i32) {
        if !(slot >= 0 && (slot as usize) < self.path_joint_store.count()) {
            return;
        }
        self.path_joint_store.set(slot as usize, PathJointData::default());
        self.path_joint_paths[slot as usize] = None;
        self.path_joint_pool.free(slot);
    }

    /// World transform of the path and the attachment point on body B.
    fn path_pose(
        &self,
        body_a: i32,
        body_b: i32,
        frame_a: Transform,
        anchor_b: Vec3,
    ) -> (WorldTransform, Pos) {
        let path_transform = mul_world_transforms(self.body_transform_quick(body_a), frame_a);
        (
            path_transform,
            transform_world_point(self.body_transform_quick(body_b), anchor_b),
        )
    }

    // MARK: Creation

    pub fn create_path_joint(&mut self, def: &PathJointDef) -> JointID {
        let path = match def.path.as_ref() {
            Some(path) => path.clone(),
            None => return JointID::NULL,
        };
        if !(self.is_valid_body(def.base.body_id_a)
            && self.is_valid_body(def.base.body_id_b)
            && def.base.body_id_a != def.base.body_id_b)
        {
            return JointID::NULL;
        }

        let mut base = def.base.clone();
        let body_a = base.body_id_a.index1 - 1;
        let body_b = base.body_id_b.index1 - 1;
        let (path_transform, anchor) =
            self.path_pose(body_a, body_b, base.local_frame_a, base.local_frame_b.p);
        let local = inv_transform_world_point(path_transform, anchor);
        let fraction = path.geometry.closest_fraction(local, def.path_fraction);
        let (_frame_position, frame_tangent, frame_normal, frame_binormal) =
            path.geometry.evaluate(fraction);
        let local_path_q =
            make_quat_from_matrix(Matrix3::new(frame_tangent, frame_normal, frame_binormal));
        let world_path_q = mul_quat(path_transform.q, local_path_q);
        // Body B's frame is the path frame at creation, so every rotation constraint starts
        // satisfied.
        base.local_frame_b.q = normalize_quat(inv_mul_quat(
            self.body_transform_quick(body_b).q,
            world_path_q,
        ));

        let mut data = PathJointData::new_init();
        data.geometry = path.geometry;
        data.rotation = def.rotation_constraint;
        data.fraction = fraction;
        data.target_rotation = local_path_q;
        data.motor_mode = def.motor_mode;
        data.target_velocity = def.target_velocity;
        data.target_fraction = def.target_fraction;
        data.max_motor_force = max_float(def.max_motor_force, 0.0);
        data.motor_hertz = def.motor_hertz;
        data.motor_damping_ratio = def.motor_damping_ratio;
        data.max_friction = max_float(def.max_friction_force, 0.0);

        let pair = self.create_joint(&base, JointType::PathJoint);
        let mut sim = pair.sim;
        let mut joint = PathJointSim::default();
        joint.slot = self.allocate_path_data(data, path);
        sim.set_path(joint);
        self.write_joint_sim(pair.joint_id, sim);
        JointID::new(
            pair.joint_id + 1,
            self.world_id,
            self.joints[pair.joint_id as usize].generation,
        )
    }

    // MARK: Prepare

    pub(crate) fn prepare_path_joint(&mut self, base: &mut JointSim, context: &StepContext) {
        let body_a: Body = self.joint_prepare_body(base.body_id_a);
        let body_b: Body = self.joint_prepare_body(base.body_id_b);
        let sim_a: BodySim = self.joint_prepare_sim(&body_a);
        let sim_b: BodySim = self.joint_prepare_sim(&body_b);
        base.inv_mass_a = sim_a.inv_mass;
        base.inv_mass_b = sim_b.inv_mass;
        base.inv_ia = sim_a.inv_inertia_world;
        base.inv_ib = sim_b.inv_inertia_world;

        let mut joint = base.path();
        joint.index_a = if body_a.set_index == SetType::AwakeSet.raw_value() {
            body_a.local_index
        } else {
            b3::NULL_INDEX
        };
        joint.index_b = if body_b.set_index == SetType::AwakeSet.raw_value() {
            body_b.local_index
        } else {
            b3::NULL_INDEX
        };
        joint.frame_a.q = mul_quat(sim_a.transform.q, base.local_frame_a.q);
        joint.frame_a.p = rotate_vector(sim_a.transform.q, base.local_frame_a.p - sim_a.local_center);
        joint.frame_b.q = mul_quat(sim_b.transform.q, base.local_frame_b.q);
        joint.frame_b.p = rotate_vector(sim_b.transform.q, base.local_frame_b.p - sim_b.local_center);
        joint.delta_center = sub_pos(sim_b.center, sim_a.center);
        base.set_path(joint);

        let slot = joint.slot as usize;
        let mut data = self.path_joint_store.get(slot);
        data.motor_softness = make_soft(data.motor_hertz, data.motor_damping_ratio, context.h);
        if !context.enable_warm_starting {
            data.perpendicular_impulse = Vec2::default();
            data.lower_impulse = 0.0;
            data.upper_impulse = 0.0;
            data.motor_impulse = 0.0;
            data.friction_impulse = 0.0;
            data.angular_impulse = Vec3::default();
        }
        if data.motor_mode == PathMotorMode::Off {
            data.motor_impulse = 0.0;
        }
        if data.motor_mode != PathMotorMode::Off || data.max_friction <= 0.0 {
            data.friction_impulse = 0.0;
        }
        if data.rotation == PathRotationConstraint::Free {
            data.angular_impulse = Vec3::default();
        }
        self.path_joint_store.set(slot, data);
    }

    // MARK: Geometry

    #[inline(always)]
    fn path_frame(
        &self,
        joint: PathJointSim,
        data: PathJointData,
        state_a: BodyState,
        state_b: BodyState,
        search: bool,
    ) -> PathFrame {
        let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
        let quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);
        let origin = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);
        // Attachment point relative to the path origin.
        let d =
            ((state_b.delta_position - state_a.delta_position) + joint.delta_center) + (r_b - origin);
        let local = inv_rotate_vector(quat_a, d);
        let fraction = if search {
            data.geometry.closest_fraction(local, data.fraction)
        } else {
            data.fraction
        };
        let (e_position, e_tangent, e_normal, e_binormal) = data.geometry.evaluate(fraction);
        let p = rotate_vector(quat_a, e_position);
        PathFrame {
            quat_a,
            quat_b,
            r_a: origin + p,
            r_b,
            u: d - p,
            tangent: rotate_vector(quat_a, e_tangent),
            normal: rotate_vector(quat_a, e_normal),
            binormal: rotate_vector(quat_a, e_binormal),
            fraction,
        }
    }

    /// Angular Jacobians of the rotation constraint, and how many there are.
    #[inline(always)]
    fn path_rotation_rows(&self, f: PathFrame, data: PathJointData) -> (Vec3, Vec3, Vec3, Vec3, usize) {
        match data.rotation {
            PathRotationConstraint::Free => (
                Vec3::default(),
                Vec3::default(),
                Vec3::default(),
                Vec3::default(),
                0,
            ),
            PathRotationConstraint::ConstrainAroundTangent
            | PathRotationConstraint::ConstrainAroundNormal
            | PathRotationConstraint::ConstrainAroundBinormal => {
                // Keep body B's copy of the hinge axis perpendicular to the other two path axes.
                let k = if data.rotation == PathRotationConstraint::ConstrainAroundTangent {
                    0
                } else if data.rotation == PathRotationConstraint::ConstrainAroundNormal {
                    1
                } else {
                    2
                };
                let axes_b = make_matrix_from_quat(f.quat_b);
                let hinge_b = if k == 0 { axes_b.cx } else if k == 1 { axes_b.cy } else { axes_b.cz };
                let p1 = path_axis(f, (k + 1) % 3);
                let p2 = path_axis(f, (k + 2) % 3);
                (
                    cross(hinge_b, p1),
                    cross(hinge_b, p2),
                    Vec3::default(),
                    Vec3::new(dot(hinge_b, p1), dot(hinge_b, p2), 0.0),
                    2,
                )
            }
            PathRotationConstraint::ConstrainToPath | PathRotationConstraint::FullyConstrained => {
                let target = if data.rotation == PathRotationConstraint::ConstrainToPath {
                    make_quat_from_matrix(Matrix3::new(f.tangent, f.normal, f.binormal))
                } else {
                    mul_quat(f.quat_a, data.target_rotation)
                };
                let mut rel = inv_mul_quat(target, f.quat_b);
                if rel.s < 0.0 {
                    rel = negate_quat(rel);
                }
                (
                    quaternion_jacobian(target, rel, Vec3::AXIS_X),
                    quaternion_jacobian(target, rel, Vec3::AXIS_Y),
                    quaternion_jacobian(target, rel, Vec3::AXIS_Z),
                    rel.v,
                    3,
                )
            }
        }
    }

    /// Signed distance along the path from the target to the current fraction.
    #[inline(always)]
    fn path_motor_error(&self, data: PathJointData, fraction: f32) -> f32 {
        let mut error = data.geometry.arc_length(fraction)
            - data.geometry.arc_length(data.target_fraction);
        if data.geometry.looping {
            let total = data.geometry.total_length;
            if error > 0.5 * total {
                error -= total;
            } else if error < -0.5 * total {
                error += total;
            }
        }
        error
    }
    // MARK: Warm start

    pub(crate) fn warm_start_path_joint(
        &self,
        base: &mut JointSim,
        states: &mut [BodyState],
    ) {
        let joint = base.path();
        let data = self.path_joint_store.get(joint.slot as usize);
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);
        let f = self.path_frame(joint, data, state_a, state_b, false);

        let axial = data.lower_impulse - data.upper_impulse + data.motor_impulse + data.friction_impulse;
        let linear = data.perpendicular_impulse.x * f.normal
            + data.perpendicular_impulse.y * f.binormal
            + axial * f.tangent;
        let (j0, j1, j2, _c, rows) = self.path_rotation_rows(f, data);
        let mut angular = Vec3::default();
        if rows > 0 {
            angular = data.angular_impulse.x * j0
                + data.angular_impulse.y * j1
                + data.angular_impulse.z * j2;
        }

        state_a.linear_velocity = mul_sub(state_a.linear_velocity, base.inv_mass_a, linear);
        state_a.angular_velocity = state_a.angular_velocity
            - mul_mv(base.inv_ia, cross(f.r_a + f.u, linear) + angular);
        state_b.linear_velocity = mul_add(state_b.linear_velocity, base.inv_mass_b, linear);
        state_b.angular_velocity = state_b.angular_velocity
            + mul_mv(base.inv_ib, cross(f.r_b, linear) + angular);
        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
    }

    // MARK: Solve

    pub(crate) fn solve_path_joint(
        &mut self,
        base: &mut JointSim,
        context: &StepContext,
        use_bias: bool,
        states: &mut [BodyState],
    ) {
        let joint = base.path();
        let slot = joint.slot as usize;
        let mut data = self.path_joint_store.get(slot);
        if data.geometry.count <= 0 {
            return;
        }
        let mut state_a = World::state_at(states, joint.index_a);
        let mut state_b = World::state_at(states, joint.index_b);
        let m_a = base.inv_mass_a;
        let m_b = base.inv_mass_b;
        let i_a = base.inv_ia;
        let i_b = base.inv_ib;
        let h = context.h;
        let soft = base.constraint_softness;

        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;

        let f = self.path_frame(joint, data, state_a, state_b, true);
        data.fraction = f.fraction;
        let lever = f.r_a + f.u;
        let mut angular_total = Vec3::default();
        let mut linear_total = Vec3::default();

        // MARK: Rotation

        let (j0, j1, j2, c, rows) = self.path_rotation_rows(f, data);
        if rows > 0 {
            let i_sum = add_mm(i_a, i_b);
            let mut bias = Vec3::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias {
                bias = soft.bias_rate * c;
                mass_scale = soft.mass_scale;
                impulse_scale = soft.impulse_scale;
            }
            let w_rel = w_b - w_a;
            let mut delta = Vec3::default();
            if rows == 2 {
                let g0 = mul_mv(i_sum, j0);
                let g1 = mul_mv(i_sum, j1);
                let k = Matrix2::new(
                    Vec2::new(dot(j0, g0), dot(j1, g0)),
                    Vec2::new(dot(j0, g1), dot(j1, g1)),
                );
                let rhs = Vec2::new(dot(w_rel, j0) + bias.x, dot(w_rel, j1) + bias.y);
                let solution = solve2(k, rhs);
                delta.x = -mass_scale * solution.x - impulse_scale * data.angular_impulse.x;
                delta.y = -mass_scale * solution.y - impulse_scale * data.angular_impulse.y;
            } else {
                let g0 = mul_mv(i_sum, j0);
                let g1 = mul_mv(i_sum, j1);
                let g2 = mul_mv(i_sum, j2);
                let k = Matrix3::new(
                    Vec3::new(dot(j0, g0), dot(j1, g0), dot(j2, g0)),
                    Vec3::new(dot(j0, g1), dot(j1, g1), dot(j2, g1)),
                    Vec3::new(dot(j0, g2), dot(j1, g2), dot(j2, g2)),
                );
                let cdot = Vec3::new(dot(w_rel, j0), dot(w_rel, j1), dot(w_rel, j2));
                delta = (-mass_scale) * solve3(k, cdot + bias)
                    - impulse_scale * data.angular_impulse;
            }
            data.angular_impulse = data.angular_impulse + delta;
            let angular = delta.x * j0 + delta.y * j1 + delta.z * j2;
            w_a = w_a - mul_mv(i_a, angular);
            w_b = w_b + mul_mv(i_b, angular);
            angular_total = data.angular_impulse.x * j0
                + data.angular_impulse.y * j1
                + data.angular_impulse.z * j2;
        } else {
            data.angular_impulse = Vec3::default();
        }

        // MARK: Along the path

        let t = f.tangent;
        let s_at = cross(lever, t);
        let s_bt = cross(f.r_b, t);
        let kt = m_a + m_b + dot(s_at, mul_mv(i_a, s_at)) + dot(s_bt, mul_mv(i_b, s_bt));
        if kt > 0.0 {
            let mass = 1.0 / kt;
            // Swift's nested `cdot()` / `apply(_:)` functions capture the velocity locals by
            // reference; the macros reproduce that aliasing at each call site.
            macro_rules! cdot {
                () => {
                    dot(v_b - v_a, t) + dot(w_b, s_bt) - dot(w_a, s_at)
                };
            }
            macro_rules! apply {
                ($impulse:expr) => {{
                    let impulse = $impulse;
                    v_a = mul_sub(v_a, m_a * impulse, t);
                    w_a = w_a - mul_mv(i_a, impulse * s_at);
                    v_b = mul_add(v_b, m_b * impulse, t);
                    w_b = w_b + mul_mv(i_b, impulse * s_bt);
                }};
            }

            match data.motor_mode {
                PathMotorMode::Velocity => {
                    let max_impulse = data.max_motor_force * h;
                    let old = data.motor_impulse;
                    data.motor_impulse = clamp_float(
                        old - mass * (cdot!() - data.target_velocity),
                        -max_impulse,
                        max_impulse,
                    );
                    apply!(data.motor_impulse - old);
                }
                PathMotorMode::Position => {
                    let ms = data.motor_softness;
                    let error = self.path_motor_error(data, f.fraction) + dot(f.u, t);
                    let max_impulse = data.max_motor_force * h;
                    let old = data.motor_impulse;
                    let delta = -ms.mass_scale * mass * (cdot!() + ms.bias_rate * error)
                        - ms.impulse_scale * old;
                    data.motor_impulse =
                        clamp_float(old + delta, -max_impulse, max_impulse);
                    apply!(data.motor_impulse - old);
                }
                PathMotorMode::Off => {
                    if data.max_friction > 0.0 {
                        let max_impulse = data.max_friction * h;
                        let old = data.friction_impulse;
                        data.friction_impulse = clamp_float(
                            old - mass * cdot!(),
                            -max_impulse,
                            max_impulse,
                        );
                        apply!(data.friction_impulse - old);
                    }
                }
            }

            // End stops of an open path: the point may not pass the first or last point.
            let max_f = data.geometry.max_fraction();
            let at_start = data.geometry.looping == false && f.fraction <= 0.0;
            let at_end = data.geometry.looping == false && f.fraction >= max_f;
            let stop_terms = |separation: f32| -> (f32, f32, f32) {
                if separation > 0.0 {
                    return (separation * context.inv_h, 1.0, 0.0);
                }
                if use_bias {
                    return (soft.bias_rate * separation, soft.mass_scale, soft.impulse_scale);
                }
                (0.0, 1.0, 0.0)
            };
            if at_start {
                let (bias, mass_scale, impulse_scale) = stop_terms(dot(f.u, t));
                let old = data.lower_impulse;
                let delta = max_float(
                    -mass_scale * mass * (cdot!() + bias) - impulse_scale * old,
                    -old,
                );
                data.lower_impulse = old + delta;
                apply!(delta);
            } else {
                data.lower_impulse = 0.0;
            }
            if at_end {
                let (bias, mass_scale, impulse_scale) = stop_terms(-dot(f.u, t));
                let old = data.upper_impulse;
                let delta = max_float(
                    -mass_scale * mass * (-cdot!() + bias) - impulse_scale * old,
                    -old,
                );
                data.upper_impulse = old + delta;
                apply!(-delta);
            } else {
                data.upper_impulse = 0.0;
            }
        }

        // MARK: Perpendicular to the path

        {
            let n = f.normal;
            let b = f.binormal;
            let s_an = cross(lever, n);
            let s_ab = cross(lever, b);
            let s_bn = cross(f.r_b, n);
            let s_bb = cross(f.r_b, b);
            let g_an = mul_mv(i_a, s_an);
            let g_bn = mul_mv(i_b, s_bn);
            let g_ab = mul_mv(i_a, s_ab);
            let g_bb = mul_mv(i_b, s_bb);
            let knn = m_a + m_b + dot(s_an, g_an) + dot(s_bn, g_bn);
            let kbb = m_a + m_b + dot(s_ab, g_ab) + dot(s_bb, g_bb);
            let knb = dot(s_an, g_ab) + dot(s_bn, g_bb);
            let k = Matrix2::new(Vec2::new(knn, knb), Vec2::new(knb, kbb));
            let cdot = Vec2::new(
                dot(v_b - v_a, n) + dot(w_b, s_bn) - dot(w_a, s_an),
                dot(v_b - v_a, b) + dot(w_b, s_bb) - dot(w_a, s_ab),
            );
            let mut bias = Vec2::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias {
                bias = Vec2::new(soft.bias_rate * dot(f.u, n), soft.bias_rate * dot(f.u, b));
                mass_scale = soft.mass_scale;
                impulse_scale = soft.impulse_scale;
            }
            let solution = solve2(k, Vec2::new(cdot.x + bias.x, cdot.y + bias.y));
            let delta = Vec2::new(
                -mass_scale * solution.x - impulse_scale * data.perpendicular_impulse.x,
                -mass_scale * solution.y - impulse_scale * data.perpendicular_impulse.y,
            );
            data.perpendicular_impulse = Vec2::new(
                data.perpendicular_impulse.x + delta.x,
                data.perpendicular_impulse.y + delta.y,
            );
            let impulse = delta.x * n + delta.y * b;
            v_a = mul_sub(v_a, m_a, impulse);
            w_a = w_a - mul_mv(i_a, delta.x * s_an + delta.y * s_ab);
            v_b = mul_add(v_b, m_b, impulse);
            w_b = w_b + mul_mv(i_b, delta.x * s_bn + delta.y * s_bb);
        }

        let axial = data.lower_impulse - data.upper_impulse + data.motor_impulse + data.friction_impulse;
        linear_total = data.perpendicular_impulse.x * f.normal
            + data.perpendicular_impulse.y * f.binormal
            + axial * t;
        data.linear_impulse_world = linear_total;
        data.angular_impulse_world = angular_total;

        if (state_a.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_a.linear_velocity = v_a;
            state_a.angular_velocity = w_a;
        }
        if (state_b.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_b.linear_velocity = v_b;
            state_b.angular_velocity = w_b;
        }
        World::set_state_at(states, joint.index_a, state_a);
        World::set_state_at(states, joint.index_b, state_b);
        self.path_joint_store.set(slot, data);
    }

    // MARK: Reactions

    pub(crate) fn path_joint_impulses(&self, sim: JointSim) -> (Vec3, Vec3) {
        let slot = sim.path().slot;
        if !(slot >= 0 && (slot as usize) < self.path_joint_store.count()) {
            return (Vec3::default(), Vec3::default());
        }
        let slot = slot as usize;
        (
            self.path_joint_store.get(slot).linear_impulse_world,
            self.path_joint_store.get(slot).angular_impulse_world,
        )
    }

    // MARK: Accessors

    fn path_slot(&self, joint_id: JointID) -> Option<usize> {
        let index = joint_id.index1 - 1;
        if !(joint_id.world0 == self.world_id
            && index >= 0
            && (index as usize) < self.joints.len())
        {
            return None;
        }
        let joint = &self.joints[index as usize];
        if !(joint.joint_id != b3::NULL_INDEX
            && joint.generation == joint_id.generation
            && joint.type_ == JointType::PathJoint)
        {
            return None;
        }
        let slot = self.read_joint_sim(index).path().slot;
        if slot >= 0 && (slot as usize) < self.path_joint_store.count() {
            Some(slot as usize)
        } else {
            None
        }
    }

    /// Fraction of the closest point on the path after the last step.
    pub fn path_joint_fraction(&self, joint_id: JointID) -> Option<f32> {
        self.path_slot(joint_id).map(|slot| self.path_joint_store.get(slot).fraction)
    }

    pub fn path_joint_path(&self, joint_id: JointID) -> Option<Arc<ConstraintPath>> {
        self.path_slot(joint_id)
            .and_then(|slot| self.path_joint_paths[slot].clone())
    }

    /// Replace the path. `fraction` is the search hint for the new path. Swift default
    /// `fraction = 0` must be passed explicitly.
    pub fn set_path_joint_path(
        &mut self,
        joint_id: JointID,
        path: Arc<ConstraintPath>,
        fraction: f32,
    ) -> bool {
        let Some(slot) = self.path_slot(joint_id) else {
            return false;
        };
        self.path_joint_paths[slot] = Some(path.clone());
        let geometry = path.geometry;
        let wrapped = geometry.wrap_fraction(fraction);
        let mut data = self.path_joint_store.get(slot);
        data.geometry = geometry;
        data.fraction = wrapped;
        data.lower_impulse = 0.0;
        data.upper_impulse = 0.0;
        self.path_joint_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    pub fn set_path_joint_motor(
        &mut self,
        joint_id: JointID,
        mode: PathMotorMode,
        max_force: Option<f32>,
    ) -> bool {
        let Some(slot) = self.path_slot(joint_id) else {
            return false;
        };
        let mut data = self.path_joint_store.get(slot);
        data.motor_mode = mode;
        if let Some(max_force) = max_force {
            data.max_motor_force = max_float(max_force, 0.0);
        }
        self.path_joint_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    pub fn set_path_joint_target_velocity(&mut self, joint_id: JointID, velocity: f32) -> bool {
        let Some(slot) = self.path_slot(joint_id) else {
            return false;
        };
        let mut data = self.path_joint_store.get(slot);
        data.target_velocity = velocity;
        self.path_joint_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    pub fn set_path_joint_target_fraction(&mut self, joint_id: JointID, fraction: f32) -> bool {
        let Some(slot) = self.path_slot(joint_id) else {
            return false;
        };
        let mut data = self.path_joint_store.get(slot);
        data.target_fraction = data.geometry.wrap_fraction(fraction);
        self.path_joint_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    pub fn set_path_joint_max_friction(&mut self, joint_id: JointID, force: f32) -> bool {
        let Some(slot) = self.path_slot(joint_id) else {
            return false;
        };
        let mut data = self.path_joint_store.get(slot);
        data.max_friction = max_float(force, 0.0);
        self.path_joint_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    pub fn set_path_joint_rotation_constraint(
        &mut self,
        joint_id: JointID,
        rotation: PathRotationConstraint,
    ) -> bool {
        let Some(slot) = self.path_slot(joint_id) else {
            return false;
        };
        let mut data = self.path_joint_store.get(slot);
        data.rotation = rotation;
        data.angular_impulse = Vec3::default();
        self.path_joint_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    // MARK: Debug draw

    pub(crate) fn draw_path_joint(
        &self,
        sim: JointSim,
        xf_a: WorldTransform,
        p_b: Pos,
        scale: f32,
        draw_segment: &mut dyn FnMut(Pos, Pos, HexColor),
        draw_point: Option<&mut dyn FnMut(Pos, f32, HexColor)>,
    ) {
        let slot = sim.path().slot;
        if !(slot >= 0 && (slot as usize) < self.path_joint_store.count()) {
            return;
        }
        let data = self.path_joint_store.get(slot as usize);
        if data.geometry.count <= 0 {
            return;
        }
        let path_transform = mul_world_transforms(xf_a, sim.local_frame_a);
        let steps = 16usize * data.geometry.count as usize;
        let mut previous =
            transform_world_point(path_transform, data.geometry.evaluate(0.0).0);
        for i in 1..=steps {
            let fraction =
                data.geometry.max_fraction() * (i as f32) / (steps as f32);
            let p = transform_world_point(
                path_transform,
                data
                    .geometry
                    .evaluate(if i == steps && data.geometry.looping { 0.0 } else { fraction })
                    .0,
            );
            draw_segment(previous, p, debug_colors::LIGHT_BLUE);
            previous = p;
        }
        let e = data.geometry.evaluate(data.fraction);
        let p = transform_world_point(path_transform, e.0);
        let length = 0.2 * scale;
        draw_segment(
            p,
            offset_pos(p, length * rotate_vector(path_transform.q, e.1)),
            debug_colors::RED,
        );
        draw_segment(
            p,
            offset_pos(p, length * rotate_vector(path_transform.q, e.2)),
            debug_colors::GREEN,
        );
        draw_segment(
            p,
            offset_pos(p, length * rotate_vector(path_transform.q, e.3)),
            debug_colors::BLUE,
        );
        draw_segment(p, p_b, debug_colors::GRAY);
        if let Some(mut draw_point) = draw_point {
            draw_point(p_b, 6.0, debug_colors::WHITE);
        }
    }
}
