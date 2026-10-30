//! Six degree of freedom joint. The joint space is body A's local frame: translations are measured
//! along its x, y, z axes and rotations about them. Every axis can be free, limited, or locked, and
//! can carry a velocity or position motor and friction. Rotation limits act on the relative rotation
//! quaternion component of each axis (exact for single axis rotation); swing can instead be limited by
//! a cone around the x axis.
//!
//! Translated from `Sources/Natsel/Simulation/SixDOFJoint.swift`.

use crate::base::constants as b3;
use crate::base::core::b3_assert;
use crate::math::functions::{
    add_mm, atan2_approx, clamp_float, compute_cos_sin, cross, dot, dot_quat,
    get_length_and_normalize, inv_mul_quat, inv_mul_world_transforms, is_valid_quat,
    make_matrix_from_quat, max_float, mul_add, mul_mv, mul_quat, mul_sub, mul_world_transforms,
    negate_quat, normalize_quat, rotate_vector, solve3, sub_pos,
};
use crate::math::types::{Matrix3, Quat, Transform, Vec3};
use crate::types::ids::JointID;
use crate::types::joint_defs::JointType;

use crate::simulation::revolute_joint::solve_point_constraint;
use crate::simulation::solver_integration::{make_soft, Softness, StepContext};
use crate::simulation::world_runtime::{BodyFlags, BodyState, JointSim, SetType, World};

// MARK: - Public API types

/// Joint axis selector. Raw values index the axis slots of [`SixDOFJointData`]; they must stay
/// identical to Swift's `SixDOFAxis.rawValue` (translation x/y/z, then rotation x/y/z).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[repr(i32)]
pub enum SixDOFAxis {
    TranslationX = 0,
    TranslationY = 1,
    TranslationZ = 2,
    RotationX = 3,
    RotationY = 4,
    RotationZ = 5,
}

impl SixDOFAxis {
    /// Swift's `CaseIterable.allCases`.
    pub const ALL: [SixDOFAxis; 6] = [
        SixDOFAxis::TranslationX,
        SixDOFAxis::TranslationY,
        SixDOFAxis::TranslationZ,
        SixDOFAxis::RotationX,
        SixDOFAxis::RotationY,
        SixDOFAxis::RotationZ,
    ];

    /// Swift's `axis.rawValue`.
    #[inline(always)]
    pub fn raw_value(self) -> i32 {
        self as i32
    }

    /// Swift's `allCases.makeIterator()` convenience; iteration order equals `ALL`.
    #[inline(always)]
    pub fn iter() -> std::array::IntoIter<SixDOFAxis, 6> {
        Self::ALL.into_iter()
    }
}

/// Motion model of one joint axis.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum SixDOFMotion {
    #[default]
    Free = 0,
    /// Between `lower_limit` and `upper_limit`.
    Limited = 1,
    Locked = 2,
}

/// Motor mode of one joint axis.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum SixDOFMotorMode {
    #[default]
    Off = 0,
    /// Drive the axis speed to `motor_target`.
    Velocity = 1,
    /// Spring the axis toward `motor_target` (translation) or [`SixDOFJointDef::target_rotation`]
    /// (rotation).
    Position = 2,
}

/// Per-axis settings stored in [`SixDOFJointDef`] and in the solver record.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SixDOFAxisSettings {
    pub motion: SixDOFMotion,
    /// Limits in meters or radians. Rotation limits must lie in (-pi, pi).
    pub lower_limit: f32,
    pub upper_limit: f32,
    /// Soft limits. Zero hertz makes the limits rigid.
    pub limit_hertz: f32,
    pub limit_damping_ratio: f32,

    pub motor_mode: SixDOFMotorMode,
    /// Target speed (velocity mode) or target translation (position mode, translation axes).
    pub motor_target: f32,
    /// Largest motor force (N) or torque (N m).
    pub max_motor_force: f32,
    /// Spring of the position motor. As with other joint springs the frequency is relative to the
    /// axis' own effective mass; a rotation spring on an anchor far from the center of mass, held by
    /// translation constraints, moves the larger inertia about the anchor and is softer than this.
    pub motor_hertz: f32,
    pub motor_damping_ratio: f32,

    /// Force (N) or torque (N m) resisting motion along or about the axis.
    pub max_friction: f32,
}

/// Swift's property initializers: `motorHertz = 2` and `motorDampingRatio = 1` are non-zero, so
/// `Default` cannot be derived.
impl Default for SixDOFAxisSettings {
    #[inline]
    fn default() -> Self {
        Self {
            motion: SixDOFMotion::Free,
            lower_limit: 0.0,
            upper_limit: 0.0,
            limit_hertz: 0.0,
            limit_damping_ratio: 0.0,
            motor_mode: SixDOFMotorMode::Off,
            motor_target: 0.0,
            max_motor_force: 0.0,
            motor_hertz: 2.0,
            motor_damping_ratio: 1.0,
            max_friction: 0.0,
        }
    }
}

impl SixDOFAxisSettings {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Swift's `init(_ motion: SixDOFMotion, lower: Float = 0, upper: Float = 0)`. Ported callers
    /// pass the defaulted limits explicitly.
    #[inline]
    pub fn with_limits(motion: SixDOFMotion, lower: f32, upper: f32) -> Self {
        Self {
            motion,
            lower_limit: lower,
            upper_limit: upper,
            ..Self::default()
        }
    }
}

/// How the swing rotation (axes y and z) is limited. Payload-free variants keep Swift's
/// discriminant order; `Cone` carries data, so the enum needs an explicit `#[repr(i32)]` for its
/// explicit discriminants.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(i32)]
pub enum SixDOFSwingLimit {
    /// Rotation y and z use their own limits.
    #[default]
    Pyramid = 0,
    /// The x axes of the two frames stay within `angle` of each other. Rotation y and z limits are
    /// ignored; locking either still locks it.
    ///
    /// Swift's `case cone(angle: Float)` associated value becomes the tuple payload.
    Cone(f32),
}

/// Definition of a six degree of freedom joint.
#[derive(Clone, Debug)]
pub struct SixDOFJointDef {
    pub base: crate::types::joint_defs::JointDef,
    /// Indexed by `SixDOFAxis::raw_value`. All axes start free.
    pub axes: Vec<SixDOFAxisSettings>,
    pub swing_limit: SixDOFSwingLimit,
    /// Target of frame B relative to frame A for rotation position motors.
    pub target_rotation: Quat,
}

impl Default for SixDOFJointDef {
    /// Swift's `init()`: `axes` is `[SixDOFAxisSettings](repeating: SixDOFAxisSettings(), count: 6)`.
    #[inline]
    fn default() -> Self {
        Self {
            base: crate::types::joint_defs::JointDef::default(),
            axes: vec![SixDOFAxisSettings::default(); 6],
            swing_limit: SixDOFSwingLimit::Pyramid,
            target_rotation: Quat::IDENTITY,
        }
    }
}

impl SixDOFJointDef {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    /// Convenience: lock every axis, then free the listed ones. Swift's defaulted
    /// `except free: [SixDOFAxis] = []` is passed explicitly.
    pub fn lock_all(&mut self, free: &[SixDOFAxis]) {
        for axis in SixDOFAxis::ALL {
            self.axes[axis.raw_value() as usize].motion = if free.contains(&axis) {
                SixDOFMotion::Free
            } else {
                SixDOFMotion::Locked
            };
        }
    }
}

/// Swift's `subscript(_ axis: SixDOFAxis) -> SixDOFAxisSettings`.
impl std::ops::Index<SixDOFAxis> for SixDOFJointDef {
    type Output = SixDOFAxisSettings;
    #[inline]
    fn index(&self, axis: SixDOFAxis) -> &Self::Output {
        &self.axes[axis.raw_value() as usize]
    }
}

impl std::ops::IndexMut<SixDOFAxis> for SixDOFJointDef {
    #[inline]
    fn index_mut(&mut self, axis: SixDOFAxis) -> &mut Self::Output {
        &mut self.axes[axis.raw_value() as usize]
    }
}

// MARK: - Solver records

/// Solver data kept in the joint record. Field order matches Swift's `SixDOFJointSim` so the
/// `JointPayload` union storage in `world_runtime.rs` stays bit-compatible.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SixDOFJointSim {
    pub slot: i32,
    pub index_a: i32,
    pub index_b: i32,
    pub frame_a: Transform,
    pub frame_b: Transform,
    pub delta_center: Vec3,
}

impl Default for SixDOFJointSim {
    /// Swift's `init()`: `slot`/`indexA`/`indexB` are `B3.nullIndex`, the transforms are
    /// `Transform()` (all-zero, not identity).
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

impl SixDOFJointSim {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }
}

/// Per axis accumulated impulses and softness.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SixDOFAxisState {
    pub settings: SixDOFAxisSettings,
    pub lock_impulse: f32,
    pub lower_impulse: f32,
    pub upper_impulse: f32,
    pub motor_impulse: f32,
    pub friction_impulse: f32,
    pub limit_softness: Softness,
    pub motor_softness: Softness,
}

impl SixDOFAxisState {
    /// Impulse along the positional Jacobian (lock and limits).
    #[inline(always)]
    pub fn positional_impulse(&self) -> f32 {
        self.lock_impulse + self.lower_impulse - self.upper_impulse
    }

    /// Impulse along the axis (motor and friction).
    #[inline(always)]
    pub fn axial_impulse(&self) -> f32 {
        self.motor_impulse + self.friction_impulse
    }
}

/// Heavy per-joint state held in `World::six_dof_store`. Swift's
/// `InlineArray<6, SixDOFAxisState>` becomes a fixed array so the record stays `Copy`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SixDOFJointData {
    pub axes: [SixDOFAxisState; 6],
    pub cone_angle: f32,
    pub use_cone: bool,
    pub cone_impulse: f32,
    pub target_rotation: Quat,
    /// Block solves when all three axes of a kind are locked.
    pub linear_block_impulse: Vec3,
    pub angular_block_impulse: Vec3,
}

impl Default for SixDOFJointData {
    /// Swift's `init()`: `targetRotation` is `Quat.identity`, not the all-zero derive.
    #[inline]
    fn default() -> Self {
        Self {
            axes: [SixDOFAxisState::default(); 6],
            cone_angle: 0.0,
            use_cone: false,
            cone_impulse: 0.0,
            target_rotation: Quat::IDENTITY,
            linear_block_impulse: Vec3::default(),
            angular_block_impulse: Vec3::default(),
        }
    }
}

impl SixDOFJointData {
    /// Swift's `init()`.
    #[inline]
    pub fn new() -> Self {
        Self::default()
    }

    #[inline(always)]
    pub fn linear_locked(&self) -> bool {
        self.axes[0].settings.motion == SixDOFMotion::Locked
            && self.axes[1].settings.motion == SixDOFMotion::Locked
            && self.axes[2].settings.motion == SixDOFMotion::Locked
    }

    #[inline(always)]
    pub fn angular_locked(&self) -> bool {
        self.axes[3].settings.motion == SixDOFMotion::Locked
            && self.axes[4].settings.motion == SixDOFMotion::Locked
            && self.axes[5].settings.motion == SixDOFMotion::Locked
    }
}

/// Derivative of relQ.v[i] with respect to the relative angular velocity:
/// d(relQ.v_i)/dt = dot(J_i, wB - wA).
///
/// Swift declared this module-internal `func` and `PathJoint.swift` calls it too, so it is
/// `pub(crate)`.
#[inline(always)]
pub(crate) fn quaternion_jacobian(frame: Quat, rel: Quat, axis: Vec3) -> Vec3 {
    0.5 * rotate_vector(frame, rel.s * axis + cross(rel.v, axis))
}

#[inline(always)]
fn component(v: Vec3, i: usize) -> f32 {
    if i == 0 { v.x } else if i == 1 { v.y } else { v.z }
}

#[inline(always)]
fn unit_axis(i: usize) -> Vec3 {
    if i == 0 {
        Vec3::AXIS_X
    } else if i == 1 {
        Vec3::AXIS_Y
    } else {
        Vec3::AXIS_Z
    }
}

/// Swift's `private struct SixDOFFrame` (local to `extension World` in this file).
#[derive(Clone, Copy)]
struct SixDofFrame {
    quat_a: Quat,
    quat_b: Quat,
    rel: Quat,
    r_a: Vec3,
    r_b: Vec3,
    d: Vec3,
    axes: Matrix3,
}

/// World space linear and angular impulse of the last substep, the two labels of Swift's
/// `(linear: Vec3, angular: Vec3)` tuple return of `sixDOFImpulses`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SixDofImpulses {
    pub linear: Vec3,
    pub angular: Vec3,
}

/// Bias terms for positional constraints: speculative when separated, soft when violated.
/// Swift's nested `limitTerms` inside `solveSixDOFJoint`; at file scope the captures become
/// parameters, the branch chain and returned triple are unchanged.
#[inline(always)]
fn limit_terms(
    c: f32,
    limit_softness: Softness,
    springy: bool,
    use_bias: bool,
    soft: Softness,
    inv_h: f32,
) -> (f32, f32, f32) {
    if c > 0.0 {
        (c * inv_h, 1.0, 0.0)
    } else if springy {
        (
            limit_softness.bias_rate * c,
            limit_softness.mass_scale,
            limit_softness.impulse_scale,
        )
    } else if use_bias {
        (soft.bias_rate * c, soft.mass_scale, soft.impulse_scale)
    } else {
        (0.0, 1.0, 0.0)
    }
}

/// Swift's nested `apply(_ impulse:)` in the linear loop of `solveSixDOFJoint`; operand order and
/// assignment sequence are preserved.
#[allow(clippy::too_many_arguments)]
#[inline(always)]
fn linear_apply(
    impulse: f32,
    axis: Vec3,
    s_a: Vec3,
    s_b: Vec3,
    m_a: f32,
    m_b: f32,
    i_a: Matrix3,
    i_b: Matrix3,
    v_a: &mut Vec3,
    w_a: &mut Vec3,
    v_b: &mut Vec3,
    w_b: &mut Vec3,
) {
    *v_a = mul_sub(*v_a, m_a * impulse, axis);
    *w_a = *w_a - mul_mv(i_a, impulse * s_a);
    *v_b = mul_add(*v_b, m_b * impulse, axis);
    *w_b = *w_b + mul_mv(i_b, impulse * s_b);
}

/// Swift's nested `cdot()` in the linear loop of `solveSixDOFJoint`. Re-evaluated at every call
/// site because `linear_apply` mutates the velocities in between.
#[inline(always)]
fn linear_cdot(v_a: Vec3, w_a: Vec3, v_b: Vec3, w_b: Vec3, axis: Vec3, s_a: Vec3, s_b: Vec3) -> f32 {
    dot(v_b - v_a, axis) + dot(w_b, s_b) - dot(w_a, s_a)
}

// MARK: - World extension

impl World {
    // Storage

    /// Write one slot of `six_dof_store` from a non-mutating method.
    ///
    /// Swift's `World` was a class, so `prepareSixDOFJoint`/`solveSixDOFJoint` wrote
    /// `sixDOFStore` without mutating `self`. The Rust solver stages are reached through
    /// `*mut World` (the worker pool) and each stage writes only its own joint's slot, so the
    /// slab base reproduces Swift's write without asserting exclusive access to the whole `World`.
    #[inline(always)]
    fn write_six_dof_data(&self, slot: usize, data: SixDOFJointData) {
        b3_assert(slot < self.six_dof_store.count());
        // SAFETY: this helper validates `slot` against `six_dof_store.count()` above, so the
        // offset stays inside the slab. The slab is never resized while a solver session is
        // active, and a joint's slot is touched only by its own constraint lane.
        unsafe {
            self.six_dof_store.as_ptr().cast_mut().add(slot).write(data);
        }
    }

    fn allocate_six_dof_data(&mut self, data: SixDOFJointData) -> i32 {
        let slot = self.six_dof_pool.alloc();
        if slot as usize == self.six_dof_store.count() {
            self.six_dof_store.append(data);
        } else {
            self.six_dof_store.set(slot as usize, data);
        }
        slot
    }

    pub(crate) fn release_six_dof_data(&mut self, slot: i32) {
        if !(slot >= 0 && (slot as usize) < self.six_dof_store.count()) {
            return;
        }
        self.six_dof_store.set(slot as usize, SixDOFJointData::default());
        self.six_dof_pool.free(slot);
    }

    // Creation

    /// Create a six degree of freedom joint. Swift's `@discardableResult`: callers ported from
    /// Swift may ignore the returned id.
    pub fn create_six_dof_joint(&mut self, def: SixDOFJointDef) -> JointID {
        if !(self.is_valid_body(def.base.body_id_a)
            && self.is_valid_body(def.base.body_id_b)
            && def.base.body_id_a != def.base.body_id_b
            && def.axes.len() == 6)
        {
            return JointID::NULL;
        }

        let mut data = SixDOFJointData::default();
        for i in 0..6 {
            let mut settings = def.axes[i];
            if settings.motion == SixDOFMotion::Limited && settings.upper_limit < settings.lower_limit
            {
                std::mem::swap(&mut settings.lower_limit, &mut settings.upper_limit);
            }
            data.axes[i].settings = settings;
        }
        if let SixDOFSwingLimit::Cone(angle) = def.swing_limit {
            data.use_cone = true;
            data.cone_angle = clamp_float(angle, 0.0, b3::PI);
        }
        data.target_rotation = normalize_quat(def.target_rotation);

        let pair = self.create_joint(&def.base, JointType::SixDOFJoint);
        let mut sim = pair.sim;
        let mut joint = SixDOFJointSim::default();
        joint.slot = self.allocate_six_dof_data(data);
        sim.set_six_dof(joint);
        self.write_joint_sim(pair.joint_id, sim);
        JointID::new(
            pair.joint_id + 1,
            self.world_id,
            self.joints[pair.joint_id as usize].generation,
        )
    }

    // Prepare

    pub(crate) fn prepare_six_dof_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
    ) {
        let body_a = self.joint_prepare_body(base.body_id_a);
        let body_b = self.joint_prepare_body(base.body_id_b);
        let sim_a = self.joint_prepare_sim(&body_a);
        let sim_b = self.joint_prepare_sim(&body_b);
        base.inv_mass_a = sim_a.inv_mass;
        base.inv_mass_b = sim_b.inv_mass;
        base.inv_ia = sim_a.inv_inertia_world;
        base.inv_ib = sim_b.inv_inertia_world;

        let mut joint = base.six_dof();
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
        base.set_six_dof(joint);

        let slot = joint.slot as usize;
        for i in 0..6 {
            let mut axis = self.six_dof_store.get(slot).axes[i];
            axis.limit_softness =
                make_soft(axis.settings.limit_hertz, axis.settings.limit_damping_ratio, context.h);
            axis.motor_softness =
                make_soft(axis.settings.motor_hertz, axis.settings.motor_damping_ratio, context.h);
            if !context.enable_warm_starting {
                axis.lock_impulse = 0.0;
                axis.lower_impulse = 0.0;
                axis.upper_impulse = 0.0;
                axis.motor_impulse = 0.0;
                axis.friction_impulse = 0.0;
            }
            if axis.settings.motion != SixDOFMotion::Limited {
                axis.lower_impulse = 0.0;
                axis.upper_impulse = 0.0;
            }
            if axis.settings.motion != SixDOFMotion::Locked {
                axis.lock_impulse = 0.0;
            }
            if axis.settings.motor_mode == SixDOFMotorMode::Off {
                axis.motor_impulse = 0.0;
            }
            if axis.settings.max_friction <= 0.0 {
                axis.friction_impulse = 0.0;
            }
            let mut data = self.six_dof_store.get(slot);
            data.axes[i] = axis;
            self.write_six_dof_data(slot, data);
        }
        if !context.enable_warm_starting {
            let mut data = self.six_dof_store.get(slot);
            data.cone_impulse = 0.0;
            data.linear_block_impulse = Vec3::default();
            data.angular_block_impulse = Vec3::default();
            self.write_six_dof_data(slot, data);
        }
        if !self.six_dof_store.get(slot).linear_locked() {
            let mut data = self.six_dof_store.get(slot);
            data.linear_block_impulse = Vec3::default();
            self.write_six_dof_data(slot, data);
        }
        if !self.six_dof_store.get(slot).angular_locked() {
            let mut data = self.six_dof_store.get(slot);
            data.angular_block_impulse = Vec3::default();
            self.write_six_dof_data(slot, data);
        }
    }

    // Geometry

    #[inline(always)]
    fn six_dof_frame(&self, joint: SixDOFJointSim, state_a: BodyState, state_b: BodyState) -> SixDofFrame {
        let quat_a = mul_quat(state_a.delta_rotation, joint.frame_a.q);
        let mut quat_b = mul_quat(state_b.delta_rotation, joint.frame_b.q);
        if dot_quat(quat_a, quat_b) < 0.0 {
            quat_b = negate_quat(quat_b);
        }
        let r_a = rotate_vector(state_a.delta_rotation, joint.frame_a.p);
        let r_b = rotate_vector(state_b.delta_rotation, joint.frame_b.p);
        let d = ((state_b.delta_position - state_a.delta_position) + joint.delta_center) + (r_b - r_a);
        SixDofFrame {
            quat_a,
            quat_b,
            rel: inv_mul_quat(quat_a, quat_b),
            r_a,
            r_b,
            d,
            axes: make_matrix_from_quat(quat_a),
        }
    }

    // Warm start

    pub(crate) fn warm_start_six_dof_joint(
        &self,
        base: &mut JointSim,
        states: &mut [BodyState],
    ) {
        let joint = base.six_dof();
        let data = self.six_dof_store.get(joint.slot as usize);
        let mut state_a = World::state_at(states.as_mut_ptr(), joint.index_a);
        let mut state_b = World::state_at(states.as_mut_ptr(), joint.index_b);
        let f = self.six_dof_frame(joint, state_a, state_b);

        let mut linear = Vec3::default();
        let mut angular_a = Vec3::default();
        let mut angular_b = Vec3::default();

        if data.linear_locked() {
            linear = data.linear_block_impulse;
            angular_a = cross(f.r_a, linear);
            angular_b = cross(f.r_b, linear);
        }
        for i in 0..3 {
            let state = data.axes[i];
            let total = (if data.linear_locked() {
                0.0
            } else {
                state.positional_impulse()
            }) + state.axial_impulse();
            if total == 0.0 {
                continue;
            }
            let axis = if i == 0 {
                f.axes.cx
            } else if i == 1 {
                f.axes.cy
            } else {
                f.axes.cz
            };
            linear = mul_add(linear, total, axis);
            angular_a = mul_add(angular_a, total, cross(f.d + f.r_a, axis));
            angular_b = mul_add(angular_b, total, cross(f.r_b, axis));
        }

        let mut angular = Vec3::default();
        if data.angular_locked() {
            let block = data.angular_block_impulse;
            angular = block.x * quaternion_jacobian(f.quat_a, f.rel, Vec3::AXIS_X)
                + block.y * quaternion_jacobian(f.quat_a, f.rel, Vec3::AXIS_Y)
                + block.z * quaternion_jacobian(f.quat_a, f.rel, Vec3::AXIS_Z);
        }
        for i in 0..3 {
            let state = data.axes[3 + i];
            let axis = if i == 0 {
                f.axes.cx
            } else if i == 1 {
                f.axes.cy
            } else {
                f.axes.cz
            };
            if state.settings.motor_mode == SixDOFMotorMode::Position && state.motor_impulse != 0.0 {
                // The position motor acts along the Jacobian of the rotation error.
                let mut error = inv_mul_quat(data.target_rotation, f.rel);
                if error.s < 0.0 {
                    error = negate_quat(error);
                }
                let jacobian = 2.0
                    * quaternion_jacobian(mul_quat(f.quat_a, data.target_rotation), error, unit_axis(i));
                angular = mul_add(angular, state.motor_impulse, jacobian);
                angular = mul_add(angular, state.friction_impulse, axis);
            } else {
                angular = mul_add(angular, state.axial_impulse(), axis);
            }
            if !data.angular_locked() && state.positional_impulse() != 0.0 {
                angular = mul_add(
                    angular,
                    state.positional_impulse(),
                    quaternion_jacobian(f.quat_a, f.rel, unit_axis(i)),
                );
            }
        }
        if data.use_cone && data.cone_impulse != 0.0 {
            let x_a = rotate_vector(f.quat_a, Vec3::AXIS_X);
            let x_b = rotate_vector(f.quat_b, Vec3::AXIS_X);
            let (len, cone_axis) = get_length_and_normalize(cross(x_a, x_b));
            if len > 1.0e-6 {
                angular = mul_sub(angular, data.cone_impulse, cone_axis);
            }
        }

        state_a.linear_velocity = mul_sub(state_a.linear_velocity, base.inv_mass_a, linear);
        state_a.angular_velocity =
            state_a.angular_velocity - mul_mv(base.inv_ia, angular_a + angular);
        state_b.linear_velocity = mul_add(state_b.linear_velocity, base.inv_mass_b, linear);
        state_b.angular_velocity =
            state_b.angular_velocity + mul_mv(base.inv_ib, angular_b + angular);
        World::set_state_at(states.as_mut_ptr(), joint.index_a, state_a);
        World::set_state_at(states.as_mut_ptr(), joint.index_b, state_b);
    }

    // Solve

    pub(crate) fn solve_six_dof_joint(
        &self,
        base: &mut JointSim,
        context: &StepContext,
        use_bias: bool,
        states: &mut [BodyState],
    ) {
        let joint = base.six_dof();
        let slot = joint.slot as usize;
        let mut data = self.six_dof_store.get(slot);
        let mut state_a = World::state_at(states.as_mut_ptr(), joint.index_a);
        let mut state_b = World::state_at(states.as_mut_ptr(), joint.index_b);
        let m_a = base.inv_mass_a;
        let m_b = base.inv_mass_b;
        let i_a = base.inv_ia;
        let i_b = base.inv_ib;
        let i_sum = add_mm(i_a, i_b);
        let h = context.h;
        let soft = base.constraint_softness;

        let mut v_a = state_a.linear_velocity;
        let mut w_a = state_a.angular_velocity;
        let mut v_b = state_b.linear_velocity;
        let mut w_b = state_b.angular_velocity;

        let f = self.six_dof_frame(joint, state_a, state_b);

        // MARK: Angular

        // Velocity motors and friction about the frame A axes.
        for i in 0..3 {
            let mut state = data.axes[3 + i];
            let s = state.settings;
            if !(s.motor_mode != SixDOFMotorMode::Off || s.max_friction > 0.0) {
                continue;
            }
            let axis = if i == 0 {
                f.axes.cx
            } else if i == 1 {
                f.axes.cy
            } else {
                f.axes.cz
            };
            let k = dot(axis, mul_mv(i_sum, axis));
            if !(k > 0.0) {
                continue;
            }
            let mass = 1.0 / k;

            if s.motor_mode == SixDOFMotorMode::Velocity {
                let cdot = dot(w_b - w_a, axis) - s.motor_target;
                let max_impulse = s.max_motor_force * h;
                let old = state.motor_impulse;
                state.motor_impulse = clamp_float(old - mass * cdot, -max_impulse, max_impulse);
                let impulse = state.motor_impulse - old;
                w_a = w_a - mul_mv(i_a, impulse * axis);
                w_b = w_b + mul_mv(i_b, impulse * axis);
            } else if s.motor_mode == SixDOFMotorMode::Position {
                // Spring the axis component of the rotation error toward the target.
                let mut error = inv_mul_quat(data.target_rotation, f.rel);
                if error.s < 0.0 {
                    error = negate_quat(error);
                }
                // Twice the quaternion component is the angle for small errors, so the impulse is
                // a torque impulse and `maxMotorForce` is a true torque limit.
                let frame = mul_quat(f.quat_a, data.target_rotation);
                let jacobian = 2.0 * quaternion_jacobian(frame, error, unit_axis(i));
                let kj = dot(jacobian, mul_mv(i_sum, jacobian));
                if kj > 0.0 {
                    let c = 2.0 * component(error.v, i);
                    let cdot = dot(w_b - w_a, jacobian);
                    let ms = state.motor_softness;
                    let max_impulse = s.max_motor_force * h;
                    let old = state.motor_impulse;
                    let delta =
                        -ms.mass_scale / kj * (cdot + ms.bias_rate * c) - ms.impulse_scale * old;
                    state.motor_impulse = clamp_float(old + delta, -max_impulse, max_impulse);
                    let impulse = state.motor_impulse - old;
                    w_a = w_a - mul_mv(i_a, impulse * jacobian);
                    w_b = w_b + mul_mv(i_b, impulse * jacobian);
                }
            }

            if s.max_friction > 0.0 {
                let cdot = dot(w_b - w_a, axis);
                let max_impulse = s.max_friction * h;
                let old = state.friction_impulse;
                state.friction_impulse = clamp_float(old - mass * cdot, -max_impulse, max_impulse);
                let impulse = state.friction_impulse - old;
                w_a = w_a - mul_mv(i_a, impulse * axis);
                w_b = w_b + mul_mv(i_b, impulse * axis);
            }
            data.axes[3 + i] = state;
        }

        // Swing cone.
        if data.use_cone
            && (data.axes[4].settings.motion == SixDOFMotion::Limited
                || data.axes[5].settings.motion == SixDOFMotion::Limited)
        {
            let x_a = rotate_vector(f.quat_a, Vec3::AXIS_X);
            let x_b = rotate_vector(f.quat_b, Vec3::AXIS_X);
            let (len, cone_axis) = get_length_and_normalize(cross(x_a, x_b));
            if len > 1.0e-6 {
                let angle = atan2_approx(len, dot(x_a, x_b));
                let c = data.cone_angle - angle;
                let (bias, mass_scale, impulse_scale) = limit_terms(
                    c,
                    data.axes[4].limit_softness,
                    data.axes[4].settings.limit_hertz > 0.0,
                    use_bias,
                    soft,
                    context.inv_h,
                );
                let k = dot(cone_axis, mul_mv(i_sum, cone_axis));
                if k > 0.0 {
                    let cdot = -dot(w_b - w_a, cone_axis);
                    let old = data.cone_impulse;
                    let delta =
                        max_float(-mass_scale / k * (cdot + bias) - impulse_scale * old, -old);
                    data.cone_impulse = old + delta;
                    w_a = w_a + mul_mv(i_a, delta * cone_axis);
                    w_b = w_b - mul_mv(i_b, delta * cone_axis);
                }
            }
        } else {
            data.cone_impulse = 0.0;
        }

        // Locks and limits on the relative rotation components.
        let rel_q = f.rel;
        if data.angular_locked() {
            let jx = quaternion_jacobian(f.quat_a, rel_q, Vec3::AXIS_X);
            let jy = quaternion_jacobian(f.quat_a, rel_q, Vec3::AXIS_Y);
            let jz = quaternion_jacobian(f.quat_a, rel_q, Vec3::AXIS_Z);
            let gx = mul_mv(i_sum, jx);
            let gy = mul_mv(i_sum, jy);
            let gz = mul_mv(i_sum, jz);
            let k = Matrix3::new(
                Vec3::new(dot(jx, gx), dot(jy, gx), dot(jz, gx)),
                Vec3::new(dot(jx, gy), dot(jy, gy), dot(jz, gy)),
                Vec3::new(dot(jx, gz), dot(jy, gz), dot(jz, gz)),
            );
            let w_rel = w_b - w_a;
            let cdot = Vec3::new(dot(w_rel, jx), dot(w_rel, jy), dot(w_rel, jz));
            let mut bias = Vec3::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias {
                bias = soft.bias_rate * rel_q.v;
                mass_scale = soft.mass_scale;
                impulse_scale = soft.impulse_scale;
            }
            let solution = solve3(k, cdot + bias);
            let delta = (-mass_scale) * solution - impulse_scale * data.angular_block_impulse;
            data.angular_block_impulse = data.angular_block_impulse + delta;
            let angular = delta.x * jx + delta.y * jy + delta.z * jz;
            w_a = w_a - mul_mv(i_a, angular);
            w_b = w_b + mul_mv(i_b, angular);
        } else {
            for i in 0..3 {
                let mut state = data.axes[3 + i];
                let s = state.settings;
                let swing = i > 0;
                if s.motion == SixDOFMotion::Free
                    || (swing && data.use_cone && s.motion == SixDOFMotion::Limited)
                {
                    continue;
                }
                let jacobian = quaternion_jacobian(f.quat_a, rel_q, unit_axis(i));
                let k = dot(jacobian, mul_mv(i_sum, jacobian));
                if !(k > 0.0) {
                    continue;
                }
                let value = component(rel_q.v, i);
                let cdot = dot(w_b - w_a, jacobian);

                if s.motion == SixDOFMotion::Locked {
                    let mut bias: f32 = 0.0;
                    let mut mass_scale: f32 = 1.0;
                    let mut impulse_scale: f32 = 0.0;
                    if use_bias {
                        bias = soft.bias_rate * value;
                        mass_scale = soft.mass_scale;
                        impulse_scale = soft.impulse_scale;
                    }
                    let delta =
                        -mass_scale / k * (cdot + bias) - impulse_scale * state.lock_impulse;
                    state.lock_impulse += delta;
                    w_a = w_a - mul_mv(i_a, delta * jacobian);
                    w_b = w_b + mul_mv(i_b, delta * jacobian);
                } else {
                    let springy = s.limit_hertz > 0.0;
                    let lower = compute_cos_sin(0.5 * s.lower_limit).sine;
                    let upper = compute_cos_sin(0.5 * s.upper_limit).sine;
                    {
                        let (bias, mass_scale, impulse_scale) =
                            limit_terms(value - lower, state.limit_softness, springy, use_bias, soft, context.inv_h);
                        let old = state.lower_impulse;
                        let delta = max_float(
                            -mass_scale / k * (cdot + bias) - impulse_scale * old,
                            -old,
                        );
                        state.lower_impulse = old + delta;
                        w_a = w_a - mul_mv(i_a, delta * jacobian);
                        w_b = w_b + mul_mv(i_b, delta * jacobian);
                    }
                    {
                        let cdot_upper = -dot(w_b - w_a, jacobian);
                        let (bias, mass_scale, impulse_scale) =
                            limit_terms(upper - value, state.limit_softness, springy, use_bias, soft, context.inv_h);
                        let old = state.upper_impulse;
                        let delta = max_float(
                            -mass_scale / k * (cdot_upper + bias) - impulse_scale * old,
                            -old,
                        );
                        state.upper_impulse = old + delta;
                        w_a = w_a + mul_mv(i_a, delta * jacobian);
                        w_b = w_b - mul_mv(i_b, delta * jacobian);
                    }
                }
                data.axes[3 + i] = state;
            }
        }

        // MARK: Linear

        let d = f.d;
        for i in 0..3 {
            let mut state = data.axes[i];
            let s = state.settings;
            let axis = if i == 0 {
                f.axes.cx
            } else if i == 1 {
                f.axes.cy
            } else {
                f.axes.cz
            };
            let s_a = cross(d + f.r_a, axis);
            let s_b = cross(f.r_b, axis);
            let k = m_a
                + m_b
                + dot(s_a, mul_mv(i_a, s_a))
                + dot(s_b, mul_mv(i_b, s_b));
            if !(k > 0.0) {
                continue;
            }
            let mass = 1.0 / k;
            let translation = dot(d, axis);

            if s.motor_mode == SixDOFMotorMode::Velocity {
                let max_impulse = s.max_motor_force * h;
                let old = state.motor_impulse;
                let cdot = linear_cdot(v_a, w_a, v_b, w_b, axis, s_a, s_b);
                state.motor_impulse =
                    clamp_float(old - mass * (cdot - s.motor_target), -max_impulse, max_impulse);
                linear_apply(
                    state.motor_impulse - old,
                    axis,
                    s_a,
                    s_b,
                    m_a,
                    m_b,
                    i_a,
                    i_b,
                    &mut v_a,
                    &mut w_a,
                    &mut v_b,
                    &mut w_b,
                );
            } else if s.motor_mode == SixDOFMotorMode::Position {
                let ms = state.motor_softness;
                let c = translation - s.motor_target;
                let max_impulse = s.max_motor_force * h;
                let old = state.motor_impulse;
                let cdot = linear_cdot(v_a, w_a, v_b, w_b, axis, s_a, s_b);
                let delta = -ms.mass_scale * mass * (cdot + ms.bias_rate * c) - ms.impulse_scale * old;
                state.motor_impulse = clamp_float(old + delta, -max_impulse, max_impulse);
                linear_apply(
                    state.motor_impulse - old,
                    axis,
                    s_a,
                    s_b,
                    m_a,
                    m_b,
                    i_a,
                    i_b,
                    &mut v_a,
                    &mut w_a,
                    &mut v_b,
                    &mut w_b,
                );
            }
            if s.max_friction > 0.0 {
                let max_impulse = s.max_friction * h;
                let old = state.friction_impulse;
                let cdot = linear_cdot(v_a, w_a, v_b, w_b, axis, s_a, s_b);
                state.friction_impulse = clamp_float(old - mass * cdot, -max_impulse, max_impulse);
                linear_apply(
                    state.friction_impulse - old,
                    axis,
                    s_a,
                    s_b,
                    m_a,
                    m_b,
                    i_a,
                    i_b,
                    &mut v_a,
                    &mut w_a,
                    &mut v_b,
                    &mut w_b,
                );
            }

            if !data.linear_locked() {
                if s.motion == SixDOFMotion::Locked {
                    let mut bias: f32 = 0.0;
                    let mut mass_scale: f32 = 1.0;
                    let mut impulse_scale: f32 = 0.0;
                    if use_bias {
                        bias = soft.bias_rate * translation;
                        mass_scale = soft.mass_scale;
                        impulse_scale = soft.impulse_scale;
                    }
                    let cdot = linear_cdot(v_a, w_a, v_b, w_b, axis, s_a, s_b);
                    let delta = -mass_scale * mass * (cdot + bias) - impulse_scale * state.lock_impulse;
                    state.lock_impulse += delta;
                    linear_apply(
                        delta,
                        axis,
                        s_a,
                        s_b,
                        m_a,
                        m_b,
                        i_a,
                        i_b,
                        &mut v_a,
                        &mut w_a,
                        &mut v_b,
                        &mut w_b,
                    );
                } else if s.motion == SixDOFMotion::Limited {
                    let springy = s.limit_hertz > 0.0;
                    {
                        let (bias, mass_scale, impulse_scale) = limit_terms(
                            translation - s.lower_limit,
                            state.limit_softness,
                            springy,
                            use_bias,
                            soft,
                            context.inv_h,
                        );
                        let old = state.lower_impulse;
                        let cdot = linear_cdot(v_a, w_a, v_b, w_b, axis, s_a, s_b);
                        let delta = max_float(
                            -mass_scale * mass * (cdot + bias) - impulse_scale * old,
                            -old,
                        );
                        state.lower_impulse = old + delta;
                        linear_apply(
                            delta,
                            axis,
                            s_a,
                            s_b,
                            m_a,
                            m_b,
                            i_a,
                            i_b,
                            &mut v_a,
                            &mut w_a,
                            &mut v_b,
                            &mut w_b,
                        );
                    }
                    {
                        let (bias, mass_scale, impulse_scale) = limit_terms(
                            s.upper_limit - translation,
                            state.limit_softness,
                            springy,
                            use_bias,
                            soft,
                            context.inv_h,
                        );
                        let old = state.upper_impulse;
                        let cdot = linear_cdot(v_a, w_a, v_b, w_b, axis, s_a, s_b);
                        let delta = max_float(
                            -mass_scale * mass * (-cdot + bias) - impulse_scale * old,
                            -old,
                        );
                        state.upper_impulse = old + delta;
                        linear_apply(
                            -delta,
                            axis,
                            s_a,
                            s_b,
                            m_a,
                            m_b,
                            i_a,
                            i_b,
                            &mut v_a,
                            &mut w_a,
                            &mut v_b,
                            &mut w_b,
                        );
                    }
                }
            }
            data.axes[i] = state;
        }

        if data.linear_locked() {
            // Point to point, as a 3-by-3 block.
            let cdot = (v_b + cross(w_b, f.r_b)) - v_a - cross(w_a, f.r_a);
            let mut bias = Vec3::default();
            let mut mass_scale: f32 = 1.0;
            let mut impulse_scale: f32 = 0.0;
            if use_bias {
                bias = soft.bias_rate * d;
                mass_scale = soft.mass_scale;
                impulse_scale = soft.impulse_scale;
            }
            let b = solve_point_constraint(m_a, m_b, i_a, i_b, f.r_a, f.r_b, cdot + bias);
            let impulse = (-mass_scale) * b - impulse_scale * data.linear_block_impulse;
            data.linear_block_impulse = data.linear_block_impulse + impulse;
            v_a = v_a - m_a * impulse;
            w_a = w_a - mul_mv(i_a, cross(f.r_a, impulse));
            v_b = v_b + m_b * impulse;
            w_b = w_b + mul_mv(i_b, cross(f.r_b, impulse));
        }

        if (state_a.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_a.linear_velocity = v_a;
            state_a.angular_velocity = w_a;
        }
        if (state_b.flags & BodyFlags::DYNAMIC_FLAG.0) != 0 {
            state_b.linear_velocity = v_b;
            state_b.angular_velocity = w_b;
        }
        World::set_state_at(states.as_mut_ptr(), joint.index_a, state_a);
        World::set_state_at(states.as_mut_ptr(), joint.index_b, state_b);
        self.write_six_dof_data(slot, data);
    }

    // Reactions

    /// World space linear and angular impulse of the last substep.
    pub(crate) fn six_dof_impulses(&self, sim: JointSim) -> SixDofImpulses {
        let slot = sim.six_dof().slot;
        if !(slot >= 0 && (slot as usize) < self.six_dof_store.count()) {
            return SixDofImpulses::default();
        }
        let slot = slot as usize;
        let data = self.six_dof_store.get(slot);
        let q = mul_quat(
            self.body_transform_quick(sim.body_id_a).q,
            sim.local_frame_a.q,
        );
        let axes = make_matrix_from_quat(q);
        let mut linear = data.linear_block_impulse;
        let mut angular = Vec3::default();
        for i in 0..3 {
            let axis = if i == 0 {
                axes.cx
            } else if i == 1 {
                axes.cy
            } else {
                axes.cz
            };
            linear = mul_add(
                linear,
                data.axes[i].positional_impulse() + data.axes[i].axial_impulse(),
                axis,
            );
            angular = mul_add(angular, data.axes[3 + i].axial_impulse(), axis);
            // Quaternion component impulses act about roughly half the axis.
            angular = mul_add(
                angular,
                0.5 * (data.axes[3 + i].positional_impulse() + component(data.angular_block_impulse, i)),
                axis,
            );
        }
        SixDofImpulses { linear, angular }
    }

    // Accessors

    fn six_dof_slot(&self, joint_id: JointID) -> Option<usize> {
        let index = joint_id.index1 - 1;
        if !(joint_id.world0 == self.world_id && index >= 0 && (index as usize) < self.joints.len()) {
            return None;
        }
        let joint = &self.joints[index as usize];
        if joint.joint_id == b3::NULL_INDEX
            || joint.generation != joint_id.generation
            || joint.type_ != JointType::SixDOFJoint
        {
            return None;
        }
        let slot = self.read_joint_sim(index).six_dof().slot;
        if slot >= 0 && (slot as usize) < self.six_dof_store.count() {
            Some(slot as usize)
        } else {
            None
        }
    }

    pub fn six_dof_joint_axis(&self, joint_id: JointID, axis: SixDOFAxis) -> Option<SixDOFAxisSettings> {
        self.six_dof_slot(joint_id)
            .map(|slot| self.six_dof_store.get(slot).axes[axis.raw_value() as usize].settings)
    }

    /// Change one axis at run time, for example to unlock a door or retarget a motor.
    /// Swift's `@discardableResult`: the success flag may be ignored.
    pub fn set_six_dof_joint_axis(
        &mut self,
        joint_id: JointID,
        axis: SixDOFAxis,
        settings: SixDOFAxisSettings,
    ) -> bool {
        let Some(slot) = self.six_dof_slot(joint_id) else {
            return false;
        };
        let mut s = settings;
        if s.motion == SixDOFMotion::Limited && s.upper_limit < s.lower_limit {
            std::mem::swap(&mut s.lower_limit, &mut s.upper_limit);
        }
        let mut data = self.six_dof_store.get(slot);
        data.axes[axis.raw_value() as usize].settings = s;
        self.six_dof_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    /// Swift's `@discardableResult`: the success flag may be ignored.
    pub fn set_six_dof_joint_motor_target(
        &mut self,
        joint_id: JointID,
        axis: SixDOFAxis,
        target: f32,
    ) -> bool {
        let Some(slot) = self.six_dof_slot(joint_id) else {
            return false;
        };
        let mut data = self.six_dof_store.get(slot);
        data.axes[axis.raw_value() as usize].settings.motor_target = target;
        self.six_dof_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    /// Swift's `@discardableResult`: the success flag may be ignored.
    pub fn set_six_dof_joint_target_rotation(&mut self, joint_id: JointID, rotation: Quat) -> bool {
        let Some(slot) = self.six_dof_slot(joint_id) else {
            return false;
        };
        if !is_valid_quat(rotation) {
            return false;
        }
        let mut data = self.six_dof_store.get(slot);
        data.target_rotation = normalize_quat(rotation);
        self.six_dof_store.set(slot, data);
        self.wake_joint_bodies(joint_id);
        true
    }

    /// Current translation of frame B in frame A, and the rotation of frame B relative to frame A.
    pub fn six_dof_joint_pose(&self, joint_id: JointID) -> Option<Transform> {
        let sim = self.joint_sim(joint_id)?;
        if sim.type_ != JointType::SixDOFJoint {
            return None;
        }
        let frame_a = mul_world_transforms(
            self.body_transform_quick(sim.body_id_a),
            sim.local_frame_a,
        );
        let frame_b = mul_world_transforms(
            self.body_transform_quick(sim.body_id_b),
            sim.local_frame_b,
        );
        Some(inv_mul_world_transforms(frame_a, frame_b))
    }
}
