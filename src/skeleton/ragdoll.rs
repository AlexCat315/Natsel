//! Ragdolls built from a skeleton (Jolt's RagdollSettings and Ragdoll). Each skeleton joint gets
//! one body whose origin and rotation are the joint's model transform, and every joint with a
//! parent is connected to it by a six DOF joint: swing-twist, hinge, or fixed. A ragdoll can be
//! posed directly, driven to an animated pose with joint motors (it stays physical and reacts to
//! hits), or driven kinematically by velocities.
//!
//! Translated from `Sources/Natsel/Skeleton/Ragdoll.swift`.
//!
//! Borrowing adaptation (behaviour identical): Swift held the world as `public let world`, a
//! shared class reference usable for both reads and mutations. Rust models it as a single
//! `&'w mut World` field, the precedent set by `character/character_virtual.rs`; the field is not
//! `pub` because exposing it would let callers move the borrow out, while every Swift use was an
//! in-place call through the reference. `public private(set)` arrays become private fields with
//! slice accessors.
//!
//! Deviations:
//! - Swift struct copies (`let part = settings.parts[i]`, `var def = part.bodyDef`) are either
//!   plain borrows (the copy was never written and never aliased) or `.clone()`s where Swift
//!   mutated the copy afterwards - same values either way.
//! - `ClosedRange<Float>` (Swift stdlib, used as enum payload) becomes the shim [`ClosedRange`];
//!   only `lower_bound` / `upper_bound` are read.
//! - Swift's defaulted arguments (`userData = nil`, `resetVelocities = false`, `hertz = 8`,
//!   `dampingRatio = 1`, `setBodyTargetTransform(wake: true)`, `RagdollPart.init(joint: .fixed,
//!   jointFrame: .identity)`) are passed explicitly by ported callers per the migration
//!   convention.
//! - `JointID()` (Swift's null handle) is `JointID::default()`, zero-initialised.

use std::ffi::c_void;

use crate::collision::hull::{make_transformed_box_hull, Hull};
use crate::math::functions::{
    aabb_inflate, aabb_overlaps, aabb_union, clamp_float, inv_mul_quat, inv_mul_transforms,
    max_float, mul_quat, mul_sm, mul_transforms, normalize_quat,
};
use crate::math::types::{AABB, Transform, Vec3};
use crate::skeleton::skeleton::{Skeleton, SkeletonPose};
use crate::simulation::six_dof_joint::{
    SixDOFAxis, SixDOFAxisSettings, SixDOFJointDef, SixDOFMotion, SixDOFMotorMode,
    SixDOFSwingLimit,
};
use crate::simulation::world_runtime::World;
use crate::types::body_def::{BodyDef, BodyType};
use crate::types::geometry::{Capsule, MassData, Sphere};
use crate::types::ids::{BodyID, JointID};
use crate::types::joint_defs::FilterJointDef;
use crate::types::shape_def::ShapeDef;

/// Swift's `ClosedRange<Float>`: only `lowerBound` and `upperBound` are read by the ragdoll code.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ClosedRange {
    pub lower_bound: f32,
    pub upper_bound: f32,
}

impl ClosedRange {
    /// Swift's `lower...upper` range formation.
    #[inline]
    pub fn new(lower_bound: f32, upper_bound: f32) -> Self {
        Self {
            lower_bound,
            upper_bound,
        }
    }
}

/// Collision geometry of a ragdoll part, in the part's joint space.
#[derive(Clone)]
pub enum RagdollShape {
    Capsule(Capsule),
    Sphere(Sphere),
    Box {
        half_extents: Vec3,
        transform: Transform,
    },
    Hull(Hull),
}

/// Connection of a part to its parent. Angles in radians; the twist axis is x of `joint_frame`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RagdollJointKind {
    /// Twist about x within [twist_min, twist_max], and swing of x inside a cone.
    SwingTwist {
        cone_angle: f32,
        twist_min: f32,
        twist_max: f32,
    },
    /// Twist about x, and separate swing limits about y and z.
    Pyramid {
        twist: ClosedRange,
        swing_y: ClosedRange,
        swing_z: ClosedRange,
    },
    /// Rotation about x only.
    Hinge { lower: f32, upper: f32 },
    Fixed,
}

#[derive(Clone)]
pub struct RagdollPart {
    pub shapes: Vec<RagdollShape>,
    /// Density, friction, filter, and events for the part's shapes.
    pub shape_def: ShapeDef,
    /// Damping, gravity scale, sleep, bullet, and so on. Type, position, and rotation are set by
    /// the ragdoll.
    pub body_def: BodyDef,
    /// Ignored for root parts.
    pub joint: RagdollJointKind,
    /// Joint frame in the part's joint space; its x axis is the twist axis.
    pub joint_frame: Transform,
    /// Torque resisting rotation in the joint (N m).
    pub joint_friction: f32,
    /// Soft limits; zero hertz is rigid.
    pub limit_hertz: f32,
    pub limit_damping_ratio: f32,
}

impl Default for RagdollPart {
    /// Swift's `init()` and the field defaults.
    fn default() -> Self {
        Self {
            shapes: Vec::new(),
            shape_def: ShapeDef::default(),
            body_def: BodyDef::default(),
            joint: RagdollJointKind::SwingTwist {
                cone_angle: 0.5,
                twist_min: -0.3,
                twist_max: 0.3,
            },
            joint_frame: Transform::IDENTITY,
            joint_friction: 0.0,
            limit_hertz: 0.0,
            limit_damping_ratio: 0.0,
        }
    }
}

impl RagdollPart {
    /// Swift's `init()`.
    pub fn new() -> Self {
        Self::default()
    }

    /// Swift's `init(_ shapes:joint:jointFrame:)`; the defaulted `joint = .fixed` and
    /// `jointFrame = .identity` are passed explicitly.
    pub fn with_shapes(
        shapes: Vec<RagdollShape>,
        joint: RagdollJointKind,
        joint_frame: Transform,
    ) -> Self {
        Self {
            shapes,
            joint,
            joint_frame,
            ..Self::default()
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RagdollSelfCollision {
    /// Parts never collide with each other (a shared negative collision group).
    None,
    /// Only parts connected by a joint skip collision.
    SkipConnected,
    /// Also skip parts whose bounds overlap in the creation pose, enlarged by `margin`, like Jolt's
    /// `DisableParentChildCollisions`.
    SkipOverlapping { margin: f32 },
    /// Every pair collides, connected ones too.
    All,
}

#[derive(Clone)]
pub struct RagdollSettings {
    pub skeleton: Skeleton,
    /// One part per skeleton joint, in skeleton order.
    pub parts: Vec<RagdollPart>,
    pub self_collision: RagdollSelfCollision,
    /// Negative group used by `.None`.
    pub collision_group: i32,
    /// Limit the mass ratio across each joint (Jolt's Stabilize). Very light parts hanging from
    /// heavy ones, or the reverse, make joint chains converge slowly. Zero disables it.
    pub max_mass_ratio: f32,
}

impl RagdollSettings {
    /// Swift's `init(skeleton:parts:)`; the remaining properties keep their Swift defaults.
    pub fn new(skeleton: Skeleton, parts: Vec<RagdollPart>) -> Self {
        Self {
            skeleton,
            parts,
            self_collision: RagdollSelfCollision::SkipOverlapping { margin: 0.02 },
            collision_group: -1,
            max_mass_ratio: 4.0,
        }
    }
}

pub struct Ragdoll<'w> {
    pub settings: RagdollSettings,
    /// Swift's `public let world` shared reference, see the borrowing adaptation at the top.
    world: &'w mut World,
    /// One body per skeleton joint. Swift's `public private(set) var bodies`.
    bodies: Vec<BodyID>,
    /// The joint connecting each part to its parent; null for roots.
    joints: Vec<JointID>,
    /// Joints that only disable collision between overlapping parts.
    filter_joints: Vec<JointID>,
    motors_enabled: bool,
}

impl<'w> Ragdoll<'w> {
    /// Swift's `public private(set)` accessors.
    #[inline]
    pub fn bodies(&self) -> &[BodyID] {
        &self.bodies
    }

    #[inline]
    pub fn joints(&self) -> &[JointID] {
        &self.joints
    }

    #[inline]
    pub fn filter_joints(&self) -> &[JointID] {
        &self.filter_joints
    }

    /// Create the parts at `pose`'s joint matrices. Returns None when the part count does not
    /// match the skeleton.
    pub fn new(
        world: &'w mut World,
        settings: RagdollSettings,
        pose: &SkeletonPose,
        user_data: Option<*mut c_void>,
    ) -> Option<Ragdoll<'w>> {
        let count = settings.skeleton.joint_count();
        if !(settings.parts.len() == count && pose.joint_count() == count && count > 0) {
            return None;
        }
        let mut ragdoll = Self {
            settings,
            world,
            bodies: Vec::new(),
            joints: Vec::new(),
            filter_joints: Vec::new(),
            motors_enabled: false,
        };

        for i in 0..count {
            let part = &ragdoll.settings.parts[i];
            let mut def = part.body_def.clone();
            def.type_ = BodyType::DynamicBody;
            let transform = pose.world_transform(i as i32);
            def.position = transform.p;
            def.rotation = transform.q;
            def.user_data = user_data;
            let body = ragdoll.world.create_body(&def);
            let mut shape_def = part.shape_def.clone();
            if matches!(
                ragdoll.settings.self_collision,
                RagdollSelfCollision::None
            ) {
                shape_def.filter.group_index = ragdoll.settings.collision_group;
            }
            for shape in &part.shapes {
                match shape {
                    RagdollShape::Capsule(capsule) => {
                        let _ =
                            ragdoll
                                .world
                                .create_capsule_shape(body, &shape_def, *capsule);
                    }
                    RagdollShape::Sphere(sphere) => {
                        let _ = ragdoll
                            .world
                            .create_sphere_shape(body, &shape_def, *sphere);
                    }
                    RagdollShape::Box {
                        half_extents,
                        transform,
                    } => {
                        let _ = ragdoll.world.create_hull_shape(
                            body,
                            &shape_def,
                            &make_transformed_box_hull(
                                half_extents.x,
                                half_extents.y,
                                half_extents.z,
                                *transform,
                            ),
                        );
                    }
                    RagdollShape::Hull(hull) => {
                        let _ = ragdoll.world.create_hull_shape(body, &shape_def, hull);
                    }
                }
            }
            ragdoll.bodies.push(body);
        }

        if ragdoll.settings.max_mass_ratio > 1.0 {
            let max_mass_ratio = ragdoll.settings.max_mass_ratio;
            ragdoll.stabilize_masses(max_mass_ratio);
        }

        let collide_connected: bool = if matches!(
            ragdoll.settings.self_collision,
            RagdollSelfCollision::All
        ) {
            true
        } else {
            false
        };
        for i in 0..count {
            let parent = ragdoll.settings.skeleton.joints()[i].parent_index;
            if parent < 0 {
                ragdoll.joints.push(JointID::default());
                continue;
            }
            let joint = ragdoll.create_joint(i, parent, pose, collide_connected);
            ragdoll.joints.push(joint);
        }

        if let RagdollSelfCollision::SkipOverlapping { margin } =
            ragdoll.settings.self_collision
        {
            ragdoll.create_filter_joints(margin);
        }
        Some(ragdoll)
    }

    // MARK: Construction

    fn create_joint(
        &mut self,
        child: usize,
        parent: i32,
        pose: &SkeletonPose,
        collide_connected: bool,
    ) -> JointID {
        let part = &self.settings.parts[child];
        let mut def = SixDOFJointDef::new();
        def.base.body_id_a = self.bodies[parent as usize];
        def.base.body_id_b = self.bodies[child];
        def.base.collide_connected = collide_connected;
        def.base.local_frame_b = part.joint_frame;
        def.base.local_frame_a = mul_transforms(
            inv_mul_transforms(
                pose.joint_matrices[parent as usize],
                pose.joint_matrices[child],
            ),
            part.joint_frame,
        );
        def.lock_all(&[
            SixDOFAxis::RotationX,
            SixDOFAxis::RotationY,
            SixDOFAxis::RotationZ,
        ]);

        let limited = |range: ClosedRange| -> SixDOFAxisSettings {
            let mut axis = SixDOFAxisSettings::with_limits(
                SixDOFMotion::Limited,
                range.lower_bound,
                range.upper_bound,
            );
            axis.limit_hertz = part.limit_hertz;
            axis.limit_damping_ratio = part.limit_damping_ratio;
            axis
        };
        match &part.joint {
            RagdollJointKind::SwingTwist {
                cone_angle,
                twist_min,
                twist_max,
            } => {
                let (cone, twist_min, twist_max) = (*cone_angle, *twist_min, *twist_max);
                def[SixDOFAxis::RotationX] =
                    limited(ClosedRange::new(twist_min, max_float(twist_min, twist_max)));
                if cone > 0.0 {
                    def[SixDOFAxis::RotationY] = limited(ClosedRange::new(-cone, cone));
                    def[SixDOFAxis::RotationZ] = limited(ClosedRange::new(-cone, cone));
                    def.swing_limit = SixDOFSwingLimit::Cone(cone);
                } else {
                    def[SixDOFAxis::RotationY].motion = SixDOFMotion::Locked;
                    def[SixDOFAxis::RotationZ].motion = SixDOFMotion::Locked;
                }
            }
            RagdollJointKind::Pyramid {
                twist,
                swing_y,
                swing_z,
            } => {
                def[SixDOFAxis::RotationX] = limited(*twist);
                def[SixDOFAxis::RotationY] = limited(*swing_y);
                def[SixDOFAxis::RotationZ] = limited(*swing_z);
            }
            RagdollJointKind::Hinge { lower, upper } => {
                let (lower, upper) = (*lower, *upper);
                def[SixDOFAxis::RotationX] = limited(ClosedRange::new(lower, max_float(lower, upper)));
                def[SixDOFAxis::RotationY].motion = SixDOFMotion::Locked;
                def[SixDOFAxis::RotationZ].motion = SixDOFMotion::Locked;
            }
            RagdollJointKind::Fixed => {
                def.lock_all(&[]);
            }
        }
        for axis in [
            SixDOFAxis::RotationX,
            SixDOFAxis::RotationY,
            SixDOFAxis::RotationZ,
        ] {
            def[axis].max_friction = part.joint_friction;
        }
        self.world.create_six_dof_joint(def)
    }

    /// Clamp the mass ratio across each joint, parents first, then restore the total mass.
    /// Inertia scales with mass, so each part keeps its shape of inertia.
    fn stabilize_masses(&mut self, max_ratio: f32) {
        let mut masses: Vec<MassData> = Vec::with_capacity(self.bodies.len());
        for body in &self.bodies {
            masses.push(self.world.body_mass_data(*body));
        }
        let total: f32 = masses.iter().fold(0.0, |acc, m| acc + m.mass);
        if !(total > 0.0) {
            return;
        }
        let mut changed = false;
        for i in 0..self.bodies.len() {
            let parent = self.settings.skeleton.joints()[i].parent_index;
            if parent >= 0
                && masses[parent as usize].mass > 0.0
                && masses[i].mass > 0.0
            {
                let ratio = masses[i].mass / masses[parent as usize].mass;
                let clamped = clamp_float(ratio, 1.0 / max_ratio, max_ratio);
                if clamped != ratio {
                    let scale = clamped / ratio;
                    masses[i].mass *= scale;
                    masses[i].inertia = mul_sm(scale, masses[i].inertia);
                    changed = true;
                }
            }
        }
        if changed == false {
            return;
        }
        let scale = total / masses.iter().fold(0.0, |acc, m| acc + m.mass);
        for i in 0..self.bodies.len() {
            masses[i].mass *= scale;
            masses[i].inertia = mul_sm(scale, masses[i].inertia);
            self.world
                .set_body_mass_data(self.bodies[i], masses[i]);
        }
    }

    /// Filter joints between unconnected parts whose bounds overlap.
    fn create_filter_joints(&mut self, margin: f32) {
        let mut boxes: Vec<AABB> = Vec::with_capacity(self.bodies.len());
        for body in &self.bodies {
            boxes.push(aabb_inflate(
                self.world.body_compute_aabb(*body),
                margin,
            ));
        }
        let skeleton = &self.settings.skeleton;
        for i in 0..self.bodies.len() {
            for j in (i + 1)..self.bodies.len() {
                if skeleton.joints()[j].parent_index == i as i32
                    || skeleton.joints()[i].parent_index == j as i32
                {
                    continue;
                }
                if aabb_overlaps(boxes[i], boxes[j]) == false {
                    continue;
                }
                let mut def = FilterJointDef::default();
                def.base.body_id_a = self.bodies[i];
                def.base.body_id_b = self.bodies[j];
                let joint = self.world.create_filter_joint(&def);
                self.filter_joints.push(joint);
            }
        }
    }

    /// Destroy every body and joint of the ragdoll.
    pub fn destroy(&mut self) {
        for body in self.bodies.iter() {
            if self.world.body_is_valid(*body) {
                self.world.destroy_body(*body);
            }
        }
        self.bodies.clear();
        self.joints.clear();
        self.filter_joints.clear();
    }

    // MARK: Pose

    pub fn root_body(&self) -> BodyID {
        self.bodies[0]
    }

    /// Current pose, with the root body's position as the root offset.
    pub fn pose(&self) -> SkeletonPose {
        let mut pose = SkeletonPose::new(self.settings.skeleton.clone());
        pose.root_offset = self.world.body_position(self.bodies[0]);
        for i in 0..self.bodies.len() {
            pose.set_world_transform(i as i32, self.world.body_transform(self.bodies[i]));
        }
        pose.calculate_joint_states();
        pose
    }

    /// Teleport the parts to a pose. Velocities are kept unless `reset_velocities`.
    pub fn set_pose(&mut self, pose: &SkeletonPose, reset_velocities: bool) {
        for i in 0..self.bodies.len() {
            let transform = pose.world_transform(i as i32);
            self.world
                .set_body_transform(self.bodies[i], transform.p, transform.q);
            if reset_velocities {
                self.world
                    .set_body_linear_velocity(self.bodies[i], Vec3::default());
                self.world
                    .set_body_angular_velocity(self.bodies[i], Vec3::default());
            }
        }
    }

    /// Drive each joint toward the pose's relative rotations with position motors. The parts stay
    /// dynamic, so the ragdoll follows the animation while it reacts to contacts and gravity.
    /// Swift defaulted `hertz = 8` and `dampingRatio = 1`; ported callers pass them explicitly.
    pub fn drive_to_pose_using_motors(
        &mut self,
        pose: &SkeletonPose,
        max_torque: f32,
        hertz: f32,
        damping_ratio: f32,
    ) {
        for i in 0..self.bodies.len() {
            let parent = self.settings.skeleton.joints()[i].parent_index;
            if parent < 0 {
                continue;
            }
            if self.world.joint_is_valid(self.joints[i]) == false {
                continue;
            }
            let local_frame_a = match self.joint_local_frame_a(i) {
                Some(frame) => frame,
                None => continue,
            };
            // Frame B relative to frame A in the target pose.
            let q_a = mul_quat(pose.joint_matrices[parent as usize].q, local_frame_a.q);
            let q_b = mul_quat(
                pose.joint_matrices[i].q,
                self.settings.parts[i].joint_frame.q,
            );
            let target_b = inv_mul_quat(q_a, q_b);
            for axis in [
                SixDOFAxis::RotationX,
                SixDOFAxis::RotationY,
                SixDOFAxis::RotationZ,
            ] {
                if let Some(mut settings) = self.world.six_dof_joint_axis(self.joints[i], axis) {
                    settings.motor_mode = SixDOFMotorMode::Position;
                    settings.max_motor_force = max_torque;
                    settings.motor_hertz = hertz;
                    settings.motor_damping_ratio = damping_ratio;
                    self.world
                        .set_six_dof_joint_axis(self.joints[i], axis, settings);
                }
            }
            self.world.set_six_dof_joint_target_rotation(
                self.joints[i],
                normalize_quat(target_b),
            );
        }
        self.motors_enabled = true;
    }

    /// Turn the joint motors off again.
    pub fn disable_motors(&mut self) {
        if self.motors_enabled == false {
            return;
        }
        for i in 0..self.joints.len() {
            if self.world.joint_is_valid(self.joints[i]) == false {
                continue;
            }
            for axis in [
                SixDOFAxis::RotationX,
                SixDOFAxis::RotationY,
                SixDOFAxis::RotationZ,
            ] {
                if let Some(mut settings) = self.world.six_dof_joint_axis(self.joints[i], axis) {
                    settings.motor_mode = SixDOFMotorMode::Off;
                    self.world
                        .set_six_dof_joint_axis(self.joints[i], axis, settings);
                }
            }
        }
        self.motors_enabled = false;
    }

    /// Set every part's velocity so it reaches the pose in `time_step` (Jolt's
    /// DriveToPoseUsingKinematics). Call before each step. Swift's
    /// `setBodyTargetTransform(wake: true)` default is passed explicitly.
    pub fn drive_to_pose_using_kinematics(&mut self, pose: &SkeletonPose, time_step: f32) {
        for i in 0..self.bodies.len() {
            self.world.set_body_target_transform(
                self.bodies[i],
                pose.world_transform(i as i32),
                time_step,
                true,
            );
        }
    }

    fn joint_local_frame_a(&self, part: usize) -> Option<Transform> {
        let sim = self.world.joint_sim(self.joints[part])?;
        Some(sim.local_frame_a)
    }

    // MARK: Whole body

    pub fn set_linear_velocity(&mut self, velocity: Vec3) {
        for body in self.bodies.iter() {
            self.world.set_body_linear_velocity(*body, velocity);
        }
    }

    /// Apply an impulse spread over the parts by mass, so every part gains the same velocity.
    pub fn apply_linear_impulse(&mut self, impulse: Vec3) {
        let total = self.total_mass();
        if !(total > 0.0) {
            return;
        }
        for body in self.bodies.iter() {
            let mass_fraction = self.world.body_mass(*body) / total;
            self.world
                .apply_body_linear_impulse_to_center(*body, mass_fraction * impulse, true);
        }
    }

    pub fn total_mass(&self) -> f32 {
        self.bodies
            .iter()
            .fold(0.0, |acc, body| acc + self.world.body_mass(*body))
    }

    pub fn is_awake(&self) -> bool {
        self.bodies
            .iter()
            .any(|body| self.world.body_is_awake(*body))
    }

    pub fn wake(&mut self) {
        for body in self.bodies.iter() {
            self.world.set_body_awake(*body, true);
        }
    }

    /// Bounds of all parts.
    pub fn bounds(&self) -> AABB {
        let mut box_ = self.world.body_compute_aabb(self.bodies[0]);
        for body in self.bodies.iter().skip(1) {
            box_ = aabb_union(box_, self.world.body_compute_aabb(*body));
        }
        box_
    }
}
