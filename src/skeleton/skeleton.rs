//! Skeletons and poses (Jolt's Skeleton and SkeletonPose). A skeleton is a list of named joints,
//! each after its parent. A pose holds every joint's transform relative to its parent (the joint
//! states) and in model space (the joint matrices); model space is relative to the pose's root
//! offset, so world transforms are `root_offset` plus the joint matrix.
//!
//! Translated from `Sources/Natsel/Skeleton/Skeleton.swift`.

use crate::base::core::internal_assert;
use crate::math::functions::{
    atan2_approx, compute_cos_sin, dot_quat, inv_mul_transforms, mul_transforms, negate_quat,
    nlerp, normalize_quat, offset_pos, sub_pos,
};
use crate::math::types::{Pos, Quat, Transform, Vec3, WorldTransform};
use std::collections::HashMap;

/// A named joint in the skeleton hierarchy. Swift's nested `Skeleton.Joint`; Rust has no nested
/// types, so it sits at module level in this file.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Joint {
    pub name: String,
    /// Index of the parent joint, or -1 for a root.
    pub parent_index: i32,
}

impl Joint {
    pub fn new(name: String, parent_index: i32) -> Self {
        Self { name, parent_index }
    }
}

#[derive(Clone, Debug)]
pub struct Skeleton {
    joints: Vec<Joint>,
    indices: HashMap<String, i32>,
}

impl Default for Skeleton {
    fn default() -> Self {
        Self {
            joints: Vec::new(),
            indices: HashMap::new(),
        }
    }
}

impl Skeleton {
    pub fn new() -> Self {
        Self::default()
    }

    /// Swift's `private(set) var joints`.
    #[inline]
    pub fn joints(&self) -> &[Joint] {
        &self.joints
    }

    #[inline]
    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Add a joint after its parent. Returns its index. Names must be unique.
    pub fn add_joint(&mut self, name: &str, parent: i32) -> i32 {
        if !((parent >= -1) && ((parent as i64) < self.joints.len() as i64)) {
            internal_assert("a joint must follow its parent", file!(), line!());
        }
        if self.indices.contains_key(name) {
            internal_assert(
                &format!("duplicate joint name {name}"),
                file!(),
                line!(),
            );
        }
        self.indices.insert(name.to_owned(), self.joints.len() as i32);
        self.joints.push(Joint::new(name.to_owned(), parent));
        self.joints.len() as i32 - 1
    }

    /// Swift's `addJoint(_ name:parentName:)` overload.
    pub fn add_joint_with_parent_name(&mut self, name: &str, parent_name: &str) -> i32 {
        let parent = match self.indices.get(parent_name) {
            Some(parent) => *parent,
            None => internal_assert(
                &format!("unknown parent {parent_name}"),
                file!(),
                line!(),
            ),
        };
        self.add_joint(name, parent)
    }

    pub fn joint_index(&self, name: &str) -> Option<i32> {
        self.indices.get(name).copied()
    }

    pub fn parent_index(&self, joint: i32) -> i32 {
        self.joints[joint as usize].parent_index
    }

    /// True when `ancestor` is on the parent chain of `joint` (not counting the joint itself).
    pub fn is_ancestor(&self, ancestor: i32, joint: i32) -> bool {
        let mut current = self.joints[joint as usize].parent_index;
        while current >= 0 {
            if current == ancestor {
                return true;
            }
            current = self.joints[current as usize].parent_index;
        }
        false
    }
}

/// A joint transform relative to its parent.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct SkeletonJointState {
    pub translation: Vec3,
    pub rotation: Quat,
}

impl Default for SkeletonJointState {
    fn default() -> Self {
        Self {
            translation: Vec3::default(),
            rotation: Quat::IDENTITY,
        }
    }
}

impl SkeletonJointState {
    pub fn new(translation: Vec3, rotation: Quat) -> Self {
        Self {
            translation,
            rotation,
        }
    }

    pub fn from_transform(transform: Transform) -> Self {
        Self {
            translation: transform.p,
            rotation: transform.q,
        }
    }

    #[inline]
    pub fn transform(&self) -> Transform {
        Transform::new(self.translation, self.rotation)
    }
}

#[derive(Clone, Debug)]
pub struct SkeletonPose {
    pub skeleton: Skeleton,
    /// World position of the model space origin.
    pub root_offset: Pos,
    /// Joint transforms relative to their parents.
    pub joint_states: Vec<SkeletonJointState>,
    /// Joint transforms in model space.
    pub joint_matrices: Vec<Transform>,
}

impl SkeletonPose {
    pub fn new(skeleton: Skeleton) -> Self {
        let joint_count = skeleton.joint_count();
        Self {
            skeleton,
            root_offset: Pos::default(),
            joint_states: vec![SkeletonJointState::default(); joint_count],
            joint_matrices: vec![Transform::IDENTITY; joint_count],
        }
    }

    #[inline]
    pub fn joint_count(&self) -> usize {
        self.joint_states.len()
    }

    /// Joint matrices from the joint states.
    pub fn calculate_joint_matrices(&mut self) {
        for i in 0..self.joint_states.len() {
            let local = self.joint_states[i].transform();
            let parent = self.skeleton.joints()[i].parent_index;
            self.joint_matrices[i] = if parent < 0 {
                local
            } else {
                mul_transforms(self.joint_matrices[parent as usize], local)
            };
        }
    }

    /// Joint states from the joint matrices.
    pub fn calculate_joint_states(&mut self) {
        for i in 0..self.joint_matrices.len() {
            let parent = self.skeleton.joints()[i].parent_index;
            let local = if parent < 0 {
                self.joint_matrices[i]
            } else {
                inv_mul_transforms(
                    self.joint_matrices[parent as usize],
                    self.joint_matrices[i],
                )
            };
            self.joint_states[i] =
                SkeletonJointState::from_transform(Transform::new(local.p, normalize_quat(local.q)));
        }
    }

    pub fn world_transform(&self, joint: i32) -> WorldTransform {
        let model = self.joint_matrices[joint as usize];
        WorldTransform::new(offset_pos(self.root_offset, model.p), model.q)
    }

    pub fn set_world_transform(&mut self, joint: i32, transform: WorldTransform) {
        self.joint_matrices[joint as usize] =
            Transform::new(sub_pos(transform.p, self.root_offset), transform.q);
    }
}

/// Spherical interpolation along the shorter arc; falls back to nlerp for nearly equal rotations.
/// Swift's file-internal `slerpQuat`; also used by `skeletal_animation`.
pub(crate) fn slerp_quat(q1: Quat, q2: Quat, t: f32) -> Quat {
    let mut q2 = q2;
    let mut cosine = dot_quat(q1, q2);
    if cosine < 0.0 {
        q2 = negate_quat(q2);
        cosine = -cosine;
    }
    if cosine > 0.9995 {
        return nlerp(q1, q2, t);
    }
    // The engine's own trigonometry keeps results identical across platforms.
    let sine = (1.0 - cosine * cosine).sqrt();
    let angle = atan2_approx(sine, cosine);
    let a = compute_cos_sin((1.0 - t) * angle).sine / sine;
    let b = compute_cos_sin(t * angle).sine / sine;
    normalize_quat(Quat::new(
        a * q1.v + b * q2.v,
        a * q1.s + b * q2.s,
    ))
}
