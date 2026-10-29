//! Maps poses between a low detail skeleton (usually a ragdoll's) and a high detail one (usually
//! an animation's), as Jolt's SkeletonMapper does. Joints with the same name are mapped through the
//! offset between the two neutral poses. High detail joints between two mapped joints form a chain:
//! they keep their animated local transforms and the chain is turned about its start so its end
//! meets the mapped end joint. Other high detail joints follow their parents with animated locals.
//!
//! Translated from `Sources/Natsel/Skeleton/SkeletonMapper.swift`.

use crate::math::functions::{
    compute_quat_between_unit_vectors, invert_transform, inv_mul_transforms, length_squared,
    mul_quat, mul_transforms, normalize, normalize_quat, rotate_vector,
};
use crate::math::types::{Quat, Transform};
use crate::skeleton::skeleton::{Skeleton, SkeletonPose};
use std::collections::HashMap;

/// Swift's nested `SkeletonMapper.Mapping`.
#[derive(Clone, Copy, Debug, PartialEq)]
#[repr(C)]
pub struct Mapping {
    pub low_index: i32,
    pub high_index: i32,
    /// High detail joint relative to the low detail joint in the neutral poses.
    pub offset: Transform,
}

/// Swift's nested `SkeletonMapper.Chain`.
#[derive(Clone, Debug, PartialEq)]
pub struct Chain {
    /// High detail joints from the mapped start to the mapped end, both included.
    pub high_indices: Vec<i32>,
}

pub struct SkeletonMapper {
    pub low: Skeleton,
    pub high: Skeleton,
    mappings: Vec<Mapping>,
    chains: Vec<Chain>,
    /// High detail joints that are neither mapped nor inside a chain.
    unmapped: Vec<i32>,
    high_to_mapping: Vec<i32>,
    high_in_chain: Vec<bool>,
}

impl SkeletonMapper {
    /// `low_neutral` and `high_neutral` are model space joint matrices of the two skeletons in the
    /// same neutral pose. Returns `None` when no joint names match.
    pub fn new(
        low: Skeleton,
        low_neutral: &[Transform],
        high: Skeleton,
        high_neutral: &[Transform],
    ) -> Option<Self> {
        if low_neutral.len() != low.joint_count() || high_neutral.len() != high.joint_count() {
            return None;
        }
        let high_joint_count = high.joint_count();
        let mut mapper = SkeletonMapper {
            low,
            high,
            mappings: Vec::new(),
            chains: Vec::new(),
            unmapped: Vec::new(),
            high_to_mapping: vec![-1i32; high_joint_count],
            high_in_chain: Vec::new(),
        };
        for (low_index, joint) in mapper.low.joints().iter().enumerate() {
            let Some(high_index) = mapper.high.joint_index(&joint.name) else {
                continue;
            };
            mapper.high_to_mapping[high_index as usize] = mapper.mappings.len() as i32;
            mapper.mappings.push(Mapping {
                low_index: low_index as i32,
                high_index,
                offset: inv_mul_transforms(
                    low_neutral[low_index],
                    high_neutral[high_index as usize],
                ),
            });
        }
        if mapper.mappings.is_empty() {
            return None;
        }

        // A chain runs from a mapped joint's nearest mapped ancestor through unmapped joints.
        mapper.high_in_chain = vec![false; high_joint_count];
        for mapping_index in 0..mapper.mappings.len() {
            let mapping_high_index = mapper.mappings[mapping_index].high_index;
            let mut path: Vec<i32> = Vec::new();
            let mut current =
                mapper.high.joints()[mapping_high_index as usize].parent_index;
            while current >= 0 && mapper.high_to_mapping[current as usize] < 0 {
                path.push(current);
                current = mapper.high.joints()[current as usize].parent_index;
            }
            if current < 0 || path.is_empty() {
                continue;
            }
            let mut indices: Vec<i32> = Vec::with_capacity(path.len() + 2);
            indices.push(current);
            indices.extend(path.iter().rev().copied());
            indices.push(mapping_high_index);
            mapper.chains.push(Chain {
                high_indices: indices,
            });
            for index in &path {
                mapper.high_in_chain[*index as usize] = true;
            }
        }
        mapper.unmapped = (0..high_joint_count as i32)
            .filter(|&index| {
                mapper.high_to_mapping[index as usize] < 0 && !mapper.high_in_chain[index as usize]
            })
            .collect();
        Some(mapper)
    }

    /// Swift's `public private(set)` accessors.
    #[inline]
    pub fn mappings(&self) -> &[Mapping] {
        &self.mappings
    }

    #[inline]
    pub fn chains(&self) -> &[Chain] {
        &self.chains
    }

    #[inline]
    pub fn unmapped(&self) -> &[i32] {
        &self.unmapped
    }

    /// Fill the high detail pose's joint matrices from the low detail pose's joint matrices.
    /// `animated` supplies local transforms for chains and unmapped joints. Joint states of
    /// `high_pose` are recomputed.
    pub fn map(&self, low_pose: &SkeletonPose, animated: &SkeletonPose, high_pose: &mut SkeletonPose) {
        high_pose.root_offset = low_pose.root_offset;
        let high_joint_count = self.high.joint_count();
        let mut done = vec![false; high_joint_count];
        for mapping in &self.mappings {
            high_pose.joint_matrices[mapping.high_index as usize] = mul_transforms(
                low_pose.joint_matrices[mapping.low_index as usize],
                mapping.offset,
            );
            done[mapping.high_index as usize] = true;
        }

        // Joints in skeleton order: a chain is handled when its first inner joint comes up, at
        // which point its start (an ancestor) is final.
        let chain_by_first_inner: HashMap<i32, usize> = self
            .chains
            .iter()
            .enumerate()
            .map(|(chain_index, chain)| (chain.high_indices[1], chain_index))
            .collect();
        for index in 0..high_joint_count {
            if done[index] {
                continue;
            }
            let index_i = index as i32;
            if let Some(&chain_index) = chain_by_first_inner.get(&index_i) {
                let chain = &self.chains[chain_index];
                self.map_chain(chain, animated, high_pose);
                for inner in &chain.high_indices[1..chain.high_indices.len() - 1] {
                    done[*inner as usize] = true;
                }
                continue;
            }
            if self.high_in_chain[index] {
                continue;
            }
            let parent = self.high.joints()[index].parent_index;
            let local = animated.joint_states[index].transform();
            high_pose.joint_matrices[index] = if parent < 0 {
                local
            } else {
                mul_transforms(high_pose.joint_matrices[parent as usize], local)
            };
            done[index] = true;
        }
        high_pose.calculate_joint_states();
    }

    fn map_chain(&self, chain: &Chain, animated: &SkeletonPose, pose: &mut SkeletonPose) {
        let indices = &chain.high_indices;
        let start = pose.joint_matrices[indices[0] as usize];
        // Lay the inner joints out with the animated locals from the start.
        let mut matrices: Vec<Transform> = Vec::with_capacity(indices.len());
        matrices.push(start);
        for k in 1..indices.len() {
            matrices.push(mul_transforms(
                matrices[k - 1],
                animated.joint_states[indices[k] as usize].transform(),
            ));
        }
        // Turn the chain about its start so the animated end meets the mapped end.
        let target = pose.joint_matrices[indices[indices.len() - 1] as usize].p - start.p;
        let current = matrices[matrices.len() - 1].p - start.p;
        let mut turn = Quat::IDENTITY;
        if length_squared(target) > 1.0e-12 && length_squared(current) > 1.0e-12 {
            turn = compute_quat_between_unit_vectors(normalize(current), normalize(target));
        }
        for k in 1..(indices.len() - 1) {
            let relative = matrices[k].p - start.p;
            pose.joint_matrices[indices[k] as usize] = Transform::new(
                start.p + rotate_vector(turn, relative),
                normalize_quat(mul_quat(turn, matrices[k].q)),
            );
        }
    }

    /// Fill the low detail pose's joint matrices from the high detail pose's. Joints of the low
    /// skeleton without a match keep their transforms relative to their parents.
    pub fn map_reverse(&self, high_pose: &SkeletonPose, low_pose: &mut SkeletonPose) {
        low_pose.root_offset = high_pose.root_offset;
        let low_joint_count = self.low.joint_count();
        let mut mapped = vec![false; low_joint_count];
        for mapping in &self.mappings {
            low_pose.joint_matrices[mapping.low_index as usize] = mul_transforms(
                high_pose.joint_matrices[mapping.high_index as usize],
                invert_transform(mapping.offset),
            );
            mapped[mapping.low_index as usize] = true;
        }
        for index in 0..low_joint_count {
            if mapped[index] {
                continue;
            }
            let parent = self.low.joints()[index].parent_index;
            let local = low_pose.joint_states[index].transform();
            low_pose.joint_matrices[index] = if parent < 0 {
                local
            } else {
                mul_transforms(low_pose.joint_matrices[parent as usize], local)
            };
        }
        low_pose.calculate_joint_states();
    }
}
