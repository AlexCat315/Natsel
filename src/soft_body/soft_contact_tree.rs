//! A refittable feature BVH. Swept faces and edges appear once even when their motion spans
//! many spatial cells. Reuses the mesh builder and keeps the original feature indices.
//!
//! Translated from `Sources/Natsel/SoftBody/SoftContactTree.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"): the landed `crate::collision::mesh::tree_builder::{MeshPrimitive,
//! MeshTreeBuild}` and `crate::types::geometry::MeshNode` (whose `is_leaf` / `child_offset` /
//! `leaf_triangle_count` accessors live in `crate::collision::mesh::layout`).
//!
//! Notes for callers (`soft_body_contacts.rs` and the other soft-body passes):
//! - Swift's `refit(_ bounds: [AABB], referenceBounds: () -> [AABB] = { [] })` took the box array by
//!   value with copy-on-write semantics, so the caller could keep using its own `boxes` afterwards.
//!   Rust takes a slice and stores an owned copy, giving the caller the same freedom; the
//!   `referenceBounds` closure has no default, so ported callers pass it explicitly (they all do in
//!   Swift). Returning a vector whose length differs from `bounds` disables the rest-space
//!   exclusion exactly as in Swift.
//! - Swift's `visit(_:afterFeature:referenceExclusion:_:)` defaults (`afterFeature = -1`,
//!   `referenceExclusion = nil`) are passed explicitly here; `referenceExclusion` is the labelled
//!   tuple `(center: Vec3, radiusSquared: Float)` as `Option<(Vec3, f32)>`.
//! - Rebuild/refit index order, the reverse sweeps, the escape/maximum-feature tables, and the
//!   traversal and callback order are unchanged from Swift.

use crate::collision::internal_math::bounds3_empty;
use crate::collision::mesh::tree_builder::{MeshPrimitive, MeshTreeBuild};
use crate::math::functions::{
    aabb_center, aabb_overlaps, aabb_union, abs_vec3, length_squared, max_vec3,
};
use crate::math::types::{AABB, Vec3};
use crate::types::geometry::MeshNode;

/// A refittable BVH over swept feature boxes (faces or edges), keyed by feature index.
#[derive(Clone, Debug, Default)]
pub struct SoftContactTree {
    nodes: Vec<MeshNode>,
    features: Vec<i32>,
    escape: Vec<usize>,
    maximum_feature: Vec<i32>,
    boxes: Vec<AABB>,
    reference_nodes: Vec<AABB>,
}

impl SoftContactTree {
    pub fn refit<F: FnOnce() -> Vec<AABB>>(&mut self, bounds: &[AABB], reference_bounds: F) {
        let rebuild = bounds.len() != self.boxes.len() || self.nodes.is_empty();
        self.boxes = bounds.to_vec();
        if bounds.is_empty() {
            // Swift's `removeAll()`; `Vec::clear` empties the same buffers and only differs in
            // keeping the allocation, which is unobservable to the traversal below.
            self.nodes.clear();
            self.features.clear();
            self.escape.clear();
            self.maximum_feature.clear();
            self.reference_nodes.clear();
            return;
        }
        if rebuild {
            let mut primitives: Vec<MeshPrimitive> = Vec::with_capacity(bounds.len());
            for (index, element) in bounds.iter().enumerate() {
                primitives.push(MeshPrimitive {
                    bounds: *element,
                    center: aabb_center(*element),
                    triangle_index: index,
                });
            }
            let built = MeshTreeBuild::new(&mut primitives);
            self.nodes = built.nodes;
            self.features = primitives.iter().map(|p| p.triangle_index as i32).collect();
            self.escape = vec![0usize; self.nodes.len()];
            self.maximum_feature = vec![-1i32; self.nodes.len()];
            let nodes = &self.nodes;
            let features = &self.features;
            let escape = &mut self.escape;
            let maximum_feature = &mut self.maximum_feature;
            for index in (0..nodes.len()).rev() {
                let node = nodes[index];
                escape[index] = if node.is_leaf() {
                    index + 1
                } else {
                    escape[index + node.child_offset()]
                };
                if node.is_leaf() {
                    let start = node.triangle_offset as usize;
                    for slot in start..(start + node.leaf_triangle_count()) {
                        maximum_feature[index] = maximum_feature[index].max(features[slot]);
                    }
                } else {
                    maximum_feature[index] = maximum_feature[index + 1]
                        .max(maximum_feature[index + node.child_offset()]);
                }
            }
            let reference = reference_bounds();
            self.reference_nodes.clear();
            if reference.len() == bounds.len() {
                self.reference_nodes = vec![bounds3_empty(); self.nodes.len()];
                let nodes = &self.nodes;
                let features = &self.features;
                let reference_nodes = &mut self.reference_nodes;
                for index in (0..nodes.len()).rev() {
                    let node = nodes[index];
                    if node.is_leaf() {
                        let start = node.triangle_offset as usize;
                        let mut box_ = bounds3_empty();
                        for slot in start..(start + node.leaf_triangle_count()) {
                            box_ = aabb_union(box_, reference[features[slot] as usize]);
                        }
                        reference_nodes[index] = box_;
                    } else {
                        reference_nodes[index] = aabb_union(
                            reference_nodes[index + 1],
                            reference_nodes[index + node.child_offset()],
                        );
                    }
                }
            }
            return;
        }
        let nodes = &mut self.nodes;
        let features = &self.features;
        let bounds = &self.boxes;
        for index in (0..nodes.len()).rev() {
            let node = nodes[index];
            let mut box_ = bounds3_empty();
            if node.is_leaf() {
                let start = node.triangle_offset as usize;
                for slot in start..(start + node.leaf_triangle_count()) {
                    box_ = aabb_union(box_, bounds[features[slot] as usize]);
                }
            } else {
                let left = nodes[index + 1];
                let right = nodes[index + node.child_offset()];
                box_ = aabb_union(
                    AABB::new(left.lower_bound, left.upper_bound),
                    AABB::new(right.lower_bound, right.upper_bound),
                );
            }
            nodes[index].lower_bound = box_.lower_bound;
            nodes[index].upper_bound = box_.upper_bound;
        }
    }

    #[inline(always)]
    pub fn visit<F>(
        &self,
        query: AABB,
        after_feature: i32,
        reference_exclusion: Option<(Vec3, f32)>,
        mut body: F,
    ) where
        F: FnMut(i32),
    {
        let mut index = 0usize;
        while index < self.nodes.len() {
            let node = self.nodes[index];
            if self.maximum_feature[index] <= after_feature {
                index = self.escape[index];
                continue;
            }
            if !aabb_overlaps(query, AABB::new(node.lower_bound, node.upper_bound)) {
                index = self.escape[index];
                continue;
            }
            // Swift's `!node.isLeaf, let exclusion = referenceExclusion,
            // referenceNodes.count == nodes.count` chain; spelled as nested conditions because
            // `let` chains need a newer compiler than the crate's `rust-version = 1.85`. The
            // fall-through behaviour is unchanged: none of these guards do anything but skip.
            if !node.is_leaf() {
                if let Some((center, radius_squared)) = reference_exclusion {
                    if self.reference_nodes.len() == self.nodes.len() {
                        let reference = self.reference_nodes[index];
                        let farthest = max_vec3(
                            abs_vec3(reference.lower_bound - center),
                            abs_vec3(reference.upper_bound - center),
                        );
                        // Every point in this rest-space node is already excluded. Keep a roundoff
                        // margin at the exclusion boundary; folded surfaces with distant rest
                        // positions survive.
                        if length_squared(farthest) < radius_squared * (1.0 - 16.0 * f32::EPSILON)
                        {
                            index = self.escape[index];
                            continue;
                        }
                    }
                }
            }
            if node.is_leaf() {
                let start = node.triangle_offset as usize;
                for slot in start..(start + node.leaf_triangle_count()) {
                    let feature = self.features[slot];
                    if feature > after_feature
                        && aabb_overlaps(query, self.boxes[feature as usize])
                    {
                        body(feature);
                    }
                }
            }
            index += 1;
        }
    }
}
