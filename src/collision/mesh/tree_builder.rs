//! Mesh bounding volume hierarchy construction.

use crate::collision::internal_math::bounds3_empty;
use crate::math::functions::*;
use crate::math::types::{AABB, Vec3};
use crate::types::geometry::MeshNode;

use super::build::split_index;

pub struct MeshPrimitive {
    pub bounds: AABB,
    pub center: Vec3,
    pub triangle_index: usize,
}

pub struct MeshTreeBuild {
    pub nodes: Vec<MeshNode>,
    pub height: usize,
}

impl MeshTreeBuild {
    pub fn new(primitives: &mut [MeshPrimitive]) -> Self {
        assert!(!primitives.is_empty());
        let mut builder = Self {
            nodes: Vec::with_capacity(2 * primitives.len() - 1),
            height: 0,
        };
        let mut height = 0;
        let _ = builder.build(primitives, 0, primitives.len(), &mut height);
        builder.height = height;
        builder
    }

    fn build(&mut self, primitives: &mut [MeshPrimitive], start: usize, end: usize, height: &mut usize) -> usize {
        if end - start > 4 {
            let (axis, split) = split_index(primitives, start, end);
            let index = self.nodes.len();
            self.nodes.push(MeshNode::default());
            let middle = start + split;
            let mut left_height = 0;
            let mut right_height = 0;
            let left = self.build(primitives, start, middle, &mut left_height);
            let right = self.build(primitives, middle, end, &mut right_height);
            *height = left_height.max(right_height) + 1;
            debug_assert!(left - index == 1 && right - index > 1);
            let bounds = aabb_union(
                self.nodes[left].bounds(),
                self.nodes[right].bounds(),
            );
            self.nodes[index].data = axis as u32 | ((right - index) as u32) << 2;
            self.nodes[index].lower_bound = bounds.lower_bound;
            self.nodes[index].upper_bound = bounds.upper_bound;
            self.nodes[index].triangle_offset = 0;
            index
        } else {
            let mut bounds = bounds3_empty();
            for p in &primitives[start..end] {
                bounds = aabb_union(bounds, p.bounds);
            }
            let mut node = MeshNode::default();
            node.lower_bound = bounds.lower_bound;
            node.upper_bound = bounds.upper_bound;
            node.data = 3 | ((end - start) as u32) << 2;
            node.triangle_offset = start as u32;
            let index = self.nodes.len();
            self.nodes.push(node);
            *height = 1;
            index
        }
    }

    /// The traversal stack must handle 254 pending nodes.
    pub fn fits_traversal_stack(&self) -> bool {
        let mut stack = vec![0usize];
        while let Some(index) = stack.pop() {
            let node = self.nodes[index];
            if !node.is_leaf() {
                if stack.len() >= 254 {
                    return false;
                }
                stack.push(index + node.child_offset());
                stack.push(index + 1);
            }
        }
        true
    }
}

trait NodeBounds {
    fn bounds(&self) -> AABB;
}

impl NodeBounds for MeshNode {
    fn bounds(&self) -> AABB {
        AABB::new(self.lower_bound, self.upper_bound)
    }
}
