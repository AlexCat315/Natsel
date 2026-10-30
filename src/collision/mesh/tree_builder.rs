//! Mesh bounding volume hierarchy construction.

use crate::collision::internal_math::{bounds3_empty, get_by_index, major_axis};
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
    /// Swift's `init(primitives:useMedianSplit:)`. `use_median_split` selects the median split;
    /// otherwise the binned-SAH split is used, falling back to a half split when SAH finds no
    /// usable axis.
    pub fn new(primitives: &mut [MeshPrimitive]) -> Self {
        Self::with_options(primitives, false)
    }

    pub fn with_options(primitives: &mut [MeshPrimitive], use_median_split: bool) -> Self {
        assert!(!primitives.is_empty());
        let mut builder = Self {
            nodes: Vec::with_capacity(2 * primitives.len() - 1),
            height: 0,
        };
        let mut height = 0;
        let _ = builder.build(primitives, 0, primitives.len(), use_median_split, &mut height);
        builder.height = height;
        builder
    }

    fn build(
        &mut self,
        primitives: &mut [MeshPrimitive],
        start: usize,
        end: usize,
        use_median_split: bool,
        height: &mut usize,
    ) -> usize {
        if end - start > 4 {
            let mut split = if use_median_split {
                split_median(primitives, start, end)
            } else {
                split_binned_sah(primitives, start, end)
            };
            if split.axis < 0 {
                if end - start > 8 {
                    split = split_half(primitives, start, end);
                } else {
                    // Swift returns a leaf here without touching the caller's zero height.
                    return self.store_leaf(primitives, start, end);
                }
            }
            let (axis, split_index) = (split.axis, split.index);
            let index = self.nodes.len();
            self.nodes.push(MeshNode::default());
            let middle = start + split_index;
            let mut left_height = 0;
            let mut right_height = 0;
            let left = self.build(primitives, start, middle, use_median_split, &mut left_height);
            let right = self.build(primitives, middle, end, use_median_split, &mut right_height);
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
            let index = self.store_leaf(primitives, start, end);
            *height = 1;
            index
        }
    }

    fn store_leaf(&mut self, primitives: &[MeshPrimitive], start: usize, end: usize) -> usize {
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
        index
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

/// Swift's `MeshSplit`.
struct MeshSplit {
    left_bounds: AABB,
    right_bounds: AABB,
    axis: i32,
    index: usize,
}

impl MeshSplit {
    fn empty() -> Self {
        Self {
            left_bounds: bounds3_empty(),
            right_bounds: bounds3_empty(),
            axis: -1,
            index: usize::MAX,
        }
    }
}

/// Swift's `meshSplitBounds(_:range:index:)`.
fn mesh_split_bounds(primitives: &[MeshPrimitive], start: usize, end: usize, index: usize) -> MeshSplit {
    let mut split = MeshSplit::empty();
    split.index = index;
    for i in start..(start + index) {
        split.left_bounds = aabb_union(split.left_bounds, primitives[i].bounds);
    }
    for i in (start + index)..end {
        split.right_bounds = aabb_union(split.right_bounds, primitives[i].bounds);
    }
    split
}

/// Swift's `splitMeshHalf(_:range:)`.
fn split_half(primitives: &[MeshPrimitive], start: usize, end: usize) -> MeshSplit {
    let index = (end - start) / 2;
    let mut split = mesh_split_bounds(primitives, start, end, index);
    split.axis = major_axis(aabb_extents(aabb_union(split.left_bounds, split.right_bounds))) as i32;
    split
}

/// Swift's `splitMeshMedian(_:range:)`: partition around the centre on the major axis.
fn split_median(primitives: &mut [MeshPrimitive], start: usize, end: usize) -> MeshSplit {
    let count = end - start;
    let mut lower = primitives[start].center;
    let mut upper = lower;
    for p in &primitives[start + 1..end] {
        lower = min_vec3(lower, p.center);
        upper = max_vec3(upper, p.center);
    }
    let d = upper - lower;
    let c = 0.5 * (lower + upper);
    let axis = if d.x >= d.y && d.x >= d.z {
        0
    } else if d.y >= d.z {
        1
    } else {
        2
    };
    let pivot = get_by_index(c, axis);
    let mut i1 = start;
    let mut i2 = end;
    while i1 < i2 {
        while i1 < i2 && get_by_index(primitives[i1].center, axis) < pivot {
            i1 += 1;
        }
        while i1 < i2 && get_by_index(primitives[i2 - 1].center, axis) >= pivot {
            i2 -= 1;
        }
        if i1 < i2 {
            primitives.swap(i1, i2 - 1);
            i1 += 1;
            i2 -= 1;
        }
    }
    let mut index = i1 - start;
    if index == 0 || index == count - 1 {
        index = count / 2;
    }
    let mut split = mesh_split_bounds(primitives, start, end, index);
    split.axis = axis as i32;
    split
}

/// Swift's `splitMeshBinnedSAH(_:range:)`: 8 buckets per axis, lowest surface-area cost wins.
fn split_binned_sah(primitives: &mut [MeshPrimitive], start: usize, end: usize) -> MeshSplit {
    #[derive(Clone, Copy)]
    struct Bucket {
        count: usize,
        bounds: AABB,
    }
    let mut split = MeshSplit::empty();
    let first = primitives[start].center;
    let mut bounds = AABB::new(first, first);
    for p in &primitives[start + 1..end] {
        bounds = aabb_union(bounds, AABB::new(p.center, p.center));
    }
    let extents = aabb_extents(bounds);
    let mut best_bucket: i32 = -1;
    let mut best_cost = f32::MAX;

    for axis in 0..3usize {
        let axis_usize: usize = axis;
        if get_by_index(extents, axis_usize) < crate::base::constants::linear_slop() {
            continue;
        }
        let mut buckets = [Bucket {
            count: 0,
            bounds: bounds3_empty(),
        }; 8];
        let factor = 8.0 * (1.0 - f32::EPSILON)
            / (get_by_index(bounds.upper_bound, axis_usize)
                - get_by_index(bounds.lower_bound, axis_usize));
        for i in start..end {
            let index = (factor
                * (get_by_index(primitives[i].center, axis_usize)
                    - get_by_index(bounds.lower_bound, axis_usize))) as usize;
            let index = index.min(7);
            buckets[index].count += 1;
            buckets[index].bounds = aabb_union(buckets[index].bounds, primitives[i].bounds);
        }
        for i in 0..7usize {
            let mut left_count = 0usize;
            let mut left_bounds = bounds3_empty();
            for k in 0..=i {
                left_count += buckets[k].count;
                left_bounds = aabb_union(left_bounds, buckets[k].bounds);
            }
            let mut right_count = 0usize;
            let mut right_bounds = bounds3_empty();
            for k in (i + 1)..8 {
                right_count += buckets[k].count;
                right_bounds = aabb_union(right_bounds, buckets[k].bounds);
            }
            if left_count > 0 && right_count > 0 {
                let cost = left_count as f32 * aabb_area(left_bounds)
                    + right_count as f32 * aabb_area(right_bounds);
                if cost < best_cost {
                    best_bucket = i as i32;
                    best_cost = cost;
                    split.axis = axis_usize as i32;
                    split.index = left_count;
                    split.left_bounds = left_bounds;
                    split.right_bounds = right_bounds;
                }
            }
        }
    }

    if best_bucket >= 0 {
        let axis = split.axis as usize;
        let factor = 8.0 * (1.0 - f32::EPSILON)
            / (get_by_index(bounds.upper_bound, axis)
                - get_by_index(bounds.lower_bound, axis));
        let mut split_index = start;
        for i in start..end {
            let index = (factor
                * (get_by_index(primitives[i].center, axis)
                    - get_by_index(bounds.lower_bound, axis))) as usize;
            if index as i32 <= best_bucket {
                primitives.swap(i, split_index);
                split_index += 1;
            }
        }
    }
    split
}
