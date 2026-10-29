//! Dynamic AABB tree, inspired by Nathanael Presson's btDbvt. Nodes are pooled and relocatable,
//! so indices are used instead of pointers. Sibling nodes are paired at even indices.

use crate::base::constants as b3;
use crate::base::core::{b3_assert, b3_validate};
use crate::collision::internal_math::{enlarge_aabb, modified_cross, perimeter};
use crate::math::functions::*;
use crate::math::types::{AABB, Vec3};
use crate::types::queries::{BoxCastInput, RayCastInput};

pub const TREE_STACK_SIZE: usize = 512;

pub const MOVED_NODE_FLAG: u32 = 1 << 30;
pub const LEAF_NODE_FLAG: u32 = 1 << 31;
pub const NODE_INDEX_MASK: u32 = !(MOVED_NODE_FLAG | LEAF_NODE_FLAG);
pub const EMPTY_NODE_FLAG: u32 = NODE_INDEX_MASK | LEAF_NODE_FLAG;
pub const ROOT_NODE: i32 = 0;

/// A node in the dynamic tree.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct TreeNode {
    pub aabb: AABB,
    /// bit 31: 1 for leaf node, bit 30: 1 for moved flag, bits 0-29: sibling pair or proxy id.
    pub flag_index: u32,
    /// Height of an internal node or the shape index for a leaf.
    pub u: i32,
}

impl TreeNode {
    #[inline]
    pub fn height(&self) -> i32 {
        self.u
    }

    #[inline]
    pub fn shape_index(&self) -> i32 {
        self.u
    }
}

/// Separate storage for tree leaves.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct TreeProxy {
    pub user_data: u64,
    pub category_bits: u64,
    pub node: i32,
    pub next: i32,
}

/// Performance results returned by dynamic tree queries.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct TreeStats {
    pub node_visits: i32,
    pub leaf_visits: i32,
}

#[inline]
pub fn is_leaf(node: TreeNode) -> bool {
    (node.flag_index & LEAF_NODE_FLAG) == LEAF_NODE_FLAG
}

#[inline]
pub fn is_node_moved(node: TreeNode) -> bool {
    (node.flag_index & MOVED_NODE_FLAG) == MOVED_NODE_FLAG
}

#[inline]
pub fn is_empty_node(node: TreeNode) -> bool {
    node.flag_index == EMPTY_NODE_FLAG
}

#[inline]
pub fn left_child(node: TreeNode) -> i32 {
    (node.flag_index & NODE_INDEX_MASK) as i32
}

#[inline]
pub fn proxy_id_of(node: TreeNode) -> i32 {
    (node.flag_index & NODE_INDEX_MASK) as i32
}

/// Test a ray for edge separation with an AABB (Gino, p80).
#[inline]
pub fn test_bounds_ray_overlap(
    node_min: Vec3,
    node_max: Vec3,
    ray_start: Vec3,
    ray_delta: Vec3,
) -> bool {
    let node_center = 0.5 * (node_min + node_max);
    let node_extent = node_max - node_center;
    let start = ray_start - node_center;
    let edge_separation = abs_vec3(cross(ray_delta, start)) - modified_cross(abs_vec3(ray_delta), node_extent);
    edge_separation.x <= 0.0 && edge_separation.y <= 0.0 && edge_separation.z <= 0.0
}

#[inline]
fn make_empty_node() -> TreeNode {
    TreeNode {
        aabb: AABB {
            lower_bound: Vec3::new(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            upper_bound: Vec3::new(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
        },
        flag_index: EMPTY_NODE_FLAG,
        u: 0,
    }
}

/// On-disk header for [`DynamicTree::save`] and [`DynamicTree::load`].
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct TreeFileHeader {
    pub version: u64,
    pub node_end: i32,
    pub node_capacity: i32,
    pub proxy_count: i32,
    pub proxy_capacity: i32,
    pub pair_free_list: i32,
    pub proxy_free_list: i32,
    pub dfs_ordered: i32,
}

/// A dynamic AABB tree.
#[derive(Clone, Debug)]
pub struct DynamicTree {
    pub version: u64,
    pub nodes: Vec<TreeNode>,
    pub parents: Vec<i32>,
    pub proxies: Vec<TreeProxy>,
    pub node_end: i32,
    pub node_capacity: i32,
    pub pair_free_list: i32,
    pub proxy_count: i32,
    pub proxy_capacity: i32,
    pub proxy_free_list: i32,
    pub swap_nodes: Vec<TreeNode>,
    pub leaf_indices: Vec<i32>,
    pub leaf_nodes: Vec<TreeNode>,
    pub leaf_centers: Vec<Vec3>,
    pub bin_indices: Vec<i32>,
    pub rebuild_capacity: i32,
    pub rebuild_stack: Vec<i32>,
    pub copy_stack: Vec<(i32, i32)>,
    pub dfs_ordered: bool,
}

impl DynamicTree {
    /// Constructing the tree initializes the node pool.
    pub fn new(proxy_capacity: i32) -> Self {
        let capacity = proxy_capacity.max(16);
        let node_capacity = 2 * capacity;
        let mut nodes = vec![TreeNode::default(); node_capacity as usize];
        let mut parents = vec![0i32; node_capacity as usize];

        nodes[ROOT_NODE as usize] = make_empty_node();
        nodes[ROOT_NODE as usize + 1] = make_empty_node();
        parents[ROOT_NODE as usize] = b3::NULL_INDEX;
        parents[ROOT_NODE as usize + 1] = b3::NULL_INDEX;

        let mut proxies = vec![TreeProxy::default(); capacity as usize];
        for i in 0..capacity as usize - 1 {
            proxies[i].node = b3::NULL_INDEX;
            proxies[i].next = (i + 1) as i32;
        }
        proxies[capacity as usize - 1].node = b3::NULL_INDEX;
        proxies[capacity as usize - 1].next = b3::NULL_INDEX;

        Self {
            version: b3::DYNAMIC_TREE_VERSION,
            nodes,
            parents,
            proxies,
            node_end: 2,
            node_capacity,
            pair_free_list: b3::NULL_INDEX,
            proxy_count: 0,
            proxy_capacity: capacity,
            proxy_free_list: 0,
            swap_nodes: Vec::new(),
            leaf_indices: Vec::new(),
            leaf_nodes: Vec::new(),
            leaf_centers: Vec::new(),
            bin_indices: Vec::new(),
            rebuild_capacity: 0,
            rebuild_stack: Vec::new(),
            copy_stack: Vec::new(),
            dfs_ordered: true,
        }
    }

    // MARK: Pool management

    fn allocate_sibling_pair(&mut self) -> i32 {
        if self.pair_free_list != b3::NULL_INDEX {
            let pair = self.pair_free_list;
            self.pair_free_list = self.parents[pair as usize];
            return pair;
        }

        if self.node_end + 2 > self.node_capacity {
            let old_capacity = self.node_capacity;
            let mut new_capacity = old_capacity + (old_capacity >> 1);
            new_capacity += new_capacity & 1;
            self.grow_node_capacity(new_capacity);
            self.node_capacity = new_capacity;
            self.swap_nodes.clear();
        }

        let pair = self.node_end;
        self.node_end += 2;
        pair
    }

    fn grow_node_capacity(&mut self, new_capacity: i32) {
        let extra = (new_capacity - self.nodes.len() as i32).max(0) as usize;
        self.nodes.resize(self.nodes.len() + extra, TreeNode::default());
        self.parents.resize(self.parents.len() + extra, 0);
    }

    fn free_pair(&mut self, pair: i32) {
        b3_assert((pair & 1) == 0 && 2 <= pair && pair < self.node_end);
        self.nodes[pair as usize] = make_empty_node();
        self.nodes[pair as usize + 1] = make_empty_node();
        self.parents[pair as usize] = self.pair_free_list;
        self.parents[pair as usize + 1] = b3::NULL_INDEX;
        self.pair_free_list = pair;
    }

    fn allocate_proxy(&mut self) -> i32 {
        if self.proxy_free_list == b3::NULL_INDEX {
            b3_assert(self.proxy_count == self.proxy_capacity);
            let old_capacity = self.proxy_capacity;
            self.proxy_capacity += (old_capacity >> 1).max(1);
            let extra = (self.proxy_capacity - old_capacity) as usize;
            self.proxies
                .resize(self.proxies.len() + extra, TreeProxy::default());

            for i in old_capacity as usize..self.proxy_capacity as usize - 1 {
                self.proxies[i].node = b3::NULL_INDEX;
                self.proxies[i].next = (i + 1) as i32;
            }
            let last = self.proxy_capacity as usize - 1;
            self.proxies[last].node = b3::NULL_INDEX;
            self.proxies[last].next = b3::NULL_INDEX;
            self.proxy_free_list = old_capacity;
        }

        let proxy_index = self.proxy_free_list;
        self.proxy_free_list = self.proxies[proxy_index as usize].next;
        self.proxies[proxy_index as usize] = TreeProxy::default();
        self.proxies[proxy_index as usize].node = b3::NULL_INDEX;
        self.proxies[proxy_index as usize].next = b3::NULL_INDEX;
        self.proxy_count += 1;
        proxy_index
    }

    fn free_proxy(&mut self, proxy_id: i32) {
        b3_assert(0 <= proxy_id && proxy_id < self.proxy_capacity);
        b3_assert(0 < self.proxy_count);
        self.proxies[proxy_id as usize].node = b3::NULL_INDEX;
        self.proxies[proxy_id as usize].next = self.proxy_free_list;
        self.proxy_free_list = proxy_id;
        self.proxy_count -= 1;
    }

    #[inline]
    fn get_node_height(node: TreeNode) -> i32 {
        if is_leaf(node) {
            0
        } else {
            node.height()
        }
    }

    #[inline]
    fn make_internal_from(&self, nodes: &[TreeNode], pair: i32) -> TreeNode {
        let c1 = nodes[pair as usize];
        let c2 = nodes[pair as usize + 1];
        TreeNode {
            aabb: aabb_union(c1.aabb, c2.aabb),
            flag_index: pair as u32 | ((c1.flag_index | c2.flag_index) & MOVED_NODE_FLAG),
            u: 1 + Self::get_node_height(c1).max(Self::get_node_height(c2)),
        }
    }

    #[inline]
    fn make_internal_node(&self, pair: i32) -> TreeNode {
        self.make_internal_from(&self.nodes, pair)
    }

    fn make_leaf_node(aabb: AABB, proxy_id: i32, user_data: u64, moved: bool) -> TreeNode {
        TreeNode {
            aabb,
            flag_index: proxy_id as u32
                | LEAF_NODE_FLAG
                | if moved { MOVED_NODE_FLAG } else { 0 },
            u: user_data as i32,
        }
    }

    #[inline]
    fn link_children(&mut self, node_index: i32) {
        let node = self.nodes[node_index as usize];
        if is_leaf(node) {
            let proxy = proxy_id_of(node);
            self.proxies[proxy as usize].node = node_index;
        } else {
            let pair = left_child(node);
            self.parents[pair as usize] = node_index;
            self.parents[pair as usize + 1] = node_index;
        }
    }

    #[inline]
    fn is_node_ordered(&self, node_index: i32) -> bool {
        let node = self.nodes[node_index as usize];
        is_leaf(node) || node_index < left_child(node)
    }

    // MARK: Sibling selection

    fn find_best_sibling(&self, box_d: AABB) -> i32 {
        let center_d = aabb_center(box_d);
        let area_d = perimeter(box_d);

        let mut node_index = ROOT_NODE;

        if is_leaf(self.nodes[node_index as usize]) {
            return node_index;
        }

        let root_box = self.nodes[node_index as usize].aabb;
        let mut area_base = perimeter(root_box);
        let mut direct_cost = perimeter(aabb_union(root_box, box_d));
        let mut inherited_cost: f32 = 0.0;

        let mut best_sibling = node_index;
        let mut best_cost = direct_cost;

        loop {
            let child1 = left_child(self.nodes[node_index as usize]);
            let child2 = child1 + 1;

            let cost = direct_cost + inherited_cost;
            if cost < best_cost {
                best_sibling = node_index;
                best_cost = cost;
            }

            inherited_cost += direct_cost - area_base;

            let leaf1 = is_leaf(self.nodes[child1 as usize]);
            let leaf2 = is_leaf(self.nodes[child2 as usize]);

            let mut lower_cost1 = f32::MAX;
            let box1 = self.nodes[child1 as usize].aabb;
            let direct_cost1 = perimeter(aabb_union(box1, box_d));
            let mut area1: f32 = 0.0;
            if leaf1 {
                let cost1 = direct_cost1 + inherited_cost;
                if cost1 < best_cost {
                    best_sibling = child1;
                    best_cost = cost1;
                }
            } else {
                area1 = perimeter(box1);
                lower_cost1 = inherited_cost + direct_cost1 + (area_d - area1).min(0.0);
            }

            let mut lower_cost2 = f32::MAX;
            let box2 = self.nodes[child2 as usize].aabb;
            let direct_cost2 = perimeter(aabb_union(box2, box_d));
            let mut area2: f32 = 0.0;
            if leaf2 {
                let cost2 = direct_cost2 + inherited_cost;
                if cost2 < best_cost {
                    best_sibling = child2;
                    best_cost = cost2;
                }
            } else {
                area2 = perimeter(box2);
                lower_cost2 = inherited_cost + direct_cost2 + (area_d - area2).min(0.0);
            }

            if leaf1 && leaf2 {
                break;
            }

            if best_cost <= lower_cost1 && best_cost <= lower_cost2 {
                break;
            }

            if lower_cost1 == lower_cost2 && !leaf1 {
                let d1 = aabb_center(box1) - center_d;
                let d2 = aabb_center(box2) - center_d;
                lower_cost1 = length_squared(d1);
                lower_cost2 = length_squared(d2);
            }

            if lower_cost1 < lower_cost2 && !leaf1 {
                node_index = child1;
                area_base = area1;
                direct_cost = direct_cost1;
            } else {
                node_index = child2;
                area_base = area2;
                direct_cost = direct_cost2;
            }
        }

        best_sibling
    }

    fn swap_tree_nodes(&mut self, i_down: i32, i_up: i32) {
        self.nodes.swap(i_down as usize, i_up as usize);
        self.link_children(i_down);
        self.link_children(i_up);

        if !self.is_node_ordered(i_down) || !self.is_node_ordered(i_up) {
            self.dfs_ordered = false;
        }

        let i_c = i_down ^ 1;
        let pair = left_child(self.nodes[i_c as usize]);
        self.nodes[i_c as usize] = self.make_internal_node(pair);
    }

    fn rotate_nodes(&mut self, i_a: i32) {
        let node_a = self.nodes[i_a as usize];
        b3_assert(!is_leaf(node_a));

        let i_b = left_child(node_a);
        let i_c = i_b + 1;
        let node_b = self.nodes[i_b as usize];
        let node_c = self.nodes[i_c as usize];

        let leaf_b = is_leaf(node_b);
        let leaf_c = is_leaf(node_c);
        if leaf_b && leaf_c {
            return;
        }

        let mut best_down = b3::NULL_INDEX;
        let mut best_up = b3::NULL_INDEX;
        let mut best_delta: f32 = 0.0;

        if !leaf_c {
            let i_f = left_child(node_c);
            let i_g = i_f + 1;
            let area_c = perimeter(node_c.aabb);

            let delta_bf = perimeter(aabb_union(node_b.aabb, self.nodes[i_g as usize].aabb)) - area_c;
            if delta_bf < best_delta {
                best_down = i_b;
                best_up = i_f;
                best_delta = delta_bf;
            }

            let delta_bg = perimeter(aabb_union(node_b.aabb, self.nodes[i_f as usize].aabb)) - area_c;
            if delta_bg < best_delta {
                best_down = i_b;
                best_up = i_g;
                best_delta = delta_bg;
            }
        }

        if !leaf_b {
            let i_d = left_child(node_b);
            let i_e = i_d + 1;
            let area_b = perimeter(node_b.aabb);

            let delta_cd = perimeter(aabb_union(node_c.aabb, self.nodes[i_e as usize].aabb)) - area_b;
            if delta_cd < best_delta {
                best_down = i_c;
                best_up = i_d;
                best_delta = delta_cd;
            }

            let delta_ce = perimeter(aabb_union(node_c.aabb, self.nodes[i_d as usize].aabb)) - area_b;
            if delta_ce < best_delta {
                best_down = i_c;
                best_up = i_e;
                best_delta = delta_ce;
            }
        }

        if best_down != b3::NULL_INDEX {
            self.swap_tree_nodes(best_down, best_up);
        }
    }

    // MARK: Insert and remove

    fn insert_leaf(&mut self, aabb: AABB, proxy_id: i32, moved: bool, should_rotate: bool) {
        let user_data = self.proxies[proxy_id as usize].user_data;
        let leaf = Self::make_leaf_node(aabb, proxy_id, user_data, moved);

        if is_empty_node(self.nodes[ROOT_NODE as usize]) {
            self.nodes[ROOT_NODE as usize] = leaf;
            self.proxies[proxy_id as usize].node = ROOT_NODE;
            return;
        }

        let sibling = self.find_best_sibling(aabb);

        let pair = self.allocate_sibling_pair();
        self.nodes[pair as usize] = self.nodes[sibling as usize];
        self.nodes[pair as usize + 1] = leaf;
        self.parents[pair as usize] = sibling;
        self.parents[pair as usize + 1] = sibling;
        self.link_children(pair);
        self.proxies[proxy_id as usize].node = pair + 1;

        let node = self.make_internal_node(pair);
        self.nodes[sibling as usize] = node;

        if !self.is_node_ordered(sibling) || !self.is_node_ordered(pair) {
            self.dfs_ordered = false;
        }

        let mut index = sibling;
        while index != b3::NULL_INDEX {
            if should_rotate {
                self.rotate_nodes(index);
            }
            let pair = left_child(self.nodes[index as usize]);
            self.nodes[index as usize] = self.make_internal_node(pair);
            index = self.parents[index as usize];
        }
    }

    fn remove_leaf(&mut self, proxy_id: i32) {
        let leaf = self.proxies[proxy_id as usize].node;
        b3_assert(0 <= leaf && leaf < self.node_end);
        b3_assert(is_leaf(self.nodes[leaf as usize])
            && proxy_id_of(self.nodes[leaf as usize]) == proxy_id);

        if leaf == ROOT_NODE {
            self.nodes[ROOT_NODE as usize] = make_empty_node();
            return;
        }

        let parent = self.parents[leaf as usize];
        self.nodes[parent as usize] = self.nodes[(leaf ^ 1) as usize];
        self.link_children(parent);
        self.free_pair(leaf & !1);

        let mut index = self.parents[parent as usize];
        while index != b3::NULL_INDEX {
            let pair = left_child(self.nodes[index as usize]);
            self.nodes[index as usize] = self.make_internal_node(pair);
            index = self.parents[index as usize];
        }
    }

    /// Create a proxy in the tree as a leaf node.
    pub fn create_proxy(&mut self, aabb: AABB, category_bits: u64, user_data: u64) -> i32 {
        b3_validate(is_valid_aabb(aabb));
        let proxy_id = self.allocate_proxy();
        self.proxies[proxy_id as usize].category_bits = category_bits;
        self.proxies[proxy_id as usize].user_data = user_data;
        self.insert_leaf(aabb, proxy_id, false, true);
        proxy_id
    }

    /// Internal proxy creation with the moved flag control.
    pub fn create_proxy_internal(
        &mut self,
        aabb: AABB,
        category_bits: u64,
        user_data: u64,
        mark_moved: bool,
    ) -> i32 {
        b3_validate(is_valid_aabb(aabb));
        let proxy_id = self.allocate_proxy();
        self.proxies[proxy_id as usize].category_bits = category_bits;
        self.proxies[proxy_id as usize].user_data = user_data;
        self.insert_leaf(aabb, proxy_id, mark_moved, true);
        proxy_id
    }

    /// Destroy a proxy. The proxy id must be valid.
    pub fn destroy_proxy(&mut self, proxy_id: i32) {
        b3_assert(0 <= proxy_id && proxy_id < self.proxy_capacity);
        self.remove_leaf(proxy_id);
        self.free_proxy(proxy_id);
    }

    pub fn move_proxy_internal(&mut self, proxy_id: i32, aabb: AABB, mark_moved: bool) {
        b3_validate(is_valid_aabb(aabb));
        b3_validate(aabb.upper_bound.x - aabb.lower_bound.x < b3::huge());
        b3_validate(aabb.upper_bound.y - aabb.lower_bound.y < b3::huge());
        b3_validate(aabb.upper_bound.z - aabb.lower_bound.z < b3::huge());
        b3_assert(0 <= proxy_id && proxy_id < self.proxy_capacity);
        self.remove_leaf(proxy_id);
        self.insert_leaf(aabb, proxy_id, mark_moved, false);
    }

    /// Move a proxy to a new AABB.
    pub fn move_proxy(&mut self, proxy_id: i32, aabb: AABB) {
        self.move_proxy_internal(proxy_id, aabb, false);
    }

    /// Enlarge a proxy and enlarge ancestors as necessary.
    pub fn enlarge_proxy(&mut self, proxy_id: i32, aabb: AABB) {
        b3_validate(is_valid_aabb(aabb));
        b3_validate(aabb.upper_bound.x - aabb.lower_bound.x < b3::huge());
        b3_validate(aabb.upper_bound.y - aabb.lower_bound.y < b3::huge());
        b3_validate(aabb.upper_bound.z - aabb.lower_bound.z < b3::huge());
        b3_assert(0 <= proxy_id && proxy_id < self.proxy_capacity);

        let mut index = self.proxies[proxy_id as usize].node;
        b3_assert(is_leaf(self.nodes[index as usize]));
        b3_validate(!aabb_contains(self.nodes[index as usize].aabb, aabb));

        self.nodes[index as usize].aabb = aabb;
        self.nodes[index as usize].flag_index |= MOVED_NODE_FLAG;

        index = self.parents[index as usize];
        while index != b3::NULL_INDEX {
            let changed = enlarge_aabb(&mut self.nodes[index as usize].aabb, aabb);
            self.nodes[index as usize].flag_index |= MOVED_NODE_FLAG;
            index = self.parents[index as usize];
            if !changed {
                break;
            }
        }

        while index != b3::NULL_INDEX {
            if self.nodes[index as usize].flag_index & MOVED_NODE_FLAG != 0 {
                break;
            }
            self.nodes[index as usize].flag_index |= MOVED_NODE_FLAG;
            index = self.parents[index as usize];
        }
    }

    pub fn set_category_bits(&mut self, proxy_id: i32, category_bits: u64) {
        b3_assert(0 <= proxy_id && proxy_id < self.proxy_capacity);
        self.proxies[proxy_id as usize].category_bits = category_bits;
    }

    pub fn get_category_bits(&self, proxy_id: i32) -> u64 {
        b3_assert(0 <= proxy_id && proxy_id < self.proxy_capacity);
        self.proxies[proxy_id as usize].category_bits
    }

    pub fn get_user_data(&self, proxy_id: i32) -> u64 {
        self.proxies[proxy_id as usize].user_data
    }

    pub fn get_aabb(&self, proxy_id: i32) -> AABB {
        self.nodes[self.proxies[proxy_id as usize].node as usize].aabb
    }

    pub fn height(&self) -> i32 {
        Self::get_node_height(self.nodes[ROOT_NODE as usize])
    }

    pub fn area_ratio(&self) -> f32 {
        if self.proxy_count == 0 {
            return 0.0;
        }
        let root_area = perimeter(self.nodes[ROOT_NODE as usize].aabb);
        if root_area <= 0.0 {
            return 0.0;
        }
        let mut internal_area: f32 = 0.0;
        let mut i: i32 = 2;
        while i < self.node_end {
            if !is_leaf(self.nodes[i as usize]) {
                internal_area += perimeter(self.nodes[i as usize].aabb);
            }
            i += 1;
        }
        internal_area / root_area
    }

    pub fn root_bounds(&self) -> AABB {
        if self.proxy_count == 0 {
            return AABB::default();
        }
        self.nodes[ROOT_NODE as usize].aabb
    }

    pub fn byte_count(&self) -> i32 {
        let mut size: i32 = 256;
        size += self.node_capacity * std::mem::size_of::<TreeNode>() as i32;
        size += self.node_capacity * 4;
        size += self.proxy_capacity * std::mem::size_of::<TreeProxy>() as i32;
        if !self.swap_nodes.is_empty() {
            size += self.node_capacity * std::mem::size_of::<TreeNode>() as i32;
        }
        size += self.rebuild_capacity * 4;
        size += self.rebuild_capacity * std::mem::size_of::<TreeNode>() as i32;
        size += self.rebuild_capacity * 12;
        size += self.rebuild_capacity * 4;
        size
    }

    // MARK: Queries

    #[inline]
    fn root_pair(&self) -> i32 {
        let root = self.nodes[ROOT_NODE as usize];
        if is_leaf(root) {
            ROOT_NODE
        } else {
            left_child(root)
        }
    }

    #[inline]
    fn user_data_for(&self, node: TreeNode, mask: u64, require_all: bool) -> Option<u64> {
        let proxy = self.proxies[proxy_id_of(node) as usize];
        let passes = if require_all {
            (proxy.category_bits & mask) == mask
        } else {
            (proxy.category_bits & mask) != 0
        };
        if passes {
            Some(proxy.user_data)
        } else {
            None
        }
    }

    /// Query an AABB for overlapping proxies. Return `false` from the callback to stop.
    pub fn query<F>(&self, aabb: AABB, mask_bits: u64, require_all_bits: bool, mut callback: F) -> TreeStats
    where
        F: FnMut(i32, u64) -> bool,
    {
        let mut result = TreeStats::default();
        if self.nodes.is_empty() || is_empty_node(self.nodes[ROOT_NODE as usize]) {
            return result;
        }

        let mut stack = [0i32; 256];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root_pair();
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            result.node_visits += 1;

            for i in 0..2 {
                let node_index = pair + i;
                let node = self.nodes[node_index as usize];
                if !aabb_overlaps(aabb, node.aabb) {
                    continue;
                }

                if is_leaf(node) {
                    let proxy_id = proxy_id_of(node);
                    if let Some(user_data) = self.user_data_for(node, mask_bits, require_all_bits) {
                        result.leaf_visits += 1;
                        if !callback(proxy_id, user_data) {
                            return result;
                        }
                    }
                } else {
                    b3_assert(stack_count < TREE_STACK_SIZE - 1);
                    stack[stack_count] = left_child(node);
                    stack_count += 1;
                }
            }
        }

        result
    }

    /// Query an AABB for the closest object.
    pub fn query_closest<F>(
        &self,
        point: Vec3,
        mask_bits: u64,
        require_all_bits: bool,
        min_distance_sqr: f32,
        mut callback: F,
    ) -> (TreeStats, f32)
    where
        F: FnMut(f32, i32, u64) -> f32,
    {
        let mut result = TreeStats::default();
        let mut min_sqr = min_distance_sqr;

        if self.nodes.is_empty() || is_empty_node(self.nodes[ROOT_NODE as usize]) {
            return (result, min_sqr);
        }

        let distance_to_node_sqr = |node: TreeNode| -> f32 {
            let r = point - clamp_vec3(point, node.aabb.lower_bound, node.aabb.upper_bound);
            dot(r, r)
        };

        let mut stack = [(0i32, 0.0f32); 128];
        let mut stack_count = 0usize;

        let root = self.root_pair();
        let seed1 = (root, distance_to_node_sqr(self.nodes[root as usize]));

        if root == ROOT_NODE {
            stack[stack_count] = seed1;
            stack_count += 1;
        } else {
            let seed2 = (root + 1, distance_to_node_sqr(self.nodes[root as usize + 1]));
            if seed2.1 < seed1.1 {
                stack[stack_count] = seed1;
                stack[stack_count + 1] = seed2;
            } else {
                stack[stack_count] = seed2;
                stack[stack_count + 1] = seed1;
            }
            stack_count += 2;
        }

        while stack_count > 0 {
            stack_count -= 1;
            let (node_index, distance_to_node) = stack[stack_count];
            let node = self.nodes[node_index as usize];
            result.node_visits += 1;

            if distance_to_node >= min_sqr {
                continue;
            }

            if is_leaf(node) {
                let proxy_id = proxy_id_of(node);
                if let Some(user_data) = self.user_data_for(node, mask_bits, require_all_bits) {
                    let dd = callback(min_sqr, proxy_id, user_data);
                    if dd < min_sqr {
                        min_sqr = dd;
                    }
                    result.leaf_visits += 1;
                }
                continue;
            }

            b3_assert(stack_count + 2 <= TREE_STACK_SIZE);
            let pair = left_child(node);
            let item1 = (pair, distance_to_node_sqr(self.nodes[pair as usize]));
            let item2 = (pair + 1, distance_to_node_sqr(self.nodes[pair as usize + 1]));

            if item2.1 < item1.1 {
                stack[stack_count] = item1;
                stack[stack_count + 1] = item2;
            } else {
                stack[stack_count] = item2;
                stack[stack_count + 1] = item1;
            }
            stack_count += 2;
        }

        (result, min_sqr)
    }

    /// Ray cast against the proxies in the tree.
    pub fn ray_cast<F>(
        &self,
        input: RayCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        mut callback: F,
    ) -> TreeStats
    where
        F: FnMut(&mut RayCastInput, i32, u64) -> f32,
    {
        let mut result = TreeStats::default();
        if self.nodes.is_empty() || is_empty_node(self.nodes[ROOT_NODE as usize]) {
            return result;
        }

        let p1 = input.origin;
        let d = input.translation;
        let mut max_fraction = input.max_fraction;
        let mut p2 = mul_add(p1, max_fraction, d);
        let mut segment_aabb = AABB {
            lower_bound: min_vec3(p1, p2),
            upper_bound: max_vec3(p1, p2),
        };

        let mut stack = [0i32; 256];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root_pair();
        stack_count += 1;

        let mut sub_input = input;

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            result.node_visits += 1;

            let mut hits = [TreeNode::default(); 2];
            let mut is_leaf_hit = [false; 2];
            let mut hit_count = 0usize;
            for i in 0..2 {
                let node = self.nodes[(pair + i) as usize];
                if !aabb_overlaps(segment_aabb, node.aabb) {
                    continue;
                }
                if !test_bounds_ray_overlap(node.aabb.lower_bound, node.aabb.upper_bound, p1, d) {
                    continue;
                }
                is_leaf_hit[hit_count] = is_leaf(node);
                hits[hit_count] = node;
                hit_count += 1;
            }

            if hit_count == 2 && !is_leaf_hit[0] && !is_leaf_hit[1] {
                let d1 = distance_squared(aabb_center(hits[0].aabb), p1);
                let d2 = distance_squared(aabb_center(hits[1].aabb), p1);
                if d1 < d2 {
                    hits.swap(0, 1);
                }
            }

            for i in 0..hit_count {
                if is_leaf_hit[i] {
                    let proxy_id = proxy_id_of(hits[i]);
                    if let Some(user_data) = self.user_data_for(hits[i], mask_bits, require_all_bits) {
                        sub_input.max_fraction = max_fraction;
                        let value = callback(&mut sub_input, proxy_id, user_data);
                        result.leaf_visits += 1;

                        if value == 0.0 {
                            return result;
                        }

                        if 0.0 < value && value <= max_fraction {
                            max_fraction = value;
                            p2 = mul_add(p1, max_fraction, d);
                            segment_aabb.lower_bound = min_vec3(p1, p2);
                            segment_aabb.upper_bound = max_vec3(p1, p2);
                        }
                    }
                } else {
                    b3_assert(stack_count < TREE_STACK_SIZE - 1);
                    stack[stack_count] = left_child(hits[i]);
                    stack_count += 1;
                }
            }
        }

        result
    }

    /// Sweep an AABB through the tree.
    pub fn box_cast<F>(
        &self,
        input: BoxCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        mut callback: F,
    ) -> TreeStats
    where
        F: FnMut(&mut BoxCastInput, i32, u64) -> f32,
    {
        let mut result = TreeStats::default();
        if self.nodes.is_empty() || is_empty_node(self.nodes[ROOT_NODE as usize]) {
            return result;
        }

        let origin_aabb = input.box_;
        let p1 = aabb_center(origin_aabb);
        let extension = max_vec3(p1 - origin_aabb.lower_bound, origin_aabb.upper_bound - p1);
        let d = input.translation;
        let mut max_fraction = input.max_fraction;
        let mut t = max_fraction * input.translation;
        let mut total_aabb = AABB {
            lower_bound: min_vec3(origin_aabb.lower_bound, origin_aabb.lower_bound + t),
            upper_bound: max_vec3(origin_aabb.upper_bound, origin_aabb.upper_bound + t),
        };

        let mut stack = [0i32; 256];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root_pair();
        stack_count += 1;

        let mut sub_input = input;

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            result.node_visits += 1;

            let mut hits = [TreeNode::default(); 2];
            let mut is_leaf_hit = [false; 2];
            let mut hit_count = 0usize;
            for i in 0..2 {
                let node = self.nodes[(pair + i) as usize];
                if !aabb_overlaps(total_aabb, node.aabb) {
                    continue;
                }
                let lower = node.aabb.lower_bound - extension;
                let upper = node.aabb.upper_bound + extension;
                if !test_bounds_ray_overlap(lower, upper, p1, d) {
                    continue;
                }
                is_leaf_hit[hit_count] = is_leaf(node);
                hits[hit_count] = node;
                hit_count += 1;
            }

            if hit_count == 2 && !is_leaf_hit[0] && !is_leaf_hit[1] {
                let d1 = distance_squared(aabb_center(hits[0].aabb), p1);
                let d2 = distance_squared(aabb_center(hits[1].aabb), p1);
                if d1 < d2 {
                    hits.swap(0, 1);
                }
            }

            for i in 0..hit_count {
                if is_leaf_hit[i] {
                    let proxy_id = proxy_id_of(hits[i]);
                    if let Some(user_data) = self.user_data_for(hits[i], mask_bits, require_all_bits) {
                        sub_input.max_fraction = max_fraction;
                        let value = callback(&mut sub_input, proxy_id, user_data);
                        result.leaf_visits += 1;

                        if value == 0.0 {
                            return result;
                        }

                        if 0.0 < value && value < max_fraction {
                            max_fraction = value;
                            t = max_fraction * input.translation;
                            total_aabb.lower_bound =
                                min_vec3(origin_aabb.lower_bound, origin_aabb.lower_bound + t);
                            total_aabb.upper_bound =
                                max_vec3(origin_aabb.upper_bound, origin_aabb.upper_bound + t);
                        }
                    }
                } else {
                    b3_assert(stack_count < TREE_STACK_SIZE - 1);
                    stack[stack_count] = left_child(hits[i]);
                    stack_count += 1;
                }
            }
        }

        result
    }

    // MARK: Validation

    pub fn validate(&self) {
        b3_assert(2 <= self.node_end && self.node_end <= self.node_capacity);
        b3_assert((self.node_end & 1) == 0);
        b3_assert(self.parents[ROOT_NODE as usize] == b3::NULL_INDEX);
        b3_assert(is_empty_node(self.nodes[ROOT_NODE as usize + 1]));

        let mut free_pair_count = 0;
        let mut pair = self.pair_free_list;
        while pair != b3::NULL_INDEX {
            b3_assert((pair & 1) == 0 && 2 <= pair && pair < self.node_end);
            b3_assert(is_empty_node(self.nodes[pair as usize]));
            b3_assert(is_empty_node(self.nodes[pair as usize + 1]));
            pair = self.parents[pair as usize];
            free_pair_count += 1;
            b3_assert(2 * free_pair_count < self.node_end);
        }

        let mut free_proxy_count = 0;
        let mut free_index = self.proxy_free_list;
        while free_index != b3::NULL_INDEX {
            b3_assert(0 <= free_index && free_index < self.proxy_capacity);
            b3_assert(self.proxies[free_index as usize].node == b3::NULL_INDEX);
            free_index = self.proxies[free_index as usize].next;
            free_proxy_count += 1;
        }
        b3_assert(self.proxy_count + free_proxy_count == self.proxy_capacity);
        b3_assert(self.node_end == 2 * self.proxy_count.max(1) + 2 * free_pair_count);

        if self.proxy_count == 0 {
            b3_assert(is_empty_node(self.nodes[ROOT_NODE as usize]));
            return;
        }

        let mut leaf_count = 0;
        self.validate_subtree(ROOT_NODE, &mut leaf_count);
        b3_assert(leaf_count == self.proxy_count);
    }

    fn validate_subtree(&self, node_index: i32, leaf_count: &mut i32) -> i32 {
        b3_assert(0 <= node_index && node_index < self.node_end);
        let node = self.nodes[node_index as usize];
        b3_assert(!is_empty_node(node));
        b3_assert(is_valid_aabb(node.aabb));

        if is_leaf(node) {
            let proxy_id = proxy_id_of(node);
            b3_assert(0 <= proxy_id && proxy_id < self.proxy_capacity);
            b3_assert(self.proxies[proxy_id as usize].node == node_index);
            b3_assert(self.proxies[proxy_id as usize].user_data as i32 == node.shape_index());
            *leaf_count += 1;
            return 0;
        }

        let pair = left_child(node);
        b3_assert((pair & 1) == 0 && 2 <= pair && pair < self.node_end);
        b3_assert(self.parents[pair as usize] == node_index);
        b3_assert(self.parents[pair as usize + 1] == node_index);
        b3_assert(!self.dfs_ordered || node_index < pair);

        let c1 = self.nodes[pair as usize];
        let c2 = self.nodes[pair as usize + 1];
        b3_assert(aabb_contains(node.aabb, c1.aabb));
        b3_assert(aabb_contains(node.aabb, c2.aabb));
        b3_assert(is_node_moved(node) == (is_node_moved(c1) || is_node_moved(c2)));

        let height1 = self.validate_subtree(pair, leaf_count);
        let height2 = self.validate_subtree(pair + 1, leaf_count);
        let height = 1 + height1.max(height2);
        b3_assert(node.height() == height);
        height
    }

    pub fn validate_no_moved(&self) {
        let mut i: i32 = 0;
        while i < self.node_end {
            b3_assert(!is_node_moved(self.nodes[i as usize]));
            i += 1;
        }
    }

    // MARK: Rebuild

    fn partition_mid(indices: &mut [i32], centers: &mut [Vec3], start: usize, count: usize) -> usize {
        if count <= 2 {
            return count / 2;
        }

        let mut lower_bound = centers[start];
        let mut upper_bound = centers[start];
        for i in 1..count {
            lower_bound = min_vec3(lower_bound, centers[start + i]);
            upper_bound = max_vec3(upper_bound, centers[start + i]);
        }

        let d = upper_bound - lower_bound;
        let c = 0.5 * (lower_bound + upper_bound);

        let mut i1 = 0usize;
        let mut i2 = count;

        macro_rules! partition_axis {
            ($component:ident) => {{
                let pivot = c.$component;
                while i1 < i2 {
                    while i1 < i2 && centers[start + i1].$component < pivot {
                        i1 += 1;
                    }
                    while i1 < i2 && centers[start + i2 - 1].$component >= pivot {
                        i2 -= 1;
                    }
                    if i1 < i2 {
                        indices.swap(start + i1, start + i2 - 1);
                        centers.swap(start + i1, start + i2 - 1);
                        i1 += 1;
                        i2 -= 1;
                    }
                }
            }};
        }

        if d.x >= d.y && d.x >= d.z {
            partition_axis!(x);
        } else if d.y >= d.z {
            partition_axis!(y);
        } else {
            partition_axis!(z);
        }
        b3_assert(i1 == i2);

        if i1 > 0 && i1 < count {
            return i1;
        }
        count / 2
    }

    #[inline]
    fn set_left_child(node: &mut TreeNode, pair: i32) {
        node.flag_index = (node.flag_index & !NODE_INDEX_MASK) | pair as u32;
    }

    #[inline]
    fn bump_pair(&mut self, parent: i32) -> i32 {
        let pair = self.node_end;
        b3_assert(pair + 2 <= self.node_capacity);
        self.node_end += 2;
        self.parents[pair as usize] = parent;
        self.parents[pair as usize + 1] = parent;
        pair
    }

    fn copy_subtree(&mut self, node: TreeNode, new_index: i32) {
        if is_leaf(node) {
            self.swap_nodes[new_index as usize] = node;
            let proxy_id = proxy_id_of(node);
            self.proxies[proxy_id as usize].node = new_index;
            return;
        }

        if self.copy_stack.len() < TREE_STACK_SIZE {
            self.copy_stack.resize(TREE_STACK_SIZE, (0, 0));
        }
        let mut stack_count = 0usize;

        let mut old_pair = left_child(node);
        let mut new_pair = self.bump_pair(new_index);
        let mut root_node_copy = node;
        Self::set_left_child(&mut root_node_copy, new_pair);
        self.swap_nodes[new_index as usize] = root_node_copy;

        loop {
            let pair_base = new_pair as usize;
            self.swap_nodes[pair_base] = self.nodes[old_pair as usize];
            self.swap_nodes[pair_base + 1] = self.nodes[old_pair as usize + 1];

            if is_leaf(self.swap_nodes[pair_base + 1]) {
                let proxy_id = proxy_id_of(self.swap_nodes[pair_base + 1]);
                self.proxies[proxy_id as usize].node = new_pair + 1;
            } else {
                b3_assert(stack_count < TREE_STACK_SIZE);
                let left_child_index = left_child(self.swap_nodes[pair_base + 1]);
                self.copy_stack[stack_count] = (left_child_index, new_pair + 1);
                stack_count += 1;
            }

            let mut left_node = self.swap_nodes[pair_base];
            if !is_leaf(left_node) {
                let left_index = new_pair;
                old_pair = left_child(left_node);
                new_pair = self.bump_pair(left_index);
                Self::set_left_child(&mut left_node, new_pair);
                self.swap_nodes[left_index as usize] = left_node;
                continue;
            }

            let proxy_id = proxy_id_of(left_node);
            self.proxies[proxy_id as usize].node = new_pair;

            if stack_count == 0 {
                break;
            }

            stack_count -= 1;
            let item = self.copy_stack[stack_count];
            old_pair = item.0;
            new_pair = self.bump_pair(item.1);
            let mut right_node = self.swap_nodes[item.1 as usize];
            Self::set_left_child(&mut right_node, new_pair);
            self.swap_nodes[item.1 as usize] = right_node;
        }
    }

    fn place_leaf(&mut self, leaf: TreeNode, new_index: i32) {
        if is_leaf(leaf) {
            self.swap_nodes[new_index as usize] = leaf;
            let proxy_id = proxy_id_of(leaf);
            self.proxies[proxy_id as usize].node = new_index;
        } else {
            self.copy_subtree(leaf, new_index);
        }
    }

    fn build_tree(&mut self, leaf_count: i32) {
        let leaves = self.leaf_nodes.clone();

        self.node_end = 2;
        let empty = make_empty_node();
        self.swap_nodes[ROOT_NODE as usize + 1] = empty;
        self.parents[ROOT_NODE as usize] = b3::NULL_INDEX;
        self.parents[ROOT_NODE as usize + 1] = b3::NULL_INDEX;

        if leaf_count == 1 {
            let leaf = leaves[self.leaf_indices[0] as usize];
            self.place_leaf(leaf, ROOT_NODE);
            return;
        }

        #[derive(Clone, Copy, Default)]
        struct BuildItem {
            node_index: i32,
            pair: i32,
            child_count: i32,
            start_index: usize,
            end_index: usize,
            split_index: usize,
        }

        let mut stack = [BuildItem::default(); TREE_STACK_SIZE];
        let mut top = 0usize;

        stack[0].node_index = ROOT_NODE;
        stack[0].pair = self.bump_pair(ROOT_NODE);
        stack[0].child_count = -1;
        stack[0].start_index = 0;
        stack[0].end_index = leaf_count as usize;
        stack[0].split_index = Self::partition_mid(
            &mut self.leaf_indices,
            &mut self.leaf_centers,
            0,
            leaf_count as usize,
        );

        loop {
            stack[top].child_count += 1;

            if stack[top].child_count == 2 {
                let item = stack[top];
                let node = self.make_internal_from(&self.swap_nodes, item.pair);
                self.swap_nodes[item.node_index as usize] = node;
                if top == 0 {
                    break;
                }
                top -= 1;
                continue;
            }

            let slot = stack[top].child_count as usize;
            let start_index = if slot == 0 {
                stack[top].start_index
            } else {
                stack[top].split_index
            };
            let end_index = if slot == 0 {
                stack[top].split_index
            } else {
                stack[top].end_index
            };
            let count = end_index - start_index;
            b3_assert(count > 0);

            let node_index = stack[top].pair + slot as i32;
            if count == 1 {
                let leaf = leaves[self.leaf_indices[start_index] as usize];
                self.place_leaf(leaf, node_index);
                continue;
            }

            b3_assert(top < TREE_STACK_SIZE - 1);
            top += 1;
            stack[top].node_index = node_index;
            stack[top].pair = self.bump_pair(node_index);
            stack[top].child_count = -1;
            stack[top].start_index = start_index;
            stack[top].end_index = end_index;
            stack[top].split_index = start_index
                + Self::partition_mid(
                    &mut self.leaf_indices,
                    &mut self.leaf_centers,
                    start_index,
                    count,
                );
        }
    }

    /// Rebuild the stale parts of the tree. The entire tree is put into DFS order.
    pub fn rebuild(&mut self, full_build: bool) -> i32 {
        let count = self.proxy_count;
        if count == 0 {
            return 0;
        }

        let root = self.nodes[ROOT_NODE as usize];
        if !full_build && !is_node_moved(root) && self.dfs_ordered {
            return 0;
        }

        if self.swap_nodes.is_empty() {
            self.swap_nodes = vec![TreeNode::default(); self.node_capacity as usize];
        }

        if count > self.rebuild_capacity {
            let new_capacity = count + count / 2;
            self.leaf_indices = vec![0; new_capacity as usize];
            self.leaf_nodes = vec![TreeNode::default(); new_capacity as usize];
            self.leaf_centers = vec![Vec3::default(); new_capacity as usize];
            self.rebuild_capacity = new_capacity;
        }

        let mut leaf_count: i32 = 0;
        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;

        if !is_leaf(root) && (full_build || is_node_moved(root)) {
            stack[stack_count] = left_child(root);
            stack_count += 1;
        } else {
            let mut node = root;
            if !full_build {
                node.flag_index &= !MOVED_NODE_FLAG;
            }
            self.leaf_indices[0] = 0;
            self.leaf_nodes[0] = node;
            self.leaf_centers[0] = aabb_center(node.aabb);
            leaf_count = 1;
        }

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            for i in 0..2 {
                let mut node = self.nodes[(pair + i) as usize];
                if !is_leaf(node) && (full_build || is_node_moved(node)) {
                    b3_assert(stack_count < TREE_STACK_SIZE);
                    stack[stack_count] = left_child(node);
                    stack_count += 1;
                    continue;
                }

                if !full_build {
                    node.flag_index &= !MOVED_NODE_FLAG;
                }
                let index = leaf_count as usize;
                self.leaf_indices[index] = leaf_count;
                self.leaf_nodes[index] = node;
                self.leaf_centers[index] = aabb_center(node.aabb);
                leaf_count += 1;
            }
        }

        b3_assert(0 < leaf_count && leaf_count <= count);
        self.build_tree(leaf_count);
        std::mem::swap(&mut self.nodes, &mut self.swap_nodes);
        self.pair_free_list = b3::NULL_INDEX;
        self.dfs_ordered = true;

        #[cfg(debug_assertions)]
        {
            self.validate();
            if !full_build {
                self.validate_no_moved();
            }
        }

        leaf_count
    }

    // MARK: Moved tracking

    pub fn mark_proxy_moved_serial(&mut self, proxy_id: i32) {
        b3_validate(0 <= proxy_id && proxy_id < self.proxy_capacity);
        let mut index = self.proxies[proxy_id as usize].node;
        b3_validate(0 <= index && index < self.node_end);
        b3_validate(is_leaf(self.nodes[index as usize]));

        while index != b3::NULL_INDEX {
            self.nodes[index as usize].flag_index |= MOVED_NODE_FLAG;
            index = self.parents[index as usize];
        }
    }

    pub fn mark_proxy_moved(&mut self, proxy_id: i32, aabb: AABB) {
        b3_validate(0 <= proxy_id && proxy_id < self.proxy_capacity);
        let mut index = self.proxies[proxy_id as usize].node;
        b3_validate(0 <= index && index < self.node_end);
        b3_validate(is_leaf(self.nodes[index as usize]));
        b3_validate(!aabb_contains(self.nodes[index as usize].aabb, aabb));

        self.nodes[index as usize].aabb = aabb;
        self.nodes[index as usize].flag_index |= MOVED_NODE_FLAG;

        index = self.parents[index as usize];
        while index != b3::NULL_INDEX {
            if self.nodes[index as usize].flag_index & MOVED_NODE_FLAG != 0 {
                break;
            }
            let previous_flags = self.nodes[index as usize].flag_index;
            self.nodes[index as usize].flag_index |= MOVED_NODE_FLAG;
            if previous_flags & MOVED_NODE_FLAG != 0 {
                break;
            }
            index = self.parents[index as usize];
        }
    }

    pub fn clear_moved(&mut self) {
        if !is_node_moved(self.nodes[ROOT_NODE as usize]) {
            return;
        }

        self.nodes[ROOT_NODE as usize].flag_index &= !MOVED_NODE_FLAG;
        if is_leaf(self.nodes[ROOT_NODE as usize]) {
            return;
        }

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = left_child(self.nodes[ROOT_NODE as usize]);
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            for i in 0..2 {
                let node_index = (pair + i) as usize;
                if self.nodes[node_index].flag_index & MOVED_NODE_FLAG != 0 {
                    self.nodes[node_index].flag_index &= !MOVED_NODE_FLAG;
                    if !is_leaf(self.nodes[node_index]) {
                        b3_assert(stack_count < TREE_STACK_SIZE);
                        stack[stack_count] = left_child(self.nodes[node_index]);
                        stack_count += 1;
                    }
                }
            }
        }
    }

    pub fn gather_moved_proxies(&self, proxy_ids: &mut [i32]) -> i32 {
        let root = self.nodes[ROOT_NODE as usize];
        if !is_node_moved(root) {
            return 0;
        }
        if is_leaf(root) {
            proxy_ids[0] = proxy_id_of(root);
            return 1;
        }

        let mut count: i32 = 0;
        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = left_child(root);
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            for i in 0..2 {
                let node = self.nodes[(pair + i) as usize];
                if !is_node_moved(node) {
                    continue;
                }
                if is_leaf(node) {
                    proxy_ids[count as usize] = proxy_id_of(node);
                    count += 1;
                } else {
                    b3_assert(stack_count < TREE_STACK_SIZE);
                    stack[stack_count] = left_child(node);
                    stack_count += 1;
                }
            }
        }
        count
    }

    fn refit_subtree(&mut self, root_index: i32) {
        let root = self.nodes[root_index as usize];
        if is_leaf(root) || !is_node_moved(root) {
            return;
        }

        let mut stack = [0i32; TREE_STACK_SIZE];
        let mut stack_count = 0usize;
        stack[stack_count] = !root_index;
        stack_count += 1;
        stack[stack_count] = left_child(root);
        stack_count += 1;
        stack[stack_count] = left_child(root) + 1;
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let item = stack[stack_count];
            if item < 0 {
                let node_index = !item;
                let pair = left_child(self.nodes[node_index as usize]);
                self.nodes[node_index as usize].aabb =
                    aabb_union(self.nodes[pair as usize].aabb, self.nodes[pair as usize + 1].aabb);
                continue;
            }

            if is_leaf(self.nodes[item as usize]) || !is_node_moved(self.nodes[item as usize]) {
                continue;
            }

            b3_assert(stack_count + 3 <= TREE_STACK_SIZE);
            let pair = left_child(self.nodes[item as usize]);
            stack[stack_count] = !item;
            stack_count += 1;
            stack[stack_count] = pair;
            stack_count += 1;
            stack[stack_count] = pair + 1;
            stack_count += 1;
        }
    }

    /// Refit the tree using the moved flags. This requires DFS order.
    pub fn refit(&mut self) {
        if !is_node_moved(self.nodes[ROOT_NODE as usize]) {
            return;
        }

        if !self.dfs_ordered {
            self.refit_subtree(ROOT_NODE);
            return;
        }

        let mut pair = self.node_end - 2;
        while pair >= 0 {
            let base = pair as usize;
            let flags1 = self.nodes[base].flag_index;
            let flags2 = self.nodes[base + 1].flag_index;

            if ((flags1 | flags2) & MOVED_NODE_FLAG) == 0 {
                pair -= 2;
                continue;
            }

            let refit1 = (flags1 & (LEAF_NODE_FLAG | MOVED_NODE_FLAG)) == MOVED_NODE_FLAG;
            let refit2 = (flags2 & (LEAF_NODE_FLAG | MOVED_NODE_FLAG)) == MOVED_NODE_FLAG;

            let children1 = if refit1 {
                (flags1 & NODE_INDEX_MASK) as i32
            } else {
                pair
            };
            let children2 = if refit2 {
                (flags2 & NODE_INDEX_MASK) as i32
            } else {
                pair
            };

            if refit1 {
                self.nodes[base].aabb = aabb_union(
                    self.nodes[children1 as usize].aabb,
                    self.nodes[children1 as usize + 1].aabb,
                );
            }
            if refit2 {
                self.nodes[base + 1].aabb = aabb_union(
                    self.nodes[children2 as usize].aabb,
                    self.nodes[children2 as usize + 1].aabb,
                );
            }

            pair -= 2;
        }
    }

    // MARK: State

    pub fn has_moved(&self) -> bool {
        is_node_moved(self.nodes[ROOT_NODE as usize])
    }

    pub fn needs_rebuild(&self) -> bool {
        is_node_moved(self.nodes[ROOT_NODE as usize]) || !self.dfs_ordered
    }

    // MARK: File IO

    /// Save this tree to a file for debugging.
    pub fn save(&self, file_name: &str) {
        let header = TreeFileHeader {
            version: self.version,
            node_end: self.node_end,
            node_capacity: self.node_end,
            proxy_count: self.proxy_count,
            proxy_capacity: self.proxy_capacity,
            pair_free_list: self.pair_free_list,
            proxy_free_list: self.proxy_free_list,
            dfs_ordered: if self.dfs_ordered { 1 } else { 0 },
        };

        let node_bytes = self.node_end as usize * std::mem::size_of::<TreeNode>();
        let parent_bytes = self.node_end as usize * std::mem::size_of::<i32>();
        let proxy_bytes = self.proxy_capacity as usize * std::mem::size_of::<TreeProxy>();

        let mut data = Vec::with_capacity(
            std::mem::size_of::<TreeFileHeader>() + node_bytes + parent_bytes + proxy_bytes,
        );
        unsafe {
            let header_ptr = &header as *const TreeFileHeader as *const u8;
            data.extend_from_slice(std::slice::from_raw_parts(
                header_ptr,
                std::mem::size_of::<TreeFileHeader>(),
            ));
            let node_ptr = self.nodes.as_ptr() as *const u8;
            data.extend_from_slice(std::slice::from_raw_parts(node_ptr, node_bytes));
            let parent_ptr = self.parents.as_ptr() as *const u8;
            data.extend_from_slice(std::slice::from_raw_parts(parent_ptr, parent_bytes));
            let proxy_ptr = self.proxies.as_ptr() as *const u8;
            data.extend_from_slice(std::slice::from_raw_parts(proxy_ptr, proxy_bytes));
        }
        let _ = std::fs::write(file_name, data);
    }

    /// Load a tree from a file for debugging.
    pub fn load(file_name: &str, scale: f32) -> Option<Self> {
        let data = std::fs::read(file_name).ok()?;
        let stride = std::mem::size_of::<TreeFileHeader>();
        if data.len() < stride {
            return None;
        }
        let header = unsafe { std::ptr::read_unaligned(data.as_ptr() as *const TreeFileHeader) };
        if header.version != b3::DYNAMIC_TREE_VERSION {
            return None;
        }

        let node_end = header.node_end;
        let node_capacity = header.node_capacity;
        let proxy_capacity = header.proxy_capacity;
        if !(node_capacity >= 2
            && proxy_capacity >= 1
            && (node_end & 1) == 0
            && node_end >= 2
            && node_end <= node_capacity)
        {
            return None;
        }

        let node_bytes = node_capacity as usize * std::mem::size_of::<TreeNode>();
        let parent_bytes = node_capacity as usize * std::mem::size_of::<i32>();
        let proxy_bytes = proxy_capacity as usize * std::mem::size_of::<TreeProxy>();
        if data.len() < stride + node_bytes + parent_bytes + proxy_bytes {
            return None;
        }

        let mut tree = Self::new(proxy_capacity);
        tree.nodes = vec![TreeNode::default(); node_capacity as usize];
        tree.parents = vec![0i32; node_capacity as usize];
        tree.proxies = vec![TreeProxy::default(); proxy_capacity as usize];

        let node_offset = stride;
        let parent_offset = node_offset + node_bytes;
        let proxy_offset = parent_offset + parent_bytes;

        unsafe {
            let src = data.as_ptr();
            std::ptr::copy_nonoverlapping(
                src.add(node_offset) as *const TreeNode,
                tree.nodes.as_mut_ptr(),
                node_capacity as usize,
            );
            std::ptr::copy_nonoverlapping(
                src.add(parent_offset) as *const i32,
                tree.parents.as_mut_ptr(),
                node_capacity as usize,
            );
            std::ptr::copy_nonoverlapping(
                src.add(proxy_offset) as *const TreeProxy,
                tree.proxies.as_mut_ptr(),
                proxy_capacity as usize,
            );
        }

        tree.node_end = node_end;
        tree.node_capacity = node_capacity;
        tree.proxy_capacity = proxy_capacity;
        tree.proxy_count = header.proxy_count;
        tree.pair_free_list = header.pair_free_list;
        tree.proxy_free_list = header.proxy_free_list;
        tree.dfs_ordered = header.dfs_ordered != 0;
        tree.rebuild_capacity = 0;

        for i in 0..node_end as usize {
            if is_empty_node(tree.nodes[i]) {
                continue;
            }
            tree.nodes[i].aabb.lower_bound = scale * tree.nodes[i].aabb.lower_bound;
            tree.nodes[i].aabb.upper_bound = scale * tree.nodes[i].aabb.upper_bound;
        }

        Some(tree)
    }

    /// Immutable view used by queries; compound trees supply nodes without proxies.
    pub fn view(&self) -> TreeView<'_> {
        TreeView::new(&self.nodes, Some(&self.proxies))
    }
}

/// Read-only traversal of dynamic trees or immutable compound tree nodes.
#[derive(Clone, Copy, Debug)]
pub struct TreeView<'a> {
    pub nodes: &'a [TreeNode],
    pub proxies: Option<&'a [TreeProxy]>,
}

impl<'a> TreeView<'a> {
    pub fn new(nodes: &'a [TreeNode], proxies: Option<&'a [TreeProxy]>) -> Self {
        Self { nodes, proxies }
    }

    #[inline]
    fn root_pair(&self) -> i32 {
        let root = self.nodes[ROOT_NODE as usize];
        if is_leaf(root) {
            ROOT_NODE
        } else {
            left_child(root)
        }
    }

    #[inline]
    fn user_data_for(&self, node: TreeNode, mask: u64, require_all: bool) -> Option<u64> {
        match self.proxies {
            Some(proxies) => {
                let proxy = proxies[proxy_id_of(node) as usize];
                let passes = if require_all {
                    (proxy.category_bits & mask) == mask
                } else {
                    (proxy.category_bits & mask) != 0
                };
                if passes {
                    Some(proxy.user_data)
                } else {
                    None
                }
            }
            None => Some(node.shape_index() as u32 as u64),
        }
    }

    /// Query an AABB for overlapping proxies. Return `false` from the callback to stop.
    pub fn query<F>(&self, aabb: AABB, mask_bits: u64, require_all_bits: bool, mut callback: F) -> TreeStats
    where
        F: FnMut(i32, u64) -> bool,
    {
        let mut result = TreeStats::default();
        if self.nodes.is_empty() || is_empty_node(self.nodes[ROOT_NODE as usize]) {
            return result;
        }

        let mut stack = [0i32; 256];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root_pair();
        stack_count += 1;

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            result.node_visits += 1;

            for i in 0..2 {
                let node_index = pair + i;
                let node = self.nodes[node_index as usize];
                if !aabb_overlaps(aabb, node.aabb) {
                    continue;
                }

                if is_leaf(node) {
                    let proxy_id = proxy_id_of(node);
                    let Some(user_data) = self.user_data_for(node, mask_bits, require_all_bits) else {
                        continue;
                    };
                    result.leaf_visits += 1;
                    if !callback(proxy_id, user_data) {
                        return result;
                    }
                } else {
                    b3_assert(stack_count < TREE_STACK_SIZE - 1);
                    stack[stack_count] = left_child(node);
                    stack_count += 1;
                }
            }
        }

        result
    }

    /// Query an AABB for the closest object.
    pub fn query_closest<F>(
        &self,
        point: Vec3,
        mask_bits: u64,
        require_all_bits: bool,
        min_distance_sqr: f32,
        mut callback: F,
    ) -> (TreeStats, f32)
    where
        F: FnMut(f32, i32, u64) -> f32,
    {
        let mut result = TreeStats::default();
        let mut min_sqr = min_distance_sqr;

        if self.nodes.is_empty() || is_empty_node(self.nodes[ROOT_NODE as usize]) {
            return (result, min_sqr);
        }

        let distance_to_node_sqr = |node: TreeNode| -> f32 {
            let r = point - clamp_vec3(point, node.aabb.lower_bound, node.aabb.upper_bound);
            dot(r, r)
        };

        let mut stack = [(0i32, 0.0f32); 128];
        let mut stack_count = 0usize;

        let root = self.root_pair();
        let seed1 = (root, distance_to_node_sqr(self.nodes[root as usize]));

        if root == ROOT_NODE {
            stack[stack_count] = seed1;
            stack_count += 1;
        } else {
            let seed2 = (root + 1, distance_to_node_sqr(self.nodes[root as usize + 1]));
            if seed2.1 < seed1.1 {
                stack[stack_count] = seed1;
                stack[stack_count + 1] = seed2;
            } else {
                stack[stack_count] = seed2;
                stack[stack_count + 1] = seed1;
            }
            stack_count += 2;
        }

        while stack_count > 0 {
            stack_count -= 1;
            let (node_index, distance_to_node) = stack[stack_count];
            let node = self.nodes[node_index as usize];
            result.node_visits += 1;

            if distance_to_node >= min_sqr {
                continue;
            }

            if is_leaf(node) {
                let proxy_id = proxy_id_of(node);
                if let Some(user_data) = self.user_data_for(node, mask_bits, require_all_bits) {
                    let dd = callback(min_sqr, proxy_id, user_data);
                    if dd < min_sqr {
                        min_sqr = dd;
                    }
                    result.leaf_visits += 1;
                }
                continue;
            }

            b3_assert(stack_count + 2 <= TREE_STACK_SIZE);
            let pair = left_child(node);
            let item1 = (pair, distance_to_node_sqr(self.nodes[pair as usize]));
            let item2 = (pair + 1, distance_to_node_sqr(self.nodes[pair as usize + 1]));

            if item2.1 < item1.1 {
                stack[stack_count] = item1;
                stack[stack_count + 1] = item2;
            } else {
                stack[stack_count] = item2;
                stack[stack_count + 1] = item1;
            }
            stack_count += 2;
        }

        (result, min_sqr)
    }

    /// Ray cast against the proxies in the tree.
    pub fn ray_cast<F>(
        &self,
        input: RayCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        mut callback: F,
    ) -> TreeStats
    where
        F: FnMut(&mut RayCastInput, i32, u64) -> f32,
    {
        let mut result = TreeStats::default();
        if self.nodes.is_empty() || is_empty_node(self.nodes[ROOT_NODE as usize]) {
            return result;
        }

        let p1 = input.origin;
        let d = input.translation;
        let mut max_fraction = input.max_fraction;
        let mut p2 = mul_add(p1, max_fraction, d);
        let mut segment_aabb = AABB {
            lower_bound: min_vec3(p1, p2),
            upper_bound: max_vec3(p1, p2),
        };

        let mut stack = [0i32; 256];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root_pair();
        stack_count += 1;

        let mut sub_input = input;

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            result.node_visits += 1;

            let mut hits = [TreeNode::default(); 2];
            let mut is_leaf_hit = [false; 2];
            let mut hit_count = 0usize;
            for i in 0..2 {
                let node = self.nodes[(pair + i) as usize];
                if !aabb_overlaps(segment_aabb, node.aabb) {
                    continue;
                }
                if !test_bounds_ray_overlap(node.aabb.lower_bound, node.aabb.upper_bound, p1, d) {
                    continue;
                }
                is_leaf_hit[hit_count] = is_leaf(node);
                hits[hit_count] = node;
                hit_count += 1;
            }

            if hit_count == 2 && !is_leaf_hit[0] && !is_leaf_hit[1] {
                let d1 = distance_squared(aabb_center(hits[0].aabb), p1);
                let d2 = distance_squared(aabb_center(hits[1].aabb), p1);
                if d1 < d2 {
                    hits.swap(0, 1);
                }
            }

            for i in 0..hit_count {
                if is_leaf_hit[i] {
                    let proxy_id = proxy_id_of(hits[i]);
                    if let Some(user_data) = self.user_data_for(hits[i], mask_bits, require_all_bits) {
                        sub_input.max_fraction = max_fraction;
                        let value = callback(&mut sub_input, proxy_id, user_data);
                        result.leaf_visits += 1;

                        if value == 0.0 {
                            return result;
                        }

                        if 0.0 < value && value <= max_fraction {
                            max_fraction = value;
                            p2 = mul_add(p1, max_fraction, d);
                            segment_aabb.lower_bound = min_vec3(p1, p2);
                            segment_aabb.upper_bound = max_vec3(p1, p2);
                        }
                    }
                } else {
                    b3_assert(stack_count < TREE_STACK_SIZE - 1);
                    stack[stack_count] = left_child(hits[i]);
                    stack_count += 1;
                }
            }
        }

        result
    }

    /// Sweep an AABB through the tree.
    pub fn box_cast<F>(
        &self,
        input: BoxCastInput,
        mask_bits: u64,
        require_all_bits: bool,
        mut callback: F,
    ) -> TreeStats
    where
        F: FnMut(&mut BoxCastInput, i32, u64) -> f32,
    {
        let mut result = TreeStats::default();
        if self.nodes.is_empty() || is_empty_node(self.nodes[ROOT_NODE as usize]) {
            return result;
        }

        let origin_aabb = input.box_;
        let p1 = aabb_center(origin_aabb);
        let extension = max_vec3(p1 - origin_aabb.lower_bound, origin_aabb.upper_bound - p1);
        let d = input.translation;
        let mut max_fraction = input.max_fraction;
        let mut t = max_fraction * input.translation;
        let mut total_aabb = AABB {
            lower_bound: min_vec3(origin_aabb.lower_bound, origin_aabb.lower_bound + t),
            upper_bound: max_vec3(origin_aabb.upper_bound, origin_aabb.upper_bound + t),
        };

        let mut stack = [0i32; 256];
        let mut stack_count = 0usize;
        stack[stack_count] = self.root_pair();
        stack_count += 1;

        let mut sub_input = input;

        while stack_count > 0 {
            stack_count -= 1;
            let pair = stack[stack_count];
            result.node_visits += 1;

            let mut hits = [TreeNode::default(); 2];
            let mut is_leaf_hit = [false; 2];
            let mut hit_count = 0usize;
            for i in 0..2 {
                let node = self.nodes[(pair + i) as usize];
                if !aabb_overlaps(total_aabb, node.aabb) {
                    continue;
                }
                let lower = node.aabb.lower_bound - extension;
                let upper = node.aabb.upper_bound + extension;
                if !test_bounds_ray_overlap(lower, upper, p1, d) {
                    continue;
                }
                is_leaf_hit[hit_count] = is_leaf(node);
                hits[hit_count] = node;
                hit_count += 1;
            }

            if hit_count == 2 && !is_leaf_hit[0] && !is_leaf_hit[1] {
                let d1 = distance_squared(aabb_center(hits[0].aabb), p1);
                let d2 = distance_squared(aabb_center(hits[1].aabb), p1);
                if d1 < d2 {
                    hits.swap(0, 1);
                }
            }

            for i in 0..hit_count {
                if is_leaf_hit[i] {
                    let proxy_id = proxy_id_of(hits[i]);
                    if let Some(user_data) = self.user_data_for(hits[i], mask_bits, require_all_bits) {
                        sub_input.max_fraction = max_fraction;
                        let value = callback(&mut sub_input, proxy_id, user_data);
                        result.leaf_visits += 1;

                        if value == 0.0 {
                            return result;
                        }

                        if 0.0 < value && value < max_fraction {
                            max_fraction = value;
                            t = max_fraction * input.translation;
                            total_aabb.lower_bound =
                                min_vec3(origin_aabb.lower_bound, origin_aabb.lower_bound + t);
                            total_aabb.upper_bound =
                                max_vec3(origin_aabb.upper_bound, origin_aabb.upper_bound + t);
                        }
                    }
                } else {
                    b3_assert(stack_count < TREE_STACK_SIZE - 1);
                    stack[stack_count] = left_child(hits[i]);
                    stack_count += 1;
                }
            }
        }

        result
    }
}
