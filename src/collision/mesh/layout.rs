//! Mesh storage layout and header types.

use crate::math::types::AABB;
use crate::types::geometry::MeshNode;

/// Internal header for the owned mesh allocation.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct MeshData {
    pub version: u64,
    pub hash: u64,
    pub byte_count: i32,
    pub bounds: AABB,
    pub surface_area: f32,
    pub tree_height: i32,
    pub degenerate_count: i32,
    pub node_offset: i32,
    pub node_count: i32,
    pub vertex_offset: i32,
    pub vertex_count: i32,
    pub triangle_offset: i32,
    pub triangle_count: i32,
    pub material_offset: i32,
    pub material_count: i32,
    pub flags_offset: i32,
    pub padding: i32,
}

/// Byte offsets for a mesh allocation and its trailing arrays.
#[derive(Clone, Copy, Debug, Default)]
pub struct MeshStorageLayout {
    pub nodes: usize,
    pub vertices: usize,
    pub triangles: usize,
    pub materials: usize,
    pub flags: usize,
    pub byte_count: usize,
}

impl MeshStorageLayout {
    pub fn new(node_count: usize, vertex_count: usize, triangle_count: usize, material_count: usize) -> Self {
        fn aligned(count: usize) -> usize {
            (count + 7) & !7
        }
        let nodes = aligned(std::mem::size_of::<MeshData>());
        let vertices = nodes + aligned(node_count * std::mem::size_of::<MeshNode>());
        let triangles = vertices + aligned(vertex_count * std::mem::size_of::<crate::math::types::Vec3>());
        let materials = triangles + aligned(triangle_count * std::mem::size_of::<crate::types::geometry::MeshTriangle>());
        let flags = materials + aligned(material_count * 1);
        Self {
            nodes,
            vertices,
            triangles,
            materials,
            flags,
            byte_count: flags + aligned(triangle_count),
        }
    }
}

impl MeshNode {
    #[inline]
    pub fn is_leaf(&self) -> bool {
        self.data & 3 == 3
    }

    #[inline]
    pub fn child_offset(&self) -> usize {
        (self.data >> 2) as usize
    }

    #[inline]
    pub fn leaf_triangle_count(&self) -> usize {
        (self.data >> 2) as usize
    }

    #[inline]
    pub fn axis(&self) -> usize {
        (self.data & 3) as usize
    }
}
