//! Immutable triangle geometry shared by scaled mesh instances.

use std::sync::Arc;

use crate::base::constants as b3;
use crate::base::hash::hash64_non_zero;
use crate::collision::geometry_storage::GeometryStorage;
use crate::math::functions::*;
use crate::math::types::{AABB, Transform, Vec3};
use crate::types::geometry::{MeshTriangle, MeshNode};

use super::build::{mesh_hash_inputs, prepare_mesh, MeshBuildOptions, PreparedMesh};
use super::layout::{MeshData, MeshStorageLayout};

#[derive(Clone)]
struct MeshInner {
    data: super::layout::MeshData,
}

/// Immutable triangle geometry shared by scaled mesh instances.
#[derive(Clone)]
pub struct TriangleMesh {
    inner: Arc<MeshInner>,
    vertices: Arc<Vec<Vec3>>,
    triangles: Arc<Vec<MeshTriangle>>,
    nodes: Arc<Vec<MeshNode>>,
    materials: Arc<Vec<u8>>,
    flags: Arc<Vec<u8>>,
}

impl core::fmt::Debug for TriangleMesh {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("TriangleMesh")
            .field("vertex_count", &self.vertex_count())
            .field("triangle_count", &self.triangle_count())
            .field("hash", &self.hash())
            .finish()
    }
}

impl TriangleMesh {
    /// Build from triangle soup.
    pub fn new(vertices: &[Vec3], indices: &[i32], materials: Option<&[u8]>, options: MeshBuildOptions) -> Option<Self> {
        let prepared = prepare_mesh(vertices, indices, materials, options)?;
        Some(Self::from_prepared(prepared))
    }

    pub fn from_prepared(prepared: PreparedMesh) -> Self {
        let layout = super::layout::MeshStorageLayout::new(
            prepared.nodes.len(),
            prepared.vertices.len(),
            prepared.triangles.len(),
            prepared.material_count,
        );

        let mut data = super::layout::MeshData::default();
        data.version = b3::MESH_VERSION;
        data.byte_count = layout.byte_count as i32;
        data.bounds = prepared.bounds;
        data.surface_area = prepared.surface_area;
        data.tree_height = prepared.tree_height as i32;
        data.degenerate_count = prepared.degenerate_count as i32;
        data.node_offset = layout.nodes as i32;
        data.node_count = prepared.nodes.len() as i32;
        data.vertex_offset = layout.vertices as i32;
        data.vertex_count = prepared.vertices.len() as i32;
        data.triangle_offset = layout.triangles as i32;
        data.triangle_count = prepared.triangles.len() as i32;
        data.material_offset = layout.materials as i32;
        data.material_count = prepared.material_count as i32;
        data.flags_offset = layout.flags as i32;

        let hash_input = mesh_hash_inputs(
            &prepared.vertices,
            &prepared.triangles,
            &prepared.nodes,
            &prepared.materials,
            &prepared.flags,
        );
        data.hash = hash64_non_zero(&hash_input);

        Self::from_parts(
            data,
            prepared.vertices,
            prepared.triangles,
            prepared.nodes,
            prepared.materials,
            prepared.flags,
        )
    }

    fn from_parts(
        data: MeshData,
        vertices: Vec<Vec3>,
        triangles: Vec<MeshTriangle>,
        nodes: Vec<MeshNode>,
        materials: Vec<u8>,
        flags: Vec<u8>,
    ) -> Self {
        Self {
            inner: Arc::new(MeshInner { data }),
            vertices: Arc::new(vertices),
            triangles: Arc::new(triangles),
            nodes: Arc::new(nodes),
            materials: Arc::new(materials),
            flags: Arc::new(flags),
        }
    }

    /// Rebuild a triangle mesh from a serialized blob.
    pub fn from_storage(storage: GeometryStorage) -> Option<Self> {
        let data = unsafe { std::ptr::read_unaligned(storage.base() as *const MeshData) };
        if data.version != b3::MESH_VERSION {
            return None;
        }
        let byte_count = data.byte_count;
        if byte_count < (std::mem::size_of::<MeshData>() as i32) || byte_count as usize > storage.byte_count() {
            return None;
        }
        let region = |offset: i32, count: i32, stride: usize| -> bool {
            offset >= 0
                && count >= 0
                && (offset as usize) + (count as usize) * stride <= byte_count as usize
        };
        if !region(data.node_offset, data.node_count, std::mem::size_of::<MeshNode>())
            || !region(data.vertex_offset, data.vertex_count, std::mem::size_of::<Vec3>())
            || !region(data.triangle_offset, data.triangle_count, std::mem::size_of::<MeshTriangle>())
            || !region(data.material_offset, data.material_count, 1)
            || !region(data.flags_offset, data.triangle_count, 1)
        {
            return None;
        }

        let nodes = unsafe {
            storage
                .slice_ref::<MeshNode>(data.node_offset as usize, data.node_count as usize)
                .to_vec()
        };
        let vertices = unsafe {
            storage
                .slice_ref::<Vec3>(data.vertex_offset as usize, data.vertex_count as usize)
                .to_vec()
        };
        let triangles = unsafe {
            storage
                .slice_ref::<MeshTriangle>(data.triangle_offset as usize, data.triangle_count as usize)
                .to_vec()
        };
        let materials = unsafe {
            storage
                .slice_ref::<u8>(data.material_offset as usize, data.material_count as usize)
                .to_vec()
        };
        let flags = unsafe {
            storage
                .slice_ref::<u8>(data.flags_offset as usize, data.triangle_count as usize)
                .to_vec()
        };

        Some(Self::from_parts(data, vertices, triangles, nodes, materials, flags))
    }

    /// Serialize the mesh into a contiguous allocation layout.
    pub fn to_bytes(&self) -> Vec<u8> {
        let layout = MeshStorageLayout::new(
            self.nodes.len(),
            self.vertices.len(),
            self.triangles.len(),
            self.inner.data.material_count as usize,
        );
        let mut bytes = vec![0u8; layout.byte_count];
        unsafe {
            std::ptr::write_unaligned(bytes.as_mut_ptr() as *mut MeshData, self.inner.data);
            write_array(&mut bytes, layout.nodes, &self.nodes);
            write_array(&mut bytes, layout.vertices, &self.vertices);
            write_array(&mut bytes, layout.triangles, &self.triangles);
            write_array(&mut bytes, layout.materials, &self.materials);
            write_array(&mut bytes, layout.flags, &self.flags);
        }
        bytes
    }

    pub fn header(&self) -> &super::layout::MeshData {
        &self.inner.data
    }

    pub fn vertices(&self) -> &[Vec3] {
        &self.vertices
    }

    pub fn triangles(&self) -> &[MeshTriangle] {
        &self.triangles
    }

    pub fn nodes(&self) -> &[MeshNode] {
        &self.nodes
    }

    pub fn material_indices(&self) -> &[u8] {
        &self.materials
    }

    pub fn edge_flags(&self) -> &[u8] {
        &self.flags
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    pub fn bounds(&self) -> AABB {
        self.inner.data.bounds
    }

    pub fn surface_area(&self) -> f32 {
        self.inner.data.surface_area
    }

    pub fn hash(&self) -> u64 {
        self.inner.data.hash
    }

    pub fn byte_count(&self) -> i32 {
        self.inner.data.byte_count
    }

    pub fn degenerate_triangle_count(&self) -> usize {
        self.inner.data.degenerate_count as usize
    }

    pub fn triangle_vertices(&self, index: usize) -> [Vec3; 3] {
        let t = self.triangles[index];
        [
            self.vertices[t.index1 as usize],
            self.vertices[t.index2 as usize],
            self.vertices[t.index3 as usize],
        ]
    }
}

unsafe fn write_array<T>(bytes: &mut [u8], offset: usize, values: &[T]) {
    if values.is_empty() {
        return;
    }
    std::ptr::copy_nonoverlapping(
        values.as_ptr() as *const u8,
        bytes.as_mut_ptr().add(offset),
        std::mem::size_of::<T>() * values.len(),
    );
}

/// Compute the world AABB of a mesh instance.
pub fn compute_mesh_aabb(mesh: &TriangleMesh, transform: Transform, scale: Vec3) -> AABB {
    let lower = Vec3::new(
        scale.x * mesh.bounds().lower_bound.x,
        scale.y * mesh.bounds().lower_bound.y,
        scale.z * mesh.bounds().lower_bound.z,
    );
    let upper = Vec3::new(
        scale.x * mesh.bounds().upper_bound.x,
        scale.y * mesh.bounds().upper_bound.y,
        scale.z * mesh.bounds().upper_bound.z,
    );
    aabb_transform(transform, AABB::new(min_vec3(lower, upper), max_vec3(lower, upper)))
}
