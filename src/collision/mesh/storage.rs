//! Immutable triangle geometry shared by scaled mesh instances.

use std::sync::Arc;

use crate::base::constants as b3;
use crate::base::hash::hash64_non_zero;
use crate::math::functions::*;
use crate::math::types::{AABB, Transform, Vec3};
use crate::types::geometry::{MeshTriangle, MeshNode};

use super::build::{mesh_hash_inputs, prepare_mesh, MeshBuildOptions, PreparedMesh};

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
        data.hash = hash64_non_zero(&hash_input, hash_input.len() as i32);

        Self {
            inner: Arc::new(MeshInner { data }),
            vertices: Arc::new(prepared.vertices),
            triangles: Arc::new(prepared.triangles),
            nodes: Arc::new(prepared.nodes),
            materials: Arc::new(prepared.materials),
            flags: Arc::new(prepared.flags),
        }
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
