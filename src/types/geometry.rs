//! Geometry definitions and shared geometry instance types.

use crate::collision::compound::compound::Compound;
use crate::collision::height_field::quantization::HeightField;
use crate::collision::hull::Hull;
use crate::collision::mesh::storage::TriangleMesh;
use crate::math::types::{AABB, Matrix3, Transform, Vec3};

// MARK: - Sphere

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Sphere {
    pub center: Vec3,
    pub radius: f32,
}

impl Sphere {
    pub fn new(center: Vec3, radius: f32) -> Self {
        Self { center, radius }
    }
}

// MARK: - Capsule

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct Capsule {
    pub center1: Vec3,
    pub center2: Vec3,
    pub radius: f32,
}

impl Capsule {
    pub fn new(center1: Vec3, center2: Vec3, radius: f32) -> Self {
        Self {
            center1,
            center2,
            radius,
        }
    }
}

// MARK: - Hull

/// A hull vertex. Identified by a half-edge with this vertex as its tail.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct HullVertex {
    pub edge: u8,
}

/// Half-edge for the hull data structure.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct HullHalfEdge {
    pub next: u8,
    pub twin: u8,
    pub origin: u8,
    pub face: u8,
}

/// A hull face. Hulls use a half-edge data structure.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct HullFace {
    pub edge: u8,
}

/// A convex hull serialized header.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct HullData {
    pub version: u64,
    pub hash: u64,
    pub aabb: AABB,
    pub surface_area: f32,
    pub volume: f32,
    pub inner_radius: f32,
    pub center: Vec3,
    pub central_inertia: Matrix3,
    pub vertex_count: i32,
    pub vertex_offset: i32,
    pub point_offset: i32,
    pub edge_count: i32,
    pub edge_offset: i32,
    pub face_count: i32,
    pub plane_offset: i32,
    pub face_offset: i32,
    pub soa_vertex_offset: i32,
    pub soa_normal_offset: i32,
    pub edge_cosine_offset: i32,
    pub byte_count: i32,
    pub padding: i32,
}

// MARK: - Mesh

/// Mesh edge flags.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct MeshEdgeFlags(pub u8);

impl MeshEdgeFlags {
    pub const CONCAVE_EDGE1: MeshEdgeFlags = MeshEdgeFlags(0x01);
    pub const CONCAVE_EDGE2: MeshEdgeFlags = MeshEdgeFlags(0x02);
    pub const CONCAVE_EDGE3: MeshEdgeFlags = MeshEdgeFlags(0x04);
    pub const INVERSE_CONCAVE_EDGE1: MeshEdgeFlags = MeshEdgeFlags(0x10);
    pub const INVERSE_CONCAVE_EDGE2: MeshEdgeFlags = MeshEdgeFlags(0x20);
    pub const INVERSE_CONCAVE_EDGE3: MeshEdgeFlags = MeshEdgeFlags(0x40);
}

/// A mesh triangle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct MeshTriangle {
    pub index1: i32,
    pub index2: i32,
    pub index3: i32,
}

/// A mesh BVH node.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct MeshNode {
    pub lower_bound: Vec3,
    pub data: u32,
    pub upper_bound: Vec3,
    pub triangle_offset: u32,
}

/// This allows mesh data to be re-used with different scales.
#[derive(Clone, Debug, Default)]
pub struct Mesh {
    /// Shared immutable triangle geometry.
    pub data: Option<TriangleMesh>,
    /// This scale may be non-uniform and have negative components.
    pub scale: Vec3,
}

impl Mesh {
    pub fn new(data: Option<TriangleMesh>, scale: Vec3) -> Self {
        Self { data, scale }
    }
}

// MARK: - Height Field

/// Data used to create a height field.
#[derive(Clone, Debug, PartialEq)]
pub struct HeightFieldDef {
    pub heights: Vec<f32>,
    pub material_indices: Vec<u8>,
    pub scale: Vec3,
    pub count_x: i32,
    pub count_z: i32,
    pub global_minimum_height: f32,
    pub global_maximum_height: f32,
    pub clockwise_winding: bool,
}

impl Default for HeightFieldDef {
    fn default() -> Self {
        Self {
            heights: Vec::new(),
            material_indices: Vec::new(),
            scale: Vec3::new(1.0, 1.0, 1.0),
            count_x: 0,
            count_z: 0,
            global_minimum_height: 0.0,
            global_maximum_height: 0.0,
            clockwise_winding: false,
        }
    }
}

// MARK: - Compound

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct CompoundCapsuleDef {
    pub capsule: Capsule,
    pub material: crate::types::shape_def::SurfaceMaterial,
}

#[derive(Clone, Debug, Default)]
pub struct CompoundHullDef {
    pub hull: Option<Hull>,
    pub transform: Transform,
    pub material: crate::types::shape_def::SurfaceMaterial,
}

#[derive(Clone, Debug, Default)]
pub struct CompoundMeshDef {
    pub mesh_data: Option<TriangleMesh>,
    pub transform: Transform,
    pub scale: Vec3,
    pub materials: Vec<crate::types::shape_def::SurfaceMaterial>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct CompoundSphereDef {
    pub sphere: Sphere,
    pub material: crate::types::shape_def::SurfaceMaterial,
}

#[derive(Clone, Debug, Default)]
pub struct CompoundDef {
    pub capsules: Vec<CompoundCapsuleDef>,
    pub hulls: Vec<CompoundHullDef>,
    pub meshes: Vec<CompoundMeshDef>,
    pub spheres: Vec<CompoundSphereDef>,
}

// MARK: - Mass Data

/// This holds the mass data computed for a shape.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct MassData {
    pub mass: f32,
    pub center: Vec3,
    pub inertia: Matrix3,
}

/// The compound owner type is re-exported for debug drawing.
pub type CompoundGeometry = Compound;
/// The height-field owner type is re-exported for debug drawing.
pub type HeightFieldGeometry = HeightField;
