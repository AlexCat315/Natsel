//! Raw layouts stored inside a baked compound allocation.

use crate::math::types::{AABB, Transform, Vec3};
use crate::types::geometry::{Capsule, Sphere};

/// The compound header. The version is always first.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct CompoundData {
    pub version: u64,
    pub byte_count: i32,
    pub bounds: AABB,
    pub tree_height: i32,
    pub node_offset: i32,
    pub node_count: i32,
    pub material_offset: i32,
    pub material_count: i32,
    pub capsule_offset: i32,
    pub capsule_count: i32,
    pub hull_offset: i32,
    pub hull_count: i32,
    pub shared_hull_count: i32,
    pub mesh_offset: i32,
    pub mesh_count: i32,
    pub shared_mesh_count: i32,
    pub sphere_offset: i32,
    pub sphere_count: i32,
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct PackedCapsule {
    pub capsule: Capsule,
    pub material_index: u16,
    pub padding: u16,
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct PackedSphere {
    pub sphere: Sphere,
    pub material_index: u16,
    pub padding: u16,
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct PackedHullInstance {
    pub transform: Transform,
    pub hull_offset: u32,
    pub material_index: u16,
    pub padding: u16,
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct PackedMeshInstance {
    pub transform: Transform,
    pub scale: Vec3,
    pub mesh_offset: u32,
    pub material_offset: u32,
    pub material_count: u32,
}
