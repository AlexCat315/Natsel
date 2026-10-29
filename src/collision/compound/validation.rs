//! Validation of serialized compound bytes.

use crate::base::constants as b3;
use crate::collision::mesh::layout::MeshData;
use crate::types::geometry::HullData;

use super::packing::{CompoundData, PackedHullInstance, PackedMeshInstance, PackedCapsule, PackedSphere};

/// Check that a serialized compound blob is self-consistent.
pub fn validate_compound_bytes(bytes: &[u8]) -> bool {
    let header_size = std::mem::size_of::<CompoundData>();
    if bytes.len() < header_size || bytes.len() > i32::MAX as usize {
        return false;
    }

    let header = unsafe { std::ptr::read_unaligned(bytes.as_ptr() as *const CompoundData) };
    if header.version != b3::COMPOUND_VERSION
        || header.byte_count as usize != bytes.len()
        || (header.node_offset as usize) < header_size
        || header.node_offset & 7 != 0
        || header.node_count < 2
    {
        return false;
    }

    let region = |offset: i32, count: i32, stride: usize| -> bool {
        offset >= 0
            && count >= 0
            && (offset as i64) + (count as i64) * (stride as i64) <= bytes.len() as i64
    };

    if !region(header.node_offset, header.node_count, std::mem::size_of::<crate::collision::dynamic_tree::TreeNode>())
        || !region(header.material_offset, header.material_count, std::mem::size_of::<crate::types::shape_def::SurfaceMaterial>())
        || !region(header.capsule_offset, header.capsule_count, std::mem::size_of::<PackedCapsule>())
        || !region(header.hull_offset, header.hull_count, std::mem::size_of::<PackedHullInstance>())
        || !region(header.mesh_offset, header.mesh_count, std::mem::size_of::<PackedMeshInstance>())
        || !region(header.sphere_offset, header.sphere_count, std::mem::size_of::<PackedSphere>())
    {
        return false;
    }

    let hull_stride = std::mem::size_of::<PackedHullInstance>();
    for i in 0..header.hull_count as usize {
        let instance = unsafe {
            std::ptr::read_unaligned(
                bytes
                    .as_ptr()
                    .add(header.hull_offset as usize + i * hull_stride)
                    as *const PackedHullInstance,
            )
        };
        let offset = instance.hull_offset as usize;
        if offset & 7 != 0 || offset + std::mem::size_of::<HullData>() > bytes.len() {
            return false;
        }
        let hull = unsafe { std::ptr::read_unaligned(bytes.as_ptr().add(offset) as *const HullData) };
        if hull.version != b3::HULL_VERSION
            || (hull.byte_count as usize) < std::mem::size_of::<HullData>()
            || offset + hull.byte_count as usize > bytes.len()
        {
            return false;
        }
    }

    let mesh_stride = std::mem::size_of::<PackedMeshInstance>();
    for i in 0..header.mesh_count as usize {
        let instance = unsafe {
            std::ptr::read_unaligned(
                bytes
                    .as_ptr()
                    .add(header.mesh_offset as usize + i * mesh_stride)
                    as *const PackedMeshInstance,
            )
        };
        let offset = instance.mesh_offset as usize;
        let material_offset = instance.material_offset as usize;
        let material_count = instance.material_count as usize;
        if offset & 7 != 0
            || instance.material_count == 0
            || offset + std::mem::size_of::<MeshData>() > bytes.len()
            || material_offset + material_count * 2 > bytes.len()
        {
            return false;
        }
        let mesh = unsafe { std::ptr::read_unaligned(bytes.as_ptr().add(offset) as *const MeshData) };
        if mesh.version != b3::MESH_VERSION
            || (mesh.byte_count as usize) < std::mem::size_of::<MeshData>()
            || offset + mesh.byte_count as usize > bytes.len()
        {
            return false;
        }
    }

    true
}
