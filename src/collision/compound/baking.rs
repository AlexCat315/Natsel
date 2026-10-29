//! Baking a compound builder into a single immutable allocation.

use crate::base::constants as b3;
use crate::base::core::{b3_alloc_zero, b3_assert};
use crate::collision::geometry_storage::GeometryStorage;

use super::builder::{CompoundBuilder, CompoundOffsets};
use super::packing::CompoundData;

unsafe fn store<T>(base: *mut u8, offset: usize, values: &[T]) {
    if values.is_empty() {
        return;
    }
    std::ptr::copy_nonoverlapping(
        values.as_ptr() as *const u8,
        base.add(offset),
        std::mem::size_of::<T>() * values.len(),
    );
}

/// Bake a compound builder into a single allocation.
pub fn bake_compound(builder: &CompoundBuilder) -> GeometryStorage {
    let layout = CompoundOffsets::new(builder);
    let base = b3_alloc_zero(layout.byte_count);
    b3_assert(!base.is_null());

    let header = CompoundData {
        version: b3::COMPOUND_VERSION,
        byte_count: layout.byte_count as i32,
        bounds: builder.tree.root_bounds(),
        tree_height: builder.tree.height(),
        node_offset: layout.nodes as i32,
        node_count: builder.tree.node_end,
        material_offset: layout.materials as i32,
        material_count: builder.materials.values.len() as i32,
        capsule_offset: layout.capsules as i32,
        capsule_count: builder.capsule_instances.len() as i32,
        hull_offset: layout.hulls as i32,
        hull_count: builder.hull_instances.len() as i32,
        shared_hull_count: builder.hulls.values.len() as i32,
        mesh_offset: layout.meshes as i32,
        mesh_count: builder.mesh_instances.len() as i32,
        shared_mesh_count: builder.meshes.values.len() as i32,
        sphere_offset: layout.spheres as i32,
        sphere_count: builder.sphere_instances.len() as i32,
    };

    unsafe {
        (base as *mut CompoundData).write(header);

        store(base, layout.nodes, &builder.tree.nodes[..builder.tree.node_end as usize]);
        store(base, layout.materials, &builder.materials.values);
        store(base, layout.capsules, &builder.capsule_instances);

        let mut hull_instances = builder.hull_instances.clone();
        for instance in &mut hull_instances {
            instance.hull_offset = layout.hull_blobs[instance.hull_offset as usize] as u32;
        }
        store(base, layout.hulls, &hull_instances);
        for (index, hull) in builder.hulls.values.iter().enumerate() {
            std::ptr::copy_nonoverlapping(
                hull.storage().base() as *const u8,
                base.add(layout.hull_blobs[index]),
                hull.byte_count() as usize,
            );
        }

        let mut mesh_instances = builder.mesh_instances.clone();
        for instance in &mut mesh_instances {
            instance.mesh_offset = layout.mesh_blobs[instance.mesh_offset as usize] as u32;
            instance.material_offset =
                layout.mesh_materials as u32 + instance.material_offset * 2;
        }
        store(base, layout.meshes, &mesh_instances);
        store(base, layout.mesh_materials, &builder.mesh_material_map);
        for (index, mesh) in builder.meshes.values.iter().enumerate() {
            let bytes = mesh.to_bytes();
            std::ptr::copy_nonoverlapping(
                bytes.as_ptr(),
                base.add(layout.mesh_blobs[index]),
                bytes.len(),
            );
        }

        store(base, layout.spheres, &builder.sphere_instances);

        GeometryStorage::from_raw(base, layout.byte_count)
    }
}
