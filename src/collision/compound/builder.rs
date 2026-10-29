//! Compound construction and serialized layout offsets.

use crate::base::constants as b3;
use crate::collision::capsule::compute_capsule_aabb;
use crate::collision::dynamic_tree::{DynamicTree, TreeNode};
use crate::collision::hull::Hull;
use crate::collision::hull_queries::compute_hull_aabb;
use crate::collision::mesh::compute_mesh_aabb;
use crate::collision::mesh::storage::TriangleMesh;
use crate::collision::sphere::compute_sphere_aabb;
use crate::types::geometry::CompoundDef;
use crate::types::shape_def::SurfaceMaterial;

use super::interning::{geometry_bytes, CompoundInternTable};
use super::packing::{
    CompoundData, PackedCapsule, PackedHullInstance, PackedMeshInstance, PackedSphere,
};

pub struct CompoundBuilder {
    pub tree: DynamicTree,
    pub materials: CompoundInternTable<SurfaceMaterial>,
    pub hulls: CompoundInternTable<Hull>,
    pub meshes: CompoundInternTable<TriangleMesh>,
    pub capsule_instances: Vec<PackedCapsule>,
    pub hull_instances: Vec<PackedHullInstance>,
    pub mesh_instances: Vec<PackedMeshInstance>,
    pub sphere_instances: Vec<PackedSphere>,
    pub mesh_material_map: Vec<u16>,
}

impl CompoundBuilder {
    pub fn new(definition: &CompoundDef) -> Option<Self> {
        let count = definition.capsules.len()
            + definition.hulls.len()
            + definition.meshes.len()
            + definition.spheres.len();
        if count == 0 || count as i64 >= b3::MAX_CHILD_SHAPES {
            return None;
        }

        let mut builder = Self {
            tree: DynamicTree::new(count as i32),
            materials: CompoundInternTable::new(),
            hulls: CompoundInternTable::new(),
            meshes: CompoundInternTable::new(),
            capsule_instances: Vec::new(),
            hull_instances: Vec::new(),
            mesh_instances: Vec::new(),
            sphere_instances: Vec::new(),
            mesh_material_map: Vec::new(),
        };

        let mut child_index: u64 = 0;

        for item in &definition.capsules {
            let material_index = builder.intern_material(item.material)?;
            let instance = PackedCapsule {
                capsule: item.capsule,
                material_index,
                padding: 0,
            };
            builder.capsule_instances.push(instance);
            builder.tree.create_proxy(
                compute_capsule_aabb(&item.capsule, crate::math::types::Transform::IDENTITY),
                u64::MAX,
                child_index,
            );
            child_index += 1;
        }

        for item in &definition.hulls {
            let hull = item.hull.as_ref()?;
            let material_index = builder.intern_material(item.material)?;
            builder
                .tree
                .create_proxy(compute_hull_aabb(hull, item.transform), u64::MAX, child_index);
            child_index += 1;

            let hull_offset = builder
                .hulls
                .insert(hull.clone(), geometry_bytes(hull.storage())) as u32;
            builder.hull_instances.push(PackedHullInstance {
                transform: item.transform,
                hull_offset,
                material_index,
                padding: 0,
            });
        }

        for item in &definition.meshes {
            let mesh = item.mesh_data.as_ref()?;
            if item.materials.is_empty()
                || item.materials.len() != mesh.header().material_count as usize
            {
                return None;
            }
            builder.tree.create_proxy(
                compute_mesh_aabb(mesh, item.transform, item.scale),
                u64::MAX,
                child_index,
            );
            child_index += 1;

            let start = builder.mesh_material_map.len();
            for material in &item.materials {
                let index = builder.intern_material(*material)?;
                builder.mesh_material_map.push(index);
            }

            let mesh_offset = builder.meshes.insert(mesh.clone(), mesh.to_bytes()) as u32;
            builder.mesh_instances.push(PackedMeshInstance {
                transform: item.transform,
                scale: item.scale,
                mesh_offset,
                material_offset: start as u32,
                material_count: item.materials.len() as u32,
            });
        }

        for item in &definition.spheres {
            let material_index = builder.intern_material(item.material)?;
            let instance = PackedSphere {
                sphere: item.sphere,
                material_index,
                padding: 0,
            };
            builder.sphere_instances.push(instance);
            builder.tree.create_proxy(
                compute_sphere_aabb(&item.sphere, crate::math::types::Transform::IDENTITY),
                u64::MAX,
                child_index,
            );
            child_index += 1;
        }

        builder.tree.rebuild(true);
        Some(builder)
    }

    fn intern_material(&mut self, material: SurfaceMaterial) -> Option<u16> {
        let bytes = unsafe {
            std::slice::from_raw_parts(
                &material as *const SurfaceMaterial as *const u8,
                std::mem::size_of::<SurfaceMaterial>(),
            )
        }
        .to_vec();
        let index = self.materials.insert(material, bytes);
        if self.materials.values.len() > u16::MAX as usize {
            return None;
        }
        Some(index as u16)
    }
}

/// Byte offsets of every array inside a baked compound allocation.
#[derive(Clone, Debug, Default)]
pub struct CompoundOffsets {
    pub nodes: usize,
    pub materials: usize,
    pub capsules: usize,
    pub hulls: usize,
    pub hull_blobs: Vec<usize>,
    pub meshes: usize,
    pub mesh_materials: usize,
    pub mesh_blobs: Vec<usize>,
    pub spheres: usize,
    pub byte_count: usize,
}

impl CompoundOffsets {
    pub fn new(builder: &CompoundBuilder) -> Self {
        fn aligned(count: usize) -> usize {
            (count + 7) & !7
        }

        let nodes = aligned(std::mem::size_of::<CompoundData>());
        let node_bytes = builder.tree.node_end as usize * std::mem::size_of::<TreeNode>();
        let materials = nodes + aligned(node_bytes);
        let material_bytes =
            builder.materials.values.len() * std::mem::size_of::<SurfaceMaterial>();
        let capsules = materials + aligned(material_bytes);
        let capsule_bytes = builder.capsule_instances.len() * std::mem::size_of::<PackedCapsule>();
        let hulls = capsules + aligned(capsule_bytes);

        let hull_instance_bytes =
            builder.hull_instances.len() * std::mem::size_of::<PackedHullInstance>();
        let mut offset = hulls + aligned(hull_instance_bytes);
        let mut hull_blobs = Vec::with_capacity(builder.hulls.values.len());
        for hull in &builder.hulls.values {
            hull_blobs.push(offset);
            offset += aligned(hull.byte_count() as usize);
        }
        let meshes = offset;

        let mesh_instance_bytes =
            builder.mesh_instances.len() * std::mem::size_of::<PackedMeshInstance>();
        let mesh_materials = meshes + aligned(mesh_instance_bytes);
        offset = mesh_materials + aligned(builder.mesh_material_map.len() * 2);
        let mut mesh_blobs = Vec::with_capacity(builder.meshes.values.len());
        for mesh in &builder.meshes.values {
            mesh_blobs.push(offset);
            offset += aligned(mesh.byte_count() as usize);
        }
        let spheres = offset;
        let sphere_bytes = builder.sphere_instances.len() * std::mem::size_of::<PackedSphere>();
        let byte_count = spheres + aligned(sphere_bytes);

        Self {
            nodes,
            materials,
            capsules,
            hulls,
            hull_blobs,
            meshes,
            mesh_materials,
            mesh_blobs,
            spheres,
            byte_count,
        }
    }
}
