//! A baked, immutable compound with shared child geometry and materials.

use std::collections::HashMap;

use crate::collision::dynamic_tree::{TreeNode, TreeView};
use crate::collision::geometry_storage::GeometryStorage;
use crate::collision::hull::Hull;
use crate::collision::mesh::storage::TriangleMesh;
use crate::math::functions::aabb_transform;
use crate::math::types::{AABB, Transform};
use crate::types::geometry::{Capsule, CompoundDef, HullData, Mesh, Sphere};
use crate::types::shape_def::SurfaceMaterial;

use super::baking::bake_compound;
use super::builder::CompoundBuilder;
use super::interning::geometry_bytes;
use super::packing::{CompoundData, PackedCapsule, PackedHullInstance, PackedMeshInstance, PackedSphere};
use super::validation::validate_compound_bytes;

/// The geometry of a compound child.
#[derive(Clone, Debug)]
pub enum CompoundChildGeometry {
    Capsule(Capsule),
    Hull(Hull),
    Mesh(Mesh),
    Sphere(Sphere),
}

/// A compound child with its local transform and material indices.
#[derive(Clone, Debug)]
pub struct CompoundChild {
    pub geometry: CompoundChildGeometry,
    pub transform: Transform,
    pub material_indices: Vec<u16>,
}

/// A baked, immutable compound with shared child geometry and materials.
#[derive(Clone)]
pub struct Compound {
    storage: GeometryStorage,
    hull_views: HashMap<u32, Hull>,
    mesh_views: HashMap<u32, TriangleMesh>,
}

impl Compound {
    pub fn new(definition: &CompoundDef) -> Option<Self> {
        let builder = CompoundBuilder::new(definition)?;
        Some(Self::from_storage(bake_compound(&builder)))
    }

    pub fn from_serialized_bytes(bytes: &[u8]) -> Option<Self> {
        if !validate_compound_bytes(bytes) {
            return None;
        }
        let storage = GeometryStorage::new(bytes.len())?;
        unsafe {
            std::ptr::copy_nonoverlapping(bytes.as_ptr(), storage.base(), bytes.len());
        }
        Some(Self::from_storage(storage))
    }

    fn from_storage(storage: GeometryStorage) -> Self {
        let header = unsafe { std::ptr::read_unaligned(storage.base() as *const CompoundData) };
        let mut hull_views = HashMap::new();
        for i in 0..header.hull_count as usize {
            let instance: PackedHullInstance = unsafe { read_value(&storage, header.hull_offset, i) };
            let offset = instance.hull_offset as usize;
            if !hull_views.contains_key(&instance.hull_offset) {
                let data =
                    unsafe { std::ptr::read_unaligned(storage.base().add(offset) as *const HullData) };
                hull_views.insert(
                    instance.hull_offset,
                    Hull::from_storage(storage.slice(offset, data.byte_count as usize)),
                );
            }
        }

        let mut mesh_views = HashMap::new();
        for i in 0..header.mesh_count as usize {
            let instance: PackedMeshInstance = unsafe { read_value(&storage, header.mesh_offset, i) };
            let offset = instance.mesh_offset as usize;
            if !mesh_views.contains_key(&instance.mesh_offset) {
                let data = unsafe {
                    std::ptr::read_unaligned(
                        storage.base().add(offset) as *const crate::collision::mesh::layout::MeshData,
                    )
                };
                if let Some(mesh) =
                    TriangleMesh::from_storage(storage.slice(offset, data.byte_count as usize))
                {
                    mesh_views.insert(instance.mesh_offset, mesh);
                }
            }
        }

        Self {
            storage,
            hull_views,
            mesh_views,
        }
    }

    pub fn header(&self) -> CompoundData {
        unsafe { std::ptr::read_unaligned(self.storage.base() as *const CompoundData) }
    }

    pub fn byte_count(&self) -> i32 {
        self.storage.byte_count() as i32
    }

    pub fn child_count(&self) -> usize {
        let header = self.header();
        (header.capsule_count + header.hull_count + header.mesh_count + header.sphere_count)
            as usize
    }

    pub fn bounds(&self) -> AABB {
        self.header().bounds
    }

    pub fn materials(&self) -> Vec<SurfaceMaterial> {
        let header = self.header();
        unsafe {
            self.storage
                .slice_ref::<SurfaceMaterial>(
                    header.material_offset as usize,
                    header.material_count as usize,
                )
                .to_vec()
        }
    }

    pub fn serialized_bytes(&self) -> Vec<u8> {
        geometry_bytes(&self.storage)
    }

    pub fn child(&self, index: usize) -> CompoundChild {
        assert!(index < self.child_count());
        let header = self.header();
        let mut index = index;

        if index < header.capsule_count as usize {
            let instance: PackedCapsule =
                unsafe { read_value(&self.storage, header.capsule_offset, index) };
            return CompoundChild {
                geometry: CompoundChildGeometry::Capsule(instance.capsule),
                transform: Transform::IDENTITY,
                material_indices: vec![instance.material_index],
            };
        }
        index -= header.capsule_count as usize;

        if index < header.hull_count as usize {
            let instance: PackedHullInstance =
                unsafe { read_value(&self.storage, header.hull_offset, index) };
            let hull = self.hull_views[&instance.hull_offset].clone();
            return CompoundChild {
                geometry: CompoundChildGeometry::Hull(hull),
                transform: instance.transform,
                material_indices: vec![instance.material_index],
            };
        }
        index -= header.hull_count as usize;

        if index < header.mesh_count as usize {
            let instance: PackedMeshInstance =
                unsafe { read_value(&self.storage, header.mesh_offset, index) };
            let indices = unsafe {
                self.storage
                    .slice_ref::<u16>(
                        instance.material_offset as usize,
                        instance.material_count as usize,
                    )
                    .to_vec()
            };
            let mesh = Mesh::new(Some(self.mesh_views[&instance.mesh_offset].clone()), instance.scale);
            return CompoundChild {
                geometry: CompoundChildGeometry::Mesh(mesh),
                transform: instance.transform,
                material_indices: indices,
            };
        }
        index -= header.mesh_count as usize;

        let instance: PackedSphere = unsafe { read_value(&self.storage, header.sphere_offset, index) };
        CompoundChild {
            geometry: CompoundChildGeometry::Sphere(instance.sphere),
            transform: Transform::IDENTITY,
            material_indices: vec![instance.material_index],
        }
    }

    pub fn with_tree_view<R>(&self, body: impl FnOnce(TreeView<'_>) -> R) -> R {
        let header = self.header();
        let nodes = unsafe {
            self.storage
                .slice_ref::<TreeNode>(header.node_offset as usize, header.node_count as usize)
        };
        body(TreeView::new(nodes, None))
    }
}

impl core::fmt::Debug for Compound {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Compound")
            .field("child_count", &self.child_count())
            .field("byte_count", &self.byte_count())
            .finish()
    }
}

unsafe fn read_value<T: Copy>(storage: &GeometryStorage, offset: i32, index: usize) -> T {
    std::ptr::read_unaligned(
        (storage.base().add(offset as usize) as *const T).add(index),
    )
}

/// Compute the world AABB of a compound.
pub fn compute_compound_aabb(compound: &Compound, transform: Transform) -> AABB {
    aabb_transform(transform, compound.bounds())
}
