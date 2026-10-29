//! Compound contact generation.
//!
//! Translated from `Sources/Natsel/Simulation/CompoundContact.swift`.
//!
//! Foreign vocabulary assumed from sibling modules (translated concurrently; see MIGRATION.md
//! "Shared simulation vocabulary"):
//! - `crate::simulation::mesh_contact::update`: `World::compute_mesh_manifolds(contact_id,
//!   shape_a, transform_a, shape_b, transform_b, is_fast, material_indices, scratch, cs) -> bool`
//!   with `material_indices: Option<&[u16]>`, replacing Swift's defaulted
//!   `materialIndices: [UInt16]? = nil`. This call site passes the child's indices.

use crate::collision::compound::compound::CompoundChildGeometry;
use crate::math::functions::{mul_world_transforms, rotate_vector};
use crate::math::types::WorldTransform;
use crate::simulation::body_api::ContactFlags;
use crate::simulation::contact_payload::MeshContactScratch;
use crate::simulation::contact_store::ContactStore;
use crate::simulation::shape_runtime::{Shape, ShapeFlags, ShapeGeometry};
use crate::simulation::world_runtime::World;
use crate::types::shape_def::ShapeType;

impl World {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn update_compound_contact(
        &self,
        contact_id: i32,
        shape_a: &Shape,
        xf_a: WorldTransform,
        shape_b: &Shape,
        xf_b: WorldTransform,
        is_fast: bool,
        scratch: &mut MeshContactScratch,
        cs: &mut ContactStore,
    ) -> bool {
        let ShapeGeometry::Compound(compound) = &shape_a.geometry else {
            return false;
        };
        let child = compound.child(cs.get(contact_id).child_index as usize);
        let mut temporary = shape_a.clone();
        let touching: bool;
        match &child.geometry {
            CompoundChildGeometry::Capsule(capsule) => {
                temporary.type_ = ShapeType::CapsuleShape;
                temporary.geometry = ShapeGeometry::Capsule(*capsule);
                temporary.materials = vec![shape_a.material(child.material_indices[0] as i32)];
                if shape_b.type_ == ShapeType::HullShape {
                    touching = self.update_convex_contact(
                        contact_id,
                        shape_b,
                        xf_b,
                        &temporary,
                        xf_a,
                        true,
                        scratch,
                        cs,
                    );
                } else {
                    touching = self.update_convex_contact(
                        contact_id,
                        &temporary,
                        xf_a,
                        shape_b,
                        xf_b,
                        false,
                        scratch,
                        cs,
                    );
                }
            }
            CompoundChildGeometry::Hull(hull) => {
                temporary.type_ = ShapeType::HullShape;
                temporary.geometry = ShapeGeometry::Hull(hull.clone());
                temporary.materials = vec![shape_a.material(child.material_indices[0] as i32)];
                touching = self.update_convex_contact(
                    contact_id,
                    &temporary,
                    mul_world_transforms(xf_a, child.transform),
                    shape_b,
                    xf_b,
                    false,
                    scratch,
                    cs,
                );
            }
            CompoundChildGeometry::Mesh(mesh) => {
                temporary.type_ = ShapeType::MeshShape;
                temporary.geometry = ShapeGeometry::Mesh(mesh.clone());
                touching = self.compute_mesh_manifolds(
                    contact_id,
                    &temporary,
                    mul_world_transforms(xf_a, child.transform),
                    shape_b,
                    xf_b,
                    is_fast,
                    Some(&child.material_indices),
                    scratch,
                    cs,
                );
            }
            CompoundChildGeometry::Sphere(sphere) => {
                temporary.type_ = ShapeType::SphereShape;
                temporary.geometry = ShapeGeometry::Sphere(*sphere);
                temporary.materials = vec![shape_a.material(child.material_indices[0] as i32)];
                if shape_b.type_ == ShapeType::CapsuleShape
                    || shape_b.type_ == ShapeType::HullShape
                {
                    touching = self.update_convex_contact(
                        contact_id,
                        shape_b,
                        xf_b,
                        &temporary,
                        xf_a,
                        true,
                        scratch,
                        cs,
                    );
                } else {
                    touching = self.update_convex_contact(
                        contact_id,
                        &temporary,
                        xf_a,
                        shape_b,
                        xf_b,
                        false,
                        scratch,
                        cs,
                    );
                }
            }
        }
        if matches!(child.geometry, CompoundChildGeometry::Mesh(_)) {
            let mut contact = cs.get(contact_id);
            if touching
                && ((shape_a.flags | shape_b.flags) & ShapeFlags::ENABLE_HIT_EVENTS.0) != 0
            {
                contact.flags |= ContactFlags::SIM_ENABLE_HIT_EVENT.0;
            } else {
                contact.flags &= !ContactFlags::SIM_ENABLE_HIT_EVENT.0;
            }
            cs.set(contact_id, contact);
        }
        let offset = rotate_vector(xf_a.q, child.transform.p);
        let contact = cs.get(contact_id);
        {
            let manifold_count = contact.manifold_count as usize;
            let view = contact.manifolds();
            for i in 0..manifold_count {
                let manifold = view.get_mut(i);
                for j in 0..manifold.point_count as usize {
                    manifold.points[j].anchor_a = manifold.points[j].anchor_a + offset;
                }
            }
        }
        cs.set(contact_id, contact);
        touching
    }
}
