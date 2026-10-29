//! Dynamic compound instances.
//!
//! Translated from `Sources/Natsel/Simulation/DynamicCompound.swift`.

use crate::collision::compound::compound::{Compound, CompoundChild, CompoundChildGeometry};
use crate::math::functions::transform_point;
use crate::math::types::Vec3;
use crate::simulation::world_runtime::World;
use crate::types::body_def::BodyType;
use crate::types::geometry::{Capsule, Sphere};
use crate::types::ids::{BodyID, ShapeID};
use crate::types::shape_def::ShapeDef;

/// A moving compound instantiated as convex fixtures on one body. Child ids follow the baked
/// Compound's child order, so query hits can be mapped back without changing existing ShapeID APIs.
/// Each child uses the world's broad phase, contact solver, sensors, and CCD.
#[derive(Clone, Debug)]
pub struct DynamicCompoundInstance {
    pub body_id: BodyID,
    pub child_shape_ids: Vec<ShapeID>,
}

impl DynamicCompoundInstance {
    /// Swift's `fileinit`; only `World`'s compound factory builds an instance.
    pub(crate) fn new(body_id: BodyID, child_shape_ids: Vec<ShapeID>) -> Self {
        Self {
            body_id,
            child_shape_ids,
        }
    }

    pub fn child_index_of(&self, shape: ShapeID) -> Option<usize> {
        self.child_shape_ids.iter().position(|id| *id == shape)
    }
}

impl World {
    /// Instantiate an immutable compound on a dynamic or kinematic body. Moving compounds must
    /// contain only convex children; meshes must be decomposed before instantiation. Invalid input
    /// leaves the body unchanged. Mass is computed once after all children are attached.
    pub fn create_dynamic_compound(
        &mut self,
        body: BodyID,
        definition: &ShapeDef,
        compound: &Compound,
    ) -> Option<DynamicCompoundInstance> {
        if self.locked || !self.is_valid_body(body) || self.body_type(body) == BodyType::StaticBody {
            return None;
        }
        let children: Vec<CompoundChild> =
            (0..compound.child_count()).map(|i| compound.child(i)).collect();
        if !children
            .iter()
            .all(|child| !matches!(child.geometry, CompoundChildGeometry::Mesh(_)))
        {
            return None;
        }
        let mut ids: Vec<ShapeID> = Vec::new();
        for child in &children {
            let mut def = definition.clone();
            def.update_body_mass = false;
            def.base_material = compound.materials()[child.material_indices[0] as usize];
            let id = match &child.geometry {
                CompoundChildGeometry::Sphere(sphere) => self.create_sphere_shape(
                    body,
                    &def,
                    Sphere::new(
                        transform_point(child.transform, sphere.center),
                        sphere.radius,
                    ),
                ),
                CompoundChildGeometry::Capsule(capsule) => self.create_capsule_shape(
                    body,
                    &def,
                    Capsule::new(
                        transform_point(child.transform, capsule.center1),
                        transform_point(child.transform, capsule.center2),
                        capsule.radius,
                    ),
                ),
                CompoundChildGeometry::Hull(hull) => {
                    self.create_transformed_hull_shape(body, &def, hull, child.transform, Vec3::ONE)
                }
                CompoundChildGeometry::Mesh(_) => {
                    // Validated before attaching any child.
                    return None;
                }
            };
            if !self.shape_is_valid(id) {
                for created in ids.iter() {
                    self.destroy_shape(*created, false);
                }
                if definition.update_body_mass {
                    self.apply_mass_from_shapes(body);
                }
                return None;
            }
            ids.push(id);
        }
        if definition.update_body_mass {
            self.apply_mass_from_shapes(body);
        }
        Some(DynamicCompoundInstance::new(body, ids))
    }

    pub fn dynamic_compound_is_valid(&self, instance: &DynamicCompoundInstance) -> bool {
        self.is_valid_body(instance.body_id)
            && instance.child_shape_ids.iter().all(|id| {
                self.shape_is_valid(*id)
                    && self.shapes[(id.index1 - 1) as usize].body_id == instance.body_id.index1 - 1
            })
    }

    /// Replace the children atomically with respect to stepping. Validate and instantiate the new
    /// children before releasing the old ones, retaining unrelated fixtures on the same body.
    pub fn replace_dynamic_compound(
        &mut self,
        instance: &DynamicCompoundInstance,
        definition: &ShapeDef,
        compound: &Compound,
    ) -> Option<DynamicCompoundInstance> {
        if self.locked || !self.dynamic_compound_is_valid(instance) {
            return None;
        }
        let mut def = definition.clone();
        def.update_body_mass = false;
        let body_id = instance.body_id;
        let Some(replacement) = self.create_dynamic_compound(body_id, &def, compound) else {
            return None;
        };
        self.destroy_dynamic_compound(instance, false);
        if definition.update_body_mass {
            self.apply_mass_from_shapes(body_id);
        }
        Some(replacement)
    }

    /// Release the remaining live children, including when some children were removed individually.
    /// Swift defaulted `updateBodyMass` to `true`; callers ported from the defaulted form pass
    /// `true`.
    pub fn destroy_dynamic_compound(
        &mut self,
        instance: &DynamicCompoundInstance,
        update_body_mass: bool,
    ) {
        if self.locked || !self.is_valid_body(instance.body_id) {
            return;
        }
        for id in instance.child_shape_ids.iter() {
            if self.shape_is_valid(*id)
                && self.shapes[(id.index1 - 1) as usize].body_id == instance.body_id.index1 - 1
            {
                self.destroy_shape(*id, false);
            }
        }
        if update_body_mass {
            self.apply_mass_from_shapes(instance.body_id);
        }
    }
}
