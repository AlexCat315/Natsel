//! Blending per-triangle materials into a mesh contact's response parameters.
//!
//! Translated from `Sources/Natsel/Simulation/MeshContact/Materials.swift`.

use crate::math::functions::{clamp_int, rotate_vector};
use crate::math::types::{Vec3, WorldTransform};
use crate::simulation::shape_runtime::{Shape, ShapeGeometry};
use crate::simulation::triangle_surface::TriangleSurface;
use crate::simulation::world_runtime::{Contact, World};

impl World {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn mix_mesh_materials(
        &self,
        contact: &mut Contact,
        surface: &TriangleSurface,
        shape_a: &Shape,
        transform_a: WorldTransform,
        shape_b: &Shape,
        transform_b: WorldTransform,
        material_indices: Option<&[u16]>,
    ) {
        let material_b = shape_b.material(0);
        let mut tangent_a = Vec3::ZERO;
        if shape_a.material_count() > 0 {
            let mut friction: f32 = 0.0;
            let mut restitution: f32 = 0.0;
            let mut sample_count: f32 = 0.0;
            {
                let manifolds = contact.manifolds();
                for manifold in manifolds.buffer() {
                    for i in 0..manifold.point_count as usize {
                        let triangle_index = manifold.points[i].triangle_index as usize;
                        let local_index = surface.material(triangle_index as i32);
                        let mapped = match material_indices {
                            Some(indices) => indices[local_index as usize] as i32,
                            None => local_index,
                        };
                        let material_index =
                            clamp_int(mapped, 0, shape_a.material_count() - 1);
                        let material = shape_a.material(material_index);
                        friction += self.mix_friction(
                            material.friction,
                            material.user_material_id,
                            material_b.friction,
                            material_b.user_material_id,
                        );
                        restitution += self.mix_restitution(
                            material.restitution,
                            material.user_material_id,
                            material_b.restitution,
                            material_b.user_material_id,
                        );
                        tangent_a = tangent_a + material.tangent_velocity;
                        sample_count += 1.0;
                    }
                }
            }
            if sample_count > 0.0 {
                let inverse: f32 = 1.0 / sample_count;
                contact.friction = inverse * friction;
                contact.restitution = inverse * restitution;
                tangent_a = inverse * tangent_a;
            }
        } else {
            let material = shape_a.material(0);
            contact.friction = self.mix_friction(
                material.friction,
                material.user_material_id,
                material_b.friction,
                material_b.user_material_id,
            );
            contact.restitution = self.mix_restitution(
                material.restitution,
                material.user_material_id,
                material_b.restitution,
                material_b.user_material_id,
            );
            tangent_a = material.tangent_velocity;
        }
        tangent_a = rotate_vector(transform_a.q, tangent_a);
        let radius: f32 = match &shape_b.geometry {
            ShapeGeometry::Sphere(sphere) => sphere.radius,
            ShapeGeometry::Capsule(capsule) => capsule.radius,
            ShapeGeometry::Hull(hull) => hull.inner_radius(),
            _ => 0.0,
        };
        contact.rolling_resistance = material_b.rolling_resistance * radius;
        contact.tangent_velocity = tangent_a - rotate_vector(transform_b.q, material_b.tangent_velocity);
    }
}
