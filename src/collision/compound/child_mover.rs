//! Mover collisions against compound children.

use crate::collision::capsule::collide_mover_and_capsule;
use crate::collision::hull_mover::collide_mover_and_hull;
use crate::collision::mesh::overlap::collide_mover_and_mesh;
use crate::collision::sphere::collide_mover_and_sphere;
use crate::math::functions::{inv_transform_point, max_vec3, min_vec3, rotate_vector, transform_point};
use crate::math::types::{AABB, Vec3};
use crate::types::geometry::Capsule;
use crate::types::queries::PlaneResult;

use super::child_queries::query_compound;
use super::compound::{Compound, CompoundChildGeometry};

/// Collide a mover capsule with a compound.
pub fn collide_mover_and_compound(compound: &Compound, mover: &Capsule, capacity: usize) -> Vec<PlaneResult> {
    let mut results: Vec<PlaneResult> = Vec::new();
    if capacity == 0 {
        return results;
    }

    let radius = Vec3::new(mover.radius, mover.radius, mover.radius);
    let bounds = AABB::new(
        min_vec3(mover.center1, mover.center2) - radius,
        max_vec3(mover.center1, mover.center2) + radius,
    );

    query_compound(compound, bounds, |index| {
        let child = compound.child(index);
        let local = Capsule::new(
            inv_transform_point(child.transform, mover.center1),
            inv_transform_point(child.transform, mover.center2),
            mover.radius,
        );

        let mut planes = match &child.geometry {
            CompoundChildGeometry::Capsule(capsule) => {
                let mut plane = PlaneResult::default();
                if collide_mover_and_capsule(&mut plane, capsule, &local) == 1 {
                    vec![plane]
                } else {
                    Vec::new()
                }
            }
            CompoundChildGeometry::Hull(hull) => {
                let mut plane = PlaneResult::default();
                if collide_mover_and_hull(&mut plane, hull, &local) == 1 {
                    vec![plane]
                } else {
                    Vec::new()
                }
            }
            CompoundChildGeometry::Mesh(mesh) => {
                collide_mover_and_mesh(mesh, local, capacity - results.len())
            }
            CompoundChildGeometry::Sphere(sphere) => {
                let mut plane = PlaneResult::default();
                if collide_mover_and_sphere(&mut plane, sphere, &local) == 1 {
                    vec![plane]
                } else {
                    Vec::new()
                }
            }
        };

        for result in &mut planes {
            result.plane.normal = rotate_vector(child.transform.q, result.plane.normal);
            result.point = transform_point(child.transform, result.point);
            result.child_index = index as i32;
            result.material_index = child.material_indices[result.material_index as usize] as i32;
        }
        results.append(&mut planes);

        results.len() < capacity
    });

    results
}
