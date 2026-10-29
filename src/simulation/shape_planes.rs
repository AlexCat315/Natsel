//! Collision planes between a character mover and a shape.

use crate::collision::capsule::collide_mover_and_capsule;
use crate::collision::compound::child_mover::collide_mover_and_compound;
use crate::collision::height_field::planes::collide_mover_and_height_field;
use crate::collision::hull_mover::collide_mover_and_hull;
use crate::collision::mesh::overlap::collide_mover_and_mesh;
use crate::collision::sphere::collide_mover_and_sphere;
use crate::math::functions::{clamp_int, inv_transform_point, rotate_vector, transform_point};
use crate::math::types::Transform;
use crate::simulation::shape_runtime::{Shape, ShapeGeometry};
use crate::types::geometry::Capsule;
use crate::types::queries::PlaneResult;

/// Collide a mover capsule with a shape in `transform`, returning up to `capacity` planes in
/// world space.
pub fn collide_mover(
    shape: &Shape,
    transform: Transform,
    mover: &Capsule,
    capacity: usize,
) -> Vec<PlaneResult> {
    if capacity == 0 {
        return Vec::new();
    }

    let local = Capsule::new(
        inv_transform_point(transform, mover.center1),
        inv_transform_point(transform, mover.center2),
        mover.radius,
    );

    let mut plane = PlaneResult::default();
    let mut planes: Vec<PlaneResult>;
    match &shape.geometry {
        ShapeGeometry::Capsule(capsule) => {
            planes = if collide_mover_and_capsule(&mut plane, capsule, &local) == 1 {
                vec![plane]
            } else {
                Vec::new()
            }
        }
        ShapeGeometry::Sphere(sphere) => {
            planes = if collide_mover_and_sphere(&mut plane, sphere, &local) == 1 {
                vec![plane]
            } else {
                Vec::new()
            }
        }
        ShapeGeometry::Hull(hull) => {
            planes = if collide_mover_and_hull(&mut plane, hull, &local) == 1 {
                vec![plane]
            } else {
                Vec::new()
            }
        }
        ShapeGeometry::Compound(compound) => {
            planes = collide_mover_and_compound(compound, &local, capacity)
        }
        ShapeGeometry::Mesh(mesh) => planes = collide_mover_and_mesh(mesh, local, capacity),
        ShapeGeometry::HeightField(field) => {
            planes = collide_mover_and_height_field(field, &local, capacity)
        }
    }

    for i in 0..planes.len() {
        planes[i].plane.normal = rotate_vector(transform.q, planes[i].plane.normal);
        planes[i].point = transform_point(transform, planes[i].point);
        planes[i].material_index = clamp_int(planes[i].material_index, 0, shape.material_count() - 1);
    }
    planes
}
