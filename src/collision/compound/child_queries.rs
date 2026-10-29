//! Public queries against compound children.

use crate::collision::capsule::{overlap_capsule, ray_cast_capsule, shape_cast_capsule};
use crate::collision::hull_queries::{overlap_hull, ray_cast_hull, shape_cast_hull};
use crate::collision::mesh::overlap::{overlap_mesh, ray_cast_mesh, shape_cast_mesh};
use crate::collision::proxy::{compute_proxy_aabb, make_local_proxy};
use crate::collision::sphere::{overlap_sphere, ray_cast_sphere, shape_cast_sphere};
use crate::math::functions::*;
use crate::math::types::AABB;
use crate::types::queries::{
    BoxCastInput, CastOutput, RayCastInput, ShapeCastInput, ShapeProxy,
};

use super::compound::{Compound, CompoundChildGeometry};

/// Query children whose bounds overlap. Return `false` from the callback to stop.
pub fn query_compound(compound: &Compound, bounds: AABB, mut callback: impl FnMut(usize) -> bool) {
    compound.with_tree_view(|view| {
        view.query(bounds, u64::MAX, false, |_, data| {
            callback(data as u32 as usize)
        });
    });
}

/// Do a compound and a proxy overlap?
pub fn overlap_compound(compound: &Compound, transform: crate::math::types::Transform, proxy: ShapeProxy) -> bool {
    let mut points = Vec::new();
    let local = make_local_proxy(proxy, transform, &mut points);
    let mut touching = false;
    let bounds = compute_proxy_aabb(local);

    compound.with_tree_view(|view| {
        view.query(bounds, u64::MAX, false, |_, data| {
            let child = compound.child(data as u32 as usize);
            touching = match &child.geometry {
                CompoundChildGeometry::Capsule(capsule) => {
                    overlap_capsule(capsule, child.transform, &local)
                }
                CompoundChildGeometry::Hull(hull) => overlap_hull(hull, child.transform, &local),
                CompoundChildGeometry::Mesh(mesh) => overlap_mesh(mesh, child.transform, local),
                CompoundChildGeometry::Sphere(sphere) => {
                    overlap_sphere(sphere, child.transform, &local)
                }
            };
            !touching
        });
    });

    touching
}

/// Ray cast a compound.
pub fn ray_cast_compound(compound: &Compound, input: RayCastInput) -> CastOutput {
    let mut result = CastOutput::default();

    compound.with_tree_view(|view| {
        view.ray_cast(input, u64::MAX, false, |input, _, data| {
            let child = compound.child(data as u32 as usize);
            let mut local = *input;
            local.origin = inv_transform_point(child.transform, input.origin);
            local.translation = inv_rotate_vector(child.transform.q, input.translation);

            let mut output = match &child.geometry {
                CompoundChildGeometry::Capsule(capsule) => ray_cast_capsule(capsule, &local),
                CompoundChildGeometry::Hull(hull) => ray_cast_hull(hull, &local),
                CompoundChildGeometry::Mesh(mesh) => ray_cast_mesh(mesh, local),
                CompoundChildGeometry::Sphere(sphere) => ray_cast_sphere(sphere, &local),
            };

            match &child.geometry {
                CompoundChildGeometry::Mesh(_) => {
                    output.material_index =
                        child.material_indices[output.material_index as usize] as i32;
                }
                _ => output.material_index = child.material_indices[0] as i32,
            }

            if output.hit {
                output.point = transform_point(child.transform, output.point);
                output.normal = rotate_vector(child.transform.q, output.normal);
                output.child_index = data as u32 as i32;
                result = output;
                return output.fraction;
            }

            input.max_fraction
        });
    });

    result
}

/// Shape cast a compound.
pub fn shape_cast_compound(compound: &Compound, input: &ShapeCastInput) -> CastOutput {
    let mut result = CastOutput::default();
    if input.proxy.count == 0 {
        return result;
    }

    let tree_input = BoxCastInput {
        box_: make_aabb(input.proxy.points(), input.proxy.radius),
        translation: input.translation,
        max_fraction: input.max_fraction,
    };

    compound.with_tree_view(|view| {
        view.box_cast(tree_input, u64::MAX, false, |advancing, _, data| {
            let child = compound.child(data as u32 as usize);
            let mut points = Vec::new();
            let local = ShapeCastInput {
                proxy: make_local_proxy(input.proxy, child.transform, &mut points),
                translation: inv_rotate_vector(child.transform.q, input.translation),
                max_fraction: advancing.max_fraction,
                can_encroach: input.can_encroach,
            };

            let mut output = match &child.geometry {
                CompoundChildGeometry::Capsule(capsule) => shape_cast_capsule(capsule, &local),
                CompoundChildGeometry::Hull(hull) => shape_cast_hull(hull, &local),
                CompoundChildGeometry::Mesh(mesh) => shape_cast_mesh(mesh, local),
                CompoundChildGeometry::Sphere(sphere) => shape_cast_sphere(sphere, &local),
            };

            match &child.geometry {
                CompoundChildGeometry::Mesh(_) => {
                    output.material_index =
                        child.material_indices[output.material_index as usize] as i32;
                }
                _ => output.material_index = child.material_indices[0] as i32,
            }

            if output.hit {
                output.point = transform_point(child.transform, output.point);
                output.normal = rotate_vector(child.transform.q, output.normal);
                output.child_index = data as u32 as i32;
                result = output;
                return output.fraction;
            }

            advancing.max_fraction
        });
    });

    result
}
