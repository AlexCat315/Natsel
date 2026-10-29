//! Runtime boundary for the triangle operations shared by mesh and terrain contacts.

use crate::base::core::internal_assert;
use crate::collision::height_field::regions::query_height_field;
use crate::collision::height_field::HeightField;
use crate::collision::internal_math::Triangle;
use crate::collision::mesh::access::mesh_triangle;
use crate::collision::mesh::overlap::query_mesh;
use crate::math::types::{AABB, Vec3};
use crate::simulation::shape_runtime::ShapeGeometry;
use crate::types::geometry::Mesh;

/// The two triangle-bearing geometries the contact paths share.
#[derive(Clone, Debug)]
pub enum TriangleSurface {
    Mesh(Mesh),
    HeightField(HeightField),
}

impl TriangleSurface {
    /// `init?(_ geometry: ShapeGeometry)` - only triangle geometries make a surface.
    pub fn from_geometry(geometry: &ShapeGeometry) -> Option<Self> {
        match geometry {
            ShapeGeometry::Mesh(mesh) => Some(TriangleSurface::Mesh(mesh.clone())),
            ShapeGeometry::HeightField(field) => Some(TriangleSurface::HeightField(field.clone())),
            _ => None,
        }
    }

    /// Visit the triangles overlapping `bounds`. Returning false from the callback stops the query.
    pub fn query<F>(&self, bounds: AABB, callback: F)
    where
        F: FnMut(Vec3, Vec3, Vec3, i32) -> bool,
    {
        match self {
            TriangleSurface::Mesh(mesh) => query_mesh(mesh, bounds, callback),
            TriangleSurface::HeightField(field) => query_height_field(field, bounds, callback),
        }
    }

    /// The triangle at `index`, scaled and winding-corrected.
    pub fn triangle(&self, index: i32) -> Triangle {
        match self {
            TriangleSurface::Mesh(mesh) => mesh_triangle(mesh, index),
            TriangleSurface::HeightField(field) => field.triangle(index),
        }
    }

    /// The material index of the triangle at `index`.
    pub fn material(&self, index: i32) -> i32 {
        match self {
            TriangleSurface::Mesh(mesh) => match mesh.data.as_ref() {
                Some(data) => data.material_indices()[index as usize] as i32,
                None => internal_assert("mesh has no shared data", file!(), line!()),
            },
            TriangleSurface::HeightField(field) => field.material(index),
        }
    }
}
