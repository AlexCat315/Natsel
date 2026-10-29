//! Procedural triangle meshes.

use crate::base::constants as b3;
use crate::math::functions::compute_cos_sin;
use crate::math::types::Vec3;

use super::build::MeshBuildOptions;
use super::storage::TriangleMesh;

const BOX_MESH_INDICES: [i32; 36] = [
    0, 1, 3, 1, 2, 3, 0, 4, 1, 1, 4, 5, 0, 3, 7, 4, 0, 7,
    4, 7, 5, 6, 5, 7, 1, 5, 2, 6, 2, 5, 3, 2, 7, 6, 7, 2,
];

fn mesh_box_vertices(center: Vec3, extent: Vec3) -> [Vec3; 8] {
    let (x, y, z) = (extent.x, extent.y, extent.z);
    [
        Vec3::new(x, y, z) + center,
        Vec3::new(-x, y, z) + center,
        Vec3::new(-x, -y, z) + center,
        Vec3::new(x, -y, z) + center,
        Vec3::new(x, y, -z) + center,
        Vec3::new(-x, y, -z) + center,
        Vec3::new(-x, -y, -z) + center,
        Vec3::new(x, -y, -z) + center,
    ]
}

/// Indices for a rectangular grid in the XZ plane.
pub fn mesh_grid_indices(x_count: i32, z_count: i32) -> Vec<i32> {
    let mut indices = Vec::with_capacity(6 * x_count as usize * z_count as usize);
    for ix in 0..x_count {
        for iz in 0..z_count {
            let i1 = iz + (z_count + 1) * ix;
            let i2 = i1 + 1;
            let i3 = i2 + (z_count + 1);
            let i4 = i3 - 1;
            indices.extend_from_slice(&[i1, i2, i3, i3, i4, i1]);
        }
    }
    indices
}

pub fn create_wave_mesh(
    x_count: i32,
    z_count: i32,
    cell_width: f32,
    amplitude: f32,
    row_frequency: f32,
    column_frequency: f32,
) -> Option<TriangleMesh> {
    if x_count <= 0 || z_count <= 0 || !(cell_width > 0.0) {
        return None;
    }
    let mut vertices = Vec::new();
    let x_width = cell_width * x_count as f32;
    let z_width = cell_width * z_count as f32;
    let omega_z = 2.0 * b3::PI * row_frequency * cell_width;
    let omega_x = 2.0 * b3::PI * column_frequency * cell_width;
    let mut x = -0.5 * x_width;
    for ix in 0..=x_count {
        let row_height = compute_cos_sin(omega_x * ix as f32).sine;
        let mut z = -0.5 * z_width;
        for iz in 0..=z_count {
            let column_height = compute_cos_sin(omega_z * iz as f32).sine;
            vertices.push(Vec3::new(x, amplitude * row_height * column_height, z));
            z += cell_width;
        }
        x += cell_width;
    }
    let options = MeshBuildOptions {
        use_median_split: true,
        identify_edges: true,
        ..Default::default()
    };
    TriangleMesh::new(&vertices, &mesh_grid_indices(x_count, z_count), None, options)
}

pub fn create_torus_mesh(
    radial_resolution: i32,
    tubular_resolution: i32,
    radius: f32,
    thickness: f32,
) -> Option<TriangleMesh> {
    if radial_resolution < 3 || tubular_resolution < 3 {
        return None;
    }
    let mut vertices = Vec::new();
    for radial in 0..radial_resolution {
        for tubular in 0..tubular_resolution {
            let u = tubular as f32 / tubular_resolution as f32 * b3::TWO_PI;
            let v = radial as f32 / radial_resolution as f32 * b3::TWO_PI;
            let cs_u = compute_cos_sin(u);
            let cs_v = compute_cos_sin(v);
            vertices.push(Vec3::new(
                (radius + thickness * cs_v.cosine) * cs_u.cosine,
                (radius + thickness * cs_v.cosine) * cs_u.sine,
                thickness * cs_v.sine,
            ));
        }
    }
    let mut indices = Vec::new();
    for radial1 in 0..radial_resolution {
        let radial2 = (radial1 + 1) % radial_resolution;
        for tubular1 in 0..tubular_resolution {
            let tubular2 = (tubular1 + 1) % tubular_resolution;
            let i1 = radial1 * tubular_resolution + tubular1;
            let i2 = radial1 * tubular_resolution + tubular2;
            let i3 = radial2 * tubular_resolution + tubular2;
            let i4 = radial2 * tubular_resolution + tubular1;
            indices.extend_from_slice(&[i1, i2, i3, i3, i4, i1]);
        }
    }
    let options = MeshBuildOptions {
        identify_edges: true,
        ..Default::default()
    };
    TriangleMesh::new(&vertices, &indices, None, options)
}

pub fn create_box_mesh(center: Vec3, extent: Vec3, identify_edges: bool) -> Option<TriangleMesh> {
    let options = MeshBuildOptions {
        identify_edges,
        ..Default::default()
    };
    TriangleMesh::new(&mesh_box_vertices(center, extent), &BOX_MESH_INDICES, None, options)
}

pub fn create_hollow_box_mesh(center: Vec3, extent: Vec3) -> Option<TriangleMesh> {
    let mut indices = BOX_MESH_INDICES;
    let mut i = 0;
    while i < indices.len() {
        indices.swap(i, i + 2);
        i += 3;
    }
    let options = MeshBuildOptions {
        identify_edges: true,
        ..Default::default()
    };
    TriangleMesh::new(&mesh_box_vertices(center, extent), &indices, None, options)
}

pub fn create_platform_mesh(center: Vec3, height: f32, top_width: f32, bottom_width: f32) -> Option<TriangleMesh> {
    let hb = 0.5 * bottom_width;
    let ht = 0.5 * top_width;
    let hy = 0.5 * height;
    let vertices = [
        Vec3::new(ht, hy, ht) + center,
        Vec3::new(-ht, hy, ht) + center,
        Vec3::new(-hb, -hy, hb) + center,
        Vec3::new(hb, -hy, hb) + center,
        Vec3::new(ht, hy, -ht) + center,
        Vec3::new(-ht, hy, -ht) + center,
        Vec3::new(-hb, -hy, -hb) + center,
        Vec3::new(hb, -hy, -hb) + center,
    ];
    let options = MeshBuildOptions {
        use_median_split: true,
        identify_edges: true,
        ..Default::default()
    };
    TriangleMesh::new(&vertices, &BOX_MESH_INDICES, None, options)
}

pub fn create_grid_mesh(
    x_count: i32,
    z_count: i32,
    cell_width: f32,
    material_count: i32,
    identify_edges: bool,
) -> Option<TriangleMesh> {
    if x_count <= 0 || z_count <= 0 || !cell_width.is_finite() || cell_width <= 0.0 {
        return None;
    }
    if !(0..=255).contains(&material_count) {
        return None;
    }
    let mut vertices = Vec::with_capacity((x_count + 1) as usize * (z_count + 1) as usize);
    let x_width = cell_width * x_count as f32;
    let z_width = cell_width * z_count as f32;
    let mut x = -0.5 * x_width;
    for _ in 0..=x_count {
        let mut z = -0.5 * z_width;
        for _ in 0..=z_count {
            vertices.push(Vec3::new(x, 0.0, z));
            z += cell_width;
        }
        x += cell_width;
    }
    let indices = mesh_grid_indices(x_count, z_count);
    let mut materials = Vec::with_capacity(2 * x_count as usize * z_count as usize);
    let mut material_index = 0;
    for _ in 0..x_count {
        for _ in 0..z_count {
            if material_count > 0 {
                let m = (material_index % material_count) as u8;
                materials.push(m);
                materials.push(m);
            }
            material_index += 1;
        }
    }
    let options = MeshBuildOptions {
        use_median_split: true,
        identify_edges,
        ..Default::default()
    };
    let material_slice = if material_count > 0 { Some(materials.as_slice()) } else { None };
    TriangleMesh::new(&vertices, &indices, material_slice, options)
}
