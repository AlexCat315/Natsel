//! Programmatic height-field factories used by authoring, demos and tests.

use crate::base::constants as b3;
use crate::collision::height_field::HeightField;
use crate::math::functions::compute_cos_sin;
use crate::math::types::Vec3;
use crate::types::geometry::HeightFieldDef;

/// The flat grid terrain; `make_holes` punches every sixteenth cell out of the surface.
///
/// Swift defaulted `scale` to `Vec3.one` and `makeHoles` to `false`; Rust has no default
/// arguments, so callers pass both explicitly (`Vec3::ONE`, `false`).
pub fn create_grid_height_field(
    row_count: usize,
    column_count: usize,
    scale: Vec3,
    make_holes: bool,
) -> Option<HeightField> {
    if row_count >= 2 && column_count >= 2 {
        let mut definition = height_field_grid_definition(row_count, column_count, scale, make_holes);
        definition.heights = vec![0.0f32; row_count * column_count];
        HeightField::new(&definition)
    } else {
        None
    }
}

/// The sine-product wave terrain sampled on the same grid as `create_grid_height_field`.
///
/// Swift defaulted `scale` to `Vec3.one` and `makeHoles` to `false`; Rust has no default
/// arguments, so callers pass both explicitly (`Vec3::ONE`, `false`).
pub fn create_wave_height_field(
    row_count: usize,
    column_count: usize,
    scale: Vec3,
    row_frequency: f32,
    column_frequency: f32,
    make_holes: bool,
) -> Option<HeightField> {
    if row_count >= 2 && column_count >= 2 {
        let mut definition = height_field_grid_definition(row_count, column_count, scale, make_holes);
        let omega_z = 2.0 * b3::PI * row_frequency;
        let omega_x = 2.0 * b3::PI * column_frequency;
        for row in 0..row_count {
            let row_height = compute_cos_sin(omega_z * row as f32).sine;
            for column in 0..column_count {
                let column_height = compute_cos_sin(omega_x * column as f32).sine;
                definition.heights.push(row_height * column_height);
            }
        }
        HeightField::new(&definition)
    } else {
        None
    }
}

/// The shared grid, bounds and material layout of both factories.
fn height_field_grid_definition(
    rows: usize,
    columns: usize,
    scale: Vec3,
    holes: bool,
) -> HeightFieldDef {
    let mut definition = HeightFieldDef::default();
    definition.count_x = columns as i32;
    definition.count_z = rows as i32;
    definition.scale = scale;
    definition.global_minimum_height = -256.0;
    definition.global_maximum_height = 256.0;
    for cell in 0..((rows - 1) * (columns - 1)) {
        definition
            .material_indices
            .push(if holes && cell > 0 && cell % 16 == 0 {
                0xff
            } else {
                0
            });
    }
    definition
}
