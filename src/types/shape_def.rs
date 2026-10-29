//! Public shape definitions and materials.

use crate::base::constants as b3;
use crate::math::types::Vec3;
use crate::types::world_def::{default_category_bits, default_mask_bits};

/// This is used to filter collision on shapes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct Filter {
    pub category_bits: u64,
    pub mask_bits: u64,
    pub group_index: i32,
}

impl Filter {
    /// Collide with everything.
    pub fn default_filter() -> Self {
        Self {
            category_bits: default_category_bits(),
            mask_bits: default_mask_bits(),
            group_index: 0,
        }
    }

    pub fn new(category_bits: u64, mask_bits: u64, group_index: i32) -> Self {
        Self {
            category_bits,
            mask_bits,
            group_index,
        }
    }
}

/// Material properties supported per triangle on meshes and height fields.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct SurfaceMaterial {
    /// The Coulomb (dry) friction coefficient, usually in the range [0,1].
    pub friction: f32,
    /// The coefficient of restitution (bounce) usually in the range [0,1].
    pub restitution: f32,
    /// The rolling resistance usually in the range [0,1].
    pub rolling_resistance: f32,
    /// The tangent velocity for conveyor belts, local to the shape.
    pub tangent_velocity: Vec3,
    /// User material identifier.
    pub user_material_id: u64,
    /// Custom debug draw color. Ignored if 0.
    pub custom_color: u32,
    /// Explicit padding. Must be zero.
    pub padding: u32,
}

impl SurfaceMaterial {
    /// The default surface material (friction 0.6).
    pub fn default_material() -> Self {
        Self {
            friction: 0.6,
            ..Default::default()
        }
    }
}

/// Shape type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum ShapeType {
    /// A capsule is an extruded sphere.
    #[default]
    CapsuleShape = 0,
    /// A baked compound shape composed of spheres, capsules, hulls, and meshes.
    CompoundShape = 1,
    /// A height field useful for terrain.
    HeightShape = 2,
    /// A convex hull.
    HullShape = 3,
    /// A triangle soup.
    MeshShape = 4,
    /// A sphere with an offset.
    SphereShape = 5,
}

impl ShapeType {
    pub const COUNT: i32 = 6;
}

/// Used to create a shape.
#[derive(Clone, Debug)]
pub struct ShapeDef {
    pub name: Option<String>,
    pub user_data: Option<*mut core::ffi::c_void>,
    /// Surface material used on mesh shapes per triangle.
    pub materials: Vec<SurfaceMaterial>,
    /// The base surface material.
    pub base_material: SurfaceMaterial,
    /// The density, usually in kg/m^3.
    pub density: f32,
    /// Explosion scale.
    pub explosion_scale: f32,
    /// Contact filtering data.
    pub filter: Filter,
    pub enable_custom_filtering: bool,
    pub is_sensor: bool,
    pub enable_sensor_events: bool,
    pub enable_contact_events: bool,
    pub enable_hit_events: bool,
    pub enable_pre_solve_events: bool,
    pub invoke_contact_creation: bool,
    pub update_body_mass: bool,
    pub enable_speculative_contact: bool,
}

impl Default for ShapeDef {
    fn default() -> Self {
        let units = b3::length_units_per_meter();
        Self {
            name: None,
            user_data: None,
            materials: Vec::new(),
            base_material: SurfaceMaterial::default_material(),
            density: 1000.0 / (units * units * units),
            explosion_scale: 1.0,
            filter: Filter::default_filter(),
            enable_custom_filtering: false,
            is_sensor: false,
            enable_sensor_events: false,
            enable_contact_events: false,
            enable_hit_events: false,
            enable_pre_solve_events: false,
            invoke_contact_creation: true,
            update_body_mass: true,
            enable_speculative_contact: true,
        }
    }
}

/// The query filter is used to filter collisions between queries and shapes.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct QueryFilter {
    pub category_bits: u64,
    pub mask_bits: u64,
    pub id: u64,
    pub name: Option<String>,
}

impl QueryFilter {
    /// Query everything.
    pub fn default_filter() -> Self {
        Self {
            category_bits: default_category_bits(),
            mask_bits: default_mask_bits(),
            id: 0,
            name: None,
        }
    }

    pub fn new(category_bits: u64, mask_bits: u64) -> Self {
        Self {
            category_bits,
            mask_bits,
            id: 0,
            name: None,
        }
    }
}
