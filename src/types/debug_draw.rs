//! Debug draw geometry, colors, and callbacks.

use std::sync::Arc;

use core::ffi::c_void;

use crate::collision::compound::compound::Compound;
use crate::collision::height_field::quantization::HeightField;
use crate::math::types::{AABB, Pos, Vec3, WorldTransform};
use crate::types::geometry::{Capsule, HullData, Sphere};
use crate::types::geometry::Mesh;
use crate::types::ids::ShapeID;
use crate::types::shape_def::ShapeType;

/// The geometry carried by a debug shape.
#[derive(Clone, Debug)]
pub enum DebugShapeGeometry {
    Capsule(*const Capsule),
    Compound(Compound),
    HeightField(HeightField),
    Hull(*const HullData),
    Mesh(*const Mesh),
    Sphere(*const Sphere),
}

/// This is sent to the user for debug shape creation.
#[derive(Clone, Debug)]
pub struct DebugShape {
    pub shape_id: ShapeID,
    pub type_: ShapeType,
    pub geometry: DebugShapeGeometry,
}

impl DebugShape {
    pub fn new(shape_id: ShapeID, type_: ShapeType, geometry: DebugShapeGeometry) -> Self {
        Self {
            shape_id,
            type_,
            geometry,
        }
    }
}

/// A packed debug draw color. The low 24 bits are RGB; the high byte optionally carries a
/// [`DebugMaterial`] preset.
pub type HexColor = u32;

/// PBR presets packed into the high byte of a [`HexColor`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(u32)]
pub enum DebugMaterial {
    #[default]
    Default = 0,
    Matte = 1,
    Soft = 2,
    Dead = 3,
    Glossy = 4,
    Metallic = 5,
}

/// Pack an RGB color with a material preset.
#[inline]
pub fn make_debug_color(rgb: HexColor, material: DebugMaterial) -> HexColor {
    (rgb & 0x00FF_FFFF) | ((material as u32) << 24)
}

/// The CSS color names the debug draw path uses.
pub mod debug_colors {
    use super::HexColor;

    pub const DARK_BLUE: HexColor = 0x00008B;
    pub const BLUE: HexColor = 0x0000FF;
    pub const LIME_GREEN: HexColor = 0x32CD32;
    pub const LIGHT_BLUE: HexColor = 0xADD8E6;
    pub const MAGENTA: HexColor = 0xFF00FF;
    pub const YELLOW: HexColor = 0xFFFF00;
    pub const RED: HexColor = 0xFF0000;
    pub const ORANGE: HexColor = 0xFFA500;
    pub const GREEN: HexColor = 0x008000;
    pub const CYAN: HexColor = 0x00FFFF;
    pub const VIOLET: HexColor = 0xEE82EE;
    pub const PINK: HexColor = 0xFFC0CB;
    pub const CHOCOLATE: HexColor = 0xD2691E;
    pub const GOLDEN_ROD: HexColor = 0xDAA520;
    pub const CORAL: HexColor = 0xFF7F50;
    pub const ROSY_BROWN: HexColor = 0xBC8F8F;
    pub const AQUA: HexColor = 0x00FFFF;
    pub const PERU: HexColor = 0xCD853F;
    pub const LIME: HexColor = 0x00FF00;
    pub const GOLD: HexColor = 0xFFD700;
    pub const PLUM: HexColor = 0xDDA0DD;
    pub const SNOW: HexColor = 0xFFFAFA;
    pub const TEAL: HexColor = 0x008080;
    pub const KHAKI: HexColor = 0xF0E68C;
    pub const SALMON: HexColor = 0xFA8072;
    pub const PEACH_PUFF: HexColor = 0xFFDAB9;
    pub const HONEY_DEW: HexColor = 0xF0FFF0;
    pub const BLACK: HexColor = 0x000000;
    pub const LIGHT_GRAY: HexColor = 0xD3D3D3;
    pub const WHITE: HexColor = 0xFFFFFF;
    pub const SKY_BLUE: HexColor = 0x87CEEB;
    pub const SLATE_GRAY: HexColor = 0x708090;
    pub const WHEAT: HexColor = 0xF5DEB3;
    pub const TURQUOISE: HexColor = 0x40E0D0;
    pub const DARK_GRAY: HexColor = 0xA9A9A9;
    pub const STEEL_BLUE: HexColor = 0x4682B4;
    pub const LIGHT_STEEL_BLUE: HexColor = 0xB0C4DE;
    pub const TAN: HexColor = 0xD2B48C;
    pub const LIGHT_SLATE_GRAY: HexColor = 0x778899;
    pub const ORANGE_RED: HexColor = 0xFF4500;
    pub const LIGHT_GREEN: HexColor = 0x90EE90;
    pub const GRAY: HexColor = 0x808080;
    pub const DIM_GRAY: HexColor = 0x696969;
    pub const DARK_ORANGE: HexColor = 0xFF8C00;
    pub const DARK_CYAN: HexColor = 0x008B8B;

    /// The per-graph-color palette. The last slot is the overflow color.
    pub const GRAPH: [HexColor; 24] = [
        RED, ORANGE, YELLOW, GREEN, CYAN, BLUE, VIOLET, PINK, CHOCOLATE, GOLDEN_ROD, CORAL,
        ROSY_BROWN, AQUA, PERU, LIME, GOLD, PLUM, SNOW, TEAL, KHAKI, SALMON, PEACH_PUFF,
        HONEY_DEW, BLACK,
    ];
}

/// The callbacks and options a renderer supplies to world drawing. All coordinates are world
/// space.
#[derive(Clone, Default)]
pub struct DebugDraw {
    /// Draws a user shape. `user_shape` is the opaque pointer your factory returned.
    pub draw_shape: Option<Arc<dyn Fn(*mut c_void, WorldTransform, HexColor)>>,
    /// Draw a line segment.
    pub draw_segment: Option<Arc<dyn Fn(Pos, Pos, HexColor)>>,
    /// Draw a transform.
    pub draw_transform: Option<Arc<dyn Fn(WorldTransform)>>,
    /// Draw a point.
    pub draw_point: Option<Arc<dyn Fn(Pos, f32, HexColor)>>,
    /// Draw a sphere.
    pub draw_sphere: Option<Arc<dyn Fn(Pos, f32, HexColor, f32)>>,
    /// Draw a capsule.
    pub draw_capsule: Option<Arc<dyn Fn(Pos, Pos, f32, HexColor, f32)>>,
    /// Draw an axis aligned box.
    pub draw_bounds: Option<Arc<dyn Fn(AABB, HexColor)>>,
    /// Draw an oriented box.
    pub draw_box: Option<Arc<dyn Fn(Vec3, WorldTransform, HexColor)>>,
    /// Draw a string in world space.
    pub draw_string: Option<Arc<dyn Fn(Pos, &str, HexColor)>>,

    /// World bounds used for culling.
    pub drawing_bounds: AABB,
    /// Scale used when drawing forces.
    pub force_scale: f32,
    /// Global scaling for joint drawing.
    pub joint_scale: f32,

    pub draw_shapes: bool,
    pub draw_joints: bool,
    pub draw_joint_extras: bool,
    pub draw_bounds_option: bool,
    pub draw_mass: bool,
    pub draw_sleep: bool,
    pub draw_body_names: bool,
    pub draw_contacts: bool,
    pub draw_anchor_a: bool,
    pub draw_graph_colors: bool,
    pub draw_contact_features: bool,
    pub draw_contact_normals: bool,
    pub draw_contact_forces: bool,
    pub draw_islands: bool,
    pub draw_vehicles: bool,
    pub draw_soft_bodies: bool,

    /// Factory invoked the first time a shape needs a renderable representation.
    pub create_debug_shape: Option<Arc<dyn Fn(&DebugShape) -> Option<*mut c_void>>>,
}

impl std::fmt::Debug for DebugDraw {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("DebugDraw")
            .field("drawing_bounds", &self.drawing_bounds)
            .field("force_scale", &self.force_scale)
            .field("joint_scale", &self.joint_scale)
            .finish_non_exhaustive()
    }
}

impl DebugDraw {
    pub fn new() -> Self {
        Self {
            drawing_bounds: AABB::new(Vec3::new(-1e6, -1e6, -1e6), Vec3::new(1e6, 1e6, 1e6)),
            force_scale: 1.0,
            joint_scale: 1.0,
            draw_shapes: true,
            draw_joints: true,
            draw_vehicles: true,
            draw_soft_bodies: true,
            ..Default::default()
        }
    }
}
