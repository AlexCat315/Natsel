//! Body definitions and simulation type enumeration.

use crate::base::constants as b3;
use crate::math::types::{Pos, Quat, Vec3};

/// The body simulation type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum BodyType {
    /// zero mass, zero velocity, may be manually moved
    #[default]
    StaticBody = 0,
    /// zero mass, velocity set by user, moved by solver
    KinematicBody = 1,
    /// positive mass, velocity determined by forces, moved by solver
    DynamicBody = 2,
}

impl BodyType {
    pub const COUNT: i32 = 3;
}

/// Motion locks to restrict the body movement.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct MotionLocks {
    pub linear_x: bool,
    pub linear_y: bool,
    pub linear_z: bool,
    pub angular_x: bool,
    pub angular_y: bool,
    pub angular_z: bool,
}

impl MotionLocks {
    pub fn new(
        linear_x: bool,
        linear_y: bool,
        linear_z: bool,
        angular_x: bool,
        angular_y: bool,
        angular_z: bool,
    ) -> Self {
        Self {
            linear_x,
            linear_y,
            linear_z,
            angular_x,
            angular_y,
            angular_z,
        }
    }
}

/// A body definition holds all the data needed to construct a rigid body. Shapes are added to a
/// body after construction.
#[derive(Clone, Debug)]
pub struct BodyDef {
    pub type_: BodyType,
    pub position: Pos,
    pub rotation: Quat,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub linear_damping: f32,
    pub angular_damping: f32,
    pub gravity_scale: f32,
    pub sleep_threshold: f32,
    pub safety_factor: f32,
    pub name: Option<String>,
    pub user_data: Option<*mut core::ffi::c_void>,
    pub motion_locks: MotionLocks,
    pub enable_sleep: bool,
    pub is_awake: bool,
    pub is_bullet: bool,
    pub is_enabled: bool,
    pub allow_fast_rotation: bool,
    pub enable_contact_recycling: bool,
}

impl Default for BodyDef {
    fn default() -> Self {
        Self {
            type_: BodyType::StaticBody,
            position: Pos::ZERO,
            rotation: Quat::IDENTITY,
            linear_velocity: Vec3::ZERO,
            angular_velocity: Vec3::ZERO,
            linear_damping: 0.0,
            angular_damping: 0.0,
            gravity_scale: 1.0,
            sleep_threshold: 0.05 * b3::length_units_per_meter(),
            safety_factor: 0.5,
            name: None,
            user_data: None,
            motion_locks: MotionLocks::default(),
            enable_sleep: true,
            is_awake: true,
            is_bullet: false,
            is_enabled: true,
            allow_fast_rotation: false,
            enable_contact_recycling: true,
        }
    }
}
