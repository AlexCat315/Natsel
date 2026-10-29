//! Buffered events and contact data.

use crate::math::types::{Pos, Vec3, WorldTransform};
use crate::types::collision_types::Manifold;
use crate::types::ids::{BodyID, ContactID, JointID, ShapeID};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct SensorBeginTouchEvent {
    pub sensor_shape_id: ShapeID,
    pub visitor_shape_id: ShapeID,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct SensorEndTouchEvent {
    pub sensor_shape_id: ShapeID,
    pub visitor_shape_id: ShapeID,
}

#[derive(Clone, Debug, Default)]
pub struct SensorEvents {
    pub begin_events: Vec<SensorBeginTouchEvent>,
    pub end_events: Vec<SensorEndTouchEvent>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct ContactBeginTouchEvent {
    pub shape_id_a: ShapeID,
    pub shape_id_b: ShapeID,
    pub contact_id: ContactID,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct ContactEndTouchEvent {
    pub shape_id_a: ShapeID,
    pub shape_id_b: ShapeID,
    pub contact_id: ContactID,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
#[repr(C)]
pub struct ContactHitEvent {
    pub shape_id_a: ShapeID,
    pub shape_id_b: ShapeID,
    pub contact_id: ContactID,
    pub point: Pos,
    pub normal: Vec3,
    pub approach_speed: f32,
    pub user_material_id_a: u64,
    pub user_material_id_b: u64,
}

#[derive(Clone, Debug, Default)]
pub struct ContactEvents {
    pub begin_events: Vec<ContactBeginTouchEvent>,
    pub end_events: Vec<ContactEndTouchEvent>,
    pub hit_events: Vec<ContactHitEvent>,
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct BodyMoveEvent {
    pub user_data: Option<*mut core::ffi::c_void>,
    pub transform: WorldTransform,
    pub body_id: BodyID,
    pub fell_asleep: bool,
}

#[derive(Clone, Debug, Default)]
pub struct BodyEvents {
    pub move_events: Vec<BodyMoveEvent>,
}

#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
pub struct JointEvent {
    pub joint_id: JointID,
    pub user_data: Option<*mut core::ffi::c_void>,
}

#[derive(Clone, Debug, Default)]
pub struct JointEvents {
    pub joint_events: Vec<JointEvent>,
}

/// The contact data for two shapes. By convention the manifold normal points from shape A to
/// shape B.
#[derive(Clone, Debug, Default)]
pub struct ContactData {
    pub contact_id: ContactID,
    pub shape_id_a: ShapeID,
    pub shape_id_b: ShapeID,
    pub manifolds: Vec<Manifold>,
}
