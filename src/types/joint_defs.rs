//! Joint definitions.

use crate::base::constants as b3;
use crate::math::types::{Pos, Quat, Transform, Vec3};
use crate::types::ids::{BodyID, JointID};

/// Joint type enumeration.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(i32)]
pub enum JointType {
    #[default]
    ParallelJoint = 0,
    DistanceJoint,
    FilterJoint,
    MotorJoint,
    PrismaticJoint,
    RevoluteJoint,
    SphericalJoint,
    WeldJoint,
    WheelJoint,
    GearJoint,
    RackAndPinionJoint,
    PulleyJoint,
    SixDOFJoint,
    PathJoint,
}

/// Base joint definition used by all joint types.
#[derive(Clone, Debug)]
pub struct JointDef {
    pub user_data: Option<*mut core::ffi::c_void>,
    pub body_id_a: BodyID,
    pub body_id_b: BodyID,
    pub local_frame_a: Transform,
    pub local_frame_b: Transform,
    pub force_threshold: f32,
    pub torque_threshold: f32,
    pub constraint_hertz: f32,
    pub constraint_damping_ratio: f32,
    pub draw_scale: f32,
    pub collide_connected: bool,
}

impl Default for JointDef {
    fn default() -> Self {
        Self {
            user_data: None,
            body_id_a: BodyID::NULL,
            body_id_b: BodyID::NULL,
            local_frame_a: Transform::IDENTITY,
            local_frame_b: Transform::IDENTITY,
            force_threshold: f32::MAX,
            torque_threshold: f32::MAX,
            constraint_hertz: 60.0,
            constraint_damping_ratio: 2.0,
            draw_scale: b3::length_units_per_meter(),
            collide_connected: false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct DistanceJointDef {
    pub base: JointDef,
    pub length: f32,
    pub enable_spring: bool,
    pub lower_spring_force: f32,
    pub upper_spring_force: f32,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub enable_limit: bool,
    pub min_length: f32,
    pub max_length: f32,
    pub enable_motor: bool,
    pub max_motor_force: f32,
    pub motor_speed: f32,
}

impl Default for DistanceJointDef {
    fn default() -> Self {
        Self {
            base: JointDef::default(),
            length: 1.0,
            enable_spring: false,
            lower_spring_force: -f32::MAX,
            upper_spring_force: f32::MAX,
            hertz: 0.0,
            damping_ratio: 0.0,
            enable_limit: false,
            min_length: 0.0,
            max_length: b3::huge(),
            enable_motor: false,
            max_motor_force: 0.0,
            motor_speed: 0.0,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct MotorJointDef {
    pub base: JointDef,
    pub linear_velocity: Vec3,
    pub max_velocity_force: f32,
    pub angular_velocity: Vec3,
    pub max_velocity_torque: f32,
    pub linear_hertz: f32,
    pub linear_damping_ratio: f32,
    pub max_spring_force: f32,
    pub angular_hertz: f32,
    pub angular_damping_ratio: f32,
    pub max_spring_torque: f32,
}

#[derive(Clone, Debug, Default)]
pub struct FilterJointDef {
    pub base: JointDef,
}

#[derive(Clone, Debug)]
pub struct ParallelJointDef {
    pub base: JointDef,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub max_torque: f32,
}

impl Default for ParallelJointDef {
    fn default() -> Self {
        Self {
            base: JointDef::default(),
            hertz: 1.0,
            damping_ratio: 1.0,
            max_torque: f32::MAX,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct PrismaticJointDef {
    pub base: JointDef,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub target_translation: f32,
    pub enable_limit: bool,
    pub lower_translation: f32,
    pub upper_translation: f32,
    pub enable_motor: bool,
    pub max_motor_force: f32,
    pub motor_speed: f32,
}

#[derive(Clone, Debug, Default)]
pub struct RevoluteJointDef {
    pub base: JointDef,
    pub target_angle: f32,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub enable_limit: bool,
    pub lower_angle: f32,
    pub upper_angle: f32,
    pub enable_motor: bool,
    pub max_motor_torque: f32,
    pub motor_speed: f32,
}

#[derive(Clone, Debug)]
pub struct SphericalJointDef {
    pub base: JointDef,
    pub enable_spring: bool,
    pub hertz: f32,
    pub damping_ratio: f32,
    pub target_rotation: Quat,
    pub enable_cone_limit: bool,
    pub cone_angle: f32,
    pub enable_twist_limit: bool,
    pub lower_twist_angle: f32,
    pub upper_twist_angle: f32,
    pub enable_motor: bool,
    pub max_motor_torque: f32,
    pub motor_velocity: Vec3,
}

impl Default for SphericalJointDef {
    fn default() -> Self {
        Self {
            base: JointDef::default(),
            enable_spring: false,
            hertz: 0.0,
            damping_ratio: 0.0,
            target_rotation: Quat::IDENTITY,
            enable_cone_limit: false,
            cone_angle: 0.0,
            enable_twist_limit: false,
            lower_twist_angle: 0.0,
            upper_twist_angle: 0.0,
            enable_motor: false,
            max_motor_torque: 0.0,
            motor_velocity: Vec3::ZERO,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct WeldJointDef {
    pub base: JointDef,
    pub linear_hertz: f32,
    pub angular_hertz: f32,
    pub linear_damping_ratio: f32,
    pub angular_damping_ratio: f32,
}

#[derive(Clone, Debug)]
pub struct WheelJointDef {
    pub base: JointDef,
    pub enable_suspension_spring: bool,
    pub suspension_hertz: f32,
    pub suspension_damping_ratio: f32,
    pub enable_suspension_limit: bool,
    pub lower_suspension_limit: f32,
    pub upper_suspension_limit: f32,
    pub enable_spin_motor: bool,
    pub max_spin_torque: f32,
    pub spin_speed: f32,
    pub enable_steering: bool,
    pub steering_hertz: f32,
    pub steering_damping_ratio: f32,
    pub target_steering_angle: f32,
    pub max_steering_torque: f32,
    pub enable_steering_limit: bool,
    pub lower_steering_limit: f32,
    pub upper_steering_limit: f32,
}

impl Default for WheelJointDef {
    fn default() -> Self {
        Self {
            base: JointDef::default(),
            enable_suspension_spring: true,
            suspension_hertz: 1.0,
            suspension_damping_ratio: 0.7,
            enable_suspension_limit: false,
            lower_suspension_limit: 0.0,
            upper_suspension_limit: 0.0,
            enable_spin_motor: false,
            max_spin_torque: 0.0,
            spin_speed: 0.0,
            enable_steering: false,
            steering_hertz: 1.0,
            steering_damping_ratio: 0.7,
            target_steering_angle: 0.0,
            max_steering_torque: 0.0,
            enable_steering_limit: false,
            lower_steering_limit: 0.0,
            upper_steering_limit: 0.0,
        }
    }
}

#[derive(Clone, Debug)]
pub struct GearJointDef {
    pub base: JointDef,
    pub ratio: f32,
    pub reference_joint_a: JointID,
    pub reference_joint_b: JointID,
}

impl Default for GearJointDef {
    fn default() -> Self {
        Self {
            base: JointDef::default(),
            ratio: 1.0,
            reference_joint_a: JointID::NULL,
            reference_joint_b: JointID::NULL,
        }
    }
}

#[derive(Clone, Debug)]
pub struct RackAndPinionJointDef {
    pub base: JointDef,
    pub ratio: f32,
    pub reference_pinion_joint: JointID,
    pub reference_rack_joint: JointID,
}

impl Default for RackAndPinionJointDef {
    fn default() -> Self {
        Self {
            base: JointDef::default(),
            ratio: 1.0,
            reference_pinion_joint: JointID::NULL,
            reference_rack_joint: JointID::NULL,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PulleyJointDef {
    pub base: JointDef,
    pub ground_anchor_a: Pos,
    pub ground_anchor_b: Pos,
    pub ratio: f32,
    pub min_length: f32,
    pub max_length: f32,
}

impl Default for PulleyJointDef {
    fn default() -> Self {
        Self {
            base: JointDef::default(),
            ground_anchor_a: Pos::ZERO,
            ground_anchor_b: Pos::ZERO,
            ratio: 1.0,
            min_length: 0.0,
            max_length: 0.0,
        }
    }
}
