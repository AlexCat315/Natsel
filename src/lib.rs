//! Natsel is a deterministic 3D physics engine.
//!
//! World positions use double precision; local geometry, velocities, and constraint solving
//! use single precision.

pub mod base;
pub mod character;
pub mod collision;
pub mod containers;
pub mod math;
pub mod simulation;
pub mod skeleton;
pub mod soft_body;
pub mod types;
pub mod vehicle;

pub use base::core::{get_max_manifold_points, get_version, is_double_precision, Version};
