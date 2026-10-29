//! Triangle mesh geometry.

pub mod access;
pub mod bounds;
pub mod build;
pub mod edges;
pub mod grid;
pub mod layout;
pub mod mover;
pub mod overlap;
pub mod partition;
pub mod procedural;
pub mod ray_cast;
pub mod shape_cast;
pub mod storage;
pub mod tree_builder;
pub mod welding;

pub use storage::{compute_mesh_aabb, TriangleMesh};
