//! Ported tests from Tests/NatselTests. They live inside the crate so they can
//! reach `pub(crate)` internals, exactly as the Swift suite used `@testable import`.

/// Swift's `Float.pi`: `0x4049_0FDA`, one ULP below the correctly rounded π. The Swift stdlib
/// constant differs from `B3.pi` (`0x4049_0FDB`), and the reference tests pass `Float.pi` to
/// `makeQuatFromAxisAngle`; ports of those tests must use this value to stay bit-identical.
pub(crate) const SWIFT_FLOAT_PI: f32 = f32::from_bits(0x4049_0FDA);

mod body_tests;
mod bounce_tests;
mod broad_phase_tests;
mod ccd_tests;
mod collision_step_tests;
mod contact_update_tests;
mod convex_manifold_tests;
mod coupling_joint_tests;
mod distance_joint_tests;
mod distance_tests;
mod dynamic_tree_tests;
mod graph_tests;
mod hull_builder_tests;
mod hull_tests;
mod island_split_tests;
mod math_tests;
mod motor_joint_tests;
mod pair_tests;
mod parallel_joint_tests;
mod path_joint_tests;
mod prismatic_joint_tests;
mod ragdoll_tests;
mod revolute_joint_tests;
mod shape_collision_tests;
mod shape_creation_tests;
mod shape_geometry_tests;
mod six_dof_joint_tests;
mod sleep_tests;
mod sphere_capsule_tests;
mod spherical_joint_tests;
mod stack_tests;
mod step_tests;
mod triangle_manifold_tests;
mod weld_joint_tests;
mod wheel_joint_tests;
pub mod world_position_assertions;
mod world_query_tests;
mod base;
mod character;
mod collision;
mod containers;
mod simulation;
mod soft_body;
mod vehicle;
