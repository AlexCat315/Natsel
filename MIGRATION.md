# Natsel: Swift → Rust migration

This document tracks the full rewrite of the Natsel deterministic 3D physics engine from Swift to
Rust. It is the working contract for every translation step: read it before translating a file.

## Target

- Crate: `natsel` (see [Cargo.toml](file:///Cargo.toml)), edition 2024, `rust-version = 1.85`,
  `std`-only (no third-party dependencies).
- Sources: `src/` mirrors `Sources/Natsel/` one Rust file per Swift file.
- Binary: `src/bin/benchmarks.rs` from `Sources/Benchmarks/`.
- Tests: Rust `#[cfg(test)]` modules / `tests/` from `Tests/NatselTests/`.
- The Swift tree (`Sources/`, `Tests/`, `Package.swift`) stays in place as the translation
  reference until the port is complete and validated, then it is removed.

## Progress

Updated as batches land. "Lines" are source lines.

| Module | Swift files | Swift lines | Rust files | Status |
|---|---|---|---|---|
| Base | 5 | 1,028 | 6 | translated |
| Math | 4 | 1,149 | 5 | translated |
| Containers | 10 | 1,110 | 11 | translated |
| Types | 10 | 2,425 | 11 | translated |
| Collision | 52 | 13,680 | 53 | translated |
| Simulation | 63 | 24,647 | 25 | in progress (world, bodies, shapes, solver infra, joints A) |
| Character | 1 | 567 | 0 | **pending** |
| Vehicle | 5 | 2,290 | 0 | **pending** |
| Skeleton | 4 | 696 | 0 | **pending** |
| SoftBody | 10 | 4,385 | 0 | **pending** |
| Benchmarks | 4 | 1,190 | 0 (stub) | **pending** |
| Tests | 134 | ~15,000 | 0 | **pending** |

Migrated core so far: 20,386 Rust lines across 97 files. Remaining Swift: ~34,000 lines plus tests.

### Batch log

- Batch 0 (setup): placeholder `mod.rs` for `simulation`, `character`, `vehicle`, `skeleton`,
  `soft_body`; stub `src/bin/benchmarks.rs` so the manifest resolves. Baseline `cargo check`:
  39 errors in already-translated files, recorded in "Known defects" below and deferred to the
  final build phase per the no-testing-during-migration rule.
- Batch 1 (anchor): `collision/height_field/{factories,text_file}.rs` (the last two collision
  files; `DynamicTree/Traversal.swift` was already folded into `collision/dynamic_tree.rs`);
  `simulation/world_runtime.rs` (1,797 lines) as the `World` anchor; leaf types
  `{wide_types, contact_payload, contact_store, end_event_buffer, triangle_surface, shape_planes}.rs`,
  `simulation/continuous/{impact, compound_impact}.rs`,
  `simulation/mesh_contact/{cache, reduction, selection}.rs` — 2,506 lines.
  14 of 63 simulation files done.
- Batch 2 (solver infrastructure + bodies): `simulation/{body_api, body_queries, shape_runtime,
  constraint_graph, solver_integration, solver_prepare, solver_set_ops, solver_stages}.rs`
  (6,621 Rust lines). 20 of 63 simulation files done.
- Batch 3 (joints A + snapshot): `simulation/{joint, path_joint, wheel_joint, parallel_joint,
  six_dof_joint, coupling_joints, world_state}.rs` (8,064 Rust lines). 25 of 63 simulation files
  done, 23 files in `src/simulation/`. No `todo!()`/`unimplemented!()` anywhere in the port so far.
  39,475 Rust lines total.
- Agent sizing lesson: a single subagent tops out around 150 tool calls, which is roughly
  900–1,500 Swift lines of faithful translation. Batches are now capped at ~900 Swift lines and
  split one-file-per-agent when a file exceeds ~700 lines.

## Known defects (baseline, to fix in the final build phase)

All in already-translated files; no new translation is blocked by them.

- `base/hash.rs:216,229` — `E0618`: value called as a function.
- `base/worker_pool.rs:168` — `E0277`: closure `F` not `Send`/`Sync`.
- `collision/compound/compound.rs` (6) — `E0133`: unsafe calls outside `unsafe` block (edition 2024).
- `collision/compound/interning.rs:16` — `E0599`: `Default` bound unsatisfied.
- `collision/convex_manifold.rs:1488,1532` — `E0425`: missing `find_hull_support_vertex`.
- `collision/height_field/mod.rs` — `E0583`: `factories`, `text_file` modules absent (batch 1).
- `collision/height_field/{planes,proxy_overlap,regions}.rs` — `E0382`: moved `region.columns`.
- `collision/height_field/sweep_cast.rs:206`, `collision/hull_queries.rs:96`,
  `collision/mesh/overlap.rs:218` — `E0308`: `&ShapeCastPairInput` vs value.
- `collision/hull.rs:263,296,698` — mutability, arity, missing `box_inertia`.
- `collision/hull_queries.rs:39` — missing `b3_assert` import.
- `collision/internal_math.rs:186,200` — `Vec2::ZERO` missing.
- `collision/manifold.rs:166,168` — `i32` vs `u8`.
- `collision/mesh/overlap.rs:40,288` — arity; `:154,237` missing `get_by_index`.
- `collision/proxy.rs:43,51` — `Hull::vertex_count` is a method.
- `containers/dynamic_array.rs:119` — missing `b3_assert_msg`.
- `math/types.rs:133` — `Vec3` vs `Transform`.
- `types/debug_draw.rs:9` — `Capsule`/`HullData`/`Sphere` live in `types::geometry`, not `math::types`.

## Shared simulation vocabulary

`src/simulation/world_runtime.rs` is the anchor: it defines `World` plus `BodyFlags`, `SetType`,
`BodyState`, `BodySim`, `Body`, `ContactLink`, `JointLink`, `Island`, `IslandSim`, `SolverSet`,
`ContactEdge`, `Contact`, `JointEdge`, `Joint`, `JointSim`, `JointPayload`, `TriangleCache`,
`ContactCache`, `ContactCacheKind`. Read that file before translating any `extension World` file;
use the names it already declares instead of inventing new ones.

Callback and closure fields keep Swift's optionality (`Option<Box<dyn Fn(...) + Send + Sync>>`).
`World::solver_set` borrows rather than copying; `mutate_body_sim` / `mutate_body_state` take a
closure and write back in place. Fixed traversal stacks are raw pointers owned by `World` and freed
in `Drop`. Shared-ownership appears only at `Vec<Option<Arc<ConstraintPath>>>` for path joints.

Foreign paths assumed before their file landed (the file must define exactly these):
`broad_phase::{BroadPhase, BroadPhase::new(Capacity)}`,
`constraint_graph::ConstraintGraph::new(i32)`,
`contact_creation::{TraversalPair, NodePair}`,
`contact_payload::{ContactPayload, ContactPayloadPool, MeshContactScratch}`,
`contact_solver::{ManifoldConstraint, SolverContactData}`,
`continuous::{ContinuousContext, impact, compound_impact}`,
`continuous::dynamic_impact::DynamicContinuousImpulse`,
`coupling_joints::{GearJointSim, RackAndPinionJointSim, PulleyJointSim}`,
`end_event_buffer::EndEventBuffer<T>`, `joint::DistanceJointSim`,
`motor_joint::MotorJointSim`, `parallel_joint::ParallelJointSim`,
`path_joint::{ConstraintPath, PathJointSim}`, `prismatic_joint::PrismaticJointSim`,
`revolute_joint::RevoluteJointSim`, `six_dof_joint::{SixDOFJointSim, SixDOFJointData}`,
`spherical_joint::SphericalJointSim`, `weld_joint::WeldJointSim`, `wheel_joint::WheelJointSim`,
`overflow_contact_solver::OverflowContactConstraint`, `sensor::Sensor`,
`shape_creation::HullDatabase`,
`shape_runtime::{Shape, ShapeGeometry, destroy_shape_proxy, destroy_shape_allocations, make_shape_proxy}`,
`solver_integration::{Softness, StepContext, TaskContext, FinalizeThreadData, FinalizeViews}`,
`solver_prepare::{ContactPrepareItem, ContactPrepareViews, WideJointPrepareItem}`,
`solver_stages::SolverViews`, `wide_joint_solver::SolverJointData`,
`vehicle::vehicle_runtime::VehicleRecord`,
`soft_body::soft_body_runtime::SoftBodyRecord`,
`soft_body::soft_body_ray_cache::SoftBodyRayCache`.

## Translation conventions

### File and module layout

- One Swift file → one Rust file with the snake_case equivalent name:
  `SolverIntegration.swift` → `solver_integration.rs`, `WorldState.swift` → `world_state.rs`.
  Subdirectories are preserved (`Simulation/Continuous/Impact.swift` → `src/simulation/continuous/impact.rs`).
- Each directory gets a `mod.rs` declaring its children in the Swift file order, plus the
  `pub use` re-exports that `src/lib.rs` needs.
- A Swift `extension Type` inside file F becomes an `impl Type` block inside the Rust file
  translated from F. Keep the Swift file boundaries: do not merge extensions into one file.
- Free functions and constants keep their file placement.

### Naming

- `camelCase` → `snake_case` for functions, methods and variables; acronyms stay words
  (`toAABB` → `to_aabb`, `ccd` → `ccd`).
- Type names keep Swift spelling in `PascalCase`, except `SoftBody` → `SoftBody` (type) but
  module `soft_body`.
- `init(...)` → `new(...)`; `static let x` → `pub const X` for scalars, `pub fn x()` for
  non-const values; `SomeName.null` → `SomeName::NULL`; `isNull` → `is_null()`.
- Swift `deinit` → an explicit `fn destroy(&mut self)` or a `Drop` impl; prefer `Drop` only when
  the cleanup cannot fail and needs no arguments.
- Protocol → trait; `Sendable` is implied and dropped; `BitwiseCopyable` → `Copy` + `Clone`.
- Private Swift members (`internal`/`private`) become Rust `fn` without `pub`, or `pub(crate)`
  when another module needs them.

### Types and memory

- Value types (`struct`) → `#[derive(Clone, Copy, Debug, Default, PartialEq)] #[repr(C)] pub
  struct` with `pub` fields, matching the Swift field order and widths exactly so packed buffers
  and serialization stay compatible.
- Reference types (`class`) → `struct` owned by the world/arena that created it. Use `Arc`,
  `Rc`, or `RefCell` only where Swift relies on shared mutable reference semantics; document the
  reason at the type.
- Optional: Swift `T?` → `Option<T>`. Do not use `NaN`-or-sentinel optionals unless the Swift code
  already does.
- `inout` parameters → `&mut`. Swift's exclusive-access checks are gone; do not create overlapping
  borrows, and use raw pointers (`*mut`) with a `// SAFETY:` comment when the Swift code depends on
  aliasing it was relying on unchecked exclusivity for.
- Swift arrays → `Vec<T>` for growable storage, fixed `#[repr(C)]` arrays (`[T; N]`) for
  manifold/contact batches and any buffer whose layout is load-bearing.
- Swift `withUnsafePointer(to:)`, `UnsafeMutableBufferPointer`, `malloc`-backed storage →
  `unsafe` blocks over the same containers already in `src/containers/` (`arena`, `block_allocator`,
  `dynamic_array`, `stable_storage`, `bit_set`, `id_pool`), not `Vec` re-allocations.
- No `unwrap()` on engine paths: propagate with `Option`/`Result` where Swift returns nil or
  throws, and use `b3_assert!` where Swift asserts.

### Determinism (hard requirement)

- Preserve operand order, association, rounding, signed zero, NaN propagation and tie-breaking.
  Never reassociate a float expression, never replace an explicit comparison chain with
  `f32::min/max` if the Swift version had different tie behavior.
- `Float.rounded(.toNearestOrAwayFromZero)` → the existing helper in `src/math/functions.rs`;
  reuse those helpers instead of calling libm so scalar/SIMD lanes keep agreeing.
- Worker count must not change results. Any stage that partitions work must derive its
  accumulation order from item indices, not from completion order.
- Where Swift used explicit SIMD, keep the lane structure. Use `core::arch` intrinsics guarded by
  `#[cfg(target_arch = ...)]` with a scalar fallback that performs the same per-lane operations in
  the same order; do not let the auto-vectorizer change semantics by relying on the same explicit
  lane arithmetic as the Swift original.

### Logging, assertions, config

- `b3Assert` → `b3_assert!`, `b3AssertMsg` → `b3_assert_msg!`, `b3Validate` → `b3_validate!` from
  `crate::base::core`; constants from `crate::base::constants as b3`.
- The `validate` cargo feature gates validation-only code, mirroring Swift's `-DCDEBUG`-style splits.

### Public API

- Keep Swift's public surface shape: method names, argument order, returned structs. Callers ported
  from Swift should need only camelCase→snake_case renames.
- `World` methods that Swift exposed on the class become methods on `crate::simulation::World`.
- Output-parameter structs (Swift `inout SomeOutput`) stay as `&mut SomeOutput`.

## Rules for the migration process

- Translate file by file, in dependency order: leaves first (`WideTypes`, `ConstraintGraph`),
  then state owners (`WorldState`, `WorldRuntime`), then systems.
- Do not stub, truncate, `todo!()`, or "simplify" a function. Every Swift branch, early return,
  and constant must exist in the Rust file. If a Swift file references something not yet
  translated, reference the planned Rust path (`crate::simulation::x::Y`) and keep going;
  resolve at build time.
- No builds, tests, or benchmarks during translation. `cargo` runs only in the final phase, where
  compile errors are fixed without changing algorithm semantics.
- Every batch updates the Progress table and Batch log in this file.
- Platform coverage stays complete: threading, prefetch, and time primitives in `base/platform.rs`
  keep aarch64, x86_64, and generic paths; nothing may be implemented only for one target.
