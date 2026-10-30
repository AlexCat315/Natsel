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
| Base | 5 | 862 | 5 | translated |
| Math | 4 | 957 | 4 | translated |
| Containers | 10 | 934 | 10 | translated |
| Types | 10 | 1,829 | 10 | translated |
| Collision | 52 | 11,349 | 52 | translated |
| Simulation | 63 | 21,011 | 63 | translated |
| Character | 1 | 495 | 1 | translated |
| Vehicle | 5 | 1,973 | 5 | translated |
| Skeleton | 4 | 620 | 4 | translated |
| SoftBody | 10 | 4,023 | 10 | translated |
| Benchmarks | 4 | 1,190 | 4 | translated |
| Tests | 134 | ~15,000 | 134 | translated (530 `#[test]`) |

Every Swift file under `Sources/Natsel/` has a Rust counterpart: 164/164 files, one Rust file per
Swift file. The crate is ~100,000 Rust lines across 330 files, `cargo build --lib` and
`cargo build --bin benchmarks` both succeed, and the full test suite compiles.
`Sources/`, `Tests/` and `Package.swift` are still in place as the translation reference.

Hash parity: both binaries build in release mode and every one of the 18 benchmark scenes
(`--all`, `NATSEL_HASH=1`) ends with the same `state:` hash in Rust and Swift, including the
longest runs (`washer` 1000 steps, `spinner` 800, `vehicles` 600, `trees25`/`convex_pile` 500).

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
- Batch 3b (mesh contact completion): `simulation/mesh_contact/{materials, persistence, triangles,
  update}.rs` from the last four Swift `MeshContact` files (268 Swift lines, 268+ Rust lines).
  29 of 63 simulation files done. `MeshTriangleManifold`/`MeshSurvivor` now exist as
  `cache/reduction/selection` import them; `World::compute_mesh_manifolds` matches both landed call
  sites (`contact_update.rs`, `compound_contact.rs`) with `material_indices: Option<&[u16]>`.
  Swift's `ArraySlice<LocalManifoldPoint>` became the `Copy` pointer-view `MeshPointSlice`; the
  scratch-arena stages pass owned `Vec`s between select → cluster → persist where Swift relied on
  COW aliasing.
- Agent sizing lesson: a single subagent tops out around 150 tool calls, which is roughly
  900–1,500 Swift lines of faithful translation. Batches are now capped at ~900 Swift lines and
  split one-file-per-agent when a file exceeds ~700 lines.
- Batch 4 (debug draw): `simulation/world_debug_draw.rs` (901 Rust lines) and
  `simulation/world_debug_draw_extras.rs` (166 Rust lines) from `WorldDebugDraw.swift` +
  `WorldDebugDrawExtras.swift` (646 Swift lines). 31 of 63 simulation files done. The extras pass
  assumes `vehicle::vehicle_types::VehicleID`, the `VehicleRecord`/`SoftBodyRecord` field shapes
  recorded above, and `World::vehicle_wheel_transform` with Swift's default wheel axes passed
  explicitly (the `world_mover.rs` precedent for defaulted Swift arguments).
- Batch 4 (test scaffolding): `src/lib.rs` gained `#[cfg(test)] mod tests;`, and `src/tests/**/mod.rs`
  was generated from the Swift test tree — 12 `mod.rs` files declaring all 134 test modules, one per
  `Tests/NatselTests/**/*.swift`, with `world_position_assertions` the only `pub mod`.
  `Tests/NatselTests/WorldPositionAssertions.swift` is ported to
  `src/tests/world_position_assertions.rs`: `abs_float`, plus a single generic
  `distance<A: IntoPos, B: IntoPos>` that replaces Swift's two `Pos`/`Vec3` overloads, so a call site
  reads the same in both languages. `IntoPos` is the only new abstraction the port needs there.
- Concurrency lesson: more than ~4 parallel translation subagents makes the model queue fail
  ("Queuing failed") and the whole wave is lost. Batches 5+ dispatch at most 4 agents at a time and
  verify landed files before starting the next wave. Shared briefs live in `.migration/test-agent-brief.md`
  and `.migration/bench-agent-brief.md`.
- Batch 5 (validation and hash parity): full test suite runs green (530 `#[test]`, `--test-threads=1`),
  and all 18 benchmark scenes now produce bit-identical `worldStateHash` values against the Swift
  reference. Three behavioural gaps were found by hash bisection and closed:
  - `collision/hull_builder.rs` was replaced with a faithful index-based QuickHull port. The critical
    detail is Swift's `listPushBack` quirk: it inserts before `head.prev`, so a node pushed into an
    empty list stays at the tail forever. `link_vertex_back`/`link_face_back` replicate that ordering;
    without it every hull (and the tree/washer/convex_pile scenes) diverged in mass properties.
  - `math/functions.rs::unwind_angle` used `x - round(x / period) * period` in f32. Swift's
    `Float.remainder(dividingBy:)` is the IEEE 754 remainder; the f32 form loses low bits and made
    every procedural wave-mesh vertex differ by a few ULP (trees25 and vehicles then diverged at the
    first mesh contact). It now computes the exact remainder in f64 and narrows.
  - Swift's `Float.pi` is `0x40490FDA` while `B3.pi` is `0x40490FDB`; scenes and tests that used
    `Float.pi` use the `SWIFT_FLOAT_PI` constant.

## Known defects

The baseline compile-error list is closed: `cargo check --lib` reports **0 errors** and
`cargo build --bin benchmarks` succeeds. What remains are behavioural discrepancies the ported
tests found, tracked here rather than hidden.

Fixed during the final build phase (do not reintroduce):

- `base/hash.rs` `rapidhash` tail read `p + i - 16` with `usize` arithmetic. `p` has already
  advanced past the 112-byte blocks, so the remainder can be smaller than 16 even though the read
  is in bounds; Swift's `p + i - 16` was signed pointer arithmetic. Now uses `p.offset(i as isize - 16)`.
  This was a SubtractWithOverflow panic on any buffer whose length after block peeling is < 16.
- `base/worker_pool.rs`: the stage closure captured the bare `F` field because of precise capture,
  demanding `F: Send + Sync` and defeating `SendFn`'s erasure. Now calls through `SendFn::call(&self)`.
- `simulation/continuous.rs` + `continuous/`: Rust cannot have both `continuous.rs` and
  `continuous/mod.rs`. The directory's children are declared inside `continuous.rs`.
- Duplicate `MeshContactScratch`: `collision::convex_manifold` had a wrapper struct that duplicated
  `simulation::contact_payload::MeshContactScratch`. The wrapper is gone;
  `collide_hulls_with_scratch` now takes `sat: &mut SatScratch, face: &mut FaceScratch`, and the
  payload keeps the two typed scratches. The dead `sat_scratch`/`face_scratch` byte slabs and their
  `*_BYTE_COUNT` consts were removed (Swift reached those arenas by pointer arithmetic).
- `collision/hull.rs::hull_from_faces` linked twins through a `HashMap`, so twin pairs were not
  adjacent. `is_valid_hull`, the manifold feature queries and `hull_builder` all require
  `edges[2k].twin == 2k + 1` and `twin.twin == index`. The builder now renumbers edges so twins land
  in adjacent slot pairs.
- `collision/hull_builder.rs` iteration order: `merge_coplanar`'s boundary and the horizon walk both
  read out of `HashSet`/`HashMap`, whose order is randomised per process, so `create_hull` returned a
  different polygon rotation on every run. Both are sorted before use.
- `collision/hull_builder.rs::create_cone` rotated the upper ring by a half step and centred the
  ring heights; Swift advances the same `alpha` from zero with `y = 0` and `y = height`. The rotated
  version made the side quads non-planar so the coplanar merge could not fold them (64 half edges
  instead of 48).
- `collision/hull_builder.rs::create_rock` and `create_complex_hull` used invented RNGs. Both now
  follow Swift: the Fibonacci lattice for the rock, and the PEEL 32-bit LCG (`state = state *
  2147001325 + 715136305`, seed 42, `& 0xffff / 65535 - 0.5`) for the complex hull.
- `collision/mesh/tree_builder.rs` implemented only the median split and used it unconditionally.
  `MeshTreeBuild::with_options(primitives, use_median_split)` now implements Swift's three
  strategies: binned SAH (8 buckets per axis, cost = count × area), median (partition about the
  centre on the major axis, falling back to `count / 2` at the ends), and the half split used when
  SAH finds no axis and the range exceeds 8.
- `simulation/world_runtime.rs` had no `impl Default for World`; Swift's `init(capacity:)` defaulted
  its argument. Added, delegating to `World::new(Capacity::default())`.
- `simulation/sensor.rs::sensor_events` returned a positional tuple where Swift returned the
  labelled `(begin:end:)`. Now returns `SensorEvents { begin, end }`.
- `simulation/six_dof_joint.rs`: `SixDOFSwingLimit` has data-carrying variants, so its explicit
  discriminants need `#[repr(i32)]`.
- Edition 2024 `dangerous_implicit_autorefs`: field access through a raw-pointer deref
  (`(*ptr).field[i]`) needs explicit parentheses around the deref.
- `math/types.rs` had no `Vec2::ZERO` / `Vec2::ONE`.

Previously open discrepancies, all resolved during the final validation pass (the ported tests now
assert the Swift values and pass):

- `collision/mesh` binned-SAH tie-breaking built one node more than Swift for some scenes;
  `create_box_mesh` now reports the Swift `byte_count` of 656, and
  `generator_tests::primitives` / `ray_tests::ray_repeatability` agree.
- `collision/distance.rs` GJK picked the opposite but equally valid witness point for face-face box
  pairs; the witness now matches Swift (`(1, 1, 1)`), and
  `distance_tests::{box_box_distance, box_cast, overlap, receding_cast}` pass.
- `simulation/wide_joint_solver.rs` diverged from the scalar path for the spherical-joint grid; both
  `wide_joint_tests::joint_grid_*` now step in lockstep with the scalar solver.
- `tests/simulation/wide_equivalence_tests.rs::multi_manifold_terrain` diverged between two scalar
  worlds built by `create_wave_pile`; the height-field contact path is deterministic again.

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
`vehicle::vehicle_types::VehicleID`,
`vehicle::vehicle_runtime::World::vehicle_wheel_transform` (Swift's default `wheelRight`/`wheelUp`
axes are passed explicitly by the debug-draw caller),
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

## Test translation conventions

- `Tests/NatselTests/**/*.swift` → `src/tests/**/*.rs`, same relative path, snake_case file names.
  The `mod.rs` tree is already generated; write only the leaf test files assigned to you.
- The suite is inside the crate (`#[cfg(test)] mod tests;` in `src/lib.rs`), which replaces Swift's
  `@testable import`: use `use crate::...` and reach `pub(crate)` items directly.
- Swift Testing → built-in harness: `@Suite struct X` → `mod`-level grouping is already the file, so
  each `@Test("name") func f()` becomes `#[test] fn f()` with the Swift name in snake_case and the
  display string kept in a `// "…"` comment above it. `#expect(cond)` → `assert!(cond)`;
  `#expect(a == b)` → `assert_eq!`; `#expect(throws: ..)` → `#[should_panic]` or an explicit
  `catch_unwind` only where the Swift test asserted a throw; `Issue.record` → a `panic!` with the
  same message.
- Approximate comparisons keep the Swift tolerance exactly (`isApproximate` / epsilon arguments).
- `World` is a Rust struct: `let world = World()` becomes `let mut world = World::default()` (check
  the real constructor in `src/simulation/world_runtime.rs`), and every mutation the Swift test did
  through the class reference becomes a `&mut` call. Same for `CharacterVirtual`, `Ragdoll` and
  vehicles, which borrow the world (`&'w mut World`).
- Shared helpers in `Tests/NatselTests/WorldPositionAssertions.swift` are `pub(crate)` in
  `src/tests/world_position_assertions.rs`; other test files call them rather than redefining them.
- Do not weaken a test to make it pass and do not delete a case. If a Swift assertion cannot hold
  because the port changed semantics, keep the assertion and flag it in the report.
- `Sources/Benchmarks/*.swift` → `src/bin/benchmarks.rs` plus `src/bin/benchmarks/{scenes,
  feature_scenes,quality_scenes}.rs` (`mod scenes;` resolves into the `benchmarks/` directory).
