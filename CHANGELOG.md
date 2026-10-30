# Changelog

## Unreleased

### Changed

- Full Swift → Rust 2024 migration of the engine (`natsel` crate): every file under
  `Sources/Natsel/` has a Rust counterpart, and the benchmarks/test harness moved to
  `src/bin/benchmarks.rs` and the in-crate test suite.
- `collision/hull_builder.rs` is now a faithful index-based QuickHull port, including Swift's
  `listPushBack` ordering quirk (a node pushed into an empty list stays at the tail). Hull vertices,
  faces and mass properties are bit-identical to the Swift result.
- `math/functions.rs::unwind_angle` now implements the IEEE 754 remainder exactly, matching Swift's
  `Float.remainder(dividingBy:)`; the previous f32 form shifted procedural wave-mesh vertices by a
  few ULP.
- CI, editor configuration, and developer documentation target Cargo only.

### Removed

- The Swift implementation tree (`Sources/`, `Tests/`, `Package.swift`) and its build artifacts.
  The Swift sources remain available in git history; reference state hashes captured before
  removal live in `Tools/Benchmarks/reference-hashes.json`.

### Fixed

- All 18 benchmark scenes produce the same `worldStateHash` as the Swift reference, including
  `trees25`, `vehicles`, `washer` and `convex_pile`.
- Mesh binned-SAH node counts, GJK box-box witness points, wide vs scalar joint stepping, and the
  height-field contact determinism now agree with the reference implementation.
- `world.enable_speculative` was declared and snapshot-restored but never read, so writing it had no
  effect. It is now the master gate for speculative contacts in `create_contact`: a contact only
  gets `ENABLE_SPECULATIVE_POINTS` when the world switch and both shapes'
  `ShapeDef::enable_speculative_contact` are set. Defaults are unchanged, so existing scenes hash
  identically.
- Mesh shapes could be attached to dynamic and kinematic bodies, where they contribute no mass
  (`compute_shape_mass` only integrates capsules, hulls and spheres). The body ended up with zero
  inverse mass and silently never moved. Mesh shapes are now static-only, matching the existing
  rule for height fields and compounds and the mesh rejection already performed by
  `create_dynamic_compound`; the call returns a null `ShapeID`.

### Changed

- `World::shape_type` returns `Option<ShapeType>` instead of `ShapeType`, and `World::shape_body`
  returns a null `BodyID` for a stale id. Both previously indexed the shape array unchecked and
  panicked on a rejected or destroyed id, which the new mesh rejection made easy to hit.
- `World::shape_is_valid` is now public, so callers can test a shape id before using it.
