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

### Fixed

- All 18 benchmark scenes produce the same `worldStateHash` as the Swift reference, including
  `trees25`, `vehicles`, `washer` and `convex_pile`.
- Mesh binned-SAH node counts, GJK box-box witness points, wide vs scalar joint stepping, and the
  height-field contact determinism now agree with the reference implementation.