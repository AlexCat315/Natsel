# Natsel development

Natsel is a Rust physics engine. Changes are evaluated against its own behavior, correctness
requirements, and performance history.

## Platforms and structure

The crate is `natsel`, Rust edition 2024, `rust-version = 1.85`, `std`-only (no third-party
dependencies). The platform layer provides timing, threading, atomics, and prefetch hints; the
prefetch path is implemented for aarch64 and x86_64 and degrades to a no-op elsewhere. CI builds
and tests on Linux, Windows, and macOS.

- `src/base/`, `src/math/`, `src/containers/`: allocation, threading, hashing, arithmetic, storage.
- `src/types/` and `src/collision/`: public definitions, geometry, collision detection, queries.
- `src/simulation/`: bodies, contacts, constraint graphs, joints, CCD, events, world state.
- `src/character/`, `src/vehicle/`, `src/skeleton/`, `src/soft_body/`: specialized systems.
- `src/bin/benchmarks.rs`: benchmark and hash harness.

Public APIs use `Vec3`, `Pos` (world-space f64), `Quat`, `Transform`, `BodyID`, and `World`.
World coordinates are `f64`; local geometry and solver lanes are `f32`.
Geometry objects own their packed storage; identical geometry can be shared through the world.
Serialized allocations are zeroed before hashing so padding does not affect content identity.

## Determinism and correctness

The same version, configuration, and inputs must produce repeatable results. Worker count must
not change the simulation. Scalar and wide-lane paths preserve the evaluation order within each
lane, including operand order, expression association, signed zero, and tie-breaking rules.

Tests compare independently created Natsel scenes, scalar/wide execution, worker counts, and
save/restore sequences. Analytical geometry and physics checks cover collision, constraints,
momentum, energy, and replay quality. Intentional changes to evaluation or solver order may change
hashes between versions and must be evaluated alongside correctness tests.

The 2026-10-30 Swift → Rust migration contract additionally required bit-identical
`worldStateHash` output against the Swift implementation. Those reference hashes are kept in
[Tools/Benchmarks/reference-hashes.json](Tools/Benchmarks/reference-hashes.json) and must keep
matching unless a deliberate behavioral change is documented in the changelog.

```sh
cargo test
cargo build --release --all-targets
```

## Performance

Run only the scenes relevant to the change. Compare two Natsel versions using the same machine
and settings, outside unit tests. See [PERFORMANCE.md](PERFORMANCE.md) for the procedure and
historical results.

## Website

The Vue website lives in `Website/`. Run `npm run build` there to check types and production assets.