# Benchmarks translation brief (Swift -> Rust)

Sources/Benchmarks/*.swift -> the `benchmarks` binary crate.

Layout (already fixed, do not change it):
- `Sources/Benchmarks/main.swift`          -> `src/bin/benchmarks.rs`   (module root + `fn main`)
- `Sources/Benchmarks/Scenes.swift`        -> `src/bin/benchmarks/scenes.rs`
- `Sources/Benchmarks/FeatureScenes.swift` -> `src/bin/benchmarks/feature_scenes.rs`
- `Sources/Benchmarks/QualityScenes.swift`  -> `src/bin/benchmarks/quality_scenes.rs`

`src/bin/benchmarks.rs` must declare, after its module doc and imports:
```rust
pub mod scenes;
pub mod feature_scenes;
pub mod quality_scenes;
```

Rules
1. DO NOT compile, check, test, or format anything. Mechanical translation only.
2. Write only your assigned output file(s). Do not touch any file under `src/tests/`,
   `src/lib.rs`, `MIGRATION.md`, or another agent's module.
3. Translate every scene builder, every parameter, every printed label. Keep the exact
   iteration counts, sizes, seeds, and numeric literals — the benchmark numbers are part of
   the engine's determinism contract.
4. Use `use natsel::...` for engine types (the library crate is named `natsel`); reach the
   same public API the ported `src/simulation/world_runtime.rs` exposes. Check the real Rust
   names in `src/` rather than guessing; if a symbol is uncertain use the most plausible path
   and keep going.
5. Swift `print`/string interpolation -> `println!` with `{}` placeholders and the same text.
6. Keep the CLI argument handling of `main.swift` (`CommandLine.arguments`) as `std::env::args()`.
7. Read `MIGRATION.md` ("Translation conventions", "Shared simulation vocabulary") first.
