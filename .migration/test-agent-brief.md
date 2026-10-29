# Translation agent brief (Swift -> Rust), tests & benchmarks

You are one of many agents translating the Natsel Swift engine to Rust. Work only on the
files assigned to you. Another agent owns every other file.

## Hard rules
1. DO NOT compile, DO NOT run `cargo check`, `cargo build`, `cargo test`, or any formatter.
   Translation is mechanical and verification happens only at the very end.
2. Write ONLY the output files listed in your assignment. Never edit a `mod.rs`, `MIGRATION.md`,
   or another agent's file. The module tree is already declared.
3. Translate every test case, every assertion, every helper in the assigned Swift files.
   Never skip a case, never weaken an assertion, never replace a body with `todo!()` or
   a comment. If a Swift construct is unclear, keep the closest faithful Rust and move on;
   do not stall. Budget your turns: 1-2 large Read calls, then 3-5 large Write calls.
4. Preserve exact numeric literals, operation order, associativity, and tolerance values.
   Determinism is the engine contract.

## Conventions
- Source: `Tests/NatselTests/<path>.swift` -> `src/tests/<snake_dir>/<snake_name>.rs`
  (the exact target path is given in your assignment; the `mod x;` line already exists).
- Suite lives inside the crate, so use `use crate::...` and reach `pub(crate)` items directly.
  This replaces `@testable import Natsel`. No `extern crate`.
- `@Suite("Name") struct X` -> plain `struct X` is unnecessary; put the cases as top-level
  `#[test] fn case_name() { ... }` items in the module. `@Test("name") func f()` ->
  `#[test] fn f()`. Keep the Swift function name in snake_case; if the Swift name has spaces,
  slugify it. Prefix with something descriptive when two cases in the same file would collide.
- `#expect(cond)` -> `assert!(cond)`; `#expect(a == b)` -> `assert_eq!(a, b)` when both sides
  are `PartialEq`, otherwise `assert!(a == b)`. `#expect(a == b accuracy: t)` ->
  `assert!((a - b).abs() <= t)`. `#require(x != nil)` -> `let x = x.expect(...)`.
  `Issue.record`/`ctx.record` -> `panic!` is wrong; use `assert!(false, "...")` only if the
  Swift genuinely fails; for a recorded-but-continued issue use a plain `println!` of the
  message followed by the same evaluation, keeping the test's final assertion intact.
- Shared helpers from `WorldPositionAssertions.swift` are already ported. Import them with
  `use crate::tests::world_position_assertions::{abs_float, distance};`. `distance(a, b)`
  accepts `Pos` or `Vec3` for either operand.
- Types: `World` is a struct -> `let mut world = World::default();` (or the constructor the
  Swift used). `let world = World(...)` -> `World(...)` if the Rust type exposes it; otherwise
  follow how the ported `src/simulation/world_runtime.rs` builds a world. Read that file's
  public API surface before inventing calls.
- Swift `Optional` -> `Option`; `x!` -> `x.unwrap()`; `?? default` -> `.unwrap_or(default)`.
- Swift arrays -> slices/`Vec` as appropriate; `for i in 0..n` -> `for i in 0..n` (exclusive)
  vs `0...n` (inclusive) — mind the difference.
- `Float` -> `f32`, `Double` -> `f64`, `Int` -> `isize` only when the engine uses `isize`,
  otherwise `usize`/`i32` matching the ported Rust signature.
- Names: `camelCase` functions -> `snake_case` methods as already defined in the ported Rust;
  check `src/` for the real method name instead of guessing. When you cannot find a symbol,
  use the most plausible `crate::` path consistent with neighbouring ported code; the final
  build phase fixes cross-agent mismatches.
- Keep any Swift comment that explains WHY (tolerances, determinism, platform caveats),
  reworded into Rust doc/comment style. Delete redundant what comments.

## Reference files to read first
- `MIGRATION.md` — "Translation conventions", "Test translation conventions",
  "Shared simulation vocabulary".
- `src/tests/world_position_assertions.rs` — the helper module.
- `src/lib.rs` — the crate's module tree.
