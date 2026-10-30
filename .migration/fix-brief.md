# Rust compile-error fixing brief (Natsel Swift→Rust port)

The port is COMPLETE at file level: all 164 Swift files have Rust counterparts. The engine has
213k... no: ~64k Rust lines. The remaining work is making it COMPILE without changing semantics.

Repo: /Users/cyke666/dev/Natsel  (edition 2024, std-only, no third-party crates)

## Hard rules
1. Fix ONLY the files assigned to you. Other agents own other files. Never edit a `mod.rs`,
   `src/lib.rs`, `MIGRATION.md`, or a file outside your assignment.
2. DO NOT change algorithm semantics. Do not reassociate float expressions, do not reorder
   operations, do not change numeric literals or tolerances, do not delete branches or
   assertions. This engine's contract is bit-exact determinism.
3. Prefer the MINIMAL fix: add a missing import, add `(expr as T)` parentheses, add `&`/`mut`,
   change a type to the one consumers already expect, add `#[repr(..)]`, etc. Do not refactor.
4. NO stubs, NO `todo!()`, NO `unimplemented!()`, NO commented-out code, NO `#[allow(dead_code)]`
   to silence a real error.
5. Do NOT run `cargo test`. You may run `cargo check --lib --message-format short` to see errors,
   but the Lead runs the authoritative build. Filter to your own files:
   `cargo check --lib --message-format short 2>&1 | grep '^src/<yourdir>/'`
6. No comments unless the fix is genuinely non-obvious (e.g. why a raw pointer or an unsafe block
   is required). Never add "what" comments.

## Context you need
- Assertions are FUNCTIONS not macros: `b3_assert(cond)`, `b3_assert_msg(cond, "msg")`,
  `b3_validate(cond)` from `crate::base::core`.
- Most common error: `x as i32 < Y` parses as generics. Fix to `(x as i32) < Y`.
- `math::functions` holds free helpers (`mul_sub`, `max_vec3`, `min_vec3`, `dot`, ...). Missing
  "function not found" is almost always a missing import, not a missing function — grep `src/` for
  the `pub fn` first and import it from its real module.
- `Vec2::ZERO` / `Vec2::ONE` exist in `src/math/types.rs`.
- The `World` struct and all simulation vocabulary live in `src/simulation/world_runtime.rs`.
  Read it before inventing any name.
- `SoftBodySettings` / `SoftBodyEdge` / `U32x2` live in `src/soft_body/soft_body_settings.rs`.
- `E0308 expected f32, found integer` almost always means a literal needs `.0` (e.g. `1` → `1.0`)
  because Rust does not implicitly widen integer literals to float. Check the Swift original in
  `Sources/` when the intended value could be a genuine integer.

## Report back
State: files touched, error count before/after for your files, and ANY fix where you had to make a
semantic judgement call (so the Lead can review it). Do not claim success without pasting the
final `cargo check` filtered output for your files.

## Test translation status (Lead-maintained)

90 of 134 Swift test files have no Rust counterpart yet (~14,000 Swift lines). Missing by group:
  Simulation 42 files / 8,008 lines     Collision 6 / 1,733        Tests root 8 / 1,300
  SoftBody 8 / 1,005                    Collision/Mesh 9 / 492     Vehicle 1 / 415
  Collision/Compound 6 / 394            Collision/HeightField 4 / 226
  Character 1 / 210                     Collision/Mover 3 / 150
  Containers 1 / 95                     Base 1 / 49

`src/bin/benchmarks.rs` is a 1-line `fn main() {}` stub; the 4 Swift benchmark files
(`Sources/Benchmarks/`, 1,190 lines) are not translated.

The `mod.rs` test tree already declares every module, so a missing leaf file is a hard compile
error. Prefer writing the leaf file rather than editing any `mod.rs`.
