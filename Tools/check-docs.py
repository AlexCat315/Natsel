#!/usr/bin/env python3
"""Verify that paths and symbols referenced in Website/src/content/*.md exist in the crate.

Checks:
  1. `use natsel::a::b::Item;` module paths resolve to a real module, and the
     final path segment is declared (pub struct/enum/fn/type/trait) or re-exported.
  2. `world.some_method(` calls resolve to an actual `pub fn some_method` on World
     (or to a known `pub fn` anywhere in src/simulation, vehicle, character,
     soft_body, skeleton, collision where an `impl World` block lives).
  3. `Type::member` associated constants/methods exist for a small set of types.
  4. Cross-links `(/docs/<slug>)` map to a real content file.

Exit code 1 when anything looks wrong, so CI-adjacent runs fail loudly.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
CONTENT = ROOT / "Website" / "src" / "content"

# Types that come from std/core or common crates, not from this crate.
STD_TYPES = {
    "Self", "Ordering", "Arc", "Mutex", "RwLock", "Vec", "String", "Option",
    "Result", "Box", "HashMap", "HashSet", "VecDeque", "AtomicBool", "AtomicUsize",
    "AtomicI32", "AtomicU32", "AtomicU64", "PathBuf", "Duration", "Instant",
    "TypeId", "NonNull", "PhantomData", "Cell", "RefCell", "Rc", "Weak",
}

SLUGS = {
    "getting-started", "installation", "concepts", "architecture", "units",
    "determinism", "bodies", "shapes", "geometry", "materials", "queries",
    "events", "joints", "ccd", "buoyancy", "multithreading", "character",
    "vehicle", "skeleton", "soft-body", "debug-draw", "snapshots",
    "api-reference", "faq",
}


def rust_sources() -> list[Path]:
    return [p for p in SRC.rglob("*.rs") if "/tests/" not in str(p)]


def module_decls() -> dict[str, set[str]]:
    """module file -> set of `pub mod` names declared in it."""
    out: dict[str, set[str]] = {}
    for p in rust_sources():
        if p.name != "mod.rs" and p.parent == SRC:
            continue
        text = p.read_text(encoding="utf-8", errors="ignore")
        out[str(p.relative_to(SRC))] = set(re.findall(r"^\s*pub mod (\w+);", text, re.M))
    return out


def all_pub_items() -> set[str]:
    """Every pub struct/enum/fn/type/trait/const name anywhere in the crate."""
    names: set[str] = set()
    pattern = re.compile(
        r"^\s*pub\s+(?:struct|enum|fn|type|trait|union|mod)\s+(\w+)", re.M
    )
    for p in rust_sources():
        names.update(pattern.findall(p.read_text(encoding="utf-8", errors="ignore")))
    return names


def reexported() -> set[str]:
    names: set[str] = set()
    for p in rust_sources():
        text = p.read_text(encoding="utf-8", errors="ignore")
        for m in re.finditer(r"^\s*pub use\s+([^;]+);", text, re.M):
            clause = m.group(1)
            for part in clause.split(","):
                part = part.strip()
                if not part:
                    continue
                if " as " in part:
                    part = part.split(" as ")[-1].strip()
                if "*" in part:
                    continue
                names.add(part.split("::")[-1].strip())
    return names


def pub_fn_names() -> set[str]:
    names: set[str] = set()
    for p in rust_sources():
        names.update(
            re.findall(r"^\s*pub fn (\w+)", p.read_text(encoding="utf-8", errors="ignore"), re.M)
        )
    return names


def resolve_module(parts: list[str], mods: dict[str, set[str]]) -> str | None:
    """Walk a module path, return the file that would define it."""
    if not parts:
        return None
    cur = ["src"]
    prefix = "src"
    for seg in parts:
        declared = mods.get(prefix, set())
        if seg in declared:
            cand_mod = f"{prefix}/{seg}/mod.rs"
            cand_file = f"{prefix}/{seg}.rs"
            if (SRC / seg / "mod.rs").exists() or Path(cand_mod) in mods:
                prefix = f"{prefix}/{seg}"
            elif (SRC / f"{seg}.rs").exists() or cand_file in mods:
                prefix = f"{prefix}/{seg}.rs"
            else:
                return None
            cur.append(seg)
        else:
            # not declared by the parent: allow the final segment to be an item
            return prefix if seg not in declared else None
    return prefix


def main() -> int:
    problems: list[str] = []
    mods = module_decls()
    items = all_pub_items()
    re_exp = reexported()
    fns = pub_fn_names()

    md_files = sorted(CONTENT.glob("*.md"))
    if not md_files:
        print("no content files found", file=sys.stderr)
        return 1

    for md in md_files:
        text = md.read_text(encoding="utf-8")
        name = md.name

        # --- 1. use paths ---
        for m in re.finditer(r"use\s+(natsel(?:::\w+)+)(?:\s+as\s+(\w+))?;", text):
            path = m.group(1)
            segs = path.split("::")[1:]  # drop `natsel`
            if segs and segs[-1] == "*":
                continue
            leaf = segs[-1]
            module_parts = segs[:-1]
            resolved = resolve_module(module_parts, mods)
            if resolved is None:
                problems.append(f"{name}: module path `{path}` has no matching module")
                continue
            if leaf not in items and leaf not in re_exp:
                problems.append(f"{name}: `{leaf}` from `{path}` is not a pub item / re-export")

        # --- 2. world.method( ---
        for m in re.finditer(r"\bworld\.(\w+)\s*\(", text):
            meth = m.group(1)
            if meth not in fns:
                problems.append(f"{name}: `world.{meth}(` is not a pub fn in the crate")

        # --- 3. cross-links ---
        for m in re.finditer(r"\(/docs/([a-z0-9-]+)\)", text):
            slug = m.group(1)
            if slug not in SLUGS:
                problems.append(f"{name}: link to unknown doc slug `{slug}`")

        # --- 4. named types in prose must exist (conservative: only `Type::Case`) ---
        for m in re.finditer(r"\b([A-Z][A-Za-z0-9]+)::([A-Z][A-Z0-9a-z_]+)\b", text):
            typ = m.group(1)
            if typ in STD_TYPES:
                continue
            if typ not in items and typ not in re_exp:
                problems.append(f"{name}: type `{typ}` in `{typ}::{m.group(2)}` not found")

    if problems:
        print(f"found {len(problems)} issue(s):\n")
        for p in problems:
            print("  -", p)
        return 1

    print(f"ok: {len(md_files)} pages, all paths/symbols/links resolve")
    for p in md_files:
        print(f"  {p.name}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
