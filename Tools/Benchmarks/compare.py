#!/usr/bin/env python3
"""Compare two Natsel Release builds, alternating them within each scene."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess

SCENES = [
    "large_pyramid", "many_pyramids", "joint_grid", "spinner", "sleep",
    "trees25", "washer", "convex_pile", "vehicles", "cloth", "cloth_self64",
    "soft_cubes", "cloth_stack", "path_beads", "ragdoll_motors", "rollback",
    "soft_queries", "sparse_soft",
]
DEFAULT_SCENES = ["many_pyramids", "joint_grid"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenes", nargs="+", choices=SCENES, default=DEFAULT_SCENES)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    executables = {"before": args.before.resolve(), "after": args.after.resolve()}
    for executable in executables.values():
        if not executable.is_file():
            parser.error(f"executable not found: {executable}")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    results = {
        "repeats": args.repeats,
        "workers": 1,
        "binaries": {
            label: {"path": str(binary), "sha256": hashlib.sha256(binary.read_bytes()).hexdigest()}
            for label, binary in executables.items()
        },
        "scenes": {},
    }
    env = {key: value for key, value in os.environ.items() if not key.startswith("NATSEL_")}
    env["NATSEL_WORKERS"] = "1"
    env["NATSEL_HASH"] = "1"

    def save():
        (output / "comparison.json").write_text(json.dumps(results, indent=2) + "\n")

    for scene in args.scenes:
        data = {"before_ms": [], "after_ms": [], "before_hashes": [], "after_hashes": []}
        results["scenes"][scene] = data
        for repeat in range(args.repeats):
            # Reverse every second pair to reduce ordering and thermal bias.
            order = ["before", "after"] if repeat % 2 == 0 else ["after", "before"]
            for label in order:
                print(f"{scene} {repeat + 1}/{args.repeats} {label}", flush=True)
                run = subprocess.run(
                    [str(executables[label]), f"-b={scene}", "-r=1"],
                    cwd=output, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                )
                log = output / f"{scene}-{label}-{repeat + 1}.log"
                log.write_text(run.stdout)
                if run.returncode:
                    raise RuntimeError(f"Natsel benchmark exited {run.returncode}; see {log}")
                match = re.search(rf"^{re.escape(scene)}\.csv: threads,ms 1,([0-9.eE+-]+)$", run.stdout, re.M)
                hashes = re.findall(r"^state: ([0-9a-f]+)$", run.stdout, re.M)
                if match is None or len(hashes) != 1:
                    raise RuntimeError(f"missing timing or unique state hash for {scene} ({label})")
                timing = float(match.group(1))
                if not math.isfinite(timing) or timing <= 0:
                    raise RuntimeError(f"invalid timing for {scene} ({label}): {timing}")
                data[f"{label}_ms"].append(timing)
                data[f"{label}_hashes"].append(hashes[0])
                save()
                if len(set(data[f"{label}_hashes"])) != 1:
                    raise RuntimeError(f"non-repeatable Natsel state in {scene} ({label}); see saved logs")
                print(f"  {timing:.3f} ms", flush=True)
        data["speedup"] = min(data["before_ms"]) / min(data["after_ms"])
        data["versions_match"] = data["before_hashes"][0] == data["after_hashes"][0]
        save()
        print(f"{scene}: {data['speedup']:.3f}x", flush=True)
    results["total_before_ms"] = sum(min(d["before_ms"]) for d in results["scenes"].values())
    results["total_after_ms"] = sum(min(d["after_ms"]) for d in results["scenes"].values())
    results["speedup"] = results["total_before_ms"] / results["total_after_ms"]
    save()
    print(f"Total: {results['speedup']:.3f}x", flush=True)


if __name__ == "__main__":
    main()
