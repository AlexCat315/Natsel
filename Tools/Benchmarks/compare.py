#!/usr/bin/env python3
"""Compare two Release benchmark executables, alternating them within each scene."""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess

SCENES = [
    "large_pyramid", "many_pyramids", "joint_grid", "spinner", "sleep",
    "trees25", "trees50", "trees100", "washer", "convex_pile",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("before", type=Path)
    parser.add_argument("after", type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenes", nargs="+", choices=SCENES, default=SCENES)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    executables = {"before": args.before.resolve(), "after": args.after.resolve()}
    for executable in executables.values():
        if not executable.is_file():
            parser.error(f"executable not found: {executable}")
    args.output.mkdir(parents=True, exist_ok=True)
    results = {"repeats": args.repeats, "scenes": {}}
    env = os.environ.copy()
    # Use each binary's production default, independent of the caller's solver override.
    env.pop("NATSEL_WIDE", None)
    env.pop("NATSEL_PHASE", None)
    env["NATSEL_HASH"] = "1"
    for scene in args.scenes:
        data = {"before_ms": [], "after_ms": [], "after_hashes": []}
        results["scenes"][scene] = data
        for repeat in range(args.repeats):
            # Reverse every second pair to reduce ordering and thermal bias.
            order = ["before", "after"] if repeat % 2 == 0 else ["after", "before"]
            for label in order:
                print(f"{scene} {repeat + 1}/{args.repeats} {label}", flush=True)
                run = subprocess.run(
                    [str(executables[label]), f"-b={scene}", "-r=1"],
                    env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    check=True,
                )
                (args.output / f"{scene}-{label}-{repeat + 1}.log").write_text(run.stdout)
                match = re.search(r"\.csv: threads,ms 1,([0-9.]+)", run.stdout)
                if match is None:
                    raise RuntimeError(f"missing timing for {scene} ({label})")
                data[f"{label}_ms"].append(float(match.group(1)))
                if label == "after":
                    data["after_hashes"].extend(re.findall(r"^state: ([0-9a-f]+)$", run.stdout, re.M))
                print(f"  {float(match.group(1)):.3f} ms", flush=True)
                (args.output / "comparison.json").write_text(json.dumps(results, indent=2) + "\n")
        data["speedup"] = min(data["before_ms"]) / min(data["after_ms"])
        print(f"{scene}: {data['speedup']:.3f}x", flush=True)
    results["total_before_ms"] = sum(min(d["before_ms"]) for d in results["scenes"].values())
    results["total_after_ms"] = sum(min(d["after_ms"]) for d in results["scenes"].values())
    results["speedup"] = results["total_before_ms"] / results["total_after_ms"]
    (args.output / "comparison.json").write_text(json.dumps(results, indent=2) + "\n")
    print(f"Total: {results['speedup']:.3f}x", flush=True)


if __name__ == "__main__":
    main()
