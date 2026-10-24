#!/usr/bin/env python3
"""Compare the ten Swift scenes with a C reference built using reference.c, including hashes."""

import argparse
import json
import os
from pathlib import Path
import re
import subprocess

from compare import SCENES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("c", type=Path)
    parser.add_argument("swift", type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--scenes", nargs="+", choices=SCENES, default=SCENES)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    binaries = {"c": args.c.resolve(), "swift": args.swift.resolve()}
    for binary in binaries.values():
        if not binary.is_file():
            parser.error(f"executable not found: {binary}")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    # Tracing, CCD probes, and solver overrides must not silently change either workload.
    env = {key: value for key, value in os.environ.items() if not key.startswith("NATSEL_")}
    env["NATSEL_HASH"] = "1"
    env["NATSEL_PHASE"] = "1"
    result = {
        "repeats": args.repeats,
        "target_ratio": 1.10,
        "binaries": {key: str(value) for key, value in binaries.items()},
        "scenes": {},
    }

    def save():
        (output / "validation.json").write_text(json.dumps(result, indent=2) + "\n")

    for scene in args.scenes:
        row = {"c_ms": [], "swift_ms": [], "c_hashes": [], "swift_hashes": [], "swift_phases": []}
        result["scenes"][scene] = row
        for repeat in range(args.repeats):
            order = ["c", "swift"] if repeat % 2 == 0 else ["swift", "c"]
            for label in order:
                command = [str(binaries[label]), f"-b={scene}", "-r=1"]
                if label == "c":
                    command.extend(["-t=1", "-w=1", "-simd=4"])
                run = subprocess.run(
                    command, cwd=output, env=env, text=True,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                )
                log = output / f"{scene}-{label}-{repeat + 1}.log"
                log.write_text(run.stdout)
                if run.returncode:
                    raise RuntimeError(f"benchmark exited {run.returncode}; see {log}")
                timing = re.search(r"^run 0\s*:\s*([0-9.eE+-]+)\s*\(ms\)", run.stdout, re.M)
                hashes = re.findall(r"^state: ([0-9a-f]+)$", run.stdout, re.M)
                if timing is None or len(hashes) != 1:
                    raise RuntimeError(f"missing timing or unique state hash: {scene} {label}")
                row[f"{label}_ms"].append(float(timing.group(1)))
                row[f"{label}_hashes"].append(hashes[0])
                if label == "swift":
                    phase = next((line for line in run.stdout.splitlines() if line.startswith("AVG ")), "")
                    row["swift_phases"].append({
                        key: float(value) for key, value in re.findall(r"([A-Za-z]+)=([0-9.]+)", phase)
                    })
                save()
                print(f"{scene} {repeat + 1}/{args.repeats} {label}: {timing.group(1)} ms", flush=True)
        row["hashes_match"] = len(set(row["c_hashes"] + row["swift_hashes"])) == 1
        row["ratio"] = min(row["swift_ms"]) / min(row["c_ms"])
        row["within_target"] = row["ratio"] <= result["target_ratio"]
        save()
        print(f"{scene}: {row['ratio']:.3f}x; hashes match: {row['hashes_match']}", flush=True)
        if not row["hashes_match"]:
            raise RuntimeError(f"C/Swift state mismatch in {scene}; see saved logs")

    rows = result["scenes"].values()
    result["total_c_ms"] = sum(min(row["c_ms"]) for row in rows)
    result["total_swift_ms"] = sum(min(row["swift_ms"]) for row in rows)
    result["total_ratio"] = result["total_swift_ms"] / result["total_c_ms"]
    result["all_hashes_match"] = all(row["hashes_match"] for row in rows)
    result["all_scenes_within_target"] = all(row["within_target"] for row in rows)
    result["aggregate_within_target"] = result["total_ratio"] <= result["target_ratio"]
    save()
    print(f"Total: {result['total_ratio']:.3f}x", flush=True)


if __name__ == "__main__":
    main()
