"""Run every test-set JSON instance into output/instances/<instance-name>."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--instances", default="tests/instances")
    parser.add_argument("--output", default="output/instances")
    parser.add_argument("--config", default="configs/gurobi_test_instances.json")
    parser.add_argument("--instance-timeout", type=float, default=120.0)
    parser.add_argument("--time-limit", type=float, default=None)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args()

    root = Path(args.instances)
    output_root = Path(args.output)
    output_root.mkdir(parents=True, exist_ok=True)
    report_path = output_root / "batch_summary.csv"
    rows = []

    for input_path in sorted(root.glob("*.json")):
        name = input_path.stem
        instance_output = output_root / name
        summary_path = instance_output / "pareto_front.csv"
        if args.resume and summary_path.exists():
            rows.append({"instance": name, "status": "resumed_existing", "solutions": ""})
            continue

        command = [
            sys.executable,
            "gurobi_pallet_solver_grid_based/gurobi_pallet_solver_grid_based.py",
            "--input", str(input_path),
            "--config", args.config,
            "--output-dir", str(instance_output),
        ]
        if args.time_limit is not None:
            command.extend(["--time-limit", str(args.time_limit)])

        started = time.perf_counter()
        status = "completed"
        error = ""
        try:
            completed = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=args.instance_timeout,
                check=False,
            )
            if completed.returncode != 0:
                status = "failed"
                error = (completed.stderr or completed.stdout)[-1000:].replace("\n", " ")
        except subprocess.TimeoutExpired as exc:
            status = "timed_out"
            error = f"instance timeout after {args.instance_timeout:g}s"
            if exc.stderr:
                error += " " + str(exc.stderr)[-500:].replace("\n", " ")

        solutions = ""
        if summary_path.exists():
            try:
                with summary_path.open(newline="", encoding="utf-8") as handle:
                    solutions = sum(1 for _ in csv.DictReader(handle))
            except OSError:
                pass
        rows.append({
            "instance": name,
            "status": status,
            "solutions": solutions,
            "wall_seconds": round(time.perf_counter() - started, 3),
            "error": error,
        })
        print(f"{name}: {status}, solutions={solutions}", flush=True)

    with report_path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = ["instance", "status", "solutions", "wall_seconds", "error"]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"Wrote {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
