"""Sequentially compare full-grid, subset-sum, and coordinate Gurobi solvers."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path


FIELDS = [
    "instance", "n_boxes", "input_grid_mm", "solver", "status", "exit_code",
    "pallet_count", "objective_bound", "mip_gap", "reported_runtime_seconds",
    "wall_seconds", "output_directory", "error",
]


def read_result(solver: str, output_dir: Path) -> dict[str, str]:
    if solver != "coordinate_based":
        path = output_dir / "pareto_front.csv"
        if not path.exists():
            return {}
        with path.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
        if not rows:
            return {}
        row = min(rows, key=lambda value: int(value["pallet_count"]))
        return {
            "pallet_count": row["pallet_count"],
            "objective_bound": "",
            "mip_gap": row["mip_gap"],
            "reported_runtime_seconds": row["runtime_seconds"],
        }

    path = output_dir / "summary.csv"
    if not path.exists():
        return {}
    with path.open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return {}
    row = rows[0]
    return {
        "pallet_count": row["pallet_count"],
        "objective_bound": row["objective_bound"],
        "mip_gap": row["mip_gap"],
        "reported_runtime_seconds": row["runtime_seconds"],
    }


def write_summary(path: Path, records: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(records)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--experiment", required=True, type=Path)
    parser.add_argument("--instance-timeout", type=float, default=120.0)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--max-instances", type=int, default=None)
    args = parser.parse_args()

    experiment = args.experiment.resolve()
    input_dir = experiment / "inputs"
    summary_path = experiment / "comparison_summary.csv"
    records: list[dict[str, object]] = []
    if args.resume and summary_path.exists():
        with summary_path.open(newline="", encoding="utf-8") as handle:
            records = list(csv.DictReader(handle))
    completed = {(row["instance"], row["solver"]) for row in records}

    solvers = [
        (
            "full_grid",
            Path("gurobi_pallet_solver_grid_based/gurobi_pallet_solver_grid_based.py"),
            experiment / "configs" / "grid_based.json",
            ["--placement-domain", "full_grid"],
        ),
        (
            "subset_sum",
            Path("gurobi_pallet_solver_grid_based/gurobi_pallet_solver_grid_based.py"),
            experiment / "configs" / "grid_based.json",
            ["--placement-domain", "subset_sum"],
        ),
        (
            "coordinate_based",
            Path("gurobi_coordinate_solver.py"),
            experiment / "configs" / "coordinate_based.json",
            [],
        ),
    ]

    input_paths = sorted(input_dir.glob("*.json"))
    if args.max_instances is not None:
        input_paths = input_paths[:args.max_instances]
    for input_path in input_paths:
        payload = json.loads(input_path.read_text(encoding="utf-8"))
        instance = input_path.stem
        n_boxes = len(payload.get("boxes", payload.get("items", [])))
        input_grid = payload.get("meta", {}).get("grid_mm", "")

        for solver, entrypoint, config_path, extra_args in solvers:
            if args.resume and (instance, solver) in completed:
                continue
            output_dir = experiment / "results" / solver / instance
            log_dir = experiment / "logs" / solver
            output_dir.mkdir(parents=True, exist_ok=True)
            log_dir.mkdir(parents=True, exist_ok=True)
            command = [
                sys.executable,
                str(entrypoint),
                "--input", str(input_path),
                "--config", str(config_path),
                "--output-dir", str(output_dir),
            ] + extra_args
            started = time.perf_counter()
            status = "completed"
            exit_code: int | str = ""
            error = ""
            stdout = ""
            stderr = ""
            try:
                completed_process = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=args.instance_timeout,
                    check=False,
                )
                exit_code = completed_process.returncode
                stdout, stderr = completed_process.stdout, completed_process.stderr
                if completed_process.returncode != 0:
                    status = "failed"
                    error = (stderr or stdout)[-1000:].replace("\n", " ")
            except subprocess.TimeoutExpired as exc:
                status = "timed_out"
                error = f"external timeout after {args.instance_timeout:g}s"
                stdout = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
                stderr = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")

            (log_dir / f"{instance}.log").write_text(
                stdout + ("\n--- STDERR ---\n" + stderr if stderr else ""),
                encoding="utf-8",
            )
            result = read_result(solver, output_dir)
            record: dict[str, object] = {
                "instance": instance,
                "n_boxes": n_boxes,
                "input_grid_mm": input_grid,
                "solver": solver,
                "status": status,
                "exit_code": exit_code,
                "pallet_count": result.get("pallet_count", ""),
                "objective_bound": result.get("objective_bound", ""),
                "mip_gap": result.get("mip_gap", ""),
                "reported_runtime_seconds": result.get("reported_runtime_seconds", ""),
                "wall_seconds": round(time.perf_counter() - started, 3),
                "output_directory": str(output_dir),
                "error": error,
            }
            records.append(record)
            write_summary(summary_path, records)
            print(
                f"{instance} | {solver} | {status} | pallets={record['pallet_count']} "
                f"| wall={record['wall_seconds']}s",
                flush=True,
            )

    print(f"Comparison complete: {len(records)} solver attempts; wrote {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
