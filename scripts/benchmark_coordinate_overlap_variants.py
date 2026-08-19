"""Benchmark exact overlap formulations in the coordinate-based pallet MILP."""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from collections import Counter
from pathlib import Path

from gurobipy import GRB

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from gurobi_coordinate_solver import CoordinateBasedMILP, read_mcpp_json, status_name


FIELDS = [
    "instance", "n_boxes", "variant", "overlap_formulation", "area_auxiliary_type",
    "variables", "binary_variables", "integer_variables", "continuous_variables",
    "linear_constraints", "general_constraints", "quadratic_constraints", "nonzeros",
    "build_seconds", "gurobi_runtime_seconds", "total_wall_seconds", "status",
    "solution_count", "pallet_count", "objective_bound", "mip_gap",
]

VARIANTS = [
    ("compact_continuous", "compact", "continuous"),
    ("lookup_1d", "lookup_1d", "continuous"),
    ("lookup_2d", "lookup_2d", "continuous"),
    ("compact_integer_area", "compact", "integer"),
]


def write_results(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("inputs", nargs="+", type=Path)
    args = parser.parse_args()

    base_config = json.loads(args.config.read_text(encoding="utf-8"))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []

    for input_path in args.inputs:
        for variant, overlap_formulation, area_type in VARIANTS:
            total_started = time.perf_counter()
            config = dict(base_config)
            config["overlap_formulation"] = overlap_formulation
            config["area_auxiliary_type"] = area_type
            context, items = read_mcpp_json(input_path, config)

            build_started = time.perf_counter()
            exact = CoordinateBasedMILP(context, items, config)
            exact.model.update()
            build_seconds = time.perf_counter() - build_started
            model = exact.model
            types = Counter(variable.VType for variable in model.getVars())

            model.optimize()
            solution_count = int(model.SolCount)
            status = status_name(model.Status)
            pallet_count: int | str = ""
            objective_bound: float | str = ""
            mip_gap: float | str = ""
            if solution_count:
                pallet_count = int(round(model.ObjVal))
                objective_bound = float(model.ObjBound)
                mip_gap = float(model.MIPGap)
            elif model.Status in {GRB.TIME_LIMIT, GRB.INTERRUPTED}:
                objective_bound = float(model.ObjBound)

            row: dict[str, object] = {
                "instance": input_path.stem,
                "n_boxes": len(items),
                "variant": variant,
                "overlap_formulation": overlap_formulation,
                "area_auxiliary_type": area_type,
                "variables": model.NumVars,
                "binary_variables": types.get(GRB.BINARY, 0),
                "integer_variables": types.get(GRB.INTEGER, 0),
                "continuous_variables": types.get(GRB.CONTINUOUS, 0),
                "linear_constraints": model.NumConstrs,
                "general_constraints": model.NumGenConstrs,
                "quadratic_constraints": model.NumQConstrs,
                "nonzeros": model.NumNZs,
                "build_seconds": round(build_seconds, 6),
                "gurobi_runtime_seconds": round(float(model.Runtime), 6),
                "total_wall_seconds": round(time.perf_counter() - total_started, 6),
                "status": status,
                "solution_count": solution_count,
                "pallet_count": pallet_count,
                "objective_bound": objective_bound,
                "mip_gap": mip_gap,
            }
            rows.append(row)
            write_results(args.output, rows)
            print(
                f"{input_path.stem} | {variant} | vars={model.NumVars} "
                f"constrs={model.NumConstrs}+{model.NumGenConstrs} | {status} "
                f"| pallets={pallet_count} | wall={row['total_wall_seconds']}s",
                flush=True,
            )
            model.dispose()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
