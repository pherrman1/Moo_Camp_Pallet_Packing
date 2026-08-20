"""Benchmark and audit coordinate warm starts without running Gurobi optimization."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from gurobi_coordinate_solver import (
    audit_coordinate_warm_start,
    load_config,
    prepare_unlimited_coordinate_warm_start,
    read_mcpp_json,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", required=True)
    parser.add_argument("--pattern", default="*.json")
    parser.add_argument(
        "--config", default="configs/gurobi_coordinate_test_instances.json"
    )
    parser.add_argument("--output-csv", default=None)
    args = parser.parse_args()

    paths = sorted(Path(args.input_dir).glob(args.pattern))
    if not paths:
        raise ValueError(f"no files match {args.pattern!r} in {args.input_dir}")
    config = load_config(args.config)
    records: list[dict[str, object]] = []

    for path in paths:
        feasible = False
        error = ""
        pallet_count = 0
        context, items = read_mcpp_json(path, config)
        build_started = time.perf_counter()
        placements = prepare_unlimited_coordinate_warm_start(context, items, config)
        build_seconds = time.perf_counter() - build_started
        audit_seconds = 0.0
        try:
            if placements is None:
                raise RuntimeError("heuristic could not construct a complete packing")
            audit_started = time.perf_counter()
            audit_coordinate_warm_start(placements, context, items, config)
            audit_seconds = time.perf_counter() - audit_started
            pallet_count = len({placement.pallet for placement in placements})
            feasible = True
        except Exception as exc:  # Record every benchmark failure and continue.
            error = f"{type(exc).__name__}: {exc}"
        records.append(
            {
                "instance": path.stem,
                "n_items": len(items),
                "n_chemical": sum(item.is_chemical for item in items),
                "n_food": sum(item.is_food for item in items),
                "pallets": pallet_count,
                "build_seconds": build_seconds,
                "audit_seconds": audit_seconds,
                "total_seconds": build_seconds + audit_seconds,
                "feasible": feasible,
                "error": error,
            }
        )
        print(
            f"{path.stem}: feasible={feasible}, pallets={pallet_count}, "
            f"build={build_seconds:.4f}s, audit={audit_seconds:.4f}s"
            + (f", error={error}" if error else "")
        )

    if args.output_csv:
        output = Path(args.output_csv)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(records[0]))
            writer.writeheader()
            writer.writerows(records)

    elapsed_values = sorted(float(record["build_seconds"]) for record in records)
    audit_values = [float(record["audit_seconds"]) for record in records]
    p95_index = max(0, math.ceil(0.95 * len(elapsed_values)) - 1)
    successes = sum(bool(record["feasible"]) for record in records)
    print(
        f"SUMMARY instances={len(records)} feasible={successes} failed={len(records) - successes} "
        f"total={sum(elapsed_values):.4f}s mean={statistics.mean(elapsed_values):.4f}s "
        f"median={statistics.median(elapsed_values):.4f}s "
        f"p95={elapsed_values[p95_index]:.4f}s max={max(elapsed_values):.4f}s "
        f"audit_total={sum(audit_values):.4f}s"
    )
    return 0 if successes == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
