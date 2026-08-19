"""Canonical entry point for the grid-based Gurobi pallet solver."""

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from gurobi_pallet_solver import main  # noqa: E402


if __name__ == "__main__":
    raise SystemExit(main())
