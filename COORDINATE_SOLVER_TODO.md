# Coordinate-based Gurobi solver

The alternative solver is `gurobi_coordinate_solver.py`. It implements the
individual-box coordinate MILP developed in the project discussions rather
than the position-indexed placement formulation in `gurobi_pallet_solver.py`.

## Run in WSL

```bash
cd ~/MOOCamp
source .venv/bin/activate

python gurobi_coordinate_solver.py \
  --input input/gurobi_small_exact.json \
  --config configs/gurobi_coordinate_default.json \
  --output-dir output/gurobi_coordinate \
  --print-model
```

For a quick test with the size-limited Gurobi license, use only one candidate
pallet and a short time limit:

```bash
python gurobi_coordinate_solver.py \
  --input input/gurobi_small_exact.json \
  --config configs/gurobi_coordinate_default.json \
  --output-dir output/gurobi_coordinate_smoke \
  --max-pallets 1 \
  --time-limit 30 \
  --print-model
```

The default result directory is `output/gurobi_coordinate`. A run writes a CSV,
JSON, and interactive Plotly HTML solution plus `summary.csv`. Use `--write-lp
output/gurobi_coordinate/model.lp` to inspect the generated MILP.

## Implemented scope

- individual-box pallet assignments;
- box-dependent orthogonal orientations;
- integer coordinates on a configurable grid;
- pallet boundaries and a volume lower bound;
- pairwise six-direction non-overlap for every box pair and pallet;
- exact horizontal-overlap indicators;
- direct-contact support and an exact 75% union support-area rule on the grid;
- pallet and identical-box symmetry breaking;
- one objective: minimize the number of used pallets.

## JSON interface

The solver accepts the same MCPP JSON structure as the existing exact solver.
Each item needs `width`, `depth`, and `height` in millimetres. The existing
metadata fields (`weight_kg`, `family`, `is_food`, `is_chemical`, `fragile`,
`upright_only`, and `retrieval_priority`) are preserved in result files.

## TODO

- Add the optional blocking variable and retrieval-accessibility objective.
- Add hard/soft food-versus-chemical vertical rules.
- Add the weight or density vertical-moment objective.
- Add box compression/load-bearing limits and dynamic-stability extensions.
- Add warm starts and stronger valid inequalities for larger instances.
- Compare model size and runtime systematically with the position-indexed model.

The coordinate model grows approximately quadratically in the number of boxes
and is intended as an exact benchmark for small instances, not as the final
method for deliveries containing roughly 1,000 boxes.
