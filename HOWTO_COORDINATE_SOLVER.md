# How to run the coordinate-based Gurobi solver

This guide covers the alternative model in `gurobi_coordinate_solver.py`.
Unlike `gurobi_pallet_solver.py`, it uses individual-box coordinates and
pairwise non-overlap directions rather than enumerating every possible box
placement.

## 1. Activate the WSL environment

Open a WSL terminal and run:

```bash
cd ~/MOOCamp
source .venv/bin/activate
```

If the dependencies have not been installed yet:

```bash
python -m pip install -r requirements.txt
```

Check that Gurobi can find a license:

```bash
python -c "import gurobipy as gp; gp.Model(); print('Gurobi is available')"
```

## 2. Run the included example

```bash
python gurobi_coordinate_solver.py \
  --input input/gurobi_small_exact.json \
  --config configs/gurobi_coordinate_default.json \
  --output-dir output/gurobi_coordinate \
  --max-pallets 1 \
  --time-limit 30 \
  --print-model
```

`--max-pallets 1` keeps the included six-box example small enough for the
currently installed size-limited Gurobi license. Instances requiring several
pallets will generally require an unrestricted academic or WLS license.

## 3. Run all benchmark instances

The batch mode processes every matching JSON file in lexical order. Each
instance receives a directory named after its JSON filename:

```bash
python gurobi_coordinate_solver.py \
  --input-dir tests/instances \
  --pattern "pl*.json" \
  --config configs/gurobi_coordinate_default.json \
  --output-dir output/gurobi_coordinate_batch \
  --time-limit 60
```

If `--max-pallets` is omitted, the solver reads the heuristic upper bound from
each PL-100 instance and uses two or three candidate pallets as appropriate.
Supplying `--max-pallets` overrides this behavior for every instance.

Results are stored as:

```text
output/gurobi_coordinate_batch/
  batch_summary.csv
  batch_summary.json
  pl001_n010_H900_B2_LB2UB2/
    solution_000_p2.csv
    solution_000_p2.json
    solution_000_p2.html
  pl002_n010_H900_2_LB2UB3/
    ...
```

If an instance fails, the batch continues and writes `error.json` in that
instance directory. The combined summaries contain the status and error. Use
`--fail-fast` if the run should stop after the first failure.

The installed size-limited Gurobi license is not large enough for most of these
two- and three-pallet models. Batch iteration and error reporting still work,
but computing the benchmark solutions requires an unrestricted academic or WLS
license.

## 4. Result files

The command writes the following files under `output/gurobi_coordinate/`:

- `summary.csv`: solver status, pallet count, utilization, bound, gap, and runtime;
- `solution_000_pN.csv`: one row per placed box;
- `solution_000_pN.json`: metrics and placements in machine-readable form;
- `solution_000_pN.html`: interactive Plotly visualization.

The program checks pallet boundaries, three-dimensional non-overlap, and the
required support fraction again after extracting the Gurobi solution.

## 5. Input JSON

The solver accepts both the MCPP `items` schema used by
`gurobi_pallet_solver.py` and the PL-100 `boxes` schema under
`tests/instances`. Pallet and box dimensions are expressed in millimetres. An
MCPP item minimally needs:

```json
{
  "id": 1,
  "sku": 10,
  "width": 600,
  "depth": 400,
  "height": 300
}
```

Optional fields such as `weight_kg`, `family`, `is_food`, `is_chemical`,
`fragile`, `upright_only`, and `retrieval_priority` are preserved in the result.
They are reserved for later model extensions.

## 6. Important configuration fields

Edit `configs/gurobi_coordinate_default.json` or provide another config file:

- `grid_mm`: coordinate resolution; smaller values are more accurate but harder;
- `max_items`: input-size guard;
- `max_pallets`: number of candidate pallets;
- `time_limit_seconds`: Gurobi time limit;
- `mip_gap`: requested relative optimality gap;
- `rotation_mode`: `none`, `yaw`, `metadata`, or `six`;
- `support.mode`: `off`, `direct`, `fraction`, or `full`;
- `support.minimum_fraction`: `0.75` for the project requirement;
- `symmetry`: enables the implemented symmetry-breaking constraints.

The current objective is only the number of used pallets. Accessibility,
weight placement, and food/chemical rules are not active yet.

## 7. Run from PyCharm

Create a Python run configuration with:

- script: `/home/andrei_rotaru/MOOCamp/gurobi_coordinate_solver.py`;
- working directory: `/home/andrei_rotaru/MOOCamp`;
- interpreter: `/home/andrei_rotaru/MOOCamp/.venv/bin/python`;
- parameters:

```text
--input input/gurobi_small_exact.json --config configs/gurobi_coordinate_default.json --output-dir output/gurobi_coordinate --max-pallets 1 --time-limit 30 --print-model
```

## 8. Useful optional commands

Show all arguments:

```bash
python gurobi_coordinate_solver.py --help
```

Export the generated MILP for inspection:

```bash
python gurobi_coordinate_solver.py \
  --input input/gurobi_small_exact.json \
  --config configs/gurobi_coordinate_default.json \
  --output-dir output/gurobi_coordinate \
  --max-pallets 1 \
  --write-lp output/gurobi_coordinate/model.lp
```

Run the tests:

```bash
python -m unittest tests.test_gurobi_coordinate_solver
python -m unittest discover -s tests -p "test_*.py"
```
