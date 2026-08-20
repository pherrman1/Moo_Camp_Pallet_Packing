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

`weight_kg` is used by the pallet-payload and support-mass constraints.
`is_food` and `is_chemical` (or the equivalent `group` value) are used when
the optional food/chemical ordering rule is active; the remaining metadata is
preserved for later model extensions.

## 6. Important configuration fields

Edit `configs/gurobi_coordinate_default.json` or provide another config file:

- `grid_mm`: coordinate resolution; smaller values are more accurate but harder;
- `max_items`: input-size guard;
- `max_pallets`: number of candidate pallets;
- `fixed_pallet_count`: optional exact number of nonempty pallets; it must be
  between the volume lower bound and `max_pallets`;
- `time_limit_seconds`: Gurobi time limit;
- `mip_gap`: requested relative optimality gap;
- `objective_mode`: `pallet_count_only`, `pallets_then_max_height`,
  `pallets_then_average_height`,
  `category_distance_only`, or `category_distance_then_max_height`;
- `rotation_mode`: `none`, `yaw`, `metadata`, or `six`;
- `support.mode`: `off`, `direct`, `fraction`, or `full`;
- `support.minimum_fraction`: `0.75` for the project requirement;
- `food_chemical.mode`: `off` or `chemical_below_food`; the latter requires
  every chemical box's top coordinate to be at or below the base coordinate of
  every food box assigned to the same pallet;
- `support_area_objective.enabled`: when `true`, maximize the exact sum of
  box-on-box support areas as a third lexicographic stage after optimal pallet
  count and optimal maximum height; this requires `support.mode` to be
  `fraction` or `full`;
- `warm_start.greedy`: opt in to a fast chemical-first height-map packing
  supplied to the reduced-exact model as a partial Gurobi MIP start;
- `symmetry`: enables the implemented symmetry-breaking constraints.

The warm start is disabled when the field is omitted. To enable it without
changing the mathematical model, add:

```json
"warm_start": {
  "greedy": true
}
```

If the heuristic cannot pack every box within `max_pallets`, the solver simply
continues without a user MIP start. The legacy model variant does not use this
option.

The heuristic first packs all chemical boxes using first-fit pallets, then all
food boxes, and finally boxes in neither category. Inside each category it
starts from the heaviest boxes. Food positions supported directly by chemical
boxes are preferred over unused floor space. Every direct supporter must be at
least as heavy as the box placed above it, and every candidate is checked for
containment, height, payload, non-overlap, configured support fraction, the
chemical-below-food rule, and symmetry compatibility before it is submitted to
Gurobi. Boxes in neither category are included in the last phase so the MIP
start always covers the complete instance.

The default objective is lexicographic: first minimize used pallets and then
minimize the maximum occupied height. The independent `category_distance_only`
mode instead minimizes distances between boxes with the same
`retrieval_priority` (benchmark `priority`). For two same-category boxes on one
pallet, their contribution is the L1 distance between their oriented box
centers. If they are on different pallets, the contribution is the fixed
penalty `pallet_length + pallet_width + pallet_height + 1`; different-category
pairs contribute zero. Distances and the `1` epsilon are in grid units.
`category_distance_then_max_height` first proves and fixes that minimum distance
sum, then minimizes maximum occupied height without degrading the distance
objective.

The result visualization uses the priority category as the box hue. Distinct
SKUs/types inside one priority category use lighter shades of the same hue;
food and chemical boxes retain their `F` and `C` text labels.

When support-area maximization is enabled, it follows the selected distance
objective after fixing the proven distance optimum. In the default height mode,
support area still follows fixed optimal pallet count and height. Pallet count
and height are not optimized in `category_distance_only` mode.

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
