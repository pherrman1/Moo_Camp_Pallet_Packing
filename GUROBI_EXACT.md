# Exact Gurobi pallet solver

`gurobi_pallet_solver.py` is a position-indexed mixed-integer linear model for
small MCPP JSON instances. It is exact relative to the configured spatial grid:
every legal yaw orientation and every generated grid position is represented by
a binary placement variable. Dimensions that are not multiples of the grid are
rounded upward, so the discretization is conservative.

## Run

From PowerShell:

```powershell
& .\.venv\Scripts\python.exe gurobi_pallet_solver.py `
  --input input/gurobi_small_exact.json `
  --config configs/gurobi_exact_default.json `
  --output-dir output/gurobi_small_exact
```

The installation requires Gurobi and a working local, WLS, or other Gurobi
license. `gurobipy~=12.0` is included in `requirements.txt`.

## Objective hierarchy and Pareto front

Total height is deliberately not an objective. The solver uses nested epsilon
constraints instead of a single weighted sum, so unsupported points on a
discrete, non-convex Pareto front are not skipped:

1. Enumerate every feasible exact pallet count from the volume lower bound to
   `max_pallets`; pallet count is therefore an outer epsilon parameter.
2. For each pallet count, minimize pallet-height spread and fix that optimum.
3. Find the minimum-accessibility and minimum-vertical-moment endpoints.
4. For every integer accessibility bound between those endpoints, minimize the
   selected vertical moment.
5. Remove globally dominated and duplicate solutions in the four criteria:
   pallet count, height spread, accessibility inversions, and vertical moment.

This means height spread is lexicographically subordinate to pallet count for
each outer subproblem, while accessibility and the selected vertical moment form
the remaining two-objective frontier. A solution using an additional pallet is
retained only when it improves at least one other criterion enough to remain
nondominated.

## Configuration

- `grid_mm`: position and dimension discretization, default 50 mm.
- `max_items`: input safety limit for the exact model.
- `max_pallets`: largest exact pallet-count epsilon to examine; it is not a hard
  instruction to use that many pallets.
- `time_limit_seconds`: time limit for each Gurobi optimization call.
- `mip_gap`: requested relative MIP gap; zero requests proof of optimality.
- `log_to_console`: enables or suppresses Gurobi's detailed solver log.
- `rotation_mode`: `yaw` allows the original footprint and its 90-degree yaw;
  `none` forbids rotation, `metadata` uses item metadata, and `six` permits all
  axis-aligned orientations unless the item is upright-only.
- `support.mode`: `off`, `direct` (some direct contact), `fraction`, or `full`.
- `support.minimum_fraction`: required union contact area for `fraction`, with
  0.75 meaning 75 percent of the upper box's footprint.
- `food_chemical.mode`: `hard_overlap` prevents food and chemicals from being
  vertically aligned on the same pallet; `off` disables it.
- `fragile.cannot_support`: when true, no box may directly rest on a fragile box.
- `vertical_moment.mode`: `density` (default) or `mass`; this changes the actual
  optimized Pareto criterion, not merely the reported label.
- `accessibility.mode`: `soft_count` counts inversions in which a later-priority
  box lies above an overlapping earlier-priority box; `off` disables it.
- `pareto.max_accessibility_epsilon`: optional upper cutoff for accessibility
  epsilon enumeration; `null` enumerates through the vertical-moment endpoint.
- `pareto.render_all_solutions`: render every nondominated packing as HTML when
  true, or only the first solution when false.

## Output

`pareto_front.csv` contains one row per nondominated solution and
`pareto_front.html` plots accessibility against the selected vertical moment,
grouped by pallet count. Each solution also has neutral CSV and JSON placement
data and an interactive Plotly 3D HTML visualization. Placement rows include
the achieved support fraction, allowing the 75 percent rule to be audited.

## Scope and limitations

The model currently supports yaw-only loading by default, exact union support
on the grid, hard food/chemical separation, optional fragile-support rules, and
soft top-retrieval accessibility. Horizontal center-of-mass balance and box
load-bearing capacities are intentionally omitted. Position indexing grows
quickly with pallet dimensions, grid resolution, orientations, and item count;
the model is therefore intended for exact small-instance benchmarking.
