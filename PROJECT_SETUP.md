# Project setup

This project uses [D-Wave's 3D bin-packing example](https://github.com/dwave-examples/3d-bin-packing) as its optimization baseline. The upstream CQM handles cuboid dimensions, six axis-aligned orientations, non-overlap, bin assignment, bin boundaries, packed height, and the number of bins used.

## Install

The repository includes the upstream `requirements.txt`. From PowerShell, activate the existing virtual environment and install it:

```powershell
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

The model is designed to solve through D-Wave Leap. The inherited SciPy/HiGHS fallback remains available for compatibility, but SciPy's MILP solver cannot solve the quadratic non-overlap constraints in this full 3D formulation; use it only when experimenting with a linearized variant.

## Run the pallet demo

The default input is the small Euro-pallet instance in `input/euro_pallet_demo.txt`:

```powershell
python packing3d.py --time_limit 20 --output_filepath output/euro_pallet_demo.txt --html_filepath output/euro_pallet_demo.html
```

Configure the D-Wave credentials before running the command. A successful run writes a text solution and an interactive Plotly HTML visualization.

The Dash interface remains available with `python app.py`; open `http://127.0.0.1:8050/` in a browser.

## Scope and modeling assumptions

The supplied instance uses centimetres, three identical Euro pallets (`120 x 80 x 180`), and a deliberately small set of cuboid box types so that the model can be inspected locally. Boxes may be rotated among all six axis-aligned orientations. The parser expects the standard two metadata lines followed immediately by the column header, so keep custom input files in the same format as the supplied instance.

The inherited upstream formulation does **not** yet enforce the assignment's 75% support requirement, weight/load-bearing order, food-over-chemicals rule, or retrieval accessibility. Those are the next modeling extensions: they require item metadata and additional support, precedence, and accessibility variables/constraints. The current setup therefore provides a tested geometric and pallet-allocation baseline rather than claiming to solve every operational requirement.

## Tests

Run the upstream unit tests with:

```powershell
python -m unittest discover -s tests -p "test_*.py"
```

The source and `LICENSE` are retained from the upstream Apache-2.0 project; project-specific configuration is in `input/euro_pallet_demo.txt` and this file.

## Generate realistic MCPP inputs

The benchmark generator can now export the exact text format consumed by `packing3d.py`. Dimensions remain in integer millimetres, preserving the generator's geometric precision, while repeated SKUs are grouped through the solver's `quantity` column.

```powershell
python generator/mcpp_generator.py single --class 4 --items 20 --seed 7 --pallet euro-180 --formats solver,json --solver-bins 2 --outdir input/generated
```

This writes `*_solver.txt` for the geometric solver and a companion JSON file containing weight, family, chemical/food, fragility, permitted-orientation, zone, and retrieval-priority metadata. Run the generated geometric instance with:

```powershell
python packing3d.py --data_filepath input/generated/mcpp_c4_n0020_s7_solver.txt --time_limit 20
```

`--solver-bins` sets the maximum number of available pallets and may not be smaller than the generated instance's volume lower bound. If omitted, the exporter uses that lower bound.
