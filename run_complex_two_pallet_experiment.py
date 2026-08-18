"""Run and render a complex, CQM-feasible two-Euro-pallet experiment.

The total box volume is greater than one 120 x 80 x 180 cm pallet, making two
pallets necessary even before geometry, stability, or product rules are
considered. The deterministic layout is a feasibility baseline, not a
solver-optimized solution.
"""

from pathlib import Path

from packing3d import Bins, Cases, Variables, build_cqm
from utils import plot_cuboids, read_instance, write_solution_to_file


ROOT = Path(__file__).parent
INPUT = ROOT / "input" / "complex_two_pallet_experiment.txt"
OUTPUT = ROOT / "output"
OUTPUT.mkdir(exist_ok=True)

data = read_instance(str(INPUT))
cases = Cases(data)
bins = Bins(data, cases)
variables = Variables(cases, bins)
cqm, effective_dimensions = build_cqm(variables, bins, cases)

# Coordinates use the upstream model's global x-axis: pallet 1 occupies
# 0 <= x <= 120 and pallet 2 occupies 120 <= x <= 240.
positions = [
    # Pallet 1, base layer: four 60 x 40 x 50 boxes.
    (0, 0, 0),
    (60, 0, 0),
    (0, 40, 0),
    (60, 40, 0),
    # Pallet 1, upper layer: two 60 x 80 x 40 boxes.
    (0, 0, 50),
    (60, 0, 50),
    # Pallet 2, base layer: two 120 x 40 x 60 boxes.
    (120, 0, 0),
    (120, 40, 0),
    # Pallet 2, middle layer: eight 30 x 40 x 30 boxes.
    (120, 0, 60),
    (150, 0, 60),
    (180, 0, 60),
    (210, 0, 60),
    (120, 40, 60),
    (150, 40, 60),
    (180, 40, 60),
    (210, 40, 60),
    # Pallet 2, top layer: four 60 x 40 x 20 boxes.
    (120, 0, 90),
    (180, 0, 90),
    (120, 40, 90),
    (180, 40, 90),
]

sizes = (
    [(60, 40, 50)] * 4
    + [(60, 80, 40)] * 2
    + [(120, 40, 60)] * 2
    + [(30, 40, 30)] * 8
    + [(60, 40, 20)] * 4
)
assignments = [0] * 6 + [1] * 14

if not (len(positions) == len(sizes) == cases.num_cases):
    raise RuntimeError("Experiment coordinates do not match the expanded input cases")

sample = {label: 0 for label in cqm.variables}

for i, (x, y, z) in enumerate(positions):
    sample[f"o_{i}_0"] = 1
    sample[f"x_{i}"] = x
    sample[f"y_{i}"] = y
    sample[f"z_{i}"] = z
    if i > 0:
        for j in range(bins.num_bins):
            sample[f"case_{i}_in_bin_{j}"] = int(j == assignments[i])

sample["bin_1_is_used"] = 1
sample["upper_bound_0"] = 90
sample["upper_bound_1"] = 110

# Select one valid geometric separation for each pair. This also proves that
# no two boxes assigned to the same pallet overlap.
for i in range(len(positions)):
    xi, yi, zi = positions[i]
    dxi, dyi, dzi = sizes[i]
    for k in range(i + 1, len(positions)):
        xk, yk, zk = positions[k]
        dxk, dyk, dzk = sizes[k]
        if xi + dxi <= xk:
            relation = 0
        elif yi + dyi <= yk:
            relation = 1
        elif zi + dzi <= zk:
            relation = 2
        elif xk + dxk <= xi:
            relation = 3
        elif yk + dyk <= yi:
            relation = 4
        elif zk + dzk <= zi:
            relation = 5
        else:
            raise RuntimeError(f"Boxes {i} and {k} overlap")
        sample[f"sel_{i}_{k}_{relation}"] = 1

if not cqm.check_feasible(sample):
    raise RuntimeError("The complex two-pallet arrangement violates the CQM")

figure = plot_cuboids(
    sample,
    variables,
    cases,
    bins,
    effective_dimensions,
    color_coded=True,
)
figure.update_layout(
    title=(
        "Complex Euro-pallet experiment: 20 boxes, five sizes, "
        "two volume-required pallets"
    )
)

html_path = OUTPUT / "complex_two_pallet_experiment.html"
solution_path = OUTPUT / "complex_two_pallet_experiment.txt"
figure.write_html(html_path)
write_solution_to_file(
    str(solution_path),
    cqm,
    variables,
    sample,
    cases,
    bins,
    effective_dimensions,
)

box_volume = sum(dx * dy * dz for dx, dy, dz in sizes)
pallet_volume = bins.length * bins.width * bins.height
print(f"CQM feasible: yes")
print(f"Boxes packed: {cases.num_cases}")
print(f"Pallets used: 2 (volume lower bound: {bins.lowest_num_bin})")
print(f"Total box volume: {box_volume:,} cm^3")
print(f"One-pallet capacity: {pallet_volume:,} cm^3")
print(f"Two-pallet volume utilization: {100 * box_volume / (2 * pallet_volume):.2f}%")
print("Packed heights: pallet 1 = 90 cm, pallet 2 = 110 cm")
print("Raised-box support: 100% on both pallets")
print(f"Wrote {html_path.relative_to(ROOT)}")
print(f"Wrote {solution_path.relative_to(ROOT)}")
