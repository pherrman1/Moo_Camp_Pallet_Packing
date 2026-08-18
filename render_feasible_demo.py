"""Run and render a brief-guided Euro-pallet feasibility experiment.

The experiment uses one 120 x 80 x 180 cm Euro pallet. Four heavy chemical
boxes form a complete base layer and four lighter food boxes form a complete
upper layer. This gives every raised box 100% support, places heavy boxes below
light boxes, places food above chemicals, and leaves the high-priority food
layer directly accessible from the top.
"""

from pathlib import Path

from packing3d import Bins, Cases, Variables, build_cqm
from utils import plot_cuboids, read_instance, write_solution_to_file


ROOT = Path(__file__).parent
INPUT = ROOT / "input" / "euro_pallet_demo.txt"
OUTPUT = ROOT / "output"
OUTPUT.mkdir(exist_ok=True)

data = read_instance(str(INPUT))
cases = Cases(data)
bins = Bins(data, cases)
variables = Variables(cases, bins)
cqm, effective_dimensions = build_cqm(variables, bins, cases)

# Each tuple is (x, y, z). Four 60 x 40 boxes tile the 120 x 80 footprint.
positions = [
    (0, 0, 0),
    (60, 0, 0),
    (0, 40, 0),
    (60, 40, 0),
    (0, 0, 30),
    (60, 0, 30),
    (0, 40, 30),
    (60, 40, 30),
]

sample = {label: 0 for label in cqm.variables}

for i, (x, y, z) in enumerate(positions):
    sample[f"o_{i}_0"] = 1
    sample[f"x_{i}"] = x
    sample[f"y_{i}"] = y
    sample[f"z_{i}"] = z

# The highest occupied point is 30 + 20 = 50 cm.
sample["upper_bound_0"] = 50

# Select one valid non-overlap relation for every pair so that this is a
# feasible CQM sample, not merely a visually plausible arrangement.
sizes = [(60, 40, 30)] * 4 + [(60, 40, 20)] * 4
for i in range(len(positions)):
    xi, yi, zi = positions[i]
    dxi, dyi, dzi = sizes[i]
    for k in range(i + 1, len(positions)):
        xk, yk, zk = positions[k]
        dxk, dyk, dzk = sizes[k]
        if xi + dxi <= xk:
            relation = 0  # i is left of k
        elif yi + dyi <= yk:
            relation = 1  # i is behind k
        elif zi + dzi <= zk:
            relation = 2  # i is below k
        elif xk + dxk <= xi:
            relation = 3  # i is right of k
        elif yk + dyk <= yi:
            relation = 4  # i is in front of k
        elif zk + dzk <= zi:
            relation = 5  # i is above k
        else:
            raise RuntimeError(f"Boxes {i} and {k} overlap")
        sample[f"sel_{i}_{k}_{relation}"] = 1

if not cqm.check_feasible(sample):
    raise RuntimeError("The demonstration arrangement violates the CQM")

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
        "Euro-pallet experiment: heavy chemicals below lighter, "
        "high-priority food (100% support)"
    )
)
figure.write_html(OUTPUT / "euro_pallet_feasible_demo.html")
write_solution_to_file(
    str(OUTPUT / "euro_pallet_feasible_demo.txt"),
    cqm,
    variables,
    sample,
    cases,
    bins,
    effective_dimensions,
)
print("Wrote output/euro_pallet_feasible_demo.html")
print("Wrote output/euro_pallet_feasible_demo.txt")
print("CQM feasible: yes")
print("Pallets used: 1")
print("Packed height: 50 cm of 180 cm")
print("Raised-box support: 100% (requirement: at least 75%)")
print("Full-pallet volume utilization: 27.78%")
print("Occupied-envelope utilization: 100%")
