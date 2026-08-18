"""Build, validate, and visualize a 100-box, 20-size Euro-pallet baseline.

This deterministic experiment packs 50 boxes on each of two Euro pallets.
Each box type occupies one complete layer, so all raised boxes have 100% base
support. Total volume exceeds one pallet's legal capacity, proving that at
least two pallets are required. It is a feasible baseline, not an optimized
solver result.
"""

from pathlib import Path

from packing3d import Bins, Cases, Variables, build_cqm
from utils import plot_cuboids, read_instance, write_solution_to_file


ROOT = Path(__file__).parent
INPUT = ROOT / "input" / "hundred_box_two_pallet_experiment.txt"
OUTPUT = ROOT / "output"
OUTPUT.mkdir(exist_ok=True)

data = read_instance(str(INPUT))
cases = Cases(data)
bins = Bins(data, cases)
variables = Variables(cases, bins)
cqm, effective_dimensions = build_cqm(variables, bins, cases)


def layer_offsets(quantity: int, length: int, width: int) -> list[tuple[int, int]]:
    """Return a complete 120 x 80 tiling for one supported layer."""
    if (quantity, length, width) == (4, 60, 40):
        return [(0, 0), (60, 0), (0, 40), (60, 40)]
    if (quantity, length, width) == (5, 24, 80):
        return [(24 * column, 0) for column in range(5)]
    if (quantity, length, width) == (6, 40, 40):
        return [(40 * column, 40 * row) for row in range(2) for column in range(3)]
    raise ValueError(f"No layer tiling for {(quantity, length, width)}")


positions: list[tuple[int, int, int]] = []
sizes: list[tuple[int, int, int]] = []
assignments: list[int] = []
packed_heights: list[int] = []

for pallet in range(2):
    z = 0
    x_offset = pallet * bins.length
    first_type = pallet * 10
    for case_id in range(first_type, first_type + 10):
        quantity = int(data["Quantity"][case_id])
        length = int(data["Length"][case_id])
        width = int(data["Width"][case_id])
        height = int(data["Height"][case_id])
        for x, y in layer_offsets(quantity, length, width):
            positions.append((x_offset + x, y, z))
            sizes.append((length, width, height))
            assignments.append(pallet)
        z += height
    packed_heights.append(z)

if not (len(positions) == len(sizes) == len(assignments) == cases.num_cases == 100):
    raise RuntimeError("The generated layout does not contain exactly 100 boxes")
if len(set(sizes)) != 20:
    raise RuntimeError("The generated layout does not contain exactly 20 sizes")

sample = {label: 0 for label in cqm.variables}

for i, (x, y, z) in enumerate(positions):
    sample[f"o_{i}_0"] = 1
    sample[f"x_{i}"] = x
    sample[f"y_{i}"] = y
    sample[f"z_{i}"] = z
    if i > 0:
        for pallet in range(bins.num_bins):
            sample[f"case_{i}_in_bin_{pallet}"] = int(pallet == assignments[i])

for pallet, packed_height in enumerate(packed_heights):
    sample[f"upper_bound_{pallet}"] = packed_height

# Add one valid non-overlap relation for all 4,950 pairs. Cross-pallet pairs
# are separated along x; same-pallet pairs are separated within or by layers.
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
    raise RuntimeError("The 100-box arrangement violates the CQM")

figure = plot_cuboids(
    sample,
    variables,
    cases,
    bins,
    effective_dimensions,
    color_coded=True,
)
figure.update_layout(
    title="100-box Euro-pallet experiment: 20 sizes on two required pallets"
)

html_path = OUTPUT / "hundred_box_two_pallet_experiment.html"
solution_path = OUTPUT / "hundred_box_two_pallet_experiment.txt"
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

box_volume = sum(length * width * height for length, width, height in sizes)
pallet_volume = bins.length * bins.width * bins.height
print("CQM feasible: yes")
print(f"CQM variables: {len(cqm.variables):,}")
print(f"CQM constraints: {len(cqm.constraints):,}")
print(f"Boxes packed: {cases.num_cases}")
print(f"Distinct sizes: {len(set(sizes))}")
print(f"Boxes per pallet: {assignments.count(0)}, {assignments.count(1)}")
print(f"Pallets used: 2 (volume lower bound: {bins.lowest_num_bin})")
print(f"Total box volume: {box_volume:,} cm^3")
print(f"One-pallet capacity: {pallet_volume:,} cm^3")
print(f"Two-pallet volume utilization: {100 * box_volume / (2 * pallet_volume):.2f}%")
print(f"Packed heights: {packed_heights[0]} cm, {packed_heights[1]} cm")
print("Raised-box support: 100%")
print(f"Wrote {html_path.relative_to(ROOT)}")
print(f"Wrote {solution_path.relative_to(ROOT)}")
