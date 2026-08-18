"""Locally optimize an irregular 100-box, two-Euro-pallet experiment.

This uses a randomized height-map heuristic because the inherited SciPy/HiGHS
fallback cannot solve the quadratic CQM and no Leap token is configured. It
chooses pallet, axis-aligned orientation, and position while enforcing pallet
boundaries, non-overlap, and at least 75% support for every raised box.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from itertools import permutations
from pathlib import Path
import csv
import random

import numpy as np
import plotly.graph_objects as go

from packing3d import Bins, Cases, Variables, build_cqm
from utils import _plot_cuboids, read_instance


ROOT = Path(__file__).parent
INPUT = ROOT / "input" / "irregular_100_box_experiment.txt"
OUTPUT = ROOT / "output"
OUTPUT.mkdir(exist_ok=True)

PALLET_LENGTH = 120
PALLET_WIDTH = 80
PALLET_HEIGHT = 180
MIN_SUPPORT = 0.75
TRIALS = 24
SEED = 20260817

GROUPS = [
    "chemical", "food", "general", "food", "chemical",
    "general", "food", "chemical", "general", "food",
    "chemical", "general", "food", "chemical", "general",
    "food", "chemical", "general", "food", "general",
]
WEIGHTS = [31, 18, 24, 14, 27, 22, 16, 33, 20, 13, 29, 19, 15, 30, 21, 12, 28, 17, 11, 23]
PRIORITIES = [5, 2, 4, 1, 3, 5, 2, 4, 3, 1, 5, 4, 2, 3, 5, 1, 4, 3, 2, 1]


@dataclass(frozen=True)
class Box:
    index: int
    case_id: int
    dims: tuple[int, int, int]
    weight: int
    group: str
    priority: int


@dataclass(frozen=True)
class Placement:
    box: Box
    pallet: int
    x: int
    y: int
    z: int
    dx: int
    dy: int
    dz: int
    support: float


def rotations(dims: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    return sorted(set(permutations(dims)))


def candidate_points(
    placed: list[Placement], dx: int, dy: int
) -> set[tuple[int, int]]:
    points = {
        (0, 0),
        (PALLET_LENGTH - dx, 0),
        (0, PALLET_WIDTH - dy),
        (PALLET_LENGTH - dx, PALLET_WIDTH - dy),
    }
    for p in placed:
        points.update(
            {
                (p.x + p.dx, p.y),
                (p.x - dx, p.y),
                (p.x, p.y + p.dy),
                (p.x, p.y - dy),
                (p.x + p.dx - dx, p.y + p.dy),
                (p.x + p.dx, p.y + p.dy - dy),
                (p.x + p.dx - dx, p.y),
                (p.x, p.y + p.dy - dy),
            }
        )
    return {
        (x, y)
        for x, y in points
        if 0 <= x <= PALLET_LENGTH - dx and 0 <= y <= PALLET_WIDTH - dy
    }


def horizontal_overlap(a: Placement, b: Placement) -> bool:
    return (
        a.x < b.x + b.dx
        and b.x < a.x + a.dx
        and a.y < b.y + b.dy
        and b.y < a.y + a.dy
    )


def local_rule_penalty(candidate: Placement, placed: list[Placement]) -> float:
    penalty = 0.0
    for other in placed:
        if not horizontal_overlap(candidate, other):
            continue
        if candidate.z >= other.z + other.dz:
            if candidate.box.group == "chemical" and other.box.group == "food":
                penalty += 100.0
            if candidate.box.weight > other.box.weight:
                penalty += candidate.box.weight - other.box.weight
            if candidate.box.priority > other.box.priority:
                penalty += 10.0 * (candidate.box.priority - other.box.priority)
    return penalty


def place_boxes(order: list[Box], jitter_seed: int) -> list[Placement] | None:
    rng = random.Random(jitter_seed)
    heightmaps = [
        np.zeros((PALLET_LENGTH, PALLET_WIDTH), dtype=np.int16) for _ in range(2)
    ]
    by_pallet: list[list[Placement]] = [[], []]
    all_placements: list[Placement] = []

    for box in order:
        best: tuple[tuple[float, ...], Placement] | None = None
        current_heights = [int(h.max()) for h in heightmaps]
        for pallet in range(2):
            for dx, dy, dz in rotations(box.dims):
                if dx > PALLET_LENGTH or dy > PALLET_WIDTH or dz > PALLET_HEIGHT:
                    continue
                for x, y in candidate_points(by_pallet[pallet], dx, dy):
                    region = heightmaps[pallet][x : x + dx, y : y + dy]
                    z = int(region.max())
                    if z + dz > PALLET_HEIGHT:
                        continue
                    support = 1.0 if z == 0 else float(np.mean(region == z))
                    if support + 1e-12 < MIN_SUPPORT:
                        continue
                    candidate = Placement(box, pallet, x, y, z, dx, dy, dz, support)
                    new_heights = current_heights.copy()
                    new_heights[pallet] = max(new_heights[pallet], z + dz)
                    cavity = float(z * dx * dy - region.sum())
                    rule_penalty = local_rule_penalty(candidate, by_pallet[pallet])
                    score = (
                        max(new_heights),
                        sum(new_heights),
                        rule_penalty,
                        cavity,
                        z,
                        -support,
                        rng.random() * 0.01,
                    )
                    if best is None or score < best[0]:
                        best = (score, candidate)

        if best is None:
            return None
        placement = best[1]
        region = heightmaps[placement.pallet][
            placement.x : placement.x + placement.dx,
            placement.y : placement.y + placement.dy,
        ]
        region[:, :] = placement.z + placement.dz
        by_pallet[placement.pallet].append(placement)
        all_placements.append(placement)

    return all_placements


def solution_metrics(placements: list[Placement]) -> dict[str, float]:
    heights = [0, 0]
    product_penalty = 0
    weight_penalty = 0.0
    retrieval_penalty = 0
    for p in placements:
        heights[p.pallet] = max(heights[p.pallet], p.z + p.dz)
    for i, upper in enumerate(placements):
        for lower in placements[i + 1 :]:
            if upper.pallet != lower.pallet or not horizontal_overlap(upper, lower):
                continue
            a, b = (upper, lower) if upper.z >= lower.z else (lower, upper)
            if a.box.group == "chemical" and b.box.group == "food":
                product_penalty += 1
            if a.box.weight > b.box.weight:
                weight_penalty += a.box.weight - b.box.weight
            if a.box.priority > b.box.priority:
                retrieval_penalty += a.box.priority - b.box.priority
    return {
        "max_height": max(heights),
        "sum_height": sum(heights),
        "height_0": heights[0],
        "height_1": heights[1],
        "min_support": min(p.support for p in placements),
        "product_penalty": product_penalty,
        "weight_penalty": weight_penalty,
        "retrieval_penalty": retrieval_penalty,
    }


def objective(metrics: dict[str, float]) -> float:
    return (
        100.0 * metrics["max_height"]
        + 10.0 * metrics["sum_height"]
        + 50.0 * metrics["product_penalty"]
        + metrics["weight_penalty"]
        + 20.0 * metrics["retrieval_penalty"]
    )


data = read_instance(str(INPUT))
boxes: list[Box] = []
index = 0
for case_id in range(20):
    dims = (
        int(data["Length"][case_id]),
        int(data["Width"][case_id]),
        int(data["Height"][case_id]),
    )
    for _ in range(int(data["Quantity"][case_id])):
        boxes.append(
            Box(index, case_id, dims, WEIGHTS[case_id], GROUPS[case_id], PRIORITIES[case_id])
        )
        index += 1

group_rank = {"chemical": 0, "general": 1, "food": 2}
base_order = sorted(
    boxes,
    key=lambda b: (
        group_rank[b.group],
        -b.weight,
        -b.priority,
        -(b.dims[0] * b.dims[1] * b.dims[2]),
    ),
)

best_placements: list[Placement] | None = None
best_metrics: dict[str, float] | None = None
best_objective = float("inf")
successful_trials = 0
rng = random.Random(SEED)

for trial in range(TRIALS):
    if trial == 0:
        order = base_order.copy()
    else:
        order = sorted(
            base_order,
            key=lambda b: (
                group_rank[b.group] + rng.uniform(-0.20, 0.20),
                -b.weight + rng.uniform(-8, 8),
                -b.priority + rng.uniform(-1.5, 1.5),
                rng.random(),
            ),
        )
    placements = place_boxes(order, SEED + trial)
    if placements is None:
        continue
    successful_trials += 1
    metrics = solution_metrics(placements)
    score = objective(metrics)
    if score < best_objective:
        best_objective = score
        best_placements = placements
        best_metrics = metrics

if best_placements is None or best_metrics is None:
    raise RuntimeError("No feasible two-pallet solution found")

# Restore input expansion order for output and CQM validation.
best_placements.sort(key=lambda p: p.box.index)

# The inherited formulation fixes the first expanded box to pallet 0 as a
# symmetry break. Pallet labels are interchangeable, so relabel them when the
# heuristic happened to place box 0 on pallet 1.
if best_placements[0].pallet == 1:
    best_placements = [replace(p, pallet=1 - p.pallet) for p in best_placements]
    best_metrics = solution_metrics(best_placements)

# Validate the heuristic result against the inherited geometric CQM.
cases = Cases(data)
bins = Bins(data, cases)
variables = Variables(cases, bins)
cqm, effective_dimensions = build_cqm(variables, bins, cases)
sample = {label: 0 for label in cqm.variables}

for p in best_placements:
    i = p.box.index
    original_permutations = list(permutations(p.box.dims))
    orientation = original_permutations.index((p.dx, p.dy, p.dz))
    sample[f"o_{i}_{orientation}"] = 1
    sample[f"x_{i}"] = p.pallet * PALLET_LENGTH + p.x
    sample[f"y_{i}"] = p.y
    sample[f"z_{i}"] = p.z
    if i > 0:
        for pallet in range(2):
            sample[f"case_{i}_in_bin_{pallet}"] = int(pallet == p.pallet)

sample["upper_bound_0"] = best_metrics["height_0"]
sample["upper_bound_1"] = best_metrics["height_1"]

for i, a in enumerate(best_placements):
    ax = a.pallet * PALLET_LENGTH + a.x
    for k in range(i + 1, len(best_placements)):
        b = best_placements[k]
        bx = b.pallet * PALLET_LENGTH + b.x
        if ax + a.dx <= bx:
            relation = 0
        elif a.y + a.dy <= b.y:
            relation = 1
        elif a.z + a.dz <= b.z:
            relation = 2
        elif bx + b.dx <= ax:
            relation = 3
        elif b.y + b.dy <= a.y:
            relation = 4
        elif b.z + b.dz <= a.z:
            relation = 5
        else:
            raise RuntimeError(f"Heuristic boxes {i} and {k} overlap")
        sample[f"sel_{i}_{k}_{relation}"] = 1

if not cqm.check_feasible(sample):
    violations = sorted(
        (row for row in cqm.iter_constraint_data(sample) if row.violation > 1e-6),
        key=lambda row: row.violation,
        reverse=True,
    )
    for row in violations[:10]:
        print(f"VIOLATION {row.label}: {row.violation}")
    raise RuntimeError("Heuristic layout failed inherited CQM validation")

positions = [
    (p.pallet * PALLET_LENGTH + p.x, p.y, p.z) for p in best_placements
]
sizes = [(p.dx, p.dy, p.dz) for p in best_placements]
case_ids = np.array([p.box.case_id for p in best_placements])
figure = _plot_cuboids(
    positions,
    sizes,
    PALLET_LENGTH * 2,
    PALLET_WIDTH,
    PALLET_HEIGHT,
    True,
    case_ids,
)
for pallet in range(2):
    left = pallet * PALLET_LENGTH
    right = (pallet + 1) * PALLET_LENGTH
    figure.add_trace(
        go.Scatter3d(
            x=[left, right, right, left, left],
            y=[0, 0, PALLET_WIDTH, PALLET_WIDTH, 0],
            z=[0, 0, 0, 0, 0],
            mode="lines",
            name=f"Pallet {pallet + 1}",
            line={"color": "red", "width": 6},
        )
    )
figure.update_layout(
    title="Locally optimized irregular 100-box packing on two Euro pallets",
    scene={"aspectmode": "data"},
)

html_path = OUTPUT / "optimized_irregular_100_boxes.html"
csv_path = OUTPUT / "optimized_irregular_100_boxes.csv"
figure.write_html(html_path)

with csv_path.open("w", newline="", encoding="utf-8") as handle:
    writer = csv.writer(handle)
    writer.writerow(
        [
            "box_index", "case_id", "pallet", "x", "y", "z",
            "length", "width", "height", "support", "weight",
            "product_group", "retrieval_priority",
        ]
    )
    for p in best_placements:
        writer.writerow(
            [
                p.box.index, p.box.case_id, p.pallet + 1, p.x, p.y, p.z,
                p.dx, p.dy, p.dz, f"{p.support:.4f}", p.box.weight,
                p.box.group, p.box.priority,
            ]
        )

box_volume = sum(np.prod(p.box.dims) for p in best_placements)
pallet_volume = PALLET_LENGTH * PALLET_WIDTH * PALLET_HEIGHT
print(f"Trials: {TRIALS} ({successful_trials} feasible)")
print(f"Best heuristic objective: {best_objective:.2f}")
print("Inherited CQM feasible: yes")
print(f"Boxes: {len(best_placements)}; sizes: {len(set(b.dims for b in boxes))}")
print(f"Packed heights: {best_metrics['height_0']:.0f} cm, {best_metrics['height_1']:.0f} cm")
print(f"Minimum support: {100 * best_metrics['min_support']:.2f}%")
print(f"Product-order penalty: {best_metrics['product_penalty']:.0f}")
print(f"Weight-order penalty: {best_metrics['weight_penalty']:.0f}")
print(f"Retrieval-order penalty: {best_metrics['retrieval_penalty']:.0f}")
print(f"Two-pallet volume utilization: {100 * box_volume / (2 * pallet_volume):.2f}%")
print(f"CQM variables: {len(cqm.variables):,}; constraints: {len(cqm.constraints):,}")
print(f"Wrote {html_path.relative_to(ROOT)}")
print(f"Wrote {csv_path.relative_to(ROOT)}")
