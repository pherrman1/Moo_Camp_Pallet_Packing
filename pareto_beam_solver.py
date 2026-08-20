"""Pareto beam-search heuristic for the intelligent pallet-loading project.

The search follows the deliberately simple algorithm agreed for the project:

1. Sort the physical boxes once and place one box per search level.
2. Expand every retained partial packing at geometric candidate points.
3. Reject children that violate hard geometry, support, payload, product-order,
   fragility, mass-order, or load-bearing conditions.
4. Evaluate each child by (pallets, height, accessibility, category distance).
5. Fill the next beam front-by-front by Pareto rank.  If only part of a front
   fits, use crowding distance and only then a normalized weighted tie-break.
6. Return the nondominated complete states as an approximate Pareto archive.

This module reuses the JSON parser, placement records, CSV-row conversion, and
colour convention of ``gurobi_coordinate_solver.py``.  It does *not* build or
solve a Gurobi model and therefore does not consume a Gurobi licence.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import time
from collections import defaultdict
from dataclasses import asdict, dataclass
from itertools import permutations, product
from pathlib import Path
from typing import Any, Iterable, Sequence

import numpy as np
import plotly.graph_objects as go

from gurobi_coordinate_solver import (
    CoordinateItem,
    CoordinatePlacement,
    CoordinateSolution,
    allowed_orientations,
    audit_solution,
    category_type_colors,
    footprint_overlap,
    overlap_1d,
    read_mcpp_json,
    solution_rows,
)
from utils import _plot_cuboids


OBJECTIVE_NAMES = ("pallets", "height", "accessibility", "category_distance")
GRAVITY = 9.80665


@dataclass(frozen=True)
class SearchConfig:
    """Settings that define one deterministic Pareto beam-search run."""

    beam_width: int = 40
    max_pallets: int = 3
    grid_mm: int = 50
    rotation_mode: str = "yaw"
    support_fraction: float = 0.75
    food_chemical_mode: str = "chemical_below_food"
    stacking_mass_alpha: float | None = 1.2
    enforce_fragile_support: bool = True
    enforce_load_bearing: bool = True
    candidate_point_limit: int = 120
    children_per_state: int = 80
    cross_pallet_category_penalty: float = 3.0
    objective_weights: tuple[float, float, float, float] = (0.25, 0.25, 0.25, 0.25)
    item_order: str = "chemical-first"


@dataclass(frozen=True)
class ObjectiveVector:
    """All four minimization objectives used to compare partial states."""

    pallets: float
    height: float
    accessibility: float
    category_distance: float

    def values(self) -> tuple[float, float, float, float]:
        return self.pallets, self.height, self.accessibility, self.category_distance


@dataclass(frozen=True)
class BeamState:
    """A partial packing retained in the beam."""

    placements: tuple[CoordinatePlacement, ...]
    pallet_payloads_kg: tuple[float, ...]
    objectives: ObjectiveVector

    @property
    def pallet_count(self) -> int:
        return len(self.pallet_payloads_kg)


@dataclass
class SearchResult:
    archive: list[BeamState]
    recommended: BeamState
    runtime_seconds: float
    levels_completed: int
    states_generated: int
    states_feasible: int


@dataclass(frozen=True)
class SearchProblem:
    context: dict[str, Any]
    items: list[CoordinateItem]
    orientations: dict[int, list[tuple[int, int, int]]]
    load_bearing_kpa: dict[int, float]
    oriented_top_area_m2: dict[tuple[int, int], float]
    config: SearchConfig


def _raw_orientations(
    item: CoordinateItem, mode: str, grid_mm: int
) -> list[tuple[float, float, float]]:
    """Match unsnapped physical dimensions to the deduplicated grid orientations."""
    length, width, height = (float(value) for value in item.original_mm)
    if mode == "none":
        candidates = [(length, width, height)]
    elif item.upright_only or mode in {"yaw", "metadata"}:
        candidates = [(length, width, height), (width, length, height)]
    elif mode == "six":
        candidates = list(permutations((length, width, height), 3))
    else:
        raise ValueError("rotation_mode must be one of: none, yaw, metadata, six")

    unique: list[tuple[float, float, float]] = []
    seen_grid: set[tuple[int, int, int]] = set()
    for candidate in candidates:
        snapped = tuple(max(1, math.ceil(axis / grid_mm - 1e-12)) for axis in candidate)
        if snapped not in seen_grid:
            seen_grid.add(snapped)
            unique.append(candidate)
    return unique


def build_problem(input_path: Path, config: SearchConfig) -> SearchProblem:
    """Read an MCPP instance and retain the extended load-bearing metadata."""
    parser_config = {
        "grid_mm": config.grid_mm,
        "max_items": 1_000_000,
        "rotation_mode": config.rotation_mode,
        "stacking_mass_alpha": config.stacking_mass_alpha or 0.0,
        "food_chemical": {"mode": config.food_chemical_mode},
        "support_area_objective": {"enabled": False},
    }
    context, items = read_mcpp_json(input_path, parser_config)
    records = context["payload"].get("items", context["payload"].get("boxes", []))
    orientations = {
        item.index: allowed_orientations(item, config.rotation_mode) for item in items
    }
    load_bearing_kpa: dict[int, float] = {}
    oriented_top_area_m2: dict[tuple[int, int], float] = {}
    for item, raw in zip(items, records):
        value = raw.get("load_bearing_kpa")
        load_bearing_kpa[item.index] = math.inf if value is None else float(value)
        raw_orientations = _raw_orientations(item, config.rotation_mode, config.grid_mm)
        if len(raw_orientations) != len(orientations[item.index]):
            raise RuntimeError(f"orientation metadata mismatch for box {item.id}")
        for orientation, (dx_mm, dy_mm, _) in enumerate(raw_orientations):
            oriented_top_area_m2[item.index, orientation] = dx_mm * dy_mm / 1_000_000.0
    return SearchProblem(
        context=context,
        items=items,
        orientations=orientations,
        load_bearing_kpa=load_bearing_kpa,
        oriented_top_area_m2=oriented_top_area_m2,
        config=config,
    )


def _item_order(problem: SearchProblem) -> list[int]:
    """Return the common box order used by every state at every search level."""
    config = problem.config

    def phase(item: CoordinateItem) -> int:
        if config.item_order != "chemical-first":
            return 0
        if item.is_chemical and not item.is_food:
            return 0
        if item.is_food and not item.is_chemical:
            return 2
        return 1

    return [
        item.index
        for item in sorted(
            problem.items,
            key=lambda item: (
                phase(item),
                -math.prod(item.dims),
                -item.weight_kg,
                -max(dx * dy for dx, dy, _ in problem.orientations[item.index]),
                item.index,
            ),
        )
    ]


def _three_dimensional_overlap(a: CoordinatePlacement, b: CoordinatePlacement) -> bool:
    return (
        overlap_1d(a.x, a.x + a.dx, b.x, b.x + b.dx) > 0
        and overlap_1d(a.y, a.y + a.dy, b.y, b.y + b.dy) > 0
        and overlap_1d(a.z, a.z + a.dz, b.z, b.z + b.dz) > 0
    )


def _supporters(
    upper: CoordinatePlacement, placements: Iterable[CoordinatePlacement]
) -> list[tuple[CoordinatePlacement, int]]:
    return [
        (lower, area)
        for lower in placements
        if lower.item != upper.item
        and lower.pallet == upper.pallet
        and lower.top == upper.z
        and (area := footprint_overlap(lower, upper)) > 0
    ]


def _support_arcs(placements: Sequence[CoordinatePlacement]) -> list[tuple[int, int]]:
    return [
        (lower.item, upper.item)
        for upper in placements
        if upper.z > 0
        for lower, _ in _supporters(upper, placements)
    ]


def _load_bearing_feasible(problem: SearchProblem, placements: Sequence[CoordinatePlacement]) -> bool:
    """Propagate carried mass downward through direct supporters by contact area."""
    if not problem.config.enforce_load_bearing:
        return True
    incoming_mass = {placement.item: 0.0 for placement in placements}
    for upper in sorted(placements, key=lambda placement: placement.z, reverse=True):
        capacity_kpa = problem.load_bearing_kpa[upper.item]
        top_area_m2 = problem.oriented_top_area_m2[upper.item, upper.orientation]
        capacity_kg = capacity_kpa * top_area_m2 * 1000.0 / GRAVITY
        if incoming_mass[upper.item] > capacity_kg + 1e-9:
            return False
        if upper.z == 0:
            continue
        supporters = _supporters(upper, placements)
        total_area = sum(area for _, area in supporters)
        if total_area <= 0:
            return False
        transmitted_mass = problem.items[upper.item].weight_kg + incoming_mass[upper.item]
        for lower, area in supporters:
            incoming_mass[lower.item] += transmitted_mass * area / total_area
    return True


def _food_chemical_feasible(
    problem: SearchProblem,
    candidate: CoordinatePlacement,
    existing: Sequence[CoordinatePlacement],
) -> bool:
    if problem.config.food_chemical_mode == "off":
        return True
    item = problem.items[candidate.item]
    for other in existing:
        if other.pallet != candidate.pallet:
            continue
        other_item = problem.items[other.item]
        if item.is_chemical and other_item.is_food and candidate.top > other.z:
            return False
        if item.is_food and other_item.is_chemical and other.top > candidate.z:
            return False
    return True


def placement_feasible(
    problem: SearchProblem, state: BeamState, candidate: CoordinatePlacement
) -> bool:
    """Check every hard condition before a child state is created."""
    pallet = problem.context["pallet"]
    if (
        min(candidate.x, candidate.y, candidate.z) < 0
        or candidate.x + candidate.dx > pallet["length"]
        or candidate.y + candidate.dy > pallet["width"]
        or candidate.top > pallet["height"]
    ):
        return False

    same_pallet = [
        placement for placement in state.placements if placement.pallet == candidate.pallet
    ]
    if any(_three_dimensional_overlap(candidate, other) for other in same_pallet):
        return False

    if candidate.pallet < state.pallet_count:
        payload = state.pallet_payloads_kg[candidate.pallet]
    elif candidate.pallet == state.pallet_count:
        payload = 0.0
    else:
        return False
    if payload + problem.items[candidate.item].weight_kg > pallet["payload_kg"] + 1e-9:
        return False

    if not _food_chemical_feasible(problem, candidate, same_pallet):
        return False

    supporters = _supporters(candidate, same_pallet)
    if candidate.z > 0:
        supported_area = sum(area for _, area in supporters)
        if supported_area + 1e-9 < problem.config.support_fraction * candidate.base_area:
            return False
        if problem.config.enforce_fragile_support and any(
            problem.items[lower.item].fragile for lower, _ in supporters
        ):
            return False
        alpha = problem.config.stacking_mass_alpha
        if alpha is not None and any(
            problem.items[candidate.item].weight_kg
            > alpha * problem.items[lower.item].weight_kg + 1e-9
            for lower, _ in supporters
        ):
            return False

    placements = (*state.placements, candidate)
    return _load_bearing_feasible(problem, placements)


def candidate_points(
    state: BeamState, pallet_index: int, limit: int
) -> list[tuple[int, int, int]]:
    """Create compact extreme/corner points from the current pallet geometry."""
    placed = [p for p in state.placements if p.pallet == pallet_index]
    if not placed:
        return [(0, 0, 0)]

    primary: set[tuple[int, int, int]] = {(0, 0, 0)}
    xs, ys, zs = {0}, {0}, {0}
    for p in placed:
        primary.update(
            {
                (p.x + p.dx, p.y, p.z),
                (p.x, p.y + p.dy, p.z),
                (p.x, p.y, p.top),
            }
        )
        xs.add(p.x + p.dx)
        ys.add(p.y + p.dy)
        zs.add(p.top)

    # Coordinate combinations recover many useful projected extreme points that
    # the three immediate neighbours alone would miss.  The cap controls growth.
    secondary = set(product(xs, ys, zs)) - primary
    ordered_primary = sorted(primary, key=lambda point: (point[2], point[1], point[0]))
    ordered_secondary = sorted(secondary, key=lambda point: (point[2], point[1], point[0]))
    return (ordered_primary + ordered_secondary)[:limit]


def _accessibility_conflicts(
    placements: Sequence[CoordinatePlacement], items: Sequence[CoordinateItem]
) -> int:
    """Count later-priority boxes that vertically block earlier-priority boxes."""
    conflicts = 0
    for index, first in enumerate(placements):
        for second in placements[index + 1 :]:
            if first.pallet != second.pallet or footprint_overlap(first, second) <= 0:
                continue
            if first.top <= second.z:
                lower, upper = first, second
            elif second.top <= first.z:
                lower, upper = second, first
            else:
                continue
            if items[lower.item].retrieval_priority < items[upper.item].retrieval_priority:
                conflicts += 1
    return conflicts


def _category_distance(problem: SearchProblem, placements: Sequence[CoordinatePlacement]) -> float:
    """Mean normalized Manhattan distance for all pairs from the same family."""
    pallet = problem.context["pallet"]
    total = 0.0
    pairs = 0
    for index, first in enumerate(placements):
        first_item = problem.items[first.item]
        for second in placements[index + 1 :]:
            if first_item.family != problem.items[second.item].family:
                continue
            pairs += 1
            if first.pallet != second.pallet:
                total += problem.config.cross_pallet_category_penalty
                continue
            first_center = (
                first.x + first.dx / 2.0,
                first.y + first.dy / 2.0,
                first.z + first.dz / 2.0,
            )
            second_center = (
                second.x + second.dx / 2.0,
                second.y + second.dy / 2.0,
                second.z + second.dz / 2.0,
            )
            total += (
                abs(first_center[0] - second_center[0]) / pallet["length"]
                + abs(first_center[1] - second_center[1]) / pallet["width"]
                + abs(first_center[2] - second_center[2]) / pallet["height"]
            )
    return total / pairs if pairs else 0.0


def evaluate_state(problem: SearchProblem, placements: Sequence[CoordinatePlacement]) -> ObjectiveVector:
    if not placements:
        return ObjectiveVector(0.0, 0.0, 0.0, 0.0)
    pallet_count = max(placement.pallet for placement in placements) + 1
    heights = [
        max(
            placement.top
            for placement in placements
            if placement.pallet == pallet_index
        )
        for pallet_index in range(pallet_count)
    ]
    height = sum(heights) / (pallet_count * problem.context["pallet"]["height"])
    return ObjectiveVector(
        pallets=float(pallet_count),
        height=height,
        accessibility=float(_accessibility_conflicts(placements, problem.items)),
        category_distance=_category_distance(problem, placements),
    )


def _state_signature(state: BeamState) -> tuple[tuple[int, ...], ...]:
    return tuple(
        (
            p.item,
            p.pallet,
            p.orientation,
            p.x,
            p.y,
            p.z,
            p.dx,
            p.dy,
            p.dz,
        )
        for p in state.placements
    )


def dominates(first: BeamState, second: BeamState, tolerance: float = 1e-12) -> bool:
    a, b = first.objectives.values(), second.objectives.values()
    return all(x <= y + tolerance for x, y in zip(a, b)) and any(
        x < y - tolerance for x, y in zip(a, b)
    )


def nondominated_fronts(states: Sequence[BeamState]) -> list[list[BeamState]]:
    """NSGA-II style O(m n^2) nondominated sorting."""
    if not states:
        return []
    dominates_indices: list[list[int]] = [[] for _ in states]
    dominated_count = [0 for _ in states]
    first_front: list[int] = []
    for left in range(len(states)):
        for right in range(left + 1, len(states)):
            if dominates(states[left], states[right]):
                dominates_indices[left].append(right)
                dominated_count[right] += 1
            elif dominates(states[right], states[left]):
                dominates_indices[right].append(left)
                dominated_count[left] += 1
        if dominated_count[left] == 0:
            first_front.append(left)

    fronts: list[list[int]] = [first_front]
    while fronts[-1]:
        next_front: list[int] = []
        for left in fronts[-1]:
            for right in dominates_indices[left]:
                dominated_count[right] -= 1
                if dominated_count[right] == 0:
                    next_front.append(right)
        if next_front:
            fronts.append(next_front)
        else:
            break
    return [[states[index] for index in front] for front in fronts]


def crowding_distances(front: Sequence[BeamState]) -> dict[int, float]:
    """Return NSGA-II crowding distances keyed by object identity."""
    distances = {id(state): 0.0 for state in front}
    if len(front) <= 2:
        return {id(state): math.inf for state in front}
    for objective in range(len(OBJECTIVE_NAMES)):
        ordered = sorted(
            front,
            key=lambda state: (state.objectives.values()[objective], _state_signature(state)),
        )
        distances[id(ordered[0])] = math.inf
        distances[id(ordered[-1])] = math.inf
        minimum = ordered[0].objectives.values()[objective]
        maximum = ordered[-1].objectives.values()[objective]
        if maximum <= minimum + 1e-12:
            continue
        for index in range(1, len(ordered) - 1):
            if math.isinf(distances[id(ordered[index])]):
                continue
            previous_value = ordered[index - 1].objectives.values()[objective]
            next_value = ordered[index + 1].objectives.values()[objective]
            distances[id(ordered[index])] += (next_value - previous_value) / (maximum - minimum)
    return distances


def normalized_weighted_scores(
    states: Sequence[BeamState], weights: Sequence[float]
) -> dict[int, float]:
    values = [state.objectives.values() for state in states]
    minima = [min(row[q] for row in values) for q in range(len(OBJECTIVE_NAMES))]
    maxima = [max(row[q] for row in values) for q in range(len(OBJECTIVE_NAMES))]
    scores: dict[int, float] = {}
    for state, row in zip(states, values):
        normalized = [
            0.0 if maxima[q] <= minima[q] + 1e-12
            else (row[q] - minima[q]) / (maxima[q] - minima[q])
            for q in range(len(OBJECTIVE_NAMES))
        ]
        scores[id(state)] = sum(weight * value for weight, value in zip(weights, normalized))
    return scores


def pareto_select(
    states: Sequence[BeamState], beam_width: int, weights: Sequence[float]
) -> list[BeamState]:
    """Fill a beam by Pareto front, crowding, and finally weighted tie-break."""
    if len(states) <= beam_width:
        return sorted(states, key=_state_signature)
    scores = normalized_weighted_scores(states, weights)
    selected: list[BeamState] = []
    for front in nondominated_fronts(states):
        capacity = beam_width - len(selected)
        if len(front) <= capacity:
            selected.extend(sorted(front, key=_state_signature))
            if len(selected) == beam_width:
                break
            continue
        crowding = crowding_distances(front)
        front_order = sorted(
            front,
            key=lambda state: (
                -crowding[id(state)],
                scores[id(state)],
                _state_signature(state),
            ),
        )
        selected.extend(front_order[:capacity])
        break
    return selected


def _append_candidate(problem: SearchProblem, state: BeamState, candidate: CoordinatePlacement) -> BeamState:
    payloads = list(state.pallet_payloads_kg)
    if candidate.pallet == len(payloads):
        payloads.append(0.0)
    payloads[candidate.pallet] += problem.items[candidate.item].weight_kg
    placements = (*state.placements, candidate)
    return BeamState(
        placements=placements,
        pallet_payloads_kg=tuple(payloads),
        objectives=evaluate_state(problem, placements),
    )


def expand_state(problem: SearchProblem, state: BeamState, item_index: int) -> list[BeamState]:
    """Place the next fixed-order item on every eligible pallet/candidate/orientation."""
    config = problem.config
    children: dict[tuple[tuple[int, ...], ...], BeamState] = {}
    pallet_indices = list(range(state.pallet_count))
    if state.pallet_count < config.max_pallets:
        pallet_indices.append(state.pallet_count)

    for pallet_index in pallet_indices:
        points = (
            [(0, 0, 0)]
            if pallet_index == state.pallet_count
            else candidate_points(state, pallet_index, config.candidate_point_limit)
        )
        for orientation, (dx, dy, dz) in enumerate(problem.orientations[item_index]):
            for x, y, z in points:
                candidate = CoordinatePlacement(
                    item=item_index,
                    pallet=pallet_index,
                    orientation=orientation,
                    x=x,
                    y=y,
                    z=z,
                    dx=dx,
                    dy=dy,
                    dz=dz,
                )
                if not placement_feasible(problem, state, candidate):
                    continue
                child = _append_candidate(problem, state, candidate)
                children[_state_signature(child)] = child

    generated = list(children.values())
    if len(generated) > config.children_per_state:
        generated = pareto_select(
            generated, config.children_per_state, config.objective_weights
        )
    return generated


def solve(problem: SearchProblem, verbose: bool = True) -> SearchResult:
    """Run the complete fixed-depth Pareto beam search."""
    started = time.perf_counter()
    beam = [
        BeamState(
            placements=(),
            pallet_payloads_kg=(),
            objectives=ObjectiveVector(0.0, 0.0, 0.0, 0.0),
        )
    ]
    order = _item_order(problem)
    states_generated = 0
    states_feasible = 0

    for level, item_index in enumerate(order, start=1):
        children_by_signature: dict[tuple[tuple[int, ...], ...], BeamState] = {}
        for state in beam:
            children = expand_state(problem, state, item_index)
            states_generated += (
                len(problem.orientations[item_index])
                * (state.pallet_count * problem.config.candidate_point_limit + 1)
            )
            states_feasible += len(children)
            for child in children:
                children_by_signature[_state_signature(child)] = child
        if not children_by_signature:
            raise RuntimeError(
                f"search became infeasible at level {level}/{len(order)} while placing "
                f"box {problem.items[item_index].id}; increase max_pallets/candidate limits "
                "or relax a hard stacking policy"
            )
        beam = pareto_select(
            list(children_by_signature.values()),
            problem.config.beam_width,
            problem.config.objective_weights,
        )
        if verbose:
            first_front_size = len(nondominated_fronts(beam)[0])
            print(
                f"[{level:>3}/{len(order)}] box={problem.items[item_index].id:<4} "
                f"feasible_children={len(children_by_signature):<5} "
                f"beam={len(beam):<3} nondominated={first_front_size}"
            )

    archive = nondominated_fronts(beam)[0]
    scores = normalized_weighted_scores(archive, problem.config.objective_weights)
    recommended = min(
        archive, key=lambda state: (scores[id(state)], _state_signature(state))
    )
    return SearchResult(
        archive=sorted(archive, key=lambda state: (state.objectives.values(), _state_signature(state))),
        recommended=recommended,
        runtime_seconds=time.perf_counter() - started,
        levels_completed=len(order),
        states_generated=states_generated,
        states_feasible=states_feasible,
    )


def _coordinate_solution(state: BeamState, runtime_seconds: float) -> CoordinateSolution:
    heights = [
        max(p.top for p in state.placements if p.pallet == pallet_index)
        for pallet_index in range(state.pallet_count)
    ]
    return CoordinateSolution(
        status="PARETO_BEAM_HEURISTIC",
        pallet_count=state.pallet_count,
        objective_bound=0.0,
        mip_gap=0.0,
        runtime_seconds=runtime_seconds,
        placements=list(state.placements),
        max_height_grid=max(heights),
        average_top_height_grid=sum(heights) / len(heights),
        support_arcs=_support_arcs(state.placements),
        objective_mode="pareto_beam_P_H_A_D",
        category_distance_grid=state.objectives.category_distance,
    )


def validate_complete_state(problem: SearchProblem, state: BeamState) -> None:
    if len(state.placements) != len(problem.items):
        raise RuntimeError("complete-state audit found missing boxes")
    solution = _coordinate_solution(state, 0.0)
    audit_solution(
        solution,
        problem.context,
        support_mode="fraction",
        minimum_fraction=problem.config.support_fraction,
        items=problem.items,
        stacking_mass_alpha=problem.config.stacking_mass_alpha,
        food_chemical_mode=problem.config.food_chemical_mode,
    )
    if not _load_bearing_feasible(problem, state.placements):
        raise RuntimeError("complete-state audit found a load-bearing violation")


def render_state(
    problem: SearchProblem,
    state: BeamState,
    path: Path,
    solution_label: str,
) -> None:
    """Write an interactive Plotly view matching the coordinate MILP visual style."""
    solution = _coordinate_solution(state, 0.0)
    rows = solution_rows(solution, problem.items, problem.context)
    pallet = problem.context["pallet"]
    scale = 1.0
    positions = [
        (
            (row["pallet"] - 1) * pallet["length_mm"] + row["x_mm"],
            row["y_mm"],
            row["z_mm"],
        )
        for row in rows
    ]
    sizes = [
        (row["length_mm"], row["width_mm"], row["height_mm"]) for row in rows
    ]
    figure = _plot_cuboids(
        positions,
        sizes,
        pallet["length_mm"] * state.pallet_count * scale,
        pallet["width_mm"] * scale,
        pallet["height_mm"] * scale,
        True,
        np.array([row["sku"] for row in rows]),
    )
    visible_legend_entries: set[tuple[int, int]] = set()
    for trace, row, color in zip(figure.data, rows, category_type_colors(rows)):
        legend_key = int(row["retrieval_priority"]), int(row["sku"])
        trace.update(
            color=color,
            name=f"SKU {row['sku']} · {row['family']}",
            legendgroup=f"priority-{row['retrieval_priority']}",
            legendgrouptitle_text=f"Priority {row['retrieval_priority']}",
            showlegend=legend_key not in visible_legend_entries,
            hovertemplate=(
                f"Box {row['box_id']}<br>Priority {row['retrieval_priority']}"
                f"<br>SKU {row['sku']}<br>Family {row['family']}"
                f"<br>Class {row['class_label']}<br>Support {row['support_fraction']:.0%}"
                "<extra></extra>"
            ),
        )
        visible_legend_entries.add(legend_key)

    for label, trace_name, color in (
        ("F", "Food (F)", "darkgreen"),
        ("C", "Chemical (C)", "firebrick"),
        ("FC", "Food and chemical (FC)", "darkorange"),
    ):
        labelled = [
            (row, position, size)
            for row, position, size in zip(rows, positions, sizes)
            if row["class_label"] == label
        ]
        if labelled:
            figure.add_trace(
                go.Scatter3d(
                    x=[position[0] + size[0] / 2 for _, position, size in labelled],
                    y=[position[1] + size[1] / 2 for _, position, size in labelled],
                    z=[position[2] + size[2] + 4 for _, position, size in labelled],
                    mode="text",
                    text=[label] * len(labelled),
                    textfont={"color": color, "size": 18},
                    name=trace_name,
                    hoverinfo="skip",
                )
            )

    for pallet_index in range(state.pallet_count):
        left = pallet_index * pallet["length_mm"]
        right = (pallet_index + 1) * pallet["length_mm"]
        figure.add_trace(
            go.Scatter3d(
                x=[left, right, right, left, left],
                y=[0, 0, pallet["width_mm"], pallet["width_mm"], 0],
                z=[0, 0, 0, 0, 0],
                mode="lines",
                name=f"Pallet {pallet_index + 1}",
                line={"color": "red", "width": 6},
            )
        )
    objective = state.objectives
    pallet_word = "pallet" if state.pallet_count == 1 else "pallets"
    figure.update_layout(
        title=(
            f"{solution_label}: {state.pallet_count} {pallet_word} · "
            f"H={objective.height:.3f}, "
            f"A={objective.accessibility:.0f}, D={objective.category_distance:.3f} "
            "· hue=priority, shade=SKU/type"
        ),
        scene={
            "aspectmode": "data",
            "xaxis_title": "x (mm; pallets side by side)",
            "yaxis_title": "y (mm)",
            "zaxis_title": "z (mm)",
        },
    )
    instance_name = str(problem.context["payload"].get("name", "MCPP instance"))
    document_title = (
        f"{solution_label} - {state.pallet_count} {pallet_word} - {instance_name}"
    )
    description = (
        f"Interactive pallet loading for {instance_name}. {solution_label} uses "
        f"{state.pallet_count} {pallet_word}; H={objective.height:.3f}, "
        f"A={objective.accessibility:.0f}, D={objective.category_distance:.3f}."
    )
    document = figure.to_html(full_html=True, include_plotlyjs=True)
    document = document.replace(
        "<head>",
        "<head>"
        f"<title>{html.escape(document_title)}</title>"
        f'<meta name="description" content="{html.escape(description, quote=True)}">',
        1,
    )
    document = document.replace("<title>plotly-logomark</title>", "", 1)
    path.write_text(document, encoding="utf-8")


def render_pareto_front(result: SearchResult, path: Path) -> None:
    """Save a static 3D projection of all four Pareto objectives as a PNG."""
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError as exc:
        raise RuntimeError(
            "Pareto-front PNG generation requires matplotlib; run pip install -r requirements.txt"
        ) from exc

    grouped: dict[tuple[float, float, float, float], int] = defaultdict(int)
    for state in result.archive:
        grouped[state.objectives.values()] += 1
    objective_rows = [(*vector, multiplicity) for vector, multiplicity in grouped.items()]
    pallet_counts = sorted({int(row[0]) for row in objective_rows})
    color_map = plt.get_cmap("viridis", max(2, len(pallet_counts)))
    colors = {
        pallets: color_map(index / max(1, len(pallet_counts) - 1))
        for index, pallets in enumerate(pallet_counts)
    }
    markers = ("o", "^", "s", "D", "P", "X")

    figure = plt.figure(figsize=(12, 9), dpi=170)
    axis = figure.add_subplot(111, projection="3d")
    for index, pallets in enumerate(pallet_counts):
        rows = [row for row in objective_rows if int(row[0]) == pallets]
        axis.scatter(
            [100.0 * row[1] for row in rows],
            [row[3] for row in rows],
            [row[2] for row in rows],
            s=[70.0 + 35.0 * math.log2(row[4] + 1.0) for row in rows],
            marker=markers[index % len(markers)],
            color=colors[pallets],
            edgecolor="white",
            linewidth=0.8,
            alpha=0.9,
            depthshade=True,
            label=f"{pallets} pallet{'s' if pallets != 1 else ''}",
        )

    recommended = result.recommended.objectives
    axis.scatter(
        [100.0 * recommended.height],
        [recommended.category_distance],
        [recommended.accessibility],
        s=260,
        marker="*",
        color="#f2c14e",
        edgecolor="#202020",
        linewidth=1.2,
        depthshade=False,
        label="Recommended solution",
        zorder=20,
    )
    axis.set_xlabel("Average pallet height H (% of height limit)", labelpad=12)
    axis.set_ylabel("Same-category Manhattan distance D", labelpad=12)
    axis.set_zlabel("Accessibility conflicts A", labelpad=10)
    axis.set_title(
        "Approximate Pareto front\n"
        "Position shows H, D and A; colour and marker shape show pallet count P",
        pad=24,
        fontsize=14,
    )
    axis.view_init(elev=24, azim=-58)
    axis.grid(True, alpha=0.3)
    axis.xaxis.pane.set_alpha(0.06)
    axis.yaxis.pane.set_alpha(0.06)
    axis.zaxis.pane.set_alpha(0.06)
    axis.legend(loc="upper left", bbox_to_anchor=(0.01, 0.98), frameon=True)
    figure.text(
        0.5,
        0.025,
        "Larger markers represent several distinct packings with the same objective vector. "
        "All objectives are minimized.",
        ha="center",
        fontsize=10,
        color="#444444",
    )
    figure.subplots_adjust(left=0.03, right=0.93, bottom=0.09, top=0.88)
    figure.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(figure)


def _state_payload(problem: SearchProblem, state: BeamState, runtime_seconds: float) -> dict[str, Any]:
    solution = _coordinate_solution(state, runtime_seconds)
    rows = solution_rows(solution, problem.items, problem.context)
    pallet = problem.context["pallet"]
    volume = sum(p.dx * p.dy * p.dz for p in state.placements)
    capacity = state.pallet_count * pallet["length"] * pallet["width"] * pallet["height"]
    return {
        "instance": problem.context["payload"].get("name", "unknown"),
        "algorithm": "fixed-order Pareto beam search",
        "objectives": dict(zip(OBJECTIVE_NAMES, state.objectives.values())),
        "metrics": {
            "pallet_count": state.pallet_count,
            "volume_utilization": volume / capacity,
            "pallet_payloads_kg": list(state.pallet_payloads_kg),
            "runtime_seconds": runtime_seconds,
            "grid_mm": problem.config.grid_mm,
        },
        "settings": asdict(problem.config),
        "placements": rows,
    }


def _write_state_files(
    problem: SearchProblem,
    state: BeamState,
    runtime_seconds: float,
    output_dir: Path,
    stem: str,
    render: bool,
) -> None:
    payload = _state_payload(problem, state, runtime_seconds)
    rows = payload["placements"]
    (output_dir / f"{stem}.json").write_text(
        json.dumps(payload, indent=2), encoding="utf-8"
    )
    with (output_dir / f"{stem}.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    if render:
        render_state(
            problem,
            state,
            output_dir / f"{stem}.html",
            solution_label=stem.replace("_", " ").title(),
        )


def write_outputs(
    problem: SearchProblem,
    result: SearchResult,
    output_dir: Path,
    render_recommended: bool = True,
    render_all: bool = False,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    archive_rows = []
    for index, state in enumerate(result.archive):
        validate_complete_state(problem, state)
        stem = f"solution_{index:03d}_p{state.pallet_count}"
        archive_rows.append(
            {
                "solution": stem,
                **dict(zip(OBJECTIVE_NAMES, state.objectives.values())),
                "recommended": state is result.recommended,
            }
        )
        _write_state_files(
            problem,
            state,
            result.runtime_seconds,
            output_dir,
            stem,
            render=render_all,
        )

    with (output_dir / "pareto_front.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(archive_rows[0].keys()))
        writer.writeheader()
        writer.writerows(archive_rows)
    (output_dir / "pareto_front.json").write_text(
        json.dumps(archive_rows, indent=2), encoding="utf-8"
    )
    if render_recommended or render_all:
        render_pareto_front(result, output_dir / "pareto_front.png")
    _write_state_files(
        problem,
        result.recommended,
        result.runtime_seconds,
        output_dir,
        "recommended_solution",
        render=render_recommended,
    )
    summary = {
        "algorithm": "fixed-order Pareto beam search",
        "archive_size": len(result.archive),
        "levels_completed": result.levels_completed,
        "states_generated_estimate": result.states_generated,
        "feasible_children_after_local_filtering": result.states_feasible,
        "runtime_seconds": result.runtime_seconds,
        "recommended_objectives": dict(
            zip(OBJECTIVE_NAMES, result.recommended.objectives.values())
        ),
        "output_files": {
            "front": "pareto_front.csv",
            "front_plot": (
                "pareto_front.png" if render_recommended or render_all else None
            ),
            "recommended_json": "recommended_solution.json",
            "recommended_csv": "recommended_solution.csv",
            "recommended_visualization": (
                "recommended_solution.html" if render_recommended else None
            ),
        },
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )


def _parse_weights(value: str) -> tuple[float, float, float, float]:
    weights = tuple(float(part.strip()) for part in value.split(","))
    if len(weights) != 4 or any(weight < 0 for weight in weights):
        raise argparse.ArgumentTypeError("weights must be four nonnegative comma-separated values")
    total = sum(weights)
    if total <= 0:
        raise argparse.ArgumentTypeError("at least one weight must be positive")
    return tuple(weight / total for weight in weights)  # type: ignore[return-value]


def _recommended_max_pallets(payload: dict[str, Any], extra: int) -> int:
    stats = payload.get("stats", {})
    lower_bounds = [
        int(value)
        for key in ("lower_bound_pallets_L0", "lower_bound_pallets_weight")
        if (value := stats.get(key)) is not None
    ]
    return max(lower_bounds or [1]) + extra


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Multiobjective fixed-order Pareto beam search for MCPP JSON instances"
    )
    parser.add_argument("--input", required=True, type=Path, help="MCPP JSON instance")
    parser.add_argument(
        "--output-dir", type=Path, default=Path("output/pareto_beam"), help="result directory"
    )
    parser.add_argument("--beam-width", type=int, default=40)
    parser.add_argument("--max-pallets", type=int, default=None)
    parser.add_argument("--extra-pallets", type=int, default=2)
    parser.add_argument("--grid-mm", type=int, default=50)
    parser.add_argument("--rotation-mode", choices=("none", "yaw", "metadata", "six"), default="yaw")
    parser.add_argument("--support-fraction", type=float, default=0.75)
    parser.add_argument(
        "--food-chemical-mode",
        choices=("off", "chemical_below_food"),
        default="chemical_below_food",
    )
    parser.add_argument(
        "--stacking-mass-alpha",
        type=float,
        default=1.2,
        help="upper mass <= alpha * each direct supporter mass; <=0 disables",
    )
    parser.add_argument("--allow-stacking-on-fragile", action="store_true")
    parser.add_argument("--ignore-load-bearing", action="store_true")
    parser.add_argument("--candidate-point-limit", type=int, default=120)
    parser.add_argument("--children-per-state", type=int, default=80)
    parser.add_argument("--cross-pallet-category-penalty", type=float, default=3.0)
    parser.add_argument(
        "--weights",
        type=_parse_weights,
        default=(0.25, 0.25, 0.25, 0.25),
        metavar="P,H,A,D",
        help="tie-break weights only; Pareto rank and crowding remain primary",
    )
    parser.add_argument("--item-order", choices=("volume", "chemical-first"), default="chemical-first")
    parser.add_argument("--no-visualization", action="store_true")
    visualization_group = parser.add_mutually_exclusive_group()
    visualization_group.add_argument(
        "--render-all",
        dest="render_all",
        action="store_true",
        default=True,
        help="render every Pareto solution (default)",
    )
    visualization_group.add_argument(
        "--recommended-only",
        dest="render_all",
        action="store_false",
        help="render only the representative recommended solution",
    )
    parser.add_argument("--quiet", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_argument_parser().parse_args(argv)
    if args.beam_width <= 0 or args.candidate_point_limit <= 0 or args.children_per_state <= 0:
        raise ValueError("beam and candidate/child limits must be positive")
    if not 0 < args.support_fraction <= 1:
        raise ValueError("support_fraction must be in (0, 1]")

    payload = json.loads(args.input.read_text(encoding="utf-8"))
    max_pallets = args.max_pallets or _recommended_max_pallets(payload, args.extra_pallets)
    config = SearchConfig(
        beam_width=args.beam_width,
        max_pallets=max_pallets,
        grid_mm=args.grid_mm,
        rotation_mode=args.rotation_mode,
        support_fraction=args.support_fraction,
        food_chemical_mode=args.food_chemical_mode,
        stacking_mass_alpha=(
            args.stacking_mass_alpha if args.stacking_mass_alpha > 0 else None
        ),
        enforce_fragile_support=not args.allow_stacking_on_fragile,
        enforce_load_bearing=not args.ignore_load_bearing,
        candidate_point_limit=args.candidate_point_limit,
        children_per_state=args.children_per_state,
        cross_pallet_category_penalty=args.cross_pallet_category_penalty,
        objective_weights=args.weights,
        item_order=args.item_order,
    )
    problem = build_problem(args.input, config)
    if not args.quiet:
        print(
            f"Instance {payload.get('name', args.input.stem)}: {len(problem.items)} boxes, "
            f"beam={config.beam_width}, max_pallets={config.max_pallets}"
        )
    result = solve(problem, verbose=not args.quiet)
    write_outputs(
        problem,
        result,
        args.output_dir,
        render_recommended=not args.no_visualization,
        render_all=args.render_all and not args.no_visualization,
    )
    if not args.quiet:
        objective = result.recommended.objectives
        print(
            f"Finished in {result.runtime_seconds:.2f}s; archive={len(result.archive)}; "
            f"recommended=(P={objective.pallets:.0f}, H={objective.height:.3f}, "
            f"A={objective.accessibility:.0f}, D={objective.category_distance:.3f})"
        )
        print(f"Outputs: {args.output_dir.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
