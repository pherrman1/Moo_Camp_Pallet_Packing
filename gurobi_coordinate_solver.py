"""Coordinate-based exact MILP for the intelligent pallet-loading project.

The default reduced-exact formulation uses pallet-assignment variables plus one
local orientation and integer-coordinate state per physical box.  A legacy
pallet-indexed geometry formulation remains selectable for equivalence tests.
Neither formulation uses position-indexed placement binaries. The active
lexicographic objectives minimize the number of used pallets and then the
maximum occupied height; a standalone same-category center-distance objective
is also selectable.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import time
from collections import defaultdict
from dataclasses import asdict, dataclass, field
from fractions import Fraction
from itertools import permutations
from pathlib import Path
from typing import Any, Iterable

import gurobipy as gp
import numpy as np
import plotly.graph_objects as go
from gurobipy import GRB

from utils import _plot_cuboids


DEFAULT_CONFIG = Path(__file__).parent / "configs" / "gurobi_coordinate_default.json"
CATEGORY_DISTANCE_OBJECTIVE_MODES = {
    "category_distance_only",
    "category_distance_then_max_height",
}


@dataclass(frozen=True)
class CoordinateItem:
    index: int
    id: int
    sku: int
    original_mm: tuple[int, int, int]
    dims: tuple[int, int, int]
    weight_kg: float
    volume_dm3: float
    family: str
    is_food: bool
    is_chemical: bool
    fragile: bool
    upright_only: bool
    retrieval_priority: int


@dataclass(frozen=True)
class CoordinatePlacement:
    item: int
    pallet: int
    orientation: int
    x: int
    y: int
    z: int
    dx: int
    dy: int
    dz: int

    @property
    def top(self) -> int:
        return self.z + self.dz

    @property
    def base_area(self) -> int:
        return self.dx * self.dy


@dataclass
class CoordinateSolution:
    status: str
    pallet_count: int
    objective_bound: float
    mip_gap: float
    runtime_seconds: float
    placements: list[CoordinatePlacement]
    max_height_grid: int = 0
    average_top_height_grid: float = 0.0
    height_objective_bound_grid: float | None = None
    height_mip_gap: float | None = None
    height_stage_attempted: bool = False
    footprint_depth_lower_bound: int = 0
    footprint_height_lower_bound_grid: int = 0
    support_area_grid2: float = 0.0
    support_area_objective_bound_grid2: float | None = None
    support_area_mip_gap: float | None = None
    support_area_stage_attempted: bool = False
    support_arcs: list[tuple[int, int]] = field(default_factory=list)
    objective_mode: str = "pallet_count_only"
    category_distance_grid: float | None = None
    category_distance_objective_bound_grid: float | None = None
    category_distance_mip_gap: float | None = None
    category_distance_stage_attempted: bool = False
    fixed_pallet_count: int | None = None


@dataclass
class _WarmStartPallet:
    """Mutable skyline state used only by the coordinate warm-start heuristic."""

    height: np.ndarray
    supporting_mass: np.ndarray
    supporting_kind: np.ndarray
    payload_kg: float
    placements: list[CoordinatePlacement]
    max_chemical_top: int = 0


def load_config(path: str | Path | None) -> dict[str, Any]:
    config_path = Path(path) if path else DEFAULT_CONFIG
    return json.loads(config_path.read_text(encoding="utf-8"))


def configured_food_chemical_mode(config: dict[str, Any]) -> str:
    """Validate the optional pallet-wide chemical/food vertical policy."""
    mode = str(config.get("food_chemical", {}).get("mode", "off"))
    if mode not in {"off", "chemical_below_food"}:
        raise ValueError("food_chemical.mode must be off or chemical_below_food")
    return mode


def configured_support_area_objective(config: dict[str, Any]) -> bool:
    """Return whether exact support area is the final lexicographic objective."""
    settings = config.get("support_area_objective", {})
    if not isinstance(settings, dict):
        raise ValueError("support_area_objective must be an object")
    enabled = settings.get("enabled", False)
    if not isinstance(enabled, bool):
        raise ValueError("support_area_objective.enabled must be true or false")
    return enabled


def configured_objective_mode(config: dict[str, Any]) -> str:
    """Validate and return the selected standalone/lexicographic objective mode."""
    mode = str(config.get("objective_mode", "pallet_count_only"))
    allowed = {
        "pallet_count_only",
        "pallets_then_max_height",
        "pallets_then_average_height",
        *CATEGORY_DISTANCE_OBJECTIVE_MODES,
    }
    if mode not in allowed:
        raise ValueError(
            "objective_mode must be pallet_count_only, pallets_then_max_height, "
            "pallets_then_average_height, category_distance_only, or "
            "category_distance_then_max_height"
        )
    return mode


def snap_up(value_mm: float, grid_mm: int) -> int:
    """Round a box dimension conservatively upward to grid units."""
    return max(1, math.ceil(float(value_mm) / grid_mm - 1e-12))


def pallet_axes(raw: dict[str, Any]) -> tuple[float, float, float]:
    """Accept both width/depth/height and length/width/height schemas."""
    if {"L_mm", "W_mm", "H_mm"} <= raw.keys():
        return float(raw["L_mm"]), float(raw["W_mm"]), float(raw["H_mm"])
    if "depth" in raw:
        return float(raw.get("length", raw["width"])), float(raw["depth"]), float(raw["height"])
    return float(raw["length"]), float(raw["width"]), float(raw["height"])


def item_axes(raw: dict[str, Any]) -> tuple[float, float, float]:
    if {"w_mm", "d_mm", "h_mm"} <= raw.keys():
        return float(raw["w_mm"]), float(raw["d_mm"]), float(raw["h_mm"])
    if "depth" in raw:
        return float(raw.get("length", raw["width"])), float(raw["depth"]), float(raw["height"])
    return float(raw["length"]), float(raw["width"]), float(raw["height"])


def read_mcpp_json(path: str | Path, config: dict[str, Any]) -> tuple[dict[str, Any], list[CoordinateItem]]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    configured_grid = config.get("grid_mm", 50)
    if configured_grid == "input":
        grid = int(payload.get("meta", {}).get("grid_mm", 50))
    else:
        grid = int(configured_grid)
    if grid <= 0:
        raise ValueError("grid_mm must be positive")

    pallet_raw = payload["pallet"]
    length_mm, width_mm, height_mm = pallet_axes(pallet_raw)
    # Flooring the pallet and ceiling box dimensions never creates a physically
    # invalid packing when data are not exact multiples of the grid.
    pallet = {
        "length": int(math.floor(length_mm / grid + 1e-12)),
        "width": int(math.floor(width_mm / grid + 1e-12)),
        "height": int(math.floor(height_mm / grid + 1e-12)),
        "length_mm": int(round(length_mm)),
        "width_mm": int(round(width_mm)),
        "height_mm": int(round(height_mm)),
        "name": str(pallet_raw.get("name", "pallet")),
        "payload_kg": float(
            pallet_raw.get(
                "payload_kg",
                pallet_raw.get("max_payload_kg", config.get("default_payload_kg", 1000.0)),
            )
        ),
    }
    if min(pallet["length"], pallet["width"], pallet["height"]) <= 0:
        raise ValueError("pallet dimensions must be at least one grid unit")
    if not math.isfinite(pallet["payload_kg"]) or pallet["payload_kg"] <= 0:
        raise ValueError("pallet payload capacity must be a positive finite number")

    records = payload.get("items", payload.get("boxes"))
    if records is None:
        raise ValueError("instance JSON must contain an 'items' or 'boxes' array")
    items: list[CoordinateItem] = []
    for index, raw in enumerate(records):
        axes = item_axes(raw)
        original_axes = (
            float(raw.get("raw_w_mm", axes[0])),
            float(raw.get("raw_d_mm", axes[1])),
            float(raw.get("raw_h_mm", axes[2])),
        )
        original = tuple(int(round(value)) for value in original_axes)
        dims = tuple(snap_up(value, grid) for value in axes)
        volume_dm3 = float(raw.get("volume_dm3", np.prod(original) / 1e6))
        group = str(raw.get("group", "")).lower()
        items.append(
            CoordinateItem(
                index=index,
                id=int(raw.get("id", index + 1)),
                sku=int(raw.get("sku", index + 1)),
                original_mm=original,
                dims=dims,
                weight_kg=float(raw.get("weight_kg", 1.0)),
                volume_dm3=volume_dm3,
                family=str(raw.get("family", group or "unknown")),
                is_food=bool(raw.get("is_food", group == "food")),
                is_chemical=bool(raw.get("is_chemical", group == "chemical")),
                fragile=bool(raw.get("fragile", False)),
                upright_only=bool(raw.get("upright_only", False)),
                retrieval_priority=int(raw.get("retrieval_priority", raw.get("priority", 1))),
            )
        )

    if not items:
        raise ValueError("the instance must contain at least one item")
    max_items = int(config.get("max_items", 30))
    if len(items) > max_items:
        raise ValueError(f"coordinate model is configured for at most {max_items} items; got {len(items)}")
    return {
        "payload": payload,
        "pallet": pallet,
        "grid_mm": grid,
        "visualization_unit": config.get("visualization_unit", "mm"),
        "stacking_mass_alpha": float(config.get("stacking_mass_alpha", 1.2)),
        "food_chemical_mode": configured_food_chemical_mode(config),
        "support_area_objective_enabled": configured_support_area_objective(config),
    }, items


def recommended_max_pallets(payload: dict[str, Any]) -> int | None:
    """Read a certified/heuristic upper bound when a benchmark provides one."""
    bounds = payload.get("meta", {}).get("bounds", {})
    value = bounds.get("ub_pallets_heuristic")
    if value is None:
        certified = bounds.get("certified_optimum_in")
        if isinstance(certified, list) and certified:
            value = certified[-1]
    return int(value) if value is not None else None


def allowed_orientations(item: CoordinateItem, mode: str) -> list[tuple[int, int, int]]:
    length, width, height = item.dims
    if mode == "none":
        candidates = [(length, width, height)]
    elif item.upright_only or mode in {"yaw", "metadata"}:
        candidates = [(length, width, height), (width, length, height)]
    elif mode == "six":
        candidates = list(permutations((length, width, height), 3))
    else:
        raise ValueError("rotation_mode must be one of: none, yaw, metadata, six")
    return list(dict.fromkeys(candidates))


def footprint_height_lower_bound(
    orientations: dict[int, list[tuple[int, int, int]]],
    pallet_length: int,
    pallet_width: int,
    pallet_count: int,
) -> tuple[int, int]:
    """Return a valid (projection depth, height) lower bound in grid units.

    Integrating box-footprint multiplicity over all pallet floors proves that
    some floor point is covered by at least ``depth`` projected boxes. Their
    vertical intervals must be disjoint, so their minimum heights add.
    Minimum orientation footprints and heights keep the cut valid for all
    rotation modes, even when those two minima use different orientations.
    """
    if pallet_count <= 0:
        raise ValueError("pallet_count must be positive for the footprint-height bound")
    floor_capacity = pallet_count * pallet_length * pallet_width
    if floor_capacity <= 0:
        raise ValueError("pallet floor dimensions must be positive")
    minimum_footprint_sum = sum(
        min(dx * dy for dx, dy, _ in item_orientations)
        for item_orientations in orientations.values()
    )
    depth = max(1, math.ceil(minimum_footprint_sum / floor_capacity))
    minimum_heights = sorted(
        min(dz for _, _, dz in item_orientations)
        for item_orientations in orientations.values()
    )
    depth = min(depth, len(minimum_heights))
    return depth, sum(minimum_heights[:depth])


def overlap_1d(a0: int, a1: int, b0: int, b1: int) -> int:
    return max(0, min(a1, b1) - max(a0, b0))


def footprint_overlap(a: CoordinatePlacement, b: CoordinatePlacement) -> int:
    return overlap_1d(a.x, a.x + a.dx, b.x, b.x + b.dx) * overlap_1d(
        a.y, a.y + a.dy, b.y, b.y + b.dy
    )


def support_fraction(upper: CoordinatePlacement, selected: Iterable[CoordinatePlacement]) -> float:
    if upper.z == 0:
        return 1.0
    supported = sum(
        footprint_overlap(lower, upper)
        for lower in selected
        if lower.item != upper.item and lower.pallet == upper.pallet and lower.top == upper.z
    )
    return min(1.0, supported / upper.base_area)


def _warm_start_item_groups(items: list[CoordinateItem]) -> list[list[int]]:
    groups: dict[tuple[Any, ...], list[int]] = defaultdict(list)
    for item in items:
        groups[
            (
                item.sku, item.dims, item.weight_kg, item.family, item.is_food,
                item.is_chemical, item.fragile, item.upright_only, item.retrieval_priority,
            )
        ].append(item.index)
    return [indices for indices in groups.values() if len(indices) > 1]


def _canonicalize_warm_start(
    placements: list[CoordinatePlacement],
    items: list[CoordinateItem],
    pallet: dict[str, Any],
    symmetry: dict[str, Any],
) -> list[CoordinatePlacement] | None:
    """Relabel equivalent pallets and items to satisfy enabled symmetry rules."""
    item_volume = {item.index: math.prod(item.dims) for item in items}
    pallet_indices = sorted({placement.pallet for placement in placements})
    pallet_volume = {
        p: sum(item_volume[q.item] for q in placements if q.pallet == p)
        for p in pallet_indices
    }
    if symmetry.get("order_pallet_loads", True):
        pallet_indices.sort(key=lambda p: (-pallet_volume[p], p))
    pallet_map = {old: new for new, old in enumerate(pallet_indices)}
    canonical = [
        CoordinatePlacement(
            item=q.item,
            pallet=pallet_map[q.pallet],
            orientation=q.orientation,
            x=q.x,
            y=q.y,
            z=q.z,
            dx=q.dx,
            dy=q.dy,
            dz=q.dz,
        )
        for q in placements
    ]

    if symmetry.get("order_identical_items", True):
        W, H = pallet["width"], pallet["height"]

        def location_key(q: CoordinatePlacement) -> tuple[int, int]:
            location = q.x * (W + 1) * (H + 1) + q.y * (H + 1) + q.z
            return q.pallet, location

        by_item = {q.item: q for q in canonical}
        for indices in _warm_start_item_groups(items):
            ordered_positions = sorted((by_item[i] for i in indices), key=location_key)
            for item_index, old in zip(sorted(indices), ordered_positions):
                by_item[item_index] = CoordinatePlacement(
                    item=item_index,
                    pallet=old.pallet,
                    orientation=old.orientation,
                    x=old.x,
                    y=old.y,
                    z=old.z,
                    dx=old.dx,
                    dy=old.dy,
                    dz=old.dz,
                )
        canonical = list(by_item.values())

    if symmetry.get("fix_first_item", True):
        first = next(q for q in canonical if q.item == 0)
        if first.pallet != 0:
            return None

    return sorted(canonical, key=lambda q: q.item)


def _warm_start_candidate(
    state: _WarmStartPallet,
    item: CoordinateItem,
    orientation: int,
    dimensions: tuple[int, int, int],
    pallet_index: int,
    pallet_height: int,
    support_mode: str,
    minimum_fraction: float,
) -> tuple[tuple[int, int, int, int, int], CoordinatePlacement] | None:
    """Return a feasible position, preferring chemical support for food."""
    dx, dy, dz = dimensions
    L, W = state.height.shape
    if dx > L or dy > W or dz > pallet_height:
        return None

    windows = np.lib.stride_tricks.sliding_window_view(state.height, (dx, dy))
    base_z = windows.max(axis=(2, 3))
    ok = base_z + dz <= pallet_height
    mass_windows = np.lib.stride_tricks.sliding_window_view(
        state.supporting_mass, (dx, dy)
    )
    kind_windows = np.lib.stride_tricks.sliding_window_view(
        state.supporting_kind, (dx, dy)
    )
    contact_cells = windows == base_z[:, :, None, None]
    # The heuristic deliberately uses a stricter mass rule than the MILP:
    # every direct supporter must be at least as heavy as the box above it.
    weight_order_ok = (
        (~contact_cells) | (item.weight_kg <= mass_windows + 1e-12)
    ).all(axis=(2, 3))
    ok &= (base_z == 0) | weight_order_ok
    supported_cells = (
        contact_cells & (item.weight_kg <= mass_windows + 1e-12)
    ).sum(axis=(2, 3))
    if support_mode == "direct":
        ok &= (base_z == 0) | (supported_cells > 0)
    elif support_mode == "fraction":
        ok &= (base_z == 0) | (supported_cells + 1e-12 >= minimum_fraction * dx * dy)
    elif support_mode == "full":
        ok &= (base_z == 0) | (supported_cells == dx * dy)

    # Chemicals are placed in the first phase. Keeping every food base above
    # the tallest chemical on that pallet satisfies chemical_below_food and
    # implements the intended chemical-bottom/food-top construction even when
    # the optional MILP policy is disabled.
    if item.is_food:
        ok &= base_z >= state.max_chemical_top

    if not ok.any():
        return None
    chemical_support_cells = (
        contact_cells
        & (kind_windows == 1)
        & (item.weight_kg <= mass_windows + 1e-12)
    ).sum(axis=(2, 3))
    valid = np.argwhere(ok)
    x, y = min(
        ((int(index[0]), int(index[1])) for index in valid),
        key=lambda point: (
            -int(chemical_support_cells[point]) if item.is_food else 0,
            int(base_z[point]),
            point[1],
            point[0],
        ),
    )
    z = int(base_z[x, y])
    placement = CoordinatePlacement(
        item=item.index,
        pallet=pallet_index,
        orientation=orientation,
        x=x,
        y=y,
        z=z,
        dx=dx,
        dy=dy,
        dz=dz,
    )
    return (
        -int(chemical_support_cells[x, y]) if item.is_food else 0,
        z,
        y,
        x,
        orientation,
    ), placement


def greedy_coordinate_warm_start(
    context: dict[str, Any],
    items: list[CoordinateItem],
    orientations: dict[int, list[tuple[int, int, int]]],
    config: dict[str, Any],
    max_pallets: int | None = None,
) -> list[CoordinatePlacement] | None:
    """Build a feasible start, opening pallets as needed when no limit is given."""
    pallet = context["pallet"]
    L, W, H = pallet["length"], pallet["width"], pallet["height"]
    payload_limit = float(pallet["payload_kg"])
    support = config.get("support", {"mode": "fraction", "minimum_fraction": 0.75})
    support_mode = str(support.get("mode", "fraction"))
    minimum_fraction = float(support.get("minimum_fraction", 0.75))
    alpha = float(config.get("stacking_mass_alpha", 1.2))
    symmetry = config.get("symmetry", {})
    pallet_limit = len(items) if max_pallets is None else int(max_pallets)
    if pallet_limit <= 0:
        return None

    max_base = {
        i: max(dx * dy for dx, dy, _ in orientations[i]) for i in range(len(items))
    }
    max_height = {
        i: max(dz for _, _, dz in orientations[i]) for i in range(len(items))
    }
    volume = {item.index: math.prod(item.dims) for item in items}
    def phase(item: CoordinateItem) -> int:
        if item.is_chemical:
            return 0
        if item.is_food:
            return 1
        return 2

    # All trials preserve the requested category phases and descending weight;
    # only the geometric tie-break changes to improve the chance of completion.
    order_keys = (
        lambda item: (
            phase(item), -item.weight_kg, -max_base[item.index],
            -volume[item.index], -max_height[item.index], item.index,
        ),
        lambda item: (
            phase(item), -item.weight_kg, -volume[item.index],
            -max_base[item.index], -max_height[item.index], item.index,
        ),
        lambda item: (
            phase(item), -item.weight_kg, -max_height[item.index],
            -max_base[item.index], -volume[item.index], item.index,
        ),
    )
    orderings: list[list[CoordinateItem]] = []
    seen_orders: set[tuple[int, ...]] = set()
    for order_key in order_keys:
        base_order = sorted(items, key=order_key)
        variants = [base_order]
        if symmetry.get("fix_first_item", True):
            # Try a small deterministic set of positions for item zero inside
            # its category phase. This reserves a chance for it on pallet 0
            # without abandoning the chemical -> food -> other phase order.
            anchor = items[0]
            without_anchor = [item for item in base_order if item.index != 0]
            phase_start = next(
                (
                    index
                    for index, candidate in enumerate(without_anchor)
                    if phase(candidate) >= phase(anchor)
                ),
                len(without_anchor),
            )
            phase_size = sum(phase(item) == phase(anchor) for item in without_anchor)
            natural_rank = sum(
                phase(item) == phase(anchor)
                for item in base_order[: base_order.index(anchor)]
            )
            ranks = {
                0,
                phase_size // 4,
                phase_size // 2,
                (3 * phase_size) // 4,
                min(natural_rank, phase_size),
            }
            variants = []
            for rank in sorted(ranks):
                variant = list(without_anchor)
                variant.insert(phase_start + rank, anchor)
                variants.append(variant)
        for variant in variants:
            signature = tuple(item.index for item in variant)
            if signature not in seen_orders:
                seen_orders.add(signature)
                orderings.append(variant)

    best: tuple[tuple[int, int, int], list[CoordinatePlacement]] | None = None

    for ordered_items in orderings:
        states = [
            _WarmStartPallet(
                np.zeros((L, W), dtype=np.int64),
                np.full((L, W), np.inf, dtype=float),
                np.zeros((L, W), dtype=np.int8),
                0.0,
                [],
            )
            for _ in range(pallet_limit)
        ]
        failed = False
        for item in ordered_items:
            chosen: tuple[_WarmStartPallet, CoordinatePlacement] | None = None
            for pallet_index, state in enumerate(states):
                if state.payload_kg + item.weight_kg > payload_limit + 1e-9:
                    continue
                candidates = []
                for orientation, dimensions in enumerate(orientations[item.index]):
                    candidate = _warm_start_candidate(
                        state, item, orientation, dimensions, pallet_index, H,
                        support_mode, minimum_fraction,
                    )
                    if candidate is not None:
                        candidates.append(candidate)
                if candidates:
                    _, placement = min(candidates, key=lambda candidate: candidate[0])
                    chosen = state, placement
                    break
            if chosen is None:
                failed = True
                break
            state, placement = chosen
            state.height[
                placement.x : placement.x + placement.dx,
                placement.y : placement.y + placement.dy,
            ] = placement.top
            state.supporting_mass[
                placement.x : placement.x + placement.dx,
                placement.y : placement.y + placement.dy,
            ] = item.weight_kg
            state.supporting_kind[
                placement.x : placement.x + placement.dx,
                placement.y : placement.y + placement.dy,
            ] = 1 if item.is_chemical else (2 if item.is_food else 0)
            state.payload_kg += item.weight_kg
            state.placements.append(placement)
            if item.is_chemical:
                state.max_chemical_top = max(state.max_chemical_top, placement.top)
        if failed:
            continue

        trial = _canonicalize_warm_start(
            [q for state in states for q in state.placements], items, pallet, symmetry
        )
        if trial is None:
            continue
        fixed_pallet_count = config.get("fixed_pallet_count")
        if (
            fixed_pallet_count is not None
            and len({q.pallet for q in trial}) != int(fixed_pallet_count)
        ):
            continue
        support_arcs = [
            (lower.item, upper.item)
            for lower in trial
            for upper in trial
            if lower.item != upper.item
            and lower.pallet == upper.pallet
            and lower.top == upper.z
            and footprint_overlap(lower, upper) > 0
            and items[upper.item].weight_kg <= alpha * items[lower.item].weight_kg + 1e-12
        ]
        trial_solution = CoordinateSolution(
            "WARM_START", len({q.pallet for q in trial}), 0.0, 0.0, 0.0, trial,
            support_arcs=support_arcs,
        )
        try:
            audit_solution(
                trial_solution,
                context,
                support_mode,
                minimum_fraction,
                items,
                alpha,
                configured_food_chemical_mode(config),
            )
        except RuntimeError:
            continue
        used = sorted({q.pallet for q in trial})
        heights = [max(q.top for q in trial if q.pallet == p) for p in used]
        score = (len(used), max(heights), sum(heights))
        if best is None or score < best[0]:
            best = score, trial
    return None if best is None else best[1]


def prepare_unlimited_coordinate_warm_start(
    context: dict[str, Any],
    items: list[CoordinateItem],
    config: dict[str, Any],
) -> list[CoordinatePlacement] | None:
    """Prepare the reduced-model warm start without a configured pallet cap."""
    L, W, H = (
        context["pallet"]["length"],
        context["pallet"]["width"],
        context["pallet"]["height"],
    )
    rotation_mode = str(config.get("rotation_mode", "yaw"))
    orientations: dict[int, list[tuple[int, int, int]]] = {}
    for item in items:
        feasible = [
            dims
            for dims in allowed_orientations(item, rotation_mode)
            if dims[0] <= L and dims[1] <= W and dims[2] <= H
        ]
        if not feasible:
            return None
        orientations[item.index] = feasible
    return greedy_coordinate_warm_start(
        context, items, orientations, config, max_pallets=None
    )


def audit_coordinate_warm_start(
    placements: list[CoordinatePlacement],
    context: dict[str, Any],
    items: list[CoordinateItem],
    config: dict[str, Any],
) -> None:
    """Validate every structural value needed for a viable reduced MIP start."""
    if {placement.item for placement in placements} != set(range(len(items))):
        raise RuntimeError("warm start does not place every item exactly once")
    used_pallets = sorted({placement.pallet for placement in placements})
    if used_pallets != list(range(len(used_pallets))):
        raise RuntimeError("warm-start pallet indices are not contiguous from zero")

    alpha = float(config.get("stacking_mass_alpha", 1.2))
    support_arcs = [
        (lower.item, upper.item)
        for lower in placements
        for upper in placements
        if lower.item != upper.item
        and lower.pallet == upper.pallet
        and lower.top == upper.z
        and footprint_overlap(lower, upper) > 0
        and items[upper.item].weight_kg
        <= alpha * items[lower.item].weight_kg + 1e-12
    ]
    support_config = config.get(
        "support", {"mode": "fraction", "minimum_fraction": 0.75}
    )
    solution = CoordinateSolution(
        status="WARM_START_AUDIT",
        pallet_count=len(used_pallets),
        objective_bound=0.0,
        mip_gap=0.0,
        runtime_seconds=0.0,
        placements=placements,
        support_arcs=support_arcs,
    )
    audit_solution(
        solution,
        context,
        str(support_config.get("mode", "fraction")),
        float(support_config.get("minimum_fraction", 0.75)),
        items,
        alpha,
        configured_food_chemical_mode(config),
    )

    rotation_mode = str(config.get("rotation_mode", "yaw"))
    L, W, H = (
        context["pallet"]["length"],
        context["pallet"]["width"],
        context["pallet"]["height"],
    )
    for placement in placements:
        allowed = [
            dimensions
            for dimensions in allowed_orientations(items[placement.item], rotation_mode)
            if dimensions[0] <= L and dimensions[1] <= W and dimensions[2] <= H
        ]
        if placement.orientation >= len(allowed):
            raise RuntimeError(f"box {placement.item} has an invalid orientation index")
        if allowed[placement.orientation] != (placement.dx, placement.dy, placement.dz):
            raise RuntimeError(f"box {placement.item} dimensions do not match its orientation")

    # The heuristic promises a stricter physical order than the MILP's alpha
    # rule: every box directly touching another box is no heavier than it.
    for lower in placements:
        for upper in placements:
            if (
                lower.item != upper.item
                and lower.pallet == upper.pallet
                and lower.top == upper.z
                and footprint_overlap(lower, upper) > 0
                and items[upper.item].weight_kg > items[lower.item].weight_kg + 1e-9
            ):
                raise RuntimeError(
                    f"warm-start weight order violation: box {upper.item} is heavier "
                    f"than direct supporter {lower.item}"
                )

    fixed_pallet_count = config.get("fixed_pallet_count")
    if fixed_pallet_count is not None and len(used_pallets) != int(fixed_pallet_count):
        raise RuntimeError("warm-start pallet count differs from fixed_pallet_count")

    symmetry = config.get("symmetry", {})
    if symmetry.get("fix_first_item", True):
        first = next(placement for placement in placements if placement.item == 0)
        if first.pallet != 0:
            raise RuntimeError("warm start violates fix_first_item")
    if symmetry.get("order_pallet_loads", True):
        volumes = [
            sum(
                math.prod(items[q.item].dims)
                for q in placements
                if q.pallet == pallet
            )
            for pallet in used_pallets
        ]
        if any(first < second for first, second in zip(volumes, volumes[1:])):
            raise RuntimeError("warm start violates pallet volume ordering")
    if symmetry.get("order_identical_items", True):
        W, H = context["pallet"]["width"], context["pallet"]["height"]
        by_item = {placement.item: placement for placement in placements}
        for indices in _warm_start_item_groups(items):
            keys = [
                (
                    by_item[index].pallet,
                    by_item[index].x * (W + 1) * (H + 1)
                    + by_item[index].y * (H + 1)
                    + by_item[index].z,
                )
                for index in sorted(indices)
            ]
            if keys != sorted(keys):
                raise RuntimeError("warm start violates identical-item ordering")


def audit_solution(
    solution: CoordinateSolution,
    context: dict[str, Any],
    support_mode: str,
    minimum_fraction: float,
    items: list[CoordinateItem] | None = None,
    stacking_mass_alpha: float | None = None,
    food_chemical_mode: str = "off",
) -> None:
    """Fail fast if extracted integer coordinates violate the modeled geometry."""
    pallet = context["pallet"]
    if len({placement.item for placement in solution.placements}) != len(solution.placements):
        raise RuntimeError("solution extraction contains duplicate boxes")
    for placement in solution.placements:
        if min(placement.x, placement.y, placement.z) < 0:
            raise RuntimeError(f"box {placement.item} has a negative coordinate")
        if placement.x + placement.dx > pallet["length"]:
            raise RuntimeError(f"box {placement.item} exceeds the pallet length")
        if placement.y + placement.dy > pallet["width"]:
            raise RuntimeError(f"box {placement.item} exceeds the pallet width")
        if placement.z + placement.dz > pallet["height"]:
            raise RuntimeError(f"box {placement.item} exceeds the pallet height")

    for index, first in enumerate(solution.placements):
        for second in solution.placements[index + 1 :]:
            if first.pallet != second.pallet:
                continue
            overlap_x = overlap_1d(first.x, first.x + first.dx, second.x, second.x + second.dx)
            overlap_y = overlap_1d(first.y, first.y + first.dy, second.y, second.y + second.dy)
            overlap_z = overlap_1d(first.z, first.z + first.dz, second.z, second.z + second.dz)
            if overlap_x > 0 and overlap_y > 0 and overlap_z > 0:
                raise RuntimeError(f"boxes {first.item} and {second.item} overlap in three dimensions")

    if items is not None:
        capacity = float(pallet.get("payload_kg", math.inf))
        for pallet_index in {placement.pallet for placement in solution.placements}:
            payload = sum(items[p.item].weight_kg for p in solution.placements if p.pallet == pallet_index)
            if payload > capacity + 1e-9:
                raise RuntimeError(
                    f"pallet {pallet_index} payload {payload:.6f} kg exceeds {capacity:.6f} kg"
                )
        if stacking_mass_alpha is not None:
            alpha = float(stacking_mass_alpha)
            placements_by_item = {placement.item: placement for placement in solution.placements}
            for lower_item, upper_item in solution.support_arcs:
                lower = placements_by_item[lower_item]
                upper = placements_by_item[upper_item]
                if lower.pallet != upper.pallet or lower.top != upper.z:
                    raise RuntimeError(
                        f"invalid selected support arc {lower_item}->{upper_item}"
                    )
                if footprint_overlap(lower, upper) <= 0:
                    raise RuntimeError(
                        f"selected support arc {lower_item}->{upper_item} has no footprint overlap"
                    )
                if items[upper_item].weight_kg > alpha * items[lower_item].weight_kg + 1e-9:
                    raise RuntimeError(
                        f"mass-incompatible support: box {lower_item} weighing "
                        f"{items[lower_item].weight_kg:.6f} kg supports box {upper_item} weighing "
                        f"{items[upper_item].weight_kg:.6f} kg with alpha={alpha:.6f}"
                    )
        if food_chemical_mode == "chemical_below_food":
            for chemical in solution.placements:
                if not items[chemical.item].is_chemical:
                    continue
                for food in solution.placements:
                    if (
                        chemical.item != food.item
                        and chemical.pallet == food.pallet
                        and items[food.item].is_food
                        and chemical.top > food.z
                    ):
                        raise RuntimeError(
                            f"chemical/food vertical-order violation: chemical box "
                            f"{chemical.item} with top={chemical.top} extends above "
                            f"food box {food.item} at z={food.z} on pallet {chemical.pallet}"
                        )
        elif food_chemical_mode != "off":
            raise ValueError("food_chemical_mode must be off or chemical_below_food")

    if support_mode == "off":
        return
    required = 1e-12 if support_mode == "direct" else (1.0 if support_mode == "full" else minimum_fraction)
    for placement in solution.placements:
        achieved = support_fraction(placement, solution.placements)
        if achieved + 1e-9 < required:
            raise RuntimeError(
                f"box {placement.item} has support fraction {achieved:.6f}, below {required:.6f}"
            )


class CoordinateBasedMILP:
    """The coordinate formulation documented in the project model overview."""

    def __init__(self, context: dict[str, Any], items: list[CoordinateItem], config: dict[str, Any]):
        self.context = context
        self.items = items
        self.config = config
        self.pallet = context["pallet"]
        self.max_pallets = int(config.get("max_pallets", 2))
        if self.max_pallets <= 0:
            raise ValueError("max_pallets must be positive")

        self.I = range(len(items))
        self.mass = {i: float(items[i].weight_kg) for i in self.I}
        self.P = range(self.max_pallets)
        self.pairs = [(i, j) for i in self.I for j in self.I if i < j]
        self.ordered_pairs = [(i, j) for i in self.I for j in self.I if i != j]
        self.food_chemical_mode = configured_food_chemical_mode(config)
        self.chemical_food_pairs = [
            (i, j)
            for i, j in self.ordered_pairs
            if items[i].is_chemical and items[j].is_food
        ]
        rotation_mode = str(config.get("rotation_mode", "yaw"))
        self.orientations = {i: allowed_orientations(items[i], rotation_mode) for i in self.I}
        self.O = {i: range(len(self.orientations[i])) for i in self.I}

        self.model = gp.Model("coordinate_based_pallet_loading")
        self.model.Params.TimeLimit = float(config.get("time_limit_seconds", 300))
        self.model.Params.MIPGap = float(config.get("mip_gap", 0.0))
        self.model.Params.OutputFlag = int(bool(config.get("log_to_console", True)))

        self._create_core_variables()
        self._build_assignment_orientation_and_bounds()
        self._build_fixed_pallet_count()
        self._build_category_distance_objective()
        self._build_food_chemical_vertical_order()
        self._build_non_overlap_and_overlap_logic()
        self._build_support()
        self._build_symmetry_breaking()
        self._set_primary_objective()
        self.model.update()

    def _create_core_variables(self) -> None:
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        self.used = self.model.addVars(self.P, vtype=GRB.BINARY, name="used")
        self.assign = self.model.addVars(self.I, self.P, vtype=GRB.BINARY, name="assign")
        self.rotate = {
            (i, o, p): self.model.addVar(vtype=GRB.BINARY, name=f"rotate[{i},{o},{p}]")
            for i in self.I for o in self.O[i] for p in self.P
        }
        self.x = self.model.addVars(self.I, self.P, lb=0, ub=L, vtype=GRB.INTEGER, name="x")
        self.y = self.model.addVars(self.I, self.P, lb=0, ub=W, vtype=GRB.INTEGER, name="y")
        self.z = self.model.addVars(self.I, self.P, lb=0, ub=H, vtype=GRB.INTEGER, name="z")
        self.x_end = self.model.addVars(self.I, self.P, lb=0, ub=L, vtype=GRB.INTEGER, name="x_end")
        self.y_end = self.model.addVars(self.I, self.P, lb=0, ub=W, vtype=GRB.INTEGER, name="y_end")
        self.z_end = self.model.addVars(self.I, self.P, lb=0, ub=H, vtype=GRB.INTEGER, name="z_end")
        max_base = {i: max(dx * dy for dx, dy, _ in self.orientations[i]) for i in self.I}
        self.base_area = {
            (i, p): self.model.addVar(lb=0, ub=max_base[i], vtype=GRB.INTEGER, name=f"base[{i},{p}]")
            for i in self.I for p in self.P
        }
        self.max_height = self.model.addVar(
            lb=0, ub=H, vtype=GRB.INTEGER, name="maximum_packing_height"
        )

    def length_expr(self, i: int, p: int):
        return gp.quicksum(self.orientations[i][o][0] * self.rotate[i, o, p] for o in self.O[i])

    def width_expr(self, i: int, p: int):
        return gp.quicksum(self.orientations[i][o][1] * self.rotate[i, o, p] for o in self.O[i])

    def height_expr(self, i: int, p: int):
        return gp.quicksum(self.orientations[i][o][2] * self.rotate[i, o, p] for o in self.O[i])

    def _doubled_center_expr(self, i: int, axis: str):
        """Return twice a box center on its selected pallet."""
        starts = getattr(self, axis)
        ends = getattr(self, f"{axis}_end")
        return gp.quicksum(starts[i, p] + ends[i, p] for p in self.P)

    def top_height_expr(self, i: int):
        """Return the selected box's top-face height in grid units."""
        return gp.quicksum(self.z_end[i, p] for p in self.P)

    def _build_fixed_pallet_count(self) -> None:
        """Optionally require an exact number of nonempty candidate pallets."""
        configured = self.config.get("fixed_pallet_count")
        if configured is None:
            self.fixed_pallet_count = None
            return
        if not isinstance(configured, int) or isinstance(configured, bool):
            raise ValueError("fixed_pallet_count must be an integer")
        self.fixed_pallet_count = configured
        if not 1 <= self.fixed_pallet_count <= self.max_pallets:
            raise ValueError("fixed_pallet_count must be between 1 and max_pallets")
        if self.fixed_pallet_count < self.volume_lower_bound:
            raise ValueError(
                "fixed_pallet_count is below the volume-based pallet lower bound"
            )
        self.model.addConstr(
            gp.quicksum(self.used[p] for p in self.P) == self.fixed_pallet_count,
            name="fixed_pallet_count",
        )

    def _build_category_distance_objective(self) -> None:
        """Build exact pair distances for category-first objective modes."""
        self.category_pairs = [(i, j) for i in self.I for j in self.I if i > j]
        self.delta: dict[tuple[int, int], gp.Var] = {}
        self.same_category_pallet: dict[tuple[int, int], gp.Var] = {}
        self.category_center_difference: dict[tuple[int, int, str], gp.Var] = {}
        self.category_center_absolute_difference: dict[tuple[int, int, str], gp.Var] = {}
        self.category_distance_expr = gp.LinExpr(0.0)
        if configured_objective_mode(self.config) not in CATEGORY_DISTANCE_OBJECTIVE_MODES:
            return

        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        cross_pallet_penalty = L + W + H + 1
        axis_bounds = {"x": 2 * L, "y": 2 * W, "z": 2 * H}
        for i, j in self.category_pairs:
            same_category = self.items[i].retrieval_priority == self.items[j].retrieval_priority
            delta = self.model.addVar(
                lb=0,
                ub=cross_pallet_penalty if same_category else 0,
                vtype=GRB.CONTINUOUS,
                name=f"category_delta[{i},{j}]",
            )
            self.delta[i, j] = delta
            if not same_category:
                continue

            same_pallet = self.model.addVar(
                vtype=GRB.BINARY, name=f"same_category_pallet[{i},{j}]"
            )
            self.same_category_pallet[i, j] = same_pallet
            for p in self.P:
                self.model.addConstr(
                    same_pallet >= self.assign[i, p] + self.assign[j, p] - 1,
                    name=f"same_category_pallet_lb[{i},{j},{p}]",
                )
                self.model.addConstr(
                    same_pallet <= 1 - self.assign[i, p] + self.assign[j, p],
                    name=f"same_category_pallet_i[{i},{j},{p}]",
                )
                self.model.addConstr(
                    same_pallet <= 1 + self.assign[i, p] - self.assign[j, p],
                    name=f"same_category_pallet_j[{i},{j},{p}]",
                )

            absolute_differences = []
            for axis in ("x", "y", "z"):
                bound = axis_bounds[axis]
                difference = self.model.addVar(
                    lb=-bound,
                    ub=bound,
                    vtype=GRB.INTEGER,
                    name=f"category_center_diff2_{axis}[{i},{j}]",
                )
                absolute = self.model.addVar(
                    lb=0,
                    ub=bound,
                    vtype=GRB.INTEGER,
                    name=f"category_center_abs_diff2_{axis}[{i},{j}]",
                )
                self.category_center_difference[i, j, axis] = difference
                self.category_center_absolute_difference[i, j, axis] = absolute
                self.model.addConstr(
                    difference
                    == self._doubled_center_expr(i, axis)
                    - self._doubled_center_expr(j, axis),
                    name=f"category_center_difference_{axis}[{i},{j}]",
                )
                self.model.addGenConstrAbs(
                    absolute,
                    difference,
                    name=f"category_center_absolute_difference_{axis}[{i},{j}]",
                )
                absolute_differences.append(absolute)

            self.model.addGenConstrIndicator(
                same_pallet,
                True,
                2 * delta == gp.quicksum(absolute_differences),
                name=f"category_delta_same_pallet[{i},{j}]",
            )
            self.model.addGenConstrIndicator(
                same_pallet,
                False,
                delta == cross_pallet_penalty,
                name=f"category_delta_different_pallet[{i},{j}]",
            )

        self.category_distance_expr = gp.quicksum(self.delta.values())

    def _build_assignment_orientation_and_bounds(self) -> None:
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        pallet_volume = L * W * H
        item_volume = {i: math.prod(self.items[i].dims) for i in self.I}

        for i in self.I:
            self.model.addConstr(gp.quicksum(self.assign[i, p] for p in self.P) == 1, name=f"assign_once[{i}]")
            for p in self.P:
                self.model.addConstr(
                    gp.quicksum(self.rotate[i, o, p] for o in self.O[i]) == self.assign[i, p],
                    name=f"one_orientation[{i},{p}]",
                )
                self.model.addConstr(self.assign[i, p] <= self.used[p], name=f"activate[{i},{p}]")
                self.model.addConstr(
                    self.x_end[i, p] == self.x[i, p] + self.length_expr(i, p),
                    name=f"right_edge[{i},{p}]",
                )
                self.model.addConstr(
                    self.y_end[i, p] == self.y[i, p] + self.width_expr(i, p),
                    name=f"front_edge[{i},{p}]",
                )
                self.model.addConstr(
                    self.z_end[i, p] == self.z[i, p] + self.height_expr(i, p),
                    name=f"top_edge[{i},{p}]",
                )
                self.model.addConstr(self.x_end[i, p] <= L * self.assign[i, p], name=f"inside_x[{i},{p}]")
                self.model.addConstr(self.y_end[i, p] <= W * self.assign[i, p], name=f"inside_y[{i},{p}]")
                self.model.addConstr(self.z_end[i, p] <= H * self.assign[i, p], name=f"inside_z[{i},{p}]")
                self.model.addConstr(
                    self.base_area[i, p]
                    == gp.quicksum(
                        self.orientations[i][o][0] * self.orientations[i][o][1] * self.rotate[i, o, p]
                        for o in self.O[i]
                    ),
                    name=f"oriented_base[{i},{p}]",
                )
                self.model.addConstr(
                    self.max_height >= self.z_end[i, p],
                    name=f"maximum_height_bound[{i},{p}]",
                )

        for p in self.P:
            self.model.addConstr(
                self.used[p] <= gp.quicksum(self.assign[i, p] for i in self.I),
                name=f"used_has_box[{p}]",
            )
            self.model.addConstr(
                gp.quicksum(item_volume[i] * self.assign[i, p] for i in self.I) <= pallet_volume * self.used[p],
                name=f"volume_capacity[{p}]",
            )
            self.model.addConstr(
                gp.quicksum(self.mass[i] * self.assign[i, p] for i in self.I)
                <= self.pallet["payload_kg"] * self.used[p],
                name=f"payload_capacity[{p}]",
            )

        total_volume = sum(item_volume.values())
        self.volume_lower_bound = max(1, math.ceil(total_volume / pallet_volume))
        if self.volume_lower_bound > self.max_pallets:
            raise ValueError(
                f"at least {self.volume_lower_bound} pallets are required by volume, but max_pallets={self.max_pallets}"
            )
        self.model.addConstr(
            gp.quicksum(self.used[p] for p in self.P) >= self.volume_lower_bound,
            name="volume_pallet_lower_bound",
        )
        for p in range(self.volume_lower_bound):
            self.model.addConstr(self.used[p] == 1, name=f"fix_volume_lb_pallet[{p}]")

    def _build_food_chemical_vertical_order(self) -> None:
        """Keep every chemical's top at or below every food on their shared pallet."""
        self.food_chemical_constraint_count = 0
        if self.food_chemical_mode == "off":
            return
        height = self.pallet["height"]
        for chemical, food in self.chemical_food_pairs:
            for p in self.P:
                self.model.addConstr(
                    self.z[chemical, p] + self.height_expr(chemical, p)
                    <= self.z[food, p]
                    + height * (2 - self.assign[chemical, p] - self.assign[food, p]),
                    name=f"chemical_below_food[{chemical},{food},{p}]",
                )
                self.food_chemical_constraint_count += 1

    def _direction_var(self, name: str) -> dict[tuple[int, int, int], gp.Var]:
        return {
            (i, j, p): self.model.addVar(vtype=GRB.BINARY, name=f"{name}[{i},{j},{p}]")
            for i, j in self.ordered_pairs for p in self.P
        }

    def _build_non_overlap_and_overlap_logic(self) -> None:
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        self.left = self._direction_var("left")
        self.front = self._direction_var("front")
        self.below = self._direction_var("below")

        for i, j in self.ordered_pairs:
            for p in self.P:
                for direction in (self.left, self.front, self.below):
                    self.model.addConstr(direction[i, j, p] <= self.assign[i, p])
                    self.model.addConstr(direction[i, j, p] <= self.assign[j, p])

                self.model.addConstr(
                    self.x_end[i, p] <= self.x[j, p] + L * (1 - self.left[i, j, p]),
                    name=f"left_forward[{i},{j},{p}]",
                )
                self.model.addConstr(
                    self.y_end[i, p] <= self.y[j, p] + W * (1 - self.front[i, j, p]),
                    name=f"front_forward[{i},{j},{p}]",
                )
                self.model.addConstr(
                    self.z_end[i, p] <= self.z[j, p] + H * (1 - self.below[i, j, p]),
                    name=f"below_forward[{i},{j},{p}]",
                )

                # Reverse directions use the one-grid-unit epsilon from the
                # mathematical formulation.  They make each direction binary
                # represent the actual relative position, not just a certificate.
                self.model.addConstr(
                    self.x[j, p]
                    <= self.x_end[i, p] - 1
                    + L * (self.left[i, j, p] + 2 - self.assign[i, p] - self.assign[j, p]),
                    name=f"left_reverse[{i},{j},{p}]",
                )
                self.model.addConstr(
                    self.y[j, p]
                    <= self.y_end[i, p] - 1
                    + W * (self.front[i, j, p] + 2 - self.assign[i, p] - self.assign[j, p]),
                    name=f"front_reverse[{i},{j},{p}]",
                )
                self.model.addConstr(
                    self.z[j, p]
                    <= self.z_end[i, p] - 1
                    + H * (self.below[i, j, p] + 2 - self.assign[i, p] - self.assign[j, p]),
                    name=f"below_reverse[{i},{j},{p}]",
                )

        self.overlap_x: dict[tuple[int, int, int], gp.Var] = {}
        self.overlap_y: dict[tuple[int, int, int], gp.Var] = {}
        self.overlap_xy: dict[tuple[int, int, int], gp.Var] = {}
        for i, j in self.pairs:
            for p in self.P:
                self.model.addConstr(
                    self.left[i, j, p] + self.left[j, i, p]
                    + self.front[i, j, p] + self.front[j, i, p]
                    + self.below[i, j, p] + self.below[j, i, p]
                    >= self.assign[i, p] + self.assign[j, p] - 1,
                    name=f"separate[{i},{j},{p}]",
                )

                ox = self.model.addVar(vtype=GRB.BINARY, name=f"overlap_x[{i},{j},{p}]")
                oy = self.model.addVar(vtype=GRB.BINARY, name=f"overlap_y[{i},{j},{p}]")
                oxy = self.model.addVar(vtype=GRB.BINARY, name=f"overlap_xy[{i},{j},{p}]")
                self.overlap_x[i, j, p], self.overlap_y[i, j, p], self.overlap_xy[i, j, p] = ox, oy, oxy

                for overlap in (ox, oy):
                    self.model.addConstr(overlap <= self.assign[i, p])
                    self.model.addConstr(overlap <= self.assign[j, p])
                self.model.addConstr(ox <= 1 - self.left[i, j, p])
                self.model.addConstr(ox <= 1 - self.left[j, i, p])
                self.model.addConstr(
                    ox >= self.assign[i, p] + self.assign[j, p] - 1
                    - self.left[i, j, p] - self.left[j, i, p]
                )
                self.model.addConstr(oy <= 1 - self.front[i, j, p])
                self.model.addConstr(oy <= 1 - self.front[j, i, p])
                self.model.addConstr(
                    oy >= self.assign[i, p] + self.assign[j, p] - 1
                    - self.front[i, j, p] - self.front[j, i, p]
                )
                self.model.addConstr(oxy <= ox)
                self.model.addConstr(oxy <= oy)
                self.model.addConstr(oxy >= ox + oy - 1)

    def _pair_key(self, i: int, j: int, p: int) -> tuple[int, int, int]:
        return (min(i, j), max(i, j), p)

    def _area_vtype(self) -> str:
        value = str(self.config.get("area_auxiliary_type", "continuous"))
        if value not in {"continuous", "integer"}:
            raise ValueError("area_auxiliary_type must be continuous or integer")
        return GRB.INTEGER if value == "integer" else GRB.CONTINUOUS

    @staticmethod
    def _overlap_at_offset(first_extent: int, second_extent: int, delta: int) -> int:
        """Overlap when the second interval starts delta units after the first."""
        return max(0, min(first_extent, delta + second_extent) - max(0, delta))

    def _add_area_product(self, i: int, j: int, p: int, qx: gp.Var, qy: gp.Var) -> gp.Var:
        """Linearize qx*qy exactly using the binary expansion of integer qx."""
        L, W = self.pallet["length"], self.pallet["width"]
        bit_count = max(1, math.ceil(math.log2(L + 1)))
        bits = [self.model.addVar(vtype=GRB.BINARY, name=f"overlap_x_bit[{i},{j},{p},{k}]") for k in range(bit_count)]
        products = [
            self.model.addVar(lb=0, ub=W, vtype=self._area_vtype(), name=f"overlap_product[{i},{j},{p},{k}]")
            for k in range(bit_count)
        ]
        self.model.addConstr(qx == gp.quicksum((2**k) * bits[k] for k in range(bit_count)))
        for k, (bit, product) in enumerate(zip(bits, products)):
            self.model.addConstr(product <= qy, name=f"product_qy[{i},{j},{p},{k}]")
            self.model.addConstr(product <= W * bit, name=f"product_bit_ub[{i},{j},{p},{k}]")
            self.model.addConstr(product >= qy - W * (1 - bit), name=f"product_bit_lb[{i},{j},{p},{k}]")
        area = self.model.addVar(lb=0, ub=L * W, vtype=self._area_vtype(), name=f"overlap_area[{i},{j},{p}]")
        self.model.addConstr(area == gp.quicksum((2**k) * products[k] for k in range(bit_count)))
        return area

    def _build_overlap_area_compact(self) -> None:
        """Exact rectangle-intersection area using MIN/MAX and binary expansion."""
        L, W = self.pallet["length"], self.pallet["width"]
        self.area: dict[tuple[int, int, int], gp.Var] = {}

        for i, j in self.pairs:
            for p in self.P:
                min_x_end = self.model.addVar(lb=0, ub=L, vtype=GRB.INTEGER, name=f"min_x_end[{i},{j},{p}]")
                max_x_start = self.model.addVar(lb=0, ub=L, vtype=GRB.INTEGER, name=f"max_x_start[{i},{j},{p}]")
                diff_x = self.model.addVar(lb=-L, ub=L, vtype=GRB.INTEGER, name=f"overlap_x_diff[{i},{j},{p}]")
                qx = self.model.addVar(lb=0, ub=L, vtype=GRB.INTEGER, name=f"overlap_x_len[{i},{j},{p}]")
                self.model.addGenConstrMin(min_x_end, [self.x_end[i, p], self.x_end[j, p]])
                self.model.addGenConstrMax(max_x_start, [self.x[i, p], self.x[j, p]])
                self.model.addConstr(diff_x == min_x_end - max_x_start)
                self.model.addGenConstrMax(qx, [diff_x], constant=0.0)

                min_y_end = self.model.addVar(lb=0, ub=W, vtype=GRB.INTEGER, name=f"min_y_end[{i},{j},{p}]")
                max_y_start = self.model.addVar(lb=0, ub=W, vtype=GRB.INTEGER, name=f"max_y_start[{i},{j},{p}]")
                diff_y = self.model.addVar(lb=-W, ub=W, vtype=GRB.INTEGER, name=f"overlap_y_diff[{i},{j},{p}]")
                qy = self.model.addVar(lb=0, ub=W, vtype=GRB.INTEGER, name=f"overlap_y_len[{i},{j},{p}]")
                self.model.addGenConstrMin(min_y_end, [self.y_end[i, p], self.y_end[j, p]])
                self.model.addGenConstrMax(max_y_start, [self.y[i, p], self.y[j, p]])
                self.model.addConstr(diff_y == min_y_end - max_y_start)
                self.model.addGenConstrMax(qy, [diff_y], constant=0.0)

                ox, oy = self.overlap_x[i, j, p], self.overlap_y[i, j, p]
                self.model.addConstr(qx <= L * ox)
                self.model.addConstr(qx >= ox)
                self.model.addConstr(qy <= W * oy)
                self.model.addConstr(qy >= oy)

                self.area[i, j, p] = self._add_area_product(i, j, p, qx, qy)

    def _add_1d_lookup(
        self, i: int, j: int, p: int, axis: str, overlap_binary: gp.Var,
    ) -> gp.Var:
        limit = self.pallet["length" if axis == "x" else "width"]
        starts = self.x if axis == "x" else self.y
        dimension_index = 0 if axis == "x" else 1
        selectors: list[tuple[gp.Var, int, int, int, int]] = []
        for oi in self.O[i]:
            first_extent = self.orientations[i][oi][dimension_index]
            for oj in self.O[j]:
                second_extent = self.orientations[j][oj][dimension_index]
                for delta in range(-(second_extent - 1), first_extent):
                    overlap = self._overlap_at_offset(first_extent, second_extent, delta)
                    if overlap <= 0:
                        continue
                    var = self.model.addVar(
                        vtype=GRB.BINARY,
                        name=f"lookup_{axis}[{i},{j},{p},{oi},{oj},{delta}]",
                    )
                    selectors.append((var, oi, oj, delta, overlap))
        self.model.addConstr(gp.quicksum(v for v, *_ in selectors) == overlap_binary)
        for oi in self.O[i]:
            self.model.addConstr(
                gp.quicksum(v for v, selected_oi, *_ in selectors if selected_oi == oi)
                <= self.rotate[i, oi, p]
            )
        for oj in self.O[j]:
            self.model.addConstr(
                gp.quicksum(v for v, _, selected_oj, *_ in selectors if selected_oj == oj)
                <= self.rotate[j, oj, p]
            )
        selected_delta = gp.quicksum(delta * v for v, _, _, delta, _ in selectors)
        self.model.addConstr(starts[j, p] - starts[i, p] - selected_delta <= limit * (1 - overlap_binary))
        self.model.addConstr(starts[j, p] - starts[i, p] - selected_delta >= -limit * (1 - overlap_binary))
        overlap_length = self.model.addVar(lb=0, ub=limit, vtype=GRB.INTEGER, name=f"lookup_{axis}_length[{i},{j},{p}]")
        self.model.addConstr(overlap_length == gp.quicksum(value * v for v, _, _, _, value in selectors))
        return overlap_length

    def _build_overlap_area_lookup_1d(self) -> None:
        self.area = {}
        for i, j in self.pairs:
            for p in self.P:
                qx = self._add_1d_lookup(i, j, p, "x", self.overlap_x[i, j, p])
                qy = self._add_1d_lookup(i, j, p, "y", self.overlap_y[i, j, p])
                self.area[i, j, p] = self._add_area_product(i, j, p, qx, qy)

    def _build_overlap_area_lookup_2d(self) -> None:
        L, W = self.pallet["length"], self.pallet["width"]
        self.area = {}
        for i, j in self.pairs:
            for p in self.P:
                selectors: list[tuple[gp.Var, int, int, int, int, int]] = []
                for oi in self.O[i]:
                    li, wi, _ = self.orientations[i][oi]
                    for oj in self.O[j]:
                        lj, wj, _ = self.orientations[j][oj]
                        for dx in range(-(lj - 1), li):
                            overlap_x = self._overlap_at_offset(li, lj, dx)
                            for dy in range(-(wj - 1), wi):
                                overlap_y = self._overlap_at_offset(wi, wj, dy)
                                var = self.model.addVar(
                                    vtype=GRB.BINARY,
                                    name=f"lookup_xy[{i},{j},{p},{oi},{oj},{dx},{dy}]",
                                )
                                selectors.append((var, oi, oj, dx, dy, overlap_x * overlap_y))
                oxy = self.overlap_xy[i, j, p]
                self.model.addConstr(gp.quicksum(v for v, *_ in selectors) == oxy)
                for oi in self.O[i]:
                    self.model.addConstr(gp.quicksum(v for v, selected_oi, *_ in selectors if selected_oi == oi) <= self.rotate[i, oi, p])
                for oj in self.O[j]:
                    self.model.addConstr(gp.quicksum(v for v, _, selected_oj, *_ in selectors if selected_oj == oj) <= self.rotate[j, oj, p])
                selected_dx = gp.quicksum(dx * v for v, _, _, dx, _, _ in selectors)
                selected_dy = gp.quicksum(dy * v for v, _, _, _, dy, _ in selectors)
                self.model.addConstr(self.x[j, p] - self.x[i, p] - selected_dx <= L * (1 - oxy))
                self.model.addConstr(self.x[j, p] - self.x[i, p] - selected_dx >= -L * (1 - oxy))
                self.model.addConstr(self.y[j, p] - self.y[i, p] - selected_dy <= W * (1 - oxy))
                self.model.addConstr(self.y[j, p] - self.y[i, p] - selected_dy >= -W * (1 - oxy))
                area = self.model.addVar(lb=0, ub=L * W, vtype=self._area_vtype(), name=f"overlap_area[{i},{j},{p}]")
                self.model.addConstr(area == gp.quicksum(value * v for v, _, _, _, _, value in selectors))
                self.area[i, j, p] = area

    def _build_overlap_area(self) -> None:
        formulation = str(self.config.get("overlap_formulation", "compact"))
        if formulation == "compact":
            self._build_overlap_area_compact()
        elif formulation == "lookup_1d":
            self._build_overlap_area_lookup_1d()
        elif formulation == "lookup_2d":
            self._build_overlap_area_lookup_2d()
        else:
            raise ValueError("overlap_formulation must be compact, lookup_1d, or lookup_2d")

    def _build_support(self) -> None:
        support_config = self.config.get("support", {"mode": "fraction", "minimum_fraction": 0.75})
        mode = str(support_config.get("mode", "fraction"))
        if mode not in {"off", "direct", "fraction", "full"}:
            raise ValueError("support.mode must be off, direct, fraction, or full")
        self.support_mode = mode
        self.floor: dict[tuple[int, int], gp.Var] = {}
        self.contact: dict[tuple[int, int, int], gp.Var] = {}
        self.support_area: dict[tuple[int, int, int], gp.Var] = {}
        if mode == "off":
            return

        H = self.pallet["height"]
        for j in self.I:
            for p in self.P:
                floor = self.model.addVar(vtype=GRB.BINARY, name=f"on_floor[{j},{p}]")
                self.floor[j, p] = floor
                self.model.addConstr(floor <= self.assign[j, p])
                self.model.addConstr(self.z[j, p] <= H * (1 - floor))
                self.model.addConstr(self.z[j, p] >= self.assign[j, p] - floor)

        for i, j in self.ordered_pairs:
            for p in self.P:
                contact = self.model.addVar(vtype=GRB.BINARY, name=f"supports[{i},{j},{p}]")
                self.contact[i, j, p] = contact
                self.model.addConstr(contact <= self.assign[i, p])
                self.model.addConstr(contact <= self.assign[j, p])
                self.model.addConstr(contact <= self.below[i, j, p])
                self.model.addConstr(contact <= self.overlap_xy[self._pair_key(i, j, p)])
                self.model.addConstr(self.z[j, p] - self.z_end[i, p] <= H * (1 - contact))
                self.model.addConstr(self.z_end[i, p] - self.z[j, p] <= H * (1 - contact))

        if mode == "direct":
            for j in self.I:
                for p in self.P:
                    self.model.addConstr(
                        self.floor[j, p] + gp.quicksum(self.contact[i, j, p] for i in self.I if i != j)
                        >= self.assign[j, p],
                        name=f"direct_support[{j},{p}]",
                    )
            return

        self._build_overlap_area()
        max_area = self.pallet["length"] * self.pallet["width"]
        for i, j in self.ordered_pairs:
            for p in self.P:
                area = self.area[self._pair_key(i, j, p)]
                contact = self.contact[i, j, p]
                supported = self.model.addVar(lb=0, ub=max_area, vtype=self._area_vtype(), name=f"support_area[{i},{j},{p}]")
                self.support_area[i, j, p] = supported
                self.model.addConstr(supported <= area)
                self.model.addConstr(supported <= max_area * contact)
                self.model.addConstr(supported >= area - max_area * (1 - contact))

        fraction = 1.0 if mode == "full" else float(support_config.get("minimum_fraction", 0.75))
        if not 0 < fraction <= 1:
            raise ValueError("support.minimum_fraction must lie in (0, 1]")
        rational = Fraction(str(fraction)).limit_denominator(1000)
        numerator, denominator = rational.numerator, rational.denominator
        for j in self.I:
            max_base = max(dx * dy for dx, dy, _ in self.orientations[j])
            for p in self.P:
                self.model.addConstr(
                    denominator * gp.quicksum(self.support_area[i, j, p] for i in self.I if i != j)
                    + numerator * max_base * self.floor[j, p]
                    >= numerator * self.base_area[j, p],
                    name=f"support_fraction[{j},{p}]",
                )

    def _build_symmetry_breaking(self) -> None:
        options = self.config.get("symmetry", {})
        for p in range(self.max_pallets - 1):
            self.model.addConstr(self.used[p] >= self.used[p + 1], name=f"pallet_index_order[{p}]")

        if options.get("order_pallet_loads", True):
            volumes = {i: math.prod(self.items[i].dims) for i in self.I}
            for p in range(self.max_pallets - 1):
                self.model.addConstr(
                    gp.quicksum(volumes[i] * self.assign[i, p] for i in self.I)
                    >= gp.quicksum(volumes[i] * self.assign[i, p + 1] for i in self.I),
                    name=f"pallet_volume_order[{p}]",
                )

        if options.get("fix_first_item", True):
            self.model.addConstr(self.assign[0, 0] == 1, name="first_item_first_pallet")

        if not options.get("order_identical_items", True):
            return
        groups: dict[tuple[Any, ...], list[int]] = defaultdict(list)
        for item in self.items:
            groups[
                (
                    item.sku, item.dims, item.weight_kg, item.family, item.is_food,
                    item.is_chemical, item.fragile, item.upright_only, item.retrieval_priority,
                )
            ].append(item.index)
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        location_m = L * (W + 1) * (H + 1) + W * (H + 1) + H
        for indices in groups.values():
            for first, second in zip(indices, indices[1:]):
                self.model.addConstr(
                    gp.quicksum(p * self.assign[first, p] for p in self.P)
                    <= gp.quicksum(p * self.assign[second, p] for p in self.P),
                    name=f"identical_pallet_order[{first},{second}]",
                )
                for p in self.P:
                    first_location = self.x[first, p] * (W + 1) * (H + 1) + self.y[first, p] * (H + 1) + self.z[first, p]
                    second_location = self.x[second, p] * (W + 1) * (H + 1) + self.y[second, p] * (H + 1) + self.z[second, p]
                    self.model.addConstr(
                        first_location
                        <= second_location + location_m * (2 - self.assign[first, p] - self.assign[second, p]),
                        name=f"identical_position_order[{first},{second},{p}]",
                    )

    def dump_model(self, path: str | Path | None, print_stats: bool) -> None:
        self.model.update()
        if print_stats:
            self.model.printStats()
            print(f"Variables: {self.model.NumVars}")
            print(f"Linear constraints: {self.model.NumConstrs}")
            print(f"General constraints: {self.model.NumGenConstrs}")
            print(f"Box pairs per pallet: {len(self.pairs)}")
        if path:
            output = Path(path)
            output.parent.mkdir(parents=True, exist_ok=True)
            self.model.write(str(output))
            print(f"Wrote MILP: {output}")

    def _set_primary_objective(self) -> None:
        self.pallet_count_expr = gp.quicksum(self.used[p] for p in self.P)
        self.average_top_height_expr = gp.quicksum(
            self.top_height_expr(i) for i in self.I
        ) / len(self.items)
        mode = configured_objective_mode(self.config)
        objective = (
            self.category_distance_expr
            if mode in CATEGORY_DISTANCE_OBJECTIVE_MODES
            else self.pallet_count_expr
        )
        self.model.setObjective(objective, GRB.MINIMIZE)

    def _optimize_lexicographic(self) -> None:
        """Optimize the selected primary objective and any enabled exact tie-breakers."""
        mode = configured_objective_mode(self.config)
        maximize_support_area = configured_support_area_objective(self.config)
        if maximize_support_area and mode not in {
            "pallets_then_max_height", "category_distance_only"
        }:
            raise ValueError(
                "support-area maximization requires objective_mode=pallets_then_max_height "
                "or category_distance_only"
            )
        if maximize_support_area and self.support_mode not in {"fraction", "full"}:
            raise ValueError(
                "support-area maximization requires support.mode=fraction or full"
            )
        total_limit = float(self.config.get("time_limit_seconds", 300))
        staged_average_height = mode == "pallets_then_average_height"
        primary_limit = (
            float(self.config.get("pallet_count_time_limit_seconds", total_limit))
            if staged_average_height
            else total_limit
        )
        started = time.perf_counter()
        self._set_primary_objective()
        self.model.Params.TimeLimit = primary_limit
        self.model.optimize()
        self.primary_objective_bound = float(self.model.ObjBound)
        self.primary_mip_gap = float(self.model.MIPGap) if self.model.SolCount else math.inf
        self.primary_pallet_count = (
            int(round(sum(self.used[p].X for p in self.P))) if self.model.SolCount else None
        )
        self.secondary_optimized = False
        self.secondary_objective_bound = None
        self.secondary_mip_gap = None
        self.tertiary_optimized = False
        self.tertiary_objective_bound = None
        self.tertiary_mip_gap = None
        self.total_support_area_grid2 = 0.0
        self.category_distance_optimized = mode in CATEGORY_DISTANCE_OBJECTIVE_MODES
        self.category_distance_objective_bound = (
            self.primary_objective_bound if self.category_distance_optimized else None
        )
        self.category_distance_mip_gap = (
            self.primary_mip_gap if self.category_distance_optimized else None
        )
        self.total_category_distance_grid = (
            float(sum(variable.X for variable in self.delta.values()))
            if self.category_distance_optimized and self.model.SolCount
            else None
        )
        self.footprint_depth_lower_bound = 0
        self.footprint_height_lower_bound_grid = 0
        height_modes = {
            "pallets_then_max_height",
            "pallets_then_average_height",
            "category_distance_then_max_height",
        }
        height_predecessor_ready = (
            self.model.SolCount
            and (self.model.Status == GRB.OPTIMAL or staged_average_height)
        )
        if mode in height_modes and height_predecessor_ready:
            if mode != "pallets_then_average_height":
                (
                    self.footprint_depth_lower_bound,
                    self.footprint_height_lower_bound_grid,
                ) = footprint_height_lower_bound(
                    self.orientations,
                    self.pallet["length"],
                    self.pallet["width"],
                    self.primary_pallet_count,
                )
            if mode in {"pallets_then_max_height", "pallets_then_average_height"}:
                self.model.addConstr(
                    self.pallet_count_expr == self.primary_pallet_count,
                    name="fix_lexicographic_pallet_count",
                )
            else:
                fixed_distance = round(
                    2 * sum(variable.X for variable in self.delta.values())
                ) / 2
                self.model.addConstr(
                    self.category_distance_expr == fixed_distance,
                    name="fix_lexicographic_category_distance",
                )
            if mode != "pallets_then_average_height":
                self.model.addConstr(
                    self.max_height >= self.footprint_height_lower_bound_grid,
                    name=f"footprint_height_lower_bound[{self.primary_pallet_count}]",
                )
            secondary_limit = (
                float(self.config.get("height_time_limit_seconds", total_limit))
                if staged_average_height
                else max(1e-3, total_limit - (time.perf_counter() - started))
            )
            self.model.Params.TimeLimit = secondary_limit
            secondary_objective = (
                self.average_top_height_expr
                if staged_average_height
                else self.max_height
            )
            self.model.setObjective(secondary_objective, GRB.MINIMIZE)
            self.model.optimize()
            self.secondary_optimized = True
            self.secondary_objective_bound = float(self.model.ObjBound)
            self.secondary_mip_gap = float(self.model.MIPGap) if self.model.SolCount else None

        support_predecessor_optimal = (
            (mode == "pallets_then_max_height" and self.secondary_optimized)
            or (mode == "category_distance_only" and self.category_distance_optimized)
        )
        if (
            maximize_support_area
            and support_predecessor_optimal
            and self.model.SolCount
            and self.model.Status == GRB.OPTIMAL
        ):
            if mode == "pallets_then_max_height":
                fixed_height = int(round(self.max_height.X))
                self.model.addConstr(
                    self.max_height == fixed_height,
                    name="fix_lexicographic_max_height",
                )
            else:
                fixed_distance = round(
                    2 * sum(variable.X for variable in self.delta.values())
                ) / 2
                self.model.addConstr(
                    self.category_distance_expr == fixed_distance,
                    name="fix_lexicographic_category_distance",
                )
            remaining = max(1e-3, total_limit - (time.perf_counter() - started))
            self.model.Params.TimeLimit = remaining
            self.total_support_area_expr = gp.quicksum(self.support_area.values())
            self.model.setObjective(self.total_support_area_expr, GRB.MAXIMIZE)
            self.model.optimize()
            self.tertiary_optimized = True
            self.tertiary_objective_bound = float(self.model.ObjBound)
            self.tertiary_mip_gap = float(self.model.MIPGap) if self.model.SolCount else None
        if self.model.SolCount and self.support_area:
            self.total_support_area_grid2 = float(
                sum(variable.X for variable in self.support_area.values())
            )
        if self.category_distance_optimized and self.model.SolCount:
            self.total_category_distance_grid = float(
                sum(variable.X for variable in self.delta.values())
            )
        self.total_optimization_runtime = time.perf_counter() - started

    def solve(self) -> CoordinateSolution:
        self._optimize_lexicographic()
        if self.model.SolCount == 0:
            status = status_name(self.model.Status)
            raise RuntimeError(f"no feasible solution found; Gurobi status={status} ({self.model.Status})")

        selected: list[CoordinatePlacement] = []
        for i in self.I:
            pallet = next(p for p in self.P if self.assign[i, p].X > 0.5)
            orientation = next(o for o in self.O[i] if self.rotate[i, o, pallet].X > 0.5)
            dx, dy, dz = self.orientations[i][orientation]
            selected.append(
                CoordinatePlacement(
                    item=i,
                    pallet=pallet,
                    orientation=orientation,
                    x=int(round(self.x[i, pallet].X)),
                    y=int(round(self.y[i, pallet].X)),
                    z=int(round(self.z[i, pallet].X)),
                    dx=dx,
                    dy=dy,
                    dz=dz,
                )
            )
        return CoordinateSolution(
            status=status_name(self.model.Status),
            pallet_count=int(round(sum(self.used[p].X for p in self.P))),
            objective_bound=self.primary_objective_bound,
            mip_gap=self.primary_mip_gap,
            runtime_seconds=self.total_optimization_runtime,
            placements=selected,
            max_height_grid=max(placement.top for placement in selected),
            average_top_height_grid=sum(placement.top for placement in selected) / len(selected),
            height_objective_bound_grid=self.secondary_objective_bound,
            height_mip_gap=self.secondary_mip_gap,
            height_stage_attempted=self.secondary_optimized,
            footprint_depth_lower_bound=self.footprint_depth_lower_bound,
            footprint_height_lower_bound_grid=self.footprint_height_lower_bound_grid,
            support_area_grid2=self.total_support_area_grid2,
            support_area_objective_bound_grid2=self.tertiary_objective_bound,
            support_area_mip_gap=self.tertiary_mip_gap,
            support_area_stage_attempted=self.tertiary_optimized,
            objective_mode=configured_objective_mode(self.config),
            category_distance_grid=self.total_category_distance_grid,
            category_distance_objective_bound_grid=self.category_distance_objective_bound,
            category_distance_mip_gap=self.category_distance_mip_gap,
            category_distance_stage_attempted=self.category_distance_optimized,
            fixed_pallet_count=self.fixed_pallet_count,
        )


class ReducedExactCoordinateMILP(CoordinateBasedMILP):
    """Equivalent compact model with pallet-independent local geometry.

    Every box has exactly one local position and orientation. Candidate pallets
    are identical, so pallet indices are needed for assignment and activation,
    not for copies of the same geometric state. Pair geometry is consequently
    represented once and activated only when two boxes share a pallet.
    """

    def __init__(
        self,
        context: dict[str, Any],
        items: list[CoordinateItem],
        config: dict[str, Any],
        prepared_warm_start: list[CoordinatePlacement] | None = None,
    ):
        self.context = context
        self.items = items
        self.config = config
        self.prepared_warm_start = prepared_warm_start
        self.pallet = context["pallet"]
        self.max_pallets = int(config.get("max_pallets", 2))
        if self.max_pallets <= 0:
            raise ValueError("max_pallets must be positive")

        self.I = range(len(items))
        self.mass = {i: float(items[i].weight_kg) for i in self.I}
        self.P = range(self.max_pallets)
        self.pairs = [(i, j) for i in self.I for j in self.I if i < j]
        self.ordered_pairs = [(i, j) for i in self.I for j in self.I if i != j]
        self.food_chemical_mode = configured_food_chemical_mode(config)
        self.chemical_food_pairs = [
            (i, j)
            for i, j in self.ordered_pairs
            if items[i].is_chemical and items[j].is_food
        ]
        alpha = float(config.get("stacking_mass_alpha", 1.2))
        if not math.isfinite(alpha) or alpha < 1.0:
            raise ValueError("stacking_mass_alpha must be finite and at least 1.0")
        self.stacking_mass_alpha = alpha
        self.allowed_support_arcs = [
            (lower, upper)
            for lower, upper in self.ordered_pairs
            if self.mass[upper] <= alpha * self.mass[lower] + 1e-12
        ]
        self.allowed_supporters = {
            upper: [
                lower
                for lower in self.I
                if lower != upper and (lower, upper) in self.allowed_support_arcs
            ]
            for upper in self.I
        }
        self.forbidden_support_arc_count = len(self.ordered_pairs) - len(self.allowed_support_arcs)
        rotation_mode = str(config.get("rotation_mode", "yaw"))
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        self.orientations = {}
        for i in self.I:
            feasible = [
                dims for dims in allowed_orientations(items[i], rotation_mode)
                if dims[0] <= L and dims[1] <= W and dims[2] <= H
            ]
            if not feasible:
                raise ValueError(f"box {items[i].item_id} has no orientation that fits the pallet")
            self.orientations[i] = feasible
        self.O = {i: range(len(self.orientations[i])) for i in self.I}

        self.model = gp.Model("reduced_exact_coordinate_pallet_loading")
        self.model.Params.TimeLimit = float(config.get("time_limit_seconds", 300))
        self.model.Params.MIPGap = float(config.get("mip_gap", 0.0))
        self.model.Params.OutputFlag = int(bool(config.get("log_to_console", True)))

        self._create_reduced_variables()
        self._build_reduced_assignment_and_bounds()
        self._build_fixed_pallet_count()
        self._build_category_distance_objective()
        self._build_food_chemical_vertical_order()
        self._build_reduced_non_overlap()
        self._build_reduced_support()
        self._build_reduced_symmetry()
        self._set_primary_objective()
        self.model.update()
        self.greedy_start_attempted = False
        self.greedy_start_applied = False

    def _create_reduced_variables(self) -> None:
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        self.used = self.model.addVars(self.P, vtype=GRB.BINARY, name="used")
        self.assign = self.model.addVars(self.I, self.P, vtype=GRB.BINARY, name="assign")
        self.rotate = {
            (i, o): self.model.addVar(vtype=GRB.BINARY, name=f"rotate[{i},{o}]")
            for i in self.I for o in self.O[i]
        }
        self.x = self.model.addVars(self.I, lb=0, ub=L, vtype=GRB.INTEGER, name="x")
        self.y = self.model.addVars(self.I, lb=0, ub=W, vtype=GRB.INTEGER, name="y")
        self.z = self.model.addVars(self.I, lb=0, ub=H, vtype=GRB.INTEGER, name="z")
        self.x_end = self.model.addVars(self.I, lb=0, ub=L, vtype=GRB.INTEGER, name="x_end")
        self.y_end = self.model.addVars(self.I, lb=0, ub=W, vtype=GRB.INTEGER, name="y_end")
        self.z_end = self.model.addVars(self.I, lb=0, ub=H, vtype=GRB.INTEGER, name="z_end")
        self.max_height = self.model.addVar(
            lb=0, ub=H, vtype=GRB.INTEGER, name="maximum_packing_height"
        )

    def length_expr(self, i: int):
        return gp.quicksum(self.orientations[i][o][0] * self.rotate[i, o] for o in self.O[i])

    def width_expr(self, i: int):
        return gp.quicksum(self.orientations[i][o][1] * self.rotate[i, o] for o in self.O[i])

    def height_expr(self, i: int):
        return gp.quicksum(self.orientations[i][o][2] * self.rotate[i, o] for o in self.O[i])

    def _doubled_center_expr(self, i: int, axis: str):
        starts = getattr(self, axis)
        ends = getattr(self, f"{axis}_end")
        return starts[i] + ends[i]

    def top_height_expr(self, i: int):
        return self.z_end[i]

    def base_area_expr(self, i: int):
        return gp.quicksum(
            self.orientations[i][o][0] * self.orientations[i][o][1] * self.rotate[i, o]
            for o in self.O[i]
        )

    def _build_reduced_assignment_and_bounds(self) -> None:
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        pallet_volume = L * W * H
        item_volume = {i: math.prod(self.items[i].dims) for i in self.I}
        for i in self.I:
            self.model.addConstr(gp.quicksum(self.assign[i, p] for p in self.P) == 1, name=f"assign_once[{i}]")
            self.model.addConstr(gp.quicksum(self.rotate[i, o] for o in self.O[i]) == 1, name=f"one_orientation[{i}]")
            self.model.addConstr(self.x_end[i] == self.x[i] + self.length_expr(i), name=f"right_edge[{i}]")
            self.model.addConstr(self.y_end[i] == self.y[i] + self.width_expr(i), name=f"front_edge[{i}]")
            self.model.addConstr(self.z_end[i] == self.z[i] + self.height_expr(i), name=f"top_edge[{i}]")
            self.model.addConstr(
                self.max_height >= self.z_end[i], name=f"maximum_height_bound[{i}]"
            )
            for p in self.P:
                self.model.addConstr(self.assign[i, p] <= self.used[p], name=f"activate[{i},{p}]")

        for p in self.P:
            self.model.addConstr(self.used[p] <= gp.quicksum(self.assign[i, p] for i in self.I), name=f"used_has_box[{p}]")
            self.model.addConstr(
                gp.quicksum(item_volume[i] * self.assign[i, p] for i in self.I) <= pallet_volume * self.used[p],
                name=f"volume_capacity[{p}]",
            )
            self.model.addConstr(
                gp.quicksum(self.mass[i] * self.assign[i, p] for i in self.I)
                <= self.pallet["payload_kg"] * self.used[p],
                name=f"payload_capacity[{p}]",
            )

        total_volume = sum(item_volume.values())
        self.volume_lower_bound = max(1, math.ceil(total_volume / pallet_volume))
        if self.volume_lower_bound > self.max_pallets:
            raise ValueError(
                f"at least {self.volume_lower_bound} pallets are required by volume, but max_pallets={self.max_pallets}"
            )
        self.model.addConstr(
            gp.quicksum(self.used[p] for p in self.P) >= self.volume_lower_bound,
            name="volume_pallet_lower_bound",
        )
        for p in range(self.volume_lower_bound):
            self.model.addConstr(self.used[p] == 1, name=f"fix_volume_lb_pallet[{p}]")

    def _build_food_chemical_vertical_order(self) -> None:
        """Reduced-geometry version of the pallet-conditional top-ordering rows."""
        self.food_chemical_constraint_count = 0
        if self.food_chemical_mode == "off":
            return
        height = self.pallet["height"]
        for chemical, food in self.chemical_food_pairs:
            for p in self.P:
                self.model.addConstr(
                    self.z[chemical] + self.height_expr(chemical)
                    <= self.z[food]
                    + height * (2 - self.assign[chemical, p] - self.assign[food, p]),
                    name=f"chemical_below_food[{chemical},{food},{p}]",
                )
                self.food_chemical_constraint_count += 1

    def _global_direction_var(self, name: str) -> dict[tuple[int, int], gp.Var]:
        return {
            (i, j): self.model.addVar(vtype=GRB.BINARY, name=f"{name}[{i},{j}]")
            for i, j in self.ordered_pairs
        }

    def _build_reduced_non_overlap(self) -> None:
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        self.left = self._global_direction_var("left")
        self.front = self._global_direction_var("front")
        self.below = self._global_direction_var("below")
        for i, j in self.ordered_pairs:
            self.model.addConstr(self.x_end[i] <= self.x[j] + L * (1 - self.left[i, j]), name=f"left_forward[{i},{j}]")
            self.model.addConstr(self.x[j] <= self.x_end[i] - 1 + L * self.left[i, j], name=f"left_reverse[{i},{j}]")
            self.model.addConstr(self.y_end[i] <= self.y[j] + W * (1 - self.front[i, j]), name=f"front_forward[{i},{j}]")
            self.model.addConstr(self.y[j] <= self.y_end[i] - 1 + W * self.front[i, j], name=f"front_reverse[{i},{j}]")
            self.model.addConstr(self.z_end[i] <= self.z[j] + H * (1 - self.below[i, j]), name=f"below_forward[{i},{j}]")
            self.model.addConstr(self.z[j] <= self.z_end[i] - 1 + H * self.below[i, j], name=f"below_reverse[{i},{j}]")
        for i, j in self.pairs:
            directions = (
                self.left[i, j] + self.left[j, i] + self.front[i, j] + self.front[j, i]
                + self.below[i, j] + self.below[j, i]
            )
            for p in self.P:
                self.model.addConstr(
                    directions >= self.assign[i, p] + self.assign[j, p] - 1,
                    name=f"separate[{i},{j},{p}]",
                )

    def _build_reduced_overlap_indicators(self) -> None:
        self.overlap_x = {}
        self.overlap_y = {}
        for i, j in self.pairs:
            ox = self.model.addVar(vtype=GRB.BINARY, name=f"overlap_x[{i},{j}]")
            oy = self.model.addVar(vtype=GRB.BINARY, name=f"overlap_y[{i},{j}]")
            self.overlap_x[i, j], self.overlap_y[i, j] = ox, oy
            self.model.addConstr(ox <= 1 - self.left[i, j])
            self.model.addConstr(ox <= 1 - self.left[j, i])
            self.model.addConstr(ox >= 1 - self.left[i, j] - self.left[j, i])
            self.model.addConstr(oy <= 1 - self.front[i, j])
            self.model.addConstr(oy <= 1 - self.front[j, i])
            self.model.addConstr(oy >= 1 - self.front[i, j] - self.front[j, i])

    def _reduced_area_vtype(self) -> str:
        value = str(self.config.get("area_auxiliary_type", "integer"))
        if value not in {"continuous", "integer"}:
            raise ValueError("area_auxiliary_type must be continuous or integer")
        return GRB.INTEGER if value == "integer" else GRB.CONTINUOUS

    def _build_reduced_overlap_area(self) -> None:
        L, W = self.pallet["length"], self.pallet["width"]
        self.area = {}
        for i, j in self.pairs:
            min_x_end = self.model.addVar(lb=0, ub=L, vtype=GRB.INTEGER, name=f"min_x_end[{i},{j}]")
            max_x_start = self.model.addVar(lb=0, ub=L, vtype=GRB.INTEGER, name=f"max_x_start[{i},{j}]")
            diff_x = self.model.addVar(lb=-L, ub=L, vtype=GRB.INTEGER, name=f"overlap_x_diff[{i},{j}]")
            qx = self.model.addVar(lb=0, ub=L, vtype=GRB.INTEGER, name=f"overlap_x_len[{i},{j}]")
            self.model.addGenConstrMin(min_x_end, [self.x_end[i], self.x_end[j]])
            self.model.addGenConstrMax(max_x_start, [self.x[i], self.x[j]])
            self.model.addConstr(diff_x == min_x_end - max_x_start)
            self.model.addGenConstrMax(qx, [diff_x], constant=0.0)

            min_y_end = self.model.addVar(lb=0, ub=W, vtype=GRB.INTEGER, name=f"min_y_end[{i},{j}]")
            max_y_start = self.model.addVar(lb=0, ub=W, vtype=GRB.INTEGER, name=f"max_y_start[{i},{j}]")
            diff_y = self.model.addVar(lb=-W, ub=W, vtype=GRB.INTEGER, name=f"overlap_y_diff[{i},{j}]")
            qy = self.model.addVar(lb=0, ub=W, vtype=GRB.INTEGER, name=f"overlap_y_len[{i},{j}]")
            self.model.addGenConstrMin(min_y_end, [self.y_end[i], self.y_end[j]])
            self.model.addGenConstrMax(max_y_start, [self.y[i], self.y[j]])
            self.model.addConstr(diff_y == min_y_end - max_y_start)
            self.model.addGenConstrMax(qy, [diff_y], constant=0.0)

            max_qx = min(
                max(dims[0] for dims in self.orientations[i]),
                max(dims[0] for dims in self.orientations[j]),
            )
            max_qy = min(
                max(dims[1] for dims in self.orientations[i]),
                max(dims[1] for dims in self.orientations[j]),
            )
            if math.ceil(math.log2(max_qy + 1)) < math.ceil(math.log2(max_qx + 1)):
                binary_length, other_length = qy, qx
                max_binary, max_other = max_qy, max_qx
            else:
                binary_length, other_length = qx, qy
                max_binary, max_other = max_qx, max_qy
            bit_count = max(1, math.ceil(math.log2(max_binary + 1)))
            bits = [
                self.model.addVar(vtype=GRB.BINARY, name=f"overlap_bit[{i},{j},{k}]")
                for k in range(bit_count)
            ]
            products = [
                self.model.addVar(lb=0, ub=max_other, vtype=self._reduced_area_vtype(), name=f"overlap_product[{i},{j},{k}]")
                for k in range(bit_count)
            ]
            self.model.addConstr(binary_length == gp.quicksum((2**k) * bits[k] for k in range(bit_count)))
            for k, (bit, product) in enumerate(zip(bits, products)):
                self.model.addConstr(product <= other_length, name=f"product_other[{i},{j},{k}]")
                self.model.addConstr(product <= max_other * bit, name=f"product_bit_ub[{i},{j},{k}]")
                self.model.addConstr(product >= other_length - max_other * (1 - bit), name=f"product_bit_lb[{i},{j},{k}]")
            area = self.model.addVar(lb=0, ub=max_qx * max_qy, vtype=self._reduced_area_vtype(), name=f"overlap_area[{i},{j}]")
            self.model.addConstr(area == gp.quicksum((2**k) * products[k] for k in range(bit_count)))
            self.area[i, j] = area

    def _same_pallet_contact_constraints(self, contact: gp.Var, i: int, j: int) -> None:
        for p in self.P:
            self.model.addConstr(contact <= 1 - self.assign[i, p] + self.assign[j, p])
            self.model.addConstr(contact <= 1 + self.assign[i, p] - self.assign[j, p])

    def _build_reduced_support(self) -> None:
        support_config = self.config.get("support", {"mode": "fraction", "minimum_fraction": 0.75})
        mode = str(support_config.get("mode", "fraction"))
        if mode not in {"off", "direct", "fraction", "full"}:
            raise ValueError("support.mode must be off, direct, fraction, or full")
        self.support_mode = mode
        self.floor = {}
        self.contact = {}
        self.support_area = {}
        if mode == "off":
            return

        H = self.pallet["height"]
        for j in self.I:
            floor = self.model.addVar(vtype=GRB.BINARY, name=f"on_floor[{j}]")
            self.floor[j] = floor
            self.model.addConstr(self.z[j] <= H * (1 - floor))
            self.model.addConstr(self.z[j] >= 1 - floor)

        if mode == "direct":
            self._build_reduced_overlap_indicators()
        for i, j in self.allowed_support_arcs:
            contact = self.model.addVar(vtype=GRB.BINARY, name=f"supports[{i},{j}]")
            self.contact[i, j] = contact
            self._same_pallet_contact_constraints(contact, i, j)
            self.model.addConstr(self.z[j] - self.z_end[i] <= H * (1 - contact))
            self.model.addConstr(self.z_end[i] - self.z[j] <= H * (1 - contact))
            if mode == "direct":
                key = (min(i, j), max(i, j))
                self.model.addConstr(contact <= self.overlap_x[key])
                self.model.addConstr(contact <= self.overlap_y[key])

        if mode == "direct":
            for j in self.I:
                self.model.addConstr(
                    self.floor[j]
                    + gp.quicksum(self.contact[i, j] for i in self.allowed_supporters[j])
                    >= 1,
                    name=f"direct_support[{j}]",
                )
            return

        self._build_reduced_overlap_area()
        max_area = self.pallet["length"] * self.pallet["width"]
        for i, j in self.allowed_support_arcs:
            area = self.area[min(i, j), max(i, j)]
            contact = self.contact[i, j]
            supported = self.model.addVar(lb=0, ub=max_area, vtype=self._reduced_area_vtype(), name=f"support_area[{i},{j}]")
            self.support_area[i, j] = supported
            self.model.addConstr(supported <= area)
            self.model.addConstr(supported <= max_area * contact)
            self.model.addConstr(supported >= area - max_area * (1 - contact))

        fraction = 1.0 if mode == "full" else float(support_config.get("minimum_fraction", 0.75))
        if not 0 < fraction <= 1:
            raise ValueError("support.minimum_fraction must lie in (0, 1]")
        rational = Fraction(str(fraction)).limit_denominator(1000)
        numerator, denominator = rational.numerator, rational.denominator
        for j in self.I:
            max_base = max(dx * dy for dx, dy, _ in self.orientations[j])
            self.model.addConstr(
                denominator
                * gp.quicksum(self.support_area[i, j] for i in self.allowed_supporters[j])
                + numerator * max_base * self.floor[j]
                >= numerator * self.base_area_expr(j),
                name=f"support_fraction[{j}]",
            )

    def _build_reduced_symmetry(self) -> None:
        options = self.config.get("symmetry", {})
        for p in range(self.max_pallets - 1):
            self.model.addConstr(self.used[p] >= self.used[p + 1], name=f"pallet_index_order[{p}]")
        if options.get("order_pallet_loads", True):
            volumes = {i: math.prod(self.items[i].dims) for i in self.I}
            for p in range(self.max_pallets - 1):
                self.model.addConstr(
                    gp.quicksum(volumes[i] * self.assign[i, p] for i in self.I)
                    >= gp.quicksum(volumes[i] * self.assign[i, p + 1] for i in self.I),
                    name=f"pallet_volume_order[{p}]",
                )
        if options.get("fix_first_item", True):
            self.model.addConstr(self.assign[0, 0] == 1, name="first_item_first_pallet")
        if not options.get("order_identical_items", True):
            return
        groups: dict[tuple[Any, ...], list[int]] = defaultdict(list)
        for item in self.items:
            groups[(item.sku, item.dims, item.weight_kg, item.family, item.is_food, item.is_chemical,
                    item.fragile, item.upright_only, item.retrieval_priority)].append(item.index)
        L, W, H = self.pallet["length"], self.pallet["width"], self.pallet["height"]
        location_m = L * (W + 1) * (H + 1) + W * (H + 1) + H
        for indices in groups.values():
            for first, second in zip(indices, indices[1:]):
                self.model.addConstr(
                    gp.quicksum(p * self.assign[first, p] for p in self.P)
                    <= gp.quicksum(p * self.assign[second, p] for p in self.P),
                    name=f"identical_pallet_order[{first},{second}]",
                )
                first_location = self.x[first] * (W + 1) * (H + 1) + self.y[first] * (H + 1) + self.z[first]
                second_location = self.x[second] * (W + 1) * (H + 1) + self.y[second] * (H + 1) + self.z[second]
                for p in self.P:
                    self.model.addConstr(
                        first_location <= second_location + location_m * (2 - self.assign[first, p] - self.assign[second, p]),
                        name=f"identical_position_order[{first},{second},{p}]",
                    )

    def apply_greedy_start(self) -> bool:
        """Populate a partial MIP start without changing the formulation."""
        if self.greedy_start_attempted:
            return self.greedy_start_applied
        self.greedy_start_attempted = True
        if not self.config.get("warm_start", {}).get("greedy", False):
            return False
        placements = self.prepared_warm_start
        if placements is None:
            placements = greedy_coordinate_warm_start(
                self.context, self.items, self.orientations, self.config,
                max_pallets=None,
            )
        if placements is None:
            return False
        required_pallets = 1 + max(placement.pallet for placement in placements)
        if required_pallets > self.max_pallets:
            # A directly instantiated model cannot gain pallet-indexed
            # variables after construction. solve_instance prepares the start
            # first and expands max_pallets before it builds the model.
            return False
        try:
            audit_coordinate_warm_start(
                placements, self.context, self.items, self.config
            )
        except RuntimeError:
            return False

        by_item = {placement.item: placement for placement in placements}
        used_pallets = {placement.pallet for placement in placements}
        for p in self.P:
            self.used[p].Start = 1.0 if p in used_pallets else 0.0
        for i in self.I:
            placement = by_item[i]
            for p in self.P:
                self.assign[i, p].Start = 1.0 if p == placement.pallet else 0.0
            for o in self.O[i]:
                self.rotate[i, o].Start = 1.0 if o == placement.orientation else 0.0
            self.x[i].Start = placement.x
            self.y[i].Start = placement.y
            self.z[i].Start = placement.z
            self.x_end[i].Start = placement.x + placement.dx
            self.y_end[i].Start = placement.y + placement.dy
            self.z_end[i].Start = placement.top
        self.max_height.Start = max(placement.top for placement in placements)
        self.model.update()
        self.greedy_start_applied = True
        return True

    def solve(self) -> CoordinateSolution:
        self.apply_greedy_start()
        self._optimize_lexicographic()
        if self.model.SolCount == 0:
            status = status_name(self.model.Status)
            raise RuntimeError(f"no feasible solution found; Gurobi status={status} ({self.model.Status})")
        selected = []
        for i in self.I:
            pallet = next(p for p in self.P if self.assign[i, p].X > 0.5)
            orientation = next(o for o in self.O[i] if self.rotate[i, o].X > 0.5)
            dx, dy, dz = self.orientations[i][orientation]
            selected.append(CoordinatePlacement(
                item=i, pallet=pallet, orientation=orientation,
                x=int(round(self.x[i].X)), y=int(round(self.y[i].X)), z=int(round(self.z[i].X)),
                dx=dx, dy=dy, dz=dz,
            ))
        selected_support_arcs = []
        for arc, contact in self.contact.items():
            if contact.X <= 0.5:
                continue
            supported_area = self.support_area.get(arc)
            if supported_area is not None and supported_area.X <= 1e-6:
                continue
            selected_support_arcs.append(arc)
        return CoordinateSolution(
            status=status_name(self.model.Status),
            pallet_count=int(round(sum(self.used[p].X for p in self.P))),
            objective_bound=self.primary_objective_bound,
            mip_gap=self.primary_mip_gap,
            runtime_seconds=self.total_optimization_runtime,
            placements=selected,
            max_height_grid=max(placement.top for placement in selected),
            average_top_height_grid=sum(placement.top for placement in selected) / len(selected),
            height_objective_bound_grid=self.secondary_objective_bound,
            height_mip_gap=self.secondary_mip_gap,
            height_stage_attempted=self.secondary_optimized,
            footprint_depth_lower_bound=self.footprint_depth_lower_bound,
            footprint_height_lower_bound_grid=self.footprint_height_lower_bound_grid,
            support_area_grid2=self.total_support_area_grid2,
            support_area_objective_bound_grid2=self.tertiary_objective_bound,
            support_area_mip_gap=self.tertiary_mip_gap,
            support_area_stage_attempted=self.tertiary_optimized,
            support_arcs=selected_support_arcs,
            objective_mode=configured_objective_mode(self.config),
            category_distance_grid=self.total_category_distance_grid,
            category_distance_objective_bound_grid=self.category_distance_objective_bound,
            category_distance_mip_gap=self.category_distance_mip_gap,
            category_distance_stage_attempted=self.category_distance_optimized,
            fixed_pallet_count=self.fixed_pallet_count,
        )


def status_name(status: int) -> str:
    names = {
        GRB.OPTIMAL: "OPTIMAL",
        GRB.TIME_LIMIT: "TIME_LIMIT",
        GRB.INFEASIBLE: "INFEASIBLE",
        GRB.INF_OR_UNBD: "INF_OR_UNBD",
        GRB.UNBOUNDED: "UNBOUNDED",
        GRB.INTERRUPTED: "INTERRUPTED",
    }
    return names.get(status, f"STATUS_{status}")


def solution_rows(
    solution: CoordinateSolution, items: list[CoordinateItem], context: dict[str, Any]
) -> list[dict[str, Any]]:
    grid = context["grid_mm"]
    by_item = {placement.item: placement for placement in solution.placements}
    rows: list[dict[str, Any]] = []
    for item in items:
        placement = by_item[item.index]
        rows.append(
            {
                "box_index": item.index,
                "box_id": item.id,
                "sku": item.sku,
                "pallet": placement.pallet + 1,
                "orientation": placement.orientation,
                "x_mm": placement.x * grid,
                "y_mm": placement.y * grid,
                "z_mm": placement.z * grid,
                "length_mm": placement.dx * grid,
                "width_mm": placement.dy * grid,
                "height_mm": placement.dz * grid,
                "original_length_mm": item.original_mm[0],
                "original_width_mm": item.original_mm[1],
                "original_height_mm": item.original_mm[2],
                "weight_kg": item.weight_kg,
                "family": item.family,
                "is_food": item.is_food,
                "is_chemical": item.is_chemical,
                "class_label": (
                    "FC" if item.is_food and item.is_chemical
                    else "F" if item.is_food
                    else "C" if item.is_chemical
                    else ""
                ),
                "fragile": item.fragile,
                "retrieval_priority": item.retrieval_priority,
                "support_fraction": support_fraction(placement, solution.placements),
            }
        )
    return rows


_CATEGORY_BASE_COLORS = (
    "#1f77b4",  # priority 1: blue
    "#ff7f0e",  # priority 2: orange
    "#2ca02c",  # priority 3: green
    "#d62728",  # priority 4: red
    "#9467bd",  # priority 5: purple
    "#8c564b",
    "#e377c2",
    "#17becf",
)


def _shade_toward_white(hex_color: str, fraction: float) -> str:
    """Return a lighter shade while retaining the base category hue."""
    fraction = min(1.0, max(0.0, fraction))
    channels = [int(hex_color[offset : offset + 2], 16) for offset in (1, 3, 5)]
    shaded = [round(channel + (255 - channel) * fraction) for channel in channels]
    return "#" + "".join(f"{channel:02x}" for channel in shaded)


def category_type_colors(rows: list[dict[str, Any]]) -> list[str]:
    """Map priority categories to hues and SKUs within each category to shades."""
    skus_by_category: dict[int, list[int]] = {}
    for row in rows:
        priority = int(row["retrieval_priority"])
        skus_by_category.setdefault(priority, []).append(int(row["sku"]))
    skus_by_category = {
        priority: sorted(set(skus)) for priority, skus in skus_by_category.items()
    }

    colors_by_category_sku: dict[tuple[int, int], str] = {}
    for priority, skus in skus_by_category.items():
        base = _CATEGORY_BASE_COLORS[(priority - 1) % len(_CATEGORY_BASE_COLORS)]
        for shade_index, sku in enumerate(skus):
            shade = 0.0 if len(skus) == 1 else 0.42 * shade_index / (len(skus) - 1)
            colors_by_category_sku[priority, sku] = _shade_toward_white(base, shade)
    return [
        colors_by_category_sku[int(row["retrieval_priority"]), int(row["sku"])]
        for row in rows
    ]


def render_solution(
    solution: CoordinateSolution,
    items: list[CoordinateItem],
    context: dict[str, Any],
    path: Path,
) -> None:
    grid = context["grid_mm"]
    pallet = context["pallet"]
    visualization_unit = context.get("visualization_unit", "mm")
    if visualization_unit == "mm":
        scale = 1.0
    elif visualization_unit == "cm":
        scale = 0.1
    else:
        raise ValueError("visualization_unit must be 'mm' or 'cm'")
    rows = solution_rows(solution, items, context)
    positions = [
        (
            ((row["pallet"] - 1) * pallet["length_mm"] + row["x_mm"]) * scale,
            row["y_mm"] * scale,
            row["z_mm"] * scale,
        )
        for row in rows
    ]
    sizes = [
        (row["length_mm"] * scale, row["width_mm"] * scale, row["height_mm"] * scale)
        for row in rows
    ]
    case_ids = np.array([row["sku"] for row in rows])
    figure = _plot_cuboids(
        positions,
        sizes,
        pallet["length_mm"] * solution.pallet_count * scale,
        pallet["width_mm"] * scale,
        pallet["height_mm"] * scale,
        True,
        case_ids,
    )
    box_colors = category_type_colors(rows)
    visible_legend_entries: set[tuple[int, int]] = set()
    for trace, row, color in zip(figure.data, rows, box_colors):
        priority = int(row["retrieval_priority"])
        sku = int(row["sku"])
        legend_key = priority, sku
        trace.update(
            color=color,
            name=f"SKU {sku} · {row['family']}",
            legendgroup=f"priority-{priority}",
            legendgrouptitle_text=f"Priority {priority}",
            showlegend=legend_key not in visible_legend_entries,
            hovertemplate=(
                f"Box {row['box_id']}<br>Priority {priority}<br>SKU {sku}"
                f"<br>Family {row['family']}<br>Class {row['class_label']}<extra></extra>"
            ),
        )
        visible_legend_entries.add(legend_key)
    label_offset = max(0.04 * grid * scale, 0.5)
    label_styles = {
        "F": ("Food (F)", "darkgreen"),
        "C": ("Chemical (C)", "firebrick"),
        "FC": ("Food and chemical (FC)", "darkorange"),
    }
    for label, (trace_name, color) in label_styles.items():
        labelled = [
            (row, position, size)
            for row, position, size in zip(rows, positions, sizes)
            if row["class_label"] == label
        ]
        if not labelled:
            continue
        figure.add_trace(
            go.Scatter3d(
                x=[position[0] + size[0] / 2 for _, position, size in labelled],
                y=[position[1] + size[1] / 2 for _, position, size in labelled],
                z=[position[2] + size[2] + label_offset for _, position, size in labelled],
                mode="text",
                text=[label] * len(labelled),
                textfont={"color": color, "size": 18},
                name=trace_name,
                hovertext=[
                    f"Box {row['box_id']} · SKU {row['sku']} · {trace_name}"
                    for row, _, _ in labelled
                ],
                hoverinfo="text",
            )
        )
    for p in range(solution.pallet_count):
        left = p * pallet["length_mm"] * scale
        right = (p + 1) * pallet["length_mm"] * scale
        figure.add_trace(
            go.Scatter3d(
                x=[left, right, right, left, left],
                y=[0, 0, pallet["width_mm"] * scale, pallet["width_mm"] * scale, 0],
                z=[0, 0, 0, 0, 0],
                mode="lines",
                name=f"Pallet {p + 1}",
                line={"color": "red", "width": 6},
            )
        )
    figure.update_layout(
        title=(
            f"Coordinate MILP: pallets={solution.pallet_count}, status={solution.status}, "
            f"gap={100 * solution.mip_gap:.2f}% · hue=priority, shade=SKU/type"
        ),
        scene={
            "aspectmode": "data",
            "xaxis_title": f"x ({visualization_unit})",
            "yaxis_title": f"y ({visualization_unit})",
            "zaxis_title": f"z ({visualization_unit})",
        },
    )
    figure.write_html(path)


def write_outputs(
    solution: CoordinateSolution,
    items: list[CoordinateItem],
    context: dict[str, Any],
    output_dir: Path,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    rows = solution_rows(solution, items, context)
    stem = f"solution_000_p{solution.pallet_count}"

    with (output_dir / f"{stem}.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    pallet = context["pallet"]
    total_volume = sum(placement.dx * placement.dy * placement.dz for placement in solution.placements)
    available_volume = solution.pallet_count * pallet["length"] * pallet["width"] * pallet["height"]
    pallet_payloads = {
        str(pallet_index + 1): sum(
            items[placement.item].weight_kg
            for placement in solution.placements
            if placement.pallet == pallet_index
        )
        for pallet_index in range(solution.pallet_count)
    }
    payload = {
        "instance": context.get("payload", {}).get("meta", {}).get("name", "unknown"),
        "formulation": "coordinate-based pairwise non-overlap MILP",
        "metrics": {
            "status": solution.status,
            "objective_mode": solution.objective_mode,
            "fixed_pallet_count": solution.fixed_pallet_count,
            "pallet_count": solution.pallet_count,
            "max_height_mm": solution.max_height_grid * context["grid_mm"],
            "average_top_height_mm": solution.average_top_height_grid * context["grid_mm"],
            "footprint_depth_lower_bound": solution.footprint_depth_lower_bound,
            "footprint_height_lower_bound_mm": (
                solution.footprint_height_lower_bound_grid * context["grid_mm"]
            ),
            "height_objective_bound_mm": (
                solution.height_objective_bound_grid * context["grid_mm"]
                if solution.height_objective_bound_grid is not None else None
            ),
            "height_mip_gap": solution.height_mip_gap,
            "height_stage_attempted": solution.height_stage_attempted,
            "support_area_mm2": solution.support_area_grid2 * context["grid_mm"] ** 2,
            "support_area_objective_bound_mm2": (
                solution.support_area_objective_bound_grid2 * context["grid_mm"] ** 2
                if solution.support_area_objective_bound_grid2 is not None else None
            ),
            "support_area_mip_gap": solution.support_area_mip_gap,
            "support_area_stage_attempted": solution.support_area_stage_attempted,
            "support_area_objective_enabled": context.get(
                "support_area_objective_enabled", False
            ),
            "category_distance_grid": solution.category_distance_grid,
            "category_distance_mm": (
                solution.category_distance_grid * context["grid_mm"]
                if solution.category_distance_grid is not None else None
            ),
            "category_distance_objective_bound_grid": (
                solution.category_distance_objective_bound_grid
            ),
            "category_distance_objective_bound_mm": (
                solution.category_distance_objective_bound_grid * context["grid_mm"]
                if solution.category_distance_objective_bound_grid is not None else None
            ),
            "category_distance_mip_gap": solution.category_distance_mip_gap,
            "category_distance_stage_attempted": solution.category_distance_stage_attempted,
            "volume_utilization": total_volume / available_volume,
            "objective_bound": solution.objective_bound,
            "mip_gap": solution.mip_gap,
            "runtime_seconds": solution.runtime_seconds,
            "grid_mm": context["grid_mm"],
            "pallet_payload_capacity_kg": pallet["payload_kg"],
            "pallet_payloads_kg": pallet_payloads,
            "stacking_mass_alpha": context.get("stacking_mass_alpha", 1.2),
            "food_chemical_mode": context.get("food_chemical_mode", "off"),
            "selected_support_arc_count": len(solution.support_arcs),
        },
        "support_arcs": [
            {
                "lower_box_index": lower,
                "lower_box_id": items[lower].id,
                "lower_mass_kg": items[lower].weight_kg,
                "upper_box_index": upper,
                "upper_box_id": items[upper].id,
                "upper_mass_kg": items[upper].weight_kg,
            }
            for lower, upper in solution.support_arcs
        ],
        "placements": rows,
    }
    (output_dir / f"{stem}.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow([
            "status", "objective_mode", "fixed_pallet_count", "pallet_count",
            "max_height_mm", "average_top_height_mm", "footprint_depth_lower_bound",
            "footprint_height_lower_bound_mm", "height_objective_bound_mm",
            "height_mip_gap", "height_stage_attempted", "volume_utilization",
            "support_area_mm2", "support_area_objective_bound_mm2",
            "support_area_mip_gap", "support_area_stage_attempted",
            "support_area_objective_enabled",
            "category_distance_grid", "category_distance_mm",
            "category_distance_objective_bound_grid",
            "category_distance_objective_bound_mm", "category_distance_mip_gap",
            "category_distance_stage_attempted",
            "objective_bound", "mip_gap", "runtime_seconds",
        ])
        writer.writerow(
            [
                solution.status,
                solution.objective_mode,
                solution.fixed_pallet_count,
                solution.pallet_count,
                payload["metrics"]["max_height_mm"],
                payload["metrics"]["average_top_height_mm"],
                solution.footprint_depth_lower_bound,
                payload["metrics"]["footprint_height_lower_bound_mm"],
                payload["metrics"]["height_objective_bound_mm"],
                solution.height_mip_gap,
                solution.height_stage_attempted,
                payload["metrics"]["volume_utilization"],
                payload["metrics"]["support_area_mm2"],
                payload["metrics"]["support_area_objective_bound_mm2"],
                solution.support_area_mip_gap,
                solution.support_area_stage_attempted,
                payload["metrics"]["support_area_objective_enabled"],
                solution.category_distance_grid,
                payload["metrics"]["category_distance_mm"],
                solution.category_distance_objective_bound_grid,
                payload["metrics"]["category_distance_objective_bound_mm"],
                solution.category_distance_mip_gap,
                solution.category_distance_stage_attempted,
                solution.objective_bound,
                solution.mip_gap,
                solution.runtime_seconds,
            ]
        )
    render_solution(solution, items, context, output_dir / f"{stem}.html")


def instance_config(
    base_config: dict[str, Any],
    input_path: Path,
    time_limit: float | None,
    max_pallets: int | None,
) -> dict[str, Any]:
    config = copy.deepcopy(base_config)
    if time_limit is not None:
        config["time_limit_seconds"] = time_limit
    if max_pallets is not None:
        config["max_pallets"] = max_pallets
    else:
        payload = json.loads(input_path.read_text(encoding="utf-8"))
        recommended = recommended_max_pallets(payload)
        if recommended is not None:
            config["max_pallets"] = max(int(config.get("max_pallets", 1)), recommended)
    return config


def solve_instance(
    input_path: Path,
    config: dict[str, Any],
    output_dir: Path,
    model_dump: str | Path | None,
    print_model: bool,
) -> tuple[CoordinateSolution, dict[str, Any], list[CoordinateItem], dict[str, int]]:
    context, items = read_mcpp_json(input_path, config)
    started = time.perf_counter()
    model_variant = str(config.get("model_variant", "legacy"))
    prepared_warm_start = None
    if (
        model_variant == "reduced_exact"
        and config.get("warm_start", {}).get("greedy", False)
    ):
        prepared_warm_start = prepare_unlimited_coordinate_warm_start(
            context, items, config
        )
        if prepared_warm_start is not None:
            required_pallets = 1 + max(
                placement.pallet for placement in prepared_warm_start
            )
            fixed_pallet_count = config.get("fixed_pallet_count")
            if fixed_pallet_count is None:
                config["max_pallets"] = max(
                    int(config.get("max_pallets", 1)), required_pallets
                )
            elif int(fixed_pallet_count) != required_pallets:
                prepared_warm_start = None
    if model_variant == "legacy":
        exact = CoordinateBasedMILP(context, items, config)
    elif model_variant == "reduced_exact":
        exact = ReducedExactCoordinateMILP(
            context, items, config, prepared_warm_start=prepared_warm_start
        )
    else:
        raise ValueError("model_variant must be legacy or reduced_exact")
    try:
        exact.dump_model(model_dump, print_model)
        stats = {
            "variables": exact.model.NumVars,
            "linear_constraints": exact.model.NumConstrs,
            "general_constraints": exact.model.NumGenConstrs,
            "food_chemical_constraints": exact.food_chemical_constraint_count,
        }
        if model_variant == "reduced_exact":
            stats["allowed_support_arcs"] = len(exact.allowed_support_arcs)
            stats["forbidden_support_arcs"] = exact.forbidden_support_arc_count
        solution = exact.solve()
        support_config = config.get("support", {"mode": "fraction", "minimum_fraction": 0.75})
        audit_solution(
            solution,
            context,
            str(support_config.get("mode", "fraction")),
            float(support_config.get("minimum_fraction", 0.75)),
            items,
            float(config.get("stacking_mass_alpha", 1.2)) if model_variant == "reduced_exact" else None,
            configured_food_chemical_mode(config),
        )
        solution.runtime_seconds = time.perf_counter() - started
        write_outputs(solution, items, context, output_dir)
        return solution, context, items, stats
    finally:
        exact.model.dispose()


def batch_lp_path(requested: str | None, instance_stem: str) -> Path | None:
    if requested is None:
        return None
    path = Path(requested)
    if path.suffix:
        return path.with_name(f"{path.stem}_{instance_stem}{path.suffix}")
    return path / instance_stem / "model.lp"


def write_batch_summary(records: list[dict[str, Any]], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fields = [
        "instance",
        "input_file",
        "status",
        "objective_mode",
        "fixed_pallet_count",
        "n_boxes",
        "max_pallets",
        "pallet_count",
        "max_height_mm",
        "average_top_height_mm",
        "footprint_depth_lower_bound",
        "footprint_height_lower_bound_mm",
        "height_objective_bound_mm",
        "height_mip_gap",
        "height_stage_attempted",
        "support_area_mm2",
        "support_area_objective_bound_mm2",
        "support_area_mip_gap",
        "support_area_stage_attempted",
        "support_area_objective_enabled",
        "category_distance_grid",
        "category_distance_mm",
        "category_distance_objective_bound_grid",
        "category_distance_objective_bound_mm",
        "category_distance_mip_gap",
        "category_distance_stage_attempted",
        "objective_bound",
        "mip_gap",
        "runtime_seconds",
        "variables",
        "linear_constraints",
        "general_constraints",
        "allowed_support_arcs",
        "forbidden_support_arcs",
        "food_chemical_constraints",
        "output_directory",
        "error_type",
        "error",
    ]
    with (output_dir / "batch_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows({field: record.get(field, "") for field in fields} for record in records)
    (output_dir / "batch_summary.json").write_text(json.dumps(records, indent=2), encoding="utf-8")


def run_batch(args: argparse.Namespace, base_config: dict[str, Any]) -> int:
    input_dir = Path(args.input_dir)
    instances = sorted(path for path in input_dir.glob(args.pattern) if path.is_file())
    if not instances:
        raise ValueError(f"no files matching {args.pattern!r} found in {input_dir}")

    output_root = Path(args.output_dir)
    records: list[dict[str, Any]] = []
    failures = 0
    for number, input_path in enumerate(instances, start=1):
        name = input_path.stem
        instance_output = output_root / name
        config = instance_config(base_config, input_path, args.time_limit, args.max_pallets)
        started = time.perf_counter()
        print(f"[{number}/{len(instances)}] {name}")
        try:
            solution, context, items, stats = solve_instance(
                input_path,
                config,
                instance_output,
                batch_lp_path(args.write_lp, name),
                args.print_model,
            )
            record = {
                "instance": context.get("payload", {}).get("meta", {}).get("name", name),
                "input_file": str(input_path),
                "status": solution.status,
                "objective_mode": solution.objective_mode,
                "fixed_pallet_count": solution.fixed_pallet_count,
                "n_boxes": len(items),
                "max_pallets": int(config["max_pallets"]),
                "pallet_count": solution.pallet_count,
                "max_height_mm": solution.max_height_grid * context["grid_mm"],
                "average_top_height_mm": solution.average_top_height_grid * context["grid_mm"],
                "footprint_depth_lower_bound": solution.footprint_depth_lower_bound,
                "footprint_height_lower_bound_mm": (
                    solution.footprint_height_lower_bound_grid * context["grid_mm"]
                ),
                "height_objective_bound_mm": (
                    solution.height_objective_bound_grid * context["grid_mm"]
                    if solution.height_objective_bound_grid is not None else ""
                ),
                "height_mip_gap": (
                    solution.height_mip_gap if solution.height_mip_gap is not None else ""
                ),
                "height_stage_attempted": solution.height_stage_attempted,
                "support_area_mm2": solution.support_area_grid2 * context["grid_mm"] ** 2,
                "support_area_objective_bound_mm2": (
                    solution.support_area_objective_bound_grid2 * context["grid_mm"] ** 2
                    if solution.support_area_objective_bound_grid2 is not None else ""
                ),
                "support_area_mip_gap": (
                    solution.support_area_mip_gap
                    if solution.support_area_mip_gap is not None else ""
                ),
                "support_area_stage_attempted": solution.support_area_stage_attempted,
                "support_area_objective_enabled": configured_support_area_objective(config),
                "category_distance_grid": (
                    solution.category_distance_grid
                    if solution.category_distance_grid is not None else ""
                ),
                "category_distance_mm": (
                    solution.category_distance_grid * context["grid_mm"]
                    if solution.category_distance_grid is not None else ""
                ),
                "category_distance_objective_bound_grid": (
                    solution.category_distance_objective_bound_grid
                    if solution.category_distance_objective_bound_grid is not None else ""
                ),
                "category_distance_objective_bound_mm": (
                    solution.category_distance_objective_bound_grid * context["grid_mm"]
                    if solution.category_distance_objective_bound_grid is not None else ""
                ),
                "category_distance_mip_gap": (
                    solution.category_distance_mip_gap
                    if solution.category_distance_mip_gap is not None else ""
                ),
                "category_distance_stage_attempted": solution.category_distance_stage_attempted,
                "objective_bound": solution.objective_bound,
                "mip_gap": solution.mip_gap,
                "runtime_seconds": solution.runtime_seconds,
                **stats,
                "output_directory": str(instance_output),
                "error_type": "",
                "error": "",
            }
            height_gap = (
                f"{100 * solution.height_mip_gap:.3f}%"
                if solution.height_mip_gap is not None else "n/a"
            )
            objective_detail = (
                f"category_distance={solution.category_distance_grid:.3f} grid units, "
                if solution.category_distance_grid is not None
                else f"height={solution.max_height_grid * context['grid_mm']} mm, "
            )
            print(
                f"  {solution.status}: pallets={solution.pallet_count}, "
                f"primary_gap={100 * solution.mip_gap:.3f}%, "
                f"{objective_detail}"
                f"height_gap={height_gap}, runtime={solution.runtime_seconds:.2f}s"
            )
        except Exception as exc:  # Continue so one difficult instance does not lose the batch report.
            failures += 1
            instance_output.mkdir(parents=True, exist_ok=True)
            error_payload = {
                "instance": name,
                "input_file": str(input_path),
                "error_type": type(exc).__name__,
                "error": str(exc),
            }
            (instance_output / "error.json").write_text(json.dumps(error_payload, indent=2), encoding="utf-8")
            record = {
                "instance": name,
                "input_file": str(input_path),
                "status": "ERROR",
                "objective_mode": str(config.get("objective_mode", "pallet_count_only")),
                "fixed_pallet_count": config.get("fixed_pallet_count", ""),
                "n_boxes": "",
                "max_pallets": int(config.get("max_pallets", 0)),
                "pallet_count": "",
                "max_height_mm": "",
                "average_top_height_mm": "",
                "footprint_depth_lower_bound": "",
                "footprint_height_lower_bound_mm": "",
                "height_objective_bound_mm": "",
                "height_mip_gap": "",
                "height_stage_attempted": "",
                "support_area_mm2": "",
                "support_area_objective_bound_mm2": "",
                "support_area_mip_gap": "",
                "support_area_stage_attempted": "",
                "support_area_objective_enabled": configured_support_area_objective(config),
                "category_distance_grid": "",
                "category_distance_mm": "",
                "category_distance_objective_bound_grid": "",
                "category_distance_objective_bound_mm": "",
                "category_distance_mip_gap": "",
                "category_distance_stage_attempted": "",
                "objective_bound": "",
                "mip_gap": "",
                "runtime_seconds": time.perf_counter() - started,
                "variables": "",
                "linear_constraints": "",
                "general_constraints": "",
                "output_directory": str(instance_output),
                "error_type": type(exc).__name__,
                "error": str(exc),
            }
            print(f"  ERROR ({type(exc).__name__}): {exc}")
        records.append(record)
        write_batch_summary(records, output_root)
        if failures and args.fail_fast:
            break

    print(f"Batch complete: {len(records) - failures} solved, {failures} failed; wrote {output_root}")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--input", help="one MCPP/PL-100 JSON instance")
    source.add_argument("--input-dir", help="directory containing JSON instances for a batch run")
    parser.add_argument("--pattern", default="*.json", help="batch filename pattern (default: *.json)")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="coordinate-model JSON configuration")
    parser.add_argument("--output-dir", default="output/gurobi_coordinate", help="result directory")
    parser.add_argument("--time-limit", type=float, default=None, help="override the configuration time limit")
    parser.add_argument("--max-pallets", type=int, default=None, help="override the maximum candidate pallets")
    parser.add_argument("--write-lp", default=None, metavar="PATH", help="write the instantiated MILP before solving")
    parser.add_argument("--print-model", action="store_true", help="print Gurobi model statistics")
    parser.add_argument("--fail-fast", action="store_true", help="stop a batch after the first failed instance")
    args = parser.parse_args()

    base_config = load_config(args.config)
    if args.input_dir:
        return run_batch(args, base_config)

    input_path = Path(args.input)
    config = instance_config(base_config, input_path, args.time_limit, args.max_pallets)
    output_dir = Path(args.output_dir)
    solution, context, _, _ = solve_instance(
        input_path,
        config,
        output_dir,
        args.write_lp,
        args.print_model,
    )
    if solution.category_distance_grid is not None:
        objective_detail = f"category_distance={solution.category_distance_grid:.3f} grid units; "
    elif solution.objective_mode == "pallets_then_average_height":
        objective_detail = (
            f"average_top_height={solution.average_top_height_grid * context['grid_mm']:.3f} mm; "
        )
    else:
        objective_detail = f"max_height={solution.max_height_grid * context['grid_mm']} mm; "
    print(
        f"Status={solution.status}; pallets={solution.pallet_count}; "
        f"primary_gap={100 * solution.mip_gap:.3f}%; "
        f"{objective_detail}"
        f"height_gap="
        + (
            f"{100 * solution.height_mip_gap:.3f}%"
            if solution.height_mip_gap is not None else "n/a"
        )
        + f"; runtime={solution.runtime_seconds:.2f}s"
    )
    print(f"Wrote {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
