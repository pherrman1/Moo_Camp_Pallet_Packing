"""Coordinate-based exact MILP for the intelligent pallet-loading project.

The formulation uses one set of assignment, orientation, and integer-coordinate
variables per physical box and candidate pallet.  It intentionally does not use
position-indexed placement binaries.  The active objective is the number of
used pallets; the project attributes remain in the input and output for later
multi-objective extensions.
"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import math
import time
from collections import defaultdict
from dataclasses import asdict, dataclass
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


def load_config(path: str | Path | None) -> dict[str, Any]:
    config_path = Path(path) if path else DEFAULT_CONFIG
    return json.loads(config_path.read_text(encoding="utf-8"))


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
    grid = int(config.get("grid_mm", 50))
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
    }
    if min(pallet["length"], pallet["width"], pallet["height"]) <= 0:
        raise ValueError("pallet dimensions must be at least one grid unit")

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
    return {"payload": payload, "pallet": pallet, "grid_mm": grid}, items


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


def audit_solution(
    solution: CoordinateSolution,
    context: dict[str, Any],
    support_mode: str,
    minimum_fraction: float,
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
        self.P = range(self.max_pallets)
        self.pairs = [(i, j) for i in self.I for j in self.I if i < j]
        self.ordered_pairs = [(i, j) for i in self.I for j in self.I if i != j]
        rotation_mode = str(config.get("rotation_mode", "yaw"))
        self.orientations = {i: allowed_orientations(items[i], rotation_mode) for i in self.I}
        self.O = {i: range(len(self.orientations[i])) for i in self.I}

        self.model = gp.Model("coordinate_based_pallet_loading")
        self.model.Params.TimeLimit = float(config.get("time_limit_seconds", 300))
        self.model.Params.MIPGap = float(config.get("mip_gap", 0.0))
        self.model.Params.OutputFlag = int(bool(config.get("log_to_console", True)))

        self._create_core_variables()
        self._build_assignment_orientation_and_bounds()
        self._build_non_overlap_and_overlap_logic()
        self._build_support()
        self._build_symmetry_breaking()
        self.model.setObjective(gp.quicksum(self.used[p] for p in self.P), GRB.MINIMIZE)
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

    def length_expr(self, i: int, p: int):
        return gp.quicksum(self.orientations[i][o][0] * self.rotate[i, o, p] for o in self.O[i])

    def width_expr(self, i: int, p: int):
        return gp.quicksum(self.orientations[i][o][1] * self.rotate[i, o, p] for o in self.O[i])

    def height_expr(self, i: int, p: int):
        return gp.quicksum(self.orientations[i][o][2] * self.rotate[i, o, p] for o in self.O[i])

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

        for p in self.P:
            self.model.addConstr(
                self.used[p] <= gp.quicksum(self.assign[i, p] for i in self.I),
                name=f"used_has_box[{p}]",
            )
            self.model.addConstr(
                gp.quicksum(item_volume[i] * self.assign[i, p] for i in self.I) <= pallet_volume * self.used[p],
                name=f"volume_capacity[{p}]",
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

    def _build_overlap_area(self) -> None:
        """Exact rectangle-intersection area using integer-grid binary expansion."""
        L, W = self.pallet["length"], self.pallet["width"]
        bit_count = max(1, math.ceil(math.log2(L + 1)))
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

                bits = [self.model.addVar(vtype=GRB.BINARY, name=f"overlap_x_bit[{i},{j},{p},{k}]") for k in range(bit_count)]
                products = [
                    self.model.addVar(lb=0, ub=W, vtype=GRB.CONTINUOUS, name=f"overlap_product[{i},{j},{p},{k}]")
                    for k in range(bit_count)
                ]
                self.model.addConstr(qx == gp.quicksum((2**k) * bits[k] for k in range(bit_count)))
                for k, (bit, product) in enumerate(zip(bits, products)):
                    self.model.addConstr(product <= qy, name=f"product_qy[{i},{j},{p},{k}]")
                    self.model.addConstr(product <= W * bit, name=f"product_bit_ub[{i},{j},{p},{k}]")
                    self.model.addConstr(product >= qy - W * (1 - bit), name=f"product_bit_lb[{i},{j},{p},{k}]")
                area = self.model.addVar(lb=0, ub=L * W, vtype=GRB.CONTINUOUS, name=f"overlap_area[{i},{j},{p}]")
                self.model.addConstr(area == gp.quicksum((2**k) * products[k] for k in range(bit_count)))
                self.area[i, j, p] = area

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
                supported = self.model.addVar(lb=0, ub=max_area, vtype=GRB.CONTINUOUS, name=f"support_area[{i},{j},{p}]")
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

    def solve(self) -> CoordinateSolution:
        self.model.optimize()
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
        gap = float(self.model.MIPGap) if self.model.IsMIP else 0.0
        return CoordinateSolution(
            status=status_name(self.model.Status),
            pallet_count=int(round(sum(self.used[p].X for p in self.P))),
            objective_bound=float(self.model.ObjBound),
            mip_gap=gap,
            runtime_seconds=float(self.model.Runtime),
            placements=selected,
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
                "fragile": item.fragile,
                "retrieval_priority": item.retrieval_priority,
                "support_fraction": support_fraction(placement, solution.placements),
            }
        )
    return rows


def render_solution(
    solution: CoordinateSolution,
    items: list[CoordinateItem],
    context: dict[str, Any],
    path: Path,
) -> None:
    grid = context["grid_mm"]
    pallet = context["pallet"]
    rows = solution_rows(solution, items, context)
    positions = [
        ((row["pallet"] - 1) * pallet["length_mm"] + row["x_mm"], row["y_mm"], row["z_mm"])
        for row in rows
    ]
    sizes = [(row["length_mm"], row["width_mm"], row["height_mm"]) for row in rows]
    case_ids = np.array([row["sku"] for row in rows])
    figure = _plot_cuboids(
        positions,
        sizes,
        pallet["length_mm"] * solution.pallet_count,
        pallet["width_mm"],
        pallet["height_mm"],
        True,
        case_ids,
    )
    for p in range(solution.pallet_count):
        left, right = p * pallet["length_mm"], (p + 1) * pallet["length_mm"]
        figure.add_trace(
            go.Scatter3d(
                x=[left, right, right, left, left],
                y=[0, 0, pallet["width_mm"], pallet["width_mm"], 0],
                z=[0, 0, 0, 0, 0],
                mode="lines",
                name=f"Pallet {p + 1}",
                line={"color": "red", "width": 6},
            )
        )
    figure.update_layout(
        title=(
            f"Coordinate MILP: pallets={solution.pallet_count}, status={solution.status}, "
            f"gap={100 * solution.mip_gap:.2f}%"
        ),
        scene={"aspectmode": "data"},
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
    payload = {
        "instance": context.get("payload", {}).get("meta", {}).get("name", "unknown"),
        "formulation": "coordinate-based pairwise non-overlap MILP",
        "metrics": {
            "status": solution.status,
            "pallet_count": solution.pallet_count,
            "volume_utilization": total_volume / available_volume,
            "objective_bound": solution.objective_bound,
            "mip_gap": solution.mip_gap,
            "runtime_seconds": solution.runtime_seconds,
            "grid_mm": context["grid_mm"],
        },
        "placements": rows,
    }
    (output_dir / f"{stem}.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
    with (output_dir / "summary.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["status", "pallet_count", "volume_utilization", "objective_bound", "mip_gap", "runtime_seconds"])
        writer.writerow(
            [
                solution.status,
                solution.pallet_count,
                payload["metrics"]["volume_utilization"],
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
    exact = CoordinateBasedMILP(context, items, config)
    try:
        exact.dump_model(model_dump, print_model)
        stats = {
            "variables": exact.model.NumVars,
            "linear_constraints": exact.model.NumConstrs,
            "general_constraints": exact.model.NumGenConstrs,
        }
        solution = exact.solve()
        support_config = config.get("support", {"mode": "fraction", "minimum_fraction": 0.75})
        audit_solution(
            solution,
            context,
            str(support_config.get("mode", "fraction")),
            float(support_config.get("minimum_fraction", 0.75)),
        )
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
        "n_boxes",
        "max_pallets",
        "pallet_count",
        "objective_bound",
        "mip_gap",
        "runtime_seconds",
        "variables",
        "linear_constraints",
        "general_constraints",
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
                "n_boxes": len(items),
                "max_pallets": int(config["max_pallets"]),
                "pallet_count": solution.pallet_count,
                "objective_bound": solution.objective_bound,
                "mip_gap": solution.mip_gap,
                "runtime_seconds": solution.runtime_seconds,
                **stats,
                "output_directory": str(instance_output),
                "error_type": "",
                "error": "",
            }
            print(
                f"  {solution.status}: pallets={solution.pallet_count}, "
                f"gap={100 * solution.mip_gap:.3f}%, runtime={solution.runtime_seconds:.2f}s"
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
                "n_boxes": "",
                "max_pallets": int(config.get("max_pallets", 0)),
                "pallet_count": "",
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
    solution, _, _, _ = solve_instance(
        input_path,
        config,
        output_dir,
        args.write_lp,
        args.print_model,
    )
    print(
        f"Status={solution.status}; pallets={solution.pallet_count}; "
        f"gap={100 * solution.mip_gap:.3f}%; runtime={solution.runtime_seconds:.2f}s"
    )
    print(f"Wrote {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
