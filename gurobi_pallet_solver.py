"""Exact grid-based position-indexed MILP for small mixed-case pallet instances."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from collections import defaultdict
from dataclasses import asdict, dataclass
from itertools import combinations
from pathlib import Path
from typing import Any, Iterable

import gurobipy as gp
import numpy as np
import plotly.graph_objects as go
from gurobipy import GRB

from utils import _plot_cuboids


@dataclass(frozen=True)
class ExactItem:
    index: int
    id: int
    sku: int
    original_mm: tuple[int, int, int]
    dims: tuple[int, int, int]
    weight_kg: float
    volume_dm3: float
    density_kg_m3: float
    family: str
    is_food: bool
    is_chemical: bool
    fragile: bool
    upright_only: bool
    retrieval_priority: int


@dataclass(frozen=True)
class Placement:
    id: int
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
class ExactSolution:
    pallet_count: int
    height_spread: int
    accessibility: int
    density_moment: float
    mass_moment: float
    vertical_moment: float
    vertical_moment_mode: str
    objective_bound: float
    mip_gap: float
    runtime_seconds: float
    placements: list[Placement]


DEFAULT_CONFIG = Path(__file__).parent / "configs" / "gurobi_exact_default.json"


def load_config(path: str | Path | None) -> dict[str, Any]:
    config_path = Path(path) if path else DEFAULT_CONFIG
    return json.loads(config_path.read_text(encoding="utf-8"))


def snap_up(value_mm: float, grid_mm: int) -> int:
    return max(1, math.ceil(value_mm / grid_mm - 1e-12))


def read_mcpp_json(path: str | Path, config: dict[str, Any]) -> tuple[dict, list[ExactItem]]:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    grid = int(config["grid_mm"])
    pallet_raw = payload["pallet"]
    pallet = {
        "length": int(round(float(pallet_raw["width"]) / grid)),
        "width": int(round(float(pallet_raw["depth"]) / grid)),
        "height": int(round(float(pallet_raw["height"]) / grid)),
        "length_mm": int(round(float(pallet_raw["width"]))),
        "width_mm": int(round(float(pallet_raw["depth"]))),
        "height_mm": int(round(float(pallet_raw["height"]))),
        "name": pallet_raw.get("name", "pallet"),
    }
    items: list[ExactItem] = []
    for index, raw in enumerate(payload["items"]):
        original = (
            int(round(float(raw["width"]))),
            int(round(float(raw["depth"]))),
            int(round(float(raw["height"]))),
        )
        dims = tuple(snap_up(value, grid) for value in original)
        volume_dm3 = float(raw.get("volume_dm3", np.prod(original) / 1e6))
        weight = float(raw.get("weight_kg", 1.0))
        density = 1000.0 * weight / volume_dm3 if volume_dm3 > 0 else 0.0
        items.append(
            ExactItem(
                index=index,
                id=int(raw.get("id", index + 1)),
                sku=int(raw.get("sku", index + 1)),
                original_mm=original,
                dims=dims,
                weight_kg=weight,
                volume_dm3=volume_dm3,
                density_kg_m3=density,
                family=str(raw.get("family", "unknown")),
                is_food=bool(raw.get("is_food", False)),
                is_chemical=bool(raw.get("is_chemical", False)),
                fragile=bool(raw.get("fragile", False)),
                upright_only=bool(raw.get("upright_only", False)),
                retrieval_priority=int(raw.get("retrieval_priority", 1)),
            )
        )
    if len(items) > int(config["max_items"]):
        raise ValueError(
            f"exact model is configured for at most {config['max_items']} items; got {len(items)}"
        )
    return {"payload": payload, "pallet": pallet, "grid_mm": grid}, items


def allowed_orientations(item: ExactItem, mode: str) -> list[tuple[int, int, int]]:
    length, width, height = item.dims
    if mode == "none":
        candidates = [(length, width, height)]
    elif mode in {"yaw", "metadata"} or item.upright_only:
        candidates = [(length, width, height), (width, length, height)]
    elif mode == "six":
        candidates = [
            (length, width, height), (width, length, height),
            (length, height, width), (height, length, width),
            (width, height, length), (height, width, length),
        ]
    else:
        raise ValueError(f"unsupported rotation mode: {mode}")
    return list(dict.fromkeys(candidates))


def subset_sum_coordinates(limit: int, dimensions: Iterable[int]) -> list[int]:
    reachable = {0}
    for dimension in dimensions:
        reachable |= {value + dimension for value in tuple(reachable) if value + dimension <= limit}
    return sorted(reachable)


def generate_placements(
    items: list[ExactItem], pallet: dict[str, int], max_pallets: int, rotation_mode: str
) -> tuple[list[Placement], dict[int, list[int]]]:
    orientations = {item.index: allowed_orientations(item, rotation_mode) for item in items}
    x_dims = [dims[0] for item in items for dims in orientations[item.index]]
    y_dims = [dims[1] for item in items for dims in orientations[item.index]]
    z_dims = [dims[2] for item in items for dims in orientations[item.index]]
    xs = subset_sum_coordinates(pallet["length"], x_dims)
    ys = subset_sum_coordinates(pallet["width"], y_dims)
    zs = subset_sum_coordinates(pallet["height"], z_dims)

    placements: list[Placement] = []
    by_item: dict[int, list[int]] = defaultdict(list)
    for item in items:
        for pallet_index in range(max_pallets):
            for orientation, (dx, dy, dz) in enumerate(orientations[item.index]):
                x_values = sorted(set([x for x in xs if x + dx <= pallet["length"]] + [pallet["length"] - dx]))
                y_values = sorted(set([y for y in ys if y + dy <= pallet["width"]] + [pallet["width"] - dy]))
                z_values = [z for z in zs if z + dz <= pallet["height"]]
                for x in x_values:
                    for y in y_values:
                        for z in z_values:
                            pid = len(placements)
                            placements.append(
                                Placement(pid, item.index, pallet_index, orientation, x, y, z, dx, dy, dz)
                            )
                            by_item[item.index].append(pid)
    return placements, by_item


def overlap_1d(a0: int, a1: int, b0: int, b1: int) -> int:
    return max(0, min(a1, b1) - max(a0, b0))


def footprint_overlap(a: Placement, b: Placement) -> int:
    return overlap_1d(a.x, a.x + a.dx, b.x, b.x + b.dx) * overlap_1d(
        a.y, a.y + a.dy, b.y, b.y + b.dy
    )


def support_fraction(upper: Placement, selected: Iterable[Placement]) -> float:
    """Calculate direct geometric support; selected lower boxes must not overlap."""
    if upper.z == 0:
        return 1.0
    supported_area = sum(
        footprint_overlap(upper, lower)
        for lower in selected
        if lower.pallet == upper.pallet
        and lower.item != upper.item
        and lower.top == upper.z
    )
    return min(1.0, supported_area / upper.base_area)


def blocks(lower: Placement, upper: Placement) -> bool:
    return lower.pallet == upper.pallet and upper.z >= lower.top and footprint_overlap(lower, upper) > 0


class PositionIndexedMILP:
    def __init__(
        self,
        context: dict,
        items: list[ExactItem],
        config: dict[str, Any],
        exact_pallet_count: int,
    ) -> None:
        self.context = context
        self.items = items
        self.config = config
        self.pallet = context["pallet"]
        self.max_pallets = int(config["max_pallets"])
        self.placements, self.by_item = generate_placements(
            items, self.pallet, self.max_pallets, config["rotation_mode"]
        )
        self.by_pallet: dict[int, list[int]] = defaultdict(list)
        self.by_pallet_top: dict[tuple[int, int], list[int]] = defaultdict(list)
        self.cover_by_item_cell: dict[tuple[int, int, int, int], list[int]] = defaultdict(list)
        self.thresholds_by_pallet: dict[int, list[int]] = defaultdict(list)
        for q in self.placements:
            self.by_pallet[q.pallet].append(q.id)
            self.by_pallet_top[q.pallet, q.top].append(q.id)
            for cx in range(q.x, q.x + q.dx):
                for cy in range(q.y, q.y + q.dy):
                    self.cover_by_item_cell[q.item, q.pallet, cx, cy].append(q.id)
        for pallet_index in range(self.max_pallets):
            self.thresholds_by_pallet[pallet_index] = sorted(
                {
                    level
                    for q in self.placements
                    if q.pallet == pallet_index
                    for level in (q.z, q.top)
                    if 0 < level < self.pallet["height"]
                }
            )

        self.model = gp.Model(f"position_indexed_{exact_pallet_count}_pallets")
        self.model.Params.TimeLimit = float(config["time_limit_seconds"])
        self.model.Params.MIPGap = float(config["mip_gap"])
        self.model.Params.OutputFlag = int(bool(config.get("log_to_console", False)))
        self.x = self.model.addVars(len(self.placements), vtype=GRB.BINARY, name="place")
        self.y = self.model.addVars(self.max_pallets, vtype=GRB.BINARY, name="used")
        self.height = self.model.addVars(
            self.max_pallets, lb=0, ub=self.pallet["height"], vtype=GRB.CONTINUOUS, name="height"
        )
        self.max_height = self.model.addVar(lb=0, ub=self.pallet["height"], name="max_height")
        self.min_height = self.model.addVar(lb=0, ub=self.pallet["height"], name="min_height")
        self.spread = self.model.addVar(lb=0, ub=self.pallet["height"], name="height_spread")
        self.access_vars: dict[tuple[int, int], gp.Var] = {}
        self._build_geometry(exact_pallet_count)
        self._build_optional_constraints()
        self._build_objectives()
        self.model.update()

    def dump_model(self, lp_path: str | Path | None = None, print_stats: bool = False) -> None:
        """Export the instantiated MILP and optionally print Gurobi statistics.

        The LP file contains the placement, pallet-use, height, spread, and
        accessibility variables together with every generated constraint for
        this concrete input instance. It is written after all model-building
        routines have run, but before later Pareto-stage constraints are added.
        """
        self.model.update()
        if print_stats:
            self.model.printStats()
            print(f"Variables: {self.model.NumVars}")
            print(f"Constraints: {self.model.NumConstrs}")
            print(f"Placements: {len(self.placements)}")
        if lp_path is not None:
            path = Path(lp_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.model.write(str(path))
            print(f"Wrote MILP: {path}")

    def _build_geometry(self, exact_pallet_count: int) -> None:
        for item in self.items:
            self.model.addConstr(
                gp.quicksum(self.x[qid] for qid in self.by_item[item.index]) == 1,
                name=f"place_once[{item.index}]",
            )

        for pallet in range(self.max_pallets):
            ids = self.by_pallet[pallet]
            self.model.addConstr(gp.quicksum(self.x[qid] for qid in ids) >= self.y[pallet])
            for item in self.items:
                item_ids = [qid for qid in self.by_item[item.index] if self.placements[qid].pallet == pallet]
                self.model.addConstr(gp.quicksum(self.x[qid] for qid in item_ids) <= self.y[pallet])
            self.model.addConstr(self.height[pallet] <= self.pallet["height"] * self.y[pallet])
            self.model.addConstr(self.max_height >= self.height[pallet])
            self.model.addConstr(
                self.min_height <= self.height[pallet] + self.pallet["height"] * (1 - self.y[pallet])
            )
            for qid in ids:
                q = self.placements[qid]
                self.model.addConstr(self.height[pallet] >= q.top * self.x[qid])

        for pallet in range(self.max_pallets - 1):
            self.model.addConstr(self.y[pallet] >= self.y[pallet + 1], name=f"pallet_order[{pallet}]")
        self.model.addConstr(gp.quicksum(self.y[p] for p in range(self.max_pallets)) == exact_pallet_count)
        self.model.addConstr(self.spread == self.max_height - self.min_height)

        # Spatial-cell clique constraints provide non-overlap without pairwise placement conflicts.
        cell_occupants: dict[tuple[int, int, int, int], list[int]] = defaultdict(list)
        for q in self.placements:
            for cx in range(q.x, q.x + q.dx):
                for cy in range(q.y, q.y + q.dy):
                    for cz in range(q.z, q.z + q.dz):
                        cell_occupants[q.pallet, cx, cy, cz].append(q.id)
        for cell, ids in cell_occupants.items():
            if len(ids) > 1:
                self.model.addConstr(gp.quicksum(self.x[qid] for qid in ids) <= 1)

        support_mode = self.config["support"]["mode"]
        if support_mode != "off":
            fraction = {
                "direct": 1e-9,
                "fraction": float(self.config["support"]["minimum_fraction"]),
                "full": 1.0,
            }[support_mode]
            for q in self.placements:
                if q.z == 0:
                    continue
                supporters = []
                for rid in self.by_pallet_top.get((q.pallet, q.z), []):
                    r = self.placements[rid]
                    if r.item == q.item:
                        continue
                    area = footprint_overlap(q, r)
                    if area > 0:
                        supporters.append((rid, area))
                self.model.addConstr(
                    gp.quicksum(area * self.x[rid] for rid, area in supporters)
                    >= fraction * q.base_area * self.x[q.id],
                    name=f"support[{q.id}]",
                )

        # Identical physical copies are ordered by their selected placement rank.
        groups: dict[tuple, list[int]] = defaultdict(list)
        for item in self.items:
            key = (
                item.sku, item.dims, item.weight_kg, item.family,
                item.fragile, item.upright_only, item.retrieval_priority,
            )
            groups[key].append(item.index)
        for indices in groups.values():
            for first, second in zip(indices, indices[1:]):
                first_ids = self.by_item[first]
                second_ids = self.by_item[second]
                self.model.addConstr(
                    gp.quicksum(rank * self.x[qid] for rank, qid in enumerate(first_ids))
                    <= gp.quicksum(rank * self.x[qid] for rank, qid in enumerate(second_ids))
                )

    def _build_optional_constraints(self) -> None:
        def column_ids(
            item: int,
            pallet: int,
            cx: int,
            cy: int,
            threshold: int,
            relation: str,
        ) -> list[int]:
            ids = self.cover_by_item_cell.get((item, pallet, cx, cy), [])
            if relation == "below":
                return [qid for qid in ids if self.placements[qid].top <= threshold]
            if relation == "above":
                return [qid for qid in ids if self.placements[qid].z >= threshold]
            if relation == "top_at":
                return [qid for qid in ids if self.placements[qid].top == threshold]
            if relation == "base_at":
                return [qid for qid in ids if self.placements[qid].z == threshold]
            raise ValueError(relation)

        def add_column_relation(
            lower_item: int,
            upper_item: int,
            violation: gp.Var | None,
            direct_only: bool,
            prefix: str,
        ) -> None:
            seen: set[tuple[tuple[int, ...], tuple[int, ...]]] = set()
            for pallet in range(self.max_pallets):
                for cx in range(self.pallet["length"]):
                    for cy in range(self.pallet["width"]):
                        for threshold in self.thresholds_by_pallet[pallet]:
                            lower_relation = "top_at" if direct_only else "below"
                            upper_relation = "base_at" if direct_only else "above"
                            lower_ids = column_ids(
                                lower_item, pallet, cx, cy, threshold, lower_relation
                            )
                            if not lower_ids:
                                continue
                            upper_ids = column_ids(
                                upper_item, pallet, cx, cy, threshold, upper_relation
                            )
                            if not upper_ids:
                                continue
                            signature = (tuple(lower_ids), tuple(upper_ids))
                            if signature in seen:
                                continue
                            seen.add(signature)
                            selected_pair = (
                                gp.quicksum(self.x[qid] for qid in lower_ids)
                                + gp.quicksum(self.x[qid] for qid in upper_ids)
                            )
                            if violation is None:
                                self.model.addConstr(selected_pair <= 1)
                            else:
                                self.model.addConstr(violation >= selected_pair - 1)

        if self.config["food_chemical"]["mode"] == "hard_overlap":
            foods = [item for item in self.items if item.is_food]
            chemicals = [item for item in self.items if item.is_chemical]
            for food in foods:
                for chemical in chemicals:
                    add_column_relation(
                        food.index, chemical.index, None, False, "food_chemical"
                    )
                    add_column_relation(
                        chemical.index, food.index, None, False, "chemical_food"
                    )

        if self.config["fragile"]["cannot_support"]:
            for lower_item in [item for item in self.items if item.fragile]:
                for upper_item in self.items:
                    if lower_item.index == upper_item.index:
                        continue
                    add_column_relation(
                        lower_item.index, upper_item.index, None, True, "fragile"
                    )

        if self.config["accessibility"]["mode"] != "off":
            for earlier, later in combinations(self.items, 2):
                if earlier.retrieval_priority == later.retrieval_priority:
                    continue
                if earlier.retrieval_priority > later.retrieval_priority:
                    earlier, later = later, earlier
                violation = self.model.addVar(
                    vtype=GRB.BINARY, name=f"access[{earlier.index},{later.index}]"
                )
                self.access_vars[earlier.index, later.index] = violation
                add_column_relation(
                    earlier.index, later.index, violation, False, "access"
                )

    def _build_objectives(self) -> None:
        grid_mm = self.context["grid_mm"]
        self.access_expr = gp.quicksum(self.access_vars.values())
        self.density_expr = gp.quicksum(
            self.items[q.item].density_kg_m3
            * ((q.z + q.dz / 2.0) * grid_mm / 1000.0)
            * self.x[q.id]
            for q in self.placements
        )
        self.mass_expr = gp.quicksum(
            self.items[q.item].weight_kg
            * ((q.z + q.dz / 2.0) * grid_mm / 1000.0)
            * self.x[q.id]
            for q in self.placements
        )
        moment_mode = self.config["vertical_moment"]["mode"]
        if moment_mode not in {"density", "mass"}:
            raise ValueError("vertical_moment.mode must be 'density' or 'mass'")
        self.vertical_expr = self.density_expr if moment_mode == "density" else self.mass_expr

    def optimize_expression(self, expression, name: str) -> None:
        self.model.setObjective(expression, GRB.MINIMIZE)
        self.model.ModelName = name
        self.model.optimize()
        if self.model.SolCount == 0:
            raise RuntimeError(f"no feasible solution for {name}; status={self.model.Status}")

    def value(self, expression) -> float:
        return float(expression.getValue()) if hasattr(expression, "getValue") else float(expression.X)

    def capture_start(self) -> dict[str, float]:
        return {variable.VarName: variable.X for variable in self.model.getVars()}

    def apply_start(self, values: dict[str, float]) -> None:
        for variable in self.model.getVars():
            if variable.VarName in values:
                variable.Start = values[variable.VarName]

    def extract(self, started: float) -> ExactSolution:
        selected = [q for q in self.placements if self.x[q.id].X > 0.5]
        return ExactSolution(
            pallet_count=int(round(sum(self.y[p].X for p in range(self.max_pallets)))),
            height_spread=int(round(self.spread.X)),
            accessibility=int(round(self.value(self.access_expr))),
            density_moment=self.value(self.density_expr),
            mass_moment=self.value(self.mass_expr),
            vertical_moment=self.value(self.vertical_expr),
            vertical_moment_mode=self.config["vertical_moment"]["mode"],
            objective_bound=float(self.model.ObjBound),
            mip_gap=float(self.model.MIPGap),
            runtime_seconds=time.perf_counter() - started,
            placements=selected,
        )


def nondominated(solutions: list[ExactSolution]) -> list[ExactSolution]:
    result = []
    for candidate in solutions:
        dominated = any(
            other is not candidate
            and other.pallet_count <= candidate.pallet_count
            and other.height_spread <= candidate.height_spread
            and other.accessibility <= candidate.accessibility
            and other.vertical_moment <= candidate.vertical_moment + 1e-8
            and (
                other.pallet_count < candidate.pallet_count
                or other.height_spread < candidate.height_spread
                or other.accessibility < candidate.accessibility
                or other.vertical_moment < candidate.vertical_moment - 1e-8
            )
            for other in solutions
        )
        if not dominated:
            result.append(candidate)
    unique: dict[tuple, ExactSolution] = {}
    for solution in result:
        key = (
            solution.pallet_count,
            solution.height_spread,
            solution.accessibility,
            round(solution.vertical_moment, 6),
        )
        unique[key] = solution
    return sorted(unique.values(), key=lambda s: (s.pallet_count, s.height_spread, s.accessibility, s.vertical_moment))


def solve_pareto(
    context: dict,
    items: list[ExactItem],
    config: dict[str, Any],
    model_dump: str | Path | None = None,
    print_model: bool = False,
) -> list[ExactSolution]:
    total_volume = sum(np.prod(item.dims) for item in items)
    pallet_volume = context["pallet"]["length"] * context["pallet"]["width"] * context["pallet"]["height"]
    lower_bound = max(1, math.ceil(total_volume / pallet_volume))
    all_solutions: list[ExactSolution] = []

    for pallet_count in range(lower_bound, int(config["max_pallets"]) + 1):
        started = time.perf_counter()
        exact = PositionIndexedMILP(context, items, config, pallet_count)
        if model_dump is not None or print_model:
            dump_path = None
            if model_dump is not None:
                requested = Path(model_dump)
                if pallet_count == lower_bound:
                    dump_path = requested if requested.suffix else requested.with_suffix(".lp")
                else:
                    suffix = requested.suffix or ".lp"
                    dump_path = requested.with_name(f"{requested.stem}_p{pallet_count}{suffix}")
            # Export the first objective that Gurobi actually optimizes. Later
            # Pareto stages replace this objective and add their own bounds.
            exact.model.setObjective(exact.spread, GRB.MINIMIZE)
            exact.dump_model(dump_path, print_stats=print_model)
        exact.optimize_expression(exact.spread, f"spread_{pallet_count}_pallets")
        optimal_spread = int(math.ceil(exact.spread.X - 1e-8))
        spread_constraint = exact.model.addConstr(exact.spread <= optimal_spread, name="fix_optimal_spread")

        exact.optimize_expression(exact.access_expr, f"min_access_{pallet_count}_pallets")
        min_access = int(round(exact.value(exact.access_expr)))
        all_solutions.append(exact.extract(started))
        access_start = exact.capture_start()

        moment_mode = config["vertical_moment"]["mode"]
        exact.optimize_expression(exact.vertical_expr, f"min_{moment_mode}_{pallet_count}_pallets")
        moment_endpoint = exact.extract(started)
        all_solutions.append(moment_endpoint)
        max_epsilon = moment_endpoint.accessibility
        configured_max = config["pareto"].get("max_accessibility_epsilon")
        if configured_max is not None:
            max_epsilon = min(max_epsilon, int(configured_max))

        for epsilon in range(min_access, max_epsilon + 1):
            epsilon_constraint = exact.model.addConstr(exact.access_expr <= epsilon, name=f"access_epsilon_{epsilon}")
            exact.apply_start(access_start)
            exact.optimize_expression(exact.vertical_expr, f"pareto_p{pallet_count}_a{epsilon}")
            all_solutions.append(exact.extract(started))
            access_start = exact.capture_start()
            exact.model.remove(epsilon_constraint)
            exact.model.update()

        exact.model.remove(spread_constraint)
        exact.model.dispose()

    return nondominated(all_solutions)


def solution_rows(
    solution: ExactSolution, items: list[ExactItem], grid_mm: int
) -> list[dict[str, Any]]:
    rows = []
    by_item = {q.item: q for q in solution.placements}
    for item in items:
        q = by_item[item.index]
        rows.append(
            {
                "box_index": item.index,
                "box_id": item.id,
                "sku": item.sku,
                "pallet": q.pallet + 1,
                "orientation": q.orientation,
                "x_mm": q.x * grid_mm,
                "y_mm": q.y * grid_mm,
                "z_mm": q.z * grid_mm,
                "length_mm": q.dx * grid_mm,
                "width_mm": q.dy * grid_mm,
                "height_mm": q.dz * grid_mm,
                "weight_kg": item.weight_kg,
                "density_kg_m3": item.density_kg_m3,
                "family": item.family,
                "is_food": item.is_food,
                "is_chemical": item.is_chemical,
                "fragile": item.fragile,
                "retrieval_priority": item.retrieval_priority,
                "support_fraction": support_fraction(q, solution.placements),
            }
        )
    return rows


def render_solution(
    solution: ExactSolution,
    items: list[ExactItem],
    context: dict,
    path: Path,
) -> None:
    grid = context["grid_mm"]
    pallet = context["pallet"]
    rows = solution_rows(solution, items, grid)
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
        left = p * pallet["length_mm"]
        right = (p + 1) * pallet["length_mm"]
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
            f"Exact Pareto solution: pallets={solution.pallet_count}, "
            f"spread={solution.height_spread * grid} mm, "
            f"access={solution.accessibility}, {solution.vertical_moment_mode} moment="
            f"{solution.vertical_moment:.1f}"
        ),
        scene={"aspectmode": "data"},
    )
    figure.write_html(path)


def write_outputs(
    solutions: list[ExactSolution],
    items: list[ExactItem],
    context: dict,
    output_dir: Path,
    render_all: bool,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / "pareto_front.csv"
    with summary_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "solution", "pallet_count", "height_spread_mm", "accessibility_inversions",
                "vertical_moment_mode", "vertical_moment", "density_vertical_moment",
                "mass_vertical_moment", "mip_gap", "runtime_seconds",
            ]
        )
        for index, solution in enumerate(solutions):
            writer.writerow(
                [
                    index, solution.pallet_count, solution.height_spread * context["grid_mm"],
                    solution.accessibility, solution.vertical_moment_mode, solution.vertical_moment,
                    solution.density_moment, solution.mass_moment,
                    solution.mip_gap, solution.runtime_seconds,
                ]
            )

    pareto_figure = go.Figure()
    for pallet_count in sorted({solution.pallet_count for solution in solutions}):
        group = [solution for solution in solutions if solution.pallet_count == pallet_count]
        pareto_figure.add_trace(
            go.Scatter(
                x=[solution.accessibility for solution in group],
                y=[solution.vertical_moment for solution in group],
                mode="lines+markers",
                name=f"{pallet_count} pallet(s)",
                customdata=[solution.height_spread * context["grid_mm"] for solution in group],
                hovertemplate=(
                    "accessibility inversions=%{x}<br>"
                    f"{group[0].vertical_moment_mode} moment=%{{y:.3f}}<br>"
                    "height spread=%{customdata} mm<extra></extra>"
                ),
            )
        )
    pareto_figure.update_layout(
        title="Exact discrete Pareto front",
        xaxis_title="Accessibility inversions (lower is better)",
        yaxis_title=f"{solutions[0].vertical_moment_mode.title()}-weighted vertical moment (lower is better)",
    )
    pareto_figure.write_html(output_dir / "pareto_front.html")

    for index, solution in enumerate(solutions):
        stem = f"solution_{index:03d}_p{solution.pallet_count}_a{solution.accessibility}"
        rows = solution_rows(solution, items, context["grid_mm"])
        with (output_dir / f"{stem}.csv").open("w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        payload = {
            "metrics": {
                "pallet_count": solution.pallet_count,
                "height_spread_mm": solution.height_spread * context["grid_mm"],
                "accessibility_inversions": solution.accessibility,
                "density_vertical_moment": solution.density_moment,
                "mass_vertical_moment": solution.mass_moment,
                "vertical_moment_mode": solution.vertical_moment_mode,
                "vertical_moment": solution.vertical_moment,
                "mip_gap": solution.mip_gap,
                "runtime_seconds": solution.runtime_seconds,
            },
            "placements": rows,
        }
        (output_dir / f"{stem}.json").write_text(json.dumps(payload, indent=2), encoding="utf-8")
        if render_all or index == 0:
            render_solution(solution, items, context, output_dir / f"{stem}.html")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="MCPP JSON instance")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="JSON model configuration")
    parser.add_argument("--output-dir", default="output/gurobi_exact", help="result directory")
    parser.add_argument("--time-limit", type=float, default=None)
    parser.add_argument("--max-pallets", type=int, default=None)
    parser.add_argument(
        "--write-lp",
        default=None,
        metavar="PATH",
        help="write the instantiated Gurobi MILP to an LP file before optimization",
    )
    parser.add_argument(
        "--print-model",
        action="store_true",
        help="print Gurobi model statistics before optimization",
    )
    args = parser.parse_args()

    config = load_config(args.config)
    if args.time_limit is not None:
        config["time_limit_seconds"] = args.time_limit
    if args.max_pallets is not None:
        config["max_pallets"] = args.max_pallets
    context, items = read_mcpp_json(args.input, config)
    solutions = solve_pareto(
        context,
        items,
        config,
        model_dump=args.write_lp,
        print_model=args.print_model,
    )
    write_outputs(
        solutions,
        items,
        context,
        Path(args.output_dir),
        bool(config["pareto"]["render_all_solutions"]),
    )
    print(f"Pareto solutions: {len(solutions)}")
    for index, solution in enumerate(solutions):
        print(
            f"  {index}: pallets={solution.pallet_count}, "
            f"spread={solution.height_spread * context['grid_mm']} mm, "
            f"access={solution.accessibility}, {solution.vertical_moment_mode}="
            f"{solution.vertical_moment:.3f}"
        )
    print(f"Wrote {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
