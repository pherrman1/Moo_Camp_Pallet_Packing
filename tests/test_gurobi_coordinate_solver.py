import unittest
from pathlib import Path

from gurobi_coordinate_solver import (
    CoordinateItem,
    CoordinatePlacement,
    CoordinateSolution,
    CoordinateBasedMILP,
    ReducedExactCoordinateMILP,
    allowed_orientations,
    audit_solution,
    footprint_overlap,
    read_mcpp_json,
    recommended_max_pallets,
    snap_up,
    support_fraction,
)


def placement(item, x, y, z, dx, dy, dz, pallet=0):
    return CoordinatePlacement(item, pallet, 0, x, y, z, dx, dy, dz)


class TestCoordinateGeometry(unittest.TestCase):
    def test_snap_up_is_conservative(self):
        self.assertEqual(snap_up(251, 50), 6)
        self.assertEqual(snap_up(250, 50), 5)

    def test_upright_item_only_yaws(self):
        item = CoordinateItem(0, 1, 1, (600, 400, 300), (12, 8, 6), 1.0, 72.0, "x", False, False, False, True, 1)
        self.assertEqual(allowed_orientations(item, "six"), [(12, 8, 6), (8, 12, 6)])

    def test_exactly_seventy_five_percent_support(self):
        lower = placement(0, 0, 0, 0, 75, 100, 10)
        upper = placement(1, 0, 0, 10, 100, 100, 10)
        self.assertEqual(footprint_overlap(lower, upper), 7500)
        self.assertAlmostEqual(support_fraction(upper, [lower, upper]), 0.75)

    def test_two_supporters_are_summed(self):
        left = placement(0, 0, 0, 0, 40, 100, 10)
        right = placement(1, 60, 0, 0, 40, 100, 10)
        upper = placement(2, 0, 0, 10, 100, 100, 10)
        self.assertAlmostEqual(support_fraction(upper, [left, right, upper]), 0.8)

    def test_audit_rejects_three_dimensional_overlap(self):
        first = placement(0, 0, 0, 0, 10, 10, 10)
        second = placement(1, 5, 5, 5, 10, 10, 10)
        solution = CoordinateSolution("TEST", 1, 1.0, 0.0, 0.0, [first, second])
        context = {"pallet": {"length": 100, "width": 100, "height": 100}}
        with self.assertRaises(RuntimeError):
            audit_solution(solution, context, "off", 0.75)

    def test_all_pl100_instances_use_the_supported_schema(self):
        instance_dir = Path(__file__).parent / "instances"
        files = sorted(instance_dir.glob("pl*.json"))
        self.assertEqual(len(files), 100)
        config = {"grid_mm": 50, "max_items": 100}
        for path in files:
            context, items = read_mcpp_json(path, config)
            self.assertEqual(len(items), context["payload"]["meta"]["n"])
            self.assertEqual(context["pallet"]["length"], 24)
            self.assertEqual(context["pallet"]["width"], 16)

    def test_benchmark_upper_bound_is_used_for_batch_capacity(self):
        payload = {
            "meta": {"bounds": {"ub_pallets_heuristic": 3}}
        }
        self.assertEqual(recommended_max_pallets(payload), 3)

    def test_both_payload_capacity_field_names_are_supported(self):
        root = Path(__file__).resolve().parents[1]
        config = {"grid_mm": "input", "max_items": 100, "stacking_mass_alpha": 1.2}
        pl_context, _ = read_mcpp_json(
            root / "tests" / "instances" / "pl001_n010_H900_B2_LB2UB2.json", config
        )
        mcpp_context, _ = read_mcpp_json(root / "input" / "gurobi_small_exact.json", config)
        self.assertEqual(pl_context["pallet"]["payload_kg"], 500.0)
        self.assertEqual(mcpp_context["pallet"]["payload_kg"], 1000.0)


class TestReducedExactCoordinateMILP(unittest.TestCase):
    @staticmethod
    def _item(index, dims, weight_kg=1.0):
        return CoordinateItem(
            index, index + 1, index + 1, tuple(value * 50 for value in dims), dims,
            weight_kg, 1.0, "test", False, False, False, True, 1,
        )

    def _solve(self, model_class, support_mode):
        context = {
            "pallet": {
                "length": 4, "width": 2, "height": 2,
                "length_mm": 200, "width_mm": 100, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [
            self._item(0, (2, 2, 1)),
            self._item(1, (2, 2, 1)),
            self._item(2, (4, 2, 1)),
        ]
        config = {
            "max_pallets": 2,
            "time_limit_seconds": 10,
            "mip_gap": 0.0,
            "log_to_console": False,
            "rotation_mode": "yaw",
            "area_auxiliary_type": "integer",
            "stacking_mass_alpha": 1.2,
            "objective_mode": "pallets_then_max_height",
            "support": {"mode": support_mode, "minimum_fraction": 0.75},
            "symmetry": {
                "fix_first_item": False,
                "order_pallet_loads": False,
                "order_identical_items": False,
            },
        }
        exact = model_class(context, items, config)
        try:
            solution = exact.solve()
            audit_solution(solution, context, support_mode, 0.75)
            return solution, exact.model.NumVars, exact.model.NumConstrs, exact.model.NumGenConstrs
        finally:
            exact.model.dispose()

    def test_reduced_matches_legacy_for_all_support_modes(self):
        for support_mode in ("off", "direct", "fraction", "full"):
            with self.subTest(support_mode=support_mode):
                legacy, *_ = self._solve(CoordinateBasedMILP, support_mode)
                reduced, *_ = self._solve(ReducedExactCoordinateMILP, support_mode)
                self.assertEqual(legacy.pallet_count, reduced.pallet_count)
                self.assertEqual(legacy.objective_bound, reduced.objective_bound)

    def test_reduced_fraction_model_is_strictly_smaller(self):
        _, legacy_vars, legacy_rows, legacy_general = self._solve(CoordinateBasedMILP, "fraction")
        _, reduced_vars, reduced_rows, reduced_general = self._solve(ReducedExactCoordinateMILP, "fraction")
        self.assertLess(reduced_vars, legacy_vars)
        self.assertLess(reduced_rows, legacy_rows)
        self.assertLess(reduced_general, legacy_general)

    def test_payload_capacity_can_force_two_pallets(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 600.0), self._item(1, (1, 1, 1), 600.0)]
        config = {
            "max_pallets": 2, "time_limit_seconds": 10, "mip_gap": 0.0,
            "log_to_console": False, "rotation_mode": "none",
            "area_auxiliary_type": "integer", "stacking_mass_alpha": 1.2,
            "support": {"mode": "off", "minimum_fraction": 0.75},
            "symmetry": {"fix_first_item": False, "order_pallet_loads": False, "order_identical_items": False},
        }
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            self.assertEqual(solution.pallet_count, 2)
            self.assertEqual(
                len([c for c in exact.model.getConstrs() if c.ConstrName.startswith("payload_capacity")]),
                2,
            )
            audit_solution(solution, context, "off", 0.75, items, 1.2)
        finally:
            exact.model.dispose()

    def test_heavier_box_is_forced_below_lighter_box(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 2,
                "length_mm": 50, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 10.0), self._item(1, (1, 1, 1), 5.0)]
        config = {
            "max_pallets": 1, "time_limit_seconds": 10, "mip_gap": 0.0,
            "log_to_console": False, "rotation_mode": "none",
            "area_auxiliary_type": "integer", "stacking_mass_alpha": 1.2,
            "support": {"mode": "full", "minimum_fraction": 0.75},
            "symmetry": {"fix_first_item": False, "order_pallet_loads": False, "order_identical_items": False},
        }
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            by_item = {placement.item: placement for placement in solution.placements}
            self.assertLess(by_item[0].z, by_item[1].z)
            self.assertEqual(
                len([c for c in exact.model.getConstrs() if c.ConstrName.startswith("mass_above_ratio")]),
                1,
            )
            audit_solution(solution, context, "full", 0.75, items, 1.2)
        finally:
            exact.model.dispose()

    def test_mass_order_rule_is_inactive_across_pallets(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 1,
                "length_mm": 50, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 10.0), self._item(1, (1, 1, 1), 5.0)]
        config = {
            "max_pallets": 2, "time_limit_seconds": 10, "mip_gap": 0.0,
            "log_to_console": False, "rotation_mode": "none",
            "area_auxiliary_type": "integer", "stacking_mass_alpha": 1.2,
            "support": {"mode": "off", "minimum_fraction": 0.75},
            "symmetry": {"fix_first_item": False, "order_pallet_loads": False, "order_identical_items": False},
        }
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            self.assertEqual(solution.pallet_count, 2)
            audit_solution(solution, context, "off", 0.75, items, 1.2)
        finally:
            exact.model.dispose()

    def test_lexicographic_height_never_uses_an_extra_pallet(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 2,
                "length_mm": 50, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 5.0), self._item(1, (1, 1, 1), 5.0)]
        config = {
            "max_pallets": 2, "time_limit_seconds": 10, "mip_gap": 0.0,
            "log_to_console": False, "rotation_mode": "none",
            "area_auxiliary_type": "integer", "stacking_mass_alpha": 1.2,
            "objective_mode": "pallets_then_max_height",
            "support": {"mode": "full", "minimum_fraction": 0.75},
            "symmetry": {"fix_first_item": False, "order_pallet_loads": False, "order_identical_items": False},
        }
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            self.assertTrue(exact.secondary_optimized)
            self.assertEqual(solution.pallet_count, 1)
            self.assertEqual(solution.max_height_grid, 2)
            self.assertTrue(solution.height_stage_attempted)
            self.assertEqual(solution.height_objective_bound_grid, 2.0)
            self.assertEqual(solution.height_mip_gap, 0.0)
            self.assertIsNotNone(exact.model.getConstrByName("fix_lexicographic_pallet_count"))
        finally:
            exact.model.dispose()

    def test_secondary_objective_minimizes_height_at_fixed_pallet_count(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 3,
                "length_mm": 100, "width_mm": 50, "height_mm": 150,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 5.0), self._item(1, (1, 1, 1), 5.0)]
        config = {
            "max_pallets": 1, "time_limit_seconds": 10, "mip_gap": 0.0,
            "log_to_console": False, "rotation_mode": "none",
            "area_auxiliary_type": "integer", "stacking_mass_alpha": 1.2,
            "objective_mode": "pallets_then_max_height",
            "support": {"mode": "off", "minimum_fraction": 0.75},
            "symmetry": {"fix_first_item": False, "order_pallet_loads": False, "order_identical_items": False},
        }
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            self.assertEqual(solution.pallet_count, 1)
            self.assertEqual(solution.max_height_grid, 1)
            self.assertTrue(solution.height_stage_attempted)
            self.assertEqual(solution.height_objective_bound_grid, 1.0)
            self.assertEqual(solution.height_mip_gap, 0.0)
        finally:
            exact.model.dispose()


if __name__ == "__main__":
    unittest.main()
