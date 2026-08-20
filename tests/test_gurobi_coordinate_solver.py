import unittest
from pathlib import Path

from gurobi_coordinate_solver import (
    CoordinateItem,
    CoordinatePlacement,
    CoordinateSolution,
    CoordinateBasedMILP,
    ReducedExactCoordinateMILP,
    allowed_orientations,
    audit_coordinate_warm_start,
    audit_solution,
    category_type_colors,
    footprint_height_lower_bound,
    footprint_overlap,
    greedy_coordinate_warm_start,
    prepare_unlimited_coordinate_warm_start,
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

    def test_footprint_height_bound_adds_the_lowest_required_heights(self):
        orientations = {
            index: [(1, 1, height)]
            for index, height in enumerate((1, 2, 3, 4, 5))
        }
        self.assertEqual(footprint_height_lower_bound(orientations, 2, 1, 1), (3, 6))
        self.assertEqual(footprint_height_lower_bound(orientations, 2, 1, 2), (2, 3))

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

    def test_visualization_uses_category_hues_and_type_shades(self):
        rows = [
            {"retrieval_priority": 2, "sku": 10},
            {"retrieval_priority": 2, "sku": 10},
            {"retrieval_priority": 2, "sku": 11},
            {"retrieval_priority": 3, "sku": 10},
        ]
        colors = category_type_colors(rows)
        self.assertEqual(colors[0], colors[1])
        self.assertNotEqual(colors[0], colors[2])
        self.assertNotEqual(colors[0], colors[3])

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

    def test_food_and_chemical_flags_are_read_from_benchmark_groups(self):
        root = Path(__file__).resolve().parents[1]
        benchmark = (
            root / "tests" / "few box types test instances" / "bt100_typeset"
            / "bt100_typeset" / "instances" / "bt009_n014_T7_c4_s1.json"
        )
        _, items = read_mcpp_json(
            benchmark, {"grid_mm": "input", "max_items": 100}
        )
        self.assertEqual(sum(item.is_food for item in items), 10)
        self.assertEqual(sum(item.is_chemical for item in items), 4)
        self.assertFalse(any(item.is_food and item.is_chemical for item in items))


class TestReducedExactCoordinateMILP(unittest.TestCase):
    @staticmethod
    def _item(
        index, dims, weight_kg=1.0, is_food=False, is_chemical=False, priority=1
    ):
        return CoordinateItem(
            index, index + 1, index + 1, tuple(value * 50 for value in dims), dims,
            weight_kg, 1.0, "test", is_food, is_chemical, False, True, priority,
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

    @staticmethod
    def _config(max_pallets=2, support_mode="fraction", warm_start=True):
        return {
            "max_pallets": max_pallets,
            "time_limit_seconds": 10,
            "mip_gap": 0.0,
            "log_to_console": False,
            "rotation_mode": "yaw",
            "area_auxiliary_type": "integer",
            "stacking_mass_alpha": 1.2,
            "objective_mode": "pallet_count_only",
            "warm_start": {"greedy": warm_start},
            "support": {"mode": support_mode, "minimum_fraction": 0.75},
            "symmetry": {
                "fix_first_item": False,
                "order_pallet_loads": False,
                "order_identical_items": False,
            },
        }

    def test_greedy_warm_start_is_deterministic_and_audited_for_all_support_modes(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 2,
                "length_mm": 100, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(i, (1, 1, 1), 5.0) for i in range(3)]
        orientations = {i: [(1, 1, 1)] for i in range(3)}
        for support_mode in ("off", "direct", "fraction", "full"):
            with self.subTest(support_mode=support_mode):
                config = self._config(support_mode=support_mode)
                first = greedy_coordinate_warm_start(context, items, orientations, config, 2)
                second = greedy_coordinate_warm_start(context, items, orientations, config, 2)
                self.assertIsNotNone(first)
                self.assertEqual(first, second)
                solution = CoordinateSolution("TEST", 1, 0.0, 0.0, 0.0, first)
                audit_solution(solution, context, support_mode, 0.75, items, 1.2)

    def test_greedy_warm_start_honors_payload_and_insufficient_capacity(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 600.0), self._item(1, (1, 1, 1), 600.0)]
        orientations = {0: [(1, 1, 1)], 1: [(1, 1, 1)]}
        config = self._config(max_pallets=2, support_mode="off")
        placements = greedy_coordinate_warm_start(context, items, orientations, config, 2)
        self.assertIsNotNone(placements)
        self.assertEqual({q.pallet for q in placements}, {0, 1})
        self.assertIsNone(greedy_coordinate_warm_start(context, items, orientations, config, 1))
        unlimited = greedy_coordinate_warm_start(
            context, items, orientations, config, max_pallets=None
        )
        self.assertIsNotNone(unlimited)
        self.assertEqual({q.pallet for q in unlimited}, {0, 1})
        audit_coordinate_warm_start(unlimited, context, items, config)

    def test_prepared_warm_start_ignores_configured_max_pallets(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 1,
                "length_mm": 50, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(i, (1, 1, 1), 5.0) for i in range(4)]
        config = self._config(max_pallets=1, support_mode="full")
        placements = prepare_unlimited_coordinate_warm_start(context, items, config)
        self.assertIsNotNone(placements)
        self.assertEqual(len({placement.pallet for placement in placements}), 4)
        audit_coordinate_warm_start(placements, context, items, config)

    def test_greedy_warm_start_reorders_heavy_box_below_light_box(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 2,
                "length_mm": 50, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        orientations = {0: [(1, 1, 1)], 1: [(1, 1, 1)]}
        config = self._config(max_pallets=1, support_mode="full")
        light_then_heavy = [
            self._item(0, (1, 1, 1), 5.0), self._item(1, (1, 1, 1), 10.0),
        ]
        placements = greedy_coordinate_warm_start(
            context, light_then_heavy, orientations, config, max_pallets=1
        )
        self.assertIsNotNone(placements)
        by_item = {placement.item: placement for placement in placements}
        self.assertEqual(by_item[1].z, 0)
        self.assertEqual(by_item[0].z, 1)

        heavy_then_light = [
            self._item(0, (1, 1, 1), 10.0), self._item(1, (1, 1, 1), 5.0),
        ]
        placements = greedy_coordinate_warm_start(
            context, heavy_then_light, orientations, config, max_pallets=1
        )
        self.assertIsNotNone(placements)
        self.assertEqual(placements[0].z, 0)
        self.assertEqual(placements[1].z, 1)

    def test_greedy_warm_start_places_chemicals_then_food_by_descending_weight(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 5,
                "length_mm": 50, "width_mm": 50, "height_mm": 250,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [
            self._item(0, (1, 1, 1), 8.0, is_food=True),
            self._item(1, (1, 1, 1), 15.0, is_chemical=True),
            self._item(2, (1, 1, 1), 20.0, is_chemical=True),
            self._item(3, (1, 1, 1), 5.0, is_food=True),
            self._item(4, (1, 1, 1), 2.0),
        ]
        orientations = {item.index: [(1, 1, 1)] for item in items}
        config = self._config(max_pallets=1, support_mode="full")
        config["food_chemical"] = {"mode": "chemical_below_food"}
        placements = greedy_coordinate_warm_start(
            context, items, orientations, config, max_pallets=1
        )
        self.assertIsNotNone(placements)
        by_z = sorted(placements, key=lambda placement: placement.z)
        self.assertEqual([q.item for q in by_z], [2, 1, 0, 3, 4])
        self.assertEqual(
            [items[q.item].weight_kg for q in by_z],
            sorted((item.weight_kg for item in items), reverse=True),
        )
        audit_solution(
            CoordinateSolution("TEST", 1, 0.0, 0.0, 0.0, placements),
            context,
            "full",
            0.75,
            items,
            1.2,
            "chemical_below_food",
        )

    def test_greedy_warm_start_prefers_food_on_chemical_over_empty_floor(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 2,
                "length_mm": 100, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [
            self._item(0, (1, 1, 1), 10.0, is_chemical=True),
            self._item(1, (1, 1, 1), 5.0, is_food=True),
        ]
        orientations = {item.index: [(1, 1, 1)] for item in items}
        placements = greedy_coordinate_warm_start(
            context, items, orientations,
            self._config(max_pallets=1, support_mode="full"),
            max_pallets=1,
        )
        self.assertIsNotNone(placements)
        by_item = {placement.item: placement for placement in placements}
        self.assertEqual(by_item[0].z, 0)
        self.assertEqual(by_item[1].z, by_item[0].top)
        self.assertGreater(footprint_overlap(by_item[0], by_item[1]), 0)

    def test_greedy_warm_start_continues_chemicals_and_food_across_pallets(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 2,
                "length_mm": 50, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [
            self._item(0, (1, 1, 1), 30.0, is_chemical=True),
            self._item(1, (1, 1, 1), 25.0, is_chemical=True),
            self._item(2, (1, 1, 1), 20.0, is_chemical=True),
            self._item(3, (1, 1, 1), 10.0, is_food=True),
            self._item(4, (1, 1, 1), 5.0, is_food=True),
        ]
        orientations = {item.index: [(1, 1, 1)] for item in items}
        config = self._config(max_pallets=3, support_mode="full")
        config["food_chemical"] = {"mode": "chemical_below_food"}
        placements = greedy_coordinate_warm_start(
            context, items, orientations, config, max_pallets=3
        )
        self.assertIsNotNone(placements)
        by_item = {placement.item: placement for placement in placements}
        self.assertEqual([by_item[i].pallet for i in (0, 1, 2)], [0, 0, 1])
        self.assertEqual(by_item[3].pallet, 1)
        self.assertEqual(by_item[3].z, by_item[2].top)
        self.assertEqual(by_item[4].pallet, 2)
        self.assertEqual(by_item[4].z, 0)

    def test_greedy_warm_start_uses_a_fitting_six_way_orientation(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 3,
                "length_mm": 100, "width_mm": 50, "height_mm": 150,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        item = CoordinateItem(
            0, 1, 1, (150, 100, 50), (3, 2, 1), 5.0, 1.0,
            "test", False, False, False, False, 1,
        )
        orientations = {0: allowed_orientations(item, "six")}
        config = self._config(max_pallets=1, support_mode="full")
        placements = greedy_coordinate_warm_start(context, [item], orientations, config, 1)
        self.assertIsNotNone(placements)
        self.assertEqual((placements[0].dx, placements[0].dy, placements[0].dz), (2, 1, 3))

    def test_greedy_warm_start_canonicalizes_pallets_and_identical_items(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        prototype = self._item(0, (1, 1, 1), 5.0)
        items = [
            CoordinateItem(
                i, i + 1, prototype.sku, prototype.original_mm, prototype.dims,
                prototype.weight_kg, prototype.volume_dm3, prototype.family,
                prototype.is_food, prototype.is_chemical, prototype.fragile,
                prototype.upright_only, prototype.retrieval_priority,
            )
            for i in range(3)
        ]
        orientations = {i: [(1, 1, 1)] for i in range(3)}
        config = self._config(max_pallets=2, support_mode="off")
        config["symmetry"] = {
            "fix_first_item": True,
            "order_pallet_loads": True,
            "order_identical_items": True,
        }
        placements = greedy_coordinate_warm_start(context, items, orientations, config, 2)
        self.assertIsNotNone(placements)
        self.assertEqual(placements[0].pallet, 0)
        keys = [(q.pallet, q.x, q.y, q.z) for q in placements]
        self.assertEqual(keys, sorted(keys))
        counts = [sum(q.pallet == p for q in placements) for p in range(2)]
        self.assertGreaterEqual(counts[0], counts[1])

    def test_reduced_model_applies_partial_start_without_changing_model(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 2,
                "length_mm": 100, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 5.0), self._item(1, (1, 1, 1), 5.0)]
        config = self._config(max_pallets=1, support_mode="full")
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            dimensions_before = (
                exact.model.NumVars, exact.model.NumConstrs, exact.model.NumGenConstrs
            )
            self.assertTrue(exact.apply_greedy_start())
            self.assertEqual(exact.used[0].Start, 1.0)
            self.assertEqual(sum(exact.assign[i, 0].Start for i in exact.I), 2.0)
            self.assertTrue(all(exact.x[i].Start < 1e100 for i in exact.I))
            self.assertTrue(all(variable.Start >= 1e100 for variable in exact.contact.values()))
            self.assertEqual(
                dimensions_before,
                (exact.model.NumVars, exact.model.NumConstrs, exact.model.NumGenConstrs),
            )
            solution = exact.solve()
            audit_solution(solution, context, "full", 0.75, items, 1.2)
        finally:
            exact.model.dispose()

    def test_reduced_model_warm_start_is_opt_in_and_failure_is_safe(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 600.0), self._item(1, (1, 1, 1), 600.0)]
        disabled = ReducedExactCoordinateMILP(
            context, items, self._config(max_pallets=1, support_mode="off", warm_start=False)
        )
        try:
            self.assertFalse(disabled.apply_greedy_start())
            self.assertGreaterEqual(disabled.x[0].Start, 1e100)
        finally:
            disabled.model.dispose()

        unavailable = ReducedExactCoordinateMILP(
            context, items, self._config(max_pallets=1, support_mode="off", warm_start=True)
        )
        try:
            self.assertFalse(unavailable.apply_greedy_start())
            self.assertGreaterEqual(unavailable.x[0].Start, 1e100)
        finally:
            unavailable.model.dispose()

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

    def test_chemical_top_is_at_or_below_food_in_both_formulations(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 2,
                "length_mm": 50, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [
            self._item(0, (1, 1, 1), is_chemical=True),
            self._item(1, (1, 1, 1), is_food=True),
        ]
        config = self._config(max_pallets=1, support_mode="full", warm_start=False)
        config["food_chemical"] = {"mode": "chemical_below_food"}
        for model_class in (CoordinateBasedMILP, ReducedExactCoordinateMILP):
            with self.subTest(model_class=model_class.__name__):
                exact = model_class(context, items, config)
                try:
                    solution = exact.solve()
                    by_item = {q.item: q for q in solution.placements}
                    self.assertEqual(by_item[0].top, by_item[1].z)
                    self.assertEqual(exact.food_chemical_constraint_count, 1)
                    self.assertIsNotNone(
                        exact.model.getConstrByName("chemical_below_food[0,1,0]")
                    )
                    audit_solution(
                        solution, context, "full", 0.75, items, 1.2,
                        "chemical_below_food",
                    )
                finally:
                    exact.model.dispose()

    def test_equal_height_chemical_and_food_require_separate_pallets(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [
            self._item(0, (1, 1, 1), is_chemical=True),
            self._item(1, (1, 1, 1), is_food=True),
        ]
        config = self._config(max_pallets=2, support_mode="off", warm_start=False)
        config["food_chemical"] = {"mode": "chemical_below_food"}
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            self.assertEqual(solution.pallet_count, 2)
            self.assertEqual(exact.food_chemical_constraint_count, 2)
            audit_solution(
                solution, context, "off", 0.75, items, 1.2,
                "chemical_below_food",
            )
        finally:
            exact.model.dispose()

        config["food_chemical"] = {"mode": "off"}
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            self.assertEqual(solution.pallet_count, 1)
            self.assertEqual(exact.food_chemical_constraint_count, 0)
        finally:
            exact.model.dispose()

    def test_audit_rejects_food_below_chemical_top(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "payload_kg": 1000.0,
            }
        }
        items = [
            self._item(0, (1, 1, 1), is_chemical=True),
            self._item(1, (1, 1, 1), is_food=True),
        ]
        solution = CoordinateSolution(
            "TEST", 1, 1.0, 0.0, 0.0,
            [placement(0, 0, 0, 0, 1, 1, 1), placement(1, 1, 0, 0, 1, 1, 1)],
        )
        with self.assertRaisesRegex(RuntimeError, "chemical/food vertical-order violation"):
            audit_solution(
                solution, context, "off", 0.75, items, 1.2,
                "chemical_below_food",
            )

    def test_mass_incompatible_support_arc_is_not_created(self):
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
            self.assertEqual(by_item[0].z, 0)
            self.assertEqual(by_item[1].z, by_item[0].top)
            self.assertIn((0, 1), exact.contact)
            self.assertNotIn((1, 0), exact.contact)
            self.assertIn((0, 1), solution.support_arcs)
            self.assertEqual(exact.forbidden_support_arc_count, 1)
            self.assertEqual(
                len([c for c in exact.model.getConstrs() if c.ConstrName.startswith("mass_above_ratio")]),
                0,
            )
            audit_solution(solution, context, "full", 0.75, items, 1.2)
        finally:
            exact.model.dispose()

    def test_audit_rejects_mass_incompatible_selected_support(self):
        context = {
            "pallet": {
                "length": 1, "width": 1, "height": 2,
                "length_mm": 50, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 10.0), self._item(1, (1, 1, 1), 5.0)]
        heavy_upper = placement(0, 0, 0, 1, 1, 1, 1)
        light_lower = placement(1, 0, 0, 0, 1, 1, 1)
        solution = CoordinateSolution(
            "TEST", 1, 1.0, 0.0, 0.0, [heavy_upper, light_lower],
            support_arcs=[(1, 0)],
        )
        with self.assertRaisesRegex(RuntimeError, "mass-incompatible support"):
            audit_solution(solution, context, "full", 0.75, items, 1.2)

    def test_mass_order_rule_allows_different_masses_on_the_same_layer(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1), 10.0), self._item(1, (1, 1, 1), 5.0)]
        config = {
            "max_pallets": 1, "time_limit_seconds": 10, "mip_gap": 0.0,
            "log_to_console": False, "rotation_mode": "none",
            "area_auxiliary_type": "integer", "stacking_mass_alpha": 1.2,
            "support": {"mode": "off", "minimum_fraction": 0.75},
            "symmetry": {"fix_first_item": False, "order_pallet_loads": False, "order_identical_items": False},
        }
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            by_item = {placement.item: placement for placement in solution.placements}
            self.assertEqual(solution.pallet_count, 1)
            self.assertEqual(by_item[0].z, 0)
            self.assertEqual(by_item[1].z, 0)
            audit_solution(solution, context, "off", 0.75, items, 1.2)
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

    def test_category_delta_definition_in_both_formulations(self):
        context = {
            "pallet": {
                "length": 6, "width": 2, "height": 2,
                "length_mm": 300, "width_mm": 100, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [
            self._item(0, (1, 1, 1), priority=7),
            self._item(1, (2, 1, 1), priority=7),
            self._item(2, (1, 1, 1), priority=9),
            self._item(3, (1, 1, 1), priority=7),
        ]
        config = self._config(max_pallets=2, support_mode="off", warm_start=False)
        config["rotation_mode"] = "none"
        config["objective_mode"] = "category_distance_only"
        expected_pallet = {0: 0, 1: 0, 2: 0, 3: 1}
        expected_x = {0: 0, 1: 2, 2: 5, 3: 0}
        expected_delta = {
            (1, 0): 2.5,
            (2, 0): 0.0,
            (2, 1): 0.0,
            (3, 0): 11.0,
            (3, 1): 11.0,
            (3, 2): 0.0,
        }

        for model_class in (CoordinateBasedMILP, ReducedExactCoordinateMILP):
            with self.subTest(model_class=model_class.__name__):
                exact = model_class(context, items, config)
                try:
                    for i in exact.I:
                        for p in exact.P:
                            exact.model.addConstr(
                                exact.assign[i, p] == int(expected_pallet[i] == p)
                            )
                        if model_class is CoordinateBasedMILP:
                            p = expected_pallet[i]
                            exact.model.addConstr(exact.x[i, p] == expected_x[i])
                            exact.model.addConstr(exact.y[i, p] == 0)
                            exact.model.addConstr(exact.z[i, p] == 0)
                        else:
                            exact.model.addConstr(exact.x[i] == expected_x[i])
                            exact.model.addConstr(exact.y[i] == 0)
                            exact.model.addConstr(exact.z[i] == 0)

                    solution = exact.solve()
                    self.assertEqual(set(exact.delta), set(expected_delta))
                    for pair, value in expected_delta.items():
                        self.assertAlmostEqual(exact.delta[pair].X, value)
                    self.assertAlmostEqual(solution.category_distance_grid, 24.5)
                    self.assertAlmostEqual(
                        solution.category_distance_objective_bound_grid, 24.5
                    )
                    self.assertEqual(solution.category_distance_mip_gap, 0.0)
                    self.assertTrue(solution.category_distance_stage_attempted)
                    self.assertEqual(solution.objective_mode, "category_distance_only")
                    self.assertTrue(all(i > j for i, j in exact.delta))
                finally:
                    exact.model.dispose()

    def test_category_distance_is_a_standalone_clustering_objective(self):
        context = {
            "pallet": {
                "length": 5, "width": 1, "height": 1,
                "length_mm": 250, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(i, (1, 1, 1), priority=3) for i in range(3)]
        config = self._config(max_pallets=1, support_mode="off", warm_start=False)
        config["rotation_mode"] = "none"
        config["objective_mode"] = "category_distance_only"
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            self.assertAlmostEqual(solution.category_distance_grid, 4.0)
            self.assertTrue(all(exact.used[p].Obj == 0.0 for p in exact.P))
            self.assertEqual(exact.max_height.Obj, 0.0)
            self.assertTrue(all(variable.Obj == 1.0 for variable in exact.delta.values()))
        finally:
            exact.model.dispose()

    def test_fixed_pallet_count_is_enforced_before_distance_optimization(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1)), self._item(1, (1, 1, 1))]
        config = self._config(max_pallets=2, support_mode="off", warm_start=False)
        config["rotation_mode"] = "none"
        config["objective_mode"] = "category_distance_only"
        config["fixed_pallet_count"] = 2
        for model_class in (CoordinateBasedMILP, ReducedExactCoordinateMILP):
            with self.subTest(model_class=model_class.__name__):
                exact = model_class(context, items, config)
                try:
                    solution = exact.solve()
                    self.assertEqual(solution.fixed_pallet_count, 2)
                    self.assertEqual(solution.pallet_count, 2)
                    self.assertAlmostEqual(solution.category_distance_grid, 5.0)
                    self.assertIsNotNone(exact.model.getConstrByName("fixed_pallet_count"))
                finally:
                    exact.model.dispose()

    def test_volume_lower_bound_prunes_pallet_count_in_both_formulations(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        # Three unit-volume boxes need at least ceil(3 / 2) = 2 pallets.
        items = [self._item(i, (1, 1, 1)) for i in range(3)]
        config = self._config(max_pallets=3, support_mode="off", warm_start=False)
        config["rotation_mode"] = "none"
        for model_class in (CoordinateBasedMILP, ReducedExactCoordinateMILP):
            with self.subTest(model_class=model_class.__name__):
                exact = model_class(context, items, config)
                try:
                    self.assertEqual(exact.volume_lower_bound, 2)
                    lower_bound = exact.model.getConstrByName("volume_pallet_lower_bound")
                    self.assertIsNotNone(lower_bound)
                    self.assertEqual(lower_bound.Sense, ">")
                    self.assertEqual(lower_bound.RHS, 2.0)
                    self.assertIsNotNone(exact.model.getConstrByName("fix_volume_lb_pallet[0]"))
                    self.assertIsNotNone(exact.model.getConstrByName("fix_volume_lb_pallet[1]"))
                finally:
                    exact.model.dispose()

    def test_category_distance_then_height_is_exact_lexicographic_with_warm_start(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 2,
                "length_mm": 100, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (1, 1, 1)), self._item(1, (1, 1, 1))]
        config = self._config(max_pallets=1, support_mode="off", warm_start=True)
        config["rotation_mode"] = "none"
        config["fixed_pallet_count"] = 1
        config["objective_mode"] = "category_distance_then_max_height"
        for model_class in (CoordinateBasedMILP, ReducedExactCoordinateMILP):
            with self.subTest(model_class=model_class.__name__):
                exact = model_class(context, items, config)
                try:
                    solution = exact.solve()
                    self.assertEqual(solution.pallet_count, 1)
                    self.assertAlmostEqual(solution.category_distance_grid, 1.0)
                    self.assertEqual(solution.category_distance_objective_bound_grid, 1.0)
                    self.assertTrue(solution.category_distance_stage_attempted)
                    self.assertEqual(solution.max_height_grid, 1)
                    self.assertEqual(solution.height_objective_bound_grid, 1.0)
                    self.assertTrue(solution.height_stage_attempted)
                    self.assertIsNotNone(
                        exact.model.getConstrByName("fix_lexicographic_category_distance")
                    )
                    if model_class is ReducedExactCoordinateMILP:
                        self.assertTrue(exact.greedy_start_applied)
                finally:
                    exact.model.dispose()

    def test_support_area_follows_proven_category_distance_optimum(self):
        context = {
            "pallet": {
                "length": 3, "width": 2, "height": 2,
                "length_mm": 150, "width_mm": 100, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (2, 2, 1)), self._item(1, (2, 2, 1))]
        config = self._config(max_pallets=1, support_mode="fraction", warm_start=False)
        config["rotation_mode"] = "none"
        config["objective_mode"] = "category_distance_only"
        config["support"]["minimum_fraction"] = 0.5
        config["support_area_objective"] = {"enabled": True}
        exact = ReducedExactCoordinateMILP(context, items, config)
        try:
            solution = exact.solve()
            self.assertAlmostEqual(solution.category_distance_grid, 1.0)
            self.assertEqual(solution.support_area_grid2, 4.0)
            self.assertTrue(solution.support_area_stage_attempted)
            self.assertIsNotNone(
                exact.model.getConstrByName("fix_lexicographic_category_distance")
            )
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
            self.assertEqual(solution.footprint_depth_lower_bound, 1)
            self.assertEqual(solution.footprint_height_lower_bound_grid, 1)
            self.assertIsNotNone(
                exact.model.getConstrByName("footprint_height_lower_bound[1]")
            )
        finally:
            exact.model.dispose()

    def test_secondary_objective_minimizes_average_box_top_height(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 2,
                "length_mm": 100, "width_mm": 50, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(i, (1, 1, 1), 5.0) for i in range(3)]
        config = {
            "max_pallets": 1,
            "time_limit_seconds": 10,
            "pallet_count_time_limit_seconds": 10,
            "height_time_limit_seconds": 10,
            "mip_gap": 0.0,
            "log_to_console": False,
            "rotation_mode": "none",
            "area_auxiliary_type": "integer",
            "stacking_mass_alpha": 1.2,
            "objective_mode": "pallets_then_average_height",
            "support": {"mode": "full", "minimum_fraction": 0.75},
            "symmetry": {
                "fix_first_item": False,
                "order_pallet_loads": False,
                "order_identical_items": False,
            },
        }
        for model_class in (CoordinateBasedMILP, ReducedExactCoordinateMILP):
            with self.subTest(model_class=model_class.__name__):
                exact = model_class(context, items, config)
                try:
                    solution = exact.solve()
                    self.assertEqual(solution.pallet_count, 1)
                    self.assertAlmostEqual(solution.average_top_height_grid, 4 / 3)
                    self.assertAlmostEqual(solution.height_objective_bound_grid, 4 / 3)
                    self.assertEqual(solution.height_mip_gap, 0.0)
                    self.assertTrue(solution.height_stage_attempted)
                    self.assertIsNotNone(
                        exact.model.getConstrByName("fix_lexicographic_pallet_count")
                    )
                finally:
                    exact.model.dispose()

    def test_optional_third_objective_maximizes_exact_support_area(self):
        context = {
            "pallet": {
                "length": 3, "width": 2, "height": 2,
                "length_mm": 150, "width_mm": 100, "height_mm": 100,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        items = [self._item(0, (2, 2, 1)), self._item(1, (2, 2, 1))]
        config = self._config(max_pallets=1, support_mode="fraction", warm_start=False)
        config["rotation_mode"] = "none"
        config["objective_mode"] = "pallets_then_max_height"
        config["support"]["minimum_fraction"] = 0.5
        config["support_area_objective"] = {"enabled": True}
        for model_class in (CoordinateBasedMILP, ReducedExactCoordinateMILP):
            with self.subTest(model_class=model_class.__name__):
                exact = model_class(context, items, config)
                try:
                    solution = exact.solve()
                    self.assertEqual(solution.pallet_count, 1)
                    self.assertEqual(solution.max_height_grid, 2)
                    self.assertTrue(solution.support_area_stage_attempted)
                    self.assertEqual(solution.support_area_grid2, 4.0)
                    self.assertEqual(solution.support_area_objective_bound_grid2, 4.0)
                    self.assertEqual(solution.support_area_mip_gap, 0.0)
                    self.assertIsNotNone(
                        exact.model.getConstrByName("fix_lexicographic_max_height")
                    )
                    audit_solution(solution, context, "fraction", 0.5, items, 1.2)
                finally:
                    exact.model.dispose()

    def test_support_area_objective_rejects_modes_without_exact_area(self):
        context = {
            "pallet": {
                "length": 2, "width": 1, "height": 1,
                "length_mm": 100, "width_mm": 50, "height_mm": 50,
                "payload_kg": 1000.0,
            },
            "grid_mm": 50,
        }
        config = self._config(max_pallets=1, support_mode="off", warm_start=False)
        config["objective_mode"] = "pallets_then_max_height"
        config["support_area_objective"] = {"enabled": True}
        exact = ReducedExactCoordinateMILP(context, [self._item(0, (1, 1, 1))], config)
        try:
            with self.assertRaisesRegex(ValueError, "support.mode=fraction or full"):
                exact.solve()
        finally:
            exact.model.dispose()


if __name__ == "__main__":
    unittest.main()
