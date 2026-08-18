import unittest
from pathlib import Path

from gurobi_coordinate_solver import (
    CoordinateItem,
    CoordinatePlacement,
    CoordinateSolution,
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


if __name__ == "__main__":
    unittest.main()
