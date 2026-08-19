import unittest

from gurobi_pallet_solver import (
    ExactItem,
    Placement,
    footprint_overlap,
    generate_placements,
    support_fraction,
)


def placement(
    identifier,
    item,
    x,
    y,
    z,
    dx,
    dy,
    dz,
    pallet=0,
):
    return Placement(identifier, item, pallet, 0, x, y, z, dx, dy, dz)


class TestSupportGeometry(unittest.TestCase):
    def test_pallet_provides_full_support(self):
        upper = placement(0, 0, 0, 0, 0, 10, 10, 2)
        self.assertEqual(support_fraction(upper, []), 1.0)

    def test_exactly_seventy_five_percent_support(self):
        lower = placement(0, 0, 0, 0, 0, 75, 100, 10)
        upper = placement(1, 1, 0, 0, 10, 100, 100, 10)
        self.assertEqual(footprint_overlap(upper, lower), 7500)
        self.assertAlmostEqual(support_fraction(upper, [lower]), 0.75)

    def test_seventy_four_percent_is_below_threshold(self):
        lower = placement(0, 0, 0, 0, 0, 74, 100, 10)
        upper = placement(1, 1, 0, 0, 10, 100, 100, 10)
        self.assertAlmostEqual(support_fraction(upper, [lower]), 0.74)
        self.assertLess(support_fraction(upper, [lower]), 0.75)

    def test_bridge_support_sums_two_separated_boxes(self):
        left = placement(0, 0, 0, 0, 0, 40, 100, 10)
        right = placement(1, 1, 60, 0, 0, 40, 100, 10)
        upper = placement(2, 2, 0, 0, 10, 100, 100, 10)
        self.assertAlmostEqual(support_fraction(upper, [left, right]), 0.80)

    def test_non_contacting_lower_box_does_not_support(self):
        lower = placement(0, 0, 0, 0, 0, 100, 100, 9)
        upper = placement(1, 1, 0, 0, 10, 100, 100, 10)
        self.assertEqual(support_fraction(upper, [lower]), 0.0)


class TestFullGridPlacementGeneration(unittest.TestCase):
    def test_enumerates_every_boundary_feasible_grid_coordinate(self):
        item = ExactItem(
            index=0,
            id=1,
            sku=1,
            original_mm=(2, 2, 1),
            dims=(2, 2, 1),
            weight_kg=1.0,
            volume_dm3=1.0,
            density_kg_m3=1.0,
            family="test",
            is_food=False,
            is_chemical=False,
            fragile=False,
            upright_only=True,
            retrieval_priority=1,
        )
        pallet = {"length": 5, "width": 4, "height": 3}

        placements, by_item = generate_placements(
            [item], pallet, max_pallets=1, rotation_mode="none"
        )

        expected_coordinates = {
            (x, y, z)
            for x in range(5 - 2 + 1)
            for y in range(4 - 2 + 1)
            for z in range(3 - 1 + 1)
        }
        actual_coordinates = {(q.x, q.y, q.z) for q in placements}

        self.assertEqual(actual_coordinates, expected_coordinates)
        self.assertIn((1, 1, 1), actual_coordinates)
        self.assertEqual(len(placements), 4 * 3 * 3)
        self.assertEqual(by_item[0], list(range(len(placements))))


if __name__ == "__main__":
    unittest.main()
