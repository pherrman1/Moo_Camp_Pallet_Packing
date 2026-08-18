import unittest

from gurobi_pallet_solver import Placement, footprint_overlap, support_fraction


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


if __name__ == "__main__":
    unittest.main()
