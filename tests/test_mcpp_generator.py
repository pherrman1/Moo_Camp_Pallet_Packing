import tempfile
import unittest
from pathlib import Path

from generator.mcpp_generator import (
    PALLET_EURO_180,
    generate_instance,
    to_solver_txt,
)
from packing3d import Bins, Cases, Variables, build_cqm
from utils import read_instance


class TestMcppSolverExport(unittest.TestCase):
    def test_generated_instance_builds_solver_cqm(self):
        instance = generate_instance(
            n_items=12,
            instance_class=4,
            seed=7,
            pallet=PALLET_EURO_180,
            extended=True,
        )

        with tempfile.TemporaryDirectory() as tempdir:
            path = Path(tempdir) / "generated_solver.txt"
            path.write_text(to_solver_txt(instance, max_bins=2), encoding="utf-8")
            data = read_instance(str(path))

        cases = Cases(data)
        bins = Bins(data, cases)
        variables = Variables(cases, bins)
        cqm, _ = build_cqm(variables, bins, cases)

        self.assertEqual(cases.num_cases, 12)
        self.assertEqual(bins.num_bins, 2)
        self.assertEqual(data["bin_dimensions"], [1200, 800, 1800])
        self.assertGreater(len(cqm.variables), 0)
        self.assertGreater(len(cqm.constraints), 0)


if __name__ == "__main__":
    unittest.main()
