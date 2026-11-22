#!/usr/bin/env python3
"""Check actual CLI parsing and heading conversion without plotting."""

from pathlib import Path
import runpy
import unittest
from unittest.mock import patch

import numpy as np


class TestAStarCLI(unittest.TestCase):
    def test_cli_preserves_headings_and_positions(self):
        package = Path(__file__).resolve().parents[1]
        script = package / "scripts" / "run_astar"
        for mode, start, goal, expected_start, expected_goal in (
            ("se2", [1, 6, 90], [9, 3, 45],
             [1, 6, np.pi / 2], [9, 3, np.pi / 4]),
            ("se2", [1, 6, -90], [9, 3, 180],
             [1, 6, -np.pi / 2], [9, 3, np.pi]),
            ("r2", [1, 6], [9, 3], [1, 6], [9, 3]),
        ):
            with self.subTest(mode=mode, goal=goal):
                argv = [str(script), "--netids", "test", "-m",
                        str(package / "test/share/map1.txt"), "-n", "25",
                        "-r", "3", mode, "-s"]
                argv += [str(value) for value in start]
                argv += ["-g"] + [str(value) for value in goal]
                with patch("sys.argv", argv), \
                        patch("planning.roadmap.Roadmap") as roadmap, \
                        patch("planning.search.ASTARPlanner") as planner:
                    roadmap.return_value.add_node.side_effect = [0, 1]
                    planner.return_value.Plan.return_value = ([0, 1], None)
                    runpy.run_path(str(script), run_name="__main__")
                    calls = roadmap.return_value.add_node.call_args_list
                    self.assertEqual(len(calls), 2)
                    np.testing.assert_allclose(calls[0][0][0], expected_start)
                    np.testing.assert_allclose(calls[1][0][0], expected_goal)


if __name__ == "__main__":
    unittest.main()
