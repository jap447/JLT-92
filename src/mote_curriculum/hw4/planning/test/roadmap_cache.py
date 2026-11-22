#!/usr/bin/env python3
"""Disk cache regressions; all cache files live in a temporary directory."""

import pickle
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from planning.problems import R2Problem
from planning.roadmap import Roadmap
from planning.samplers import LatticeSampler
from planning.search import ASTARPlanner


class TestRoadmapCache(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "roadmap.pkl"
        grid = np.ones((10, 10), dtype=bool)
        grid[3:7, 3:7] = False
        self.problem = R2Problem(grid)

    def build(self):
        return Roadmap(self.problem, LatticeSampler(self.problem.extents),
                       100, 2., saveto=str(self.path))

    def test_lattice_cache_restores_actual_vertex_count(self):
        built = self.build()
        self.assertLess(built.num_vertices, 100)
        with patch.object(Roadmap, "sample_vertices", side_effect=AssertionError("must load")):
            loaded = self.build()
        self.assertTrue(loaded.loaded_from_cache)
        self.assertEqual(loaded.num_vertices, len(loaded.vertices))
        self.assertEqual(loaded.num_vertices, built.num_vertices)
        start = loaded.add_node(np.array([1., 1.]), is_start=True)
        goal = loaded.add_node(np.array([9., 9.]), is_start=False)
        path, _ = ASTARPlanner(loaded).Plan(start, goal)
        self.assertEqual(path[0], start)
        self.assertEqual(path[-1], goal)

    def test_truncated_cache_rebuilds(self):
        self.path.write_bytes(b'\x80\x04')
        rebuilt = self.build()
        self.assertFalse(rebuilt.loaded_from_cache)
        self.assertTrue(self.build().loaded_from_cache)

    def test_inconsistent_cache_rebuilds(self):
        self.build()
        with self.path.open("rb") as stream:
            data = pickle.load(stream)
        data["vertices"] = data["vertices"][:-1]
        with self.path.open("wb") as stream:
            pickle.dump(data, stream)
        self.assertFalse(self.build().loaded_from_cache)
        self.assertTrue(self.build().loaded_from_cache)

    def test_failed_atomic_write_preserves_existing_file(self):
        self.path.write_bytes(b"old incomplete cache")
        with patch("planning.roadmap.os.replace", side_effect=OSError("write failed")):
            built = self.build()
        self.assertGreater(built.num_vertices, 0)
        self.assertEqual(self.path.read_bytes(), b"old incomplete cache")
        self.assertEqual(list(self.path.parent.iterdir()), [self.path])


if __name__ == "__main__":
    unittest.main()
