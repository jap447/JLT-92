#!/usr/bin/env python

import numpy as np
import os
import rosunit
import unittest

from planning import search
from planning.problems import R2Problem


class TestRRT(unittest.TestCase):
    def setUp(self):
        fname = os.path.join(os.path.dirname(__file__), "share", "map1.txt")
        self.permissible_region = np.loadtxt(fname, dtype=bool)
        self.problem = R2Problem(self.permissible_region, check_resolution=0.5)

    def test_rrt_r2problem(self):

        rrt = search.RRTPlanner(self.problem, self.permissible_region)

        start = np.array([[1., 6.]])
        goal = np.array([[9., 3.]])
        path = rrt.Plan(start, goal)
        self.assertIsNotNone(path)

        self.assertTrue(
            np.allclose(path[0], start),
            msg="Path must begin at start",
        )

        self.assertTrue(
            np.allclose(path[-1], goal),
            msg="Path must end at goal",
        )
        
        # Correct xy sampling and unrounded distances can change the sampled
        # route. Check its geometry rather than one particular random path.
        self.assertTrue(self.problem.check_state_validity(path).all())
        for first, second in zip(path[:-1], path[1:]):
            self.assertTrue(
                self.problem.check_edge_validity(first, second),
                msg="Every path segment must be collision-free",
            )

if __name__ == "__main__":
    np.random.seed(112)
    rosunit.unitrun("planning", "test_rrt_r2", TestRRT)
