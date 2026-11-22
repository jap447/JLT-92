#!/usr/bin/env python3
"""Regression tests for Q3 result checking and worker isolation."""

import multiprocessing
import os
from pathlib import Path
import time
import unittest

import numpy as np

from tradeoff import run_rrt


class DirectPlanner:
    def __init__(self, *args, **kwargs):
        pass

    def Plan(self, start, goal):
        return np.vstack((start, goal))


class NoPathPlanner(DirectPlanner):
    def Plan(self, start, goal):
        return None


class BrokenPlanner(DirectPlanner):
    def Plan(self, start, goal):
        raise RuntimeError("deliberate planner failure")


class CrashedPlanner(DirectPlanner):
    def Plan(self, start, goal):
        os._exit(1)


class SlowPlanner(DirectPlanner):
    def Plan(self, start, goal):
        time.sleep(60)


class WrongGoalPlanner(DirectPlanner):
    def Plan(self, start, goal):
        return np.vstack((start, start + 1))


class InvalidStatePlanner(DirectPlanner):
    def Plan(self, start, goal):
        return np.vstack((start, [-1., 2.], goal))


class EmptyPlanner(DirectPlanner):
    def Plan(self, start, goal):
        return np.empty((0, 2))


class NonfinitePlanner(DirectPlanner):
    def Plan(self, start, goal):
        return np.vstack((start, [np.nan, 2.], goal))


class RandomPlanner(DirectPlanner):
    def Plan(self, start, goal):
        return np.vstack((start, np.random.uniform(0, 5, (1, 2)), goal))


class TestTradeoffRunner(unittest.TestCase):
    def setUp(self):
        self.grid = np.ones((5, 5), dtype=bool)
        self.start = np.array([0.5, 0.5])
        self.goal = np.array([4.5, 4.5])
        self.children = {child.pid for child in multiprocessing.active_children()}

    def tearDown(self):
        self.assertEqual(
            {child.pid for child in multiprocessing.active_children()}, self.children
        )

    def run_planner(self, planner, timeout=10.0):
        return run_rrt(self.grid, self.start, self.goal, timeout=timeout,
                       planner_factory=planner)

    def test_valid_path(self):
        status, path = self.run_planner(DirectPlanner)
        self.assertEqual(status, "path")
        np.testing.assert_array_equal(path, [self.start, self.goal])

    def test_real_rrt_returns_valid_path(self):
        grid = np.loadtxt(Path(__file__).parent / "share/map1.txt", dtype=bool)
        status, detail = run_rrt(grid, np.array([1., 6.]), np.array([9., 3.]))
        self.assertEqual(status, "path", msg=str(detail))

    def test_no_path_is_not_success(self):
        self.assertEqual(self.run_planner(NoPathPlanner), ("no_path", None))

    def test_exception_is_an_error(self):
        status, detail = self.run_planner(BrokenPlanner)
        self.assertEqual(status, "error")
        self.assertIn("deliberate planner failure", detail)

    def test_crashed_worker_is_an_error(self):
        status, detail = self.run_planner(CrashedPlanner)
        self.assertEqual(status, "error")
        self.assertIn("without reporting a result", detail)

    def test_timeout_reaps_worker_and_next_run_succeeds(self):
        self.assertEqual(self.run_planner(SlowPlanner, timeout=0.1), ("timeout", None))
        self.assertEqual(
            {child.pid for child in multiprocessing.active_children()}, self.children
        )
        self.assertEqual(self.run_planner(DirectPlanner)[0], "path")

    def test_invalid_results_are_errors(self):
        for planner in (WrongGoalPlanner, InvalidStatePlanner, EmptyPlanner,
                        NonfinitePlanner):
            with self.subTest(planner=planner.__name__):
                self.assertEqual(self.run_planner(planner)[0], "error")

    def test_collision_between_valid_endpoints_is_an_error(self):
        self.grid[2, 2] = False
        status, detail = self.run_planner(DirectPlanner)
        self.assertEqual(status, "error")
        self.assertIn("invalid edge", detail)

    def test_worker_seed_is_repeatable_and_parent_rng_is_unchanged(self):
        state = np.random.get_state()
        first_status, first = self.run_planner(RandomPlanner)
        second_status, second = self.run_planner(RandomPlanner)
        self.assertEqual(first_status, "path")
        self.assertEqual(second_status, "path")
        np.testing.assert_array_equal(first, second)
        after = np.random.get_state()
        self.assertEqual(state[0], after[0])
        np.testing.assert_array_equal(state[1], after[1])
        self.assertEqual(state[2:], after[2:])


if __name__ == "__main__":
    unittest.main()
