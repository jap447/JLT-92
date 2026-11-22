#!/usr/bin/env python3
"""RRT and ROS adapter regression checks; no ROS master or build required."""

import io
from pathlib import Path
import runpy
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid

from planning.problems import R2Problem
from planning.search import RRTPlanner

RRTNode = runpy.run_path(
    str(Path(__file__).resolve().parents[1] / "scripts/rrt_node.py")
)["RRTNode"]


class TestRRTMapPlanning(unittest.TestCase):
    def setUp(self):
        np.random.seed(112)

    def test_samples_use_xy_on_rectangular_map(self):
        grid = np.zeros((3, 8), dtype=bool)
        grid[1, 6] = True
        planner = RRTPlanner(R2Problem(grid), grid, bias=0)
        goal = np.array([[6., 1.]])
        np.testing.assert_array_equal(planner.sample(goal), goal)
        np.testing.assert_array_equal(planner.sample_batch(goal, 4), np.tile(goal, (4, 1)))

    def test_subcell_distance_is_not_rounded(self):
        problem = R2Problem(np.ones((3, 8), dtype=bool))
        self.assertAlmostEqual(problem.compute_distance_rrt(
            np.array([1., 1.]), np.array([1.25, 1.])
        ), 0.25)

    def test_failure_and_invalid_endpoints_return_none(self):
        grid = np.ones((3, 8), dtype=bool)
        grid[:, 4] = False
        planner = RRTPlanner(R2Problem(grid), grid, bias=1, max_iter=3)
        self.assertIsNone(planner.Plan(np.array([1., 1.]), np.array([6., 1.])))
        self.assertIsNone(planner.Plan(np.array([-1., 1.]), np.array([6., 1.])))

    def test_near_goal_connection_is_checked(self):
        grid = np.ones((3, 8), dtype=bool)
        problem = R2Problem(grid)
        planner = RRTPlanner(problem, grid, max_iter=1)
        start, near, goal = (np.array([[x, 1.]]) for x in (1., 2., 2.5))
        with patch.object(planner, "sample", return_value=near), patch.object(
            problem, "check_edge_validity", side_effect=[True, False]
        ) as check:
            self.assertIsNone(planner.Plan(start, goal, epsilon=1))
            self.assertEqual(check.call_count, 2)
            np.testing.assert_array_equal(check.call_args.args[0], near)
            np.testing.assert_array_equal(check.call_args.args[1], goal)

    def test_near_goal_vertex_is_preserved_in_path(self):
        grid = np.ones((3, 8), dtype=bool)
        planner = RRTPlanner(R2Problem(grid), grid, max_iter=1)
        start, near, goal = (np.array([[x, 1.]]) for x in (1., 2., 2.5))
        with patch.object(planner, "sample", return_value=near):
            plan = planner.Plan(start, goal, epsilon=1)
        np.testing.assert_array_equal(plan, np.vstack((start, near, goal)))

    def test_identical_endpoints_and_repeated_queries(self):
        grid = np.ones((3, 8), dtype=bool)
        planner = RRTPlanner(R2Problem(grid), grid, bias=1, max_iter=1)
        start, goal = np.array([[1., 1.]]), np.array([[6., 1.]])
        np.testing.assert_array_equal(planner.Plan(start, start), start)
        np.testing.assert_array_equal(planner.Plan(start, goal), np.vstack((start, goal)))
        np.testing.assert_array_equal(planner.Plan(goal, start), np.vstack((goal, start)))


class TestRRTROSAdapter(unittest.TestCase):
    def setUp(self):
        for name in ("Publisher", "Subscriber", "logwarn"):
            patcher = patch("rospy." + name)
            setattr(self, name, patcher.start())
            self.addCleanup(patcher.stop)
        clock = patch("rospy.Time.now", return_value=rospy.Time(1))
        clock.start()
        self.addCleanup(clock.stop)
        self.Publisher.side_effect = lambda *args, **kwargs: MagicMock()
        self.node = RRTNode()
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.width = 8
        self.map.info.height = 3
        self.map.info.resolution = 0.5
        self.map.info.origin.position.x = -2.
        self.map.info.origin.position.y = -3.
        self.map.info.origin.orientation.w = 1.
        self.map.data = [-1] + [0] * 23
        self.start = PoseWithCovarianceStamped()
        self.start.header.frame_id = "map"
        self.start.pose.pose.position.x = -1.5
        self.start.pose.pose.position.y = -2.5
        self.goal = PoseStamped()
        self.goal.header.frame_id = "map"
        self.goal.pose.position.x = 1.
        self.goal.pose.position.y = -2.5

    def test_goal_before_map_or_start(self):
        self.node.goal_callback(self.goal)
        self.node.map_callback(self.map)
        self.node.goal_callback(self.goal)
        self.assertEqual(self.logwarn.call_count, 2)

    def test_wrong_frame_and_unknown_cell_rejected(self):
        self.node.map_callback(self.map)
        self.node.initial_pose_callback(self.start)
        with patch.object(RRTPlanner, "Plan") as plan:
            self.goal.header.frame_id = "odom"
            self.node.goal_callback(self.goal)
            self.goal.header.frame_id = "map"
            self.goal.pose.position.x = -2.
            self.goal.pose.position.y = -3.
            self.node.goal_callback(self.goal)
            plan.assert_not_called()

    def test_clicked_poses_publish_world_coordinate_path(self):
        np.random.seed(112)
        self.node.map_callback(self.map)
        self.node.initial_pose_callback(self.start)
        self.node.goal_callback(self.goal)
        marker = self.node.path_pub.publish.call_args.args[0]
        self.assertEqual(marker.header.frame_id, "map")
        self.assertAlmostEqual(marker.points[0].x, -1.5)
        self.assertAlmostEqual(marker.points[0].y, -2.5)
        self.assertAlmostEqual(marker.points[-1].x, 1.)
        self.assertAlmostEqual(marker.points[-1].y, -2.5)
        marker.serialize(io.BytesIO())
        tree_marker = self.node.explore_pub.publish.call_args.args[0]
        self.assertGreater(len(tree_marker.points), 0)
        tree_marker.serialize(io.BytesIO())
        self.logwarn.assert_not_called()


if __name__ == "__main__":
    unittest.main()
