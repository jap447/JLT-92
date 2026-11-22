#!/usr/bin/env python3
"""Exercise A* and its ROS adapter without a ROS master."""

import copy
import io
from pathlib import Path
import runpy
import tempfile
import unittest
from unittest.mock import MagicMock, patch

import numpy as np
import networkx as nx
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from planning.problems import R2Problem, SE2Problem
from planning.roadmap import Roadmap
from planning import utils

AStarNode = runpy.run_path(
    str(Path(__file__).resolve().parents[1] / "scripts/astar_node.py")
)["AStarNode"]


class TestAStarROSAdapter(unittest.TestCase):
    def setUp(self):
        cache = tempfile.TemporaryDirectory()
        self.addCleanup(cache.cleanup)
        self.cache_dir = Path(cache.name)
        self.params = {
            "~num_vertices": 20, "~connection_radius": 10.0, "~sampler": "halton",
            "~cache_dir": cache.name,
        }
        for name in ("Publisher", "Subscriber", "logwarn", "loginfo", "get_param"):
            patcher = patch("rospy." + name)
            setattr(self, name, patcher.start())
            self.addCleanup(patcher.stop)
        self.get_param.side_effect = lambda name, default: self.params.get(name, default)
        self.Publisher.side_effect = self.make_publisher
        clock = patch("rospy.Time.now", return_value=rospy.Time(1))
        clock.start()
        self.addCleanup(clock.stop)
        self.node = AStarNode()
        self.map = OccupancyGrid()
        self.map.header.frame_id = "map"
        self.map.info.width, self.map.info.height = 8, 3
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

    @staticmethod
    def make_publisher(*args, **kwargs):
        publisher = MagicMock()
        publisher.messages = []
        publisher.publish.side_effect = lambda msg: publisher.messages.append(copy.deepcopy(msg))
        return publisher

    def supply_inputs(self):
        self.node.map_callback(self.map)
        self.node.initial_pose_callback(self.start)

    def test_goal_before_map_or_start(self):
        self.node.goal_callback(self.goal)
        self.node.map_callback(self.map)
        self.node.goal_callback(self.goal)
        self.assertEqual(self.logwarn.call_count, 2)
        self.node.path_pub.publish.assert_not_called()

    def test_world_coordinate_path_and_roadmap(self):
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        path = self.node.path_pub.messages[-1]
        self.assertEqual(path.action, Marker.ADD)
        self.assertEqual(path.ns, "astar_viz")
        self.assertEqual(path.header.frame_id, "map")
        np.testing.assert_allclose([path.points[0].x, path.points[0].y], [-1.5, -2.5])
        np.testing.assert_allclose([path.points[-1].x, path.points[-1].y], [1., -2.5])
        path.serialize(io.BytesIO())
        roadmap = self.node.roadmap_pub.messages[-1]
        self.assertGreater(len(roadmap.points), 0)
        self.assertEqual(len(roadmap.points) % 2, 0)
        roadmap.serialize(io.BytesIO())
        self.logwarn.assert_not_called()

    def test_invalid_poses_do_not_plan(self):
        self.supply_inputs()
        cases = [("odom", 1., -2.5), ("map", -2., -3.),
                 ("map", 100., -2.5), ("map", float("nan"), -2.5)]
        for frame, x, y in cases:
            with self.subTest(frame=frame, x=x):
                self.goal.header.frame_id = frame
                self.goal.pose.position.x, self.goal.pose.position.y = x, y
                self.node.goal_callback(self.goal)
                self.assertEqual(self.node.path_pub.messages[-1].action, Marker.DELETE)
        self.assertEqual(self.logwarn.call_count, len(cases))

    def test_identical_endpoints_clear_previous_path(self):
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        self.goal.pose.position = copy.deepcopy(self.start.pose.pose.position)
        self.node.goal_callback(self.goal)
        self.assertEqual(self.node.path_pub.messages[-1].action, Marker.DELETE)
        self.assertIn("already equals goal", self.loginfo.call_args.args[0])

    def test_obstacle_detour_is_collision_free(self):
        grid = np.zeros((3, 8), dtype=int)
        grid[0, 0] = -1
        grid[0:2, 4] = 100
        self.map.data = grid.ravel().tolist()
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        marker = self.node.path_pub.messages[-1]
        self.assertEqual(marker.action, Marker.ADD)
        problem = R2Problem(grid == 0)
        points = (np.array([[p.x, p.y] for p in marker.points]) + [2., 3.]) / 0.5
        self.assertGreater(len(points), 2)
        for i in range(0, len(points), 2):
            self.assertTrue(problem.check_edge_validity(points[i], points[i + 1]))

    def test_no_path_then_recovery_on_new_map(self):
        grid = np.zeros((3, 8), dtype=int)
        grid[:, 4] = 100
        self.map.data = grid.ravel().tolist()
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        self.assertEqual(self.node.path_pub.messages[-1].action, Marker.DELETEALL)
        self.logwarn.assert_called_once()
        self.assertEqual(self.node.roadmap_pub.messages[-1].action, Marker.ADD)
        self.map.data = [0] * 24
        self.node.map_callback(self.map)
        self.node.goal_callback(self.goal)
        self.assertEqual(self.node.path_pub.messages[-1].action, Marker.ADD)

    def test_reuse_preserves_base_and_roadmap_display(self):
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        base = self.node.base_roadmap
        vertices = base.vertices.copy()
        edges = base.weighted_edges.copy()
        graph_edges = list(base.graph.edges(data=True))
        display_count = len(self.node.roadmap_pub.messages)
        self.goal.pose.position.y = -2.
        with patch.object(Roadmap, "construct", side_effect=AssertionError("must reuse")):
            self.node.goal_callback(self.goal)
        self.assertIs(self.node.base_roadmap, base)
        np.testing.assert_array_equal(base.vertices, vertices)
        np.testing.assert_array_equal(base.weighted_edges, edges)
        self.assertEqual(list(base.graph.edges(data=True)), graph_edges)
        self.assertEqual(base.num_vertices, len(vertices))
        self.assertIsNone(base.start)
        self.assertIsNone(base.goal)
        self.assertEqual(len(self.node.roadmap_pub.messages), display_count)
        self.assertEqual(self.node.path_pub.messages[-1].action, Marker.ADD)
        self.assertEqual(len(list(self.cache_dir.glob("*.pkl"))), 1)

    def test_restart_loads_disk_cache_without_sampling(self):
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        fresh = AStarNode()
        fresh.map_callback(self.map)
        fresh.initial_pose_callback(self.start)
        with patch.object(Roadmap, "sample_vertices", side_effect=AssertionError("must load")):
            fresh.goal_callback(self.goal)
        self.assertTrue(fresh.base_roadmap.loaded_from_cache)
        self.assertEqual(fresh.base_roadmap.num_vertices, 20)
        self.assertEqual(fresh.path_pub.messages[-1].action, Marker.ADD)

    def test_timestamps_do_not_invalidate_map(self):
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        base = self.node.base_roadmap
        display_count = len(self.node.roadmap_pub.messages)
        republished = copy.deepcopy(self.map)
        republished.header.seq = 9
        republished.header.stamp = rospy.Time(100)
        republished.info.map_load_time = rospy.Time(100)
        self.node.map_callback(republished)
        with patch.object(Roadmap, "construct", side_effect=AssertionError("must reuse")):
            self.node.goal_callback(self.goal)
        self.assertIs(self.node.base_roadmap, base)
        self.assertEqual(len(self.node.roadmap_pub.messages), display_count)

    def test_changed_map_rebuilds_and_switching_back_loads(self):
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        original_key = self.node.roadmap_key
        changed = copy.deepcopy(self.map)
        changed.data[0] = 0
        self.node.map_callback(changed)
        self.assertIsNone(self.node.base_roadmap)
        self.assertEqual(self.node.roadmap_pub.messages[-1].action, Marker.DELETEALL)
        self.node.goal_callback(self.goal)
        self.assertNotEqual(self.node.roadmap_key, original_key)
        self.assertFalse(self.node.base_roadmap.loaded_from_cache)
        self.assertEqual(len(list(self.cache_dir.glob("*.pkl"))), 2)
        self.node.map_callback(self.map)
        with patch.object(Roadmap, "sample_vertices", side_effect=AssertionError("must load")):
            self.node.goal_callback(self.goal)
        self.assertTrue(self.node.base_roadmap.loaded_from_cache)
        self.assertEqual(self.node.roadmap_key, original_key)

    def test_cache_key_includes_geometry_and_settings(self):
        original = self.node.cache_key(self.map)
        for attr, value in [("num_vertices", 21), ("connection_radius", 3.),
                            ("sampler_name", "lattice"), ("check_resolution", 0.2)]:
            with self.subTest(attr=attr), patch.object(self.node, attr, value):
                self.assertNotEqual(self.node.cache_key(self.map), original)
        for attr, value in [("resolution", 0.2), ("width", 4), ("height", 6)]:
            changed = copy.deepcopy(self.map)
            setattr(changed.info, attr, value)
            self.assertNotEqual(self.node.cache_key(changed), original)
        changed = copy.deepcopy(self.map)
        changed.info.origin.position.x += 1.
        self.assertNotEqual(self.node.cache_key(changed), original)
        changed = copy.deepcopy(self.map)
        changed.info.origin.orientation.z = 0.5
        self.assertNotEqual(self.node.cache_key(changed), original)
        changed = copy.deepcopy(self.map)
        changed.header.frame_id = "new_map"
        self.assertNotEqual(self.node.cache_key(changed), original)

    def test_disabled_disk_cache_still_reuses_memory(self):
        self.node.use_disk_cache = False
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        with patch.object(Roadmap, "construct", side_effect=AssertionError("must reuse")):
            self.node.goal_callback(self.goal)
        self.assertEqual(list(self.cache_dir.iterdir()), [])

    def test_failed_query_leaves_base_unchanged(self):
        self.node.connection_radius = 0.01
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        base = self.node.base_roadmap
        self.assertEqual(base.num_vertices, 20)
        self.assertEqual(base.graph.number_of_nodes(), 20)
        self.assertEqual(len(base.vertices), 20)
        with patch.object(Roadmap, "construct", side_effect=AssertionError("must reuse")):
            self.node.goal_callback(self.goal)
        self.assertEqual(base.graph.number_of_nodes(), 20)
        self.assertEqual(self.logwarn.call_count, 2)

    def test_invalid_parameters(self):
        for name, value in [("~num_vertices", 0), ("~connection_radius", -1.),
                            ("~connection_radius", float("nan")), ("~sampler", "arm_random"),
                            ("~dubins", "false")]:
            with self.subTest(name=name, value=value), patch.dict(self.params, {name: value}):
                with self.assertRaises(ValueError):
                    AStarNode()

    def configure_dubins(self):
        self.params.update({"~dubins": True, "~curvature": 0.5,
                            "~connection_radius": 30.0})
        self.node = AStarNode()
        self.map.info.width = self.map.info.height = 40
        self.map.data = [0] * 1600
        # Quarter circle of radius 2 meters followed by a 1 meter straight.
        self.start.pose.pose.position.x, self.start.pose.pose.position.y = 3., 2.
        self.start.pose.pose.orientation = utils.angle_to_quaternion(0.)
        self.goal.pose.position.x, self.goal.pose.position.y = 5., 5.
        self.goal.pose.orientation = utils.angle_to_quaternion(np.pi / 2)
        self.supply_inputs()

    def test_dubins_curves_use_headings_and_metric_curvature(self):
        self.configure_dubins()
        self.node.goal_callback(self.goal)
        base = self.node.base_roadmap
        self.assertIsInstance(base.problem, SE2Problem)
        self.assertTrue(base.graph.is_directed())
        self.assertEqual(base.vertices.shape, (20, 3))
        self.assertEqual(base.problem.curvature, 0.25)
        marker = self.node.path_pub.messages[-1]
        self.assertEqual(marker.action, Marker.ADD)
        points = np.array([[p.x, p.y] for p in marker.points])
        np.testing.assert_allclose(points[0], [3., 2.], atol=1e-8)
        np.testing.assert_allclose(points[-1], [5., 5.], atol=1e-8)
        # Check the analytic arc and straight, not just the endpoint positions.
        arc = points[points[:, 1] < 4.]
        np.testing.assert_allclose(np.linalg.norm(arc - [3., 4.], axis=1), 2., atol=1e-7)
        np.testing.assert_allclose(points[points[:, 1] >= 4., 0], 5., atol=1e-7)
        length = np.linalg.norm(points[1::2] - points[::2], axis=1).sum()
        self.assertAlmostEqual(length, np.pi + 1., places=2)
        self.assertGreater(np.max(np.abs((points[:, 0] - 3.) - (points[:, 1] - 2.))), 0.5)
        marker.serialize(io.BytesIO())
        roadmap = self.node.roadmap_pub.messages[-1]
        self.assertGreater(len(roadmap.points), 2 * base.graph.number_of_edges())
        roadmap.serialize(io.BytesIO())
        self.logwarn.assert_not_called()

    def test_dubins_same_position_different_heading_requires_motion(self):
        self.configure_dubins()
        self.goal.pose.position = copy.deepcopy(self.start.pose.pose.position)
        self.node.goal_callback(self.goal)
        self.assertEqual(self.node.path_pub.messages[-1].action, Marker.ADD)
        self.assertGreater(len(self.node.path_pub.messages[-1].points), 2)
        self.logwarn.assert_not_called()

    def test_dubins_rejects_invalid_orientation(self):
        self.configure_dubins()
        for value in (0., float("nan")):
            with self.subTest(value=value):
                self.goal.pose.orientation.x = value
                self.goal.pose.orientation.y = 0.
                self.goal.pose.orientation.z = 0.
                self.goal.pose.orientation.w = 0.
                self.node.goal_callback(self.goal)
                self.assertEqual(self.node.path_pub.messages[-1].action, Marker.DELETE)
        self.assertEqual(self.logwarn.call_count, 2)
        self.assertIsNone(self.node.base_roadmap)

    def test_dubins_cache_separates_mode_and_curvature_and_reloads(self):
        self.configure_dubins()
        self.node.goal_callback(self.goal)
        key = self.node.roadmap_key
        for attr, value in (("dubins", False), ("curvature", 1.)):
            with patch.object(self.node, attr, value):
                self.assertNotEqual(self.node.cache_key(self.map), key)
        self.assertEqual(len(list(self.cache_dir.glob("se2_*.pkl"))), 1)
        fresh = AStarNode()
        fresh.map_callback(self.map)
        fresh.initial_pose_callback(self.start)
        with patch.object(Roadmap, "sample_vertices", side_effect=AssertionError("must load")):
            fresh.goal_callback(self.goal)
        self.assertTrue(fresh.base_roadmap.loaded_from_cache)
        self.assertTrue(fresh.base_roadmap.graph.is_directed())
        self.assertEqual(fresh.path_pub.messages[-1].action, Marker.ADD)

    def test_dubins_lattice_samples_three_dimensions(self):
        self.configure_dubins()
        self.params["~sampler"] = "lattice"
        self.node = AStarNode()
        self.supply_inputs()
        self.node.goal_callback(self.goal)
        self.assertEqual(self.node.base_roadmap.vertices.shape[1], 3)
        self.assertEqual(self.node.path_pub.messages[-1].action, Marker.ADD)
        self.logwarn.assert_not_called()

    def test_invalid_dubins_curvature(self):
        for value in (0., -1., float("nan"), float("inf")):
            with self.subTest(value=value), patch.dict(
                self.params, {"~dubins": True, "~curvature": value}
            ):
                with self.assertRaises(ValueError):
                    AStarNode()

    def test_path_publishes_before_roadmap_drawing(self):
        self.configure_dubins()
        draw = self.node.visualizer.visualize_roadmap

        def check_order(*args, **kwargs):
            self.assertEqual(self.node.path_pub.messages[-1].action, Marker.ADD)
            return draw(*args, **kwargs)

        with patch.object(self.node.visualizer, "visualize_roadmap", side_effect=check_order):
            self.node.goal_callback(self.goal)

    def test_hidden_roadmap_still_plans_and_reuses_cache(self):
        self.configure_dubins()
        self.node.show_roadmap = False
        with patch.object(self.node.visualizer, "visualize_roadmap") as draw:
            self.node.goal_callback(self.goal)
            base = self.node.base_roadmap
            self.node.goal_callback(self.goal)
            self.assertIs(self.node.base_roadmap, base)
            self.assertEqual(self.node.path_pub.messages[-1].action, Marker.ADD)
            draw.assert_not_called()

    def test_display_settings_do_not_invalidate_collision_checked_cache(self):
        self.configure_dubins()
        key = self.node.cache_key(self.map)
        self.node.show_roadmap = False
        self.node.roadmap_display_resolution = 0.2
        self.assertEqual(self.node.cache_key(self.map), key)

    def test_roadmap_display_has_hard_size_limit(self):
        self.configure_dubins()
        self.node.goal_callback(self.goal)
        base = self.node.base_roadmap
        graph_edges = list(base.graph.edges(data=True))
        with patch.object(base.problem, "steer", wraps=base.problem.steer) as steer:
            self.node.visualizer.visualize_roadmap(
                base, resolution=0.05, max_edges=10, max_points=100
            )
        marker = self.node.roadmap_pub.messages[-1]
        self.assertLessEqual(steer.call_count, 10)
        self.assertGreater(len(marker.points), 0)
        self.assertLessEqual(len(marker.points), 100)
        self.assertEqual(len(marker.points) % 2, 0)
        self.assertEqual(marker.colors, [])
        self.assertEqual(marker.color.r, 1.)
        self.assertEqual(list(base.graph.edges(data=True)), graph_edges)
        self.assertEqual(base.problem.check_resolution, 0.1)
        payload = io.BytesIO()
        marker.serialize(payload)
        self.assertLess(len(payload.getvalue()), 3000)

    def test_invalid_display_settings(self):
        for name, value in (("~show_roadmap", "false"),
                            ("~roadmap_display_resolution", 0.),
                            ("~roadmap_display_resolution", float("nan"))):
            with self.subTest(name=name, value=value), patch.dict(self.params, {name: value}):
                with self.assertRaises(ValueError):
                    AStarNode()

    def test_dense_roadmap_stays_below_bridge_buffer(self):
        self.configure_dubins()
        self.node.goal_callback(self.goal)
        base = self.node.base_roadmap
        # Replicate valid geometry to exercise an arena-sized edge count without
        # rebuilding tens of thousands of collision-checked curves in this test.
        dense = copy.copy(base)
        dense.vertices = np.tile(base.vertices, (250, 1))
        dense.graph = nx.DiGraph()
        for block in range(250):
            offset = block * len(base.vertices)
            dense.graph.add_weighted_edges_from(
                (u + offset, v + offset, weight)
                for u, v, weight in base.graph.edges(data="weight")
            )
        self.assertGreater(dense.graph.number_of_edges(), 35000)
        with patch.object(dense.problem, "steer", wraps=dense.problem.steer) as steer:
            self.node.visualizer.visualize_roadmap(dense)
        self.assertLessEqual(steer.call_count, 1000)
        marker = self.node.roadmap_pub.messages[-1]
        self.assertGreater(len(marker.points), 0)
        self.assertLessEqual(len(marker.points), 20000)
        payload = io.BytesIO()
        marker.serialize(payload)
        self.assertLess(len(payload.getvalue()), 500000)


if __name__ == "__main__":
    unittest.main()
