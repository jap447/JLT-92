from __future__ import annotations

import rospy

import numpy as np

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
    Vector3,
)
from visualization_msgs.msg import Marker

import copy

import planning.utils as util


class rrt_visualizer:
    def __init__(self, explore_publisher, path_publisher, map, namespace="rrt_viz"):
        # Setup graph
        self.exploration_graph = Marker()
        self.exploration_graph.header.frame_id = map.header.frame_id or "map"
        self.exploration_graph.header.stamp = rospy.Time.now()
        self.exploration_graph.ns = namespace
        self.exploration_graph.type = Marker.LINE_LIST
        self.exploration_graph.action = Marker.ADD
        self.exploration_graph.pose = Pose()
        self.exploration_graph.pose.position = Point(0.0, 0.0, 0.0)
        self.exploration_graph.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        self.exploration_graph.scale = Vector3(0.02, 0.00, 0.0)
        self.exploration_graph.lifetime.set(0, 0)
        self.path_graph = copy.deepcopy(self.exploration_graph)
        self.exploration_graph.id = 0
        self.path_graph.id = 1

        # self.path_graph.header.frame_id = "base_link"
        # self.path_graph.frame_locked = True

        # publishers
        self.rrt_explore_viz_publisher = explore_publisher
        self.rrt_path_viz_publisher = path_publisher

        self.map = map

    def visualize_plan(self, plan):
        self.visualize_graph(self.path_graph, plan, [0.0, 1.0, 0.0, 1.0], z=0.1)
        self.rrt_path_viz_publisher.publish(self.path_graph)

    def visualize_tree(self, tree):
        for child, parent in tree.edges.items():
            edge = np.vstack((tree.vertices[parent], tree.vertices[child]))
            self.visualize_graph(
                self.exploration_graph, edge, [1.0, 0.0, 0.0, 1.0]
            )
        self.rrt_explore_viz_publisher.publish(self.exploration_graph)

    def visualize_roadmap(self, roadmap, resolution=0.05, max_edges=1000, max_points=20000):
        """Draw a bounded sample of edges; resolution is in display meters.

        These limits affect only the marker, never the graph or collision checks.
        """
        graph = self.exploration_graph
        graph.points = []
        graph.colors = []
        graph.header.stamp = rospy.Time.now()
        edges = list(roadmap.graph.edges())
        if len(edges) > max_edges:
            indices = np.linspace(0, len(edges) - 1, max_edges, dtype=int)
            edges = [edges[i] for i in indices]
        drawn = 0
        for source, target in edges:
            start, end = roadmap.vertices[[source, target], :]
            # Bound interpolation work even for a very long individual edge.
            length = roadmap.graph[source][target]["weight"]
            spacing = max(resolution / self.map.info.resolution,
                          length / max(1, max_points // 2 - 6))
            edge, _ = roadmap.problem.steer(
                start, end, resolution=spacing, interpolate_line=False
            )
            if len(graph.points) + 2 * (len(edge) - 1) > max_points:
                break
            self.visualize_graph(
                graph, edge[:, :2], [1.0, 0.0, 0.0, 1.0]
            )
            drawn += 1
        self.rrt_explore_viz_publisher.publish(graph)
        return drawn

    def visualize_edge(self, start, end):
        self.visualize_graph(
            self.exploration_graph, np.array([start, end]), [1.0, 0.0, 0.0, 1.0]
        )
        self.rrt_explore_viz_publisher.publish(self.exploration_graph)

    def visualize_graph(self, graph, plan, color, z=0.0):
        # Transform each polyline in one array operation instead of twice per
        # segment. Every segment in these markers shares the same color.
        world = util.map_to_world(np.array(plan[:, :2], dtype=float, copy=True), self.map.info)
        graph.color = ColorRGBA(*color)
        for s, e in zip(world[:-1], world[1:]):
            graph.points.append(Point(s[0], s[1], z))
            graph.points.append(Point(e[0], e[1], z))

    def clear_path(self):
        """Remove only the path, preserving the latched base roadmap."""
        self.path_graph.points = []
        self.path_graph.colors = []
        self.path_graph.header.stamp = rospy.Time.now()
        self.path_graph.action = Marker.DELETE
        self.rrt_path_viz_publisher.publish(self.path_graph)
        self.path_graph.action = Marker.ADD

    def clear(self):
        self.exploration_graph.points = []
        self.exploration_graph.colors = []
        self.path_graph.points = []
        self.path_graph.colors = []
        self.exploration_graph.action = Marker.DELETEALL
        self.path_graph.action = Marker.DELETEALL
        self.rrt_path_viz_publisher.publish(self.path_graph)
        self.rrt_explore_viz_publisher.publish(self.exploration_graph)
        self.exploration_graph.action = Marker.ADD
        self.path_graph.action = Marker.ADD

