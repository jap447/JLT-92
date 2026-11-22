#!/usr/bin/env python3
"""Run roadmap A* between Foxglove poses on a ROS occupancy grid."""

import copy
import hashlib
import json
import os
import threading

import networkx as nx
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from planning.problems import R2Problem, SE2Problem
from planning.roadmap import Roadmap
from planning.rrt_visualizer import rrt_visualizer as MapVisualizer
from planning.samplers import samplers
from planning.search import ASTARPlanner
import planning.utils as util


class AStarNode:
    def __init__(self):
        self.num_vertices = rospy.get_param("~num_vertices", 1000)
        self.dubins = rospy.get_param("~dubins", False)
        self.connection_radius = rospy.get_param("~connection_radius", 30.0)
        self.sampler_name = rospy.get_param("~sampler", "halton")
        self.curvature = rospy.get_param("~curvature", 1.0)
        self.cache_dir = os.path.expanduser(rospy.get_param(
            "~cache_dir", "~/.ros/planning/roadmaps"
        ))
        self.use_disk_cache = rospy.get_param("~use_disk_cache", True)
        self.show_roadmap = rospy.get_param("~show_roadmap", True)
        self.roadmap_display_resolution = rospy.get_param("~roadmap_display_resolution", 0.05)
        self.check_resolution = 0.1
        if not isinstance(self.num_vertices, int) or self.num_vertices <= 0:
            raise ValueError("num_vertices must be a positive integer")
        if not np.isfinite(self.connection_radius) or self.connection_radius <= 0:
            raise ValueError("connection_radius must be finite and positive (map cells)")
        if self.sampler_name not in ("halton", "lattice", "random"):
            raise ValueError("sampler must be halton, lattice, or random")
        if not isinstance(self.dubins, bool):
            raise ValueError("dubins must be a boolean")
        if self.dubins and (not np.isfinite(self.curvature) or self.curvature <= 0):
            raise ValueError("curvature must be finite and positive (inverse meters)")
        if not isinstance(self.show_roadmap, bool):
            raise ValueError("show_roadmap must be a boolean")
        if (not np.isfinite(self.roadmap_display_resolution)
                or self.roadmap_display_resolution <= 0):
            raise ValueError("roadmap_display_resolution must be finite and positive (meters)")

        self.map = None
        self.initial_pose = None
        self.map_key = None
        self.roadmap_key = None
        self.base_roadmap = None
        self.roadmap_displayed = False
        self.visualizer = None
        # Map and goal callbacks can arrive on different ROS connection threads.
        self.lock = threading.RLock()
        self.roadmap_pub = rospy.Publisher(
            "/astar_roadmap_viz", Marker, queue_size=1, latch=True
        )
        self.path_pub = rospy.Publisher(
            "/astar_path_viz", Marker, queue_size=1, latch=True
        )
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.start_sub = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback
        )
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback
        )

    def map_callback(self, msg):
        with self.lock:
            key = self.cache_key(msg)
            if key != self.map_key:
                if self.visualizer is not None:
                    self.visualizer.clear()
                self.base_roadmap = None
                self.roadmap_displayed = False
                self.roadmap_key = None
                self.visualizer = MapVisualizer(
                    self.roadmap_pub, self.path_pub, msg, namespace="astar_viz"
                )
                self.map_key = key
            self.map = msg

    def initial_pose_callback(self, msg):
        with self.lock:
            self.initial_pose = msg

    def cache_key(self, map_msg):
        """Identify map content and planning settings, excluding ROS timestamps."""
        info = map_msg.info
        origin = info.origin
        metadata = {
            # Bump when graph construction/collision semantics change.
            "cache_version": 1,
            "problem": "se2" if self.dubins else "r2",
            "frame": map_msg.header.frame_id or "map",
            "width": info.width, "height": info.height,
            "resolution": info.resolution,
            "origin": [origin.position.x, origin.position.y, origin.position.z,
                       origin.orientation.x, origin.orientation.y,
                       origin.orientation.z, origin.orientation.w],
            "num_vertices": self.num_vertices,
            "connection_radius": self.connection_radius,
            "sampler": self.sampler_name,
            "check_resolution": self.check_resolution,
        }
        if self.dubins:
            metadata["curvature"] = self.curvature
        digest = hashlib.sha256(json.dumps(metadata, sort_keys=True).encode("utf-8"))
        digest.update(np.asarray(map_msg.data, dtype=np.int8).tobytes())
        return digest.hexdigest()

    def get_roadmap(self, map_msg, problem):
        key = self.cache_key(map_msg)
        if self.base_roadmap is not None and self.roadmap_key == key:
            rospy.loginfo("Reusing A* roadmap from memory.")
        else:
            self.visualizer.clear()
            self.roadmap_displayed = False
            cache_path = None
            if self.use_disk_cache:
                try:
                    os.makedirs(self.cache_dir, exist_ok=True)
                    prefix = "se2_" if self.dubins else "r2_"
                    cache_path = os.path.join(self.cache_dir, prefix + key + ".pkl")
                except OSError as error:
                    rospy.logwarn("Disk cache unavailable; using memory only: %s", error)
            rospy.loginfo("Loading or building A* roadmap with %d vertices...", self.num_vertices)
            sampler = samplers[self.sampler_name](problem.extents)
            self.base_roadmap = Roadmap(
                problem, sampler, self.num_vertices, self.connection_radius,
                saveto=cache_path,
            )
            self.roadmap_key = key
            rospy.loginfo(
                "Loaded A* roadmap from disk." if self.base_roadmap.loaded_from_cache
                else "Built A* roadmap."
            )
            rospy.loginfo(
                "Roadmap has %d vertices and %d edges; searching for a path...",
                len(self.base_roadmap.vertices), self.base_roadmap.graph.number_of_edges(),
            )

        # add_node mutates graph/arrays; each query gets an independent working
        # copy, while the cached base never contains clicked start/goal nodes.
        roadmap = copy.copy(self.base_roadmap)
        roadmap.graph = self.base_roadmap.graph.copy()
        roadmap.vertices = self.base_roadmap.vertices.copy()
        roadmap.weighted_edges = self.base_roadmap.weighted_edges.copy()
        roadmap.start = roadmap.goal = None
        roadmap.edges_evaluated = 0
        roadmap.saveto = None
        return roadmap

    def goal_callback(self, goal):
        with self.lock:
            self.plan_to_goal(goal)

    def display_roadmap(self):
        if self.show_roadmap and not self.roadmap_displayed:
            drawn = self.visualizer.visualize_roadmap(
                self.base_roadmap, resolution=self.roadmap_display_resolution
            )
            self.roadmap_displayed = True
            rospy.loginfo(
                "Displayed %d of %d roadmap edges (%d marker points). "
                "Display sampling does not change planning.",
                drawn, self.base_roadmap.graph.number_of_edges(),
                len(self.visualizer.exploration_graph.points),
            )

    def plan_to_goal(self, goal):
        map_msg = self.map
        initial_pose = self.initial_pose
        if map_msg is None:
            rospy.logwarn("Waiting for /map before planning.")
            return
        if initial_pose is None:
            rospy.logwarn("Please publish an initial pose before setting a goal.")
            return

        visualizer = self.visualizer
        visualizer.clear_path()
        frame = map_msg.header.frame_id or "map"
        if any(msg.header.frame_id != frame for msg in (initial_pose, goal)):
            rospy.logwarn("Publish both poses in the map frame '%s'.", frame)
            return

        grid = np.asarray(map_msg.data).reshape(
            map_msg.info.height, map_msg.info.width
        )
        if self.dubins:
            # Planning uses map cells; launch curvature is specified in 1/meters.
            problem = SE2Problem(
                grid == 0, check_resolution=self.check_resolution,
                curvature=self.curvature * map_msg.info.resolution,
            )
        else:
            problem = R2Problem(grid == 0, check_resolution=self.check_resolution)
        try:
            start = self.pose_to_state(initial_pose.pose.pose, map_msg.info)
            end = self.pose_to_state(goal.pose, map_msg.info)
        except ValueError as error:
            rospy.logwarn("Invalid start or goal pose: %s", error)
            return
        endpoints = np.vstack((start, end))
        if not np.isfinite(endpoints).all():
            rospy.logwarn("Start and goal must have finite coordinates.")
            return
        if not problem.check_state_validity(endpoints).all():
            rospy.logwarn("Start and goal must be inside the map in free space.")
            return
        if np.array_equal(start, end):
            rospy.loginfo("Start already equals goal; no motion is needed.")
            return

        roadmap = self.get_roadmap(map_msg, problem)
        try:
            start_id = roadmap.add_node(start, is_start=True)
            goal_id = roadmap.add_node(end, is_start=False)
            vertex_path, _ = ASTARPlanner(roadmap).Plan(start_id, goal_id)
        except (ValueError, nx.NetworkXNoPath, nx.NodeNotFound) as error:
            rospy.logwarn(
                "A* could not find a path: %s. Connection limit is %.2f meters. "
                "Check pose headings and turning clearance in Dubins mode; "
                "a larger connection_radius or more vertices may help a disconnected roadmap.",
                error, self.connection_radius * map_msg.info.resolution,
            )
            self.display_roadmap()
            return

        qpath = (roadmap.compute_qpath(vertex_path) if self.dubins
                 else roadmap.vertices[vertex_path, :])
        visualizer.visualize_plan(qpath[:, :2])
        rospy.loginfo("Published A* path with %d vertices.", len(vertex_path))
        # The green result is available before any optional red graph drawing.
        self.display_roadmap()

    def pose_to_state(self, pose, map_info):
        if self.dubins:
            q = pose.orientation
            quaternion = np.array([q.x, q.y, q.z, q.w])
            if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) == 0:
                raise ValueError("Dubins planning requires a valid orientation quaternion")
            state = np.array(util.pose_to_particle(pose), dtype=float)
        else:
            state = np.array([pose.position.x, pose.position.y], dtype=float)
        state = util.world_to_map(state, map_info).reshape(-1)
        if self.dubins:
            state[2] = (state[2] + np.pi) % (2 * np.pi) - np.pi
        return state


if __name__ == "__main__":
    rospy.init_node("astar")
    node = AStarNode()
    rospy.spin()
