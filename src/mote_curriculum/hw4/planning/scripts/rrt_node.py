#!/usr/bin/env python3
"""Plan between Foxglove poses on the map published by map_server."""

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

from planning.problems import R2Problem
from planning.rrt_visualizer import rrt_visualizer as RRTVisualizer
from planning.search import RRTPlanner
import planning.utils as util


class RRTNode:
    def __init__(self):
        self.map = None
        self.initial_pose = None
        self.explore_pub = rospy.Publisher(
            "/rrt_explore_viz", Marker, queue_size=1, latch=True
        )
        self.path_pub = rospy.Publisher(
            "/rrt_path_viz", Marker, queue_size=1, latch=True
        )
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.start_sub = rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initial_pose_callback
        )
        self.goal_sub = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback
        )

    def map_callback(self, msg):
        self.map = msg

    def initial_pose_callback(self, msg):
        self.initial_pose = msg

    def goal_callback(self, goal):
        map_msg = self.map
        initial_pose = self.initial_pose
        if map_msg is None:
            rospy.logwarn("Waiting for /map before planning.")
            return
        if initial_pose is None:
            rospy.logwarn("Please publish an initial pose before setting a goal.")
            return

        frame = map_msg.header.frame_id or "map"
        if any(msg.header.frame_id != frame for msg in (initial_pose, goal)):
            rospy.logwarn("Publish both poses in the map frame '%s'.", frame)
            return

        visualizer = RRTVisualizer(self.explore_pub, self.path_pub, map_msg)
        visualizer.clear()
        grid = np.asarray(map_msg.data).reshape(
            map_msg.info.height, map_msg.info.width
        )
        # Unknown cells (-1) and occupied cells are both blocked.
        permissible_region = grid == 0
        problem = R2Problem(permissible_region)
        start_position = initial_pose.pose.pose.position
        goal_position = goal.pose.position
        start = util.world_to_map(
            np.array([start_position.x, start_position.y], dtype=float), map_msg.info
        )
        end = util.world_to_map(
            np.array([goal_position.x, goal_position.y], dtype=float), map_msg.info
        )
        if not np.isfinite(np.vstack((start, end))).all():
            rospy.logwarn("Start and goal must have finite coordinates.")
            return
        if not problem.check_state_validity(np.vstack((start, end))).all():
            rospy.logwarn("Start and goal must be inside the map in free space.")
            return

        planner = RRTPlanner(
            problem, permissible_region, bias=0.05, eta=1.0,
            show_tree=False, batch_size=1, shortcut=False,
        )
        plan = planner.Plan(start, end)
        visualizer.visualize_tree(planner.tree)
        if plan is None:
            rospy.logwarn("RRT could not find a path. Publish another goal to retry.")
            return
        visualizer.visualize_plan(plan)


if __name__ == "__main__":
    rospy.init_node("rrt")
    node = RRTNode()
    rospy.spin()
