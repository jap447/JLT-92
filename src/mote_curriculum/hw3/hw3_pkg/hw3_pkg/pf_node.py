from __future__ import annotations

import rospy

import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Time
from nav_msgs.msg import OccupancyGrid

from hw3_pkg.map_helpers import LikelihoodField
from hw3_pkg.particle_filter import particle_filter

import tf.transformations
from tf.transformations import quaternion_from_euler

import threading
import time


class pf_node:
    def __init__(self):
        self.mutex = threading.Lock()
        self.scan = None
        self.map = None
        self.map_model = None

        # initial pose
        self.initial_pose = None
        self.reset_distribution = False
        self.prev_odom = None
        self.last_scan_stamp = None

        # tunable parameters
        self.num_particles = rospy.get_param("~num_particles", 300)
        self.lidar_std = rospy.get_param("~lidar_std", 0.20)
        self.lidar_subsample_count = rospy.get_param("~lidar_subsample_count", 60)
        self.init_xy_std = 0.75
        self.init_yaw_std = 1.0

        self.pf = self.get_initial_distribution()

        # subscribe to messages of type LaserScan on the /scan topic
        self.laser_scan_subscription = rospy.Subscriber(
            "scan", LaserScan, self.scan_callback
        )

        # subscribe to messages of type PoseWithCovariance on the /initialpose topic
        self.initial_pose_topic = rospy.resolve_name("initialpose")
        self.initial_pose_subscription = self.subscribe_initial_pose()
        self.initial_pose_retry_at = time.monotonic() + 5.0

        # publish messages of type on the /scan topic
        self.pose_array_publisher = rospy.Publisher(
            "particlecloud", PoseArray, queue_size=10
        )

        # publish ground_truth
        self.gt_publisher = rospy.Publisher(
            "scan_ground_truth", LaserScan, queue_size=10
        )

        # initialize the tf tree
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster.sendTransform(
            (0.0, 0.0, 0.0),
            quaternion_from_euler(0.0, 0.0, 0.0),
            rospy.Time.now(),
            "odom",
            "map",
        )

        # map info
        self.map = None
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        # wait for publisher to init
        rospy.sleep(0.5)

        thread = threading.Thread(target=self.thread_main)
        thread.start()

        self.pose_watchdog_stop = threading.Event()
        rospy.on_shutdown(self.pose_watchdog_stop.set)
        self.pose_watchdog_thread = threading.Thread(
            target=self.initial_pose_watchdog, daemon=True
        )
        self.pose_watchdog_thread.start()

    def subscribe_initial_pose(self):
        return rospy.Subscriber(
            self.initial_pose_topic, PoseWithCovarianceStamped,
            self.initial_pose_callback, queue_size=1,
        )

    def initial_pose_watchdog(self):
        # Wall time also works when a bag's ROS clock is paused or jumps.
        # Master queries stay off the localization processing thread.
        while not self.pose_watchdog_stop.wait(1.0):
            if rospy.is_shutdown():
                return
            self.check_initial_pose_connection()

    def check_initial_pose_connection(self):
        now = time.monotonic()
        subscription = self.initial_pose_subscription
        if subscription is not None and subscription.get_num_connections() > 0:
            self.initial_pose_retry_at = now + 5.0
            return
        if now < self.initial_pose_retry_at or rospy.is_shutdown():
            return
        self.initial_pose_retry_at = now + 5.0
        try:
            code, message, state = rospy.get_master().getSystemState()
            if code != 1:
                raise RuntimeError(message)
            publishers = dict(state[0]).get(self.initial_pose_topic, [])
            if not publishers or rospy.is_shutdown():
                return
            # A connection may have completed while querying the master.
            if subscription is not None and subscription.get_num_connections() > 0:
                return
            if subscription is not None:
                subscription.unregister()
                self.initial_pose_subscription = None
            self.initial_pose_subscription = self.subscribe_initial_pose()
            rospy.logwarn(
                "Reconnecting %s: publisher exists but pose subscription was disconnected; "
                "resend any missed pose estimate", self.initial_pose_topic,
            )
        except Exception as exc:
            # A master outage or failed subscription must not stop future retries.
            rospy.logwarn("Initial-pose connection check failed: %s", exc)
        finally:
            self.initial_pose_retry_at = time.monotonic() + 5.0

    def thread_main(self):
        while not rospy.is_shutdown():
            with self.mutex:
                scan, model = self.scan, self.map_model
                reset = self.reset_distribution
                if reset:
                    self.pf = self.get_initial_distribution()
                    self.reset_distribution = False
            if reset:
                self.prev_odom = None
                self.last_scan_stamp = None
                self.publish_cloud()
            if model is None or scan is None:
                rospy.sleep(0.05)
                continue
            if scan.header.stamp == self.last_scan_stamp:
                rospy.sleep(0.005)
                continue
            if scan.header.frame_id.lstrip("/") != "lidar":
                rospy.logwarn_throttle(5.0, "Particle filter expects scans in lidar frame")
                rospy.sleep(0.05)
                continue
            try:
                curr_odom = self.get_odom_pose(scan.header.stamp)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
                rospy.logwarn_throttle(5.0, "Waiting for scan-time odometry: %s", exc)
                rospy.sleep(0.01)
                continue

            self.last_scan_stamp = scan.header.stamp
            if self.prev_odom is not None:
                self.pf.predict(self.prev_odom, curr_odom)
            self.prev_odom = curr_odom
            updated = self.calculate_weights(scan, model)
            mean, _ = self.pf.update()
            if updated and 1.0 / np.sum(self.pf.weights ** 2) < 0.5 * self.pf.n_particles:
                self.pf.resample()
            self.update_transforms(mean, curr_odom)
            self.publish_gt_scan(mean, scan, model)
            self.publish_cloud()

    def get_odom_pose(self, stamp):
        trans, rot = self.tf_listener.lookupTransform("odom", "lidar", stamp)
        return np.array([trans[0], trans[1], tf.transformations.euler_from_quaternion(rot)[2]])

    def calculate_weights(self, scan, model):
        ranges = np.asarray(scan.ranges, dtype=float)
        if not len(ranges):
            return False
        indices = np.linspace(0, len(ranges) - 1,
                              min(len(ranges), self.lidar_subsample_count), dtype=int)
        measured = ranges[indices]
        angles = scan.angle_min + indices * scan.angle_increment
        ground_truth = model.scans(self.pf.particles, angles, scan.range_max)
        likelihoods = np.empty(self.pf.n_particles)
        for i, expected in enumerate(ground_truth):
            # Measurement validity checks belong to the student's sensor_model.
            try:
                likelihood = self.pf.sensor_model(measured, expected, self.lidar_std)
                if likelihood is None:
                    rospy.logwarn_throttle(
                        5.0, "sensor_model returned None; implement it to enable sensor updates",
                    )
                    return False
                if np.ndim(likelihood) != 0:
                    raise ValueError("expected a scalar likelihood")
                likelihoods[i] = likelihood
            except Exception as exc:
                rospy.logwarn_throttle(5.0, "sensor_model failed: %s", exc)
                return False
        if np.any(~np.isfinite(likelihoods)) or np.any(likelihoods < 0.0):
            rospy.logwarn_throttle(
                5.0, "sensor_model must return finite, nonnegative likelihoods; retaining prior",
            )
            return False

        # Retain the map constraint: particle origins must be in known free space.
        row, col, inside = model.indices(self.pf.particles[:, 0], self.pf.particles[:, 1])
        free = np.zeros(self.pf.n_particles, dtype=bool)
        free[inside] = model.cells[row[inside], col[inside]] == 0
        supported = free & (likelihoods > 0.0) & (self.pf.weights > 0.0)
        if not np.any(supported):
            rospy.logwarn_throttle(
                5.0, "Sensor likelihoods retaining prior. "
                "If the particle filter is not working, check your sensor_model or set a new pose estimation",
            )
            return False

        # Scale in log space before normalization to avoid further under/overflow.
        # This cannot recover a product that already underflowed in sensor_model.
        log_weights = np.full(self.pf.n_particles, -np.inf)
        log_weights[supported] = (np.log(self.pf.weights[supported])
                                  + np.log(likelihoods[supported]))
        weights = np.exp(log_weights - np.max(log_weights))
        self.pf.weights[:] = weights / weights.sum()
        return True

    def publish_cloud(self):
        # Turn particles into a ROS PoseArray message that Foxglove can display
        particle_poses = PoseArray()
        for particle in self.pf.particles:
            pose = Pose()
            pose.position = Point(particle[0], particle[1], 0.05)
            pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, particle[2]))
            particle_poses.poses.append(pose)
        particle_poses.header.stamp = rospy.Time.now()
        particle_poses.header.frame_id = "map"

        # Publish the particles
        self.pose_array_publisher.publish(particle_poses)

    def update_transforms(self, mean, curr_odom):
        # Calculate and publish map -> odom tranformation
        # In accordance with REP-105 https://www.ros.org/reps/rep-0105.html

        mean_to_map_transform = tf.transformations.inverse_matrix(
            tf.transformations.concatenate_matrices(
                tf.transformations.translation_matrix([mean[0], mean[1], 0.0]),
                tf.transformations.quaternion_matrix(
                    quaternion_from_euler(0.0, 0.0, mean[2])
                ),
            )
        )

        trans = (curr_odom[0], curr_odom[1], 0.0)
        rot = quaternion_from_euler(0.0, 0.0, curr_odom[2])
        odom_to_mean_tranform = tf.transformations.concatenate_matrices(
            tf.transformations.translation_matrix(trans),
            tf.transformations.quaternion_matrix(rot),
        )

        inverse_tranform = tf.transformations.inverse_matrix(
            odom_to_mean_tranform @ mean_to_map_transform
        )

        self.tf_broadcaster.sendTransform(
            tf.transformations.translation_from_matrix(inverse_tranform),
            tf.transformations.quaternion_from_matrix(inverse_tranform),
            rospy.Time.now(),
            "odom",
            "map",
        )

    def publish_gt_scan(self, mean, scan, model):
        fake_scan = LaserScan()
        fake_scan.header = scan.header
        fake_scan.angle_min = scan.angle_min
        fake_scan.angle_increment = scan.angle_increment
        fake_scan.angle_max = scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment
        fake_scan.range_min = scan.range_min
        fake_scan.range_max = scan.range_max
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        fake_scan.ranges = model.scan(mean, angles, scan.range_max).tolist()
        self.gt_publisher.publish(fake_scan)

    def get_initial_distribution(self):
        rng = np.random.default_rng()
        if self.initial_pose is None:
            particles = rng.uniform(
                [-2.0, -2.0, 0.0], [2.0, 2.0, 3.14 * 2.0], size=(self.num_particles, 3)
            )
        else:
            p = self.initial_pose.pose.pose.position
            q = self.initial_pose.pose.pose.orientation
            yaw = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

            particles = np.empty((self.num_particles, 3))
            particles[:, 0] = rng.normal(p.x, self.init_xy_std, self.num_particles)
            particles[:, 1] = rng.normal(p.y, self.init_xy_std, self.num_particles)
            particles[:, 2] = rng.normal(yaw, self.init_yaw_std, self.num_particles) % (2 * np.pi)
            self.reset_distribution = False
        return particle_filter(
            particles, np.full(self.num_particles, 1 / self.num_particles)
        )

    def map_callback(self, msg: OccupancyGrid):
        model = LikelihoodField(msg)
        with self.mutex:
            self.map = msg
            self.map_model = model

    def scan_callback(self, msg: LaserScan):
        self.mutex.acquire(blocking=True)
        self.scan = msg
        self.mutex.release()

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        if msg.header.frame_id.lstrip("/") != "map":
            rospy.logwarn("Initial pose must be in map frame")
            return
        with self.mutex:
            self.initial_pose = msg
            self.reset_distribution = True


def main(args=None):
    # init the node
    rospy.init_node("particle_filter", anonymous=True)

    _ = pf_node()

    # let ROS handle ROS things
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass