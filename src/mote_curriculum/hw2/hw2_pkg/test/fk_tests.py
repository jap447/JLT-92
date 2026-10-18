PKG = "hw2_pkg"
NAME = "fk_tests"

import sys, unittest, time

import rospy, rostest

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped

from hw2_pkg.forward_kinematics import forward_kin


class TestFK(unittest.TestCase):
    # this is called when initializing the test class
    @classmethod
    def setUpClass(cls):
        cls.forward_kin = forward_kin()

        # create a publisher to send fake data
        cls.joint_states_publisher = rospy.Publisher(
            "joint_states", JointState, queue_size=10
        )

        rospy.sleep(0.5)  # wait for publishers to init

    # this is called before each test fixture
    def setUp(self):
        self.forward_kin = forward_kin()

        self.cmd_vel_received = None
        self.joint_states = JointState()
        self.joint_states.name = ["left_wheel", "right_wheel"]
        self.joint_states.header = Header(stamp=rospy.Time(0), frame_id="base_link")

    def __init__(self, *args):
        # create a subscriber to check results
        rospy.Subscriber("cmd_vel_ground_truth", TwistStamped, self.callback)

        super(TestFK, self).__init__(*args)

    def wait_for_data(self):
        # 2 second timeout
        timeout_t = time.time() + 2.0

        # Wait for the callback to get hit
        while (
            self.cmd_vel_received == None
            and not rospy.is_shutdown()
            and time.time() < timeout_t
        ):
            rospy.sleep(0.1)

        # Check for timeout
        self.assertNotEqual(
            self.cmd_vel_received, None, msg="JointState not received before timeout"
        )

        data = self.cmd_vel_received
        self.cmd_vel_received = None

        return data

    def test_no_motion(self):
        self.joint_states.velocity = [0.0, 0.0]

        self.joint_states_publisher.publish(self.joint_states)

        cmd_vel = self.wait_for_data()

        self.assertAlmostEquals(
            cmd_vel.twist.linear.x, 0.0, msg="Twist linear velocity is incorrect"
        )
        self.assertAlmostEquals(
            cmd_vel.twist.angular.z, 0.0, msg="Twist angular velocity is incorrect"
        )

    def test_straight(self):
        self.joint_states.velocity = [20.0, 20.0]

        self.joint_states_publisher.publish(self.joint_states)

        cmd_vel = self.wait_for_data()

        self.assertAlmostEquals(
            cmd_vel.twist.linear.x, 0.66, msg="Twist linear velocity is incorrect"
        )
        self.assertAlmostEquals(
            cmd_vel.twist.angular.z, 0.0, msg="Twist angular velocity is incorrect"
        )

    def test_turn_on_spot(self):
        self.joint_states.velocity = [-10.0, 10.0]

        self.joint_states_publisher.publish(self.joint_states)

        cmd_vel = self.wait_for_data()

        self.assertAlmostEquals(
            cmd_vel.twist.linear.x, 0.0, msg="Twist linear velocity is incorrect"
        )
        self.assertAlmostEquals(
            cmd_vel.twist.angular.z,
            4.4,
            msg="Twist angular velocity is incorrect",
        )

    def test_gentle_turn(self):
        self.joint_states.velocity = [-20.0, -10.0]

        self.joint_states_publisher.publish(self.joint_states)

        cmd_vel = self.wait_for_data()

        self.assertAlmostEquals(
            cmd_vel.twist.linear.x, -0.495, msg="Twist linear velocity is incorrect"
        )

        self.assertAlmostEquals(
            cmd_vel.twist.angular.z,
            2.2,
            msg="Twist angular velocity is incorrect",
        )

    def callback(self, msg: TwistStamped):
        self.cmd_vel_received = msg


if __name__ == "__main__":
    rospy.init_node(NAME, anonymous=True)

    rostest.rosrun(PKG, NAME, TestFK, sys.argv)
