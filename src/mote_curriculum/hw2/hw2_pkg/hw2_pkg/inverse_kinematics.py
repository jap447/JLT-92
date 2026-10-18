import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist

# this is b in your equations
WHEEL_SEPARATION = 0.15  # meters

# you'll need this to convert angular velocity into linear velocity
WHEEL_DIAMETER = 0.066  # meters


class inv_kin:
    def __init__(self):
        # subscribe to Twist messages on the /diff_drive_controller/cmd_vel topic, and register twist_callback as the callback
        self.twist_subscription = rospy.Subscriber(
            "/diff_drive_controller/cmd_vel", Twist, self.twist_callback
        )

        # initialize our joint states
        # what is the JointState message type?
        # read: https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/JointState.html
        self.joint_states = JointState()
        self.joint_states.header = Header(stamp=rospy.Time.now(), frame_id="base_link")
        self.joint_states.name = ["left_wheel", "right_wheel"]
        self.joint_states.velocity = [0.0, 0.0]  # rad / s

        self.joint_states.position = []  # you'll leave this one empty
        self.joint_states.effort = []  # this one too

        # publish messages of type JointState on the /joint_states_predicted topic
        self.joint_state_publisher = rospy.Publisher(
            "joint_states_predicted", JointState, queue_size=10
        )

    def twist_callback(self, msg: Twist):
        # why linear.x and angular.z?
        # read: https://www.ros.org/reps/rep-0103.html#axis-orientation
        # and: https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        """
        QUESTION 2.1 BEGINS
        """
        # calculate v_left and v_right (in rad/s, not m/s!), then ...

        # ... update self.joint_states.velocity in [left, right] order, then ...

        # ... update the header (see header in __init__), then ...

        # ... publish the message using self.joint_state_publisher
        """ 
        QUESTION 2.1 ENDS
        """


def main(args=None):
    # init the node
    rospy.init_node("inv_kin", anonymous=True)
    _ = inv_kin()

    print("Running inverse kinematics node...")

    # let ROS handle ROS things
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
