import rospy

import tf

import math

from std_msgs.msg import Header
from geometry_msgs.msg import TwistStamped, Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf.transformations


class odometry:
    def __init__(self):
        self.odometry_msg = Odometry()
        self.odometry_msg.header = Header(stamp=rospy.Time.now(), frame_id="odom")
        self.odometry_msg.child_frame_id = "base_link"

        # publish messages of type Odometry on the /odom topic
        self.odometry_publisher = rospy.Publisher("odom", Odometry, queue_size=10)

        """
        QUESTION 4.2 BEGINS
        """
        # create a TransformBroadcaster once so it can be reused in the callback

        """
        QUESTION 4.2 ENDS
        """

        self.last_twist_msg_time = 0

        self.last_twist = Twist()

        # subscribe to TwistStamped messages on the /cmd_vel_ground_truth topic, and register twist_callback as the callback
        self.twist_subscription = rospy.Subscriber(
            "cmd_vel_ground_truth", TwistStamped, self.twist_callback
        )

    def twist_callback(self, msg: TwistStamped):
        if self.last_twist_msg_time == 0:
            self.last_twist_msg_time = msg.header.stamp
            self.last_twist = msg.twist
            return

        dt = (msg.header.stamp - self.last_twist_msg_time).to_sec()

        # we do our calculations based on the last twist, not the one we just received
        linear_velocity = self.last_twist.linear.x
        angular_velocity = self.last_twist.angular.z

        """
        QUESTION 4.1 BEGINS
        """
        # the Odometry message type is a bit more complicated than Twist or JointState
        # read through the specification carefully: https://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html
        # you can click on subtypes (PoseWithCovariance, TwistWithCovariance, etc.) to read their spec

        # you don't need to assign anything to the covariance values (for now)

        # pose orientation is stored as a quaternion
        # working with quaternions can be difficult, read:
        # https://wiki.ros.org/tf2/Tutorials/Quaternions

        # This function just performs a single integration step, so you can assume that the rover is moving at a constant velocity during this time step

        # set the twist in the self.odometry_msg, then ...

        # ... update the pose orientation, then ...
        # HINT: you can use tf.transformations.euler_from_quaternion and 
        #       tf.transformations.quaternion_from_euler to convert between euler angles and quaternions

        # ... update the pose position, then ...
        # HINT: you SHOULDN'T assume the angle is constant during the integration step.
        #       so you should use the current yaw angle plus half of the angular velocity times dt (the heading at the middle of the step) to calculate the change in x and y position

        # ... update the message header, then ...

        # ... publish the message


        """ 
        QUESTION 4.1 ENDS
        """

        """
        QUESTION 4.2 BEGINS
        """
        # you may find this tutorial useful: https://wiki.ros.org/tf/Tutorials/Writing%20a%20tf%20broadcaster%20%28Python%29

        # use the TransformBroadcaster from __init__ to  broadcast a transform from "odom" to "base_link"
        # HINT: you can use tf.TransformBroadcaster.sendTransform to send the transform
        

        """
        QUESTION 4.2 ENDS
        """

        self.last_twist_msg_time = msg.header.stamp
        self.last_twist = msg.twist


def main(args=None):
    # init the node
    rospy.init_node("odometry", anonymous=True)
    _ = odometry()

    print("Running odometry node...")

    # let ROS handle ROS things
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
