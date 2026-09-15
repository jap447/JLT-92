#!/usr/bin/env python3

import threading
import unittest

import roslaunch
import rospy
import rostest
from std_msgs.msg import Bool


class TestPrimeNumberLaunch(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.launch_file = roslaunch.rlutil.resolve_launch_arguments(
            ["hw1", "prime_number.launch"]
        )[0]

    def check_input(self, input_number, expected_result):
        topic = "/hw1/prime_number_launch_test_{}".format(input_number)
        received = []
        message_event = threading.Event()

        def callback(message):
            received.append(message.data)
            message_event.set()

        subscriber = rospy.Subscriber(topic, Bool, callback, queue_size=1)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        launch = roslaunch.parent.ROSLaunchParent(
            uuid,
            [
                (
                    self.launch_file,
                    [
                        "input_number:={}".format(input_number),
                        "prime_number_output_topic:={}".format(topic),
                        "launch_subscriber:=false",
                    ],
                )
            ],
        )

        try:
            launch.start()
            self.assertTrue(
                message_event.wait(5.0),
                "Launch file did not publish a result for input {}".format(
                    input_number
                ),
            )
            self.assertEqual(received[0], expected_result)
        finally:
            launch.shutdown()
            subscriber.unregister()
            rospy.sleep(0.2)

    def test_launch_file_passes_input_number(self):
        self.check_input(11, True)
        self.check_input(12, False)


if __name__ == "__main__":
    rospy.init_node("test_prime_number_launch")
    rostest.rosrun("hw1", "test_prime_number_launch", TestPrimeNumberLaunch)
