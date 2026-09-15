#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

import rospy
import rostest
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool

# Add the parent directory to the sys.path to import the PrimeNumberSubscriber class
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from prime_number_subscriber import PrimeNumberSubscriber

class TestPrimeNumberSubscriber(unittest.TestCase):
    def setUp(self):
        self.received_messages = []
        self.publisher = rospy.Publisher(
            "/hw1/prime_number_output", Bool, queue_size=1
        )
        self.subscriber = PrimeNumberSubscriber(size=5)

    def test_prime_number_subscriber(self):
        while self.publisher.get_num_connections() == 0:
            rospy.sleep(0.05)

        expected_messages = [True, False, True, False, True]
        rate = rospy.Rate(10)

        for value in expected_messages:
            self.publisher.publish(Bool(data=value))
            rate.sleep()

        while not self.subscriber.done:
            rospy.sleep(0.1)

        self.assertEqual(self.subscriber.storage, expected_messages)


if __name__ == "__main__":
    rospy.init_node("test_prime_number_subscriber")
    rostest.rosrun("hw1", "test_prime_number_subscriber", TestPrimeNumberSubscriber)
