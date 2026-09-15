import rospy
from std_msgs.msg import Bool


class PrimeNumberSubscriber:
    """Receive and store Boolean results from the prime-number publisher."""

    def __init__(self, size=None):
        self.size = size
        self.done = False
        self.storage = []

        # Get the topic to subscribe to. This should be the same topic
        # your prime_number publisher publishes to.
        prime_topic = rospy.get_param("~output_topic", "/hw1/prime_number_output")

        # Create a subscriber. Function signature:
        # rospy.Subscriber(topic_name, msg_type, callback_function)
        # BEGIN QUESTION 4.4
        "*** REPLACE THIS LINE ***"
        # END QUESTION 4.4

    def prime_output_callback(self, message):
        """Callback function that runs every time a new message is
        published on the topic we're subscribed to. `message` is the
        Bool message that was published.
        """

        '''
        Store the received message in self.storage.
        '''
        # BEGIN QUESTION 4.4
        "*** REPLACE THIS LINE ***"
        # END QUESTION 4.4
        
        if self.size is not None and len(self.storage) >= self.size:
            self.done = True
            self.subscriber.unregister()
