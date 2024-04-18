"""
Feedback Node
==============================

This module includes a ROS 2 node for publishing feedback messages to
a /feedback topic. It provides an interface for sending runtime
feedback from various components of a system through ROS messaging.

Do not confuse with gusysros.tools.feedback, which enwraps this node
into a simplified interface
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gusyscore.constants import FEEDBACK_TOPIC


class FeedbackPublisher(Node):
    """
    A ROS 2 node for publishing feedback messages to a specific topic.
    """
    def __init__(self):
        """
        Initializes the FeedbackPublisher node and sets up the publisher for sending messages.
        """
        super().__init__("feedback_publisher")
        self.publisher_ = self.create_publisher(String, FEEDBACK_TOPIC, 100)

    def publish_feedback(self, message: str):
        """
        Publishes a feedback message to the configured topic.
        :param message: The feedback message to publish.
        """
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    feedback_publisher = FeedbackPublisher()
    try:
        while True:
            message = input("Enter a feedback message to publish: ")
            feedback_publisher.publish_feedback(message)
    except KeyboardInterrupt:
        pass
    feedback_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
