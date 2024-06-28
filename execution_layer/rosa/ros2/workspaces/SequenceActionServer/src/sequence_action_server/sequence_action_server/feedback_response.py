"""
Feedback Node
==============================

This module includes a ROS 2 node for publishing feedback messages to
a /feedback topic. It provides an interface for sending runtime
feedback from various components of a system through ROS messaging.

Do not confuse with ros2.tools.feedback, which enwraps this node
into a simplified interface
"""
import threading
import time
from typing import Any

from rclpy.node import Node
from std_msgs.msg import String

from execution_layer.rosa.constants import RESPONSE_TOPIC


class ResponsePublisher(Node):
    """
    A ROS 2 node for publishing feedback messages to a specific topic.
    """

    def __init__(self):
        """
        Initializes the FeedbackPublisher node and sets up the publisher for sending messages.
        """
        super().__init__("response_publisher")
        self.publisher_ = self.create_publisher(String, RESPONSE_TOPIC, 100)

    def publish_response(self, message: str):
        """
        Publishes a feedback message to the configured topic.
        :param message: The feedback message to publish.
        """
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)


class ResponseListener(Node):

    def __init__(self):
        super().__init__("response_listener")
        self.subscription = self.create_subscription(
            String, RESPONSE_TOPIC, self.listener_callback, 10
        )
        self.lock = threading.Lock()
        self.messages: dict[str, Any] = {}

    def listener_callback(self, msg):
        msg = msg.data
        with self.lock:
            response_code = msg[0:4]
            message = msg[4:]
            self.messages[response_code] = message

    def wait_for_message(self, response_code) -> str:
        with self.lock:
            if response_code in self.messages:
                return self.messages[response_code]

        while response_code not in self.messages:
            # TODO: Find a better waiting for this
            time.sleep(0.2)

        with self.lock:
            return self.messages[response_code]
