"""
Registry Logger Node
==============================

This module includes a ROS 2 node for publishing feedback messages to
a /task_registry topic. There will be all the logs provided from the
task created and its status
"""
import json
from sequence_action_server.client import SequenceActionClient

from rclpy.node import Node
from std_msgs.msg import String

from execution_layer.rosa.build.nodes import NodeRegistry
from execution_layer.rosa.constants import REGISTRY_TOPIC


class RegistryLogger(Node):
    """
    A ROS 2 node for publishing log messages to a specific topic.
    """
    def __init__(self):
        """
        Initializes the Logger node and sets up the publisher for sending messages.
        """
        super().__init__("registry_logger")
        self.publisher_ = self.create_publisher(String, REGISTRY_TOPIC, 100)
        self.timer = self.create_timer(0.25, self.log)
        self.registry = None

    def log(self):
        """
        Publishes the logmessage to the configured topic.
        """
        if self.registry is None:
            seq_client_node: SequenceActionClient = NodeRegistry.inited_nodes['sequence_client']
            self.registry = seq_client_node.registry

        msg = String()
        log = self.registry.get_log(only_priorities=False)
        msg.data = json.dumps(log, indent=4)
        self.publisher_.publish(msg)
