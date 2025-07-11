import sys

import pytest  # noqa
import rclpy.node  # noqa
from rclpy.node import Node

from execution_layer.rosa.constants import FEEDBACK_TOPIC
from execution_layer.rosa.constants import REQUEST_TOPIC
from execution_layer.rosa.interfaces.alb import ALB


class TestALB:

    def setup(self):
        self.alb = ALB()

    def test_load_mocks(self):
        self.alb.load_mocks()
        assert "execution_layer.rosa.gateway.mocks.gateway" in sys.modules

    def test_load_types(self):
        self.alb.load_types()
        assert "execution_layer.rosa.ros2.types.basic" in sys.modules

    def test_load_nodes(self):
        self.alb.load_nodes()
        node = Node("test_node")

        publishers_info = node.get_publishers_info_by_topic(FEEDBACK_TOPIC)
        assert len(publishers_info) > 0

        subscribers_info = node.get_subscriptions_info_by_topic(REQUEST_TOPIC)
        assert len(subscribers_info) > 0
        rclpy.shutdown()
