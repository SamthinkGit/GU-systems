import sys

import pytest # noqa
import rclpy.node  # noqa
from rclpy.node import Node

from gusysalb.alb import ALB
from gusyscore.constants import FEEDBACK_TOPIC
from gusyscore.constants import REQUEST_TOPIC


class TestALB:

    def setup(self):
        self.alb = ALB()

    def test_load_mocks(self):
        self.alb.load_mocks()
        assert "gusyscore.gateway.mocks.gateway" in sys.modules

    def test_load_types(self):
        self.alb.load_types()
        assert "gusysros.types.basic" in sys.modules

    def test_load_nodes(self):
        self.alb.load_nodes()
        node = Node("test_node")

        publishers_info = node.get_publishers_info_by_topic(FEEDBACK_TOPIC)
        assert len(publishers_info) > 0

        subscribers_info = node.get_subscriptions_info_by_topic(REQUEST_TOPIC)
        assert len(subscribers_info) > 0

        rclpy.shutdown()
