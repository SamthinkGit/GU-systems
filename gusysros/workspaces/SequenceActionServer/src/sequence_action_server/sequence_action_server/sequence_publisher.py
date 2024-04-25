from icecream import ic  # noqa
from rclpy.node import Node
from std_msgs.msg import String

from gusyscore.constants import REQUEST_TOPIC
from gusysros.tools.packages import SequencePackage


class SequencePublisher(Node):

    def __init__(self):
        super().__init__("sequence_publisher")
        self.publisher_ = self.create_publisher(String, REQUEST_TOPIC, 10)
        self.msg = String()

    def send_package(self, sequence: SequencePackage):

        self.msg.data = sequence.to_json()
        self.publisher_.publish(self.msg)
