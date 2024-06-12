import rclpy
from icecream import ic  # noqa
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ecm.registry import ItemRegistry
from execution_layer.rosa.constants import REQUEST_TOPIC
from execution_layer.rosa.gateway import empty_function
from execution_layer.rosa.interfaces.alb import ALB
from execution_layer.rosa.ros2.tools.packages import ActionPackage
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import SequencePriority
from execution_layer.rosa.ros2.types.basic import ReservedTypeCode
from execution_layer.rosa.shared import get_logger


class MyPublisherNode(Node):

    def __init__(self):
        super().__init__("test_stop_publisher")
        self.publisher_ = self.create_publisher(String, REQUEST_TOPIC, 10)
        self.msg = String()
        self._logger = get_logger("test_stop_publisher")

    def send_package(self):

        action = ActionPackage(
            action_id=ItemRegistry.get_id(empty_function)
        )

        priority = SequencePriority.INTERRUPTION
        seq = SequencePackage(
            task_id="my_task",
            type=ReservedTypeCode.SOFT_STOP.value,
            priority=priority,
            actions=[action],
        )

        self.msg.data = seq.to_json()
        self.publisher_.publish(self.msg)
        self._logger.info(f"Stop Published with priority {priority.name}")


def main():

    alb = ALB()
    alb.load_mocks()
    alb.load_types()
    rclpy.init()

    print("Starting node...")
    print("Use ENTER to send stop packages")

    node = MyPublisherNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    while True:
        input()
        node.send_package()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
