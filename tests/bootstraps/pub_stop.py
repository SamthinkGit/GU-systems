import rclpy
from icecream import ic  # noqa
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from gusysalb.alb import ALB
from gusyscore.constants import REQUEST_TOPIC
from gusyscore.core import get_logger
from gusyscore.gateway.mocks.debug import empty_function
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry
from gusysros.types.basic import ReservedTypeCode


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
