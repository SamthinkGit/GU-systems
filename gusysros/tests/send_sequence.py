import random

import rclpy
from icecream import ic # noqa
from rclpy.node import Node
from std_msgs.msg import String

from gusyscore.constants import REQUEST_TOPIC
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import ActionType
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry


class MyPublisherNode(Node):

    def __init__(self):
        super().__init__('test_sequence_publisher')
        self.publisher_ = self.create_publisher(String, REQUEST_TOPIC, 10)
        self.timer = self.create_timer(3, self.timer_callback)
        self.msg = String()

    def timer_callback(self):

        my_func_id = ItemRegistry.get_id(test_function)

        action = ActionPackage(
            type=ActionType.SIMPLE_SEQUENCE,
            action_id=my_func_id,
            num=random.randint(0, 100))

        seq = SequencePackage(
            task_id='my_task',
            priority=SequencePriority.NORMAL,
            actions=[action]
        )

        self.msg.data = seq.to_json()
        self.publisher_.publish(self.msg)
        self.get_logger().info('Task Published')


@ItemRegistry.register_function
def test_function(num: int):
    print("Succesfully executed function with argument: ", num)


def main(args=None):
    rclpy.init(args=args)
    node = MyPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
