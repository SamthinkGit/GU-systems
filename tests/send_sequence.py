import rclpy
from icecream import ic # noqa
from rclpy.node import Node
from std_msgs.msg import String

from gusysalb.alb import ALB
from gusyscore.constants import REQUEST_TOPIC
from gusyscore.gateway.mocks.debug import test_function
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry
from gusysros.types.basic import SimpleSequence


class MyPublisherNode(Node):

    def __init__(self):
        super().__init__('test_sequence_publisher')
        self.publisher_ = self.create_publisher(String, REQUEST_TOPIC, 10)
        self.timer = self.create_timer(3, self.timer_callback)
        self.msg = String()
        print("Timer built, wating 3s for starting")

    def timer_callback(self):

        actions = [
            ActionPackage(
                action_id=ItemRegistry.get_id(test_function),
                num=num
            ) for num in range(5)
        ]

        seq = SequencePackage(
            task_id='my_task',
            type=SimpleSequence.get_type(),
            priority=SequencePriority.NORMAL,
            actions=actions
        )

        self.msg.data = seq.to_json()
        self.publisher_.publish(self.msg)
        self.get_logger().info('Task Published')


def main():
    alb = ALB()
    alb.build_all()
    print("Starting node...")

    node = MyPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
