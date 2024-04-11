import traceback

import rclpy.action
import rclpy.logging
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from sys_actions.action import Sequence

from gusyscore.constants import REQUEST_TOPIC
from gusysros.tests.send_sequence import test_function # noqa
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import TaskRegistry
from gusysros.tools.packages import TaskStatus


class SequenceActionClient(Node):

    def __init__(self):

        super().__init__('sequence_action_client')

        self._action_client = ActionClient(self, Sequence, 'sequence')
        self.subscription = self.create_subscription(
            String,
            REQUEST_TOPIC,
            self.request_callback,
            qos_profile_system_default
        )
        self.registry = TaskRegistry()

    def send_goal(self, text):
        goal_msg = Sequence.Goal()
        goal_msg.goal = text
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

    def feedback_callback(self, feedback_msg):
        feedback: Sequence.Feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.feedback}')

    def request_callback(self, msg):

        try:
            sequence = SequencePackage.from_json(msg.data)
        except Exception:
            trback = traceback.format_exc()
            self.get_logger().warn(f"Invalid package received in Sequence Action client. {trback}")
            return

        self.registry.update(sequence)

        id = sequence.task_id
        if self.registry.tasks[id].status == TaskStatus.READY:
            self.get_logger().info("Task Received and preprocessed, sending to action server")
            self.registry.tasks[id].status = TaskStatus.RUNNING
            self.send_goal(msg.data)


def main(args=None):

    rclpy.init(args=args)
    action_client = SequenceActionClient()
    # future = action_client.send_goal("Miaw")
    # future = action_client.send_goal("Mow")
    # future.result()
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
