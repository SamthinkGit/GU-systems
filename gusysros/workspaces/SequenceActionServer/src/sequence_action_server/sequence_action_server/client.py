import traceback
from sequence_action_server.observer import Publisher
from sequence_action_server.observer import Subscriber

import rclpy.action
import rclpy.logging
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import String
from sys_actions.action import Sequence

from gusyscore.constants import REQUEST_TOPIC
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import TaskRegistry
from gusysros.tools.packages import TaskStatus


class SequenceActionClient(Node):

    def __init__(self):

        super().__init__("sequence_action_client")

        self.registry = TaskRegistry()
        self.subscriber = Subscriber(target=self.goal_completed_callback)
        Publisher().add_subscriber(self.subscriber)

        self._action_client = ActionClient(self, Sequence, "sequence")
        self.subscription = self.create_subscription(
            String, REQUEST_TOPIC, self.request_callback, qos_profile_system_default
        )

    def send_goal(self, text):
        goal_msg = Sequence.Goal()
        goal_msg.goal = text

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

    def request_callback(self, msg):
        try:
            sequence = SequencePackage.from_json(msg.data)
            task_id = sequence.task_id
        except Exception:
            trback = traceback.format_exc()
            self.get_logger().warn(
                f"Invalid package received in Sequence Action client. {trback}"
            )
            return

        # Add the sequence to the registry
        self.registry.update(sequence)

        # Look if the task is currently working
        if self.registry.tasks[task_id].status == TaskStatus.READY:
            self.get_logger().info(
                "Task Received and preprocessed, sending to action server"
            )

            # Execute the next task according to STP
            self.registry.tasks[task_id].status = TaskStatus.RUNNING
            sequence = self.registry.get(task_id)
            self.send_goal(sequence.to_json())

    def goal_completed_callback(self, task_id):
        self.get_logger().info("Task Completed")
        next_sequence = self.registry.get(task_id)

        if next_sequence is None:
            self.get_logger().info("Task Empty, Waiting...")
            self.registry.tasks[task_id] = TaskStatus.READY
        else:
            self.get_logger().info("Starting Next Sequence")
            self.send_goal(next_sequence.to_json())


def main(args=None):

    rclpy.init(args=args)
    action_client = SequenceActionClient()
    # future = action_client.send_goal("Miaw")
    # future = action_client.send_goal("Mow")
    # future.result()
    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
