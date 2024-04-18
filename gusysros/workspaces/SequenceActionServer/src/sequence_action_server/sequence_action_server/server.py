"""
IODA Server / Request Controller
==================================

This module integrates a ROS 2 action server that processes sequence tasks,
it must execute it without permission to cancel unless it receives a package
to do so. It launches a thread for each task execution that will be controlled
using the TaskManager until completion.
"""
import traceback
from sequence_action_server.observer import Publisher

import rclpy.action
from rclpy.action import ActionServer
from rclpy.node import Node
from sys_actions.action import Sequence

from gusysros.tools.packages import SequencePackage
from gusysros.tools.registry import ThreadRegistry
from gusysros.types.basic import SequenceType


class SequenceActionServer(Node):
    """
    A ROS 2 Node acting as an action server that handles sequence tasks,
    delegating their execution to separate threads managed by a ThreadRegistry.
    """

    _dead_tasks = []
    _thread_registry = ThreadRegistry()

    def __init__(self):
        """
        Initializes the SequenceActionServer node and sets up the action server.
        """
        super().__init__("sequence_action_server")
        self._action_server = ActionServer(
            self, Sequence, "sequence", self.execute_callback
        )

    def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):
        """
        Executes when a new package is received by the action server, processes the
        sequence package, and dispatches it for execution.

        :param goal_handle: The handle to the goal received by the action server.
        """

        try:
            sequence_pkg = SequencePackage.from_json(goal_handle.request.goal)
        except Exception:
            trback = traceback.format_exc()
            self.get_logger().warn(
                f"Invalid package received in Sequence Action client. {trback}"
            )
            return

        # Define the behavior of the pkg
        seq_type = SequenceType.from_pkg(sequence_pkg)
        seq_type.at_exit(SequenceActionServer.close_task)

        # Throw a thread to complete the task, it will return to close_task()
        self._thread_registry.watch(
            task_id=sequence_pkg.task_id, target=execute_sequence_type, seq_type=seq_type
        )

        self.check_for_clean()

        goal_handle.succeed()
        result = Sequence.Result()
        result.result = sequence_pkg.task_id
        return result

    @classmethod
    def check_for_clean(cls):
        """
        Cleans up completed or dead tasks by waiting for their threads to complete and
        removing them from the list of tasks.
        """
        for task_id in cls._dead_tasks:
            cls._thread_registry.wait(task_id)
            cls._dead_tasks.remove(task_id)

    @classmethod
    def close_task(cls, task_id: str):
        """
        Callback to close a task, marking it as complete and notifying client.
        :param task_id: The identifier of the task to close.
        """
        print("Closing Task...")
        cls._dead_tasks.append(task_id)
        Publisher.notify_subscribers(task_id)


def execute_sequence_type(seq_type: SequenceType):
    """
    [Wrapper] Executes a sequence type by running its defined behavior.
    :param seq_type: The sequence type instance to execute.
    """
    seq_type.run()


def main(args=None):
    rclpy.init(args=args)
    sequence_action_server = SequenceActionServer()
    rclpy.spin(sequence_action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
