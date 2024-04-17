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

    _dead_tasks = []
    _thread_registry = ThreadRegistry()

    def __init__(self):
        super().__init__("sequence_action_server")
        self._action_server = ActionServer(
            self, Sequence, "sequence", self.execute_callback
        )

    def execute_callback(self, goal_handle: rclpy.action.server.ServerGoalHandle):

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
        for task_id in cls._dead_tasks:
            cls._thread_registry.wait(task_id)
            cls._dead_tasks.remove(task_id)

    @classmethod
    def close_task(cls, task_id: str):
        print("Closing Task...")
        cls._dead_tasks.append(task_id)
        Publisher.notify_subscribers(task_id)


def execute_sequence_type(seq_type: SequenceType):
    seq_type.run()


def main(args=None):
    rclpy.init(args=args)
    sequence_action_server = SequenceActionServer()
    rclpy.spin(sequence_action_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
