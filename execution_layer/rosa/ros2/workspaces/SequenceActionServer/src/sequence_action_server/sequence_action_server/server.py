"""
IODA Server / Request Controller
==================================

This module integrates a ROS 2 action server that processes sequence tasks,
it must execute it without permission to cancel unless it receives a package
to do so. It launches a thread for each task execution that will be controlled
using the TaskManager until completion.
"""
import ctypes
import signal
import traceback
from sequence_action_server.observer import Publisher

import rclpy.action
from rclpy.action import ActionServer
from rclpy.node import Node
from sys_actions.action import Sequence

from ecm.registry import ThreadRegistry
from ecm.shared import get_logger
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.types.basic import ReservedTypeCode
from execution_layer.rosa.ros2.types.basic import SequenceType


class SequenceActionServer(Node):
    """
    A ROS 2 Node acting as an action server that handles sequence tasks,
    delegating their execution to separate threads managed by a ThreadRegistry.
    """

    _libc = ctypes.CDLL("libc.so.6")
    _pthread_kill = _libc.pthread_kill
    _pthread_kill.argtypes = [ctypes.c_ulong, ctypes.c_int]

    _dead_tasks = []
    _thread_registry = ThreadRegistry()
    _logger = get_logger("ActionServer")

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
            SequenceActionServer._logger.warn(
                f"Invalid package received in Sequence Action client. {trback}"
            )
            return

        try:
            reserved_code = ReservedTypeCode(sequence_pkg.type)
        except ValueError:
            reserved_code = None

        task_id = sequence_pkg.task_id

        # Check if it is a TaskManagement package
        if reserved_code == ReservedTypeCode.SOFT_STOP:
            thread = self._thread_registry.get_thread(sequence_pkg.task_id)
            SequenceActionServer._logger.debug(
                f"SOFT-STOP Detected for task {sequence_pkg.task_id}"
            )

            if thread is None:
                SequenceActionServer._logger.warn(
                    "Trying to cancel a task that is not running"
                )
                Publisher.notify_subscribers(task_id)
                return self._success(goal_handle, sequence_pkg)

            seq_type: SequenceType = thread._seq_type
            seq_type.soft_stop()
            return self._success(goal_handle, sequence_pkg)

        # --------------------------------------------------------------------------
        # Caution, this code should only be used for security reasons, could end
        # in a segmentation fault
        if reserved_code == ReservedTypeCode.HARD_STOP:
            thread = self._thread_registry.get_thread(sequence_pkg.task_id)
            SequenceActionServer._logger.warn(
                f"HARD-STOP Detected for task {sequence_pkg.task_id}"
            )
            SequenceActionServer._logger.warn(
                "HARD-STOP Should only be used for security reasons, could end up in corrupted files/values"
            )

            if thread is None:
                SequenceActionServer._logger.warn(
                    "Trying to kill a task that is not running"
                )
                Publisher.notify_subscribers(task_id)
                return self._success(goal_handle, sequence_pkg)

            self._pthread_kill(thread.ident, signal.SIGTERM)
            return self._success(goal_handle, sequence_pkg)
        # --------------------------------------------------------------------------

        # Define the behavior of the pkg
        try:
            seq_type = SequenceType.from_pkg(sequence_pkg)
        except ValueError:
            SequenceActionServer._logger.error(
                f"Sequence Type for {task_id} cannot be obtained "
                "(It is not registered or the type value is invalid). Skip"
            )
            self.check_for_clean()
            return self._success(goal_handle, sequence_pkg)

        seq_type.at_exit(SequenceActionServer.close_task)

        # Throw a thread to complete the task, it will return to close_task()
        # This means, the server is directly up to continue answering calls
        self._thread_registry.watch(
            task_id=task_id,
            target=execute_sequence_type,
            seq_type=seq_type,
        )

        self.check_for_clean()
        return self._success(goal_handle, sequence_pkg)

    @staticmethod
    def _success(goal_handle, sequence_pkg):
        """Simple refactorization for returning success to the client"""
        result = Sequence.Result()
        result.result = sequence_pkg.task_id
        goal_handle.succeed()
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
