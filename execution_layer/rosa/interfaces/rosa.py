"""
ROSA (ROS Agent)
---------------------------------------------------------------
This module contains the ROSA class which implements a simplified interface
for managing and controlling tasks in ROS layer
Handles task execution, feedback processing, and node managing.
"""
import threading
from collections import defaultdict
from typing import Callable
from typing import DefaultDict
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

import rclpy

from ecm.shared import get_logger
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.gateway.mocks.debug import empty_function
from execution_layer.rosa.interfaces.alb import ALB
from execution_layer.rosa.interfaces.nodes import NodeRegistry
from execution_layer.rosa.ros2.tools.feedback import ExecutionStatus
from execution_layer.rosa.ros2.tools.feedback import Feedback
from execution_layer.rosa.ros2.tools.packages import ActionPackage
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import SequencePriority
from execution_layer.rosa.ros2.types.basic import ReservedTypeCode


class ROSA:
    """
    A singleton class designed to manage robot tasks and handle feedback.

    Attributes:
        _sequence_publisher: The node responsible for publishing sequence packages.
        _task_callback: Stores callbacks for each task.
        _logger: Logger object for the ROSA class.
        _instance: Holds the singleton instance of ROSA.
        _built: Indicates whether ROSA has been built.
        _conditions: Stores the conditions for notifying the wait_for() threads
        _lock: A threading lock to control concurrent access.
        _closed_tasks: Contains recently closed tasks
    """

    _sequence_publisher = None
    _task_callback: DefaultDict[str, Callable] = None
    _logger = get_logger("ROSA")
    _instance = None
    _built = False
    _conditions: Dict[str, List[Tuple[ExecutionStatus, threading.Condition]]] = {}
    _lock = threading.Lock()
    _closed_tasks = []

    def __new__(cls) -> None:
        """
        Ensures that exists only one instance (singleton pattern).
        """
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        cls._build()
        return cls._instance

    @classmethod
    def _build(cls) -> None:
        """
        Builds the necessary components for ROSA, setting up the environment
        and initializing necessary nodes.
        """
        if not cls._built:
            cls._built = True
            alb = ALB()
            alb.build_all(feedback_listener=ROSA._callback_manager)
            cls._sequence_publisher = NodeRegistry.inited_nodes.get("sequence_publisher", None)
            cls._task_callback = defaultdict(ROSA.empty_callback)
            cls._logger.debug("Building completed")

            assert cls._sequence_publisher is not None, "Sequence Publisher failed to be inited in ROSA"

    @staticmethod
    def _callback_manager(msg: str) -> None:
        """
        Each time a message is publised in the /requests topic, this function will
        be called from the ALB (publisher) to manage the callback to the apropiate
        callback from higher level users.
        :param msg: The message containing feedback.
        """

        feedback = Feedback.from_pkg(msg)
        if feedback.task_id == 'null' or feedback.task_id == "" or feedback.task_id is None:
            return

        # Notify blocked threads if necessary
        threads_to_notify = ROSA._conditions.get(feedback.task_id, None)

        if threads_to_notify is not None:
            for condition, lock in threads_to_notify:
                if condition.value == feedback._exec_status.value:
                    with lock:
                        lock.notify()

        # Keep a list of closed tasks, so the user doesn't try to wait for a closed task
        if feedback._exec_status == ExecutionStatus.FINISH:
            ROSA._closed_tasks.append([feedback.task_id])

        if (
            feedback._exec_status != ExecutionStatus
            and feedback.task_id in ROSA._closed_tasks
        ):
            ROSA._closed_tasks.remove(feedback.task_id)

        # Execute the callback
        callback = ROSA._task_callback.get(feedback.task_id, None)
        if callback is not None:
            callback(feedback)

    @staticmethod
    def empty_callback(feedback: Feedback) -> None:
        """
        Default callback for feedback messages. If the user has not specified an action
        to do with the feedback from a task, this function will be executed for each pkg.
        :param feedback: The feedback object to handle.
        """
        ROSA._logger.warn(
            f"A feedback package has not been processed <[{feedback.task_id}]: {feedback.object}>"
        )

    @staticmethod
    def muted_callback(feedback: Feedback) -> None:
        """
        Callback for muting feedback messages when the feedback is no longer relevant.
        :param feedback: The feedback object to ignore.
        """
        pass

    def new_task(
        self, task_id: str = "default", feedback_callback: Optional[Callable] = None
    ) -> None:
        """
        Prepares ROSA to redirect the feedback from a task to a given feedback_callback.
        If not specified is redirected to empty_callback
        :param task_id: The unique identifier for the task.
        :param feedback_callback: The callback function to invoke for task feedback.
        """
        if feedback_callback is not None:
            self._task_callback[task_id] = feedback_callback
        else:
            self._task_callback[task_id] = ROSA.empty_callback

        if task_id not in self._conditions.keys():
            self._conditions[task_id] = []

    def execute(self, sequence: SequencePackage):
        """
        Executes a given sequence by sending it through the sequence publisher.
        :param sequence: The sequence package to execute.
        """
        self._sequence_publisher.send_package(sequence)
        ROSA._logger.debug(f"{sequence.task_id} execution started")

    def soft_stop(self, task_id: str = "default") -> None:
        """
        Initiates a soft stop for a task using a predefined action.
        A soft-stop makes the task stop as soon as the SequenceType checks for
        activity. This means, it will stop as soon as posible, but the time of
        stopping will depend on the SequenceType implementation to avoid failures.
        :param task_id: The task identifier for which the soft stop is initiated.
        """
        action = ActionPackage(action_id=ItemRegistry.get_id(empty_function))

        seq = SequencePackage(
            task_id=task_id,
            type=ReservedTypeCode.SOFT_STOP.value,
            priority=SequencePriority.INTERRUPTION,
            actions=[action],
        )
        self.execute(seq)

    # -----------------------    CAUTION    ------------------------
    # Should only be used as the last resource to stop a task
    # Could end up in corruption or failures to the main system
    def _hard_stop(self, task_id: str) -> None:
        """
        Forces a hard stop on a task. Use as a last resort to stop a task,
        as it might lead to system instability or data corruption. The
        interruption is immediate and is not handled by the SequenceType
        :param task_id: The task identifier to stop forcefully.
        """
        action = ActionPackage(action_id=ItemRegistry.get_id(empty_function))

        seq = SequencePackage(
            task_id=task_id,
            type=ReservedTypeCode.HARD_STOP.value,
            priority=SequencePriority.INTERRUPTION,
            actions=[action],
        )
        self.execute(seq)

    # --------------------------------------------------------------

    def wait_for(
        self,
        task_id: str,
        code: ExecutionStatus,
    ):
        """
        Blocks the current thread until the ExecutionStatus such is received from
        the specified task.
        :note: If the task has already finished it will return without waiting.
        :param task_id: The task identifier.
        :param code: The code to wait for.
        """
        condition = threading.Condition()
        entry = [code, condition]

        if task_id not in self._conditions.keys():
            self._conditions[task_id] = []

        with self._lock:
            self._conditions[task_id].append(entry)

        with condition:
            self._logger.debug(f"Waiting for {code.name} from {task_id}")

            if task_id not in self._closed_tasks:
                condition.wait()

            self._logger.debug(f"Waiting for {code.name} completed in {task_id}")

        with self._lock:
            self._conditions[task_id].append(entry)

    def kill(self):
        rclpy.shutdown()
