"""
Task Feedback
==============================

This module enhances task management capabilities by defining feedback mechanisms and
execution statuses. It leverages enums for clear and maintainable code representation
of execution statuses and integrates feedback publishing functionality with the task
management system.

Key Components:
- ExecutionStatus: Enum that defines various stages of task execution.
- Feedback: Class responsible for publishing feedback regarding task execution status.
"""
import json
from enum import Enum
from sequence_action_server.feedback import FeedbackPublisher
from typing import Any

from gusysalb.nodes import NodeRegistry
from gusyscore.core import get_logger
from gusysros.tools.registry import ItemEncoder
from gusysros.tools.registry import ThreadRegistry


class ExecutionStatus(Enum):
    """
    Enumeration of possible execution statuses of a task.
    Should only be used by SequenceTypes status communication.
    """

    RUNNING = 0
    STEP = 1
    SUCCESS = 2
    ABORT = 3
    FINISH = 4
    SWITCH = 5


class Feedback:
    """
    Manages the publishing of feedback about the status of tasks.
    :param None: This class does not require parameters upon instantiation, it auto-initializes its attributes.
    """

    _logger = get_logger("Feedback")

    def __init__(self) -> None:
        self.publisher: FeedbackPublisher = NodeRegistry.inited_nodes[
            "feedback_publisher"
        ]
        self.task_id = None
        self.object = None
        self._exec_status = None

    def publish(self, object: Any, _status: ExecutionStatus = ExecutionStatus.RUNNING):
        """
        Publishes the provided object and execution status as feedback.

        :param object: The object related to the task feedback, any object can be passed to higher levels.
        :param _status: IMPORTANT: Should only be used by SequenceStatus. Sets the current execution status of the task.
        """
        message = {
            "task_id": ThreadRegistry.get_task_id(),
            "object": ItemEncoder.autoencode(object),
            "_exec_status": ItemEncoder.autoencode(_status),
        }
        json_pkg = json.dumps(message, indent=4)
        self.publisher.publish_feedback(json_pkg)

    @classmethod
    def from_pkg(cls, pkg: str):
        return Feedback.from_json(pkg)

    @classmethod
    def from_json(cls, package: str):
        dict_pkg = json.loads(package)

        required_keys = ["task_id", "object", "_exec_status"]
        if not all(key in dict_pkg for key in required_keys):
            cls._logger.error(
                f"Dict received in Feedback() must contain {required_keys}"
            )
            return None

        feedback = Feedback()
        feedback.task_id = dict_pkg["task_id"]
        feedback.object = ItemEncoder.autodecode(dict_pkg["object"])
        feedback._exec_status = ItemEncoder.autodecode(dict_pkg["_exec_status"])
        return feedback
