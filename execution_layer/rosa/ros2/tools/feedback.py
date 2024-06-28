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
from sequence_action_server.feedback import FeedbackPublisher
from sequence_action_server.feedback_response import ResponseListener
from sequence_action_server.feedback_response import ResponsePublisher
from typing import Any

from ecm.mediator.feedback import ExecutionStatus
from ecm.shared import get_logger
from ecm.tools.registry import ItemEncoder
from ecm.tools.registry import ThreadRegistry
from execution_layer.rosa.interfaces.nodes import NodeRegistry


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
        self.responder: ResponsePublisher = NodeRegistry.inited_nodes[
            "response_publisher"
        ]
        self.response_listener: ResponseListener = NodeRegistry.inited_nodes[
            "response_listener"
        ]
        self.task_id = None
        self.object = None
        self._exec_status = None

    def publish(self, object: Any, _status: ExecutionStatus = ExecutionStatus.RUNNING):
        """
        Publishes the provided object and execution status as feedback.

        :param object: The object related to the task feedback, any object can be passed to higher levels.
        """
        message = {
            "task_id": ThreadRegistry.get_task_id(),
            "object": ItemEncoder.autoencode(object),
            "_exec_status": ItemEncoder.autoencode(_status),
        }
        json_pkg = json.dumps(message, indent=4)
        self.publisher.publish_feedback(json_pkg)

    def wait_for_response(self, response_code) -> "Feedback":
        """
        Waits for a response from the feedback listener until the task ID matches.
        :return: The feedback received from the response listener.
        """
        self.task_id = ThreadRegistry.get_task_id()
        pkg = self.response_listener.wait_for_message(response_code)
        feedback = Feedback.from_json(pkg)
        return feedback

    def approve(self, object: Any = "Request Approved"):
        """Sends an approval message by detecting automatically the response code."""
        response = Feedback()
        response.task_id = self.task_id
        response_code = self.object[1]
        response.response(
            object=object,
            _exec_status=ExecutionStatus.CONTINUE,
            response_code=response_code,
        )

    def response(self, object: Any, _exec_status: ExecutionStatus, response_code: str):
        """
        Publishes a response with the provided object and current task ID.

        :param object: The object to be included in the response.
        :returns: The code to receive the answer by using wait_for_response function.
        """
        assert (
            self.task_id is not None
        ), "task_id not settled, ensure to set feedback.task_id before publishing/waiting"
        message = {
            "task_id": self.task_id,
            "object": ItemEncoder.autoencode(object),
            "_exec_status": ItemEncoder.autoencode(_exec_status),
        }
        json_pkg = response_code + json.dumps(message, indent=4)
        self.responder.publish_response(json_pkg)

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
