"""
Execution Feedback Module
==============================

This module defines the `ExecutionStatus` enumeration and the Virtual `Feedback` class for managing
the execution status and feedback of tasks. It should provide mechanisms for publishing and
parsing feedback, as well as responding to execution status changes.

[HELP] A implementation of this virtual class can be seen in ecm.mediator.rosa_interpreter
"""
from enum import Enum
from typing import Any


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
    REQUEST_TO_CONTINUE = 6
    CONTINUE = 7


class Feedback:

    task_id: str
    object: Any
    _exec_status: ExecutionStatus

    def __init__(self) -> None: ...
    def publish(self, object: Any, _exec_status: ExecutionStatus = ExecutionStatus.RUNNING, **kwargs): ...
    def response(self, object: Any, _exec_status: ExecutionStatus): ...

    @classmethod
    def parse(message: Any, *args, **kwargs) -> "Feedback":
        raise SystemError(
            "The feedback function called is virtual. Please ensure that you are "
            "using the appropriate Feedback Implementation instead of the template. "
        )
