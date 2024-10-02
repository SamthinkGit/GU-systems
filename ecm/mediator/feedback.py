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
    RESULT = 8


class Feedback:

    task_id: str
    object: Any
    _exec_status: ExecutionStatus

    def __init__(self) -> None: ...

    def publish(
        self,
        object: Any,
        _exec_status: ExecutionStatus = ExecutionStatus.RUNNING,
        **kwargs
    ):
        """
        Used for publishing globally the feedback from the execution layer to other upper layers.
        The classification of the task_id/author is responsability of the interpreter/execution_layer
        implementation.
        """
        ...

    def response(self, object: Any, _exec_status: ExecutionStatus):
        """
        Used for sending feedback from the upper layers to the execution layer. It should automatically
        match the appropiate task/action.
        """

    @classmethod
    def parse(message: Any, *args, **kwargs) -> "Feedback":
        """
        [OPTIONAL]
        This function will be called always before using any Feedback function. Use this for
        parsing any object that comes from the callback into your own Feedback implementation.
        [Note]: This option has been implemented on  ROSA, but not on PyXcel
        Example:

        class MyFeedback(Feedback):
            ...
            @classmethod
            def parse(object_from_interpreter):
                return MyFeedback(object_from_interpreter.name, ...)

        def my_interpreter_callback(message):
            # All agents ensure that they will parse your message
            feedback: MyFeedback = MyFeedback.parse(message)
        """
        raise SystemError(
            "The feedback function called is virtual. Please ensure that you are "
            "using the appropriate Feedback Implementation instead of the template. "
        )
