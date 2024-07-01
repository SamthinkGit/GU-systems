"""
Rosa Interpreter Module
=================================

This module defines the `RosaInterpreter` class for executing Exelent-tasks using the ROSA framework.
It extends the `Interpreter` class and integrates with the ROSA system to manage task execution.
The module also includes `RosaInterpreterSupports` to specify the capabilities of the Rosa interpreter.
"""
from dataclasses import dataclass
from typing import Any
from typing import Callable
from typing import Optional

from ecm.exelent.parser import linearize_multiple_defs
from ecm.exelent.parser import linerize_task
from ecm.exelent.parser import ParsedTask
from ecm.exelent.parser import ParsedType
from ecm.mediator.feedback import Feedback as FeedbackTemplate
from ecm.mediator.Interpreter import Interpreter
from ecm.mediator.Interpreter import InterpreterSupports
from ecm.tools.registry import ItemRegistry
from execution_layer.rosa.interfaces.rosa import ROSA
from execution_layer.rosa.ros2.tools.feedback import ExecutionStatus
from execution_layer.rosa.ros2.tools.feedback import Feedback as OriginalRosaFeedback
from execution_layer.rosa.ros2.tools.packages import ActionPackage
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import SequencePriority
from execution_layer.rosa.ros2.types.basic import ControlledSequence
from execution_layer.rosa.ros2.types.basic import SequenceType
from execution_layer.rosa.ros2.types.basic import SimpleSequence


@dataclass
class RosaInterpreterSupports(InterpreterSupports):
    stop: bool = True
    hard_stop: bool = True
    sync_calls: bool = True
    async_calls: bool = True
    wait_for: bool = True
    callbacks: bool = True
    feedback: bool = True
    callback_keywords: tuple[str] = (
        "RUNNING",
        "STEP",
        "SUCCESS",
        "ABORT",
        "FINISH",
        "SWITCH",
    )
    types: tuple[str] = ("Sequential", "ControlledSequence")


class RosaFeedbackWrapper(FeedbackTemplate):

    task_id: str
    object: Any
    _exec_status: ExecutionStatus

    def __init__(self) -> None:
        pass

    def publish(
        self,
        object: Any,
        _exec_status: ExecutionStatus = ExecutionStatus.RUNNING,
        **kwargs,  # noqa
    ):
        self._rosa_feedback.publish(object, _exec_status)

    def response(self, object: Any, _exec_status: ExecutionStatus):
        # Note, we asumme the case were feedback is comming from rosa so we would
        # have:
        # Feedback(task_id=task_id, object=[result, response_code])

        response = OriginalRosaFeedback()
        response.task_id = self.task_id
        response_code = self.object[1]
        response.response(
            object=object,
            _exec_status=_exec_status,
            response_code=response_code,
        )

    @classmethod
    def parse(cls, message: OriginalRosaFeedback) -> "RosaFeedbackWrapper":

        assert isinstance(
            message, OriginalRosaFeedback
        ), "Trying to parse a message that is not from ROSA with RosaFeedback"

        feedback = RosaFeedbackWrapper()
        feedback.task_id = message.task_id
        feedback.object = message.object
        feedback._exec_status = message._exec_status
        feedback._rosa_feedback = message
        return feedback


class RosaInterpreter(Interpreter):

    feedback_message_class = RosaFeedbackWrapper

    type_dict: dict[str, SequenceType] = {
        "Sequential": SimpleSequence,
        "ControlledSequence": ControlledSequence,
    }
    callback_dict: dict[str, ExecutionStatus] = {
        "RUNNING": ExecutionStatus.RUNNING,
        "STEP": ExecutionStatus.STEP,
        "SUCCESS": ExecutionStatus.SUCCESS,
        "ABORT": ExecutionStatus.ABORT,
        "FINISH": ExecutionStatus.FINISH,
        "SWITCH": ExecutionStatus.SWITCH,
    }
    _running = False

    def __init__(self) -> None:
        self.rosa: ROSA = ROSA()
        RosaInterpreter._running = True

    def run(self, task: ParsedTask, callback: Optional[Callable] = None) -> None:

        if callback == "silent" or callback == "mute":
            callback = self.rosa.muted_callback

        self.rosa.new_task(task_id=task.name, feedback_callback=callback)
        packages = self._generate_packages_from_parsed_task(task)
        for pkg in packages:
            self.rosa.execute(pkg)
            self.rosa.wait_for(task.name, ExecutionStatus.FINISH)

    def arun(
        self, task: ParsedTask, feedback_callback: Optional[Callable] = None
    ) -> None:
        self.rosa.new_task(task_id=task.name, feedback_callback=feedback_callback)
        for pkg in self._generate_packages_from_parsed_task(task):
            self.rosa.execute(pkg)

    def stop(self, task: str) -> Any:
        self.rosa.soft_stop(task_id=task)

    def hard_stop(self, task: str) -> Any:
        self.rosa._hard_stop(task_id=task)

    def wait_for(self, task: str, call: str) -> None:
        self.rosa.wait_for(task_id=task, code=self.callback_dict[call])

    def kill(self):
        if RosaInterpreter._running:
            RosaInterpreter._running = False
            self.rosa.kill()

    def _generate_packages_from_parsed_task(
        self,
        task: ParsedTask | list[ParsedTask],
    ) -> list[SequencePackage]:
        # This function takes the following premises:
        # - Multi-Task_id in one task is not supported
        # - TODO: SequencePriority is Normal by default (soon we will add support to more)
        # - ROSA only supports linearized tasks (cannot deal with not-sequential nested types)
        # - kwargs in types not supported yet

        if not isinstance(task, ParsedTask):
            raise ValueError(
                "Structure received is not a valid Task. Maybe the definition does not start with 'def'?"
                f"Received: {task}"
            )

        if isinstance(task, list):
            task = linearize_multiple_defs(task)
        task = linerize_task(task)
        sequences: list[SequencePackage] = []
        task_id = task.name

        for seq in task.sequence:

            pkg_type = self._get_rosa_type(seq.type)
            priority = SequencePriority.NORMAL
            actions: list[ActionPackage] = []

            for action in seq.contains:
                func = ItemRegistry.get_from_name(action.name)
                if func is None:
                    raise NameError(
                        f"Function {action.name} cannot be found in ItemRegistry"
                    )

                id = ItemRegistry.get_id(func)
                actions.append(ActionPackage(id, *action.args, **action.kwargs))

            sequences.append(
                SequencePackage(
                    task_id=task_id, type=pkg_type, priority=priority, actions=actions
                )
            )

        return sequences

    def _get_rosa_type(self, type: ParsedType) -> int:
        # Here is the list of the supported types
        if type.name not in RosaInterpreterSupports.types:
            raise KeyError(f"{type} is not supported yet with ROSA interpreter")

        return self.type_dict[type.name].get_type()
