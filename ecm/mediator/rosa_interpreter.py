from dataclasses import dataclass
from typing import Callable
from typing import Optional

from ecm.exelent.parser import linerize_task
from ecm.exelent.parser import ParsedTask
from ecm.exelent.parser import ParsedType
from ecm.mediator.Interpreter import Interpreter
from ecm.mediator.Interpreter import InterpreterSupports
from ecm.registry import ItemRegistry
from execution_layer.rosa.interfaces.rosa import ROSA
from execution_layer.rosa.ros2.tools.feedback import ExecutionStatus
from execution_layer.rosa.ros2.tools.packages import ActionPackage
from execution_layer.rosa.ros2.tools.packages import SequencePackage
from execution_layer.rosa.ros2.tools.packages import SequencePriority
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
    types: tuple[str] = "Sequential"


class RosaInterpreter(Interpreter):

    type_dict: dict[str, SequenceType] = {"Sequential": SimpleSequence}

    def __init__(self) -> None:
        self.rosa: ROSA = ROSA()

    def run(self, task: ParsedTask, callback: Optional[Callable] = None) -> None:

        if callback == "silent" or callback == "mute":
            callback = self.rosa.muted_callback
        self.rosa.new_task(task_id=task.name, feedback_callback=callback)
        for pkg in self._generate_packages_from_parsed_task(task):
            self.rosa.execute(pkg)
        self.rosa.wait_for(task.name, ExecutionStatus.FINISH)

    def _generate_packages_from_parsed_task(
        self,
        task: ParsedTask,
    ) -> list[SequencePackage]:
        # This function takes the following premises:
        # - Multi-Task_id in one task is not supported
        # - TODO: SequencePriority is Normal by default (soon we will add support to more)
        # - ROSA only supports linearized tasks (cannot deal with nested types)
        # - kwargs in types not supported yet

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
