from dataclasses import dataclass
from typing import Any
from typing import Callable
from typing import Literal
from typing import Optional
from typing import Sequence
from typing import Type

from ecm.exelent.parser import ParsedAction
from ecm.exelent.parser import ParsedTask
from ecm.exelent.parser import ParsedType
from ecm.exelent.parser import ParsedWith
from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.Interpreter import Interpreter
from ecm.mediator.Interpreter import InterpreterSupports
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry
from execution_layer.pyxcel.core.scheduler import Scheduler
from execution_layer.pyxcel.core.scheduler import Sequential


@dataclass
class PyxcelInterpreterSupports(InterpreterSupports):
    stop: bool = True
    hard_stop: bool = False
    sync_calls: bool = True
    async_calls: bool = True
    wait_for: bool = False
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


class PyxcelInterpreter(Interpreter):

    _logger = get_logger("Pyxcel")
    type_dict: dict[str, Scheduler] = {
        "Sequential": Sequential,
    }
    callback_dict: dict[str, ExecutionStatus] = {
        "RUNNING": ExecutionStatus.RUNNING,
        "STEP": ExecutionStatus.STEP,
        "SUCCESS": ExecutionStatus.SUCCESS,
        "ABORT": ExecutionStatus.ABORT,
        "FINISH": ExecutionStatus.FINISH,
        "SWITCH": ExecutionStatus.SWITCH,
    }

    def __init__(self) -> None:
        self._current_schedulers: Optional[list[Scheduler]] = None
        self._cleaning_schedulers: list[Scheduler] = []

    def run(
        self,
        task: ParsedTask | Sequence[ParsedTask],
        callback: Callable[..., Any] | None | Literal["silent"] = "silent",
        registry: ItemRegistry = ItemRegistry(),
    ) -> None:

        if callback == "silent":
            callback = None

        self._load_schedulers(task, callback, registry)
        for sched in self._current_schedulers:
            sched.run()
        self._current_schedulers = None

    def arun(
        self,
        task: ParsedTask | Sequence[ParsedTask],
        callback: Callable[..., Any] | None | Literal["silent"],
        registry: ItemRegistry = ItemRegistry(),
    ):
        if callback == "silent":
            callback = None

        self._load_schedulers(task, callback, registry)
        seq = Sequential(
            actions=self._current_schedulers,
            feedback_callback=callback,
            registry=registry,
        )
        seq.arun()
        self._cleaning_schedulers.append(seq)

    def kill(self):
        for sched in self._cleaning_schedulers:
            sched.clean()
        self.__init__()

    def _load_schedulers(
        self, task: ParsedTask, callback: Optional[Callable], registry: ItemRegistry
    ):
        # Note: Kwargs for types are not supported yet
        if not isinstance(task, ParsedTask):
            raise ValueError(
                "Structure received is not a valid Task. Maybe the definition does not start with 'def'?"
                f"Received: {task}"
            )
        scheds = []
        for parsed_with in task.sequence:
            scheds.append(
                self._get_sched_from_parsed_with(parsed_with, callback, registry)
            )

        self._current_schedulers = scheds

    def _get_sched_from_parsed_with(
        self,
        parsed_with: ParsedWith,
        callback: Optional[Callable],
        registry: ItemRegistry,
    ) -> list[Scheduler]:
        actions = []
        for value in parsed_with.contains:
            if isinstance(value, ParsedWith):
                actions.append(
                    self._get_sched_from_parsed_with(value, callback, registry)
                )
            elif isinstance(value, ParsedAction):
                actions.append(value)
            else:
                raise ValueError(
                    f"Expecting ParsedWith or ParsedAction and received {value}"
                )

        sched_cls = self._get_sched_class(parsed_with.type)
        instance = sched_cls(
            actions=actions, feedback_callback=callback, registry=registry
        )
        return instance

    def _get_sched_class(self, type: ParsedType) -> Type[Scheduler]:
        # Here is the list of the supported types
        if type.name not in PyxcelInterpreterSupports.types:
            raise KeyError(
                f"Scheduler with type {type.name} is not supported yet with Pyxcel interpreter"
            )

        return self.type_dict[type.name]
