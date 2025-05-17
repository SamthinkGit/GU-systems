"""
Pyxcel Interpreter Module
==========================

This module defines a Pyxcel interpreter that integrates with various schedulers
for executing parsed Exelent language tasks. It supports both synchronous and
asynchronous calls while providing detailed logging and feedback through
callbacks.

The Pyxcel interpreter can handle multiple types of scheduled actions
and provides the ability to clean up resources post-execution. It also
includes features for gracefully shutting down ongoing tasks and managing
feedback during execution.
"""
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
from execution_layer.pyxcel.core.scheduler import PYXCEL_SUPPORTED_SCHEDULERS
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
        """
        Executes a given task or a sequence of tasks using the specified
        callback for feedback. If the callback is set to "silent", no feedback
        will be provided during execution.

        :param task: The task or sequence of tasks to execute.
        :param callback: The callback function to handle execution feedback.
        :param registry: An optional item registry to manage task components.
        :return: None, but initiates the execution of the task(s).
        """

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
        """
        Asynchronously executes a given task or a sequence of tasks, allowing
        for feedback during execution through the provided callback. Similar
        to the run method, but designed for asynchronous operation.

        :param task: The task or sequence of tasks to execute asynchronously.
        :param callback: The callback function to handle execution feedback.
        :param registry: An optional item registry to manage task components.
        :return: None, but initiates the asynchronous execution of the task(s).
        """
        if callback == "silent":
            callback = None

        self._load_schedulers(task, callback, registry)
        seq = Sequential(
            actions=self._current_schedulers,
            feedback_callback=callback,
            registry=registry,
        )
        seq.arun()
        self._current_schedulers.append(seq)
        self._cleaning_schedulers.append(seq)

    def stop(self):
        for sched in self._current_schedulers:
            sched.stop()

    def kill(self):
        """
        Cleans up all currently running schedulers and resets the interpreter
        state. This method ensures that any remaining resources are released
        and that the interpreter is ready for a new task execution.

        This method does not take any parameters and does not return a value.
        """
        for sched in self._cleaning_schedulers:
            sched.clean()
        self.__init__()

    def _load_schedulers(
        self, task: ParsedTask, callback: Optional[Callable], registry: ItemRegistry
    ):
        """
        Loads the schedulers based on the provided task, extracting the
        necessary components and preparing them for execution. It validates
        the task structure to ensure it is appropriate for processing.

        :param task: The parsed task to load schedulers from.
        :param callback: An optional callback for feedback during execution.
        :param registry: An item registry to manage task components.
        :return: None, but updates the internal state with loaded schedulers.
        """
        # Note: Kwargs for types are not supported yet
        # Also nested types with returning objects are not supported

        if not isinstance(task, ParsedTask):
            raise ValueError(
                "Structure received is not a valid Task. Maybe the definition does not start with 'def'?"
                f"Received: {task}"
            )
        scheds = []
        self._logger.debug(f"Building Task `{task.name}`")
        for parsed_with in task.sequence:
            self._logger.debug(f"Building Type `{parsed_with.type.name}`")
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
        """
        Retrieves a scheduler instance based on the parsed structure of actions
        and configurations. It recursively processes nested structures to build
        the complete set of actions for execution.

        :param parsed_with: The parsed structure containing actions to process.
        :param callback: An optional callback for feedback during execution.
        :param registry: An item registry to manage task components.
        :return: A list of Scheduler instances ready for execution.
        """
        actions = []
        for value in parsed_with.contains:
            if isinstance(value, ParsedWith):
                actions.append(
                    self._get_sched_from_parsed_with(value, callback, registry)
                )
            elif isinstance(value, ParsedAction):
                self._logger.debug(f"Building Action `{value}`")
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
        """
        Determines the appropriate scheduler class based on the provided type.
        It checks against the supported scheduler list and raises an error if
        the type is not recognized.

        :param type: The parsed type indicating the desired scheduler class.
        :return: The scheduler class corresponding to the specified type.
        :raises KeyError: If the type is not supported by the Pyxcel interpreter.
        """
        if type.name not in PYXCEL_SUPPORTED_SCHEDULERS:
            raise KeyError(
                f"Scheduler with type {type.name} is not supported yet with Pyxcel interpreter"
            )

        return PYXCEL_SUPPORTED_SCHEDULERS[type.name]
