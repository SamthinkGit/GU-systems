"""
FastAgentProtocol
==============================
This module provides the `FastAgentProtocol` class, a simplified and
synchronous protocol for managing agent tasks. Unlike its predecessor
(Agent Protocol), this implementation does not rely on REST APIs, making
it easier to integrate into synchronous workflows.

To use this protocol, implement an iterator that defines the steps for
your agent. Once the iterator is ready, you can connect it to the main
application and start processing tasks.
"""
from dataclasses import dataclass
from operator import attrgetter
from typing import Callable
from typing import ClassVar
from typing import Generator
from typing import Generic
from typing import Optional
from typing import TypeVar

from ecm.shared import get_logger
from ecm.tools.registry import Storage

_StepType = TypeVar("_StepType")

_fast_ap_config = Storage("FAST_AP_CONFIG")
_fast_ap_config["port"] = 9301
_fast_ap_config["enable_api"] = False


# ======================= CLASSES ============================
@dataclass
class FastAPStep:
    """
    Represents a single step in the FastAgentProtocol process.

    Attributes:
        name (str): The name of the protocol or agent.
        content (str): The content or payload of the step.
        is_last (bool): Indicates whether this is the last step in the process.
        step_name (Optional[str]): The name of the step, if applicable.
    """

    name: str
    content: str
    is_last: bool
    step_name: Optional[str] = None


class FastAgentProtocol(Generic[_StepType]):
    """
    A simplified and synchronous protocol for managing agent tasks.

    This class allows users to define a sequence of steps for an agent
    using an iterator. It processes each step and yields results until
    the task is complete.
    """

    _logger: ClassVar = get_logger("FastAP")

    def __init__(
        self,
        name: str,
        iterator: Callable[[str], _StepType],
        content_getter: Callable[[_StepType], str],
        is_last_getter: Callable[[_StepType], bool],
        step_name_getter: Optional[Callable[[_StepType], str]] = None,
    ) -> None:
        """
        Initializes the FastAgentProtocol with the required components.

        Args:
            name (str): The name of the protocol or agent.

            iterator (Callable[[str], _StepType]): A callable that returns
                an iterator for processing steps.

            content_getter (Callable[[_StepType], str]): A callable to
                extract content from a step.

            is_last_getter (Callable[[_StepType], bool]): A callable to
                determine if a step is the last one.

            step_name_getter (Optional[Callable[[_StepType], str]]): A
                callable to extract the step name, if applicable. Defaults
                to a lambda returning "null".

        """

        self.name = name
        self.iterator = iterator
        self.content_getter = content_getter
        self.is_last_getter = is_last_getter
        self.step_name_getter = step_name_getter

        if self.step_name_getter is None:
            self.step_name_getter = lambda x: "null"

    def send_task(self, input: str) -> Generator[FastAPStep, None, None]:
        """
        Processes a task by iterating through its steps and yielding results.

        Args:
            input (str): The input string to initialize the task.

        Yields:
            FastAPStep: The current step being processed.

        Examples:
            >>> protocol = FastAgentProtocol(
            ...     name="ExampleProtocol",
            ...     iterator=my_iterator,
            ...     content_getter=lambda step: step.content,
            ...     is_last_getter=lambda step: step.is_last,
            ... )
            >>> for step in protocol.send_task("this is my prompt to the agent"):
            ...     print(step)
        """

        self._logger.debug(f"New task started: `{input}` with server `{self.name}`")
        step_iter = self.iterator(input)

        done = False
        while not done:
            try:
                raw_step = next(step_iter)
            except StopIteration:
                break

            step = FastAPStep(
                name=self.name,
                step_name=self.step_name_getter(raw_step),
                content=self.content_getter(raw_step),
                is_last=self.is_last_getter(raw_step),
            )

            if _fast_ap_config["enable_api"]:
                from cognition_layer.protocols.broadcaster import publish_step_to_api

                publish_step_to_api(step, port=_fast_ap_config["port"])
            yield step
            done = step.is_last


def config_fast_ap(
    port: int = 9301,
    enable_api: bool = True,
):
    """
    Configure the FastAgentProtocol settings.

    Args:
        port (int): The port for the FastAPI server.
        enable_api (bool): Whether to enable the FastAPI server.
    """
    _fast_ap_config["port"] = port
    _fast_ap_config["enable_api"] = enable_api


def autoserver(cls, name: str):
    def server_loader(*args, **kwargs):
        instance = cls(*args, **kwargs)
        return FastAgentProtocol(
            name=name,
            iterator=lambda input: instance.invoke(input),
            step_name_getter=attrgetter("name"),
            content_getter=attrgetter("content"),
            is_last_getter=attrgetter("is_last"),
        )
    return server_loader
