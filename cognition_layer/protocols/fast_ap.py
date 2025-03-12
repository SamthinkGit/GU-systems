from dataclasses import dataclass
from ecm.shared import get_logger
from typing import Callable
from typing import ClassVar
from typing import Generator
from typing import Generic
from typing import Optional
from typing import TypeVar

_StepType = TypeVar("_StepType")


@dataclass
class FastAPStep:
    name: str
    content: str
    is_last: bool
    step_name: Optional[str] = None


class FastAgentProtocol(Generic[_StepType]):

    _logger: ClassVar = get_logger("FastAP")

    def __init__(
        self,
        name: str,
        iterator: Callable[[str], _StepType],
        content_getter: Callable[[_StepType], str],
        is_last_getter: Callable[[_StepType], bool],
        step_name_getter: Optional[Callable[[_StepType], str]] = None,
    ) -> None:

        self.name = name
        self.iterator = iterator
        self.content_getter = content_getter
        self.is_last_getter = is_last_getter
        self.step_name_getter = step_name_getter

        if self.step_name_getter is None:
            self.step_name_getter = lambda x: "null"

    def send_task(self, input: str) -> Generator[FastAPStep, None, None]:

        self._logger.debug(f"New task started: `{input}`")
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

            yield step
            done = step.is_last
