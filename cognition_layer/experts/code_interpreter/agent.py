from functools import cache
from typing import Generator

from cognition_layer.experts.code_interpreter.core.interpreter import Interpreter
from cognition_layer.protocols.fast_ap import FastAPStep


@cache
def get_interpreter(*args, **kwargs):
    return Interpreter(*args, **kwargs)


class CodeInterpreterExpert:

    def __init__(
        self, model: str = "gpt-4o-mini", auto_run: bool = False, *args, **kwargs
    ):
        self._interpreter = get_interpreter(model=model)
        self.auto_run = auto_run

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        if query == "<reset>":
            self._interpreter.reset()
            yield FastAPStep(
                name="CodeInterpreter",
                content="Interpreter reset.",
                is_last=True,
            )
            return
        result: str = self._interpreter.invoke(
            query, animation=True, auto_run=self.auto_run
        )
        yield FastAPStep(
            name="CodeInterpreter",
            content=result,
            is_last=True,
        )
        return
