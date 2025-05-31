import os
from functools import cache
from typing import Generator

from cognition_layer.experts.code_interpreter.core.interpreter import Interpreter
from cognition_layer.protocols.fast_ap import FastAPStep
from cognition_layer.tools.voice.engine import play
from ecm.shared import get_logger
from ecm.shared import get_root_path
from ecm.tools.registry import ItemRegistry
from ecm.tools.registry import Storage


@cache
def get_interpreter(*args, **kwargs):
    return Interpreter(*args, **kwargs)


class CodeInterpreterExpert:

    _logger = get_logger("CodeInterpreterExpert")

    @ItemRegistry.require_dependencies("_private_interpreter-tools")
    def __init__(
        self, model: str = "gpt-4o-mini", auto_run: bool = False, *args, **kwargs
    ):

        self.model = model
        self.auto_run = auto_run

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        if query == "<reset>":
            # TODO: Get a better animation handling for supporting reset.
            yield FastAPStep(
                name="CodeInterpreter",
                content="Code interpreter does not support resseting right now",
                is_last=True,
            )
            return

        voice = Storage("RETURN_RESPONSE_CONFIG").get("voice", True)
        if voice:
            self._logger.debug("Playing code execution message")
            play(
                get_root_path()
                / "cognition_layer"
                / "experts"
                / "code_interpreter"
                / "sounds"
                / "code_execution.mp3"
            )

        # We use the ItemRegistry to execute the animation on the client.
        interpreter_with_animation = (
            ItemRegistry().get("_invoke_interpreter_with_animation").content
        )
        result: str = interpreter_with_animation(
            query=query,
            openai_api_key=os.getenv("OPENAI_API_KEY"),
            auto_run=self.auto_run,
            model=self.model,
        )
        yield FastAPStep(
            name="CodeInterpreter",
            content=result,
            is_last=True,
        )
        return
