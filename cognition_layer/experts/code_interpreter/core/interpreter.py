import builtins
import os
import time
from contextlib import contextmanager
from typing import Literal

from ecm.shared import get_logger

os.environ["LANGSMITH_PROJECT"] = "default"
os.environ["LANGSMITH_DEFAULT_RUN_NAME"] = "Interpreter"
os.environ["LANGSMITH_API_KEY"] = os.environ["LANGCHAIN_API_KEY"]


class Interpreter:

    _logger = get_logger("CodeInterpreter")

    def __init__(self, model: str = "gpt-4o-mini", verbose: bool = False):
        self._logger.debug("Initializing Interpreter...")
        from interpreter import interpreter as _interpreter

        self._interpreter = _interpreter
        self._interpreter.messages = []
        self._interpreter.llm.model = model
        self._interpreter.verbose = verbose
        self._interpreter.highlight_active_line = False
        self._interpreter.disable_telemetry = True
        self._interpreter.system_message
        self.paused = False

    def invoke(self, query: str, animation: bool = True, auto_run: bool = False) -> str:

        self._logger.debug(f"Starting Interpreter with query: {query}")
        if animation:
            return self.invoke_with_animation(query, auto_run=auto_run)
        return self.invoke_without_animation(query)

    def invoke_without_animation(self, query: str) -> list[dict]:
        self._interpreter.chat(message=query, display=False, blocking=True)
        return combine_messages(self._interpreter.messages)

    def invoke_with_animation(self, query: str, auto_run: bool = False) -> list[dict]:
        from cognition_layer.experts.code_interpreter.core.animation import WorkerThread
        from cognition_layer.experts.code_interpreter.core.animation import (
            start as start_app,
        )
        from cognition_layer.experts.code_interpreter.core.animation import (
            finish as finish_app,
        )

        self._yes_to_all = False
        if auto_run:
            self._interpreter.auto_run = True

        def pause():
            self.paused = True

        def continue_():
            self.paused = False

        app, win = start_app(
            on_pause=pause,
            on_continue=continue_,
        )
        thread = WorkerThread(self._stream(query), delay=0.1)
        thread.data_ready.connect(win.write_stream)
        thread.finished.connect(finish_app)
        thread
        thread.ask_confirmation.connect(
            lambda: thread.confirmation_response.emit(self._request_confirmation(win))
        )
        thread.start()
        app.exec_()

        return combine_messages(self._interpreter.messages)

    def _stream(self, query: str):
        for chunk in self._interpreter.chat(query, display=False, stream=True):
            while self.paused:
                time.sleep(0.1)
            yield chunk

    def _request_confirmation(self, win):
        if self._yes_to_all:
            return "y"

        confirmation: Literal["yes", "no", "yes-all"] = win.ask_yes_no()
        self._logger.debug(f"REQUESTION CONFIRMATION={confirmation}")

        if confirmation == "yes":
            return "y"
        if confirmation == "no":
            return "n"
        if confirmation == "yes-all":
            self._yes_to_all = True
            return "y"

        return "y"

    def reset(self):
        self._interpreter.messages = []


@contextmanager
def override_input(new_input_func):
    orig_input = builtins.input
    builtins.input = new_input_func
    try:
        yield
    finally:
        builtins.input = orig_input


def combine_messages(messages):
    return "".join([str(msg["content"]) for msg in messages if "content" in msg])
