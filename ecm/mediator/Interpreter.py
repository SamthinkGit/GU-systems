from typing import Any
from typing import Callable
from typing import Optional

from ecm.exelent.parser import ParsedTask


class Interpreter:

    def __init__(self) -> None: ...
    def run(self, task: ParsedTask, callback: Optional[Callable]) -> None: ...
    def stop(self, task: str) -> Any: ...
    def hard_stop(self, task: str) -> Any: ...
    def wait_for(self, task: str, call: str) -> None: ...


class InterpreterSupports:
    stop: bool
    hard_stop: bool
    sync_calls: bool
    async_calls: bool
    wait_for: bool
    callbacks: bool
    feedback: bool
    callback_keywords: tuple[str]
    types: tuple[str]
