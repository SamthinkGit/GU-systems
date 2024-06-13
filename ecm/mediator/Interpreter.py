"""
Interpreter Module
=================================

This module defines the `Interpreter` virtual class for executing tasks parsed by the Exelent Parser.
It also defines the `InterpreterSupports` class, which details the features supported by the
interpreter, such as stopping tasks, handling synchronous and asynchronous calls, and managing
callbacks.

Classes:
- Interpreter: Manages the execution, stopping, and waiting of parsed tasks.
- InterpreterSupports: Specifies the capabilities supported by the interpreter.

Functions:
- __init__: Initializes the interpreter.
- run: Executes a parsed task with an optional callback.
- stop: Stops a running task by its name.
- hard_stop: Forcefully stops a running task by its name.
- wait_for: Waits for a specific call within a task to complete.

"""
from typing import Any
from typing import Callable
from typing import Optional

from ecm.exelent.parser import ParsedTask


class Interpreter:

    def __init__(self) -> None: ...
    def run(self, task: ParsedTask, callback: Optional[Callable]) -> None: ...
    def arun(self, task: ParsedTask) -> None: ...
    def stop(self, task: str) -> Any: ...
    def hard_stop(self, task: str) -> Any: ...
    def wait_for(self, task: str, call: str) -> None: ...
    def kill(self): ...


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
