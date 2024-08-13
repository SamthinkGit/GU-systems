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
from typing import Literal
from typing import Optional

from ecm.exelent.parser import ParsedTask
from ecm.mediator.feedback import Feedback


class Interpreter:

    feedback_message_class: Feedback

    def __init__(self) -> None: ...

    def run(
        self,
        task: ParsedTask,
        callback: Optional[Callable | Literal["silent"]],
    ) -> None:
        """
        Runs a parsed task (by using the schema defined in /ecm/exelent/parser.py) and
        defines a callback function where all the feedback from the execution will return.
        This function is blocking until the execution finished
        """
        ...

    def arun(self, task: ParsedTask, callback: Optional[Callable]) -> None:
        """
        Defines a function with the same properties as run(), this call though is
        not blocking (does not wait for finish)
        """
        ...

    def stop(self, task: str) -> Any:
        """
        Stops a task that is running at the moment. (Gracefully)
        """
        ...

    def hard_stop(self, task: str) -> Any:
        """
        Kills immediately all the proccess running the task (safe exit)
        """
        ...

    def wait_for(self, task: str, call: str) -> None:
        """
        Waits for a signal/feedback from the execution layer. The signals
        can be defined in the InterpreterSupports
        """
        ...

    def kill(self):
        """
        Cleans all the interpreter before exiting (if necesary)
        """
        ...


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
