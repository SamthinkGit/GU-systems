"""
Scheduler
===============================

This module implements a scheduler system for managing tasks and actions within
the Pyxcel execution framework. It provides a Feedback mechanism that allows
task execution status updates and error handling through callbacks. The core
classes include `Feedback`, which handles the communication of execution status,
and `Scheduler`, which serves as an abstract base class for specific scheduling
strategies like `Sequential`.

The module leverages threading to enable concurrent execution of tasks while
maintaining a lock mechanism to ensure thread safety. It also includes the ability
to respond to feedback events and manage a history of responses.
"""
import logging
import threading
from abc import ABC
from abc import abstractmethod
from traceback import format_exc
from typing import Any
from typing import Callable
from typing import Optional
from typing import Union

from ecm.exelent.parser import ParsedAction
from ecm.mediator.feedback import ExecutionStatus
from ecm.mediator.feedback import Feedback as FeedbackTemplate
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import Action
from ecm.tools.item_registry_v2 import Item
from ecm.tools.item_registry_v2 import ItemRegistry
from ecm.tools.item_registry_v2 import Storage
from ecm.tools.item_registry_v2 import Tool
from execution_layer.pyxcel.constants import ENABLE_FEEDBACK_RESPONSE_HISTORY
from execution_layer.pyxcel.constants import RAISE_FEEDBACK_FAILURES

ResponseStorage = Storage("pyxcel-responses")

ASSUME_ONLY_ACTIONS = True


class Feedback(FeedbackTemplate):

    _feedback_callback: Optional[Callable[["Feedback"], None]] = None
    _logger = get_logger("Feedback")

    def __init__(self, task_id: str) -> None:
        self.task_id = task_id

    def publish(
        self,
        object: Any,
        _exec_status: ExecutionStatus = ExecutionStatus.RUNNING,
        **kwargs,
    ):
        """
        Publishes feedback for the task, invoking the feedback callback if set.

        This method captures the object and execution status, then triggers
        the feedback callback function to notify listeners. If an error occurs
        during this process, it logs the error and may raise an exception
        based on the configuration.

        :param object: The object representing the feedback to publish.
        :param _exec_status: The execution status of the task.
        """

        if self._feedback_callback is None:
            return
        try:
            self.object = object
            self._exec_status = _exec_status
            self._feedback_callback(self)
        except Exception as e:
            self._logger.error(
                f"Error when calling feedback callback in task `{self.task_id}`:",
                exc_info=True,
            )
            if RAISE_FEEDBACK_FAILURES:
                raise e

    def response(self, object: Any, _exec_status: ExecutionStatus):
        """
        Records the latest response for the feedback and updates the storage.

        This method creates a new Feedback instance and stores it in the
        ResponseStorage. If feedback response history tracking is enabled,
        it appends the feedback as a list of feedback messages.

        :param object: The object representing the feedback response.
        :param _exec_status: The execution status to associate with the feedback.
        """

        feedback = Feedback(task_id=self.task_id)
        feedback.object = object
        feedback._exec_status = _exec_status

        ResponseStorage["latest_response"] = feedback
        if ENABLE_FEEDBACK_RESPONSE_HISTORY:
            if self.task_id not in ResponseStorage.keys():
                ResponseStorage[self.task_id] = []

            ResponseStorage[self.task_id].append(feedback)

    @classmethod
    def parse(feedback, *args, **kwargs):
        return feedback


class Scheduler(ABC):

    _lock = threading.Lock()
    name: str
    actions: list[Union[ParsedAction, "Scheduler"]]
    _feedback: Optional[Feedback] = None
    _thread: Optional[threading.Thread] = None
    _logger: logging.Logger

    def __init__(
        self,
        actions: list[Union[ParsedAction, "Scheduler"]],
        feedback_callback: Optional[Callable] = None,
        registry: ItemRegistry = ItemRegistry(),
    ) -> None:
        self._registry = registry
        self.actions = list(actions)
        self._feedback = Feedback(self.name)
        self._feedback._feedback_callback = feedback_callback

    @abstractmethod
    def run(self):
        """
        Executes the scheduled actions in the defined order.

        This method must be implemented by subclasses, defining the specific
        behavior for how actions are processed during execution. It is
        intended to be the main loop for running tasks.
        """
        ...

    @abstractmethod
    def stop(self):
        """
        Halts the execution of the scheduled actions.

        This method must be implemented by subclasses to provide a defined
        mechanism for stopping the execution process, ensuring that any
        necessary cleanup is performed.
        """
        ...

    def arun(self):
        """
        Starts the execution of the run method in a separate thread.

        This method initializes a new thread to run the scheduled tasks,
        allowing the main program to continue executing without blocking.

        :warning: The user must use Scheduler.clean() after arunning a
        scheduler
        """

        self._thread = threading.Thread(target=self.run)
        self._thread.start()

    def clean(self):
        """
        Waits for the running thread to complete.

        This method ensures that the thread executing the tasks has finished
        before proceeding, allowing for proper resource management and
        synchronization.
        """
        if self._thread is not None:
            self._thread.join()

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        self.run()


class Sequential(Scheduler):

    name: str = "Sequential"
    _logger = get_logger("Sequential")
    soft_stop: bool = False

    def run(self) -> None:
        """
        Runs the scheduled actions sequentially and manages feedback.

        This method iterates over each action, executing them in order,
        handling any exceptions, and providing feedback on the execution
        status. It also implements soft-stop functionality to halt execution
        gracefully when requested.
        """

        self._logger.debug(
            f"Starting Sequential Task with {len(self.actions)} actions."
        )
        success = True

        for action in self.actions:
            try:
                if self.soft_stop:
                    self._logger.debug(
                        f"Soft-stop called, action `{action.name}` has been prevented."
                    )
                    self._feedback.publish(
                        object="Abort (By Soft-Stop)",
                        _exec_status=ExecutionStatus.ABORT,
                    )
                    success = False
                    break

                target_struct: Scheduler | Action | Tool | None = None

                if isinstance(action, Scheduler):
                    target_struct = action

                if target_struct is None and not ASSUME_ONLY_ACTIONS:
                    target_struct = self._registry.get(
                        name=action.name, return_multiple=False
                    )
                if target_struct is None and ASSUME_ONLY_ACTIONS:
                    target_struct = self._registry.get(
                        name=action.name, type="action", return_multiple=False
                    )

                if target_struct is None:
                    self._logger.debug(
                        f"Action `{action.name}` cannot be found on the ItemRegistry. (Maybe "
                        "ItemRegistry has not been loaded?), aborting..."
                    )
                    self._feedback.publish(
                        object="Abort (By Invalid Action)",
                        _exec_status=ExecutionStatus.ABORT,
                    )
                    success = False
                    break

                if isinstance(target_struct, Item) and not target_struct.active:
                    self._logger.debug(
                        f"Action `{action.name}` has not been loaded (action inactive), aborting..."
                    )
                    self._feedback.publish(
                        object="Abort (By Inactive Action)",
                        _exec_status=ExecutionStatus.ABORT,
                    )
                    success = False
                    break

                result = None
                if isinstance(target_struct, Item):
                    result = target_struct.content(*action.args, **action.kwargs)
                    self._logger.debug(
                        f"Action `{action.name}`({action.args}{action.kwargs}) completed."
                    )

                if isinstance(target_struct, Scheduler):
                    result = target_struct.run()

                if result is not None:
                    self._feedback.publish(
                        object=result, _exec_status=ExecutionStatus.RESULT
                    )

                self._feedback.publish(
                    object=action.name, _exec_status=ExecutionStatus.STEP
                )

            except Exception:
                self._logger.error(
                    f"Exception when running {action.name}({action.args},{action.kwargs}):",
                    exc_info=True,
                )
                self._feedback.publish(
                    object=format_exc(),
                    _exec_status=ExecutionStatus.RESULT,
                )
                success = False

        if success:
            self._feedback.publish(
                object="Success", _exec_status=ExecutionStatus.SUCCESS
            )
        self._feedback.publish(object="Finish", _exec_status=ExecutionStatus.FINISH)

    def stop(self):
        self.soft_stop = True


# ----- YOU MUST ADD HERE ALL THE SCHEDULER NAMES --------
PYXCEL_SUPPORTED_SCHEDULERS: dict[str, "Scheduler"] = {
    "Sequential": Sequential,
}
# ---------------------------------------------------------
