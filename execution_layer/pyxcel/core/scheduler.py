import logging
import threading
from abc import ABC
from abc import abstractmethod
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
    def run(self): ...

    @abstractmethod
    def stop(self): ...

    def arun(self):
        self._thread = threading.Thread(target=self.run)
        self._thread.start()

    def clean(self):
        if self._thread is not None:
            self._thread.join()

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        self.run()


class Sequential(Scheduler):

    name: str = "Sequential"
    _logger = get_logger("Sequential")
    soft_stop: bool = False

    def run(self) -> None:
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

                if target_struct is None:
                    target_struct = self._registry.actions.get(action.name)

                if target_struct is None:
                    target_struct = self._registry.tools.get(action.name)

                if target_struct is None:
                    self._logger.debug(
                        f"Action `{target_struct}` cannot be found on the ItemRegistry, aborting..."
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

                if isinstance(target_struct, Scheduler):
                    result = target_struct.run()

                if result is not None:
                    self._feedback.publish(
                        object=result, _exec_status=ExecutionStatus.RESULT
                    )

                self._feedback.publish(
                    object=action.name, _exec_status=ExecutionStatus.STEP
                )
                self._logger.debug(f"Action `{action.name}` completed.")

            except Exception:
                self._logger.error(
                    f"Exception when running {action.name}({action.args},{action.kwargs}):",
                    exc_info=True,
                )
                success = False

        if success:
            self._feedback.publish(
                object="Success", _exec_status=ExecutionStatus.SUCCESS
            )
        self._feedback.publish(object="Finish", _exec_status=ExecutionStatus.FINISH)

    def stop(self):
        self.soft_stop = True


# ----- YOU MUST ADD HERE ALL THEY SCHEDULER NAMES --------
PYXCEL_SUPPORTED_SCHEDULERS: dict[str, "Scheduler"] = {
    "Sequential": Sequential,
}
# ---------------------------------------------------------
