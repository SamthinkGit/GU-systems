from collections import defaultdict
from typing import Callable
from typing import DefaultDict

from gusysalb.alb import ALB
from gusysalb.nodes import NodeRegistry
from gusyscore.core import get_logger
from gusysros.tools.feedback import Feedback
from gusysros.tools.packages import SequencePackage


class ROSA:

    _sequence_publisher = None
    _task_callback: DefaultDict[str, Callable] = None
    _logger = get_logger("ROSA")
    _instance = None

    def __new__(cls) -> None:
        """
        Ensures that exists only one instance (singleton pattern).
        """
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        cls._build()
        return cls._instance

    @classmethod
    def _build(cls) -> None:
        alb = ALB()
        alb.build_all(feedback_listener=ROSA.callback_selector)
        cls._sequence_publisher = NodeRegistry.inited_nodes["sequence_publisher"]
        cls._task_callback = defaultdict(ROSA.empty_callback)
        cls._logger.debug("Building completed")

    @staticmethod
    def callback_selector(msg: str) -> None:
        feedback = Feedback.from_pkg(msg)
        callback = ROSA._task_callback[feedback.task_id]
        callback(feedback)

    @staticmethod
    def empty_callback(feedback: Feedback) -> None:
        ROSA._logger.warn(
            f"A feedback package has not been processed <[{feedback.task_id}]: {feedback.object}>"
        )

    def new_task(
        self, task_id: str = "default", feedback_callback: Callable = empty_callback
    ) -> None:
        self._task_callback[task_id] = feedback_callback

    def execute(self, sequence: SequencePackage):
        self._sequence_publisher.send_package(sequence)
