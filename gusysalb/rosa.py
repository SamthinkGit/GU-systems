from collections import defaultdict
from typing import Callable
from typing import DefaultDict

from gusysalb.alb import ALB
from gusysalb.nodes import NodeRegistry
from gusyscore.core import get_logger
from gusyscore.gateway.mocks.debug import empty_function
from gusysros.tools.feedback import Feedback
from gusysros.tools.packages import ActionPackage
from gusysros.tools.packages import SequencePackage
from gusysros.tools.packages import SequencePriority
from gusysros.tools.registry import ItemRegistry
from gusysros.types.basic import ReservedTypeCode


class ROSA:

    _sequence_publisher = None
    _task_callback: DefaultDict[str, Callable] = None
    _logger = get_logger("ROSA")
    _instance = None
    _built = False

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
        if not cls._built:
            cls._built = True
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

    def soft_stop(self, task_id: str = "default") -> None:
        action = ActionPackage(
            action_id=ItemRegistry.get_id(empty_function)
        )

        priority = SequencePriority.INTERRUPTION
        seq = SequencePackage(
            task_id=task_id,
            type=ReservedTypeCode.SOFT_STOP.value,
            priority=priority,
            actions=[action],
        )
        self.execute(seq)
