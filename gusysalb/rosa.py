import threading
from collections import defaultdict
from typing import Callable
from typing import DefaultDict
from typing import Dict
from typing import List
from typing import Tuple

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
    _conditions: Dict[str, List[Tuple[ReservedTypeCode, threading.Condition]]] = {}
    _lock = threading.Lock()

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
            alb.build_all(feedback_listener=ROSA.callback_manager)
            cls._sequence_publisher = NodeRegistry.inited_nodes["sequence_publisher"]
            cls._task_callback = defaultdict(ROSA.empty_callback)
            cls._logger.debug("Building completed")

    @staticmethod
    def callback_manager(msg: str) -> None:
        feedback = Feedback.from_pkg(msg)
        with ROSA._lock:
            threads_to_notify = ROSA._conditions.get(feedback.task_id, None)

        if threads_to_notify is not None:
            for condition, lock in threads_to_notify:
                if condition.value == feedback._exec_status.value:
                    with lock:
                        lock.notify()

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

        if task_id not in self._conditions.keys():
            self._conditions[task_id] = []

    def execute(self, sequence: SequencePackage):
        self._sequence_publisher.send_package(sequence)

    def soft_stop(self, task_id: str = "default") -> None:
        action = ActionPackage(action_id=ItemRegistry.get_id(empty_function))

        priority = SequencePriority.INTERRUPTION
        seq = SequencePackage(
            task_id=task_id,
            type=ReservedTypeCode.SOFT_STOP.value,
            priority=priority,
            actions=[action],
        )
        self.execute(seq)

    def wait_for(self, task_id: str, code: ReservedTypeCode):
        condition = threading.Condition()
        entry = [code, condition]

        if task_id not in self._conditions.keys():
            self._conditions[task_id] = []

        with self._lock:
            self._conditions[task_id].append(entry)
        with condition:
            self._logger.debug(f"Waiting for {code.name} from {task_id}")
            condition.wait()
            self._logger.debug(f"Waiting for {code.name} completed in {task_id}")

        with self._lock:
            self._conditions[task_id].append(entry)
