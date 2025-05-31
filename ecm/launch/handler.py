import multiprocessing
from typing import Optional

from ecm.launch.core.engine import autorun
from ecm.launch.core.engine import CoreConfig


class CoreHandler:
    """
    Manages a Core instance in a separate process.
    """

    def __init__(self, config: CoreConfig) -> None:
        self._feedback_queue: multiprocessing.Queue = multiprocessing.Queue()
        self._control_queue: multiprocessing.Queue = multiprocessing.Queue()
        self._soft_stop_event = multiprocessing.Event()

        self._process = multiprocessing.Process(
            target=autorun,
            args=(
                config,
                self._feedback_queue,
                self._control_queue,
                self._soft_stop_event,
            ),
        )
        self._started = False

    def start(self) -> None:
        if self._started:
            raise RuntimeError("The process is already running.")
        self._process.start()
        self._started = True

    def send(self, message: str) -> None:
        if not self._started:
            raise RuntimeError("The process has not been started.")
        self._control_queue.put(message)

    def get_next_step(self, timeout: Optional[float] = None) -> str:
        if not self._started:
            raise RuntimeError("The process has not been started.")
        step = self._feedback_queue.get(timeout=timeout)
        return step

    def continue_execution(self) -> None:
        if not self._started:
            raise RuntimeError("The process has not been started.")
        self._control_queue.put(True)

    def soft_stop(self) -> None:
        if not self._started:
            raise RuntimeError("The process has not been started.")
        self._control_queue.put(False)
        self._soft_stop_event.set()

    def hard_stop(self, timeout: float = 10) -> None:
        if not self._started:
            raise RuntimeError("The process has not been started.")
        self._process.terminate()
        self._process.join(timeout=timeout)
        self._started = False

    def is_alive(self) -> bool:
        return self._process.is_alive()
