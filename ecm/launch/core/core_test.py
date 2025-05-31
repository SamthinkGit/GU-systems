import multiprocessing
import random
import string
from typing import Any
from typing import Optional


class CoreTest:
    """
    Represents the “core” process logic:
      - Initializes once at the beginning.
      - Waits for an input message.
      - Produces N feedback steps.
      - Waits for a continue signal after each step.
      - Terminates cleanly if soft_stop is triggered.
    """

    def __init__(
        self,
        input_queue: multiprocessing.Queue,
        feedback_queue: multiprocessing.Queue,
        control_queue: multiprocessing.Queue,
        soft_stop_event,
        config: Optional[dict[str, Any]] = None,
    ) -> None:

        self.input_queue = input_queue
        self.feedback_queue = feedback_queue
        self.control_queue = control_queue
        self.soft_stop_event = soft_stop_event
        self.config = config or {"num_steps": 5}

    def run(self) -> None:
        initial_config_value = self.config.get("initial_value", "default_value")
        self.feedback_queue.put(f"initialized (config: {initial_config_value})")

        try:
            input_msg = self.input_queue.get(timeout=10)
        except Exception:
            self.feedback_queue.put("no_input_received; exiting")
            return

        self.feedback_queue.put(f"received input: {input_msg}")

        num_steps = self.config.get("num_steps", 5)
        for _ in range(1, num_steps + 1):
            if self.soft_stop_event.is_set():
                self.feedback_queue.put("soft_stop_detected; terminating early")
                return

            rand_suffix = "".join(random.choices(string.digits, k=4))
            step_str = f"step-{rand_suffix}"
            self.feedback_queue.put(step_str)

            try:
                _ = self.control_queue.get(timeout=None)
            except Exception:
                self.feedback_queue.put("control_queue_closed; exiting")
                return

        if not self.soft_stop_event.is_set():
            self.feedback_queue.put("result")
        else:
            self.feedback_queue.put("soft_stop_before_final; exiting")
