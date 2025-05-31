import multiprocessing
import threading
from typing import TypedDict

from langsmith import traceable

from cognition_layer.deploy.loader import autodeploy_schema
from cognition_layer.protocols.fast_ap import config_fast_ap
from cognition_layer.protocols.fast_ap import FastAgentProtocol
from ecm.launch.core.status import CoreStatus
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import Storage
from ecm.tools.prettify import pretty_print_schema
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter


class CoreConfig(TypedDict):
    schema: str
    tts: bool
    stt: bool
    language: str


class NovaCore1:
    """
    Represents the “core” process logic:
      - Initializes once at the beginning.
      - Waits for an input message.
      - Produces N feedback steps.
      - Waits for a continue signal after each step.
      - Terminates cleanly if soft_stop is triggered.
    """

    _logger = get_logger("NovaCore")

    def __init__(
        self,
        config: CoreConfig,
        feedback_queue: multiprocessing.Queue,
        control_queue: multiprocessing.Queue,
        soft_stop_event,
    ) -> None:

        self._logger.debug(f"Initializing NovaCore1 with config: {config}")
        self.feedback_queue = feedback_queue
        self.control_queue = control_queue
        self.soft_stop_event = soft_stop_event
        self.schema = config["schema"]
        self.config = config
        self.interpreter = PyxcelInterpreter()
        self.finish_event = threading.Event()

        config_fast_ap(enable_listening=True)
        pretty_print_schema(id=self.schema)

        Storage("RETURN_RESPONSE_CONFIG")["voice"] = self.config["tts"]
        Storage("RETURN_RESPONSE_CONFIG")["language"] = self.config["language"]
        self.input = None

        self.server = autodeploy_schema(id=self.schema, interpreter=self.interpreter)
        self._logger.debug("Schema deployed successfully")

        if self.config["stt"]:
            self._logger.debug("Loading STT Engine...")
            from ecm.tools.stt.engine import STT

            self.stt = STT()
            self._logger.debug("Successfully loaded STT Engine.")

        @traceable(run_type="chain", name=self.server.name)
        def execute_user_query(query: str):
            for step in self.server.send_task(query):
                self._logger.debug(f"Step completed: -> {step}")
            self.finish_event.set()

        self.execute_user_query = execute_user_query

    def obtain_input(self) -> None:
        self.feedback_queue.put(CoreStatus.OBTAINING_INPUT.value)
        if self.config["stt"]:
            input("Press Enter to start STT...")
            self.stt.start()
            input("Press Enter to stop STT...")
            self.stt.stop()

            self.input = self.stt.text
            self._logger.info(f"Recognized: {self.input}")
            self.stt.clean()
        else:
            self.feedback_queue.put(CoreStatus.INPUT_REQUIRED.value)
            self.input = self.control_queue.get()

    def run(self) -> None:

        if self.input is None:
            self._logger.debug("No input provided; obtaining input now.")
            self.obtain_input()

        self.feedback_queue.put(CoreStatus.STARTING_CHAIN.value)
        step_queue = FastAgentProtocol.get_local_step_queue()
        runner = threading.Thread(
            target=self.execute_user_query,
            args=(self.input,),
            daemon=True,
        )
        runner.start()

        while not self.finish_event.is_set():
            self.feedback_queue.put(CoreStatus.STEP_STARTED.value)
            step = step_queue.get(timeout=None)
            self.feedback_queue.put(CoreStatus.STEP_COMPLETED.value)
            self.feedback_queue.put(str(step))

            try:
                self._logger.debug("Waiting for control signal...")
                signal: bool = self.control_queue.get(timeout=None)

                if not signal:
                    self._logger.debug("Received False on control signal; exiting.")
                    self._exit()
                    break

            except Exception:
                self._logger.debug("Control queue closed; exiting.", exc_info=True)
                self._exit()
                break

            if self.soft_stop_event.is_set():
                self._logger.debug("Soft stop detected; terminating early.")
                self._exit()
                break

            FastAgentProtocol.confirm_step()

        self.input = None
        runner.join(timeout=10)

    def _exit(self) -> None:
        self.feedback_queue.put(CoreStatus.EXITING_CHAIN.value)
        FastAgentProtocol.soft_stop()
        FastAgentProtocol.confirm_step()


def autorun(
    config: CoreConfig,
    feedback_queue: multiprocessing.Queue = None,
    control_queue: multiprocessing.Queue = None,
    soft_stop_event=None,
):
    core = NovaCore1(
        config=config,
        feedback_queue=feedback_queue or multiprocessing.Queue(),
        control_queue=control_queue or multiprocessing.Queue(),
        soft_stop_event=soft_stop_event or multiprocessing.Event(),
    )
    while True:
        core.obtain_input()
        core.run()
