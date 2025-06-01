import multiprocessing
import queue
import threading
from enum import Enum
from typing import Dict
from typing import Optional
from typing import TypedDict

from langsmith import traceable

from cognition_layer.deploy.loader import autodeploy_schema
from cognition_layer.protocols.fast_ap import config_fast_ap
from cognition_layer.protocols.fast_ap import FastAgentProtocol
from cognition_layer.tools.voice.engine import base64_to_wav_temp
from cognition_layer.tools.voice.engine import get_audios
from cognition_layer.tools.voice.engine import wav_to_base64
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import Storage
from ecm.tools.prettify import pretty_print_schema
from ecm.tools.stt.engine import speech2text
from execution_layer.pyxcel.interpreter.pyxcel_interpreter import PyxcelInterpreter

TIME_UNTIL_HARD_STOP = 5


class InputRequest(Enum):
    START = "<start>"
    PAUSE = "<pause>"
    RESUME = "<resume>"
    EXIT = "<exit>"


class OutputRequest(Enum):
    NOVA_FEEDBACK = "<nova_feedback>"
    NOVA_FINISHED = "<nova_finished>"


class CoreConfig(TypedDict):
    schema: str
    tts: bool
    stt: bool
    language: str


class NovaCore2:
    _logger = get_logger("NovaCore")

    def __init__(self, config: CoreConfig):

        manager = multiprocessing.Manager()
        self.extra: Dict[str, str] = manager.dict()
        self.config = config

        self.control_queue = multiprocessing.Queue()
        self.input_queue = multiprocessing.Queue()
        self.finish_event = multiprocessing.Event()
        self.output_event = multiprocessing.Event()
        self.voice_api_enabled = config.get("voice_api_enabled", False)
        self._process = None

        pretty_print_schema(id=config.get("schema"))

    def update(
        self,
        input_request: Optional[str] = None,
        output_request: Optional[str] = None,
        extra_input: Optional[Dict[str, str]] = None,
        _extra_output: Optional[Dict[str, str]] = None,
    ) -> None:

        if extra_input is not None and "audio" in extra_input:
            self._logger.debug(f"Audio input received: {extra_input['audio']}")
            self.input_queue.put(extra_input["audio"])

        input_request = InputRequest(input_request) if input_request else None
        output_request = OutputRequest(output_request) if output_request else None

        match input_request:
            case InputRequest.START:
                if self._process is not None:
                    self._logger.error("NovaCore2 is already running xD.")
                    return

                self._process = multiprocessing.Process(
                    target=self.work,
                    args=(self.config,),
                )
                self._process.start()

            case InputRequest.PAUSE:
                self._logger.debug("Pausing NovaCore2...")
                self.control_queue.put(True)
            case InputRequest.RESUME:
                self._logger.debug("Resuming NovaCore2...")
                self.control_queue.put(False)

            case InputRequest.EXIT:
                self._logger.debug("Killing NovaCore2...")
                self._process.terminate()
                self._process = None
                output_request = OutputRequest.NOVA_FINISHED

        match output_request:
            case OutputRequest.NOVA_FEEDBACK:
                self.extra["type"] = "feedback"
                self.extra["content"] = (
                    _extra_output if _extra_output is not None else {}
                )

            case OutputRequest.NOVA_FINISHED:
                self.extra["type"] = "finish"
                self.extra["content"] = (
                    _extra_output if _extra_output is not None else {}
                )

        if output_request is not None:
            self.output_event.set()

    def wait_for_new_state(self) -> None:
        self.output_event.wait()
        self.output_event.clear()

    def work(self, config: CoreConfig) -> None:

        logger = get_logger("NovaProcess")
        interpreter = PyxcelInterpreter()
        config_fast_ap(enable_listening=True)
        if config["tts"]:
            Storage("TTS_CONFIG")["API_ENABLED"] = True
        Storage("RETURN_RESPONSE_CONFIG")["voice"] = config["tts"]
        Storage("RETURN_RESPONSE_CONFIG")["language"] = config["language"]

        server = autodeploy_schema(id=config["schema"], interpreter=interpreter)
        logger.debug("Schema deployed successfully")

        @traceable(run_type="chain", name=server.name)
        def execute_user_query(query: str):
            for step in server.send_task(query):
                logger.debug(f"Step completed: -> {step}")
            self.finish_event.set()

        logger.debug("Waiting for input...")
        input = speech2text(base64_to_wav_temp(self.input_queue.get()))

        step_queue = FastAgentProtocol.get_local_step_queue()
        runner = threading.Thread(
            target=execute_user_query,
            args=(input,),
            daemon=True,
        )
        runner.start()

        while True:
            step = step_queue.get(timeout=None)  # noqa

            if self.voice_api_enabled:
                audios = get_audios()
                for audio in audios:
                    self.update(
                        output_request=OutputRequest.NOVA_FEEDBACK,
                        _extra_output={"audio": wav_to_base64(audio)},
                    )

            pause = False
            try:
                pause = self.control_queue.get_nowait()
            except queue.Empty:
                pass

            while pause:
                logger.debug("ECM is paused...")
                pause = self.control_queue.get()

            if self.finish_event.is_set():
                self.update(
                    output_request=OutputRequest.NOVA_FINISHED,
                )
                break

            FastAgentProtocol.confirm_step()

        self._logger.debug("Finished task")
