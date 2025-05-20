import json
import subprocess
from typing import Generator
from typing import Optional

import requests

from cognition_layer.protocols.fast_ap import FastAPStep
from ecm.shared import get_logger
from ecm.shared import get_root_path
from ecm.tools.registry import Storage

MAIN_DEPLOY_FILE_PATH = get_root_path() / "ecm" / "core" / "main" / "deploy.py"


class SchemaListener:
    _logger = get_logger("SchemaListener")

    def __init__(self, schema_name: str, api_port: int = -1):

        path = MAIN_DEPLOY_FILE_PATH.resolve().absolute()
        self.schema = schema_name
        self.target_command = ["python", path, "--api", schema_name]
        self.api_port = (
            Storage("FAST_AP_CONFIG")["port"] if api_port == -1 else api_port
        )
        self.proc: Optional[subprocess.Popen] = None

    def start_process(self, query: str):
        """Start the Scheme in background."""
        if self.proc is not None:
            raise RuntimeError("Proccess has already been started.")
        command = self.target_command + ["--prompt", query]
        self.proc = subprocess.Popen(command)
        self._logger.debug(f"Launched schema in background: {self.schema}")

    def stop_process(self):
        """Stop the Scheme process."""
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            self.proc.wait()
            self._logger.debug(f"Schema process stopped: {self.schema}")
        self.proc = None

    def invoke(self, query: str) -> Generator[FastAPStep, None, None]:
        """Invoke the Scheme process and listen to its output."""

        if self.proc is None:
            self.start_process(query)

        url = f"http://localhost:{self.api_port}/steps/stream"
        self._logger.info("Waiting for connection...")

        try:
            with requests.get(url, stream=True) as response:
                if response.status_code != 200:
                    raise RuntimeError(f"Error when connecting: {response.status_code}")

                for line in response.iter_lines(decode_unicode=True):
                    if line.startswith("data: "):
                        payload = line.replace("data: ", "").strip()
                        try:
                            data = json.loads(payload)
                            step = FastAPStep(
                                name=data["name"],
                                step_name=data.get("step_name", "null"),
                                content=data["content"],
                                is_last=False,
                            )
                            if step.name == "FAST_AP_EXIT":
                                self._logger.info("Received exit signal.")
                                self.stop_process()
                                break

                            yield step
                        except Exception as e:
                            self.stop_process()
                            self._logger.error(f"Error processing step: {e}")
                            raise e

        except requests.exceptions.ConnectionError:
            self._logger.error(
                "Error on connection to the backend server. "
                "\nEnsure the backend server is running and the port is correct."
                "\nYou can use the flag --backend to start the server.",
                exc_info=False,
            )
            raise
        finally:
            self.stop_process()
