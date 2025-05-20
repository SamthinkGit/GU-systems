import subprocess

import requests

from cognition_layer.protocols.fast_ap import FastAPStep
from ecm.shared import get_logger
from ecm.shared import get_root_path
from ecm.tools.registry import Storage


_logger = get_logger("broadcaster")
_uvicorn_process = None


def publish_step_to_api(step: FastAPStep, port: int):
    """Publish a FastAPStep to a local API endpoint.
    Args:
        step (FastAPStep): The step to publish.
        port (int): The port of the local API server.
    """
    url = f"http://localhost:{port}/steps"

    payload = {
        "name": step.name,
        "step_name": step.step_name,
        "content": step.content,
        "is_last": step.is_last,
    }

    response = requests.post(url, json=payload)
    response.raise_for_status()


def listen_to_steps_from_api(port: int):
    """Listen to steps from a local API endpoint using Server-Sent Events (SSE).
    Args:
        port (int): The port of the local API server.
    """
    url = f"http://localhost:{port}/steps/stream"
    _logger.info(f"Waiting for connection from {url}...")

    with requests.get(url, stream=True) as response:
        if response.status_code != 200:
            raise RuntimeError(f"Error when connecting: {response.status_code}")
        _logger.info("Connected. Waiting for steps...\n")

        for line in response.iter_lines(decode_unicode=True):
            if line.startswith("data: "):
                payload = line.replace("data: ", "").strip()
                _logger.debug(f"Received step: {payload}")


def deploy_backend():
    global _uvicorn_process
    if _uvicorn_process is not None:
        return

    port = Storage("FAST_AP_CONFIG")["port"]

    _uvicorn_process = subprocess.Popen(
        ["uvicorn", "app:app", "--port", str(port)],
        cwd=str(get_root_path() / "cognition_layer" / "protocols"),
    )


def terminate_backend():
    global _uvicorn_process
    if _uvicorn_process is None:
        raise RuntimeError("Server is not running.")

    _uvicorn_process.terminate()
    _uvicorn_process.wait()
    _uvicorn_process = None
