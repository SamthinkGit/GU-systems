import subprocess
import time

from ecm.shared import get_root_path


def test_parallel_execution():

    client_bootstrap_path = (
        get_root_path() / "tests" / "bootstraps" / "run_ecm_client.py"
    )
    client_process = subprocess.Popen(
        ["python", str(client_bootstrap_path.absolute())],
    )
    time.sleep(1)
    server_bootstrap_path = (
        get_root_path() / "tests" / "bootstraps" / "run_ecm_server.py"
    )
    server_process = subprocess.Popen(["python", str(server_bootstrap_path.absolute())])
    server_process.terminate()
    client_process.terminate()
