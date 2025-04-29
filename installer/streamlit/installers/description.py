import locale
import os
import subprocess
from dataclasses import dataclass
from typing import Literal


@dataclass
class InstallerDescription:
    cognition_layers: list[str]
    execution_layers: list[str]
    ecm_dependencies: list[str]
    precommit: bool
    api_keys: dict[str, str]
    os: Literal["windows", "linux", "raspbian"]
    install_with_conda: bool
    conda_path: str = "ecm"
    git_pull: bool = False
    setup_python_path: bool = True


def detect_os() -> str:
    """
    Detect the operating system.
    """
    import platform

    os_name = platform.system().lower()
    if os_name == "windows":
        return "windows"
    elif os_name == "linux":
        return "linux"
    elif os_name == "darwin":
        return "macos"
    else:
        raise ValueError(f"Unsupported OS: {os_name}")


def run_command_live(cmd: list[str]) -> tuple[int, str]:
    use_shell = os.name == "nt"
    system_encoding = locale.getpreferredencoding()

    process = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        shell=use_shell,
        encoding=system_encoding,
    )
    result = []

    for line in process.stdout:
        if line:
            result.append(line)
    result = "".join(result)

    process.stdout.close()
    return_code = process.wait()
    return return_code, result
