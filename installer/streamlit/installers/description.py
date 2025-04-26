import locale
import os
import subprocess
from dataclasses import dataclass
from typing import Literal

import streamlit as st


@dataclass
class InstallerDescription:
    cognition_layers: list[str]
    execution_layers: list[str]
    ecm_dependencies: list[str]
    precommit: bool
    api_keys: dict[str, str]
    os: Literal["windows", "linux", "raspbian"]
    source_path: str
    install_with_conda: bool
    conda_path: str = "ecm"


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


def run_command_live(cmd: list[str], write_output_with_st: bool = True) -> int:
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
            if write_output_with_st:
                st.write(line, end="")
            result.append(line)
    result = "".join(result)

    process.stdout.close()
    return_code = process.wait()
    return return_code, result
