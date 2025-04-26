from dataclasses import dataclass
from typing import Literal


@dataclass
class InstallerDescription:
    cognition_layers: list[str]
    execution_layers: list[str]
    install_with_conda: bool
    precommit: bool
    api_keys: dict[str, str]
    os: Literal["windows", "linux", "raspbian"]
    source_path: str
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
