import subprocess
from pathlib import Path
from typing import Literal

from installers.fix_execution_policy_windows import fix_execution_policy_permanent


def add_to_pythonpath(
    directory: Path,
    os: Literal["Windows", "LinuxLike"],
):
    """
    Add the specified directory to the PYTHONPATH environment variable.
    """
    if os == "Windows" and not fix_execution_policy_permanent():
        return False
    if os == "Windows":
        _add_to_pythonpath_windows(directory)
    else:
        _add_to_pythonpath_linux(directory)


def _add_to_pythonpath_linux(directory: Path):
    bashrc = Path.home() / ".bashrc"
    path_line = f'\n# Added by ECM Installer\nexport PYTHONPATH="${{PYTHONPATH}}:{directory.resolve()}"\n'

    if bashrc.exists():
        content = bashrc.read_text()

        if str(directory.resolve()) in content:
            return True
    else:
        bashrc.touch()

    bashrc.write_text(content + path_line)
    return True


def _add_to_pythonpath_windows(directory: Path):
    profile = Path.home() / "Documents" / "WindowsPowerShell" / "profile.ps1"
    path_line = (
        f'\n# Added by ECM Installer\n$env:PYTHONPATH += ";{directory.resolve()}"\n'
    )

    profile.parent.mkdir(parents=True, exist_ok=True)

    if profile.exists():
        content = profile.read_text(encoding="utf-8")
        if str(directory.resolve()) in content:
            return True
    else:
        profile.touch()

    profile.write_text(content + path_line, encoding="utf-8")
    return True


def test_import_cognition_layer_subprocess(os: Literal["Windows", "LinuxLike"]):
    python_code = "import cognition_layer"

    if os == "Windows":
        command = [
            "powershell",
            "-Command",
            f'python -c "{python_code}"',
        ]
    else:
        command = [
            "bash",
            "-c",
            f'python -c "{python_code}"',
        ]

    result = subprocess.run(command, capture_output=True, text=True)
    return result.returncode == 0
