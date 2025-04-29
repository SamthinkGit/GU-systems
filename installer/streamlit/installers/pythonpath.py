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


def get_documents_folder():
    import ctypes.wintypes

    CSIDL_PERSONAL = 5  # Folder ID for 'My Documents'
    SHGFP_TYPE_CURRENT = 0

    buf = ctypes.create_unicode_buffer(ctypes.wintypes.MAX_PATH)
    result = ctypes.windll.shell32.SHGetFolderPathW(
        None, CSIDL_PERSONAL, None, SHGFP_TYPE_CURRENT, buf
    )

    if result != 0:
        raise RuntimeError(
            "Couldn't obtain the Documents folder for integrating pythonpath"
        )

    return Path(buf.value)


def _add_to_pythonpath_windows(directory: Path):

    documents_folder = get_documents_folder()
    profiles = [
        documents_folder / "WindowsPowerShell" / "profile.ps1",
        documents_folder / "PowerShell" / "profile.ps1"
    ]
    for profile in profiles:
        path_line = (
            f'\n# Added by ECM Installer\n$env:PYTHONPATH += ";{directory.resolve()}"\n'
        )

        if not profile.parent.exists():
            profile.parent.mkdir(parents=True, exist_ok=True)

        if profile.exists():
            content = profile.read_text(encoding="utf-8")
            if str(directory.resolve()) in content:
                continue
        else:
            content = ""
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
