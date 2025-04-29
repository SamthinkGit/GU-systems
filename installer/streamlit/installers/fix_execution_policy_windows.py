import ctypes
import os
import subprocess


def is_admin():
    try:
        return ctypes.windll.shell32.IsUserAnAdmin()
    except Exception:
        return False


def get_current_execution_policy():
    result = subprocess.run(
        ["powershell", "-Command", "Get-ExecutionPolicy -Scope LocalMachine"],
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def fix_execution_policy_permanent() -> bool:
    if os.name != "nt":
        return True

    current_policy = get_current_execution_policy()
    if current_policy.lower() == "remotesigned":
        return True

    if not is_admin():
        raise PermissionError(
            "Administrator privileges required to change Execution Policy permanently."
            "\nPlease re-run this script as Administrator."
        )

    try:
        subprocess.run(
            [
                "powershell",
                "-Command",
                "Set-ExecutionPolicy RemoteSigned -Scope LocalMachine -Force",
            ],
            check=True,
        )
        return True
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Failed to change Execution Policy. Details: {e}") from e
