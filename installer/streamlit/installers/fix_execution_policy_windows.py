import ctypes
import os
import subprocess

import streamlit as st


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


def fix_execution_policy_permanent(write_output_with_st: bool = True) -> bool:
    if os.name != "nt":
        return True

    current_policy = get_current_execution_policy()
    if current_policy.lower() == "remotesigned":
        return True

    if not is_admin():
        if write_output_with_st:
            st.error(
                "Administrator privileges required to change Execution Policy permanently."
                "\nPlease re-run this script as Administrator."
            )
        return False

    try:
        subprocess.run(
            [
                "powershell",
                "-Command",
                "Set-ExecutionPolicy RemoteSigned -Scope LocalMachine -Force",
            ],
            check=True,
        )
        if write_output_with_st:
            st.write(
                "[SUCCESS] Execution policy successfully set to RemoteSigned for LocalMachine"
            )
        return True
    except subprocess.CalledProcessError as e:
        if write_output_with_st:
            st.error("[ERROR] Failed to change Execution Policy.")
            st.error(f"Details: {e}")
        return False
