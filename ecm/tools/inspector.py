import json
import platform
import subprocess
from typing import Dict
from typing import Optional

import psutil


class SystemInspector:
    def __init__(self):
        pass

    def get_system_info(self) -> Dict[str, str]:
        """Get basic information about the system."""
        return {
            "platform": platform.system(),
            "platform-release": platform.release(),
            "platform-version": platform.version(),
            "architecture": platform.machine(),
            "hostname": platform.node(),
        }

    def get_focused_window(self) -> Optional[str]:
        """Get the title of the currently focused window."""
        if platform.system() == "Windows":
            import win32gui

            window = win32gui.GetForegroundWindow()
            return win32gui.GetWindowText(window)
        elif platform.system() == "Linux":
            import subprocess

            window = subprocess.check_output(
                ["xdotool", "getactivewindow", "getwindowname"]
            )
            return window.decode("utf-8").strip()
        return None

    def get_focused_window_info(self) -> Optional[Dict[str, str]]:
        """Get detailed information about the currently focused window on Linux."""
        try:
            # Get the ID of the currently focused window
            window_id = (
                subprocess.check_output(["xdotool", "getactivewindow"])
                .strip()
                .decode("utf-8")
            )

            # Get detailed information about the window using xprop
            window_info = subprocess.check_output(["xprop", "-id", window_id]).decode(
                "utf-8"
            )

            # Parse the window information
            window_details = self._parse_xprop_output(window_info)
            pid = window_details.get("_NET_WM_PID(CARDINAL)")

            result = {
                "WM_CLASS(STRING)": window_details["WM_CLASS(STRING)"],
                "WM_NAME(UTF8_STRING)": window_details["WM_NAME(UTF8_STRING)"],
            }

            if pid:
                # Get detailed information about the process using psutil
                process_info = psutil.Process(int(pid)).as_dict(
                    attrs=["pid", "name", "username", "status", "create_time"]
                )
                result["process_info"] = process_info

            return result

        except subprocess.CalledProcessError:
            return None

    def _parse_xprop_output(self, xprop_output: str) -> Dict[str, str]:
        """
        Parse the output from xprop into a dictionary.

        Args:
            xprop_output (str): The raw output from xprop.

        Returns:
            Dict[str, str]: A dictionary containing parsed window information.
        """
        details = {}
        for line in xprop_output.splitlines():
            if "=" in line:
                key, value = line.split("=", 1)
                details[key.strip()] = value.strip()
        return details

    def summary(self) -> Dict[str, str]:
        return {
            'System Information': self.get_system_info(),
            'Focused Window': self.get_focused_window(),
            'Focused Window Properties': self.get_focused_window_info()
        }


def main():
    inspector = SystemInspector()

    system_info = inspector.get_system_info()
    focused_window = inspector.get_focused_window()
    focused_window_info = inspector.get_focused_window_info()

    print("System Information:", json.dumps(system_info, indent=4))
    print("Focused Window:", focused_window)
    print("Focused Window Info:", json.dumps(focused_window_info, indent=4))


if __name__ == "__main__":
    main()
