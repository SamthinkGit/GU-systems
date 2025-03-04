import subprocess

from ecm.tools.get_platform import osmethod


@osmethod
def take_screenshot(os, output_file: str = "screenshot.png"):

    if os in ["Ubuntu", "Windows"]:
        import pyautogui
        pyautogui.screenshot(output_file)

    elif os == "Raspbian":
        grim_command = ["grim", output_file]
        subprocess.run(grim_command, capture_output=True, text=True)
