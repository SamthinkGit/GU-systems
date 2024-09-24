import subprocess

from ecm.tools.get_platform import check_distro


if check_distro() == "Ubuntu":
    import pyautogui

    def take_screenshot(output_file="screenshot.png"):
        pyautogui.screenshot(output_file)

elif check_distro() == "Raspbian":

    def take_screenshot(output_file="screenshot.png"):
        grim_command = ["grim", output_file]
        subprocess.run(grim_command, capture_output=True, text=True)
