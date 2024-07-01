"""
Window Focus Utility
==============================

This module provides functionality to focus a window by its name using
the 'xdotool' command-line tool. It allows AI to bring a specified
window to the foreground programmatically.
"""
import subprocess

from ecm.tools.registry import ItemRegistry


@ItemRegistry.alias([
    'click_window',
    'focus_program'
])
@ItemRegistry.register_function
def focus_window(window_name: str):
    """Focus the selected window. Ensure to exactly match the name of the window. Usage: focus_window('notepad')""" # noqa
    try:
        # List all windows and filter by window name
        window_id = (
            subprocess.check_output(["xdotool", "search", "--name", window_name])
            .strip()
            .decode("utf-8")
        )

    except subprocess.CalledProcessError:
        raise Exception(f"Window with name '{window_name}' not found.")

    try:
        subprocess.run(["xdotool", "windowactivate", "--sync", window_id], check=True)
        print(f"Window with ID '{window_id}' has been focused.")
    except subprocess.CalledProcessError:
        raise Exception(f"Failed to focus window {window_name}: {window_id}.")


def main():

    try:
        focus_window(input("Window to Focus: "))
    except Exception as e:
        print(e)


if __name__ == "__main__":
    main()
