import pyautogui
import time

DEFAULT_DELAY = 0.012


def write(text: str):
    pyautogui.write(text, interval=DEFAULT_DELAY)


def press_keys(keys: list[str]):
    pyautogui.hotkey(*keys)


if __name__ == "__main__":
    time.sleep(1)
    press_keys(["altleft", "tab"])
