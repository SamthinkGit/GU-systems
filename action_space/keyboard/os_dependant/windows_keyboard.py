import time

import pyautogui

DEFAULT_DELAY = 0.012


def write(text: str):
    pyautogui.write(text, interval=DEFAULT_DELAY)


def press_keys(keys: list[str]):
    keys = [key.lower() for key in keys]
    new_keys = []

    # Adaptation from keys in UINPUT
    for key in keys:
        for bind in ["left", "right"]:
            if bind in key:
                key = key.removeprefix(bind)
                key += bind
        new_keys.append(key)

    print("Pressing", new_keys)
    pyautogui.hotkey(*new_keys)


if __name__ == "__main__":
    time.sleep(1)
    press_keys(["altleft", "tab"])
