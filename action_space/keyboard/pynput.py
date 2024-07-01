"""
PynPut
==============================
This module contains a set of instructions that are easily used by an AI for
managing the keyboard.
"""
import time

from pynput.keyboard import Controller
from pynput.keyboard import Key

from action_space.keyboard.names import KeyNames
from ecm.tools.registry import ItemRegistry

keyboard = Controller()


@ItemRegistry.alias(
    [
        "type",
        "type_text",
        "write_text",
    ]
)
@ItemRegistry.register_function
def write(text: str):
    """Writes the given text. If you are writing a text/sentence or some word/s, should use this function. This write does not include Enter. Usage: write('Hello, world!')"""  # noqa
    for char in text:
        keyboard.type(char)
        time.sleep(0.03)


@ItemRegistry.alias(["press_key", "key"])
@ItemRegistry.register_function
def press(key: str):
    """Presses the specified key. Usage: press('enter')"""
    key = key.lower()
    keyboard.tap(eval(f"Key.{key}", globals()))


@ItemRegistry.register_function
def hotkey(*keys: str):
    """Presses a combination of keys. Usage: hotkey('ctrl', 's')"""
    if isinstance(keys[0], list) or isinstance(keys[0], tuple):
        keys = keys[0]

    keys_pressed: list[str | Key] = []
    for key in keys:
        key = _parse_key(key)
        keys_pressed.append(key)
        keyboard.press(key)

    for key in keys_pressed:
        keyboard.release(key)


def _parse_key(key: str) -> str | Key:
    """Parses a special key into Pynput Key type. If not found returns the same str"""
    key = key.lower()
    match key:
        case _ if key in KeyNames.CTRL:
            return Key.ctrl
        case _ if key in KeyNames.SHIFT:
            return Key.shift
        case _ if key in KeyNames.ALT:
            return Key.alt
        case _ if key in KeyNames.ESC:
            return Key.esc
        case _ if key in KeyNames.TAB:
            return Key.tab
        case _ if key in KeyNames.HOME:
            return Key.home
        case _ if key in KeyNames.END:
            return Key.end
        case _ if key in KeyNames.BACKSPACE:
            return Key.backspace
        case _ if key in KeyNames.DELETE:
            return Key.delete
        case _:
            try:
                return eval(f"Key.{key}")
            except Exception:
                return key
