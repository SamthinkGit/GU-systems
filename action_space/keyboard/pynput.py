import time

from pynput.keyboard import Controller
from pynput.keyboard import Key

from ecm.tools.registry import ItemRegistry

keyboard = Controller()


@ItemRegistry.register_function
def write(text: str):
    """Writes the given text. If you are writing a text/sentence or some word/s, should use this function. This write does not include Enter. Usage: write('Hello, world!')"""  # noqa
    for char in text:
        keyboard.type(char)
        time.sleep(0.03)


@ItemRegistry.register_function
def press(key: str):
    """Presses the specified key. Usage: press('enter')"""
    key = key.lower()
    match key:
        case "enter":
            keyboard.press(Key.enter)
            keyboard.release(Key.enter)
        case "backspace":
            keyboard.press(Key.backspace)
            keyboard.release(Key.backspace)
        case "space":
            keyboard.press(Key.space)
            keyboard.release(Key.space)
        case "tab":
            keyboard.press(Key.tab)
            keyboard.release(Key.tab)
        case "esc":
            keyboard.press(Key.esc)
            keyboard.release(Key.esc)
        case "up":
            keyboard.press(Key.up)
            keyboard.release(Key.up)
        case "down":
            keyboard.press(Key.down)
            keyboard.release(Key.down)
        case "left":
            keyboard.press(Key.left)
            keyboard.release(Key.left)
        case "right":
            keyboard.press(Key.right)
            keyboard.release(Key.right)
        case _:
            keyboard.press(key)
            keyboard.release(key)


@ItemRegistry.register_function
def hotkey(*keys: str):
    """Presses a combination of keys. Usage: hotkey('ctrl', 's')"""
    for key in keys:
        if key == "ctrl":
            keyboard.press(Key.ctrl)
        elif key == "shift":
            keyboard.press(Key.shift)
        elif key == "alt":
            keyboard.press(Key.alt)
        elif key == "cmd" or key == "command":
            keyboard.press(Key.cmd)
        else:
            keyboard.press(key)


@ItemRegistry.register_function
def type(text: str):
    """Types the given text. Does not include enter. Use me when typing a word or text. Usage: type('Hello, world!')"""
    write(text)


@ItemRegistry.register_function
def type_text(text: str):
    """Types the given text. Does not include enter. Use me when typing a word or text. Usage: type_text('Hello, world!')""" # noqa
    write(text)


@ItemRegistry.register_function
def press_key(text: str):
    """Presses the specified key. Usage: press_key('enter')"""
    press(text)
