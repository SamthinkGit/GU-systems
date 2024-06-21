import time

from pynput.keyboard import Controller
from pynput.keyboard import Key

from ecm.tools.registry import ItemRegistry

keyboard = Controller()


@ItemRegistry.register_function
def write(text: str):
    """Writes the given text. If you are writing a text/sentence or some word/s, should use this function. Usage: write('Hello, world!')"""  # noqa
    for char in text:
        keyboard.type(char)
        time.sleep(0.03)


@ItemRegistry.register_function
def press(key: str):
    """Presses the specified key. Usage: press('enter')"""
    match key:
        case "enter":
            keyboard.press(Key.enter)
        case "backspace":
            keyboard.press(Key.backspace)
        case "space":
            keyboard.press(Key.space)
        case "tab":
            keyboard.press(Key.tab)
        case "esc":
            keyboard.press(Key.esc)
        case "up":
            keyboard.press(Key.up)
        case "down":
            keyboard.press(Key.down)
        case "left":
            keyboard.press(Key.left)
        case "right":
            keyboard.press(Key.right)
        case _:
            keyboard.press(key)


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
    """Types the given text. Use me when typing a word or text. Usage: type('Hello, world!')"""
    write(text)


@ItemRegistry.register_function
def type_text(text: str):
    """Types the given text. Use me when typing a word or text. Usage: type_text('Hello, world!')"""
    write(text)


@ItemRegistry.register_function
def press_key(text: str):
    """Presses the specified key. Usage: press_key('enter')"""
    write(text)


@ItemRegistry.register_function
def enter():
    """Presses the Enter key. Usage: enter()"""
    keyboard.press(Key.enter)


@ItemRegistry.register_function
def backspace():
    """Presses the Backspace key. Usage: backspace()"""
    keyboard.press(Key.backspace)


@ItemRegistry.register_function
def space():
    """Presses the Space key. Usage: space()"""
    keyboard.press(Key.space)


@ItemRegistry.register_function
def tab():
    """Presses the Tab key. Usage: tab()"""
    keyboard.press(Key.tab)


@ItemRegistry.register_function
def escape():
    """Presses the Escape key. Usage: escape()"""
    keyboard.press(Key.esc)


@ItemRegistry.register_function
def arrow_up():
    """Presses the Up Arrow key. Usage: arrow_up()"""
    keyboard.press(Key.up)


@ItemRegistry.register_function
def arrow_down():
    """Presses the Down Arrow key. Usage: arrow_down()"""
    keyboard.press(Key.down)


@ItemRegistry.register_function
def arrow_left():
    """Presses the Left Arrow key. Usage: arrow_left()"""
    keyboard.press(Key.left)


@ItemRegistry.register_function
def arrow_right():
    """Presses the Right Arrow key. Usage: arrow_right()"""
    keyboard.press(Key.right)
