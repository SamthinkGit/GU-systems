from action_space.tools.wrappers import listable
from ecm.tools.get_platform import osmethod
from ecm.tools.item_registry_v2 import ItemRegistry


@ItemRegistry.alias(["hotkey", "press"])
@ItemRegistry.register(type="action")
def press_keys(keys: list[str]):
    """Presses the given keys as one combination. Note: It is vital to specify the side of the key if there are multiple, e.g. ['LEFTSHIFT', 'RIGHTCTRL', ...] # noqa
    @param key: A simple list of key strings, e.g. ['LEFTCTRL', 'F4', 'W'] or ['WIN', 'D'].
    """
    _press_keys(keys)


@osmethod
def _press_keys(os, keys):
    if os == "Windows":
        from action_space.keyboard.os_dependant.windows_keyboard import press_keys as f
    else:
        from action_space.keyboard.os_dependant.virtual_keyboard import press_keys as f

    f(keys)


@ItemRegistry.alias(["type"])
@ItemRegistry.register(type="action")
def write(text: str):
    """Translates ASCII text to keystrokes and sends them as events.
    Note: Use press_keys for sending enter, here only send text as a string.
    @param text: string as a unique string text to send as events to keyboard, e.g. write('Hello World!')
    """
    _write(text)


@osmethod
@listable
def _write(os, text):
    if os == "Windows":
        from action_space.keyboard.os_dependant.windows_keyboard import write as f
    else:
        from action_space.keyboard.os_dependant.virtual_keyboard import write as f

    f(text)
