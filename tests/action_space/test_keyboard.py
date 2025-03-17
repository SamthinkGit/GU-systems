import action_space.keyboard.actions  # noqa
import time
from ecm.tools.item_registry_v2 import ItemRegistry
from tests.action_space.tools.screen_wrapper import Tracker


def test_writting(tracker: Tracker):
    registry = ItemRegistry(name="test_writting")
    registry.load_all()

    write = registry.actions["write"].content

    for text in ["Hello World", "Im a text", "I'm a text"]:
        del tracker.keys[:]
        write(text)
        time.sleep(0.1)
        keys_pressed = "".join(
            [key.key for key in tracker.keys if "shift" not in key.key.lower()]
        )
        keys_pressed = keys_pressed.replace("space", " ")
        keys_pressed = keys_pressed.replace("apostrophe", "'")
        assert keys_pressed == text, f"Expected {text} and received {keys_pressed}"


def test_using_enter(tracker: Tracker):
    registry = ItemRegistry(name="test_using_enter")
    registry.load_all()

    write = registry.actions["write"].content

    del tracker.keys[:]
    text = "test\ntest\n\n"
    write(text)
    time.sleep(0.1)

    keys_pressed = "".join([key.key for key in tracker.keys])
    keys_pressed = keys_pressed.replace("Return", "\n")
    assert keys_pressed == text, f"Expected {text} and received {keys_pressed}"


def test_keybindings(tracker: Tracker):
    registry = ItemRegistry(name="test_keybindings")
    registry.load_all()

    press_keys = registry.actions["press_keys"].content

    for bind, solution in [
        (["LEFTCTRL"], ["Control_L"]),
        (["LEFTCTRL", "a"], ["Control_La", "aControl_L"]),
        (["CTRL", "x"], ["Control_Lx", "xControl_L"]),
        (["SHIFT", "x"], ["Shift_LX", "XShift_L"]),
        (["ALT", "a"], ["Alt_La", "aAlt_L"]),
    ]:
        del tracker.keys[:]
        press_keys(bind)
        time.sleep(0.1)
        keys_pressed = "".join([key.key for key in tracker.keys])
        assert (
            keys_pressed in solution
        ), f"Expected {solution} and received {keys_pressed}"


def test_alt_tab(tracker: Tracker):
    del tracker.keys[:]
    registry = ItemRegistry(name="test_alt_tab")
    registry.load_all()

    press_keys = registry.actions["press_keys"].content
    press_keys(["ALT", "TAB"])
    time.sleep(0.5)

    press_keys(["ALT", "TAB"])
    time.sleep(0.5)

    press_keys(["ENTER"])
    time.sleep(0.2)
    assert len(tracker.keys) == 1
    assert tracker.keys[-1].key == "Return"
