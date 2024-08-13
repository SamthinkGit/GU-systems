import uinput

from action_space.experimental.mouse.agent import MouseAgent
from action_space.experimental.mouse.calibrated_move import CALIBRATED_MOUSE_DEVICE
from action_space.experimental.mouse.calibrated_move import move
from ecm.shared import load_env  # noqa
from ecm.tools.registry import ItemRegistry

MOUSE_DEVICE = CALIBRATED_MOUSE_DEVICE


@ItemRegistry.register_util
def move_mouse_to(x, y):
    move(x, y)


@ItemRegistry.register_util
def send_click_event():
    MOUSE_DEVICE.emit(uinput.BTN_LEFT, 1)
    MOUSE_DEVICE.emit(uinput.BTN_LEFT, 0)


@ItemRegistry.register_function
def click(element: str):
    """Clicks with the mouse on the specified element. Example: click('Firefox Icon') or click('Navigation Bar'). Ensure to provide an especific description if needed."""  # noqa
    MouseAgent.find(element)
    ItemRegistry._utils["send_click_event"]()


# @ItemRegistry.register_function
# def focus(window: str):
#     """Focus an application by clicking on the center of the window. Use me only if the target tab is big and easily clickable, else use 'click' function. Example: focus('Word'). Ensure to provide an especific description if needed."""  # noqa
#     MouseAgent.find("The center of " + window, iterations=1)
#     ItemRegistry._utils["send_click_event"]()
