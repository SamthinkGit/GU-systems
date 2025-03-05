import time

from action_space.experimental.mouse.agent import MouseAgent
from action_space.experimental.mouse.os_wrapper import move, click as osclick
from ecm.shared import load_env  # noqa
from ecm.tools.item_registry_v2 import ItemRegistry
from action_space.tools.wrappers import listable


@ItemRegistry.register_util
def move_mouse_to(x, y):
    move(x, y)


@ItemRegistry.register_util
def send_click_event():
    osclick()


@ItemRegistry.register_function
def click(element: str):
    """Clicks with the mouse on the specified element. Example: click('Firefox Icon') or click('Navigation Bar'). Ensure to provide an especific description if needed."""  # noqa
    _click(element)


@listable
def _click(element):
    MouseAgent.find(element)
    ItemRegistry._utils["send_click_event"]()
    time.sleep(2)


# @ItemRegistry.register_function
# def focus(window: str):
#     """Focus an application by clicking on the center of the window. Use me only if the target tab is big and easily clickable, else use 'click' function. Example: focus('Word'). Ensure to provide an especific description if needed."""  # noqa
#     MouseAgent.find("The center of " + window, iterations=1)
#     ItemRegistry._utils["send_click_event"]()
