from action_space.experimental.mouse.os_wrapper import click as osclick
from action_space.experimental.mouse.os_wrapper import move
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "mouse-simple"


@ItemRegistry.register(type="action", package=PKG_NAME)
def move_mouse_to(x: int, y: int):
    """
    Moves the mouse to the specified coordinates.
    Example: move_mouse_to(100, 200)
    """
    move(x, y)


@ItemRegistry.register(type="action", package=PKG_NAME)
def click():
    """
    Clicks with the mouse at the current position.
    Example: click()
    """
    osclick()


@ItemRegistry.register(type="action", package=PKG_NAME)
def double_click():
    """
    DoubleClicks with the mouse at the current position.
    Example: double_click()
    """
    osclick()
    osclick()
