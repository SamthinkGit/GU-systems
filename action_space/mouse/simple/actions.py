from action_space.experimental.mouse.os_wrapper import click as osclick
from action_space.experimental.mouse.os_wrapper import move
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "mouse-simple"


@ItemRegistry.register(type="action", package=PKG_NAME)
def click(x: int, y: int):
    """
    Clicks with the mouse at the current position.
    Example: click(200, 50)
    """
    move(x, y)
    osclick()


@ItemRegistry.register(type="action", package=PKG_NAME)
def double_click(x: int, y: int):
    """
    DoubleClicks with the mouse at the current position.
    Example: double_click(925, 30)
    """
    move(x, y)
    osclick()
    osclick()
