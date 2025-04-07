from action_space.experimental.mouse.os_wrapper import click as osclick
from action_space.experimental.mouse.os_wrapper import move
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "ostools"


@ItemRegistry.register(type="tool", package=PKG_NAME)
def move_mouse_to(x, y):
    move(x, y)


@ItemRegistry.register(type="tool", package=PKG_NAME)
def send_click_event():
    osclick()
