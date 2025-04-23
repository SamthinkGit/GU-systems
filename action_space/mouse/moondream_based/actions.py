from action_space.experimental.mouse.os_wrapper import click as osclick
from action_space.experimental.mouse.os_wrapper import move as osmove
from action_space.tools.image import load_image
from action_space.vision.moondream.request import query_point
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "moondream_mouse_actions"


@ItemRegistry.register(type="action", package=PKG_NAME)
def click_on(
    prompt: str,
) -> str:
    """Clicks with the mouse on given a specified description and location.
    Example: click_on('Outlook')
    Exmple: click_on('miaw.png label')
    Example: click_on('Close window')
    Use me only when you are sure that the element is easily locable on the screen and which element you want to click.
    """

    return _click_on(prompt)


@ItemRegistry.register(type="action", package=PKG_NAME)
def double_click_on(
    prompt: str,
) -> str:
    """Double Clicks with the mouse on given a specified description and location.
    Example: double_click_on('Outlook')
    Exmple: double_click_on('miaw.png label')
    Example: double_click_on('Close window')
    Use me only when you are sure that the element is easily locable on the screen and which element you want to click.
    """

    result = _click_on(prompt)
    if result != "Success":
        return result
    osclick()


@ItemRegistry.require_dependencies("screenshot")
def _click_on(prompt):
    screenshot = load_image(ItemRegistry().get("screenshot", type="tool").content())
    try:
        x, y = query_point(screenshot, prompt)
    except ValueError:
        return f"Moondream could not find any point for the prompt '{prompt}'."
    osmove(x, y)
    osclick()
    return "Success"
