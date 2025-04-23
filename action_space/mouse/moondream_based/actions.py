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
    Example: click_on('Outlook icon on the windows bottom bar')
    Exmple: click_on('miaw.png label in the square of the app')
    Example: click_on('Close window at the top right')
    Use me only when you are sure that the element is easily locable on the screen and which element you want to click.
    Note: This tool could return false positives
    """

    return _click_on(prompt)


@ItemRegistry.register(type="action", package=PKG_NAME)
def double_click_on(
    prompt: str,
) -> str:
    """Double Clicks with the mouse on given a specified description and location.
    Example: double_click_on('Outlook icon on the windows bottom bar')
    Exmple: double_click_on('miaw.png label in the square of the app')
    Example: double_click_on('Close window at the top right')
    Use me only when you are sure that the element is easily locable on the screen and which element you want to click.
    Note: This tool could return false positives, please verify the result.
    """

    result = _click_on(prompt)
    if "Succes" not in result:
        return result
    osclick()


@ItemRegistry.require_dependencies("screenshot")
def _click_on(prompt):
    screenshot = load_image(ItemRegistry().get("screenshot", type="tool").content())
    try:
        x, y, confident = query_point(screenshot, prompt)
    except ValueError:
        return f"Moondream could not find any point for the prompt '{prompt}'."
    osmove(x, y)
    osclick()
    if not confident:
        return "Succesfully clicked, but multiple results have been matched, please verify the result."

    return "Success"
