from typing import Literal

from action_space.mouse.molmo_based.word2click import _click_on_
from action_space.mouse.molmo_based.word2click import _double_click_on_
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "molmo_mouse_actions"


@ItemRegistry.register(type="action", package=PKG_NAME)
def click_on(
    prompt: str,
    location: Literal[
        "center",
        "top left",
        "top",
        "top right",
        "right",
        "bottom right",
        "bottom",
        "bottom left",
        "left",
        "fullscreen",
    ],
) -> str:
    """Clicks with the mouse on given a specified description and location.
    Example: click_on('Firefox fox alike Icon in the Windows bar', 'bottom')
    Example: click_on('Windows Explorer (a file yellow-like manager icon)', 'bottom')
    Note: Prompts must be made on english.
    Note: Please focus the screen where you want to click before executing this action.
    Note: Please ensure to describe the visuals if the element is an icon.
    Use me only when you are sure that the element is on the screen and which element you want to click.
    """

    return _click_on_(prompt, location)


@ItemRegistry.register(type="action", package=PKG_NAME)
def double_click_on(
    prompt: str,
    location: Literal[
        "center",
        "top left",
        "top",
        "top right",
        "right",
        "bottom right",
        "bottom",
        "bottom left",
        "left",
        "fullscreen",
    ],
) -> str:
    """Double clicks with the mouse on given a specified description and location.
    Note: Please focus the screen where you want to click before executing this action.
    Note: Please ensure to describe the visuals if the element is an icon.
    Note: Prompts must be made on english.
    Use me only when you are sure that the element is on the screen and which element you want to click.
    """
    return _double_click_on_(prompt, location)
