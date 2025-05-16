from action_space.tools.wrappers import throttle
from action_space.vision.moondream.request import query_screenshot
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "moondream_vision_actions"


@ItemRegistry.register(type="action", package=PKG_NAME)
def ask_to_image(question: str) -> str:
    """Use this tool for asking to an advanced Vision Model about what is on the screen.
    This tool will take a screenshot of the screen and ask the model about it.
    Example: ask_to_image("What app is on the screen?")
    Example: ask_to_image("Describe shortly this part of the image")
    Note: This tool is faster (and usually better than your own vision), so use it frequently, however it can return wrong results.
    Note: Entities must be described in English.
    """  # noqa

    return _ask_to_image("fullscreen", question)


@throttle(min_interval_seconds=10)
def _ask_to_image(*args, **kwargs) -> str:

    return query_screenshot(*args, **kwargs)
