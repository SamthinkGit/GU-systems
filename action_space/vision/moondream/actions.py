from action_space.tools.wrappers import throttle
from action_space.vision.moondream.request import query_screenshot
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "moondream_vision_actions"


@ItemRegistry.register(type="action", package=PKG_NAME)
def describe_screen() -> str:
    """Use this tool to obtain an advanced Vision Model description about what is on the screen.
    This tool will take a screenshot of the screen and ask the model about it.
    Please note the model sometimes may be wrong or hallucinate.
    """  # noqa

    return _ask_to_image("fullscreen", "Describe this image screen.")


@throttle(min_interval_seconds=10)
def _ask_to_image(*args, **kwargs) -> str:

    return query_screenshot(*args, **kwargs)
