from action_space.vision.moondream.request import (
    obtain_coordinates as _obtain_coordinates,
)
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "moondream_element_finder"


@ItemRegistry.register(type="action", package=PKG_NAME)
def obtain_coordinates(query: str) -> str:
    """
    Use this tool to obtain the coordinates of a specific entity on the screen.
    This tool will return one or various matches with coordinates for the entity.
    """
    return _obtain_coordinates(query)
