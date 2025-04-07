from action_space.mouse.ocr_based.word2click import _click
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "ocr_mouse_actions"


@ItemRegistry.register(type="action", package=PKG_NAME)
def click(element: str) -> str:
    """Clicks with the mouse on the specified label. Example: click('Firefox Icon') or click('Navigation Bar')"""
    return _click(element)
