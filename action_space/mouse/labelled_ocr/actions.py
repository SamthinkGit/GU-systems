from typing import Literal

from action_space.mouse.labelled_ocr.word2click import _click
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "labelled_ocr_mouse_actions"


@ItemRegistry.register(type="action", package=PKG_NAME)
def click(label: str, type: Literal["text", "icon"]) -> str:
    """Receives the id/label of the clickable and sends a click to that bounding box. Example: click('A22', 'text')"""  # noqa
    return _click(label, type)
