from functools import cache

from action_space.tools.image import load_image
from cognition_layer.tools.ocr.engine import TextOCR
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "simple-read-ocr"


@cache
def _get_ocr():
    return TextOCR()


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME)
def read_screen():
    """
    Parse all the texts and its coordinates from the screen using OCR.
    Use it when you need to read the screen.
    Use it also when you need to find a specific text/label in the screen.
    """
    registry = ItemRegistry("DatalabOCR")
    registry.autoload("screenshot")
    tool = registry.get("screenshot", type="tool")
    screenshot = load_image(tool.content())

    ocr = _get_ocr()
    results = ocr.invoke(screenshot)
    return str(results)
