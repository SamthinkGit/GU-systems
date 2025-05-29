import pickle
from functools import cache

from action_space.tools.image import load_image
from action_space.tools.wrappers import throttle
from cognition_layer.tools.ocr.datalab.extract_bboxes import animate_bboxes
from cognition_layer.tools.ocr.engine import TextOCR
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "simple-read-ocr"


@cache
def _get_ocr():
    return TextOCR()


# ========================= ACTIONS ======================
@ItemRegistry.register(type="action", package=PKG_NAME, labels=["enforce-host"])
def read_screen():
    """
    IMPORTANT: Do not overuse this action/tool, extract the features you need and save
    them in your memory after this action.

    Parse all the texts and its coordinates from the screen using OCR.
    Use it when you need to read the screen or click in text/labels/buttons that contain texts.
    Use it also when you need to find a specific text/label in the screen.
    You can also use it find where to click or where to move the mouse.
    """
    return _read_screen()


@throttle(min_interval_seconds=15)
def _read_screen():
    # Obtain image on the client
    registry = ItemRegistry()
    tool = registry.get("screenshot", type="tool")
    screenshot = load_image(tool.content())

    # Perform OCR on the screenshot (on the host)
    ocr = _get_ocr()
    results = ocr.invoke(screenshot)
    response = [
        f"Text at coordinates {box.center}: ```{box.content}```" for box in results
    ]

    # Animate the bounding boxes on the client
    animation = registry.get("_animate_ocr", type="tool").content
    boxes_bytes = pickle.dumps(results)
    animation(boxes_bytes)

    return "\n".join(response) + "You can use the mouse with the given coordinates."


@ItemRegistry.register(type="tool", package=PKG_NAME)
def _animate_ocr(boxes_bytes: str):
    boxes = pickle.loads(boxes_bytes)
    animate_bboxes(boxes, animation_duration=2000)
