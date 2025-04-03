from action_space.experimental.mouse.os_wrapper import click as osclick
from action_space.experimental.mouse.os_wrapper import move as osmove
from action_space.tools.wrappers import listable
from cognition_layer.tools.nlp.simple_postprocessing import match
from cognition_layer.tools.ocr.template import BoundingBox
from ecm.tools.item_registry_v2 import Storage


@listable
def _click(target: str):
    """Receives an phrase and tries to click a matched element in the screen"""  # noqa
    ocr = Storage(name="OCR")
    elements: list[BoundingBox] = ocr.get("latest_detections", [])

    if len(elements) == 0:
        raise ValueError("No elements detected. Please run OCR first.")

    matches = []
    for word in target.split():
        matches.append(match(word, elements, key=lambda element: element.content))

    for element, match_type in matches:
        element: BoundingBox
        if match_type == "exact":
            osmove(element.center[0], element.center[1])
            osclick()
            return "Successfully clicked on the element."

    for element, match_type in matches:
        element: BoundingBox
        if match_type == "partial":
            osmove(element.center[0], element.center[1])
            osclick()
            return "Successfully clicked on the element."

    return "Element not found."
