from typing import Literal

from action_space.experimental.mouse.os_wrapper import click as osclick
from action_space.experimental.mouse.os_wrapper import move as osmove
from cognition_layer.tools.ocr.labeler import Labeler
from cognition_layer.tools.ocr.template import BoundingBox
from ecm.tools.item_registry_v2 import Storage


def _click(label: str, type: Literal["text", "icon"]) -> str:
    """Receives the id of the clickable and sends a click to that bounding box"""  # noqa

    ocr = Storage(name="OCR")
    labeler = Labeler.get_freezed_instance()

    elements: list[BoundingBox] = ocr.get("latest_detections", [])

    if len(elements) == 0:
        return "No elements detected. Please run OCR first."

    bbox = labeler.labelled_boxes.get(label, None)

    if bbox is None:
        return "Label not found."
    if type != bbox.additional_info.get("type", "null"):
        return "Label type mismatch."

    osmove(bbox.center[0], bbox.center[1])
    osclick()
    return "Successfully clicked on the element."
