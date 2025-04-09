"""
Ocr Module
==============================
This module contains the implementation of an Optical Character Recognition (OCR)
engine using abstract base classes and data classes. It provides a structure for
defining bounding boxes around detected text in images and an abstract engine
that must be implemented by specific OCR algorithms.
"""
from abc import ABC
from abc import abstractmethod
from dataclasses import dataclass
from dataclasses import field

import cv2
from PIL import Image

from ecm.tools.item_registry_v2 import Storage


# ======================= CLASSES ============================


@dataclass
class BoundingBox:
    """
    Represents a bounding box around detected text in an image, defined by its
    corners and center position, along with associated content and additional
    information.
    """

    top_left: int
    top_right: int
    bottom_left: int
    bottom_right: int
    center: tuple[int, int]

    content: str
    additional_info: dict = field(default_factory=dict)


class OcrEngine(ABC):
    """
    An abstract base class for OCR engines that defines the required interface
    for implementing specific OCR algorithms.
    """

    storage: Storage = Storage(name="OCR")  # Contains only the latest detections

    def __init__(self, *args, **kwargs):
        """Initializes the OCR engine with optional parameters."""
        latest_detections: list[BoundingBox] = []
        self.storage["latest_detections"] = latest_detections

    @abstractmethod
    def invoke(self, image: Image.Image, *args, **kwargs) -> list[BoundingBox]:
        """
        - Invokes the OCR processing on the given image and returns a list
          of bounding boxes around detected text.
        - :params image: The image to process.
        - :return: A list of BoundingBox instances containing detected text
          information.
        - :raises NotImplementedError: If not implemented in a subclass.
        """
        raise NotImplementedError()

    def clean(self, *args, **kwargs):
        """
        - Cleans or resets the OCR engine state, if needed.
        """
        pass


def contained(bbox_a: BoundingBox, bbox_b: BoundingBox, tolerance=0):
    """
    Check if bounding box A is contained within bounding box B with a given tolerance.
    """
    return all(
        (bbox_b.top_left[0] - tolerance) <= p[0] <= (bbox_b.top_right[0] + tolerance)
        and (bbox_b.top_left[1] - tolerance)
        <= p[1]
        <= (bbox_b.bottom_left[1] + tolerance)
        for p in [
            bbox_a.top_left,
            bbox_a.top_right,
            bbox_a.bottom_left,
            bbox_a.bottom_right,
        ]
    )


def draw_bounding_boxes(
    image: cv2.Mat, boxes: list[BoundingBox], thickness=1, color=(255, 255, 255)
):
    """
    Draws bounding boxes on the image.
    Args:
        imagen: The image on which to draw the bounding boxes.
        boxes: A list of BoundingBox objects to draw.
        thickness: Thickness of the lines used to draw the boxes.
        color: Color of the lines used to draw the boxes.
    Returns:
        The image with bounding boxes drawn on it.
    """
    img_out = image.copy()

    for box in boxes:
        pts = [box.top_left, box.top_right, box.bottom_right, box.bottom_left]
        pts = [(int(p[0]), int(p[1])) for p in pts]

        for i in range(4):
            cv2.line(img_out, pts[i], pts[(i + 1) % 4], color, thickness)

    return img_out
