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
import numpy as np
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

    top_left: tuple[int, int]
    top_right: tuple[int, int]
    bottom_left: tuple[int, int]
    bottom_right: tuple[int, int]
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


def pil_to_cv2(pil_image: Image.Image) -> np.ndarray:
    """Translates a PIL image to a cv2 image."""
    img_np = np.array(pil_image)

    # Detectar si hay canal alfa
    if pil_image.mode == "RGBA":
        return cv2.cvtColor(img_np, cv2.COLOR_RGBA2BGRA)
    elif pil_image.mode == "RGB":
        return cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
    elif pil_image.mode == "L":  # escala de grises
        return img_np
    else:
        raise ValueError(f"Unsuported image type: {pil_image.mode}")


def cv2_to_pil(cv2_image: np.ndarray) -> Image.Image:
    """Translates a cv2 image to a PIL image."""
    if len(cv2_image.shape) == 2:  # imagen en escala de grises
        return Image.fromarray(cv2_image)
    elif cv2_image.shape[2] == 4:
        rgba_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGRA2RGBA)
        return Image.fromarray(rgba_image)
    elif cv2_image.shape[2] == 3:
        rgb_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb_image)
    else:
        raise ValueError("Unsupported image format: expected 3 or 4 channels")
