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
from dataclasses import Field

from PIL import Image


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
    additional_info: dict = Field(default_factory=dict)


class OcrEngine(ABC):
    """
    An abstract base class for OCR engines that defines the required interface
    for implementing specific OCR algorithms.
    """

    def __init__(self, *args, **kwargs):
        """Initializes the OCR engine with optional parameters."""
        pass

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
