"""
RapidOCR Engine Module
==============================
This module provides an implementation of the RapidOCR engine, which is
used for Optical Character Recognition (OCR) tasks. It leverages the
RapidOCR library to process images and extract text along with bounding
box information. The module also includes logging and debugging features
to track OCR operations and save detection results.
"""
# ======================= IMPORTS ============================
import time

from PIL import Image
from rapidocr import RapidOCR as _OCR

from cognition_layer.tools.ocr.template import BoundingBox
from cognition_layer.tools.ocr.template import OcrEngine
from ecm.shared import get_logger
from ecm.shared import get_root_path


# ======================= CLASSES ============================
class RapidOCR(OcrEngine):
    """
    Implementation of the RapidOCR engine.

    This class implements the `OcrEngine` base class and provides functionality
    to perform OCR on images using the RapidOCR library.
    """

    _logger = get_logger("RapidOCR")

    def __init__(self, *args, **kwargs):
        """
        Initializes the RapidOCR engine.

        Args:
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.
        """
        self.engine = _OCR()

    def invoke(self, image: Image.Image, *args, **kwargs) -> list[BoundingBox]:
        """
        Processes an image to perform OCR and extract text with bounding boxes.

        Args:
            image (Image.Image): The input image to process.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        Returns:
            list[BoundingBox]: A list of bounding boxes containing detected
            text and their corresponding positions.

        Warnings:
            RapidOCR may return mixed texts in the same bounding box.
            Ensure to handle such cases in the application logic.

        Examples:
            >>> from PIL import Image
            >>> image = Image.open("example.jpg")
            >>> ocr_engine = RapidOCR()
            >>> detections = ocr_engine.invoke(image)
            >>> for detection in detections:
            ...     print(detection.content, detection.center)
        """
        self._logger.debug("Running Engine")
        start = time.perf_counter()
        result = self.engine(image)
        end = time.perf_counter()
        self._logger.debug(f"OCR completed after {end - start:.2f}s")

        results: list[BoundingBox] = []

        debug_save_path = (
            get_root_path()
            / "cognition_layer"
            / "tools"
            / "ocr"
            / "rapidocr"
            / "ocr_debugging.jpg"
        )
        result.vis(str(debug_save_path.absolute()))
        self._logger.debug(f"Detections saved to {debug_save_path}")

        for text, box in zip(result.txts, result.boxes):
            upper_left, upper_right, bottom_right, bottom_left = box
            center = (
                int((upper_left[0] + upper_right[0]) / 2),
                int((upper_left[1] + bottom_left[1]) / 2),
            )
            results.append(
                BoundingBox(
                    top_left=upper_left,
                    top_right=upper_right,
                    bottom_left=bottom_left,
                    bottom_right=bottom_right,
                    center=center,
                    content=text,
                    additional_info={},
                )
            )

        self.storage["latest_detections"] = results
        self._logger.debug(f"{len(results)} entities detected")
        return results
