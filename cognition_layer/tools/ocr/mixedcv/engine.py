"""
MixedCV Engine
==============================
This module provides the MixedCV class, which extends the OcrEngine to
perform Optical Character Recognition (OCR) using a combination of text
detection and image analysis techniques. It integrates various tools
from the cognition_layer for processing images and extracting relevant
information.
"""
import cv2
import numpy as np
from PIL import Image

from cognition_layer.tools.ocr.mixedcv.core.analyse_image import (
    classic_cv_bbox_extraction,
)
from cognition_layer.tools.ocr.rapidocr.engine import RapidOCR as TextOCR
from cognition_layer.tools.ocr.template import BoundingBox
from cognition_layer.tools.ocr.template import contained
from cognition_layer.tools.ocr.template import OcrEngine
from ecm.shared import get_logger


class MixedCV(OcrEngine):
    """
    MixedCV class extending OcrEngine.

    This class combines text recognition and image analysis to extract
    relevant bounding boxes from images.
    """

    _logger = get_logger("MixedCV")

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.textocr = TextOCR()

    def invoke(self, image: Image.Image, *args, **kwargs) -> list[BoundingBox]:
        """
        Perform OCR processing on the given image.

        - This method orchestrates the OCR process, logging the start
          and completion of the operation. It retrieves text detections
          and analyzes the image for bounding box extraction, returning
          a list of valid detections.
        - :params image: The input image on which OCR is to be performed.
        - :params args: Additional positional arguments.
        - :params kwargs: Additional keyword arguments, including
          hyperparameters for bounding box extraction.
        - :return: A list of detected bounding boxes.
        """
        self._logger.debug("Starting MixedCV OCR processing")
        hyperparams = kwargs.get("hyperparams", {})

        img_np = np.array(image)
        opencv_image = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

        texts = self.textocr.invoke(image)
        for text in texts:
            text.additional_info["type"] = "text"

        icons = classic_cv_bbox_extraction(opencv_image, hyperparams)

        valid_icons = []
        for icon in icons:
            if any([contained(icon, text, tolerance=30) for text in texts]):
                continue

            valid_icons.append(icon)

        detections = texts + valid_icons
        self.storage["latest_detections"] = detections
        self._logger.debug(
            f"MixedCV OCR processing completed with {len(detections)} detections"
        )
        return detections

    def clean(self, *args, **kwargs):
        return
