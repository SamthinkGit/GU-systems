"""
detect_salient_regions
==============================
This module provides functionality for detecting salient regions in an image
using the OpenCV library. It utilizes saliency detection techniques to identify
regions of interest based on their visual prominence.
"""
import cv2

from cognition_layer.tools.ocr.mixedcv.core.combinator import TupledBoundingBox


# ======================= FUNCTIONS ============================
def detect_salient_regions(
    image,
    saliency_thresh: int = 200,
    min_width: int = 10,
    min_height: int = 10,
    max_width: int = 200,
    max_height: int = 200,
) -> list[TupledBoundingBox]:
    """
    - Detects salient regions in the provided image based on saliency
      thresholds and bounding box dimensions.
    - :params:
        - image: The input image in which salient regions are to be detected.
        - saliency_thresh: Threshold to determine salient regions.
        - min_width: Minimum width of the bounding boxes.
        - min_height: Minimum height of the bounding boxes.
        - max_width: Maximum width of the bounding boxes.
        - max_height: Maximum height of the bounding boxes.
    - :return: A list of TupledBoundingBox tuples representing detected regions.
    """

    saliency = cv2.saliency.StaticSaliencyFineGrained_create()
    success, saliencyMap = saliency.computeSaliency(image)
    if not success:
        raise RuntimeError("Saliency computation failed")

    saliencyMap = (saliencyMap * 255).astype("uint8")
    _, thresh = cv2.threshold(saliencyMap, saliency_thresh, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    boxes: list[TupledBoundingBox] = []
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)
        if w >= min_width and h >= min_height and w <= max_width and h <= max_height:
            boxes.append((x, y, w, h))

    return boxes
