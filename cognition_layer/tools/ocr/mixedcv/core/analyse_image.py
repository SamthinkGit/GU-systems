# flake8: noqa
"""
classic_cv_bbox_extraction
==============================
This module contains functions for extracting bounding boxes from images
using classic computer vision techniques, including contour detection and
saliency detection. The main function, `classic_cv_bbox_extraction`,
combines these methods and utilizes configurations from a YAML file to
fine-tune the detection parameters.
"""
import yaml

from cognition_layer.tools.ocr.mixedcv.core.combinator import merge_boxes
from cognition_layer.tools.ocr.mixedcv.methods.countours import detect_contour_regions
from cognition_layer.tools.ocr.mixedcv.methods.saliency import detect_salient_regions
from cognition_layer.tools.ocr.mixedcv.methods.sharp import sharp_image
from cognition_layer.tools.ocr.template import BoundingBox
from ecm.shared import default_kwargs
from ecm.shared import get_root_path


# ======================= FUNCTIONS ============================
def classic_cv_bbox_extraction(image, hyperparams: dict) -> list[BoundingBox]:
    """
    - Extracts bounding boxes from an image using classic computer vision
      techniques.
    - :params:
        - image: The input image from which to extract bounding boxes.
        - hyperparams: Dictionary containing hyperparameters for bounding
          box extraction.
    - :return: A list of BoundingBox objects representing the detected
      bounding boxes.
    - Warnings: Ensure the input image is in the correct format for
      processing.
    """
    with open(get_root_path() / "cognition_layer/tools/ocr/mixedcv/params.yaml") as f:
        defaults = yaml.safe_load(f)
    hyperparams = default_kwargs(hyperparams, defaults["hyperparams"])

    sharpen = hyperparams["sharpen"]
    sal_thresh = hyperparams["sal_thresh"]
    min_size = hyperparams["min_size"]
    max_size = hyperparams["max_size"]
    big_num = hyperparams["big_num"]
    big_blur = hyperparams["big_blur"]
    iou_thresh = hyperparams["nms_iou"]

    sharpened_image = sharp_image(image, k=sharpen)

    # Mixing contours and saliency detection methods
    detections = [
        # Countour
        detect_contour_regions(image, num_contours=big_num, blur=big_blur),
        # Saliency
        detect_salient_regions(
            image,
            saliency_thresh=sal_thresh,
            min_width=min_size,
            min_height=min_size,
            max_width=max_size,
            max_height=max_size,
        ),
        # Saliency + Sharpen
        detect_salient_regions(
            sharpened_image,
            saliency_thresh=sal_thresh,
            min_width=min_size,
            min_height=min_size,
            max_width=max_size,
            max_height=max_size,
        ),
    ]

    # Merge boxes from all detections
    boxes = []
    for box_list in detections:
        boxes.extend(box_list)
    boxes = merge_boxes(boxes, iou_thresh)

    # Convert TupledBoundingBox to BoundingBox objects
    bnding_boxes = []
    for box in boxes:
        x, y, w, h = box
        bnding_boxes.append(
            BoundingBox(
                top_left=(x, y),
                top_right=(x + w, y),
                bottom_left=(x, y + h),
                bottom_right=(x + w, y + h),
                center=((x + x + w) // 2, (y + y + h) // 2),
                content="",
                additional_info={"type": "icon"},
            )
        )

    return bnding_boxes
