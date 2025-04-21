from functools import cache

from PIL.Image import Image
from transformers import pipeline

from action_space.experimental.mouse.os_wrapper import click as osclick
from action_space.experimental.mouse.os_wrapper import move as osmove
from cognition_layer.tools.ocr.labeler import Labeler
from cognition_layer.tools.ocr.template import BoundingBox


@cache
def get_icon_detection_pipeline():
    return pipeline(
        task="zero-shot-image-classification",
        model="openai/clip-vit-base-patch32",
        use_fast=True,
    )


def _click_on_icon(description_keywords: list[str]):

    punctuations = _get_icon_punctuations(description_keywords)
    bbox = max(punctuations, key=punctuations[0])

    osmove(bbox.center[0], bbox.center[1])
    osclick()
    return "Successfully clicked on the element."


def _get_icon_punctuations(
    description_keywords: list[str], show_debug: bool = False
) -> tuple[float, str, BoundingBox, Image]:
    labeler = Labeler.get_freezed_instance()
    classifier = get_icon_detection_pipeline()
    punctuations = []

    labelled_crops = labeler.get_labelled_crops(
        filter=lambda bbox: bbox.additional_info["type"] == "icon"
    )
    for label, (bbox, icon) in labelled_crops.items():

        result = classifier(
            icon,
            candidate_labels=[*description_keywords, "other"],
        )
        score = sum(label["score"] for label in result[: len(description_keywords)])
        if show_debug:
            print(f"Label {label}: {score}")

        punctuations.append((score, label, bbox, icon))

    return punctuations
