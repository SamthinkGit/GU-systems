import os
from functools import cache
from typing import Literal
from typing import Optional

import moondream as md
from PIL.Image import Image

from action_space.tools.image import load_image
from cognition_layer.tools.ocr.image_edition import partial_image
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

_logger = get_logger("MoonDreamAPI")


@cache
def get_moondream_api_key() -> str:
    API_KEY: Optional[str] = os.getenv("MOONDREAM_API_KEY")
    if not API_KEY or API_KEY == "":
        raise OSError("MOONDREAM_API_KEY not found in environment variables.")
    return API_KEY


@ItemRegistry.require_dependencies("screenshot")
def query_screenshot(
    location: Literal[
        "center",
        "top left",
        "top",
        "top right",
        "right",
        "bottom right",
        "bottom",
        "bottom left",
        "left",
        "fullscreen",
    ],
    question: str,
):
    screenshot = load_image(ItemRegistry().get("screenshot", type="tool").content())
    screenshot = partial_image(screenshot, position=location)
    return query_image(screenshot, question)


def query_image(image: Image, question: str) -> str:
    api_key = get_moondream_api_key()
    model = md.vl(api_key=api_key)
    response: str = model.query(image, question=question)
    _logger.debug(f"Moondream response obtained: {response}")
    return response


def query_point(image: Image, query: str) -> tuple[int, int, bool]:
    api_key = get_moondream_api_key()
    model = md.vl(api_key=api_key)
    result = model.point(image, query)
    _logger.debug(f"Moondream point response obtained: {result}")
    points = result["points"]

    confident = True
    if len(points) > 1:
        confident = False

    if len(points) == 0:
        raise ValueError("No points found in the image.")

    x = points[0]["x"] * image.width
    y = points[0]["y"] * image.height
    return (int(x), int(y), confident)
