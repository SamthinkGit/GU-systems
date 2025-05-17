"""
Moondream API Actions
==============================
This module provides utility functions for interacting with the Moondream API,
which allows querying images with questions or locating points within images.
Moondream is a fast and cost-effective solution for image-based queries,
although its point detection quality is low. For higher-quality point detection,
it is recommended to use Molmo instead.

The module includes functions for retrieving the API key, querying images,
and processing screenshots.
"""
import os
from functools import cache
from typing import Optional

import moondream as md
from PIL.Image import Image

from action_space.tools.image import load_image
from cognition_layer.tools.ocr.image_edition import ImagePosition
from cognition_layer.tools.ocr.image_edition import partial_image
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

_logger = get_logger("MoonDreamAPI")


@cache
def get_moondream_api_key() -> str:
    """
    Retrieves the Moondream API key from environment variables.

    Raises:
        OSError: If the MOONDREAM_API_KEY environment variable is not set
        or is empty.

    warnings:
        This function is not using .dotenv to load environment variables.
        Ensure that the MOONDREAM_API_KEY is set in the environment before
        calling this function.

    Returns:
        str: The Moondream API key.
    """
    API_KEY: Optional[str] = os.getenv("MOONDREAM_API_KEY")
    if not API_KEY or API_KEY == "":
        raise OSError("MOONDREAM_API_KEY not found in environment variables.")
    return API_KEY


@ItemRegistry.require_dependencies("screenshot")
def query_screenshot(
    location: ImagePosition,
    question: str,
):
    """
    Queries a screenshot image with a question using the Moondream API.

    Args:
        location (ImagePosition): The position within the image to focus on.
        question (str): The question to ask about the image.

    Returns:
        str: The response from the Moondream API.
    """
    screenshot = load_image(ItemRegistry().get("screenshot", type="tool").content())
    screenshot = partial_image(screenshot, position=location)
    return query_image(screenshot, question)


def query_image(image: Image, question: str) -> str:
    """
    Queries an image with a question using the Moondream API.

    Args:
        image (Image): The image to query.
        question (str): The question to ask about the image.

    Returns:
        str: The response from the Moondream API.

    Examples:
        >>> from PIL import Image
        >>> image = Image.open("example.jpg")
        >>> response = query_image(image, "What is in this image?")
        >>> print(response)
        >>> # Output: "This is a cat."
    """
    api_key = get_moondream_api_key()
    model = md.vl(api_key=api_key)
    response: str = model.query(image, question=question)
    _logger.debug(f"Moondream response obtained: {response}")
    return response


def query_point(image: Image, query: str) -> tuple[int, int, bool]:
    """
    Locates a point in an image based on a query using the Moondream API.

    Args:
        image (Image): The image to analyze.
        query (str): The query to locate a point in the image.

    Returns:
        tuple[int, int, bool]: A tuple containing the x-coordinate,
        y-coordinate, and a confidence flag indicating whether the result
        is reliable.

    Raises:
        ValueError: If no points are found in the image.

    Warnings:
        The point detection quality is low. For better results, consider
        using Molmo instead.

    Examples:
        >>> x, y, confident = query_point(image, "Find the center of the object.")
        >>> print(f"Point found at ({x}, {y}), confident: {confident}")
        >>> # Output: Point found at (100, 200), confident: True
    """
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


def obtain_coordinates(query: str):
    image = load_image(ItemRegistry().get("screenshot", type="tool").content())

    api_key = get_moondream_api_key()
    model = md.vl(api_key=api_key)

    result = model.point(image, query)
    points = result["points"]
    print(f"Found {points}")

    result = []
    for point in points:
        x = point["x"] * image.width
        y = point["y"] * image.height
        result.append((int(x), int(y)))

    return "The following points were found: " + str(result)
