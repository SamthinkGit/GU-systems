"""
image_edition
==============================
This module provides utilities for editing images, including cropping
specific regions, resizing images, and converting relative coordinates
to absolute coordinates. It is designed to work with the Python Imaging
Library (PIL) and supports various predefined positions for cropping.
"""
from typing import Literal

from PIL import Image

# ======================= CONSTANTS ============================

ImagePosition = Literal[
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
]  # Predefined positions for cropping

# ======================= UTILITIES ============================


def partial_image(image: Image.Image, position: ImagePosition) -> Image.Image:
    """
    Crop a specific region of the image based on the given position.

    Args:
        image (Image.Image): The input image to crop.
        position (ImagePosition): The predefined position to crop. Supported
            values include "center", "top left", "top", "top right", "right",
            "bottom right", "bottom", "bottom left", "left", and "fullscreen".

    Returns:
        Image.Image: The cropped image.

    Raises:
        ValueError: If the provided position is not supported.

    Examples:
        >>> from PIL import Image
        >>> img = Image.open("example.jpg")
        >>> cropped_img = partial_image(img, "center")
    """
    width, height = image.size

    crops = {
        "left": (0.0, 0.0, 0.3, 1.0),
        "right": (0.7, 0.0, 1.0, 1.0),
        "top left": (0.0, 0.0, 0.4, 0.4),
        "top": (0.0, 0.0, 1.0, 0.4),
        "top right": (0.6, 0.0, 1.0, 0.4),
        "bottom right": (0.6, 0.6, 1.0, 1.0),
        "bottom": (0.0, 0.6, 1.0, 1.0),
        "bottom left": (0.0, 0.6, 0.4, 1.0),
        "center": (0.10, 0.10, 0.9, 0.9),
        "fullscreen": (0.0, 0.0, 1.0, 1.0),
    }

    if position not in crops:
        raise ValueError(f"Unsupported position: {position}")

    left_ratio, top_ratio, right_ratio, bottom_ratio = crops[position]

    left = int(width * left_ratio)
    top = int(height * top_ratio)
    right = int(width * right_ratio)
    bottom = int(height * bottom_ratio)

    return image.crop((left, top, right, bottom))


def resize_image(image: Image.Image, percent: int) -> Image.Image:
    """
    Resize the image by a given percentage.

    Args:
        image (Image.Image): The input image to resize.
        percent (int): The scaling factor as a percentage (0 < percent <= 1).

    Returns:
        Image.Image: The resized image.

    Raises:
        ValueError: If the percentage is not between 0 and 1.

    Examples:
        >>> from PIL import Image
        >>> img = Image.open("example.jpg")
        >>> resized_img = resize_image(img, 0.5)
    """
    if not (0 < percent <= 1):
        raise ValueError("Percentage should be between 0 and 1")

    width, height = image.size
    new_width = int(width * percent)
    new_height = int(height * percent)

    resized_image = image.resize((new_width, new_height), Image.LANCZOS)
    return resized_image


def relative_coords_to_absolute(
    x: int, y: int, original_image: Image.Image, position: ImagePosition
) -> tuple[int, int]:
    """
    Convert relative coordinates to absolute coordinates based on the
    cropped region of the image.

    Args:
        x (int): The x-coordinate relative to the cropped region.
        y (int): The y-coordinate relative to the cropped region.
        original_image (Image.Image): The original image.
        position (ImagePosition): The predefined position of the cropped
            region. Supported values include "center", "top left", "top",
            "top right", "right", "bottom right", "bottom", "bottom left",
            "left", and "fullscreen".

    Returns:
        tuple[int, int]: The absolute coordinates (x, y) in the original image.

    Raises:
        ValueError: If the provided position is not supported.

    Examples:
        >>> img = Image.open("example.jpg")
        >>> cropped_img = partial_image(img, "center")
        >>> abs_coords = relative_coords_to_absolute(0, 0, img, "center")
        >>> print(abs_coords)  # Output: (10, 10)
    """
    width, height = original_image.size

    crops = {
        "left": (0.0, 0.0, 0.3, 1.0),
        "right": (0.7, 0.0, 1.0, 1.0),
        "top left": (0.0, 0.0, 0.4, 0.4),
        "top": (0.0, 0.0, 1.0, 0.4),
        "top right": (0.6, 0.0, 1.0, 0.4),
        "bottom right": (0.6, 0.6, 1.0, 1.0),
        "bottom": (0.0, 0.6, 1.0, 1.0),
        "bottom left": (0.0, 0.6, 0.4, 1.0),
        "center": (0.10, 0.10, 0.9, 0.9),
        "fullscreen": (0.0, 0.0, 1.0, 1.0),
    }

    if position not in crops:
        raise ValueError(f"Unsupported position: {position}")

    left_ratio, top_ratio, right_ratio, bottom_ratio = crops[position]

    original_x = int(x + (left_ratio * width))
    original_y = int(y + (top_ratio * height))

    return original_x, original_y
