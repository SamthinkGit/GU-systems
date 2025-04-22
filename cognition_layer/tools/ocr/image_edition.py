from typing import Literal

from PIL import Image


def partial_image(
    image: Image.Image,
    position: Literal[
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
) -> Image.Image:

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
    if not (0.1 <= percent <= 1):
        raise ValueError("Percentaje should be between 0.1 and 1")

    width, height = image.size
    new_width = int(width * percent)
    new_height = int(height * percent)

    resized_image = image.resize((new_width, new_height), Image.LANCZOS)
    return resized_image


def relative_coords_to_absolute(
    x: int,
    y: int,
    original_image: Image.Image,
    position: Literal[
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
) -> tuple[int, int]:

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
