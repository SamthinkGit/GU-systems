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
