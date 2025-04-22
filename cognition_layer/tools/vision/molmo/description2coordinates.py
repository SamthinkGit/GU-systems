import re
import tempfile
import time
from typing import Literal

import replicate
from PIL import Image
from PIL import ImageDraw

from ecm.shared import get_logger

PointDict = dict[Literal["x", "y", "alt", "text"], float | str]

_logger = get_logger("Molmo")


def description2coordinates(image: Image, description: str) -> list[PointDict]:
    """Given an image and a description, return the coordinates of the point in the image using the MolMo model."""

    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as tmp:
        image.save(tmp.name)

        _logger.debug(
            f"Querying MolMo with image {tmp.name} and description '{description}'"
        )
        start = time.perf_counter()
        output = replicate.run(
            "zsxkib/molmo-7b:76ebd700864218a4ca97ac1ccff068be7222272859f9ea2ae1dd4ac073fa8de8",
            input={
                "text": f"Point to `{description}`",
                "image": open(tmp.name, "rb"),
                "top_k": 5,
                "top_p": 1,
                "temperature": 1,
                "length_penalty": 1,
                "max_new_tokens": 50,
            },
            wait=10,
        )
        end = time.perf_counter()
        _logger.debug(f"MolMo took {end - start:.2f} seconds to process the image")

    return parse_multiple_points(output)


def parse_multiple_points(text: str) -> list[PointDict]:
    """Parse the output of the MolMo model to extract multiple points."""
    pattern = r'<point x="([\d\.]+)" y="([\d\.]+)" alt="([^"]*)">(.+?)</point>'
    matches = re.findall(pattern, text)

    points = []
    for x, y, alt, body in matches:
        points.append({"x": float(x), "y": float(y), "alt": alt, "text": body})

    return points


def proportion2pixels(image: Image.Image, x_pct: float, y_pct: float) -> tuple[int, int]:
    """Convert a percentage of the image size to absolute pixel coordinates."""
    x_abs = int(x_pct / 100 * image.width)
    y_abs = int(y_pct / 100 * image.height)
    return (x_abs, y_abs)


def draw_point_on_image(image: Image.Image, x_abs: int, y_abs: int) -> Image.Image:
    """Draw a point on the image at the given percentage coordinates."""
    r = 5
    r_outside = 10
    result = image.copy().convert("L").convert("RGB")

    draw = ImageDraw.Draw(result)
    draw.ellipse((x_abs - r, y_abs - r, x_abs + r, y_abs + r), fill="red")
    draw.ellipse((x_abs - r_outside, y_abs - r_outside, x_abs + r_outside, y_abs + r_outside), outline="red", width=2)
    return result
