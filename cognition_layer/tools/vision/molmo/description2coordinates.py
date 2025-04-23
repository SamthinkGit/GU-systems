import base64
import os
import re
import time
from io import BytesIO
from typing import Literal

import requests
from PIL import Image
from PIL import ImageDraw

from ecm.shared import get_logger

PointDict = dict[Literal["x", "y", "alt", "text"], float | str]

_logger = get_logger("Molmo")


def description2coordinates(image: Image, description: str) -> list[PointDict]:
    """Given an image and a description, return the coordinates of the point in the image using the MolMo model."""

    buffered = BytesIO()

    image.save(buffered, format="PNG")
    image_base64 = base64.b64encode(buffered.getvalue()).decode("utf-8")
    image_data_url = f"data:image/png;base64,{image_base64}"

    _logger.debug(f"Querying MolMo with description '{description}'")

    # Prepare headers and payload
    api_token = os.getenv("REPLICATE_API_TOKEN")
    headers = {
        "Authorization": f"Bearer {api_token}",
        "Content-Type": "application/json",
        "Prefer": "wait=10",
    }

    payload = {
        "version": "76ebd700864218a4ca97ac1ccff068be7222272859f9ea2ae1dd4ac073fa8de8",
        "input": {
            "text": f"Find one point to {description}",
            "image": image_data_url,
            "top_k": 5,
            "top_p": 1,
            "temperature": 0.6,
            "length_penalty": 1,
            "max_new_tokens": 100,
        },
    }

    # Send POST request
    start = time.perf_counter()
    response = requests.post(
        "https://api.replicate.com/v1/predictions",
        json=payload,
        headers=headers,
        timeout=20,
    )
    end = time.perf_counter()

    # Return the response
    output = response.json()["output"]
    _logger.debug(f"Completed after {end - start:.2f} seconds to process the image")
    _logger.debug(f"Response from MolMo: {output}")
    return parse_multiple_points(output)


def parse_multiple_points(text: str) -> list[PointDict]:
    """Parse the output of the MolMo model to extract multiple points."""
    pattern = r'<point x="([\d\.]+)" y="([\d\.]+)" alt="([^"]*)">(.+?)</point>'
    matches = re.findall(pattern, text)

    points = []
    for x, y, alt, body in matches:
        points.append({"x": float(x), "y": float(y), "alt": alt, "text": body})

    return points


def proportion2pixels(
    image: Image.Image, x_pct: float, y_pct: float
) -> tuple[int, int]:
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
    draw.ellipse(
        (x_abs - r_outside, y_abs - r_outside, x_abs + r_outside, y_abs + r_outside),
        outline="red",
        width=2,
    )
    return result
