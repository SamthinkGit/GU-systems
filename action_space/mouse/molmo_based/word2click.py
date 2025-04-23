import time
from typing import Literal

from action_space.experimental.mouse.os_wrapper import click as osclick
from action_space.experimental.mouse.os_wrapper import move as osmove
from action_space.experimental.screenshot.actions import screenshot
from action_space.tools.image import load_image
from action_space.tools.terminal import clear_terminal
from cognition_layer.tools.ocr.image_edition import partial_image
from cognition_layer.tools.ocr.image_edition import relative_coords_to_absolute
from cognition_layer.tools.ocr.image_edition import resize_image
from cognition_layer.tools.vision.molmo.description2coordinates import (
    description2coordinates,
)
from cognition_layer.tools.vision.molmo.description2coordinates import (
    draw_point_on_image,
)
from cognition_layer.tools.vision.molmo.description2coordinates import proportion2pixels
from ecm.shared import get_root_path

MOLMO_QUALITY = 1


def _click_on_(prompt, location):
    try:
        coords = _find_point_on_screen(prompt, location)
    except ValueError:
        return f"MolMo could not find any point for the prompt '{prompt}' in the location '{location}'."
    osmove(coords[0], coords[1])
    osclick()
    return "Successfully clicked on the element."


def _double_click_on_(prompt, location):
    try:
        coords = _find_point_on_screen(prompt, location)
    except ValueError:
        return f"MolMo could not find any point for the prompt '{prompt}' in the location '{location}'."

    osmove(coords[0], coords[1])
    osclick()
    osclick()
    return "Successfully double clicked on the element."


def _find_point_on_screen(
    prompt: str,
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
) -> tuple[float, float]:
    """
    Find a point on the screen based on a prompt and location.

    Args:
        prompt (str): The prompt to search for.
        location (str): The location to search in.

    Returns:
        tuple[float, float]: The coordinates of the found point.
    """

    clear_terminal(reason="Avoid inferences with MolMo vision search")
    time.sleep(0.1)
    original = load_image(screenshot())
    cropped = partial_image(original, location)
    if MOLMO_QUALITY != 1:
        cropped = resize_image(cropped, MOLMO_QUALITY)

    points = description2coordinates(cropped, prompt)
    if len(points) == 0:
        raise ValueError(
            f"MolMo could not find any point for the prompt '{prompt}' in the location '{location}'."
        )

    proportion_coords = (points[0]["x"], points[0]["y"])
    relative_coords = proportion2pixels(cropped, *proportion_coords)
    absolute_coords = relative_coords_to_absolute(*relative_coords, original, location)
    draw_point_on_image(original, *absolute_coords).save(
        get_root_path() / "logs" / "latest_molmo_click.png"
    )

    return absolute_coords
