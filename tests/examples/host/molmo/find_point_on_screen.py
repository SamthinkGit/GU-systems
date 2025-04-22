import os

from action_space.experimental.screenshot.actions import screenshot
from action_space.tools.image import load_image
from cognition_layer.tools.ocr.image_edition import partial_image
from cognition_layer.tools.ocr.image_edition import relative_coords_to_absolute
from cognition_layer.tools.vision.molmo.description2coordinates import description2coordinates
from cognition_layer.tools.vision.molmo.description2coordinates import draw_point_on_image
from cognition_layer.tools.vision.molmo.description2coordinates import proportion2pixels


def clear_terminal():
    os.system("cls" if os.name == "nt" else "clear")


if __name__ == "__main__":
    prompt = input("Select the object to find: ")
    location = input("Select a location (top, center, left, top right, ...): ")

    clear_terminal()
    print("Starting Search...")

    original = load_image(screenshot())
    cropped = partial_image(original, location)

    points = description2coordinates(cropped, prompt)
    print(f"Found {len(points)} points: {points}")

    coords = (points[0]["x"], points[0]["y"])
    pixel_coords = proportion2pixels(cropped, *coords)
    print(pixel_coords)
    absolute_coords = relative_coords_to_absolute(*pixel_coords, original, location)
    draw_point_on_image(original, *absolute_coords).show()
