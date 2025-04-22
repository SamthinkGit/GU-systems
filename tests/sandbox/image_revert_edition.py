from action_space.experimental.screenshot.actions import screenshot
from action_space.tools.image import load_image
from cognition_layer.tools.ocr.image_edition import partial_image
from cognition_layer.tools.ocr.image_edition import relative_coords_to_absolute
from cognition_layer.tools.vision.molmo.description2coordinates import (
    draw_point_on_image,
)

if __name__ == "__main__":

    side = "right"
    original = load_image(screenshot())
    cropped = partial_image(original, side)

    relative_coords = (100, 100)
    absolute_coords = relative_coords_to_absolute(*relative_coords, original, side)

    draw_point_on_image(cropped, *relative_coords).show()
    draw_point_on_image(original, *absolute_coords).show()
