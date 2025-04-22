from action_space.experimental.screenshot.actions import screenshot
from action_space.tools.image import load_image
from cognition_layer.tools.ocr.image_edition import partial_image
from cognition_layer.tools.ocr.image_edition import relative_coords_to_absolute
from cognition_layer.tools.vision.molmo.description2coordinates import (
    draw_point_on_image,
)
from cognition_layer.tools.vision.molmo.description2coordinates import proportion2pixels

if __name__ == "__main__":

    side = "top left"
    original = load_image(screenshot())
    cropped = partial_image(original, side)

    proportion_coords = (26.5, 4.5)
    relative_coords = proportion2pixels(cropped, *proportion_coords)
    absolute_coords = relative_coords_to_absolute(*relative_coords, original, side)

    draw_point_on_image(cropped, *relative_coords).show()
    draw_point_on_image(original, *absolute_coords).show()
