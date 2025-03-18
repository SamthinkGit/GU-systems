import os
from action_space.experimental.screenshot.screenshot import take_screenshot
from action_space.tools.image import return_image
from ecm.tools.registry import ItemRegistry

from PIL import Image


@ItemRegistry.register_util
def screenshot() -> bytes:
    image_path = "temp.png"
    take_screenshot(image_path)

    with Image.open(image_path) as image:
        image = image.copy()
    os.remove(image_path)

    return return_image(image)
