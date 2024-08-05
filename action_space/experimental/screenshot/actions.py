import os

from PIL import Image

from action_space.experimental.screenshot.screenshot import take_screenshot
from action_space.tools.image import return_image
from ecm.tools.registry import ItemRegistry


@ItemRegistry.register_util
def screenshot() -> Image.Image:
    image_path = "temp.png"
    take_screenshot(image_path)

    image = Image.open(image_path)
    os.remove(image_path)

    return return_image(image)
