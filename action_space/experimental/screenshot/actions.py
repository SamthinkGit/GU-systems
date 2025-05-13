import os

from PIL import Image

from action_space.experimental.screenshot.screenshot import take_screenshot
from action_space.tools.image import return_image
from ecm.tools.item_registry_v2 import ItemRegistry

PKG_NAME = "screenshot"


@ItemRegistry.register(type="tool", package=PKG_NAME)
def screenshot() -> str:
    image_path = "temp.png"
    take_screenshot(image_path)

    with Image.open(image_path) as image:
        image = image.copy()
    os.remove(image_path)

    return return_image(image)
