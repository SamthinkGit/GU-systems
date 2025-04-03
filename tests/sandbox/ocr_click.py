import action_space.experimental.screenshot.actions  # noqa
import action_space.mouse.ocr_based.actions  # noqa
from action_space.tools.image import load_image
from cognition_layer.tools.ocr.engine import OCR
from ecm.tools.item_registry_v2 import ItemRegistry

if __name__ == "__main__":
    ItemRegistry().load_all()

    screenshot_tool = ItemRegistry().tools["screenshot"].content
    click_action = ItemRegistry().get("click", type="action").content

    screenshot = load_image(screenshot_tool())

    ocr = OCR()
    ocr.invoke(screenshot)

    click_action("Hello World!")
