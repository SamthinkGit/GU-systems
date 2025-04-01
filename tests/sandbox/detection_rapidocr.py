import action_space.experimental.mouse.actions  # noqa
import action_space.experimental.screenshot.actions  # noqa
from action_space.tools.image import load_image
from cognition_layer.tools.ocr.engine import OCR
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

if __name__ == "__main__":
    logger = get_logger("main")

    ItemRegistry().load_all()

    input(
        "Write Hello world somewhere in your screen.\n"
        "Ensure the font is clearly visible to the OCR."
        "\n[Press Enter to continue]"
    )

    screenshot = load_image(ItemRegistry().tools["screenshot"].content())

    engine = OCR()
    boxes = engine.invoke(screenshot)

    for box in boxes:
        if "hello world" in box.content.lower():
            logger.info("Hello world statement detected.")
            result = box
            break
    else:
        raise SystemError("Hello world not detected with the OCR. Exit")

    ItemRegistry().tools["move_mouse_to"].content(*result.center)
    print("Mouse moved to statement")
