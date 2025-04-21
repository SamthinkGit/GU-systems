import action_space.experimental.screenshot.actions  # noqa
import action_space.mouse.labelled_ocr.actions  # noqa
from action_space.tools.image import load_image
from cognition_layer.tools.ocr.engine import OCR
from cognition_layer.tools.ocr.labeler import Labeler
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

if __name__ == "__main__":
    logger = get_logger("main")

    ItemRegistry().load_all()
    screenshot = load_image(ItemRegistry().tools["screenshot"].content())

    engine = OCR()
    boxes = engine.invoke(screenshot)
    labeler = Labeler(screenshot, boxes)
    labeler.freeze()

    for label, bbox in labeler.labelled_boxes.items():
        if bbox.additional_info.get("type", "null") == "text":
            logger.info(f"{label} ->\t{bbox.content}")

    labeler.draw().show()

    board = labeler.board(
        filter=lambda bbox: bbox.additional_info.get("type", "null") == "icon"
    )
    board.show("Board")

    selected_label = input("Select a label to click: ")
    type = ""

    while type not in ["text", "icon"]:
        type = input("Select a type (text/icon): ")

    click = ItemRegistry().get("click").content
    click(selected_label, type)
