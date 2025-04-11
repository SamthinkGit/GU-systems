import action_space.experimental.screenshot.actions  # noqa
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

    icons = [box for box in boxes if box.additional_info.get("type", "null") == "icon"]

    labeler = Labeler(screenshot, icons)
    board = labeler.board()
    board.show("Board")

#
#    labelled_img = labeler.draw(
#        color=(0, 0, 0),
#        box_thickness=1,
#        font_color="random",
#        font_scale=0.7,
#        font_thickness=2,
#        labellize=True,
#    )
#    labelled_img = labeler.draw(
#        color=(255, 255, 255),
#        box_thickness=1,
#        font_color="random",
#        font_scale=0.8,
#        font_thickness=2,
#        labellize=True,
#    )
#    labelled_img.show("Labelled Image")
