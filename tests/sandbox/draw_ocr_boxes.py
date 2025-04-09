import cv2
import numpy as np

import action_space.experimental.screenshot.actions  # noqa
from action_space.tools.image import load_image
from cognition_layer.tools.ocr.engine import OCR
from cognition_layer.tools.ocr.template import draw_bounding_boxes
from ecm.shared import get_logger
from ecm.tools.item_registry_v2 import ItemRegistry

if __name__ == "__main__":
    logger = get_logger("main")

    ItemRegistry().load_all()
    screenshot = load_image(ItemRegistry().tools["screenshot"].content())

    engine = OCR()
    boxes = engine.invoke(screenshot)

    img_np = np.array(screenshot)
    opencv_image = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
    debug_image = draw_bounding_boxes(opencv_image, boxes)

    print(boxes)
    cv2.imshow("debug", debug_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
