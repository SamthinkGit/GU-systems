import io
import time

import requests
from PIL import Image
from PyQt5.QtWidgets import QApplication

from cognition_layer.tools.ocr.datalab.animation import BoxOverlay
from cognition_layer.tools.ocr.template import BoundingBox
from ecm.shared import get_logger

ANIMATION_DURATION = 2000
MIN_DURATION_PER_BOX = 500
API_KEY = "XHfaC3JG5oGJnyL2I1tZ0cuPK18kazMPpWX1iWGi078"
WAIT_TIME = 0.1
_logger = get_logger("DatalabOCR")


def read_image(image: Image.Image):

    image_bytes = io.BytesIO()
    image.save(image_bytes, format="PNG")
    image_bytes.seek(0)

    url = "https://www.datalab.to/api/v1/ocr"
    files = {
        "file": ("test.png", image_bytes, "image/png"),
        "langs": (None, "English, Spanish"),
        "max_pages": (None, "1"),
    }

    headers = {"X-Api-Key": API_KEY}
    response = requests.post(url, files=files, headers=headers)
    if response.ok:
        data = response.json()
        _logger.debug("OCR Requested")
    else:
        _logger.error(f"{response.status_code}: {response.text}")

    check_url = data["request_check_url"]

    start = time.perf_counter()
    while True:
        result_response = requests.get(check_url, headers=headers)

        if result_response.ok:
            result_data = result_response.json()
            if result_data.get("status") == "processing":
                time.sleep(WAIT_TIME)
            else:
                break
        else:
            _logger.error(f"{result_response.status_code}: {result_response.text}")
            break

    end = time.perf_counter()
    _logger.debug(f"Total time: {end - start:.2f}s")
    return result_response.json()


def extract_bboxes(response_json: dict, animate: bool = True) -> list[BoundingBox]:
    bounding_boxes = []
    for line in response_json["pages"][0]["text_lines"]:
        x0, y0, x1, y1 = map(int, line["bbox"])
        center = ((x0 + x1) // 2, (y0 + y1) // 2)
        bbox = BoundingBox(
            top_left=(x0, y0),
            top_right=(x1, y0),
            bottom_left=(x0, y1),
            bottom_right=(x1, y1),
            center=center,
            content=line["text"],
            additional_info={"bbox": line["bbox"]},
        )
        bounding_boxes.append(bbox)

    if animate:
        app = QApplication([])
        overlay = BoxOverlay(
            bounding_boxes,
            duration_total=ANIMATION_DURATION,
            min_duration_per_box=MIN_DURATION_PER_BOX,
        )

        overlay.show()
        app.exec_()

    return bounding_boxes
