import time

from PIL import Image
from rapidocr import RapidOCR

from cognition_layer.tools.ocr.template import BoundingBox
from cognition_layer.tools.ocr.template import OcrEngine
from ecm.shared import get_logger


class RapidOcr(OcrEngine):

    _logger = get_logger("RapidOCR")

    def __init__(self, *args, **kwargs):
        self.engine = RapidOCR()

    def invoke(self, image: Image.Image, *args, **kwargs) -> list[BoundingBox]:
        self._logger.debug("Running Engine")
        start = time.perf_counter()
        result = self.engine(image)
        end = time.perf_counter()
        self._logger.debug(f"OCR completed after {end - start:.2f}s")

        results: list[BoundingBox] = []
        for text, box in zip(result.txts, result.boxes):
            upper_left, upper_right, bottom_right, bottom_left = box
            center = (
                int((upper_left[0] + upper_right[0]) / 2),
                int((upper_left[1] + bottom_left[1]) / 2),
            )
            results.append(
                BoundingBox(
                    top_left=upper_left,
                    top_right=upper_right,
                    bottom_left=bottom_left,
                    bottom_right=bottom_right,
                    center=center,
                    content=text,
                    additional_info={},
                )
            )
        self._logger.debug(f"{len(results)} entities detected")
        return results
