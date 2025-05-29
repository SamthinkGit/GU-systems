from PIL import Image

from cognition_layer.tools.ocr.datalab.extract_bboxes import extract_bboxes
from cognition_layer.tools.ocr.datalab.extract_bboxes import read_image
from cognition_layer.tools.ocr.template import BoundingBox
from cognition_layer.tools.ocr.template import OcrEngine
from ecm.shared import get_logger


class DatalabEngine(OcrEngine):

    _logger = get_logger("DatalabOCR")

    def invoke(self, image: Image.Image, *args, **kwargs) -> list[BoundingBox]:
        response_json = read_image(image)
        self.bboxes = extract_bboxes(response_json, *args, **kwargs, animate=False)
        return self.bboxes

    def clean(self):
        return
