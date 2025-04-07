from PIL import Image

from cognition_layer.tools.ocr.template import BoundingBox
from cognition_layer.tools.ocr.template import OcrEngine
from ecm.shared import get_logger


class RapidOCR(OcrEngine):

    _logger = get_logger("RapidOCR")

    def __init__(self, *args, **kwargs): ...

    def invoke(self, image: Image.Image, *args, **kwargs) -> list[BoundingBox]: ...
