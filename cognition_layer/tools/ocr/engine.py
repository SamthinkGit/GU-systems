"""
OCR Module
==============================
This module aims to be a mediator to select the module with the best Optical
Character Recognition (OCR). It provides an interface to work with the current OCR
engine, ensuring that they follow the specific template defined in the
cognition_layer.tools.ocr.template.

To import an OCR Engine use:
from cognition_layer.tools.ocr.engine import OCR

Do not try to import directly from the engine directory, so we can keep backward
compatibility
"""
from cognition_layer.tools.ocr.datalab.engine import DatalabEngine as TextOCR  # noqa
from cognition_layer.tools.ocr.mixedcv.engine import MixedCV as OCR  # noqa
