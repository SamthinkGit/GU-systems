# flake8: noqa
"""
Voice Module
==============================
This module aims to be a mediator to select the module with the best Voice Generator.
It provides an interface to work with the current voice engine.

To import a Voice Engine use:
from cognition_layer.tools.ocr.engine import OCR

Do not try to import directly from the engine directory, so we can keep backward
compatibility
"""
from cognition_layer.tools.voice.elevenlabs.tts import play as play
from cognition_layer.tools.voice.elevenlabs.tts import text2speech as text2speech
