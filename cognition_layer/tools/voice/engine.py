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
from ecm.tools.registry import Storage


def play(path):
    from cognition_layer.tools.voice.elevenlabs.tts import play as real_play
    from cognition_layer.tools.voice.voice2api.voice2api import play as api_play

    if Storage("TTS_CONFIG").get("API_ENABLED", False):
        api_play(path)
    else:
        real_play(path)


from cognition_layer.tools.voice.voice2api.voice2api import get_audios as get_audios
from cognition_layer.tools.voice.elevenlabs.tts import text2speech as text2speech
import base64
import tempfile
import os


def wav_to_base64(filepath):
    with open(filepath, "rb") as f:
        audio_bytes = f.read()
    encoded = base64.b64encode(audio_bytes).decode("utf-8")
    return encoded



def base64_to_wav_temp(encoded_str):
    audio_bytes = base64.b64decode(encoded_str.encode('utf-8'))

    with tempfile.NamedTemporaryFile(delete=False, suffix=".wav", mode='wb') as temp_file:
        temp_file.write(audio_bytes)
        temp_path = temp_file.name

    return temp_path
