"""
Speech to Text Engine
==============================
This module aims to be a mediator to select the module with the best Speech
To Text (STT). It provides an interface to work with the current STT
engine, ensuring that they follow the specific template defined in the
cognition_layer.tools.stt.engine.

To import an STT Engine use:
from cognition_layer.tools.stt.engine import STT
Do not try to import directly from the engine directory, so we can keep backward
compatibility

"""
from ecm.tools.stt.elevenlabs_tts import SimpleRecorder as STT  # noqa
from ecm.tools.stt.elevenlabs_tts import speech_to_text as speech2text  # noqa
