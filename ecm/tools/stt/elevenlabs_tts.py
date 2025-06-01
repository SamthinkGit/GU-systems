import os
import wave
from functools import cache
from io import BytesIO

import numpy as np
import sounddevice as sd
from elevenlabs.client import ElevenLabs


class SimpleRecorder:
    def __init__(self):
        self._client = self._get_client()
        self._frames = []
        self._stream = None
        self._text = None
        self.recording = False

    @cache
    def _get_client(self):
        api_key = os.getenv("ELEVENLABS_API_KEY")
        if not api_key:
            raise RuntimeError("Debes definir ELEVENLABS_API_KEY en el entorno")
        return ElevenLabs(api_key=api_key)

    @property
    def text(self):
        return self._text

    def start(self):
        if self.recording:
            return
        self._frames = []
        self.recording = True
        self._stream = sd.InputStream(
            samplerate=16000, channels=1, dtype="int16", callback=self._callback
        )
        self._stream.start()

    def _callback(self, indata, frames, time, status):
        if status:
            print(f"[WARNING] {status}")
        self._frames.append(indata.copy())

    def stop(self):
        if not self.recording:
            return None
        self.recording = False
        self._stream.stop()
        self._stream.close()

        buf = BytesIO()
        with wave.open(buf, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)  # 16 bits = 2 bytes
            wf.setframerate(16000)
            audio = np.concatenate(self._frames, axis=0)
            wf.writeframes(audio.tobytes())
        buf.seek(0)

        resp = self._client.speech_to_text.convert(
            file=buf,
            model_id="scribe_v1",
            tag_audio_events=False,
            diarize=False,
        )
        self._text = getattr(resp, "text", None) or resp.get("text", "")
        return self._text

    def clean(self):
        pass


def speech_to_text(audio_file: str):
    elevenlabs = ElevenLabs(
        api_key=os.getenv("ELEVENLABS_API_KEY"),
    )

    with open(audio_file, "rb") as f:
        mp3_bytes = f.read()

    audio_data = BytesIO(mp3_bytes)
    transcription = elevenlabs.speech_to_text.convert(
        file=audio_data,
        model_id="scribe_v1",
        tag_audio_events=False,
        diarize=False,
    )
    return transcription.text
