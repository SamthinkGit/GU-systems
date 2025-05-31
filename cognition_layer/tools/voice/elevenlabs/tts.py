import os
import threading
from functools import cache

import pyaudio
from elevenlabs import save
from elevenlabs.client import ElevenLabs
from pydub import AudioSegment

from ecm.shared import get_root_path

NOVA_VOICE = "gD1IexrzCvsXPHUuT0s3"


_last_audio_thread = None
_last_audio_lock = threading.Lock()


@cache
def get_elevenlabs_client():
    client = ElevenLabs(
        api_key=os.getenv("ELEVENLABS_API_KEY"),
    )
    return client


def text2speech(
    content: str, voice_id: str = NOVA_VOICE, model_id: str = "eleven_flash_v2_5"
):
    client = get_elevenlabs_client()
    audio = client.text_to_speech.convert(
        text=content,
        voice_id=voice_id,
        model_id=model_id,
        output_format="mp3_44100_128",
    )
    logging_path = (
        (
            get_root_path()
            / "cognition_layer"
            / "tools"
            / "voice"
            / "elevenlabs"
            / "logs"
            / "latest_nova_voice.mp3"
        )
        .resolve()
        .absolute()
    ).as_posix()
    if os.path.exists(logging_path):
        os.remove(logging_path)

    save(audio, logging_path)
    return logging_path


def play(path: str):
    global _last_audio_thread

    def _play_audio():
        audio = AudioSegment.from_file(path)
        p = pyaudio.PyAudio()

        stream = p.open(
            format=p.get_format_from_width(audio.sample_width),
            channels=audio.channels,
            rate=audio.frame_rate,
            output=True,
        )

        stream.write(audio.raw_data)

        stream.stop_stream()
        stream.close()
        p.terminate()

    new_thread = threading.Thread(target=_play_audio)

    with _last_audio_lock:
        if _last_audio_thread and _last_audio_thread.is_alive():
            _last_audio_thread.join()

        _last_audio_thread = new_thread

    new_thread.start()
