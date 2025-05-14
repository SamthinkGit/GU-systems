import os
from functools import cache

from elevenlabs import save
from elevenlabs.client import ElevenLabs
from playsound import playsound

from ecm.shared import get_logger
from ecm.shared import get_root_path

NOVA_VOICE = "gD1IexrzCvsXPHUuT0s3"

_logger = get_logger("NovaVoice")


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
    playsound(path)
