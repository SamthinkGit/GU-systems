from functools import cache

from RealtimeSTT import AudioToTextRecorder


class SimpleRecorder:
    def __init__(self):
        self.recorder = self._get_recorder()

    @cache
    def _get_recorder(self):
        return AudioToTextRecorder(
            model="tiny",
            language="es",
            device="cpu",
            use_microphone=True,
        )

    @property
    def text(self):
        return self.recorder.text()

    def start(self):
        self.recorder.start()

    def stop(self):
        self.recorder.stop()
