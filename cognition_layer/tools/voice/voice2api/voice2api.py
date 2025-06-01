import queue

from ecm.tools.registry import Storage

Storage("TTS_CONFIG")["queue"] = queue.Queue()


def play(path):
    Storage("TTS_CONFIG")["queue"].put(path)


def get_audios() -> list[str]:

    q: queue.Queue = Storage("TTS_CONFIG")["queue"]
    items = []
    while not q.empty():
        try:
            item = q.get_nowait()
            items.append(item)
        except queue.Empty:
            break
    return items
