import time
from multiprocessing import Manager
from multiprocessing import Process
from tests.compatibility.tools.screen_wrapper import sync_window_with_tracker
from tests.compatibility.tools.screen_wrapper import Tracker

import pytest


@pytest.fixture(scope="session")
def tracker():
    manager = Manager()
    tracker = Tracker(manager)
    p = Process(target=sync_window_with_tracker, args=(tracker,))
    p.start()
    time.sleep(2)

    yield tracker
    p.kill()
